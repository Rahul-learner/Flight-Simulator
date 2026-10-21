#pragma once

#include <math.h>
#include <string.h>
#include <stdint.h>

/**
 * @file AdvancedAttitudeEKF.hpp
 * @brief High-Performance Multiplicative Extended Kalman Filter (MEKF / Error-State EKF)
 *        for 6-DOF IMU (Accelerometer + Gyroscope) Attitude Estimation.
 * 
 * Features:
 * - 6-DOF Error-State representation (eliminates quaternion covariance singularity)
 * - Closed-form Rodrigues/exponential quaternion kinematics (exact at high angular rates)
 * - Stabilized Joseph-form covariance update (guarantees positive semi-definiteness)
 * - Adaptive measurement noise covariance based on dynamic linear acceleration
 * - Dynamic acceleration gating (prevents centrifugal/linear forces from tilting attitude)
 * - Online gyroscope bias estimation and tracking
 * - Instantaneous gravity alignment calibration routine
 * - Highly optimized matrix operations tailored for ARM Cortex-M0+ (Raspberry Pi Pico)
 */

class AdvancedAttitudeEKF {
public:
    // Nominal state
    // Quaternion q = [q0, q1, q2, q3] = [w, x, y, z] (Earth to Body frame)
    float q[4];
    // Gyroscope bias estimates in rad/s [bx, by, bz]
    float bg[3];

    // Covariance matrix P (6x6): [delta_theta(3), delta_bias(3)]
    float P[6][6];

    // Tuning parameters
    float q_gyro_noise;   // Process noise for angular rate (rad/s)^2
    float q_bias_noise;   // Process noise for gyro bias drift (rad/s^2)^2
    float r_accel_noise;  // Measurement noise covariance base for accelerometer (g^2)
    float r_mag_noise;    // Measurement noise covariance for magnetometer yaw (rad^2)

    // Dynamic acceleration rejection settings
    bool dynamic_rejection_enabled;
    float dynamic_accel_threshold; // In g's deviation from 1.0g (default 0.40g)

    // Diagnostic information
    float last_dt;
    float last_accel_norm;
    bool last_accel_gated;

    AdvancedAttitudeEKF() {
        reset();
    }

    /**
     * @brief Reset filter state to identity / level orientation and default covariances
     */
    void reset(float initial_roll_rad = 0.0f, float initial_pitch_rad = 0.0f, float initial_yaw_rad = 0.0f) {
        setEuler(initial_roll_rad, initial_pitch_rad, initial_yaw_rad);

        bg[0] = 0.0f;
        bg[1] = 0.0f;
        bg[2] = 0.0f;

        // Default process & measurement noise
        q_gyro_noise = 1.0e-5f;  // Tuning: gyro angular random walk
        q_bias_noise = 1.0e-8f;  // Tuning: slow bias drift (random walk)
        r_accel_noise = 0.035f;  // Tuning: accelerometer measurement variance
        r_mag_noise = 0.040f;    // Tuning: magnetometer yaw variance

        dynamic_rejection_enabled = true;
        dynamic_accel_threshold = 0.40f;

        last_dt = 0.0f;
        last_accel_norm = 1.0f;
        last_accel_gated = false;

        // Initialize covariance P
        memset(P, 0, sizeof(P));
        for (int i = 0; i < 3; i++) {
            P[i][i] = 0.1f;          // Initial attitude uncertainty
            P[i + 3][i + 3] = 1.0e-4f; // Initial bias uncertainty (tightly bounded)
        }
    }

    /**
     * @brief Set orientation from Euler angles (in radians, ZYX aerospace sequence)
     */
    void setEuler(float roll_rad, float pitch_rad, float yaw_rad) {
        float cr = cosf(roll_rad * 0.5f);
        float sr = sinf(roll_rad * 0.5f);
        float cp = cosf(pitch_rad * 0.5f);
        float sp = sinf(pitch_rad * 0.5f);
        float cy = cosf(yaw_rad * 0.5f);
        float sy = sinf(yaw_rad * 0.5f);

        q[0] = cr * cp * cy + sr * sp * sy; // w
        q[1] = sr * cp * cy - cr * sp * sy; // x
        q[2] = cr * sp * cy + sr * cp * sy; // y
        q[3] = cr * cp * sy - sr * sp * cy; // z

        normalizeQuaternion();
    }

    /**
     * @brief Calibrate and initialize filter from stationary accelerometer, gyro, and magnetometer samples
     * @param ax_avg Average stationary accel X in g's
     * @param ay_avg Average stationary accel Y in g's
     * @param az_avg Average stationary accel Z in g's
     * @param gx_bias Average stationary gyro X in rad/s
     * @param gy_bias Average stationary gyro Y in rad/s
     * @param gz_bias Average stationary gyro Z in rad/s
     * @param mx Magnetometer X reading (optional, 0 if unavailable)
     * @param my Magnetometer Y reading (optional, 0 if unavailable)
     * @param mz Magnetometer Z reading (optional, 0 if unavailable)
     */
    void initFromSensors(float ax_avg, float ay_avg, float az_avg,
                         float gx_bias, float gy_bias, float gz_bias,
                         float mx = 0.0f, float my = 0.0f, float mz = 0.0f) {
        bg[0] = gx_bias;
        bg[1] = gy_bias;
        bg[2] = gz_bias;

        float norm = sqrtf(ax_avg * ax_avg + ay_avg * ay_avg + az_avg * az_avg);
        float roll_0 = 0.0f, pitch_0 = 0.0f, yaw_0 = 0.0f;
        if (norm > 0.001f) {
            ax_avg /= norm;
            ay_avg /= norm;
            az_avg /= norm;

            // Compute initial roll and pitch from gravity vector
            roll_0 = atan2f(ay_avg, az_avg);
            pitch_0 = atan2f(-ax_avg, sqrtf(ay_avg * ay_avg + az_avg * az_avg));
        }

        // If magnetometer reading is provided, compute initial magnetic heading
        float m_norm = sqrtf(mx * mx + my * my + mz * mz);
        if (m_norm > 0.001f) {
            float cos_r = cosf(roll_0);
            float sin_r = sinf(roll_0);
            float cos_p = cosf(pitch_0);
            float sin_p = sinf(pitch_0);

            float Xh = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
            float Yh = my * cos_r - mz * sin_r;
            yaw_0 = atan2f(-Yh, Xh);
        }

        setEuler(roll_0, pitch_0, yaw_0);

        // Tightly bound initial covariance since we know initial state from calibration
        memset(P, 0, sizeof(P));
        for (int i = 0; i < 3; i++) {
            P[i][i] = 0.01f;
            P[i + 3][i + 3] = 1.0e-4f;
        }
    }

    /**
     * @brief Prediction Step (Propagation using Gyroscope readings)
     * @param dt Time delta since last update in seconds
     * @param gx Measured angular rate X in rad/s
     * @param gy Measured angular rate Y in rad/s
     * @param gz Measured angular rate Z in rad/s
     */
    void predict(float dt, float gx, float gy, float gz) {
        if (dt <= 0.0f || dt > 0.5f) return;
        last_dt = dt;

        // Subtract current estimated biases
        float wx = gx - bg[0];
        float wy = gy - bg[1];
        float wz = gz - bg[2];

        // Closed-form quaternion integration via Rodrigues axis-angle representation
        float w_mag = sqrtf(wx * wx + wy * wy + wz * wz);
        float dq[4];

        if (w_mag > 1.0e-6f) {
            float half_angle = 0.5f * w_mag * dt;
            float sin_ha = sinf(half_angle);
            float scale = sin_ha / w_mag;
            dq[0] = cosf(half_angle);
            dq[1] = wx * scale;
            dq[2] = wy * scale;
            dq[3] = wz * scale;
        } else {
            // First-order Taylor approximation for small angles
            float half_dt = 0.5f * dt;
            dq[0] = 1.0f;
            dq[1] = wx * half_dt;
            dq[2] = wy * half_dt;
            dq[3] = wz * half_dt;
        }

        // Quaternion multiplication: q_new = q * dq
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        q[0] = q0 * dq[0] - q1 * dq[1] - q2 * dq[2] - q3 * dq[3];
        q[1] = q0 * dq[1] + q1 * dq[0] + q2 * dq[3] - q3 * dq[2];
        q[2] = q0 * dq[2] - q1 * dq[3] + q2 * dq[0] + q3 * dq[1];
        q[3] = q0 * dq[3] + q1 * dq[2] - q2 * dq[1] + q3 * dq[0];

        normalizeQuaternion();

        // Error-state transition matrix F (6x6)
        // F = [ I - [w]x * dt,   -I * dt ]
        //     [ 0,                I      ]
        float F[6][6];
        memset(F, 0, sizeof(F));
        for (int i = 0; i < 6; i++) F[i][i] = 1.0f;

        // - [w]x * dt:
        // [ 0,    wz*dt, -wy*dt ]
        // [ -wz*dt, 0,    wx*dt ]
        // [ wy*dt, -wx*dt, 0    ]
        F[0][1] =  wz * dt;
        F[0][2] = -wy * dt;
        F[1][0] = -wz * dt;
        F[1][2] =  wx * dt;
        F[2][0] =  wy * dt;
        F[2][1] = -wx * dt;

        // -I * dt for bias coupling
        F[0][3] = -dt;
        F[1][4] = -dt;
        F[2][5] = -dt;

        // Covariance Propagation: P = F * P * F^T + Q
        // Compute F * P into temp
        float FP[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += F[i][k] * P[k][j];
                }
                FP[i][j] = sum;
            }
        }

        // Compute (FP) * F^T + Q into P
        float q_rot = q_gyro_noise * dt;
        float q_bias = q_bias_noise * dt;

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += FP[i][k] * F[j][k]; // F^T[k][j] = F[j][k]
                }
                if (i == j) {
                    sum += (i < 3) ? q_rot : q_bias;
                }
                P[i][j] = sum;
            }
        }

        symmetrizeP();
    }

    /**
     * @brief Measurement Update Step (Correction using Accelerometer readings)
     * @param ax Raw or calibrated accelerometer X in g's
     * @param ay Raw or calibrated accelerometer Y in g's
     * @param az Raw or calibrated accelerometer Z in g's
     */
    void update(float ax, float ay, float az) {
        float a_norm = sqrtf(ax * ax + ay * ay + az * az);
        last_accel_norm = a_norm;

        if (a_norm < 0.1f) return; // Zero or invalid reading

        // Normalize accelerometer vector
        float inv_norm = 1.0f / a_norm;
        float ax_n = ax * inv_norm;
        float ay_n = ay * inv_norm;
        float az_n = az * inv_norm;

        // Dynamic Acceleration Detection & Gating
        float delta_a = fabsf(a_norm - 1.0f);
        if (dynamic_rejection_enabled && delta_a > dynamic_accel_threshold) {
            last_accel_gated = true;
            return; // Inhibit accel update during violent dynamic G-forces or vibrations
        }
        last_accel_gated = false;

        // Adaptive measurement noise: increase R when dynamic acceleration is detected
        float R_adaptive = r_accel_noise;
        if (dynamic_rejection_enabled && delta_a > 0.05f) {
            R_adaptive *= (1.0f + 25.0f * delta_a * delta_a);
        }

        // Predicted gravity in body frame (hx, hy, hz)
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        float hx = 2.0f * (q1 * q3 - q0 * q2);
        float hy = 2.0f * (q0 * q1 + q2 * q3);
        float hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Innovation (residual) y = a_meas - h
        float y[3] = { ax_n - hx, ay_n - hy, az_n - hz };

        // Measurement Jacobian H (3x6)
        // H = [ [ 0, -hz,  hy, 0, 0, 0 ],
        //       [ hz,  0, -hx, 0, 0, 0 ],
        //       [-hy, hx,   0, 0, 0, 0 ] ]
        float H[3][6];
        memset(H, 0, sizeof(H));
        H[0][1] = -hz; H[0][2] =  hy;
        H[1][0] =  hz; H[1][2] = -hx;
        H[2][0] = -hy; H[2][1] =  hx;

        // Compute HP = H * P (3x6)
        float HP[3][6];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 6; j++) {
                HP[i][j] = H[i][0] * P[0][j] + H[i][1] * P[1][j] + H[i][2] * P[2][j];
            }
        }

        // Compute S = HP * H^T + R (3x3)
        float S[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = HP[i][0] * H[j][0] + HP[i][1] * H[j][1] + HP[i][2] * H[j][2];
                if (i == j) sum += R_adaptive;
                S[i][j] = sum;
            }
        }

        // Invert 3x3 matrix S
        float S_inv[3][3];
        if (!invert3x3(S, S_inv)) {
            // S matrix singular or ill-conditioned; reset covariance bounds
            for (int i = 0; i < 6; i++) {
                if (P[i][i] < 0.0001f) P[i][i] = 0.01f;
            }
            return;
        }

        // Compute P * H^T (6x3)
        float PHT[6][3];
        for (int i = 0; i < 6; i++) {
            PHT[i][0] = P[i][1] * (-hz) + P[i][2] * hy;
            PHT[i][1] = P[i][0] * hz    + P[i][2] * (-hx);
            PHT[i][2] = P[i][0] * (-hy) + P[i][1] * hx;
        }

        // Kalman Gain K = PHT * S_inv (6x3)
        float K[6][3];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = PHT[i][0] * S_inv[0][j] +
                          PHT[i][1] * S_inv[1][j] +
                          PHT[i][2] * S_inv[2][j];
            }
        }

        // Error State correction: dx = K * y (6x1)
        float dx[6];
        for (int i = 0; i < 6; i++) {
            dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
        }

        // Stabilized Joseph-Form Covariance Update:
        // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        float I_KH[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float kh = K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j];
                I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
            }
        }

        // Temp = (I - KH) * P
        float temp[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += I_KH[i][k] * P[k][j];
                }
                temp[i][j] = sum;
            }
        }

        // P_updated = temp * (I - KH)^T + K * R * K^T
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += temp[i][k] * I_KH[j][k];
                }
                // Add K * R * K^T term
                float krk = (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]) * R_adaptive;
                P[i][j] = sum + krk;
            }
        }

        symmetrizeP();

        // State Injection (Reset):
        // Correct nominal quaternion: q = q * [1, 0.5 * delta_theta]
        float dtheta_x = dx[0];
        float dtheta_y = dx[1];
        float dtheta_z = dx[2];

        float dq_corr[4] = { 1.0f, 0.5f * dtheta_x, 0.5f * dtheta_y, 0.5f * dtheta_z };
        float corr_norm = 1.0f / sqrtf(dq_corr[0]*dq_corr[0] + dq_corr[1]*dq_corr[1] +
                                       dq_corr[2]*dq_corr[2] + dq_corr[3]*dq_corr[3]);
        dq_corr[0] *= corr_norm;
        dq_corr[1] *= corr_norm;
        dq_corr[2] *= corr_norm;
        dq_corr[3] *= corr_norm;

        q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
        q[0] = q0 * dq_corr[0] - q1 * dq_corr[1] - q2 * dq_corr[2] - q3 * dq_corr[3];
        q[1] = q0 * dq_corr[1] + q1 * dq_corr[0] + q2 * dq_corr[3] - q3 * dq_corr[2];
        q[2] = q0 * dq_corr[2] - q1 * dq_corr[3] + q2 * dq_corr[0] + q3 * dq_corr[1];
        q[3] = q0 * dq_corr[3] + q1 * dq_corr[2] - q2 * dq_corr[1] + q3 * dq_corr[0];

        normalizeQuaternion();

        // Correct gyro biases with smooth clamping to prevent transient maneuver corruption
        const float MAX_BIAS_STEP = 0.001f; // rad/s max step (~0.05 deg/s)
        for (int i = 0; i < 3; i++) {
            float db = dx[3 + i];
            if (db > MAX_BIAS_STEP) db = MAX_BIAS_STEP;
            else if (db < -MAX_BIAS_STEP) db = -MAX_BIAS_STEP;
            bg[i] += db;
        }
    }

    /**
     * @brief 9-DOF Magnetometer Measurement Update Step (Heading & Yaw Drift Correction)
     * @param mx Calibrated magnetic field X
     * @param my Calibrated magnetic field Y
     * @param mz Calibrated magnetic field Z
     */
    void updateMag(float mx, float my, float mz) {
        float m_norm = sqrtf(mx * mx + my * my + mz * mz);
        if (m_norm < 0.001f) return;

        // Normalize magnetometer reading
        float inv_norm = 1.0f / m_norm;
        mx *= inv_norm;
        my *= inv_norm;
        mz *= inv_norm;

        // Extract current roll and pitch estimates from quaternion
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        float roll = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        float pitch;
        if (fabsf(sinp) >= 1.0f) pitch = copysignf((float)M_PI * 0.5f, sinp);
        else pitch = asinf(sinp);

        // Tilt-compensated horizontal magnetic projection
        float cos_r = cosf(roll);
        float sin_r = sinf(roll);
        float cos_p = cosf(pitch);
        float sin_p = sinf(pitch);

        float Xh = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
        float Yh = my * cos_r - mz * sin_r;

        float mag_heading = atan2f(-Yh, Xh);

        // Current EKF yaw
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        float ekf_yaw = atan2f(siny_cosp, cosy_cosp);

        // Innovation on Yaw: wrap to [-PI, +PI]
        float y_yaw = mag_heading - ekf_yaw;
        while (y_yaw > (float)M_PI) y_yaw -= 2.0f * (float)M_PI;
        while (y_yaw < -(float)M_PI) y_yaw += 2.0f * (float)M_PI;

        // Scalar Kalman update on yaw error (H = [0, 0, 1, 0, 0, 0])
        float S = P[2][2] + r_mag_noise;
        if (S < 1.0e-6f) return;
        float S_inv = 1.0f / S;

        // Kalman gain K = P[:, 2] * S_inv
        float K[6];
        for (int i = 0; i < 6; i++) {
            K[i] = P[i][2] * S_inv;
        }

        // Error state update
        float dx[6];
        for (int i = 0; i < 6; i++) {
            dx[i] = K[i] * y_yaw;
        }

        // Stabilized Joseph-Form covariance update for scalar measurement:
        // I_KH = I - K * H (where H is only non-zero at index 2)
        float I_KH[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                I_KH[i][j] = (i == j ? 1.0f : 0.0f) - (j == 2 ? K[i] : 0.0f);
            }
        }

        // Temp = I_KH * P
        float temp[6][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += I_KH[i][k] * P[k][j];
                }
                temp[i][j] = sum;
            }
        }

        // P = temp * I_KH^T + K * R_mag * K^T
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += temp[i][k] * I_KH[j][k];
                }
                P[i][j] = sum + K[i] * K[j] * r_mag_noise;
            }
        }

        symmetrizeP();

        // Inject yaw error into quaternion: delta_q = [1, 0, 0, 0.5 * dx[2]]
        float half_dyaw = 0.5f * dx[2];
        float dq_yaw[4] = { 1.0f, 0.0f, 0.0f, half_dyaw };
        float q_norm = 1.0f / sqrtf(1.0f + half_dyaw * half_dyaw);
        dq_yaw[0] *= q_norm;
        dq_yaw[3] *= q_norm;

        q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
        q[0] = q0 * dq_yaw[0] - q3 * dq_yaw[3];
        q[1] = q1 * dq_yaw[0] + q2 * dq_yaw[3];
        q[2] = q2 * dq_yaw[0] - q1 * dq_yaw[3];
        q[3] = q3 * dq_yaw[0] + q0 * dq_yaw[3];

        normalizeQuaternion();

        // Update gyro Z bias
        float dbz = dx[5];
        const float MAX_BIAS_STEP = 0.001f;
        if (dbz > MAX_BIAS_STEP) dbz = MAX_BIAS_STEP;
        else if (dbz < -MAX_BIAS_STEP) dbz = -MAX_BIAS_STEP;
        bg[2] += dbz;
    }

    /**
     * @brief Convert estimated attitude to Euler angles (in degrees, aerospace ZYX order)
     * @param roll_deg Output Roll angle in degrees (-180 to +180)
     * @param pitch_deg Output Pitch angle in degrees (-90 to +90)
     * @param yaw_deg Output Yaw angle in degrees (-180 to +180)
     */
    void toEulerAngles(float& roll_deg, float& pitch_deg, float& yaw_deg) const {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        // Roll (X-axis rotation)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

        // Pitch (Y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f) {
            pitch_deg = copysignf(90.0f, sinp);
        } else {
            pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);
        }

        // Yaw (Z-axis rotation)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);
    }

    /**
     * @brief Get current attitude as unit quaternion [w, x, y, z]
     */
    void getQuaternion(float& qw, float& qx, float& qy, float& qz) const {
        qw = q[0];
        qx = q[1];
        qy = q[2];
        qz = q[3];
    }

    /**
     * @brief Get current estimated gyro biases in rad/s
     */
    void getGyroBias(float& bx, float& by, float& bz) const {
        bx = bg[0];
        by = bg[1];
        bz = bg[2];
    }

    /**
     * @brief Reset Yaw heading to a given value (default 0 deg) without affecting Pitch & Roll
     */
    void resetHeading(float target_yaw_deg = 0.0f) {
        float r, p, y;
        toEulerAngles(r, p, y);
        setEuler(r * ((float)M_PI / 180.0f),
                 p * ((float)M_PI / 180.0f),
                 target_yaw_deg * ((float)M_PI / 180.0f));
    }

private:
    void normalizeQuaternion() {
        float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm > 1.0e-9f) {
            float inv = 1.0f / norm;
            q[0] *= inv;
            q[1] *= inv;
            q[2] *= inv;
            q[3] *= inv;
        } else {
            q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        }
    }

    void symmetrizeP() {
        for (int i = 0; i < 6; i++) {
            for (int j = i + 1; j < 6; j++) {
                float avg = 0.5f * (P[i][j] + P[j][i]);
                P[i][j] = avg;
                P[j][i] = avg;
            }
            // Enforce minimum positive variance on diagonals
            if (P[i][i] < 1.0e-8f) P[i][i] = 1.0e-8f;
        }
    }

    bool invert3x3(const float A[3][3], float inv[3][3]) {
        float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        if (fabsf(det) < 1.0e-9f) return false;

        float inv_det = 1.0f / det;
        inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
        inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
        inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;

        inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
        inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
        inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;

        inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
        inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
        inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

        return true;
    }
};
