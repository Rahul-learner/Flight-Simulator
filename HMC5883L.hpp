#pragma once

#include <stdint.h>
#include <math.h>
#include "hardware/i2c.h"
#include "pico/time.h"

/**
 * @file HMC5883L.hpp
 * @brief Universal I2C Driver for HMC5883L and QMC5883L 3-Axis Digital Magnetometers.
 * 
 * Automatically detects whether the connected sensor is a genuine Honeywell HMC5883L (0x1E)
 * or a modern QMC5883L (0x0D) clone, seamlessly handling register differences and endianness.
 */

class HMC5883L {
public:
    enum SensorType {
        TYPE_NONE = 0,
        TYPE_HMC5883L = 1,
        TYPE_QMC5883L = 2
    };

    SensorType sensor_type;
    uint8_t i2c_addr;
    i2c_inst_t* i2c_port;

    // Hard-iron and soft-iron calibration offsets & scale factors
    float offset_x, offset_y, offset_z;
    float scale_x, scale_y, scale_z;

    // Last calibrated magnetic field in microteslas (uT) or normalized units
    float mx, my, mz;

    HMC5883L(i2c_inst_t* port = i2c0) :
        sensor_type(TYPE_NONE),
        i2c_addr(0x1E),
        i2c_port(port),
        offset_x(0.0f), offset_y(0.0f), offset_z(0.0f),
        scale_x(1.0f), scale_y(1.0f), scale_z(1.0f),
        mx(0.0f), my(0.0f), mz(0.0f) {}

    /**
     * @brief Detect and initialize the magnetometer
     * @return true if HMC5883L or QMC5883L was detected and initialized
     */
    bool init() {
        // Try HMC5883L (Address 0x1E) first
        if (checkAddress(0x1E)) {
            i2c_addr = 0x1E;
            sensor_type = TYPE_HMC5883L;

            // Config A (0x00): 8 samples averaged, 75 Hz data rate, normal measurement (0x78)
            writeReg(0x00, 0x78);

            // Config B (0x01): Gain = 1090 LSB/Gauss (+/- 1.3 Gauss range) (0x20)
            writeReg(0x01, 0x20);

            // Mode (0x02): Continuous measurement mode (0x00)
            writeReg(0x02, 0x00);

            sleep_ms(20);
            return true;
        }

        // Try QMC5883L (Address 0x0D) fallback
        if (checkAddress(0x0D)) {
            i2c_addr = 0x0D;
            sensor_type = TYPE_QMC5883L;

            // Set/Reset Period register (0x0B): Recommended value 0x01
            writeReg(0x0B, 0x01);

            // Control Register 1 (0x09): Mode = Continuous, Output Data Rate = 100Hz, Range = 8G, Over Sample Rate = 512
            // 0b00011101 = 0x1D
            writeReg(0x09, 0x1D);

            sleep_ms(20);
            return true;
        }

        sensor_type = TYPE_NONE;
        return false;
    }

    /**
     * @brief Read raw 3-axis magnetic data and apply calibration
     * @return true on successful I2C read
     */
    bool read(float& out_mx, float& out_my, float& out_mz) {
        if (sensor_type == TYPE_HMC5883L) {
            uint8_t reg = 0x03; // Data Output X MSB
            uint8_t buf[6];

            if (i2c_write_blocking(i2c_port, i2c_addr, &reg, 1, true) < 0) return false;
            if (i2c_read_blocking(i2c_port, i2c_addr, buf, 6, false) < 0) return false;

            // HMC5883L register order is X, Z, Y (Big Endian)
            int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t raw_z = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t raw_y = (int16_t)((buf[4] << 8) | buf[5]);

            // Apply hard-iron offset and soft-iron scale
            mx = ((float)raw_x - offset_x) * scale_x;
            my = ((float)raw_y - offset_y) * scale_y;
            mz = ((float)raw_z - offset_z) * scale_z;

            out_mx = mx;
            out_my = my;
            out_mz = mz;
            return true;

        } else if (sensor_type == TYPE_QMC5883L) {
            uint8_t reg = 0x00; // Data Output X LSB
            uint8_t buf[6];

            if (i2c_write_blocking(i2c_port, i2c_addr, &reg, 1, true) < 0) return false;
            if (i2c_read_blocking(i2c_port, i2c_addr, buf, 6, false) < 0) return false;

            // QMC5883L register order is X, Y, Z (Little Endian)
            int16_t raw_x = (int16_t)(buf[0] | (buf[1] << 8));
            int16_t raw_y = (int16_t)(buf[2] | (buf[3] << 8));
            int16_t raw_z = (int16_t)(buf[4] | (buf[5] << 8));

            mx = ((float)raw_x - offset_x) * scale_x;
            my = ((float)raw_y - offset_y) * scale_y;
            mz = ((float)raw_z - offset_z) * scale_z;

            out_mx = mx;
            out_my = my;
            out_mz = mz;
            return true;
        }

        return false;
    }

    /**
     * @brief Compute tilt-compensated magnetic heading in degrees (0 to 360)
     * @param roll_rad Roll angle from EKF in radians
     * @param pitch_rad Pitch angle from EKF in radians
     * @return Heading in degrees relative to Magnetic North
     */
    float getHeadingDeg(float roll_rad, float pitch_rad) {
        float cos_r = cosf(roll_rad);
        float sin_r = sinf(roll_rad);
        float cos_p = cosf(pitch_rad);
        float sin_p = sinf(pitch_rad);

        // Project 3D magnetic field onto horizontal plane
        float Xh = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
        float Yh = my * cos_r - mz * sin_r;

        float heading_rad = atan2f(-Yh, Xh);
        float heading_deg = heading_rad * (180.0f / (float)M_PI);

        if (heading_deg < 0.0f) heading_deg += 360.0f;
        return heading_deg;
    }

    void setCalibration(float ox, float oy, float oz, float sx = 1.0f, float sy = 1.0f, float sz = 1.0f) {
        offset_x = ox;
        offset_y = oy;
        offset_z = oz;
        scale_x = sx;
        scale_y = sy;
        scale_z = sz;
    }

    const char* getSensorName() const {
        if (sensor_type == TYPE_HMC5883L) return "Honeywell HMC5883L (0x1E)";
        if (sensor_type == TYPE_QMC5883L) return "QST QMC5883L (0x0D)";
        return "Not Detected";
    }

private:
    bool checkAddress(uint8_t addr) {
        uint8_t rxdata;
        int ret = i2c_read_blocking_until(i2c_port, addr, &rxdata, 1, false, make_timeout_time_ms(10));
        return (ret >= 0);
    }

    void writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        i2c_write_blocking(i2c_port, i2c_addr, buf, 2, false);
    }
};
