/**
 * ============================================================================
 *   Koyal Flight Controller - 9-DOF AHRS + Barometer + Telemetry Deck
 * ============================================================================
 * 
 * Hardware Connections:
 * - Raspberry Pi Pico (overclocked to 250 MHz)
 * - MPU6050 6-DOF IMU (I2C0: SDA=GP20, SCL=GP21, INT=GP15)
 * - HMC5883L / QMC5883L Magnetometer (I2C0: SDA=GP20, SCL=GP21)
 * - BMP180 Barometer (I2C0: SDA=GP20, SCL=GP21)
 * - ESP8266 Wi-Fi Telemetry Server (UART0: TX=GP0, RX=GP1 at 115200 baud)
 * - PC USB Serial (Debug monitoring & commands)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "AdvancedAttitudeEKF.hpp" // 9-DOF Error-State Extended Kalman Filter
#include "HMC5883L.hpp"           // Auto-detecting HMC5883L / QMC5883L Digital Compass
#include "BMP180.hpp"             // High-precision Barometer & Variometer

// --- Pin Definitions ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

// UART0 to ESP8266 (soldered on perfboard to GP16 / GP17)
#define ESP_UART_PORT uart0
#define ESP_UART_TX_PIN 16  // Pico GP16 (Pin 21, UART0 TX) -> ESP8266 D5 (GPIO14)
#define ESP_UART_RX_PIN 17  // Pico GP17 (Pin 22, UART0 RX) <- ESP8266 D6 (GPIO12)
#define ESP_UART_BAUD_RATE 38400

// Proven 23-byte Binary Telemetry Packet Protocol (matches FC_UI_esp8266 and perfboard)
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    int16_t roll;            // deg * 100 (-18000 to 18000)
    int16_t pitch;           // deg * 100 (-9000 to 9000)
    int16_t yaw;             // deg * 100 (-18000 to 18000)
    int16_t dt;              // loop microseconds
    int16_t alt_cm;          // cm AGL
    int16_t vsi_cms;         // cm/s vertical speed
    int16_t prs_hpa10;       // hPa * 10
    int16_t roll_pid = 0;
    int16_t pitch_pid = 0;
    int16_t yaw_pid = 0;
    uint8_t checksum;        // XOR of 20 bytes from roll to yaw_pid
};
#pragma pack(pop)

static TelemetryPacket telemetry_packet;

// I2C0 Bus Configuration (shared by MPU6050, HMC5883L, BMP180)
#define SENSORS_I2C_PORT i2c0
#define SENSORS_SDA_PIN 20   // Pico GP20 (Pin 26)
#define SENSORS_SCL_PIN 21   // Pico GP21 (Pin 27)
#define SENSORS_BAUD_RATE 400000 // Fast mode 400 kHz

// MPU6050 Interrupt Pin
#define MPU6050_INT_PIN 15   // Pico GP15 (Pin 20)

// --- MPU6050 Registers & Configuration ---
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

#define ACCEL_SCALE_FACTOR 16384.0f // LSB/g (+/- 2g)
#define GYRO_SCALE_FACTOR 131.0f    // LSB/(deg/s) (+/- 250 deg/s)

// --- Sensor Drivers and Filter Instances ---
AdvancedAttitudeEKF ekf;
HMC5883L compass(SENSORS_I2C_PORT);
BMP180 barometer(SENSORS_I2C_PORT);

// Global raw sensor readings
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;

// Attitude angles (degrees)
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
volatile bool new_imu_data = false;

// Timing and rate counters
static uint64_t last_update_us = 0;
static uint32_t telemetry_counter = 0;
#define TELEMETRY_DIVIDER 40 // At 2000Hz IMU rate, 40 iterations = 50 Hz telemetry stream

// Telemetry transmit buffer
static char telem_buf[280];

// UART RX buffer
#define RX_BUF_SIZE 32
static char rx_buffer[RX_BUF_SIZE];
static uint8_t rx_index = 0;

// --- I2C Helper Functions ---
void i2c_write_byte(uint8_t addr, uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    i2c_write_blocking(SENSORS_I2C_PORT, addr, buf, 2, false);
}

void i2c_read_bytes(uint8_t addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    i2c_write_blocking(SENSORS_I2C_PORT, addr, &reg_addr, 1, true);
    i2c_read_blocking(SENSORS_I2C_PORT, addr, data, len, false);
}

// --- MPU6050 Initialization ---
void mpu6050_init() {
    // Wake up MPU6050
    i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    sleep_ms(80);

    // DLPF Config: 0 (widest bandwidth, 8kHz gyro, 4kHz accel rate)
    i2c_write_byte(MPU6050_ADDR, MPU6050_CONFIG, 0x00);

    // Sample rate divider: 3 -> 2000 Hz internal rate
    i2c_write_byte(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x03);

    // Full-scale ranges: Gyro +/- 250 deg/s, Accel +/- 2g
    i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);

    // Enable I2C Bypass (Bit 1 = BYPASS_EN).
    // This allows Pico to directly address HMC5883L & BMP180 on the same bus or AUX pins!
    i2c_write_byte(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02);

    // Enable Data Ready Interrupt
    i2c_write_byte(MPU6050_ADDR, MPU6050_INT_ENABLE, 0x01);
}

void mpu6050_read_raw() {
    uint8_t data[14];
    i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);

    accel_x_raw = (int16_t)(data[0] << 8 | data[1]);
    accel_y_raw = (int16_t)(data[2] << 8 | data[3]);
    accel_z_raw = (int16_t)(data[4] << 8 | data[5]);

    gyro_x_raw = (int16_t)(data[8] << 8 | data[9]);
    gyro_y_raw = (int16_t)(data[10] << 8 | data[11]);
    gyro_z_raw = (int16_t)(data[12] << 8 | data[13]);
}

void mpu6050_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        new_imu_data = true;
    }
}

// --- Startup Calibration Routine (9-DOF Alignment) ---
void calibrate_and_align_ahrs() {
    printf("\n>>> Calibrating 9-DOF Sensors & Aligning AHRS...\n");
    printf(">>> Keep aircraft STATIONARY on a level surface...\n");

    const int SAMPLES = 500;
    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    float mx_sum = 0.0f, my_sum = 0.0f, mz_sum = 0.0f;
    int mag_samples = 0;

    for (int i = 0; i < SAMPLES; i++) {
        mpu6050_read_raw();

        ax_sum += (float)accel_x_raw / ACCEL_SCALE_FACTOR;
        ay_sum += (float)accel_y_raw / ACCEL_SCALE_FACTOR;
        az_sum += (float)accel_z_raw / ACCEL_SCALE_FACTOR;

        gx_sum += ((float)gyro_x_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);
        gy_sum += ((float)gyro_y_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);
        gz_sum += ((float)gyro_z_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);

        float cur_mx, cur_my, cur_mz;
        if (compass.read(cur_mx, cur_my, cur_mz)) {
            mx_sum += cur_mx;
            my_sum += cur_my;
            mz_sum += cur_mz;
            mag_samples++;
        }

        // Pulse LED during calibration
        gpio_put(PICO_DEFAULT_LED_PIN, (i / 25) % 2);
        sleep_ms(3);
    }

    float ax_avg = ax_sum / SAMPLES;
    float ay_avg = ay_sum / SAMPLES;
    float az_avg = az_sum / SAMPLES;

    float gx_bias = gx_sum / SAMPLES;
    float gy_bias = gy_sum / SAMPLES;
    float gz_bias = gz_sum / SAMPLES;

    float mx_avg = (mag_samples > 0) ? (mx_sum / mag_samples) : 0.0f;
    float my_avg = (mag_samples > 0) ? (my_sum / mag_samples) : 0.0f;
    float mz_avg = (mag_samples > 0) ? (mz_sum / mag_samples) : 0.0f;

    // Tare ground pressure for relative AGL altitude = 0.0m
    barometer.tareGroundPressure();

    // Initialize EKF with initial tilt from gravity and initial heading from compass
    ekf.initFromSensors(ax_avg, ay_avg, az_avg, gx_bias, gy_bias, gz_bias, mx_avg, my_avg, mz_avg);
    ekf.toEulerAngles(roll, pitch, yaw);

    printf(">>> AHRS Calibration Complete!\n");
    printf("    Gravity Norm : %.3f g\n", sqrtf(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg));
    printf("    Gyro Biases  : X=%.2f, Y=%.2f, Z=%.2f deg/s\n",
           gx_bias * 180.0f / (float)M_PI, gy_bias * 180.0f / (float)M_PI, gz_bias * 180.0f / (float)M_PI);
    printf("    Compass Type : %s\n", compass.getSensorName());
    printf("    Initial Euler: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f° (Magnetic North locked)\n",
           roll, pitch, yaw);
    printf("    Ground Alt   : 0.0 m (Pressure: %.1f hPa, Temp: %.1f °C)\n\n",
           barometer.pressure_hpa, barometer.temperature_c);

    // Solid LED indicates AHRS ready
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

// --- Process UART Commands from ESP8266 or PC USB ---
void handle_incoming_commands() {
    // Check USB command from PC
    int usb_c = getchar_timeout_us(0);
    if (usb_c == 'T' || usb_c == 't') {
        ekf.resetHeading(0.0f);
        barometer.tareGroundPressure();
        printf("[USB CMD] Heading & Baro Zeroed (TARE)\n");
    } else if (usb_c == 'C' || usb_c == 'c') {
        calibrate_and_align_ahrs();
    } else if (usb_c == 'I' || usb_c == 'i') {
        printf("[INFO] clk_sys: %lu Hz, clk_peri: %lu Hz, GP16 func: %u, GP17 func: %u, uart0 writable: %d\n",
               clock_get_hz(clk_sys), clock_get_hz(clk_peri),
               gpio_get_function(ESP_UART_TX_PIN), gpio_get_function(ESP_UART_RX_PIN),
               uart_is_writable(ESP_UART_PORT));
    } else if (usb_c == 'P' || usb_c == 'p') {
        printf("[PIN_TEST] Starting pin test on GP16 and GP17...\n");
        uart_deinit(ESP_UART_PORT);
        gpio_init(ESP_UART_TX_PIN);
        gpio_init(ESP_UART_RX_PIN);
        gpio_set_dir(ESP_UART_TX_PIN, GPIO_OUT);
        gpio_set_dir(ESP_UART_RX_PIN, GPIO_OUT);

        // Keep both HIGH initially
        gpio_put(ESP_UART_TX_PIN, 1);
        gpio_put(ESP_UART_RX_PIN, 1);
        sleep_ms(1000);

        // Test GP16 alone: pull LOW for 3 seconds
        printf("[PIN_TEST] GP16_ONLY_LOW: GP16=LOW, GP17=HIGH for 3s\n");
        gpio_put(ESP_UART_TX_PIN, 0);
        gpio_put(ESP_UART_RX_PIN, 1);
        sleep_ms(3000);

        // Release GP16 back to HIGH
        printf("[PIN_TEST] GP16_RESTORE_HIGH: GP16=HIGH, GP17=HIGH for 2s\n");
        gpio_put(ESP_UART_TX_PIN, 1);
        gpio_put(ESP_UART_RX_PIN, 1);
        sleep_ms(2000);

        // Test GP17 alone: pull LOW for 3 seconds
        printf("[PIN_TEST] GP17_ONLY_LOW: GP16=HIGH, GP17=LOW for 3s\n");
        gpio_put(ESP_UART_TX_PIN, 1);
        gpio_put(ESP_UART_RX_PIN, 0);
        sleep_ms(3000);

        // Release GP17 back to HIGH
        printf("[PIN_TEST] GP17_RESTORE_HIGH: GP16=HIGH, GP17=HIGH for 2s\n");
        gpio_put(ESP_UART_TX_PIN, 1);
        gpio_put(ESP_UART_RX_PIN, 1);
        sleep_ms(2000);

        // Restore UART0
        uart_init(ESP_UART_PORT, ESP_UART_BAUD_RATE);
        gpio_set_function(ESP_UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(ESP_UART_RX_PIN, GPIO_FUNC_UART);
        printf("[PIN_TEST] Completed. UART0 restored on GP16/GP17.\n");
    }

    // Check UART from ESP8266
    while (uart_is_readable(ESP_UART_PORT)) {
        char c = (char)uart_getc(ESP_UART_PORT);
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                if (strcmp(rx_buffer, "TARE") == 0 || strcmp(rx_buffer, "ZERO") == 0) {
                    ekf.resetHeading(0.0f);
                    barometer.tareGroundPressure();
                    printf("[UART CMD] Heading & Baro Zeroed (TARE)\n");
                    uart_puts(ESP_UART_PORT, "{\"msg\":\"Tare executed\"}\n");
                } else if (strcmp(rx_buffer, "CALIB") == 0) {
                    calibrate_and_align_ahrs();
                    uart_puts(ESP_UART_PORT, "{\"msg\":\"Recalibration done\"}\n");
                }
                rx_index = 0;
            }
        } else if (rx_index < (RX_BUF_SIZE - 1)) {
            rx_buffer[rx_index++] = c;
        }
    }
}

int main() {
    // --- Overclock Pico to 250 MHz for maximum math throughput ---
    set_sys_clock_khz(250000, true);

    // Initialize USB Serial for PC debugging
    stdio_init_all();
    sleep_ms(1500); // Allow PC serial terminal to connect

    printf("\n===========================================================\n");
    printf("   Koyal Flight Controller - 9-DOF AHRS + Barometer       \n");
    printf("   RP2040 Core @ %lu MHz | Telemetry UART: GP%d/GP%d       \n",
           clock_get_hz(clk_sys) / 1000000, ESP_UART_TX_PIN, ESP_UART_RX_PIN);
    printf("===========================================================\n\n");

    // Initialize onboard LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // Initialize Hardware UART0 to ESP8266
    uint actual_baud = uart_init(ESP_UART_PORT, ESP_UART_BAUD_RATE);
    gpio_set_function(ESP_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ESP_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(ESP_UART_PORT, false, false);
    uart_set_format(ESP_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(ESP_UART_PORT, true);
    printf("[INIT] UART0 initialized on TX GP%d, RX GP%d at %d baud (actual %u baud, clk_peri=%lu Hz).\n",
           ESP_UART_TX_PIN, ESP_UART_RX_PIN, ESP_UART_BAUD_RATE, actual_baud, clock_get_hz(clk_peri));

    // Initialize I2C0 Bus for MPU6050, HMC5883L, BMP180
    i2c_init(SENSORS_I2C_PORT, SENSORS_BAUD_RATE);
    gpio_set_function(SENSORS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSORS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSORS_SDA_PIN);
    gpio_pull_up(SENSORS_SCL_PIN);
    printf("[INIT] I2C0 initialized on SDA GP%d, SCL GP%d at %d Hz.\n",
           SENSORS_SDA_PIN, SENSORS_SCL_PIN, SENSORS_BAUD_RATE);

    // Scan I2C devices on bus
    printf("[INIT] Scanning I2C bus for connected sensors...\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int res = i2c_read_blocking_until(SENSORS_I2C_PORT, addr, &dummy, 1, false, make_timeout_time_ms(5));
        if (res >= 0) {
            printf("  -> Found I2C Device at 0x%02X ", addr);
            if (addr == 0x68) printf("(MPU6050 IMU)\n");
            else if (addr == 0x1E) printf("(HMC5883L Compass)\n");
            else if (addr == 0x0D) printf("(QMC5883L Compass)\n");
            else if (addr == 0x77) printf("(BMP180 Barometer)\n");
            else printf("\n");
        }
    }

    // Initialize MPU6050
    mpu6050_init();
    printf("[INIT] MPU6050 6-DOF IMU initialized.\n");

    // Configure MPU6050 Data Ready Interrupt
    gpio_init(MPU6050_INT_PIN);
    gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU6050_INT_PIN);
    gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu6050_interrupt_handler);
    printf("[INIT] MPU6050 INT attached on GP%d.\n", MPU6050_INT_PIN);

    // Initialize HMC5883L / QMC5883L Magnetometer
    if (compass.init()) {
        printf("[INIT] Magnetometer initialized: %s.\n", compass.getSensorName());
    } else {
        printf("[WARN] Magnetometer NOT detected on 0x1E / 0x0D! Will run 6-DOF IMU mode.\n");
    }

    // Initialize BMP180 Barometer
    if (barometer.init()) {
        printf("[INIT] BMP180 Barometer initialized. Base Pressure: %.1f hPa, Temp: %.1f °C.\n",
               barometer.pressure_hpa, barometer.temperature_c);
    } else {
        printf("[WARN] BMP180 Barometer NOT detected on 0x77!\n");
    }

    // Stabilize sensors
    sleep_ms(300);

    // Perform initial 9-DOF calibration and attitude alignment
    calibrate_and_align_ahrs();

    last_update_us = time_us_64();
    uint64_t last_mag_read_us = time_us_64();

    // --- Main Flight Control Loop ---
    while (true) {
        // 1. Process asynchronous non-blocking BMP180 Barometer conversions
        barometer.updateAsync();

        // 2. Poll HMC5883L Magnetometer at 50 Hz (~20ms)
        uint64_t now_us = time_us_64();
        if ((now_us - last_mag_read_us) >= 20000) {
            last_mag_read_us = now_us;
            if (compass.read(mag_x, mag_y, mag_z)) {
                // Fuse magnetometer heading into 9-DOF Kalman Filter
                ekf.updateMag(mag_x, mag_y, mag_z);
            }
            // Check for incoming commands from ESP8266 or PC
            handle_incoming_commands();
        }

        // 3. Process high-rate IMU data (MPU6050 interrupt triggered at 2000 Hz)
        if (new_imu_data) {
            new_imu_data = false;

            // Read raw accelerometer and gyro data
            mpu6050_read_raw();

            // Calculate precise loop time dt
            uint64_t current_time_us = time_us_64();
            float dt_s = (float)(current_time_us - last_update_us) * 1.0e-6f;
            last_update_us = current_time_us;

            if (dt_s <= 0.0f || dt_s > 0.05f) {
                dt_s = 0.0005f; // Nominal 2000 Hz step
            }

            // Convert to physical units
            float ax = (float)accel_x_raw / ACCEL_SCALE_FACTOR;
            float ay = (float)accel_y_raw / ACCEL_SCALE_FACTOR;
            float az = (float)accel_z_raw / ACCEL_SCALE_FACTOR;

            float gx = ((float)gyro_x_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);
            float gy = ((float)gyro_y_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);
            float gz = ((float)gyro_z_raw / GYRO_SCALE_FACTOR) * ((float)M_PI / 180.0f);

            // --- Advanced Error-State Kalman Filter ---
            // 1) Prediction Step: high-rate quaternion kinematics
            ekf.predict(dt_s, gx, gy, gz);

            // 2) Measurement Update: accelerometer tilt correction with dynamic G-rejection
            ekf.update(ax, ay, az);

            // 3) Extract Euler angles
            ekf.toEulerAngles(roll, pitch, yaw);

            // --- Real-Time Telemetry Streaming to ESP8266 ---
            telemetry_counter++;
            if (telemetry_counter >= TELEMETRY_DIVIDER) {
                telemetry_counter = 0;

                // Populate 23-byte binary telemetry packet
                telemetry_packet.header1 = 0xAA;
                telemetry_packet.header2 = 0xBB;
                telemetry_packet.roll = (int16_t)(roll * 100.0f);
                telemetry_packet.pitch = (int16_t)(pitch * 100.0f);
                telemetry_packet.yaw = (int16_t)(yaw * 100.0f);
                telemetry_packet.dt = (int16_t)(dt_s * 1000000.0f);
                telemetry_packet.alt_cm = (int16_t)(barometer.altitude_agl_m * 100.0f);
                telemetry_packet.vsi_cms = (int16_t)(barometer.vertical_speed_mps * 100.0f);
                telemetry_packet.prs_hpa10 = (int16_t)(barometer.pressure_hpa * 10.0f);
                telemetry_packet.roll_pid = 0;
                telemetry_packet.pitch_pid = 0;
                telemetry_packet.yaw_pid = (int16_t)(compass.sensor_type != HMC5883L::TYPE_NONE ? 1 : 0);

                // Checksum: XOR of 20 bytes from roll to yaw_pid
                uint8_t chk = 0;
                uint8_t* p = (uint8_t*)&telemetry_packet.roll;
                for (size_t i = 0; i < 20; i++) {
                    chk ^= p[i];
                }
                telemetry_packet.checksum = chk;

                // Transmit binary packet into UART0 FIFO (< 0.1 us, non-blocking)
                uart_write_blocking(ESP_UART_PORT, (const uint8_t*)&telemetry_packet, sizeof(TelemetryPacket));

                // Echo to PC USB Serial Console for debugging
                printf(">R:%.1f P:%.1f Y:%.1f | Alt:%.1fm VSI:%.2fm/s | %s\n",
                       roll, pitch, yaw,
                       barometer.altitude_agl_m, barometer.vertical_speed_mps,
                       compass.sensor_type != HMC5883L::TYPE_NONE ? "9-DOF OK" : "6-DOF");
            }

        } else {
            // Relinquish CPU for 1 microsecond to prevent lockup
            sleep_us(1);
        }
    }

    return 0;
}
