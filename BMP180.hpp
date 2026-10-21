#pragma once

#include <stdint.h>
#include <math.h>
#include "hardware/i2c.h"
#include "pico/time.h"

/**
 * @file BMP180.hpp
 * @brief High-precision, non-blocking I2C Driver for Bosch BMP180 / BMP085 Barometer.
 * 
 * Features:
 * - Factory EEPROM calibration coefficients decoding
 * - Non-blocking state machine (zero jitter for 2000Hz flight controller loop)
 * - Barometric altitude calculation (Absolute ASL & Relative AGL)
 * - Variometer / Vertical Speed Indicator (VSI climb rate in m/s)
 * - Ambient temperature measurement in °C
 */

class BMP180 {
public:
    static const uint8_t I2C_ADDR = 0x77;

    // Calibration coefficients
    int16_t  ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t  b1, b2, mb, mc, md;

    i2c_inst_t* i2c_port;
    bool initialized;

    // Oversampling setting: 0 = Ultra Low Power, 1 = Standard, 2 = High Res, 3 = Ultra High Res
    uint8_t oversampling;

    // Current sensor measurements
    float temperature_c;     // Temperature in °C
    float pressure_pa;       // Pressure in Pascals (Pa)
    float pressure_hpa;      // Pressure in hPa / millibars
    float altitude_asl_m;    // Altitude Above Sea Level (meters)
    float altitude_agl_m;    // Relative Altitude Above Ground Level (meters)
    float vertical_speed_mps;// Climb / descent rate (m/s)

    float sea_level_pressure_pa; // Standard 101325.0 Pa
    float ground_pressure_pa;    // Calibrated ground pressure

    // Non-blocking state machine variables
    enum State {
        IDLE = 0,
        WAIT_TEMP,
        WAIT_PRESS
    };
    State state;
    uint64_t request_time_us;
    int32_t raw_temp;
    int32_t raw_pressure;
    float last_alt_agl;
    uint64_t last_alt_time_us;

    BMP180(i2c_inst_t* port = i2c0) :
        i2c_port(port),
        initialized(false),
        oversampling(2), // Mode 2: High Resolution (14ms conversion time)
        temperature_c(25.0f),
        pressure_pa(101325.0f),
        pressure_hpa(1013.25f),
        altitude_asl_m(0.0f),
        altitude_agl_m(0.0f),
        vertical_speed_mps(0.0f),
        sea_level_pressure_pa(101325.0f),
        ground_pressure_pa(101325.0f),
        state(IDLE),
        request_time_us(0),
        raw_temp(0),
        raw_pressure(0),
        last_alt_agl(0.0f),
        last_alt_time_us(0) {}

    /**
     * @brief Initialize BMP180 and read calibration coefficients
     */
    bool init() {
        // Verify sensor presence
        uint8_t id_reg = 0xD0;
        uint8_t chip_id = 0;
        if (i2c_write_blocking(i2c_port, I2C_ADDR, &id_reg, 1, true) < 0) return false;
        if (i2c_read_blocking(i2c_port, I2C_ADDR, &chip_id, 1, false) < 0) return false;

        if (chip_id != 0x55) {
            // Chip ID is not 0x55 (BMP180 signature)
            return false;
        }

        // Read 22 bytes of calibration coefficients (0xAA to 0xBF)
        uint8_t calib_buf[22];
        uint8_t reg_start = 0xAA;
        if (i2c_write_blocking(i2c_port, I2C_ADDR, &reg_start, 1, true) < 0) return false;
        if (i2c_read_blocking(i2c_port, I2C_ADDR, calib_buf, 22, false) < 0) return false;

        ac1 = (int16_t)((calib_buf[0] << 8) | calib_buf[1]);
        ac2 = (int16_t)((calib_buf[2] << 8) | calib_buf[3]);
        ac3 = (int16_t)((calib_buf[4] << 8) | calib_buf[5]);
        ac4 = (uint16_t)((calib_buf[6] << 8) | calib_buf[7]);
        ac5 = (uint16_t)((calib_buf[8] << 8) | calib_buf[9]);
        ac6 = (uint16_t)((calib_buf[10] << 8) | calib_buf[11]);
        b1  = (int16_t)((calib_buf[12] << 8) | calib_buf[13]);
        b2  = (int16_t)((calib_buf[14] << 8) | calib_buf[15]);
        mb  = (int16_t)((calib_buf[16] << 8) | calib_buf[17]);
        mc  = (int16_t)((calib_buf[18] << 8) | calib_buf[19]);
        md  = (int16_t)((calib_buf[20] << 8) | calib_buf[21]);

        initialized = true;
        state = IDLE;

        // Perform initial blocking read to get baseline
        blockingRead();
        ground_pressure_pa = pressure_pa;
        last_alt_agl = 0.0f;
        last_alt_time_us = time_us_64();

        return true;
    }

    /**
     * @brief Zero the relative altitude (AGL) at current location/runway
     */
    void tareGroundPressure() {
        ground_pressure_pa = pressure_pa;
        altitude_agl_m = 0.0f;
        vertical_speed_mps = 0.0f;
    }

    /**
     * @brief Non-blocking update method. Call periodically from the main loop!
     */
    void updateAsync() {
        if (!initialized) return;

        uint64_t now_us = time_us_64();

        switch (state) {
            case IDLE:
                // Start temperature conversion (0x2E into 0xF4)
                writeReg(0xF4, 0x2E);
                request_time_us = now_us;
                state = WAIT_TEMP;
                break;

            case WAIT_TEMP:
                // Temperature conversion requires 4.5ms (4500 us)
                if (now_us - request_time_us >= 5000) {
                    // Read raw temperature (0xF6, 2 bytes)
                    uint8_t buf[2];
                    readRegs(0xF6, buf, 2);
                    raw_temp = (int32_t)((buf[0] << 8) | buf[1]);

                    // Start pressure conversion (0x34 + (oss << 6) into 0xF4)
                    writeReg(0xF4, 0x34 | (oversampling << 6));
                    request_time_us = now_us;
                    state = WAIT_PRESS;
                }
                break;

            case WAIT_PRESS: {
                // Conversion delay depends on oversampling:
                // oss 0: 4.5ms, oss 1: 7.5ms, oss 2: 13.5ms, oss 3: 25.5ms
                uint64_t wait_us = (oversampling == 3) ? 26000 :
                                   (oversampling == 2) ? 14000 :
                                   (oversampling == 1) ? 8000 : 5000;

                if (now_us - request_time_us >= wait_us) {
                    // Read raw pressure (0xF6, 3 bytes: MSB, LSB, XLSB)
                    uint8_t buf[3];
                    readRegs(0xF6, buf, 3);
                    raw_pressure = (((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2]) >> (8 - oversampling);

                    // Compute compensated temperature and pressure
                    computeTrueValues();

                    state = IDLE; // Ready for next cycle
                }
                break;
            }
        }
    }

    /**
     * @brief Blocking single read (used during calibration at boot)
     */
    void blockingRead() {
        if (!initialized) return;

        // 1. Read Raw Temperature
        writeReg(0xF4, 0x2E);
        sleep_ms(5);
        uint8_t buf[3];
        readRegs(0xF6, buf, 2);
        raw_temp = (int32_t)((buf[0] << 8) | buf[1]);

        // 2. Read Raw Pressure
        writeReg(0xF4, 0x34 | (oversampling << 6));
        sleep_ms(15);
        readRegs(0xF6, buf, 3);
        raw_pressure = (((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2]) >> (8 - oversampling);

        computeTrueValues();
    }

private:
    void writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        i2c_write_blocking(i2c_port, I2C_ADDR, buf, 2, false);
    }

    void readRegs(uint8_t reg, uint8_t* dest, uint8_t len) {
        i2c_write_blocking(i2c_port, I2C_ADDR, &reg, 1, true);
        i2c_read_blocking(i2c_port, I2C_ADDR, dest, len, false);
    }

    void computeTrueValues() {
        // Bosch BMP180 official temperature compensation algorithm
        int32_t x1 = ((raw_temp - (int32_t)ac6) * (int32_t)ac5) >> 15;
        int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
        int32_t b5 = x1 + x2;
        temperature_c = (float)((b5 + 8) >> 4) / 10.0f;

        // Bosch BMP180 official pressure compensation algorithm
        int32_t b6 = b5 - 4000;
        x1 = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
        x2 = ((int32_t)ac2 * b6) >> 11;
        int32_t x3 = x1 + x2;
        int32_t b3 = ((((int32_t)ac1 * 4 + x3) << oversampling) + 2) >> 2;

        x1 = ((int32_t)ac3 * b6) >> 13;
        x2 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
        uint32_t b7 = ((uint32_t)raw_pressure - b3) * (50000 >> oversampling);

        int32_t p;
        if (b7 < 0x80000000) {
            p = (int32_t)((b7 * 2) / b4);
        } else {
            p = (int32_t)((b7 / b4) * 2);
        }

        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        p = p + ((x1 + x2 + 3791) >> 4);

        pressure_pa = (float)p;
        pressure_hpa = pressure_pa / 100.0f;

        // Hypsometric formula for altitude:
        // h = 44330.0 * (1.0 - (P / P0)^(1.0 / 5.255))
        altitude_asl_m = 44330.0f * (1.0f - powf(pressure_pa / sea_level_pressure_pa, 0.1902949f));
        float current_alt_agl = 44330.0f * (1.0f - powf(pressure_pa / ground_pressure_pa, 0.1902949f));

        // Smooth relative altitude with complementary filter
        altitude_agl_m = 0.85f * altitude_agl_m + 0.15f * current_alt_agl;

        // Calculate Vertical Speed Indicator (climb rate in m/s)
        uint64_t now_us = time_us_64();
        float dt = (float)(now_us - last_alt_time_us) * 1.0e-6f;
        if (dt > 0.01f && dt < 1.0f) {
            float instant_vsi = (altitude_agl_m - last_alt_agl) / dt;
            // Low-pass filter for smooth variometer
            vertical_speed_mps = 0.80f * vertical_speed_mps + 0.20f * instant_vsi;
            last_alt_agl = altitude_agl_m;
            last_alt_time_us = now_us;
        }
    }
};
