/* Self header */
#include <lis2dh12.h>

/* Config */
#ifndef CONFIG_LIS2DH12_DEBUG_ENABLED
#define CONFIG_LIS2DH12_DEBUG_ENABLED 1
#endif

/* Macros */
#if CONFIG_LIS2DH12_DEBUG_ENABLED
#define CONFIG_LIS2DH12_DEBUG_FUNCTION(x) Serial.println(x)
#else
#define CONFIG_LIS2DH12_DEBUG_FUNCTION(x)
#endif

/**
 * @brief Configures the driver for I2C communication
 *
 * Sets up the LIS2DH12 to communicate over I2C. The I2C address depends on the
 * SA0 pin: 0x18 when SA0 is LOW, 0x19 when SA0 is HIGH.
 *
 * @param[in] i2c_library Reference to the TwoWire I2C library instance (typically Wire)
 * @param[in] i2c_address I2C address of the device (default: 0x18)
 * @return 0 on success, or a negative error code otherwise
 * @note Call this from the Arduino setup() function
 * @note Make sure the I2C library has been initialized with Wire.begin()
 */
int lis2dh12::setup(TwoWire &i2c_library, uint8_t i2c_address) {

    /* Disable spi */
    m_spi_library = NULL;

    /* Enable i2c */
    m_i2c_library = &i2c_library;
    m_i2c_address = i2c_address;

    /* Return success */
    return 0;
}

/**
 * @brief Configures the driver for SPI communication
 *
 * Sets up the LIS2DH12 to communicate over SPI. Only 4-wire SPI is supported.
 *
 * @param[in] spi_library Reference to the SPIClass library instance (typically SPI)
 * @param[in] spi_pin_cs GPIO pin number connected to the chip select (CS) pin
 * @param[in] spi_speed SPI clock speed in Hz (default: 10000000)
 * @return 0 on success, or a negative error code otherwise
 * @note Call this from the Arduino setup() function
 * @note Make sure the SPI library has been initialized with SPI.begin()
 */
int lis2dh12::setup(SPIClass &spi_library, const int spi_pin_cs, const int spi_speed) {
    int res;

    /* Disable i2c */
    m_i2c_library = NULL;

    /* Enable spi */
    m_spi_library = &spi_library;
    m_spi_pin_cs = spi_pin_cs;
    m_spi_speed = spi_speed;

    /* Return success */
    return 0;
}

/**
 * @brief Detects if the LIS2DH12 device is present
 *
 * Reads the device ID register and verifies it matches the expected value (0x33).
 *
 * @return true if device is detected, false otherwise
 */
bool lis2dh12::detect(void) {
    int res;

    /* Read identification register */
    uint8_t reg_whoami;
    res = register_read(LIS2DH12_REGISTER_WHO_AM_I, reg_whoami);
    if (res < 0) return false;

    /* Return whether or not the value matches */
    return (reg_whoami == 0x33);
}

/**
 * @brief Gets the current measurement range
 *
 * Reads the CTRL_REG4 register to determine the current full-scale range setting.
 *
 * @param[out] range_g Output parameter for the current range setting
 * @return 0 on success, or a negative error code otherwise
 * @see Datasheet section 8.9 "CTRL_REG4 (23h)"
 */
int lis2dh12::range_get(enum lis2dh12_range &range_g) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg4;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read full scale register!");
        return res;
    }

    /* Return scale */
    range_g = (enum lis2dh12_range)(1 << (((reg_ctrl_reg4 & 0x30) >> 4) + 1));

    /* Return success */
    return 0;
}

/**
 * @brief Sets the measurement range
 *
 * Configures the full-scale range by setting the FS bits in CTRL_REG4.
 * If activity detection is configured, it will be automatically adjusted
 * to the new range.
 *
 * @param[in] range_g Desired range (LIS2DH12_RANGE_2G, LIS2DH12_RANGE_4G, etc.)
 * @return 0 on success, or a negative error code otherwise
 * @see Datasheet section 8.9 "CTRL_REG4 (23h)"
 */
int lis2dh12::range_set(const enum lis2dh12_range range_g) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg4;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read full scale register!");
        return res;
    }

    /* Modify register */
    reg_ctrl_reg4 &= 0xCF;
    switch (range_g) {
        case LIS2DH12_RANGE_2G: {
            reg_ctrl_reg4 |= (0 << 4);
            break;
        }
        case LIS2DH12_RANGE_4G: {
            reg_ctrl_reg4 |= (1 << 4);
            break;
        }
        case LIS2DH12_RANGE_8G: {
            reg_ctrl_reg4 |= (2 << 4);
            break;
        }
        case LIS2DH12_RANGE_16G: {
            reg_ctrl_reg4 |= (3 << 4);
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write full scale register!");
        return res;
    }

    /* Update activity threshold accordingly */
    if (m_activity_threshold > 0) {
        res = activity_configure(m_activity_threshold, m_activity_duration);
        if (res < 0) {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to re-adjust activity!");
            return res;
        }
    }

    /* Return success */
    return 0;
}

/**
 * @brief Gets the current resolution setting
 *
 * Reads the control registers to determine the current resolution mode
 * (8-bit, 10-bit, or 12-bit).
 *
 * @param[out] resolution Output parameter for the current resolution setting
 * @return 0 on success, or a negative error code otherwise
 * @note This function is not yet fully implemented
 */
int lis2dh12::resolution_get(enum lis2dh12_resolution &resolution) {

    // TODO

    /* Return success */
    return 0;
}

/**
 * @brief Sets the output data resolution
 *
 * Configures the resolution mode by setting the LPEN and HR bits in the
 * control registers. Higher resolution provides better accuracy but may
 * consume more power.
 *
 * @param[in] resolution Desired resolution (LIS2DH12_RESOLUTION_8BITS, etc.)
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::resolution_set(const enum lis2dh12_resolution resolution) {
    int res;

    /* Read registers */
    uint8_t reg_ctrl_reg1;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }
    uint8_t reg_ctrl_reg4;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify registers */
    switch (resolution) {
        case LIS2DH12_RESOLUTION_8BITS: {
            reg_ctrl_reg1 |= (1 << 3);   // LPEN = 1
            reg_ctrl_reg4 &= ~(1 << 3);  // HR = 0
            break;
        }
        case LIS2DH12_RESOLUTION_10BITS: {
            reg_ctrl_reg1 &= ~(1 << 3);  // LPEN = 0
            reg_ctrl_reg4 &= ~(1 << 3);  // HR = 0
            break;
        }
        case LIS2DH12_RESOLUTION_12BITS: {
            reg_ctrl_reg1 &= ~(1 << 3);  // LPEN = 0
            reg_ctrl_reg4 |= (1 << 3);   // HR = 1
            break;
        }
    }

    /* Write registers */
    res = register_write(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Gets the current sampling rate
 *
 * Reads the CTRL_REG1 register to determine the current output data rate (ODR).
 *
 * @param[out] sampling_hz Output parameter for the current sampling rate
 * @return 0 on success, or a negative error code otherwise
 * @see Datasheet section 8.8 "CTRL_REG1 (20h)"
 */
int lis2dh12::sampling_get(enum lis2dh12_sampling &sampling_hz) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read datarate register!");
        return res;
    }

    /* Convert register into hertz */
    switch (reg_ctrl_reg1 >> 4) {
        case 0b0000:
            sampling_hz = LIS2DH12_SAMPLING_NONE;
            break;
        case 0b0001:
            sampling_hz = LIS2DH12_SAMPLING_1HZ;
            break;
        case 0b0010:
            sampling_hz = LIS2DH12_SAMPLING_10HZ;
            break;
        case 0b0011 ... 0b0111:
            sampling_hz = (enum lis2dh12_sampling)(25 * pow(2, ((reg_ctrl_reg1 >> 4) - 3)));
            break;
        case 0b1000:
            sampling_hz = LIS2DH12_SAMPLING_1620HZ;
            break;
        case 0b1001:
            if (reg_ctrl_reg1 & 0x08) {
                sampling_hz = LIS2DH12_SAMPLING_5376HZ;
            } else {
                sampling_hz = LIS2DH12_SAMPLING_1344HZ;
            }
            break;
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid output data rate!");
            return -EINVAL;
        }
    }

    /* Return success */
    return 0;
}

/**
 * @brief Sets the output data rate (sampling rate)
 *
 * Configures how often the sensor samples acceleration data by setting the ODR
 * bits in CTRL_REG1. If activity detection is configured, it will be automatically
 * adjusted to the new sampling rate.
 *
 * @param[in] smapling_hz Desired sampling rate (LIS2DH12_SAMPLING_100HZ, etc.)
 * @return 0 on success, or a negative error code otherwise
 * @see Datasheet section 8.8 "CTRL_REG1 (20h)"
 */
int lis2dh12::sampling_set(const enum lis2dh12_sampling smapling_hz) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read datarate register!");
        return res;
    }

    /* Modify register */
    switch (smapling_hz) {
        case LIS2DH12_SAMPLING_1HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (1 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_10HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (2 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_25HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (3 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_50HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (4 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_100HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (5 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_200HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (6 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_400HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (7 << 4);
            break;
        }
        case LIS2DH12_SAMPLING_1620HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (8 << 4);
            reg_ctrl_reg1 |= (1 << 3);  // Force LPen low
            break;
        }
        case LIS2DH12_SAMPLING_1344HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (9 << 4);
            reg_ctrl_reg1 |= (1 << 3);  // Force LPen high
            break;
        }
        case LIS2DH12_SAMPLING_5376HZ: {
            reg_ctrl_reg1 &= 0x07;  // Force LPen low
            reg_ctrl_reg1 |= (9 << 4);
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid output data rate!");
            return -EINVAL;
        }
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write odr register!");
        return res;
    }

    /* Update activity duration accordingly */
    if (m_activity_threshold > 0) {
        res = activity_configure(m_activity_threshold, m_activity_duration);
        if (res < 0) {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to re-adjust activity!");
            return res;
        }
    }

    /* Return success */
    return 0;
}

/**
 * @brief Enables or disables individual axes
 *
 * Configures which axes are active for measurement. Disabling unused axes
 * can help reduce power consumption.
 *
 * @param[in] enabled_x Enable (true) or disable (false) X axis
 * @param[in] enabled_y Enable (true) or disable (false) Y axis
 * @param[in] enabled_z Enable (true) or disable (false) Z axis
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::axis_enabled_set(const bool enabled_x, const bool enabled_y, const bool enabled_z) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    reg_ctrl_reg1 &= ~(0b111);
    if (enabled_x) reg_ctrl_reg1 |= (1 << 0);
    if (enabled_y) reg_ctrl_reg1 |= (1 << 1);
    if (enabled_z) reg_ctrl_reg1 |= (1 << 2);
    res = register_write(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Enables or disables a specific axis
 *
 * Configures a single axis to be enabled or disabled for measurement.
 *
 * @param[in] axis Axis to configure (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
 * @param[in] enabled Enable (true) or disable (false) the axis
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::axis_enabled_set(const enum lis2dh12_axis axis, const bool enabled) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    switch (axis) {
        case LIS2DH12_AXIS_X: {
            if (enabled) reg_ctrl_reg1 |= (1 << 0);
            else
                reg_ctrl_reg1 &= ~(1 << 0);
            break;
        }
        case LIS2DH12_AXIS_Y: {
            if (enabled) reg_ctrl_reg1 |= (1 << 1);
            else
                reg_ctrl_reg1 &= ~(1 << 1);
            break;
        }
        case LIS2DH12_AXIS_Z: {
            if (enabled) reg_ctrl_reg1 |= (1 << 2);
            else
                reg_ctrl_reg1 &= ~(1 << 2);
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid axis!");
            return -EINVAL;
        }
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Reads raw acceleration value for a specific axis
 *
 * Reads the high and low byte registers for the specified axis and combines
 * them into a 16-bit signed integer value.
 *
 * @param[in] axis Axis to read (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
 * @param[out] accel Output parameter for the raw acceleration value
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::acceleration_read(const enum lis2dh12_axis axis, int16_t &accel) {
    int res;

    /* Read requested axis */
    uint8_t reg_out_h, reg_out_l;
    res = 0;
    switch (axis) {
        case LIS2DH12_AXIS_X: {
            res |= register_read(LIS2DH12_REGISTER_OUT_X_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_X_L, reg_out_l);
            break;
        }
        case LIS2DH12_AXIS_Y: {
            res |= register_read(LIS2DH12_REGISTER_OUT_Y_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_Y_L, reg_out_l);
            break;
        }
        case LIS2DH12_AXIS_Z: {
            res |= register_read(LIS2DH12_REGISTER_OUT_Z_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_Z_L, reg_out_l);
            break;
        }
    }
    accel = reg_out_h;
    accel <<= 8;
    accel |= reg_out_l;

    /* Return success */
    if (res < 0) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief Reads acceleration value in g-force for a specific axis
 *
 * Reads the raw acceleration value and converts it to g-force units based
 * on the current measurement range setting.
 *
 * @param[in] axis Axis to read (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
 * @param[out] accel Output parameter for the acceleration in g-force units
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::acceleration_read(const enum lis2dh12_axis axis, float &accel) {
    int res;

    /* Retrieve scale */
    enum lis2dh12_range range;
    res = range_get(range);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Compute divider according to range */
    float divider;
    switch (range) {
        case LIS2DH12_RANGE_2G: {
            divider = 15987;
            break;
        }
        case LIS2DH12_RANGE_4G: {
            divider = 7840;
            break;
        }
        case LIS2DH12_RANGE_8G: {
            divider = 3883;
            break;
        }
        case LIS2DH12_RANGE_16G: {
            divider = 1280;
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }

    /* Read requested axis */
    uint8_t reg_out_h, reg_out_l;
    int16_t accel_raw;
    res = 0;
    switch (axis) {
        case LIS2DH12_AXIS_X: {
            res |= register_read(LIS2DH12_REGISTER_OUT_X_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_X_L, reg_out_l);
            break;
        }
        case LIS2DH12_AXIS_Y: {
            res |= register_read(LIS2DH12_REGISTER_OUT_Y_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_Y_L, reg_out_l);
            break;
        }
        case LIS2DH12_AXIS_Z: {
            res |= register_read(LIS2DH12_REGISTER_OUT_Z_H, reg_out_h);
            res |= register_read(LIS2DH12_REGISTER_OUT_Z_L, reg_out_l);
            break;
        }
    }
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel = accel_raw / divider;

    /* Return success */
    if (res < 0) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief Reads raw acceleration values for all three axes
 *
 * Reads the output registers for all three axes and returns the raw 16-bit
 * signed integer values. This is more efficient than reading each axis separately.
 *
 * @param[out] accel_x Output parameter for X-axis raw acceleration value
 * @param[out] accel_y Output parameter for Y-axis raw acceleration value
 * @param[out] accel_z Output parameter for Z-axis raw acceleration value
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::acceleration_read(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z) {
    int res;

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    res = 0;

    /* Read x axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_X_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_X_L, reg_out_l);
    accel_x = reg_out_h;
    accel_x <<= 8;
    accel_x |= reg_out_l;

    /* Read y axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_L, reg_out_l);
    accel_y = reg_out_h;
    accel_y <<= 8;
    accel_y |= reg_out_l;

    /* Read z axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_L, reg_out_l);
    accel_z = reg_out_h;
    accel_z <<= 8;
    accel_z |= reg_out_l;

    /* Return success */
    if (res < 0) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief Reads acceleration values in g-force for all three axes
 *
 * Reads the output registers for all three axes and converts them to g-force
 * units based on the current measurement range setting.
 *
 * @param[out] accel_x Output parameter for X-axis acceleration in g
 * @param[out] accel_y Output parameter for Y-axis acceleration in g
 * @param[out] accel_z Output parameter for Z-axis acceleration in g
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::acceleration_read(float &accel_x, float &accel_y, float &accel_z) {
    int res;

    /* Retrieve scale */
    enum lis2dh12_range range;
    res = range_get(range);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Compute divider according to range */
    float divider;
    switch (range) {
        case LIS2DH12_RANGE_2G: {
            divider = 16000;
            break;
        }
        case LIS2DH12_RANGE_4G: {
            divider = 8000;
            break;
        }
        case LIS2DH12_RANGE_8G: {
            divider = 4000;
            break;
        }
        case LIS2DH12_RANGE_16G: {
            divider = 2000;
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    int16_t accel_raw;
    res = 0;

    /* Read x axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_X_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_X_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_x = accel_raw / divider;

    /* Read y axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_y = accel_raw / divider;

    /* Read z axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_z = accel_raw / divider;

    /* Return success */
    if (res < 0) return -1;
    else
        return 0;
}

/**
 * @brief Reads acceleration values in g-force for all three axes (double precision)
 *
 * Reads the output registers for all three axes and converts them to g-force
 * units with double precision. Use this when you need higher precision calculations.
 *
 * @param[out] accel_x Output parameter for X-axis acceleration in g
 * @param[out] accel_y Output parameter for Y-axis acceleration in g
 * @param[out] accel_z Output parameter for Z-axis acceleration in g
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::acceleration_read(double &accel_x, double &accel_y, double &accel_z) {
    int res;

    /* Retrieve scale */
    enum lis2dh12_range range;
    res = range_get(range);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Compute divider according to range */
    double divider;
    switch (range) {
        case LIS2DH12_RANGE_2G: {
            divider = 15987;
            break;
        }
        case LIS2DH12_RANGE_4G: {
            divider = 7840;
            break;
        }
        case LIS2DH12_RANGE_8G: {
            divider = 3883;
            break;
        }
        case LIS2DH12_RANGE_16G: {
            divider = 1280;
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    int16_t accel_raw;
    res = 0;

    /* Read x axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_X_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_X_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_x = accel_raw / divider;

    /* Read y axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Y_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_y = accel_raw / divider;

    /* Read z axis */
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_H, reg_out_h);
    res |= register_read(LIS2DH12_REGISTER_OUT_Z_L, reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    accel_z = accel_raw / divider;

    /* Return success */
    if (res < 0) return -1;
    else
        return 0;
}

/**
 * @brief Sets the interrupt signal polarity
 *
 * Configures whether interrupt pins (INT1, INT2) are active HIGH or active LOW
 * by setting the IEA bit in CTRL_REG6.
 *
 * @param[in] polarity Desired interrupt polarity
 * @return 0 on success, or a negative error code otherwise
 * @see Application note section 5.1 "Interrupt pin configuration"
 * @see Datasheet section 8.13 "CTRL_REG6 (25h)"
 */
int lis2dh12::interrupts_polarity_set(const enum lis2dh12_interrupt_polarity polarity) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg6;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    if (polarity == LIS2DH12_INTERRUPT_POLARITY_ACTIVE_LOW) {
        reg_ctrl_reg6 |= (1 << 1);
    } else {
        reg_ctrl_reg6 &= ~(1 << 1);
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Configures the activity/inactivity detection feature
 *
 * The sensor can detect when movement exceeds a threshold (activity) or when
 * movement stays below the threshold for a duration (inactivity). This is useful
 * for wake-up applications and power management. The threshold is automatically
 * adjusted based on the current measurement range.
 *
 * @param[in] threshold_mg Threshold in millig above which movements are considered activity
 * @param[in] duration_ms Duration in milliseconds before entering inactivity state
 * @return 0 on success, or a negative error code otherwise
 * @see Application note section 10 "Activity / Inactivity recognition"
 * @see Datasheet section 3.2.4 "Sleep-to-wake and Return-to-sleep"
 * @see Datasheet section 8.33 "ACT_THS (3Eh)"
 * @see Datasheet section 8.34 "ACT_DUR (3Fh)"
 */
int lis2dh12::activity_configure(const uint16_t threshold_mg, const uint32_t duration_ms) {
    int res;

    /* Retrieve scale */
    enum lis2dh12_range range;
    res = range_get(range);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Convert threshold to the closest value depending on scale */
    uint16_t reg_act_ths;
    switch (range) {
        case LIS2DH12_RANGE_2G: {
            reg_act_ths = threshold_mg / 16;
            break;
        }
        case LIS2DH12_RANGE_4G: {
            reg_act_ths = threshold_mg / 32;
            break;
        }
        case LIS2DH12_RANGE_8G: {
            reg_act_ths = threshold_mg / 62;
            break;
        }
        case LIS2DH12_RANGE_16G: {
            reg_act_ths = threshold_mg / 186;
            break;
        }
        default: {
            CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Invalid scale 2!");
            return -EINVAL;
        }
    }

    /* Ensure threshold register would not be out of scale */
    if (reg_act_ths & ~0x7F) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Threshold out of scale!");
        return -E2BIG;
    }

    /* Update threshold register */
    res = register_write(LIS2DH12_REGISTER_ACT_THS, (uint8_t)(0x7F & reg_act_ths));
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write threshold register!");
        return res;
    }

    // /* Debug */
    // Serial1.print(" [d] LIS2DH12_REGISTER_ACT_THS = 0x");
    // Serial1.println((uint8_t)(0x7F & reg_act_ths), HEX);

    /* Retrieve data rate */
    enum lis2dh12_sampling odr_hz;
    res = sampling_get(odr_hz);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve data rate!");
        return res;
    }

    /* Ensure duration register would not be out of bounds */
    const uint32_t duration_ms_max = ((8L * 255 + 1) * 1000) / odr_hz;
    if (duration_ms > duration_ms_max) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Duration out of scale!");
        return -E2BIG;
    }

    /* Update duration register */
    uint8_t reg_inact_dur = (round((duration_ms * odr_hz) / 1000.0) - 1) / 8;
    res = register_write(LIS2DH12_REGISTER_INACT_DUR, reg_inact_dur);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write threshold register!");
        return res;
    }

    // Serial1.print(" [d] LIS2DH12_REGISTER_INACT_DUR = 0x");
    // Serial1.println(reg_inact_dur, HEX);

    // Serial1.print(" [d] Using duration of ");
    // Serial1.print(((8 * reg_inact_dur) + 1) / ((float)odr_hz), 4);
    // Serial1.println("s");

    /* Store original values to be used when adjusting scale and odr */
    m_activity_threshold = threshold_mg;
    m_activity_duration = duration_ms;

    /* Return success */
    return 0;
}

/**
 * @brief Routes activity/inactivity status to INT2 pin
 *
 * When enabled, the INT2 pin will reflect the activity/inactivity status.
 * INT2 is HIGH when inactive and LOW when active. The polarity can be
 * configured using interrupts_polarity_set().
 *
 * @param[in] routed true to route to INT2, false to disable
 * @return 0 on success, or a negative error code otherwise
 * @see Application note section 10 "Activity / Inactivity recognition"
 */
int lis2dh12::activity_int2_routed_set(const bool routed) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg6;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    if (routed) {
        reg_ctrl_reg6 |= (1 << 3);
    } else {
        reg_ctrl_reg6 &= ~(1 << 3);
    }
    res = register_write(LIS2DH12_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Configures the double tap detection feature
 *
 * Sets up the sensor to detect double tap events. The configuration includes
 * threshold, timing parameters, and interrupt latching. For best results,
 * use a sampling rate of 400 Hz or higher in high-resolution mode.
 *
 * @param[in] threshold_mg Threshold in millig for tap detection
 * @param[in] time_limit_ms Maximum time for the tap to occur (in milliseconds)
 * @param[in] time_latency_ms Time between the first and second tap (in milliseconds)
 * @param[in] time_window_ms Maximum time window for the second tap (in milliseconds)
 * @param[in] latch Enable interrupt latching (true) or disable (false)
 * @return 0 on success, or a negative error code otherwise
 * @see doc/STMicroelectronics LIS2DH12 Design note DT0101.pdf for detailed configuration guide
 */
int lis2dh12::doubletap_configure(const uint16_t threshold_mg, const uint32_t time_limit_ms, const uint32_t time_latency_ms, const uint32_t time_window_ms, const bool latch) {
    int res;

    /* Retrieve scale */
    enum lis2dh12_range range;
    res = range_get(range);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Retrieve data rate */
    enum lis2dh12_sampling odr_hz;
    res = sampling_get(odr_hz);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to retrieve data rate!");
        return res;
    }

    /* Ensure input values are within bounds */
    uint32_t threshold_max_mg = 63 * (range * 1000) / 128;
    if (threshold_mg > threshold_max_mg) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Threshold exceeds maximum value!");
        return -E2BIG;
    }
    uint32_t time_limit_max_ms = (127 * odr_hz) * 1000;
    if (time_limit_ms > time_limit_max_ms) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Time limit exceeds maximum value!");
        return -E2BIG;
    }
    uint32_t time_latency_max_ms = (255 * odr_hz) * 1000;
    if (time_latency_ms > time_latency_max_ms) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Time latency exceeds maximum value!");
        return -E2BIG;
    }
    uint32_t time_window_max_ms = (255 * odr_hz) * 1000;
    if (time_window_ms > time_window_max_ms) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Time window exceeds maximum value!");
        return -E2BIG;
    }

    /* Set bit HPCLICK in CTRL_REG2 register (21h) to enable the high-pass filter on tap detection */
    uint8_t reg_ctrl_reg2;
    res = register_read(LIS2DH12_REGISTER_CTRL_REG2, reg_ctrl_reg2);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }
    reg_ctrl_reg2 |= (1 << 2);
    res = register_write(LIS2DH12_REGISTER_CTRL_REG2, reg_ctrl_reg2);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    // Set bit I1_CLICK in CTRL_REG3 register (22h)
    // TEMP

    /* Set bits ZD, YD and XD in CLICK_CFG register (38h) */
    uint8_t reg_click_cfg;
    res = register_read(LIS2DH12_REGISTER_CLICK_CFG, reg_click_cfg);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }
    reg_click_cfg |= (1 << 1);
    reg_click_cfg |= (1 << 3);
    reg_click_cfg |= (1 << 5);
    res = register_write(LIS2DH12_REGISTER_CLICK_CFG, reg_click_cfg);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Set desired threshold to bits THS[6:0] in CLICK_THS register (3Ah) */
    uint8_t reg_th = ((threshold_mg / 1000.0) / range) * 128;
    if (latch) {
        reg_th |= (1 << 7);
    }
    res = register_write(LIS2DH12_REGISTER_CLICK_THS, reg_th);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Set desired time limit to bits TLI[6:0] in TIME_LIMIT register (3Bh) */
    uint8_t reg_tli = (time_limit_ms / 1000.0) * odr_hz;
    res = register_write(LIS2DH12_REGISTER_TIME_LIMIT, reg_tli);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Set desired time latency to bits TLA[7:0] in TIME_LATENCY register (3Ch) */
    uint8_t reg_tla = (time_latency_ms / 1000.0) * odr_hz;
    res = register_write(LIS2DH12_REGISTER_TIME_LATENCY, reg_tla);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Set desired time window to bits TW[7:0] in TIME_WINDOW register (3Dh) */
    uint8_t reg_tw = (time_window_ms / 1000.0) * odr_hz;
    res = register_write(LIS2DH12_REGISTER_TIME_WINDOW, reg_tw);
    if (res < 0) {
        CONFIG_LIS2DH12_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    // Start sensor with ODR 400 Hz, high-resolution mode (recommended) - bits ODR[3:0] in CTRL_REG1 register (20h) and bit HR in CTRL_REG4 register (23h)

    /* Return success */
    return 0;
}

/**
 * @brief Reads a single register from the device
 *
 * Low-level function to directly read a register over I2C or SPI. Most users
 * should use the higher-level functions instead.
 *
 * @param[in] reg_addr Register address to read
 * @param[out] reg_value Output parameter for the register value
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::register_read(const enum lis2dh12_register reg_addr, uint8_t &reg_value) {

    /* Access over i2c
     * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        if (m_i2c_library->endTransmission(false) != 0) return -EIO;
        if (m_i2c_library->requestFrom(m_i2c_address, (uint8_t)1, (uint8_t)true) != 1) return -EIO;
        reg_value = m_i2c_library->read();
    }

    /* Access over spi
     * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr | 0x80);
        reg_value = m_spi_library->transfer(0x00);
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Reads multiple consecutive registers from the device
 *
 * Low-level function to directly read multiple registers. The device supports
 * auto-increment addressing for efficient multi-register reads over I2C or SPI.
 *
 * @param[in] reg_addr Starting register address
 * @param[out] reg_values Array to store the register values
 * @param[in] count Number of registers to read
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::register_read(const enum lis2dh12_register reg_addr, uint8_t reg_values[], const size_t count) {

    /* Access over i2c
     * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        if (m_i2c_library->endTransmission(false) != 0) return -EIO;
        if (m_i2c_library->requestFrom(m_i2c_address, (uint8_t)count, (uint8_t)true) != count) return -EIO;
        for (size_t i = 0; i < count; i++) {
            reg_values[i] = m_i2c_library->read();
        }
    }

    /* Access over spi
     * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr | 0x80 | 0x40);
        for (size_t i = 0; i < count; i++) {
            reg_values[i] = m_spi_library->transfer(0x00);
        }
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}

/**
 * @brief Writes a value to a register
 *
 * Low-level function to directly write a register over I2C or SPI. Most users
 * should use the higher-level functions instead.
 *
 * @param[in] reg_addr Register address to write
 * @param[in] reg_value Value to write to the register
 * @return 0 on success, or a negative error code otherwise
 */
int lis2dh12::register_write(const enum lis2dh12_register reg_addr, const uint8_t reg_value) {

    /* Access over i2c
     * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        m_i2c_library->write(reg_value);
        if (m_i2c_library->endTransmission(true) != 0) return -EIO;
    }

    /* Access over spi
     * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr);
        m_spi_library->transfer(reg_value);
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}
