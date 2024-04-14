#ifndef LIS2DH12_H
#define LIS2DH12_H

/* Arduino libraries */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/* C/C++ libraries */
#include <errno.h>
#include <stdint.h>

/**
 * List of available sensor ranges.
 */
enum lis2dh12_range {
    LIS2DH12_RANGE_2G = 2,
    LIS2DH12_RANGE_4G = 4,
    LIS2DH12_RANGE_8G = 8,
    LIS2DH12_RANGE_16G = 16,
};

/**
 * List of available resolutions.
 */
enum lis2dh12_resolution {
    LIS2DH12_RESOLUTION_8BITS,
    LIS2DH12_RESOLUTION_10BITS,
    LIS2DH12_RESOLUTION_12BITS,
};

/**
 * List of available sampling rates.
 */
enum lis2dh12_sampling {
    LIS2DH12_SAMPLING_NONE = 0,
    LIS2DH12_SAMPLING_1HZ = 1,
    LIS2DH12_SAMPLING_10HZ = 10,
    LIS2DH12_SAMPLING_25HZ = 25,
    LIS2DH12_SAMPLING_50HZ = 50,
    LIS2DH12_SAMPLING_100HZ = 100,
    LIS2DH12_SAMPLING_200HZ = 200,
    LIS2DH12_SAMPLING_400HZ = 400,
    LIS2DH12_SAMPLING_1344HZ = 1344,
    LIS2DH12_SAMPLING_1620HZ = 1620,
    LIS2DH12_SAMPLING_5376HZ = 5376,
};

/**
 * List of available axis.
 */
enum lis2dh12_axis {
    LIS2DH12_AXIS_X,
    LIS2DH12_AXIS_Y,
    LIS2DH12_AXIS_Z,
};

/**
 * List of possible interrupt polarities.
 */
enum lis2dh12_interrupt_polarity {
    LIS2DH12_INTERRUPT_POLARITY_ACTIVE_HIGH,
    LIS2DH12_INTERRUPT_POLARITY_ACTIVE_LOW,
};

/**
 * List of internal registers.
 */
enum lis2dh12_register {
    LIS2DH12_REGISTER_STATUS_REG_AUX = 0x07,  //!<
    LIS2DH12_REGISTER_OUT_TEMP_L = 0x0C,      //!<
    LIS2DH12_REGISTER_OUT_TEMP_H = 0x0D,      //!<
    LIS2DH12_REGISTER_WHO_AM_I = 0x0F,        //!<
    LIS2DH12_REGISTER_CTRL_REG0 = 0x1E,       //!<
    LIS2DH12_REGISTER_TEMP_CFG_REG = 0x1F,    //!<
    LIS2DH12_REGISTER_CTRL_REG1 = 0x20,       //!<
    LIS2DH12_REGISTER_CTRL_REG2 = 0x21,       //!<
    LIS2DH12_REGISTER_CTRL_REG3 = 0x22,       //!<
    LIS2DH12_REGISTER_CTRL_REG4 = 0x23,       //!<
    LIS2DH12_REGISTER_CTRL_REG5 = 0x24,       //!<
    LIS2DH12_REGISTER_CTRL_REG6 = 0x25,       //!<
    LIS2DH12_REGISTER_REFERENCE = 0x26,       //!<
    LIS2DH12_REGISTER_STATUS_REG = 0x27,      //!<
    LIS2DH12_REGISTER_OUT_X_L = 0x28,         //!<
    LIS2DH12_REGISTER_OUT_X_H = 0x29,         //!<
    LIS2DH12_REGISTER_OUT_Y_L = 0x2A,         //!<
    LIS2DH12_REGISTER_OUT_Y_H = 0x2B,         //!<
    LIS2DH12_REGISTER_OUT_Z_L = 0x2C,         //!<
    LIS2DH12_REGISTER_OUT_Z_H = 0x2D,         //!<
    LIS2DH12_REGISTER_FIFO_CTRL_REG = 0x2E,   //!<
    LIS2DH12_REGISTER_FIFO_SRC_REG = 0x2F,    //!<
    LIS2DH12_REGISTER_INT1_CFG = 0x30,        //!<
    LIS2DH12_REGISTER_INT1_SRC = 0x31,        //!<
    LIS2DH12_REGISTER_INT1_THS = 0x32,        //!<
    LIS2DH12_REGISTER_INT1_DURATION = 0x33,   //!<
    LIS2DH12_REGISTER_INT2_CFG = 0x34,        //!<
    LIS2DH12_REGISTER_INT2_SRC = 0x35,        //!<
    LIS2DH12_REGISTER_INT2_THS = 0x36,        //!<
    LIS2DH12_REGISTER_INT2_DURATION = 0x37,   //!<
    LIS2DH12_REGISTER_CLICK_CFG = 0x38,       //!<
    LIS2DH12_REGISTER_CLICK_SRC = 0x39,       //!<
    LIS2DH12_REGISTER_CLICK_THS = 0x3A,       //!<
    LIS2DH12_REGISTER_TIME_LIMIT = 0x3B,      //!<
    LIS2DH12_REGISTER_TIME_LATENCY = 0x3C,    //!<
    LIS2DH12_REGISTER_TIME_WINDOW = 0x3D,     //!<
    LIS2DH12_REGISTER_ACT_THS = 0x3E,         //!<
    LIS2DH12_REGISTER_INACT_DUR = 0x3F,       //!<
};

/**
 *
 */
class lis2dh12 {

   public:
    /* Setup */
    int setup(TwoWire &i2c_library, uint8_t i2c_address = 0x18);
    int setup(SPIClass &spi_library, const int spi_pin_cs, const int spi_speed = 10000000);
    bool detect(void);
    int range_get(enum lis2dh12_range &range_g);
    int range_set(const enum lis2dh12_range range_g);
    int resolution_get(enum lis2dh12_resolution &resolution);
    int resolution_set(const enum lis2dh12_resolution resolution);
    int sampling_get(enum lis2dh12_sampling &sampling_hz);
    int sampling_set(const enum lis2dh12_sampling sampling_hz);
    int axis_enabled_set(const bool enabled_x, const bool enabled_y, const bool enabled_z);
    int axis_enabled_set(const enum lis2dh12_axis axis, const bool enabled);
    int interrupts_polarity_set(const enum lis2dh12_interrupt_polarity polarity);

    /* Reading acceleration */
    int acceleration_read(const enum lis2dh12_axis axis, int16_t &accel);
    int acceleration_read(const enum lis2dh12_axis axis, float &accel);
    int acceleration_read(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z);
    int acceleration_read(float &accel_x, float &accel_y, float &accel_z);
    int acceleration_read(double &accel_x, double &accel_y, double &accel_z);

    /* Activity detection */
    int activity_configure(const uint16_t threshold_mg, const uint32_t duration_ms);
    int activity_int2_routed_set(const bool routed);

    /* Double tap detection
     * @see doc/STMicroelectronics LIS2DH12 Design note DT0101.pdf */
    int doubletap_configure(const uint16_t threshold_mg, const uint32_t time_limit_ms, const uint32_t time_latency_ms, const uint32_t time_window_ms, const bool latch);

    /* Register access */
    int register_read(const enum lis2dh12_register reg_addr, uint8_t &reg_value);
    int register_read(const enum lis2dh12_register reg_addr, uint8_t reg_values[], const size_t count);
    int register_write(const enum lis2dh12_register reg_addr, const uint8_t reg_value);

   protected:
    TwoWire *m_i2c_library = NULL;
    uint8_t m_i2c_address = 0;
    SPIClass *m_spi_library = NULL;
    int m_spi_pin_cs = 0;
    int m_spi_speed;
    uint16_t m_activity_threshold = 0;
    uint32_t m_activity_duration = 0;
};

#endif
