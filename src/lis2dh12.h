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
 * @brief Measurement range options for the accelerometer
 *
 * The LIS2DH12 supports four different measurement ranges. Higher ranges allow
 * detection of stronger accelerations but with reduced sensitivity. Choose the
 * smallest range that covers your expected acceleration values for best accuracy.
 */
enum lis2dh12_range {
    LIS2DH12_RANGE_2G = 2,    //!< ±2g range - highest sensitivity, best for low-g applications
    LIS2DH12_RANGE_4G = 4,    //!< ±4g range - balanced sensitivity and range
    LIS2DH12_RANGE_8G = 8,    //!< ±8g range - good for moderate acceleration applications
    LIS2DH12_RANGE_16G = 16,  //!< ±16g range - highest range, for high-g applications
};

/**
 * @brief Output data resolution options
 *
 * The LIS2DH12 can operate in different resolution modes. Higher resolution
 * provides better accuracy but may consume more power. 12-bit resolution is
 * recommended for most applications.
 */
enum lis2dh12_resolution {
    LIS2DH12_RESOLUTION_8BITS,   //!< 8-bit resolution - lowest power consumption
    LIS2DH12_RESOLUTION_10BITS,  //!< 10-bit resolution - balanced accuracy and power
    LIS2DH12_RESOLUTION_12BITS,  //!< 12-bit resolution - highest accuracy
};

/**
 * @brief Output data rate (ODR) / sampling rate options
 *
 * The LIS2DH12 can sample acceleration data at various rates from 1 Hz to 5.376 kHz.
 * Higher sampling rates provide more data points but consume more power. Choose a
 * rate appropriate for your application - 100 Hz is suitable for most general use.
 */
enum lis2dh12_sampling {
    LIS2DH12_SAMPLING_NONE = 0,       //!< Power-down mode - sensor disabled
    LIS2DH12_SAMPLING_1HZ = 1,        //!< 1 Hz - lowest power, for very slow motion
    LIS2DH12_SAMPLING_10HZ = 10,      //!< 10 Hz - low power, for slow motion detection
    LIS2DH12_SAMPLING_25HZ = 25,      //!< 25 Hz - suitable for basic motion detection
    LIS2DH12_SAMPLING_50HZ = 50,      //!< 50 Hz - good for general motion sensing
    LIS2DH12_SAMPLING_100HZ = 100,    //!< 100 Hz - recommended for most applications
    LIS2DH12_SAMPLING_200HZ = 200,    //!< 200 Hz - for faster motion detection
    LIS2DH12_SAMPLING_400HZ = 400,    //!< 400 Hz - high-speed sampling
    LIS2DH12_SAMPLING_1344HZ = 1344,  //!< 1.344 kHz - very high-speed sampling
    LIS2DH12_SAMPLING_1620HZ = 1620,  //!< 1.62 kHz - very high-speed sampling
    LIS2DH12_SAMPLING_5376HZ = 5376,  //!< 5.376 kHz - maximum sampling rate
};

/**
 * @brief Accelerometer measurement axes
 *
 * The LIS2DH12 is a three-axis accelerometer that can measure acceleration
 * along the X, Y, and Z axes independently or simultaneously.
 */
enum lis2dh12_axis {
    LIS2DH12_AXIS_X,  //!< X-axis - typically horizontal, left-right
    LIS2DH12_AXIS_Y,  //!< Y-axis - typically horizontal, forward-backward
    LIS2DH12_AXIS_Z,  //!< Z-axis - typically vertical, up-down
};

/**
 * @brief Interrupt signal polarity options
 *
 * Controls whether interrupt pins (INT1, INT2) are active HIGH or active LOW.
 * This allows compatibility with different microcontroller interrupt configurations.
 */
enum lis2dh12_interrupt_polarity {
    LIS2DH12_INTERRUPT_POLARITY_ACTIVE_HIGH,  //!< Interrupt pins are active when HIGH
    LIS2DH12_INTERRUPT_POLARITY_ACTIVE_LOW,   //!< Interrupt pins are active when LOW
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
 * @brief LIS2DH12 accelerometer driver class
 *
 * This class provides an interface to the STMicroelectronics LIS2DH12 ultra-low-power
 * high-performance three-axis linear accelerometer. The device can communicate over
 * I2C or SPI and provides functionality to:
 * - Measure acceleration along X, Y, and Z axes
 * - Configure measurement range, resolution, and sampling rate
 * - Detect activity/inactivity events
 * - Detect double tap events
 * - Generate interrupts for various events
 *
 * @note Call setup() before using any other functions
 * @note Make sure to initialize the I2C or SPI library (Wire.begin() or SPI.begin())
 *       before calling setup()
 */
class lis2dh12 {

   public:
    //!@{
    //! Initialization and setup
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
    int setup(TwoWire &i2c_library, uint8_t i2c_address = 0x18);

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
    int setup(SPIClass &spi_library, const int spi_pin_cs, const int spi_speed = 10000000);

    /**
     * @brief Detects if the LIS2DH12 device is present
     *
     * Reads the device ID register and verifies it matches the expected value (0x33).
     *
     * @return true if device is detected, false otherwise
     */
    bool detect(void);
    //!@}

    //!@{
    //! Configuration
    /**
     * @brief Gets the current measurement range
     *
     * @param[out] range_g Output parameter for the current range setting
     * @return 0 on success, or a negative error code otherwise
     */
    int range_get(enum lis2dh12_range &range_g);

    /**
     * @brief Sets the measurement range
     *
     * Configures the full-scale range of the accelerometer. Higher ranges allow
     * detection of stronger accelerations but with reduced sensitivity.
     *
     * @param[in] range_g Desired range (LIS2DH12_RANGE_2G, LIS2DH12_RANGE_4G, etc.)
     * @return 0 on success, or a negative error code otherwise
     */
    int range_set(const enum lis2dh12_range range_g);

    /**
     * @brief Gets the current resolution setting
     *
     * @param[out] resolution Output parameter for the current resolution setting
     * @return 0 on success, or a negative error code otherwise
     */
    int resolution_get(enum lis2dh12_resolution &resolution);

    /**
     * @brief Sets the output data resolution
     *
     * Configures the resolution mode. Higher resolution provides better accuracy
     * but may consume more power. 12-bit resolution is recommended for most applications.
     *
     * @param[in] resolution Desired resolution (LIS2DH12_RESOLUTION_8BITS, etc.)
     * @return 0 on success, or a negative error code otherwise
     */
    int resolution_set(const enum lis2dh12_resolution resolution);

    /**
     * @brief Gets the current sampling rate
     *
     * @param[out] sampling_hz Output parameter for the current sampling rate
     * @return 0 on success, or a negative error code otherwise
     */
    int sampling_get(enum lis2dh12_sampling &sampling_hz);

    /**
     * @brief Sets the output data rate (sampling rate)
     *
     * Configures how often the sensor samples acceleration data. Higher rates
     * provide more data points but consume more power.
     *
     * @param[in] sampling_hz Desired sampling rate (LIS2DH12_SAMPLING_100HZ, etc.)
     * @return 0 on success, or a negative error code otherwise
     */
    int sampling_set(const enum lis2dh12_sampling sampling_hz);

    /**
     * @brief Enables or disables individual axes
     *
     * Allows you to enable or disable each axis independently. Disabling unused
     * axes can save power.
     *
     * @param[in] enabled_x Enable (true) or disable (false) X axis
     * @param[in] enabled_y Enable (true) or disable (false) Y axis
     * @param[in] enabled_z Enable (true) or disable (false) Z axis
     * @return 0 on success, or a negative error code otherwise
     */
    int axis_enabled_set(const bool enabled_x, const bool enabled_y, const bool enabled_z);

    /**
     * @brief Enables or disables a specific axis
     *
     * @param[in] axis Axis to configure (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
     * @param[in] enabled Enable (true) or disable (false) the axis
     * @return 0 on success, or a negative error code otherwise
     */
    int axis_enabled_set(const enum lis2dh12_axis axis, const bool enabled);

    /**
     * @brief Sets the interrupt signal polarity
     *
     * Configures whether interrupt pins are active HIGH or active LOW.
     *
     * @param[in] polarity Desired interrupt polarity
     * @return 0 on success, or a negative error code otherwise
     */
    int interrupts_polarity_set(const enum lis2dh12_interrupt_polarity polarity);
    //!@}

    //!@{
    //! Reading acceleration data
    /**
     * @brief Reads raw acceleration value for a specific axis
     *
     * @param[in] axis Axis to read (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
     * @param[out] accel Output parameter for the raw acceleration value
     * @return 0 on success, or a negative error code otherwise
     */
    int acceleration_read(const enum lis2dh12_axis axis, int16_t &accel);

    /**
     * @brief Reads acceleration value in g-force for a specific axis
     *
     * @param[in] axis Axis to read (LIS2DH12_AXIS_X, LIS2DH12_AXIS_Y, or LIS2DH12_AXIS_Z)
     * @param[out] accel Output parameter for the acceleration in g-force units
     * @return 0 on success, or a negative error code otherwise
     */
    int acceleration_read(const enum lis2dh12_axis axis, float &accel);

    /**
     * @brief Reads raw acceleration values for all three axes
     *
     * @param[out] accel_x Output parameter for X-axis raw acceleration value
     * @param[out] accel_y Output parameter for Y-axis raw acceleration value
     * @param[out] accel_z Output parameter for Z-axis raw acceleration value
     * @return 0 on success, or a negative error code otherwise
     */
    int acceleration_read(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z);

    /**
     * @brief Reads acceleration values in g-force for all three axes
     *
     * @param[out] accel_x Output parameter for X-axis acceleration in g
     * @param[out] accel_y Output parameter for Y-axis acceleration in g
     * @param[out] accel_z Output parameter for Z-axis acceleration in g
     * @return 0 on success, or a negative error code otherwise
     */
    int acceleration_read(float &accel_x, float &accel_y, float &accel_z);

    /**
     * @brief Reads acceleration values in g-force for all three axes (double precision)
     *
     * @param[out] accel_x Output parameter for X-axis acceleration in g
     * @param[out] accel_y Output parameter for Y-axis acceleration in g
     * @param[out] accel_z Output parameter for Z-axis acceleration in g
     * @return 0 on success, or a negative error code otherwise
     */
    int acceleration_read(double &accel_x, double &accel_y, double &accel_z);
    //!@}

    //!@{
    //! Activity detection
    /**
     * @brief Configures the activity/inactivity detection feature
     *
     * The sensor can detect when movement exceeds a threshold (activity) or when
     * movement stays below the threshold for a duration (inactivity). This is useful
     * for wake-up applications and power management.
     *
     * @param[in] threshold_mg Threshold in millig above which movements are considered activity
     * @param[in] duration_ms Duration in milliseconds before entering inactivity state
     * @return 0 on success, or a negative error code otherwise
     * @see activity_int2_routed_set() to route the status to an interrupt pin
     */
    int activity_configure(const uint16_t threshold_mg, const uint32_t duration_ms);

    /**
     * @brief Routes activity/inactivity status to INT2 pin
     *
     * When enabled, the INT2 pin will reflect the activity/inactivity status.
     * INT2 is HIGH when inactive and LOW when active (polarity can be configured).
     *
     * @param[in] routed true to route to INT2, false to disable
     * @return 0 on success, or a negative error code otherwise
     */
    int activity_int2_routed_set(const bool routed);
    //!@}

    //!@{
    //! Double tap detection
    /**
     * @brief Configures the double tap detection feature
     *
     * Enables detection of double tap events on the sensor. Useful for user
     * interface applications where tapping the device can trigger actions.
     *
     * @param[in] threshold_mg Threshold in millig for tap detection
     * @param[in] time_limit_ms Maximum time for the tap to occur (in milliseconds)
     * @param[in] time_latency_ms Time between the first and second tap (in milliseconds)
     * @param[in] time_window_ms Maximum time window for the second tap (in milliseconds)
     * @param[in] latch Enable interrupt latching (true) or disable (false)
     * @return 0 on success, or a negative error code otherwise
     * @see doc/STMicroelectronics LIS2DH12 Design note DT0101.pdf for detailed configuration
     */
    int doubletap_configure(const uint16_t threshold_mg, const uint32_t time_limit_ms, const uint32_t time_latency_ms, const uint32_t time_window_ms, const bool latch);
    //!@}

    //!@{
    //! Register access
    /**
     * @brief Reads a single register from the device
     *
     * Low-level function to directly read a register. Most users should use
     * the higher-level functions instead.
     *
     * @param[in] reg_addr Register address to read
     * @param[out] reg_value Output parameter for the register value
     * @return 0 on success, or a negative error code otherwise
     */
    int register_read(const enum lis2dh12_register reg_addr, uint8_t &reg_value);

    /**
     * @brief Reads multiple consecutive registers from the device
     *
     * Low-level function to directly read multiple registers. The device supports
     * auto-increment addressing for efficient multi-register reads.
     *
     * @param[in] reg_addr Starting register address
     * @param[out] reg_values Array to store the register values
     * @param[in] count Number of registers to read
     * @return 0 on success, or a negative error code otherwise
     */
    int register_read(const enum lis2dh12_register reg_addr, uint8_t reg_values[], const size_t count);

    /**
     * @brief Writes a value to a register
     *
     * Low-level function to directly write a register. Most users should use
     * the higher-level functions instead.
     *
     * @param[in] reg_addr Register address to write
     * @param[in] reg_value Value to write to the register
     * @return 0 on success, or a negative error code otherwise
     */
    int register_write(const enum lis2dh12_register reg_addr, const uint8_t reg_value);
    //!@}

   protected:
    TwoWire *m_i2c_library = NULL;      //!< Pointer to I2C library instance
    uint8_t m_i2c_address = 0;          //!< Device I2C address
    SPIClass *m_spi_library = NULL;     //!< Pointer to SPI library instance
    int m_spi_pin_cs = 0;               //!< Chip select pin for SPI communication
    int m_spi_speed;                    //!< SPI clock speed in Hz
    uint16_t m_activity_threshold = 0;  //!< Stored activity threshold for range/ODR adjustments
    uint32_t m_activity_duration = 0;   //!< Stored activity duration for range/ODR adjustments
};

#endif
