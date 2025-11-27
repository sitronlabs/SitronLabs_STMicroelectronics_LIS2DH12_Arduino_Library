[![Designed by Sitron Labs](https://img.shields.io/badge/Designed_by-Sitron_Labs-FCE477.svg)](https://www.sitronlabs.com/)
[![Join the Discord community](https://img.shields.io/discord/552242187665145866.svg?logo=discord&logoColor=white&label=Discord&color=%237289da)](https://discord.gg/btnVDeWhfW)
[![PayPal Donate](https://img.shields.io/badge/PayPal-Donate-00457C.svg?logo=paypal&logoColor=white)](https://www.paypal.com/donate/?hosted_button_id=QLX8VU9Q3PFFL)
![License](https://img.shields.io/github/license/sitronlabs/SitronLabs_STMicroelectronics_LIS2DH12_Arduino_Library.svg)
![Latest Release](https://img.shields.io/github/release/sitronlabs/SitronLabs_STMicroelectronics_LIS2DH12_Arduino_Library.svg)
[![Arduino Library Manager](https://www.ardu-badge.com/badge/Sitron%20Labs%20LIS2DH12%20Arduino%20Library.svg)](https://www.ardu-badge.com/Sitron%20Labs%20LIS2DH12%20Arduino%20Library)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/sitronlabs/library/Sitron_Labs_LIS2DH12_Arduino_Library.svg)](https://registry.platformio.org/libraries/sitronlabs/Sitron_Labs_LIS2DH12_Arduino_Library)

# Sitron Labs STMicroelectronics LIS2DH12 Arduino Library

Arduino library for interfacing with the STMicroelectronics LIS2DH12 ultra-low-power high-performance three-axis linear accelerometer.

## Description

The LIS2DH12 is an ultra-low-power high-performance three-axis linear accelerometer belonging to the femto family with digital I2C/SPI serial interface standard output. The LIS2DH12 has user-selectable full scales of ±2g/±4g/±8g/±16g and is capable of measuring accelerations with output data rates from 1 Hz to 5.3 kHz. The device may be configured to generate interrupt signals by detecting two independent inertial wake-up/free-fall events as well as by the position of the device itself. This library provides a simple interface to access these features via I2C or SPI.

## Installation

### Arduino IDE

Install via the Arduino Library Manager by searching for "Sitron Labs LIS2DH12".

Alternatively, install manually:
1. Download or clone this repository
2. Place it in your Arduino `libraries` folder
3. Restart the Arduino IDE

### PlatformIO

Install via the PlatformIO Library Manager by searching for "Sitron Labs LIS2DH12".

Alternatively, add the library manually to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/sitronlabs/SitronLabs_STMicroelectronics_LIS2DH12_Arduino_Library.git
```

## Hardware Connections

### I2C Interface

Connect the LIS2DH12 to your Arduino using I2C:

- VCC → 3.3V (check your board's specifications)
- GND → GND
- SDA → SDA (I2C Data)
- SCL → SCL (I2C Clock)
- SA0 → GND or VCC (I2C address selection)

The I2C address is determined by the SA0 pin:
- SA0 LOW → 0x18 (default)
- SA0 HIGH → 0x19

### SPI Interface

Connect the LIS2DH12 to your Arduino using SPI:

- VCC → 3.3V (check your board's specifications)
- GND → GND
- CS → Any digital pin (Chip Select)
- SCL/SCLK → SCK (SPI Clock)
- SDA/SDI → MOSI (SPI Master Out Slave In)
- SDO → MISO (SPI Master In Slave Out)

## Usage

### Basic I2C Example

```cpp
#include <Wire.h>
#include <lis2dh12.h>

// Create device object
lis2dh12 sensor;

// I2C address (0x18 when SA0 is LOW, 0x19 when SA0 is HIGH)
const uint8_t I2C_ADDRESS = 0x18;

void setup() {
  Serial.begin(9600);
  
  // Initialize I2C
  Wire.begin();
  
  // Setup the LIS2DH12 (I2C library, I2C address)
  if (sensor.setup(Wire, I2C_ADDRESS) != 0) {
    Serial.println("Failed to setup LIS2DH12");
    return;
  }
  
  // Detect the device
  if (!sensor.detect()) {
    Serial.println("LIS2DH12 not detected");
    return;
  }
  
  // Configure sensor settings
  sensor.range_set(LIS2DH12_RANGE_2G);
  sensor.resolution_set(LIS2DH12_RESOLUTION_12BITS);
  sensor.sampling_set(LIS2DH12_SAMPLING_100HZ);
  sensor.axis_enabled_set(true, true, true);  // Enable X, Y, Z axes
  
  Serial.println("LIS2DH12 initialized");
}

void loop() {
  // Read acceleration values
  float accel_x, accel_y, accel_z;
  if (sensor.acceleration_read(accel_x, accel_y, accel_z) == 0) {
    Serial.print("X: ");
    Serial.print(accel_x);
    Serial.print(" g, Y: ");
    Serial.print(accel_y);
    Serial.print(" g, Z: ");
    Serial.print(accel_z);
    Serial.println(" g");
  }
  
  delay(100);
}
```

### Basic SPI Example

```cpp
#include <SPI.h>
#include <lis2dh12.h>

// Create device object
lis2dh12 sensor;

// SPI chip select pin
const int PIN_CS = 10;

void setup() {
  Serial.begin(9600);
  
  // Initialize SPI
  SPI.begin();
  
  // Setup the LIS2DH12 (SPI library, CS pin, SPI speed)
  if (sensor.setup(SPI, PIN_CS, 10000000) != 0) {
    Serial.println("Failed to setup LIS2DH12");
    return;
  }
  
  // Detect the device
  if (!sensor.detect()) {
    Serial.println("LIS2DH12 not detected");
    return;
  }
  
  // Configure sensor settings
  sensor.range_set(LIS2DH12_RANGE_4G);
  sensor.resolution_set(LIS2DH12_RESOLUTION_12BITS);
  sensor.sampling_set(LIS2DH12_SAMPLING_200HZ);
  sensor.axis_enabled_set(true, true, true);
  
  Serial.println("LIS2DH12 initialized");
}

void loop() {
  // Read acceleration values
  int16_t accel_x, accel_y, accel_z;
  if (sensor.acceleration_read(accel_x, accel_y, accel_z) == 0) {
    Serial.print("X: ");
    Serial.print(accel_x);
    Serial.print(", Y: ");
    Serial.print(accel_y);
    Serial.print(", Z: ");
    Serial.println(accel_z);
  }
  
  delay(50);
}
```

### Activity Detection Example

```cpp
#include <Wire.h>
#include <lis2dh12.h>

lis2dh12 sensor;
const uint8_t I2C_ADDRESS = 0x18;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  sensor.setup(Wire, I2C_ADDRESS);
  if (!sensor.detect()) {
    Serial.println("LIS2DH12 not detected");
    return;
  }
  
  // Configure sensor
  sensor.range_set(LIS2DH12_RANGE_2G);
  sensor.sampling_set(LIS2DH12_SAMPLING_100HZ);
  sensor.axis_enabled_set(true, true, true);
  
  // Configure activity detection
  // Threshold: 500 mg, Duration: 2 seconds
  sensor.activity_configure(500, 2000);
  
  // Route activity status to INT2 pin
  sensor.activity_int2_routed_set(true);
  
  Serial.println("Activity detection configured");
}

void loop() {
  // Your code here - check INT2 pin for activity status
  delay(100);
}
```

## API Reference

### setup(TwoWire &i2c_library, uint8_t i2c_address)

Initializes the LIS2DH12 device for I2C communication.

- `i2c_library`: I2C library instance to use (typically `Wire`)
- `i2c_address`: I2C address of the device (0x18 when SA0 is LOW, 0x19 when SA0 is HIGH)

Returns 0 on success, or a negative error code otherwise.

### setup(SPIClass &spi_library, const int spi_pin_cs, const int spi_speed)

Initializes the LIS2DH12 device for SPI communication.

- `spi_library`: SPI library instance to use (typically `SPI`)
- `spi_pin_cs`: GPIO pin number connected to the device's chip select pin
- `spi_speed`: SPI clock speed in Hz (default: 10000000)

Returns 0 on success, or a negative error code otherwise.

### detect(void)

Detects if the LIS2DH12 device is present by reading the device ID register.

Returns true if device is detected, false otherwise.

### range_set(enum lis2dh12_range range_g)

Sets the measurement range of the accelerometer.

- `range_g`: Desired range (LIS2DH12_RANGE_2G, LIS2DH12_RANGE_4G, LIS2DH12_RANGE_8G, or LIS2DH12_RANGE_16G)

Returns 0 on success, or a negative error code otherwise.

### range_get(enum lis2dh12_range &range_g)

Gets the current measurement range of the accelerometer.

- `range_g`: Output parameter for the current range

Returns 0 on success, or a negative error code otherwise.

### resolution_set(enum lis2dh12_resolution resolution)

Sets the resolution of the accelerometer.

- `resolution`: Desired resolution (LIS2DH12_RESOLUTION_8BITS, LIS2DH12_RESOLUTION_10BITS, or LIS2DH12_RESOLUTION_12BITS)

Returns 0 on success, or a negative error code otherwise.

### sampling_set(enum lis2dh12_sampling sampling_hz)

Sets the output data rate (sampling rate) of the accelerometer.

- `sampling_hz`: Desired sampling rate (LIS2DH12_SAMPLING_1HZ, LIS2DH12_SAMPLING_10HZ, LIS2DH12_SAMPLING_25HZ, LIS2DH12_SAMPLING_50HZ, LIS2DH12_SAMPLING_100HZ, LIS2DH12_SAMPLING_200HZ, LIS2DH12_SAMPLING_400HZ, LIS2DH12_SAMPLING_1344HZ, LIS2DH12_SAMPLING_1620HZ, or LIS2DH12_SAMPLING_5376HZ)

Returns 0 on success, or a negative error code otherwise.

### axis_enabled_set(bool enabled_x, bool enabled_y, bool enabled_z)

Enables or disables individual axes.

- `enabled_x`: Enable X axis (true) or disable (false)
- `enabled_y`: Enable Y axis (true) or disable (false)
- `enabled_z`: Enable Z axis (true) or disable (false)

Returns 0 on success, or a negative error code otherwise.

### acceleration_read(float &accel_x, float &accel_y, float &accel_z)

Reads acceleration values for all three axes in g-force units.

- `accel_x`: Output parameter for X-axis acceleration in g
- `accel_y`: Output parameter for Y-axis acceleration in g
- `accel_z`: Output parameter for Z-axis acceleration in g

Returns 0 on success, or a negative error code otherwise.

### acceleration_read(int16_t &accel_x, int16_t &accel_y, int16_t &accel_z)

Reads raw acceleration values for all three axes.

- `accel_x`: Output parameter for X-axis raw acceleration value
- `accel_y`: Output parameter for Y-axis raw acceleration value
- `accel_z`: Output parameter for Z-axis raw acceleration value

Returns 0 on success, or a negative error code otherwise.

### activity_configure(uint16_t threshold_mg, uint32_t duration_ms)

Configures the activity/inactivity detection feature.

- `threshold_mg`: Threshold in millig above which movements will be considered as activity
- `duration_ms`: Duration in milliseconds before entering inactivity state when movement is below threshold

Returns 0 on success, or a negative error code otherwise.

### activity_int2_routed_set(bool routed)

Routes the activity/inactivity status to the INT2 pin.

- `routed`: true to route to INT2, false to disable

Returns 0 on success, or a negative error code otherwise.

## Specifications

- Communication interfaces: I2C or SPI
- I2C addresses: 0x18 (SA0 LOW) or 0x19 (SA0 HIGH)
- Measurement ranges: ±2g, ±4g, ±8g, ±16g
- Resolutions: 8-bit, 10-bit, or 12-bit
- Output data rates: 1 Hz to 5.3 kHz
- Features: Activity detection, double tap detection, interrupt generation
