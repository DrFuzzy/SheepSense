# SheepSense

SheepSense is an ESP32-C3-based data logger for recording sheep motion and orientation data using an ICM-20948 9-axis IMU. Sensor measurements are stored on a microSD card in CSV format, with optional timestamps from a DS3231 real-time clock.

## Source Files

### `sheep_sense.cpp`

Main SheepSense logger application.

It:

* Reads accelerometer, gyroscope, magnetometer, and temperature data from the ICM-20948.
* Samples the IMU at approximately 100 Hz.
* Stores measurements in `/log.csv` on the microSD card.
* Adds descriptive column labels to the CSV file.
* Uses buffered SD-card writes to reduce timing delays.
* Supports optional DS3231 date and time information.
* Can print live IMU readings and logger statistics to the Serial Monitor.

Serial output can be controlled with:

```cpp
constexpr bool DEBUG_PRINT_IMU = true;
constexpr bool DEBUG_PRINT_LOGGER_STATUS = true;
```

## Helper Sources

### `i2c_scanner.cpp`

Scans the I²C bus and reports detected device addresses.

Use it to verify the ICM-20948 and DS3231 connections. The expected addresses are:

```text
0x68 — DS3231 RTC
0x69 — ICM-20948
```

### `sd_card_test.cpp`

Tests communication with the microSD card over SPI.

It:

* Initialises the SD card.
* Displays card and filesystem information.
* Creates a test file.
* Writes test data.
* Reads the file back through the Serial Monitor.

## Pin Connections

### ICM-20948 and DS3231

| Signal | XIAO ESP32-C3 |
| ------ | ------------- |
| SDA    | D4            |
| SCL    | D5            |
| VCC    | 3.3 V         |
| GND    | GND           |

### Adafruit microSD Breakout

| Breakout pin | XIAO ESP32-C3 |
| ------------ | ------------- |
| SI / MOSI    | D0            |
| SO / MISO    | D1            |
| CLK / SCK    | D2            |
| CS           | D3            |
| 3V           | 3.3 V         |
| GND          | GND           |

## Helper Program Usage

Only one source file containing `setup()` and `loop()` should be compiled at a time.

To run a helper program, temporarily use it instead of the main application source in the PlatformIO `src/` directory.
