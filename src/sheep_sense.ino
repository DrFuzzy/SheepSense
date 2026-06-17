#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <ICM_20948.h>

// -----------------------------------------------------------------------------
// Pin configuration
// -----------------------------------------------------------------------------

// Adafruit microSD SPI breakout:
//
// SI  -> XIAO D0 / MOSI
// SO  -> XIAO D1 / MISO
// CLK -> XIAO D2 / SCK
// CS  -> XIAO D3

constexpr uint8_t SD_MOSI = D0;
constexpr uint8_t SD_MISO = D1;
constexpr uint8_t SD_SCK  = D2;
constexpr uint8_t SD_CS   = D3;

// I2C:
//
// SDA -> XIAO D4
// SCL -> XIAO D5

constexpr uint8_t I2C_SDA = D4;
constexpr uint8_t I2C_SCL = D5;

// The ADDR+1 trace was cut, so the ICM-20948 uses address 0x69.
constexpr bool IMU_AD0_VALUE = true;

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000;

constexpr char LOG_FILE_PATH[] = "/log.csv";

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------

RTC_DS3231 rtc;
ICM_20948_I2C imu;
File log_file;

// -----------------------------------------------------------------------------
// Hardware state
// -----------------------------------------------------------------------------

bool rtc_ok = false;
bool imu_ok = false;
bool sd_ok = false;

// -----------------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------------

void print_two_digits(
    File &file,
    int value);

void print_two_digits_serial(int value);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println(
      "XIAO ESP32-C3 SD + DS3231 + ICM-20948 logger");

  // ---------------------------------------------------------------------------
  // I2C
  // ---------------------------------------------------------------------------

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("I2C initialisation failed");
  }

  Wire.setClock(I2C_FREQUENCY_HZ);
  Wire.setTimeOut(100);

  // ---------------------------------------------------------------------------
  // DS3231 RTC
  // ---------------------------------------------------------------------------

  rtc_ok = rtc.begin();

  if (!rtc_ok) {
    Serial.println("DS3231 RTC not found!");
  } else {
    Serial.println("DS3231 RTC OK");

    if (rtc.lostPower()) {
      Serial.println(
          "RTC lost power. Setting RTC to compile time.");

      rtc.adjust(
          DateTime(F(__DATE__), F(__TIME__)));
    }

    const DateTime current_time = rtc.now();

    Serial.print("RTC time: ");
    Serial.println(current_time.timestamp());
  }

  // ---------------------------------------------------------------------------
  // ICM-20948
  // ---------------------------------------------------------------------------

  imu.begin(Wire, IMU_AD0_VALUE);

  Serial.print(
      "ICM-20948 initialisation returned: ");

  Serial.println(imu.statusString());

  if (imu.status == ICM_20948_Stat_Ok) {
    imu_ok = true;

    Serial.println("ICM-20948 OK");

    const uint8_t who_am_i = imu.getWhoAmI();

    Serial.printf(
        "ICM-20948 WHO_AM_I: 0x%02X\n",
        static_cast<unsigned int>(who_am_i));
  } else {
    Serial.println(
        "ICM-20948 not detected. "
        "Check the wiring and address setting.");
  }

  // ---------------------------------------------------------------------------
  // SD card
  // ---------------------------------------------------------------------------

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  SPI.begin(
      SD_SCK,
      SD_MISO,
      SD_MOSI,
      SD_CS);

  sd_ok = SD.begin(
      SD_CS,
      SPI);

  if (!sd_ok) {
    Serial.println("SD card initialisation failed!");
  } else {
    Serial.println("SD card OK");

    const bool new_file =
        !SD.exists(LOG_FILE_PATH);

    log_file = SD.open(
        LOG_FILE_PATH,
        FILE_APPEND);

    if (log_file) {
      if (new_file || log_file.size() == 0) {
        log_file.println(
            "elapsed_ms,"
            "date,"
            "time,"
            "accel_x_mg,"
            "accel_y_mg,"
            "accel_z_mg,"
            "gyro_x_dps,"
            "gyro_y_dps,"
            "gyro_z_dps,"
            "mag_x_uT,"
            "mag_y_uT,"
            "mag_z_uT,"
            "temperature_C");

        log_file.flush();
      }

      log_file.close();

      Serial.println("log.csv ready");
    } else {
      Serial.println(
          "Could not create or open log.csv");
    }
  }
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {
  DateTime current_time;

  if (rtc_ok) {
    current_time = rtc.now();
  }

  float accel_x = NAN;
  float accel_y = NAN;
  float accel_z = NAN;

  float gyro_x = NAN;
  float gyro_y = NAN;
  float gyro_z = NAN;

  float mag_x = NAN;
  float mag_y = NAN;
  float mag_z = NAN;

  float temperature = NAN;

  // ---------------------------------------------------------------------------
  // Read ICM-20948
  // ---------------------------------------------------------------------------

  if (imu_ok) {
    /*
     * Read the latest sensor registers directly.
     *
     * This is more reliable with your module than depending on
     * imu.dataReady().
     */
    imu.getAGMT();

    if (imu.status == ICM_20948_Stat_Ok) {
      accel_x = imu.accX();
      accel_y = imu.accY();
      accel_z = imu.accZ();

      gyro_x = imu.gyrX();
      gyro_y = imu.gyrY();
      gyro_z = imu.gyrZ();

      mag_x = imu.magX();
      mag_y = imu.magY();
      mag_z = imu.magZ();

      temperature = imu.temp();
    } else {
      Serial.print("ICM-20948 read failed: ");
      Serial.println(imu.statusString());
    }
  }

  // ---------------------------------------------------------------------------
  // Serial output
  // ---------------------------------------------------------------------------

  if (rtc_ok) {
    Serial.print(current_time.year());
    Serial.print("-");

    print_two_digits_serial(
        current_time.month());

    Serial.print("-");

    print_two_digits_serial(
        current_time.day());

    Serial.print(" ");

    print_two_digits_serial(
        current_time.hour());

    Serial.print(":");

    print_two_digits_serial(
        current_time.minute());

    Serial.print(":");

    print_two_digits_serial(
        current_time.second());
  } else {
    Serial.print("RTC unavailable");
  }

  Serial.print(" | Accel [mg] X: ");
  Serial.print(accel_x);

  Serial.print(" Y: ");
  Serial.print(accel_y);

  Serial.print(" Z: ");
  Serial.print(accel_z);

  Serial.print(" | Gyro [dps] X: ");
  Serial.print(gyro_x);

  Serial.print(" Y: ");
  Serial.print(gyro_y);

  Serial.print(" Z: ");
  Serial.print(gyro_z);

  Serial.print(" | Mag [uT] X: ");
  Serial.print(mag_x);

  Serial.print(" Y: ");
  Serial.print(mag_y);

  Serial.print(" Z: ");
  Serial.print(mag_z);

  Serial.print(" | Temperature [C]: ");
  Serial.println(temperature);

  // ---------------------------------------------------------------------------
  // CSV logging
  // ---------------------------------------------------------------------------

  if (sd_ok) {
    log_file = SD.open(
        LOG_FILE_PATH,
        FILE_APPEND);

    if (log_file) {
      log_file.print(millis());
      log_file.print(",");

      if (rtc_ok) {
        log_file.print(
            current_time.year());

        log_file.print("-");

        print_two_digits(
            log_file,
            current_time.month());

        log_file.print("-");

        print_two_digits(
            log_file,
            current_time.day());

        log_file.print(",");

        print_two_digits(
            log_file,
            current_time.hour());

        log_file.print(":");

        print_two_digits(
            log_file,
            current_time.minute());

        log_file.print(":");

        print_two_digits(
            log_file,
            current_time.second());

        log_file.print(",");
      } else {
        log_file.print("NA,NA,");
      }

      log_file.print(accel_x);
      log_file.print(",");

      log_file.print(accel_y);
      log_file.print(",");

      log_file.print(accel_z);
      log_file.print(",");

      log_file.print(gyro_x);
      log_file.print(",");

      log_file.print(gyro_y);
      log_file.print(",");

      log_file.print(gyro_z);
      log_file.print(",");

      log_file.print(mag_x);
      log_file.print(",");

      log_file.print(mag_y);
      log_file.print(",");

      log_file.print(mag_z);
      log_file.print(",");

      log_file.println(temperature);

      log_file.close();
    } else {
      Serial.println(
          "Could not open log.csv");
    }
  }

  delay(SAMPLE_INTERVAL_MS);
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

void print_two_digits(
    File &file,
    int value) {

  if (value < 10) {
    file.print("0");
  }

  file.print(value);
}

void print_two_digits_serial(int value) {
  if (value < 10) {
    Serial.print("0");
  }

  Serial.print(value);
}