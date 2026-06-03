#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <ICM_20948.h>   // SparkFun ICM-20948 library

// SD card SPI pins
#define SD_CS    D3
#define SD_SCK   D2
#define SD_MISO  D1
#define SD_MOSI  D0

// I2C pins
#define I2C_SDA  D4
#define I2C_SCL  D5

// ICM-20948 I2C address selector.
// For many SparkFun boards the default is AD0 = 1.
// If your IMU is not detected, change this to 0.
#define AD0_VAL  1

RTC_DS1307 rtc;
ICM_20948_I2C imu;

File logFile;

bool rtcOK = false;
bool sdOK = false;
bool imuOK = false;

void print2digits(File &file, int value) {
  if (value < 10) file.print("0");
  file.print(value);
}

void print2digitsSerial(int value) {
  if (value < 10) Serial.print("0");
  Serial.print(value);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("XIAO ESP32-C3 SD + RTC + ICM-20948 logger");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // RTC
  rtcOK = rtc.begin();

  if (!rtcOK) {
    Serial.println("RTC not found!");
  } else {
    Serial.println("RTC OK");

    if (!rtc.isrunning()) {
      Serial.println("RTC is not running. Setting RTC to compile time.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // ICM-20948
  imu.begin(Wire, AD0_VAL);

  Serial.print("ICM-20948 initialisation returned: ");
  Serial.println(imu.statusString());

  if (imu.status == ICM_20948_Stat_Ok) {
    imuOK = true;
    Serial.println("ICM-20948 OK");
  } else {
    Serial.println("ICM-20948 not detected. Check wiring or try AD0_VAL = 0.");
  }

  // SD card
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  sdOK = SD.begin(SD_CS, SPI);

  if (!sdOK) {
    Serial.println("SD card init failed!");
  } else {
    Serial.println("SD card OK");

    bool newFile = !SD.exists("/log.csv");

    logFile = SD.open ("/log.csv", FILE_APPEND);

    if (logFile) {
      if (newFile || logFile.size() == 0) {
        logFile.println("millis,date,time,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,temp_C");
      }

      logFile.close();
      Serial.println("log.csv ready");
    } else {
      Serial.println("Could not create/open log.csv");
    }
  }
}

void loop() {
  DateTime now;

  if (rtcOK) {
    now = rtc.now();
  }

  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  float mx = NAN;
  float my = NAN;
  float mz = NAN;

  float temp = NAN;

  if (imuOK) {
    if (imu.dataReady()) {
      imu.getAGMT();

      ax = imu.accX();   // mg
      ay = imu.accY();   // mg
      az = imu.accZ();   // mg

      gx = imu.gyrX();   // degrees per second
      gy = imu.gyrY();   // degrees per second
      gz = imu.gyrZ();   // degrees per second

      mx = imu.magX();   // microtesla
      my = imu.magY();   // microtesla
      mz = imu.magZ();   // microtesla

      temp = imu.temp(); // Celsius
    } else {
      Serial.println("ICM-20948 data not ready");
    }
  }

  // Serial output
  if (rtcOK) {
    Serial.print(now.year());
    Serial.print("-");
    print2digitsSerial(now.month());
    Serial.print("-");
    print2digitsSerial(now.day());
    Serial.print(" ");

    print2digitsSerial(now.hour());
    Serial.print(":");
    print2digitsSerial(now.minute());
    Serial.print(":");
    print2digitsSerial(now.second());
  } else {
    Serial.print("RTC unavailable");
  }

  Serial.print(" | Accel mg: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);

  Serial.print(" | Gyro dps: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);

  Serial.print(" | Mag uT: ");
  Serial.print(mx);
  Serial.print(", ");
  Serial.print(my);
  Serial.print(", ");
  Serial.print(mz);

  Serial.print(" | Temp C: ");
  Serial.println(temp);

  // CSV logging
  if (sdOK) {
    logFile = SD.open("/log.csv", FILE_APPEND);

    if (logFile) {
      logFile.print(millis());
      logFile.print(",");

      if (rtcOK) {
        logFile.print(now.year());
        logFile.print("-");
        print2digits(logFile, now.month());
        logFile.print("-");
        print2digits(logFile, now.day());
        logFile.print(",");

        print2digits(logFile, now.hour());
        logFile.print(":");
        print2digits(logFile, now.minute());
        logFile.print(":");
        print2digits(logFile, now.second());
        logFile.print(",");
      } else {
        logFile.print("NA,NA,");
      }

      logFile.print(ax);
      logFile.print(",");
      logFile.print(ay);
      logFile.print(",");
      logFile.print(az);
      logFile.print(",");

      logFile.print(gx);
      logFile.print(",");
      logFile.print(gy);
      logFile.print(",");
      logFile.print(gz);
      logFile.print(",");

      logFile.print(mx);
      logFile.print(",");
      logFile.print(my);
      logFile.print(",");
      logFile.print(mz);
      logFile.print(",");

      logFile.println(temp);

      logFile.close();
    } else {
      Serial.println("Could not open log.csv");
    }
  }

  delay(1000);
}
