#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <ICM_20948.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>

// -----------------------------------------------------------------------------
// Debug configuration
// -----------------------------------------------------------------------------

// Print live IMU measurements to the Serial Monitor.
constexpr bool DEBUG_PRINT_IMU = true;

// Print periodic logger statistics such as acquired/logged rate and errors.
constexpr bool DEBUG_PRINT_LOGGER_STATUS = true;

// Printing all 100 samples per second at 115200 baud could reduce logging
// performance. Ten monitor updates per second is normally sufficient.
constexpr uint32_t IMU_MONITOR_RATE_HZ = 10;

constexpr uint32_t IMU_MONITOR_INTERVAL_MS =
    1000UL / IMU_MONITOR_RATE_HZ;

// -----------------------------------------------------------------------------
// Hardware feature selection
// -----------------------------------------------------------------------------

// Enable DS3231 RTC module.
constexpr bool ENABLE_RTC = true;

// Enable writing measurements to the microSD card.
constexpr bool ENABLE_SD = true;

// false: read the latest registers directly at the requested sample rate.
// true: read only when imu.dataReady() returns true.
constexpr bool REQUIRE_IMU_DATA_READY = false;

// -----------------------------------------------------------------------------
// Pin configuration
// -----------------------------------------------------------------------------

// Adafruit microSD SPI breakout:
//
// Adafruit SI  -> XIAO D0
// Adafruit SO  -> XIAO D1
// Adafruit CLK -> XIAO D2
// Adafruit CS  -> XIAO D3

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

constexpr uint32_t I2C_FREQUENCY_HZ = 400000;

// The ADDR+1 trace was cut, so the ICM-20948 is at 0x69.
constexpr bool IMU_AD0_VALUE = true;

// -----------------------------------------------------------------------------
// Button control configuration
// -----------------------------------------------------------------------------

// One control button is used to avoid ambiguous two-button sequences.
constexpr uint8_t CONTROL_BUTTON_PIN = D9; // GPIO 9 (BOOT)
constexpr bool CONTROL_BUTTON_ACTIVE_LOW = true;

constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t BUTTON_SEQUENCE_TIMEOUT_MS = 700;

constexpr uint8_t START_CLICK_COUNT = 1;
constexpr uint8_t PAUSE_CLICK_COUNT = 2;
constexpr uint8_t STOP_CLICK_COUNT  = 3;

// -----------------------------------------------------------------------------
// Status LED configuration
// -----------------------------------------------------------------------------

// Optional external status LED.
//
// The XIAO ESP32-C3 does not have a user-controllable onboard LED.
// Set ENABLE_STATUS_LED to true only if you wire an external LED through
// a suitable resistor, for example to D10.

constexpr bool ENABLE_STATUS_LED = false;
constexpr uint8_t STATUS_LED_PIN = D10;
constexpr bool STATUS_LED_ACTIVE_LOW = false;

constexpr uint32_t LED_IDLE_BLINK_INTERVAL_MS = 1000;
constexpr uint32_t LED_PAUSED_BLINK_INTERVAL_MS = 250;
constexpr uint32_t LED_STOPPED_BLINK_INTERVAL_MS = 2000;

// -----------------------------------------------------------------------------
// Logger configuration
// -----------------------------------------------------------------------------

constexpr uint32_t SERIAL_BAUD = 115200;

constexpr uint32_t SAMPLE_RATE_HZ = 100;

constexpr int64_t SAMPLE_PERIOD_US =
    1000000LL / SAMPLE_RATE_HZ;

constexpr uint32_t RTC_REFRESH_INTERVAL_MS = 1000;
constexpr uint32_t SD_FLUSH_INTERVAL_MS    = 1000;
constexpr uint32_t STATUS_INTERVAL_MS      = 1000;

// Higher than the initial diagnostic speed while still conservative.
constexpr uint32_t SD_SPI_FREQUENCY_HZ = 10000000;

constexpr char LOG_FILE_PATH[] = "/log.csv";

// Large enough to hold approximately one second of CSV measurements.
constexpr size_t LOG_BUFFER_SIZE = 16384;

char log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_used = 0;

// -----------------------------------------------------------------------------
// Logger run state
// -----------------------------------------------------------------------------

enum class LoggerState {
  IDLE,
  LOGGING,
  PAUSED,
  STOPPED
};

LoggerState logger_state = LoggerState::IDLE;

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
bool sd_ok  = false;

DateTime cached_rtc(2000, 1, 1, 0, 0, 0);

// -----------------------------------------------------------------------------
// Timing state
// -----------------------------------------------------------------------------

int64_t next_sample_us = 0;

uint32_t last_rtc_refresh_ms = 0;
uint32_t last_sd_flush_ms    = 0;
uint32_t last_status_ms     = 0;
uint32_t last_imu_print_ms   = 0;

// -----------------------------------------------------------------------------
// Button state
// -----------------------------------------------------------------------------

bool last_raw_button_pressed = false;
bool debounced_button_pressed = false;

uint32_t last_button_change_ms = 0;
uint32_t last_button_release_ms = 0;

uint8_t pending_button_clicks = 0;

// -----------------------------------------------------------------------------
// Status LED state
// -----------------------------------------------------------------------------

bool status_led_on = false;
uint32_t last_status_led_toggle_ms = 0;

// -----------------------------------------------------------------------------
// Counters
// -----------------------------------------------------------------------------

uint64_t sample_index = 0;

uint32_t samples_acquired = 0;
uint32_t samples_logged   = 0;

uint32_t imu_not_ready     = 0;
uint32_t imu_read_errors   = 0;
uint32_t scheduler_misses = 0;
uint32_t sd_write_errors   = 0;

// -----------------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------------

bool initialise_rtc();
bool initialise_imu();
bool initialise_sd();

void acquire_and_log_sample(int64_t sample_time_us);

void print_imu_measurements(
    uint64_t current_sample_index,
    uint64_t elapsed_ms,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    float mx,
    float my,
    float mz,
    float temperature);

bool append_to_log_buffer(
    const char *data,
    size_t length);

bool flush_log_buffer(bool sync_card);

void refresh_rtc(uint32_t now_ms);
void service_periodic_tasks(uint32_t now_ms);
void process_status_interval(uint32_t now_ms);

void service_control_button(uint32_t now_ms);
void process_button_sequence(uint8_t click_count);
void start_logger();
void pause_logger();
void stop_logger();
const char *logger_state_text();

void initialise_status_led();
void set_status_led(bool on);
void service_status_led(uint32_t now_ms);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  Serial.println();
  Serial.println("SheepSense");
  Serial.println("XIAO ESP32-C3 + ICM-20948 + SD");

  Serial.printf(
      "Target logging rate: %lu Hz\n",
      static_cast<unsigned long>(SAMPLE_RATE_HZ));

  if (DEBUG_PRINT_IMU) {
    Serial.printf(
        "IMU monitor rate: %lu Hz\n",
        static_cast<unsigned long>(IMU_MONITOR_RATE_HZ));
  } else {
    Serial.println("IMU monitor output disabled");
  }

  if (!DEBUG_PRINT_LOGGER_STATUS) {
    Serial.println("Logger status output disabled");
  }

  Serial.printf(
      "I2C pins: SDA=D4/GPIO%d, SCL=D5/GPIO%d\n",
      static_cast<int>(I2C_SDA),
      static_cast<int>(I2C_SCL));

  Serial.printf(
      "SD pins: MOSI=D0/GPIO%d, MISO=D1/GPIO%d, "
      "SCK=D2/GPIO%d, CS=D3/GPIO%d\n",
      static_cast<int>(SD_MOSI),
      static_cast<int>(SD_MISO),
      static_cast<int>(SD_SCK),
      static_cast<int>(SD_CS));

  pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);
  initialise_status_led();

  Serial.printf(
      "Control button: D6/GPIO%d, press sequence: "
      "1=start/resume, 2=pause, 3=stop\n",
      static_cast<int>(CONTROL_BUTTON_PIN));

  if (ENABLE_STATUS_LED) {
    Serial.printf(
        "Status LED enabled on GPIO%d\n",
        static_cast<int>(STATUS_LED_PIN));
  } else {
    Serial.println(
        "Status LED disabled: LED_BUILTIN is not defined");
  }

  // ---------------------------------------------------------------------------
  // I2C
  // ---------------------------------------------------------------------------

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("I2C initialisation failed");
  }

  Wire.setClock(I2C_FREQUENCY_HZ);
  Wire.setTimeOut(100);

  imu_ok = initialise_imu();

  if (ENABLE_RTC) {
    rtc_ok = initialise_rtc();
  } else {
    Serial.println("RTC disabled");
  }

  // ---------------------------------------------------------------------------
  // SD card
  // ---------------------------------------------------------------------------

  if (ENABLE_SD) {
    sd_ok = initialise_sd();
  } else {
    Serial.println("SD logging disabled");
  }

  const uint32_t now_ms = millis();

  last_rtc_refresh_ms = now_ms;
  last_sd_flush_ms = now_ms;
  last_status_ms = now_ms;
  last_imu_print_ms = now_ms;

  next_sample_us =
      esp_timer_get_time() + SAMPLE_PERIOD_US;

  Serial.println();
  Serial.println("Logger ready");
  Serial.println("Press the control button once to start logging");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {
  const int64_t now_us = esp_timer_get_time();
  const uint32_t now_ms = millis();

  service_control_button(now_ms);
  service_status_led(now_ms);

  if (logger_state == LoggerState::LOGGING &&
      now_us >= next_sample_us) {

    const int64_t lateness_us =
        now_us - next_sample_us;

    if (lateness_us >= SAMPLE_PERIOD_US) {
      const uint32_t missed_periods =
          static_cast<uint32_t>(
              lateness_us / SAMPLE_PERIOD_US);

      scheduler_misses += missed_periods;

      next_sample_us +=
          static_cast<int64_t>(missed_periods + 1) *
          SAMPLE_PERIOD_US;
    } else {
      next_sample_us += SAMPLE_PERIOD_US;
    }

    acquire_and_log_sample(now_us);
  }

  service_periodic_tasks(now_ms);
}


// -----------------------------------------------------------------------------
// Status LED
// -----------------------------------------------------------------------------

void initialise_status_led() {
  if (!ENABLE_STATUS_LED) {
    return;
  }

  pinMode(STATUS_LED_PIN, OUTPUT);
  set_status_led(false);
}

void set_status_led(bool on) {
  if (!ENABLE_STATUS_LED) {
    return;
  }

  status_led_on = on;

  const uint8_t output_level =
      STATUS_LED_ACTIVE_LOW
          ? (on ? LOW : HIGH)
          : (on ? HIGH : LOW);

  digitalWrite(STATUS_LED_PIN, output_level);
}

void service_status_led(uint32_t now_ms) {
  if (!ENABLE_STATUS_LED) {
    return;
  }

  switch (logger_state) {
    case LoggerState::IDLE:
      if ((now_ms - last_status_led_toggle_ms) >=
          LED_IDLE_BLINK_INTERVAL_MS) {

        set_status_led(!status_led_on);
        last_status_led_toggle_ms = now_ms;
      }
      break;

    case LoggerState::LOGGING:
      set_status_led(true);
      last_status_led_toggle_ms = now_ms;
      break;

    case LoggerState::PAUSED:
      if ((now_ms - last_status_led_toggle_ms) >=
          LED_PAUSED_BLINK_INTERVAL_MS) {

        set_status_led(!status_led_on);
        last_status_led_toggle_ms = now_ms;
      }
      break;

    case LoggerState::STOPPED:
      if ((now_ms - last_status_led_toggle_ms) >=
          LED_STOPPED_BLINK_INTERVAL_MS) {

        set_status_led(!status_led_on);
        last_status_led_toggle_ms = now_ms;
      }
      break;
  }
}


// -----------------------------------------------------------------------------
// Button control
// -----------------------------------------------------------------------------

void service_control_button(uint32_t now_ms) {
  const bool raw_button_pressed =
      CONTROL_BUTTON_ACTIVE_LOW
          ? digitalRead(CONTROL_BUTTON_PIN) == LOW
          : digitalRead(CONTROL_BUTTON_PIN) == HIGH;

  if (raw_button_pressed != last_raw_button_pressed) {
    last_raw_button_pressed = raw_button_pressed;
    last_button_change_ms = now_ms;
  }

  if ((now_ms - last_button_change_ms) <
      BUTTON_DEBOUNCE_MS) {

    return;
  }

  if (raw_button_pressed != debounced_button_pressed) {
    debounced_button_pressed = raw_button_pressed;

    if (!debounced_button_pressed) {
      ++pending_button_clicks;
      last_button_release_ms = now_ms;
    }
  }

  if (pending_button_clicks > 0 &&
      (now_ms - last_button_release_ms) >=
          BUTTON_SEQUENCE_TIMEOUT_MS) {

    const uint8_t click_count = pending_button_clicks;
    pending_button_clicks = 0;

    process_button_sequence(click_count);
  }
}

void process_button_sequence(uint8_t click_count) {
  if (click_count == START_CLICK_COUNT) {
    start_logger();
    return;
  }

  if (click_count == PAUSE_CLICK_COUNT) {
    pause_logger();
    return;
  }

  if (click_count >= STOP_CLICK_COUNT) {
    stop_logger();
    return;
  }
}

void start_logger() {
  if (logger_state == LoggerState::STOPPED) {
    Serial.println(
        "Logger already stopped; press reset to start a new run");
    return;
  }

  if (!imu_ok) {
    Serial.println("Cannot start: IMU is not ready");
    return;
  }

  if (ENABLE_SD && !sd_ok) {
    Serial.println("Cannot start: SD card is not ready");
    return;
  }

  next_sample_us =
      esp_timer_get_time() + SAMPLE_PERIOD_US;

  last_status_ms = millis();
  last_imu_print_ms = millis();

  samples_acquired = 0;
  samples_logged = 0;
  imu_not_ready = 0;
  imu_read_errors = 0;
  scheduler_misses = 0;
  sd_write_errors = 0;

  logger_state = LoggerState::LOGGING;
  last_status_led_toggle_ms = millis();
  set_status_led(true);

  Serial.println("Logger running");
}

void pause_logger() {
  if (logger_state != LoggerState::LOGGING) {
    Serial.println("Logger is not running");
    return;
  }

  if (sd_ok) {
    flush_log_buffer(true);
  }

  logger_state = LoggerState::PAUSED;
  last_status_led_toggle_ms = millis();
  set_status_led(false);

  Serial.println("Logger paused");
}

void stop_logger() {
  if (logger_state == LoggerState::STOPPED) {
    Serial.println("Logger already stopped");
    return;
  }

  if (sd_ok) {
    flush_log_buffer(true);
    log_file.close();
  }

  logger_state = LoggerState::STOPPED;
  last_status_led_toggle_ms = millis();
  set_status_led(false);

  Serial.println("Logger stopped");
  Serial.println("log.csv finalized; press reset to start a new run");
}

const char *logger_state_text() {
  switch (logger_state) {
    case LoggerState::IDLE:
      return "IDLE";

    case LoggerState::LOGGING:
      return "LOGGING";

    case LoggerState::PAUSED:
      return "PAUSED";

    case LoggerState::STOPPED:
      return "STOPPED";

    default:
      return "UNKNOWN";
  }
}

// -----------------------------------------------------------------------------
// ICM-20948 initialisation
// -----------------------------------------------------------------------------

bool initialise_imu() {
  constexpr uint8_t MAX_ATTEMPTS = 5;

  Serial.println();
  Serial.println(
      "Initializing ICM-20948 at address 0x69");

  for (uint8_t attempt = 1;
       attempt <= MAX_ATTEMPTS;
       ++attempt) {

    Serial.printf(
        "ICM-20948 initialisation attempt %u/%u\n",
        static_cast<unsigned int>(attempt),
        static_cast<unsigned int>(MAX_ATTEMPTS));

    imu.begin(Wire, IMU_AD0_VALUE);

    Serial.print("initialisation result: ");
    Serial.println(imu.statusString());

    if (imu.status == ICM_20948_Stat_Ok) {
      break;
    }

    delay(500);
  }

  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(
        "ICM-20948 initialisation failed");

    return false;
  }

  const uint8_t who_am_i = imu.getWhoAmI();

  Serial.printf(
      "ICM-20948 WHO_AM_I: 0x%02X\n",
      static_cast<unsigned int>(who_am_i));

  if (who_am_i != 0xEA) {
    Serial.println(
        "Warning: unexpected WHO_AM_I value");
  }

  delay(250);

  Serial.println("Testing direct IMU data read...");

  const uint32_t test_start_ms = millis();

  while ((millis() - test_start_ms) < 2000) {
    imu.getAGMT();

    if (imu.status == ICM_20948_Stat_Ok) {
      Serial.println(
          "First ICM-20948 sample received");

      Serial.printf(
          "Accel: X=%.2f, Y=%.2f, Z=%.2f mg\n",
          imu.accX(),
          imu.accY(),
          imu.accZ());

      Serial.printf(
          "Gyro: X=%.2f, Y=%.2f, Z=%.2f dps\n",
          imu.gyrX(),
          imu.gyrY(),
          imu.gyrZ());

      Serial.printf(
          "Mag: X=%.2f, Y=%.2f, Z=%.2f uT\n",
          imu.magX(),
          imu.magY(),
          imu.magZ());

      Serial.printf(
          "Temperature: %.2f C\n",
          imu.temp());

      Serial.println("ICM-20948 ready");
      return true;
    }

    Serial.print("Direct IMU read failed: ");
    Serial.println(imu.statusString());

    delay(100);
  }

  Serial.println(
      "ICM-20948 initialized but produced no valid data");

  return false;
}

// -----------------------------------------------------------------------------
// RTC initialisation
// -----------------------------------------------------------------------------

bool initialise_rtc() {
  Serial.println();
  Serial.println("Initializing DS3231 RTC");

  if (!rtc.begin()) {
    Serial.println("DS3231 RTC not found");
    return false;
  }

  Serial.println("DS3231 RTC detected");

  if (rtc.lostPower()) {
    Serial.println(
        "RTC lost power; setting compile time");

    rtc.adjust(
        DateTime(F(__DATE__), F(__TIME__)));
  }

  cached_rtc = rtc.now();

  Serial.print("RTC time: ");
  Serial.println(cached_rtc.timestamp());

  return true;
}

// -----------------------------------------------------------------------------
// SD initialisation
// -----------------------------------------------------------------------------

bool initialise_sd() {
  Serial.println();
  Serial.println("Initializing SD card");

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  SPI.begin(
      SD_SCK,
      SD_MISO,
      SD_MOSI,
      SD_CS);

  if (!SD.begin(
          SD_CS,
          SPI,
          SD_SPI_FREQUENCY_HZ)) {

    Serial.println("SD card initialisation failed");
    return false;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card detected");
    return false;
  }

  Serial.println("SD card detected");

  const uint64_t card_size_mb =
      SD.cardSize() / (1024ULL * 1024ULL);

  Serial.printf(
      "SD card size: %llu MB\n",
      static_cast<unsigned long long>(card_size_mb));

  // Start a fresh log on every restart.
  if (SD.exists(LOG_FILE_PATH)) {
    if (!SD.remove(LOG_FILE_PATH)) {
      Serial.println("Could not remove previous log.csv");
      return false;
    }
  }

  // Open once and keep this handle open.
  log_file = SD.open(LOG_FILE_PATH, FILE_WRITE);

  if (!log_file) {
    Serial.println("Could not create log.csv");
    return false;
  }

  static constexpr char CSV_HEADER[] =
      "sample_index,"
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
      "temperature_C\n";

  constexpr size_t HEADER_LENGTH =
      sizeof(CSV_HEADER) - 1;

  const size_t bytes_written =
      log_file.write(
          reinterpret_cast<const uint8_t *>(CSV_HEADER),
          HEADER_LENGTH);

  log_file.flush();

  if (bytes_written != HEADER_LENGTH) {
    Serial.printf(
        "Header write failed: wrote %u of %u bytes\n",
        static_cast<unsigned int>(bytes_written),
        static_cast<unsigned int>(HEADER_LENGTH));

    log_file.close();
    return false;
  }

  if (log_file.position() != HEADER_LENGTH) {
    Serial.printf(
        "Unexpected file position: %u, expected %u\n",
        static_cast<unsigned int>(log_file.position()),
        static_cast<unsigned int>(HEADER_LENGTH));

    log_file.close();
    return false;
  }

  Serial.printf(
      "CSV header written successfully: %u bytes\n",
      static_cast<unsigned int>(bytes_written));

  Serial.println("log.csv ready");
  return true;
}

// -----------------------------------------------------------------------------
// Sampling and logging
// -----------------------------------------------------------------------------

void acquire_and_log_sample(int64_t sample_time_us) {
  if (!imu_ok) {
    return;
  }

  if (REQUIRE_IMU_DATA_READY &&
      !imu.dataReady()) {

    ++imu_not_ready;
    return;
  }

  imu.getAGMT();

  if (imu.status != ICM_20948_Stat_Ok) {
    ++imu_read_errors;

    if (imu_read_errors == 1) {
      Serial.print("ICM-20948 read error: ");
      Serial.println(imu.statusString());
    }

    return;
  }

  ++sample_index;
  ++samples_acquired;

  const float ax = imu.accX();
  const float ay = imu.accY();
  const float az = imu.accZ();

  const float gx = imu.gyrX();
  const float gy = imu.gyrY();
  const float gz = imu.gyrZ();

  const float mx = imu.magX();
  const float my = imu.magY();
  const float mz = imu.magZ();

  const float temperature = imu.temp();

  const uint64_t elapsed_ms =
      static_cast<uint64_t>(
          sample_time_us / 1000LL);

  // ---------------------------------------------------------------------------
  // Serial Monitor IMU output
  // ---------------------------------------------------------------------------

  const uint32_t now_ms = millis();

  if (DEBUG_PRINT_IMU &&
      (now_ms - last_imu_print_ms) >=
          IMU_MONITOR_INTERVAL_MS) {

    print_imu_measurements(
        sample_index,
        elapsed_ms,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz,
        temperature);

    last_imu_print_ms = now_ms;
  }

  // ---------------------------------------------------------------------------
  // SD logging
  // ---------------------------------------------------------------------------

  if (!sd_ok) {
    return;
  }

  char date_text[11];
  char time_text[9];

  if (rtc_ok) {
    snprintf(
        date_text,
        sizeof(date_text),
        "%04u-%02u-%02u",
        static_cast<unsigned int>(
            cached_rtc.year()),
        static_cast<unsigned int>(
            cached_rtc.month()),
        static_cast<unsigned int>(
            cached_rtc.day()));

    snprintf(
        time_text,
        sizeof(time_text),
        "%02u:%02u:%02u",
        static_cast<unsigned int>(
            cached_rtc.hour()),
        static_cast<unsigned int>(
            cached_rtc.minute()),
        static_cast<unsigned int>(
            cached_rtc.second()));
  } else {
    snprintf(
        date_text,
        sizeof(date_text),
        "NA");

    snprintf(
        time_text,
        sizeof(time_text),
        "NA");
  }

  char line[280];

  const int line_length = snprintf(
      line,
      sizeof(line),
      "%llu,"
      "%llu,"
      "%s,"
      "%s,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f,"
      "%.3f\n",
      static_cast<unsigned long long>(
          sample_index),
      static_cast<unsigned long long>(
          elapsed_ms),
      date_text,
      time_text,
      ax,
      ay,
      az,
      gx,
      gy,
      gz,
      mx,
      my,
      mz,
      temperature);

  if (line_length <= 0 ||
      static_cast<size_t>(line_length) >=
          sizeof(line)) {

    ++sd_write_errors;
    return;
  }

  if (append_to_log_buffer(
          line,
          static_cast<size_t>(line_length))) {

    ++samples_logged;
  } else {
    ++sd_write_errors;
  }
}

// -----------------------------------------------------------------------------
// IMU Serial Monitor output
// -----------------------------------------------------------------------------

void print_imu_measurements(
    uint64_t current_sample_index,
    uint64_t elapsed_ms,
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz,
    float mx,
    float my,
    float mz,
    float temperature) {

  Serial.printf(
      "Sample: %llu | "
      "Time: %llu ms | "
      "Accel [mg] X: %.2f Y: %.2f Z: %.2f | "
      "Gyro [dps] X: %.2f Y: %.2f Z: %.2f | "
      "Mag [uT] X: %.2f Y: %.2f Z: %.2f | "
      "Temp: %.2f C\n",
      static_cast<unsigned long long>(
          current_sample_index),
      static_cast<unsigned long long>(
          elapsed_ms),
      ax,
      ay,
      az,
      gx,
      gy,
      gz,
      mx,
      my,
      mz,
      temperature);
}

// -----------------------------------------------------------------------------
// Buffered SD writing
// -----------------------------------------------------------------------------

bool append_to_log_buffer(
    const char *data,
    size_t length) {

  if (!sd_ok ||
      !log_file ||
      data == nullptr ||
      length == 0) {

    return false;
  }

  if (length > LOG_BUFFER_SIZE) {
    return false;
  }

  if ((log_buffer_used + length) >
      LOG_BUFFER_SIZE) {

    if (!flush_log_buffer(false)) {
      return false;
    }
  }

  memcpy(
      log_buffer + log_buffer_used,
      data,
      length);

  log_buffer_used += length;

  return true;
}

bool flush_log_buffer(bool sync_card) {
  if (!sd_ok || !log_file) {
    return false;
  }

  if (log_buffer_used > 0) {
    const size_t written =
        log_file.write(
            reinterpret_cast<const uint8_t *>(
                log_buffer),
            log_buffer_used);

    if (written != log_buffer_used) {
      Serial.println(
          "SD write failed; logging disabled");

      ++sd_write_errors;

      log_buffer_used = 0;
      sd_ok = false;

      log_file.close();

      return false;
    }

    log_buffer_used = 0;
  }

  if (sync_card) {
    log_file.flush();
  }

  return true;
}

// -----------------------------------------------------------------------------
// Periodic operations
// -----------------------------------------------------------------------------

void refresh_rtc(uint32_t now_ms) {
  if (!rtc_ok) {
    return;
  }

  if ((now_ms - last_rtc_refresh_ms) <
      RTC_REFRESH_INTERVAL_MS) {

    return;
  }

  cached_rtc = rtc.now();
  last_rtc_refresh_ms = now_ms;
}

void service_periodic_tasks(uint32_t now_ms) {
  refresh_rtc(now_ms);

  if (sd_ok &&
      logger_state != LoggerState::STOPPED &&
      (now_ms - last_sd_flush_ms) >=
          SD_FLUSH_INTERVAL_MS) {

    flush_log_buffer(true);
    last_sd_flush_ms = now_ms;
  }

  if ((now_ms - last_status_ms) >=
      STATUS_INTERVAL_MS) {

    process_status_interval(now_ms);
  }
}

// -----------------------------------------------------------------------------
// Logger status output
// -----------------------------------------------------------------------------

void process_status_interval(uint32_t now_ms) {
  const uint32_t elapsed_ms =
      now_ms - last_status_ms;

  const float acquired_rate =
      elapsed_ms > 0
          ? (samples_acquired * 1000.0f) /
                static_cast<float>(elapsed_ms)
          : 0.0f;

  const float logged_rate =
      elapsed_ms > 0
          ? (samples_logged * 1000.0f) /
                static_cast<float>(elapsed_ms)
          : 0.0f;

  if (DEBUG_PRINT_LOGGER_STATUS) {
    Serial.printf(
        "State: %s | "
        "Acquired: %.1f Hz | "
        "Logged: %.1f Hz | "
        "not ready: %lu | "
        "IMU errors: %lu | "
        "missed periods: %lu | "
        "SD errors: %lu | "
        "IMU: %s | "
        "SD: %s | "
        "RTC: %s\n",
        logger_state_text(),
        acquired_rate,
        logged_rate,
        static_cast<unsigned long>(
            imu_not_ready),
        static_cast<unsigned long>(
            imu_read_errors),
        static_cast<unsigned long>(
            scheduler_misses),
        static_cast<unsigned long>(
            sd_write_errors),
        imu_ok ? "OK" : "FAILED",
        sd_ok ? "OK" : "FAILED",
        rtc_ok ? "OK" : "DISABLED/FAILED");
  }

  // Reset interval counters even if status printing is disabled.
  samples_acquired = 0;
  samples_logged = 0;

  imu_not_ready = 0;
  imu_read_errors = 0;
  scheduler_misses = 0;
  sd_write_errors = 0;

  last_status_ms = now_ms;
}