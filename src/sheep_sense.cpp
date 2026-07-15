// =============================================================================
// SheepSense IMU logger - XIAO ESP32-C3 + ICM-20948 + microSD + DS3231
// =============================================================================
// Because everything ran in one cooperative loop, the once-per-second work
// (log_file.flush() SD sync, rtc.now() I2C read, and the status printf) blocked
// the loop for ~8-30 ms. now,  acquisition
// is driven by a hardware timer and pushed into a circular buffer; the SD
// work runs separately as a pure consumer. 
//
//   * A periodic esp_timer (100 Hz) notifies a HIGH-PRIORITY FreeRTOS task.
//   * That task does the I2C IMU read and pushes a raw sample into a lock-free
//     single-producer/single-consumer ring buffer. 
//   * loop() is now only the CONSUMER: it drains the ring buffer, formats CSV,
//     writes blocks to SD, flushes once/sec, and handles RTC/Serial/button/LED.
// FreeRTOS preempts the SD write mid-flight, takes the sample on schedule, and resumes
// the write afterwards. The ring buffer (sized for ~2.5 s) absorbs the backlog.
// Net effect: sample timing is governed by the timer, not by SD/RTC/Serial, so
// the periodic glitch disappears.
// Wall-clock time is anchored once when logging starts and every
// per-row date/time is derived from esp_timer elapsed micros. 
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <ICM_20948.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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
constexpr uint32_t BUTTON_SEQUENCE_TIMEOUT_MS = 1200;

constexpr uint8_t START_CLICK_COUNT = 1;
constexpr uint8_t PAUSE_CLICK_COUNT = 2;
constexpr uint8_t STOP_CLICK_COUNT  = 3;

// External TTL sync pulse (camera frame marker) on D7 / GPIO20.
constexpr uint8_t SYNC_INTERRUPT_PIN = D7;

// Set by the GPIO ISR, latched and cleared by the acquisition task once per
// sample. volatile because it is shared between an ISR and a task.
volatile uint8_t sync_bit = 0;

void IRAM_ATTR handle_sync_pulse() {
  sync_bit = 1;
}

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

// SD flush and status printing still happen once per second, but they now run
// in the low-priority consumer (loop()) and no longer disturb sampling.
constexpr uint32_t SD_FLUSH_INTERVAL_MS = 1000;
constexpr uint32_t STATUS_INTERVAL_MS   = 1000;

// Higher than the initial diagnostic speed while still conservative.
constexpr uint32_t SD_SPI_FREQUENCY_HZ = 10000000;

constexpr char LOG_FILE_PATH[] = "/log.csv";

// Large enough to hold approximately one second of CSV measurements.
constexpr size_t LOG_BUFFER_SIZE = 16384;

char log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_used = 0;

// -----------------------------------------------------------------------------
// Acquisition task / ring buffer configuration   (NEW - T40-style)
// -----------------------------------------------------------------------------

// One raw, unformatted sample as produced by the acquisition task. Keeping the
// producer minimal (just copy the numbers) keeps the high-priority task short;
// the CSV formatting is done later in the consumer.
struct Sample {
  uint64_t index;
  int64_t  t_us;        // esp_timer_get_time() captured at the moment of read
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float temperature;
  uint8_t sync;
};

// Ring capacity in samples. 256 samples at 100 Hz buffers ~2.5 s, which easily
// covers any realistic SD write/flush stall.
constexpr uint32_t RING_CAPACITY = 256;

static Sample ring[RING_CAPACITY];

// Single-producer (acquisition task) / single-consumer (loop) indices.
// The producer only writes ring_head; the consumer only writes ring_tail.
// 32-bit aligned loads/stores are atomic on this RISC-V core, so no lock is
// needed - only release/acquire fences to order the payload against the index.
static volatile uint32_t ring_head = 0;
static volatile uint32_t ring_tail = 0;

// Acquisition task plumbing.
static TaskHandle_t acq_task_handle = nullptr;
static esp_timer_handle_t sample_timer = nullptr;

// Priority must be well above the Arduino loopTask (priority 1) so the timer can
// preempt an in-progress SD write. It stays below the esp_timer dispatch task.
constexpr UBaseType_t ACQ_TASK_PRIORITY = 10;
constexpr uint32_t ACQ_TASK_STACK_BYTES = 6144;

// -----------------------------------------------------------------------------
// Logger run state
// -----------------------------------------------------------------------------

enum class LoggerState {
  IDLE,
  LOGGING,
  PAUSED,
  STOPPED
};

// Read by the acquisition task, written by the button handler in loop context.
volatile LoggerState logger_state = LoggerState::IDLE;

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

// -----------------------------------------------------------------------------
// Wall-clock anchor
// -----------------------------------------------------------------------------
//
// Captured once when logging starts. Per-row date/time is then derived from
// esp_timer elapsed micros instead of re-reading the DS3231 every second.

DateTime log_start_dt(2000, 1, 1, 0, 0, 0);
int64_t  log_start_us = 0;

// -----------------------------------------------------------------------------
// Timing state (consumer side only)
// -----------------------------------------------------------------------------

uint32_t last_sd_flush_ms  = 0;
uint32_t last_status_ms    = 0;
uint32_t last_imu_print_ms = 0;

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
//
// Counters touched by the acquisition task are marked volatile. They are plain
// statistics; an occasional torn read across the once-per-second reset is
// harmless.

uint64_t sample_index = 0;              // acquisition task only

volatile uint32_t samples_acquired = 0; // acquisition task
uint32_t          samples_logged   = 0; // consumer

volatile uint32_t imu_not_ready    = 0; // acquisition task
volatile uint32_t imu_read_errors  = 0; // acquisition task
volatile uint32_t notify_overruns  = 0; // acquisition task (timer got ahead)
volatile uint32_t buffer_overflows = 0; // acquisition task (ring full)
uint32_t          sd_write_errors  = 0; // consumer

// -----------------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------------

bool initialise_rtc();
bool initialise_imu();
bool initialise_sd();

void sample_timer_cb(void *arg);
void acquisition_task(void *arg);
void consume_ring_buffer(uint32_t now_ms);
void format_and_store_sample(const Sample &s, uint32_t now_ms);

void compute_datetime(int64_t sample_us, char *date_text, char *time_text);

void print_imu_measurements(
    uint64_t current_sample_index,
    uint64_t elapsed_ms,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float temperature);

bool append_to_log_buffer(const char *data, size_t length);
bool flush_log_buffer(bool sync_card);

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
  Serial.println("XIAO ESP32-C3 + ICM-20948 + SD (v2, timer + ring buffer)");

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

  // External TTL sync pulse -> sets sync_bit in an ISR.
  pinMode(SYNC_INTERRUPT_PIN, INPUT_PULLDOWN);
  attachInterrupt(
      digitalPinToInterrupt(SYNC_INTERRUPT_PIN),
      handle_sync_pulse,
      RISING);

  Serial.printf(
      "Control button: D9/GPIO%d, press sequence: "
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

  last_sd_flush_ms = now_ms;
  last_status_ms = now_ms;
  last_imu_print_ms = now_ms;

  // ---------------------------------------------------------------------------
  // Acquisition task + periodic sampling timer   (created now, started on run)
  // ---------------------------------------------------------------------------

  BaseType_t task_created = xTaskCreatePinnedToCore(
      acquisition_task,
      "acq",
      ACQ_TASK_STACK_BYTES,
      nullptr,
      ACQ_TASK_PRIORITY,
      &acq_task_handle,
      0); // ESP32-C3 has a single application core (core 0)

  if (task_created != pdPASS) {
    Serial.println("FATAL: could not create acquisition task");
  }

  const esp_timer_create_args_t timer_args = {
      .callback = &sample_timer_cb,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "sample",
      .skip_unhandled_events = true,
  };

  if (esp_timer_create(&timer_args, &sample_timer) != ESP_OK) {
    Serial.println("FATAL: could not create sample timer");
  }

  Serial.println();
  Serial.println("Logger ready");
  Serial.println("Press the control button once to start logging");
}

// -----------------------------------------------------------------------------
// Main loop  ->  CONSUMER ONLY
// -----------------------------------------------------------------------------
//
// No sampling happens here any more. loop() drains whatever the acquisition
// task produced, writes it to SD, and does all the slow housekeeping.

void loop() {
  const uint32_t now_ms = millis();

  service_control_button(now_ms);
  service_status_led(now_ms);

  consume_ring_buffer(now_ms);

  service_periodic_tasks(now_ms);

  // Yield so the idle task / watchdog are serviced when there is nothing to do.
  vTaskDelay(1);
}

// -----------------------------------------------------------------------------
// Sampling timer callback  ->  wakes the acquisition task
// -----------------------------------------------------------------------------
//
// Runs in the high-priority esp_timer dispatch task (not a raw ISR), so a plain
// task notification is the correct primitive here.

void sample_timer_cb(void * /*arg*/) {
  if (acq_task_handle != nullptr) {
    xTaskNotifyGive(acq_task_handle);
  }
}

// -----------------------------------------------------------------------------
// Acquisition task  ->  PRODUCER  (equivalent of the T40 scanADC ISR)
// -----------------------------------------------------------------------------

void acquisition_task(void * /*arg*/) {
  for (;;) {
    // Block until the timer fires. The return value is the number of pending
    // notifications; >1 means the timer got ahead of us (a sample slipped).
    const uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (pending > 1) {
      notify_overruns += (pending - 1);
    }

    if (logger_state != LoggerState::LOGGING || !imu_ok) {
      continue;
    }

    if (REQUIRE_IMU_DATA_READY && !imu.dataReady()) {
      ++imu_not_ready;
      continue;
    }

    // Latch and clear the sync pulse for this sample.
    const uint8_t current_sync = sync_bit;
    sync_bit = 0;

    // Timestamp captured right at the read -> the true acquisition instant.
    const int64_t t_us = esp_timer_get_time();

    imu.getAGMT();
    if (imu.status != ICM_20948_Stat_Ok) {
      ++imu_read_errors;
      continue;
    }

    Sample s;
    s.index       = ++sample_index;
    s.t_us        = t_us;
    s.ax          = imu.accX();
    s.ay          = imu.accY();
    s.az          = imu.accZ();
    s.gx          = imu.gyrX();
    s.gy          = imu.gyrY();
    s.gz          = imu.gyrZ();
    s.mx          = imu.magX();
    s.my          = imu.magY();
    s.mz          = imu.magZ();
    s.temperature = imu.temp();
    s.sync        = current_sync;

    // ---- push into the ring buffer (lock-free SPSC) ----
    const uint32_t head = ring_head;
    const uint32_t next = (head + 1) % RING_CAPACITY;

    if (next == ring_tail) {
      // Consumer has fallen behind by a whole buffer. Drop and count.
      ++buffer_overflows;
    } else {
      ring[head] = s;
      // Publish the payload before advancing head.
      __atomic_thread_fence(__ATOMIC_RELEASE);
      ring_head = next;
      ++samples_acquired;
    }
  }
}

// -----------------------------------------------------------------------------
// Ring buffer consumer
// -----------------------------------------------------------------------------

void consume_ring_buffer(uint32_t now_ms) {
  uint32_t tail = ring_tail;
  uint32_t head = ring_head;
  // See the payload written before head was advanced.
  __atomic_thread_fence(__ATOMIC_ACQUIRE);

  while (tail != head) {
    Sample s = ring[tail];
    tail = (tail + 1) % RING_CAPACITY;
    ring_tail = tail;

    format_and_store_sample(s, now_ms);

    // Pick up anything produced while we were formatting.
    head = ring_head;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
  }
}

// -----------------------------------------------------------------------------
// Wall-clock derivation (no I2C on the hot path)
// -----------------------------------------------------------------------------

void compute_datetime(int64_t sample_us, char *date_text, char *time_text) {
  if (rtc_ok) {
    const int64_t elapsed_us = sample_us - log_start_us;
    const int32_t elapsed_s =
        static_cast<int32_t>(elapsed_us / 1000000LL);

    const DateTime dt = log_start_dt + TimeSpan(elapsed_s);

    snprintf(
        date_text, 11, "%04u-%02u-%02u",
        static_cast<unsigned int>(dt.year()),
        static_cast<unsigned int>(dt.month()),
        static_cast<unsigned int>(dt.day()));

    snprintf(
        time_text, 9, "%02u:%02u:%02u",
        static_cast<unsigned int>(dt.hour()),
        static_cast<unsigned int>(dt.minute()),
        static_cast<unsigned int>(dt.second()));
  } else {
    snprintf(date_text, 11, "NA");
    snprintf(time_text, 9, "NA");
  }
}

// -----------------------------------------------------------------------------
// Format one sample to CSV and hand it to the SD buffer
// -----------------------------------------------------------------------------

void format_and_store_sample(const Sample &s, uint32_t now_ms) {
  const uint64_t elapsed_ms =
      static_cast<uint64_t>(s.t_us / 1000LL);

  // Throttled Serial Monitor output.
  if (DEBUG_PRINT_IMU &&
      (now_ms - last_imu_print_ms) >= IMU_MONITOR_INTERVAL_MS) {

    print_imu_measurements(
        s.index, elapsed_ms,
        s.ax, s.ay, s.az,
        s.gx, s.gy, s.gz,
        s.mx, s.my, s.mz,
        s.temperature);

    last_imu_print_ms = now_ms;
  }

  if (!sd_ok) {
    return;
  }

  char date_text[11];
  char time_text[9];
  compute_datetime(s.t_us, date_text, time_text);

  char line[310];

  const int line_length = snprintf(
      line, sizeof(line),
      "%llu,"   // sample_index
      "%llu,"   // elapsed_ms
      "%s,"     // date
      "%s,"     // time
      "%.3f,%.3f,%.3f,"   // accel
      "%.3f,%.3f,%.3f,"   // gyro
      "%.3f,%.3f,%.3f,"   // mag
      "%.3f,"             // temperature
      "%u\n",             // sync_pulse
      static_cast<unsigned long long>(s.index),
      static_cast<unsigned long long>(elapsed_ms),
      date_text,
      time_text,
      s.ax, s.ay, s.az,
      s.gx, s.gy, s.gz,
      s.mx, s.my, s.mz,
      s.temperature,
      static_cast<unsigned int>(s.sync));

  if (line_length <= 0 ||
      static_cast<size_t>(line_length) >= sizeof(line)) {
    ++sd_write_errors;
    return;
  }

  if (append_to_log_buffer(line, static_cast<size_t>(line_length))) {
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
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz,
    float temperature) {

  Serial.printf(
      "Sample: %llu | "
      "Time: %llu ms | "
      "Accel [mg] X: %.2f Y: %.2f Z: %.2f | "
      "Gyro [dps] X: %.2f Y: %.2f Z: %.2f | "
      "Mag [uT] X: %.2f Y: %.2f Z: %.2f | "
      "Temp: %.2f C\n",
      static_cast<unsigned long long>(current_sample_index),
      static_cast<unsigned long long>(elapsed_ms),
      ax, ay, az,
      gx, gy, gz,
      mx, my, mz,
      temperature);
}

// -----------------------------------------------------------------------------
// Buffered SD writing
// -----------------------------------------------------------------------------

bool append_to_log_buffer(const char *data, size_t length) {
  if (!sd_ok || !log_file || data == nullptr || length == 0) {
    return false;
  }

  if (length > LOG_BUFFER_SIZE) {
    return false;
  }

  if ((log_buffer_used + length) > LOG_BUFFER_SIZE) {
    if (!flush_log_buffer(false)) {
      return false;
    }
  }

  memcpy(log_buffer + log_buffer_used, data, length);
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
            reinterpret_cast<const uint8_t *>(log_buffer),
            log_buffer_used);

    if (written != log_buffer_used) {
      Serial.println("SD write failed; logging disabled");

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
// Periodic operations  (consumer side, low priority - safe to block)
// -----------------------------------------------------------------------------

void service_periodic_tasks(uint32_t now_ms) {
  if (sd_ok &&
      logger_state != LoggerState::STOPPED &&
      (now_ms - last_sd_flush_ms) >= SD_FLUSH_INTERVAL_MS) {

    // This flush() can stall for many ms - harmless now, because the
    // acquisition task preempts it and keeps sampling on time.
    flush_log_buffer(true);
    last_sd_flush_ms = now_ms;
  }

  if ((now_ms - last_status_ms) >= STATUS_INTERVAL_MS) {
    process_status_interval(now_ms);
  }
}

// -----------------------------------------------------------------------------
// Logger status output
// -----------------------------------------------------------------------------

void process_status_interval(uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - last_status_ms;

  const float acquired_rate =
      elapsed_ms > 0
          ? (samples_acquired * 1000.0f) / static_cast<float>(elapsed_ms)
          : 0.0f;

  const float logged_rate =
      elapsed_ms > 0
          ? (samples_logged * 1000.0f) / static_cast<float>(elapsed_ms)
          : 0.0f;

  if (DEBUG_PRINT_LOGGER_STATUS) {
    Serial.printf(
        "State: %s | "
        "Acquired: %.1f Hz | "
        "Logged: %.1f Hz | "
        "not ready: %lu | "
        "IMU errors: %lu | "
        "timer overruns: %lu | "
        "buf overflow: %lu | "
        "SD errors: %lu | "
        "IMU: %s | SD: %s | RTC: %s\n",
        logger_state_text(),
        acquired_rate,
        logged_rate,
        static_cast<unsigned long>(imu_not_ready),
        static_cast<unsigned long>(imu_read_errors),
        static_cast<unsigned long>(notify_overruns),
        static_cast<unsigned long>(buffer_overflows),
        static_cast<unsigned long>(sd_write_errors),
        imu_ok ? "OK" : "FAILED",
        sd_ok ? "OK" : "FAILED",
        rtc_ok ? "OK" : "DISABLED/FAILED");
  }

  // Reset interval counters even if status printing is disabled.
  samples_acquired = 0;
  samples_logged = 0;
  imu_not_ready = 0;
  imu_read_errors = 0;
  notify_overruns = 0;
  buffer_overflows = 0;
  sd_write_errors = 0;

  last_status_ms = now_ms;
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
      if ((now_ms - last_status_led_toggle_ms) >= LED_IDLE_BLINK_INTERVAL_MS) {
        set_status_led(!status_led_on);
        last_status_led_toggle_ms = now_ms;
      }
      break;

    case LoggerState::LOGGING:
      set_status_led(true);
      last_status_led_toggle_ms = now_ms;
      break;

    case LoggerState::PAUSED:
      if ((now_ms - last_status_led_toggle_ms) >= LED_PAUSED_BLINK_INTERVAL_MS) {
        set_status_led(!status_led_on);
        last_status_led_toggle_ms = now_ms;
      }
      break;

    case LoggerState::STOPPED:
      if ((now_ms - last_status_led_toggle_ms) >= LED_STOPPED_BLINK_INTERVAL_MS) {
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

  if ((now_ms - last_button_change_ms) < BUTTON_DEBOUNCE_MS) {
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
      (now_ms - last_button_release_ms) >= BUTTON_SEQUENCE_TIMEOUT_MS) {

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

  // Anchor wall-clock time once. This is the only RTC read during a run.
  if (rtc_ok) {
    log_start_dt = rtc.now();
  }
  log_start_us = esp_timer_get_time();

  // Reset the ring buffer while the timer is stopped and the task idle.
  ring_head = 0;
  ring_tail = 0;

  last_status_ms = millis();
  last_imu_print_ms = millis();
  last_sd_flush_ms = millis();

  samples_acquired = 0;
  samples_logged = 0;
  imu_not_ready = 0;
  imu_read_errors = 0;
  notify_overruns = 0;
  buffer_overflows = 0;
  sd_write_errors = 0;

  logger_state = LoggerState::LOGGING;
  last_status_led_toggle_ms = millis();
  set_status_led(true);

  // Start the precise sampling clock.
  if (sample_timer != nullptr) {
    esp_timer_start_periodic(
        sample_timer,
        static_cast<uint64_t>(SAMPLE_PERIOD_US));
  }

  Serial.println("Logger running");
}

void pause_logger() {
  if (logger_state != LoggerState::LOGGING) {
    Serial.println("Logger is not running");
    return;
  }

  if (sample_timer != nullptr) {
    esp_timer_stop(sample_timer);
  }

  logger_state = LoggerState::PAUSED;

  // Drain anything still in the ring, then sync the card.
  consume_ring_buffer(millis());
  if (sd_ok) {
    flush_log_buffer(true);
  }

  last_status_led_toggle_ms = millis();
  set_status_led(false);

  Serial.println("Logger paused");
}

void stop_logger() {
  if (logger_state == LoggerState::STOPPED) {
    Serial.println("Logger already stopped");
    return;
  }

  if (sample_timer != nullptr) {
    esp_timer_stop(sample_timer);
  }

  logger_state = LoggerState::STOPPED;

  // Drain and finalise.
  consume_ring_buffer(millis());
  if (sd_ok) {
    flush_log_buffer(true);
    log_file.close();
  }

  last_status_led_toggle_ms = millis();
  set_status_led(false);

  Serial.println("Logger stopped");
  Serial.println("log.csv finalized; press reset to start a new run");
}

const char *logger_state_text() {
  switch (logger_state) {
    case LoggerState::IDLE:    return "IDLE";
    case LoggerState::LOGGING: return "LOGGING";
    case LoggerState::PAUSED:  return "PAUSED";
    case LoggerState::STOPPED: return "STOPPED";
    default:                   return "UNKNOWN";
  }
}

// -----------------------------------------------------------------------------
// ICM-20948 initialisation
// -----------------------------------------------------------------------------

bool initialise_imu() {
  constexpr uint8_t MAX_ATTEMPTS = 5;

  Serial.println();
  Serial.println("Initializing ICM-20948 at address 0x69");

  for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
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
    Serial.println("ICM-20948 initialisation failed");
    return false;
  }

  const uint8_t who_am_i = imu.getWhoAmI();

  Serial.printf(
      "ICM-20948 WHO_AM_I: 0x%02X\n",
      static_cast<unsigned int>(who_am_i));

  if (who_am_i != 0xEA) {
    Serial.println("Warning: unexpected WHO_AM_I value");
  }

  delay(250);

  Serial.println("Testing direct IMU data read...");

  const uint32_t test_start_ms = millis();

  while ((millis() - test_start_ms) < 2000) {
    imu.getAGMT();

    if (imu.status == ICM_20948_Stat_Ok) {
      Serial.println("First ICM-20948 sample received");

      Serial.printf(
          "Accel: X=%.2f, Y=%.2f, Z=%.2f mg\n",
          imu.accX(), imu.accY(), imu.accZ());

      Serial.printf(
          "Gyro: X=%.2f, Y=%.2f, Z=%.2f dps\n",
          imu.gyrX(), imu.gyrY(), imu.gyrZ());

      Serial.printf(
          "Mag: X=%.2f, Y=%.2f, Z=%.2f uT\n",
          imu.magX(), imu.magY(), imu.magZ());

      Serial.printf("Temperature: %.2f C\n", imu.temp());

      Serial.println("ICM-20948 ready");
      return true;
    }

    Serial.print("Direct IMU read failed: ");
    Serial.println(imu.statusString());

    delay(100);
  }

  Serial.println("ICM-20948 initialized but produced no valid data");
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
    Serial.println("RTC lost power; setting compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  const DateTime now = rtc.now();
  log_start_dt = now;

  Serial.print("RTC time: ");
  Serial.println(now.timestamp());

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

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, SD_SPI_FREQUENCY_HZ)) {
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
      "temperature_C,"
      "sync_pulse\n";

  constexpr size_t HEADER_LENGTH = sizeof(CSV_HEADER) - 1;

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