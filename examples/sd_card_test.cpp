#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Existing custom SD wiring
constexpr uint8_t SD_MOSI = D0;
constexpr uint8_t SD_MISO = D1;
constexpr uint8_t SD_SCK  = D2;
constexpr uint8_t SD_CS   = D3;

constexpr uint32_t SD_SPI_FREQUENCY_HZ = 1000000;
constexpr char TEST_FILE[] = "/sd_test.txt";

void print_card_information() {
  const uint8_t card_type = SD.cardType();

  Serial.print("Card type: ");

  switch (card_type) {
    case CARD_MMC:
      Serial.println("MMC");
      break;

    case CARD_SD:
      Serial.println("SDSC");
      break;

    case CARD_SDHC:
      Serial.println("SDHC/SDXC");
      break;

    case CARD_NONE:
      Serial.println("None");
      break;

    default:
      Serial.println("Unknown");
      break;
  }

  const uint64_t card_size_mb =
      SD.cardSize() / (1024ULL * 1024ULL);

  const uint64_t total_mb =
      SD.totalBytes() / (1024ULL * 1024ULL);

  const uint64_t used_mb =
      SD.usedBytes() / (1024ULL * 1024ULL);

  Serial.printf(
      "Card size: %llu MB\n",
      static_cast<unsigned long long>(card_size_mb));

  Serial.printf(
      "Filesystem size: %llu MB\n",
      static_cast<unsigned long long>(total_mb));

  Serial.printf(
      "Used space: %llu MB\n",
      static_cast<unsigned long long>(used_mb));
}

bool write_test_file() {
  Serial.printf(
      "Opening %s for writing...\n",
      TEST_FILE);

  File file = SD.open(TEST_FILE, FILE_WRITE);

  if (!file) {
    Serial.println(
        "Failed to open test file for writing");

    return false;
  }

  file.println("SheepSense SD card test");

  file.printf(
      "Recorded at millis(): %lu\n",
      static_cast<unsigned long>(millis()));

  file.flush();
  file.close();

  Serial.println("Test data written");
  return true;
}

bool read_test_file() {
  Serial.printf(
      "Opening %s for reading...\n",
      TEST_FILE);

  File file = SD.open(TEST_FILE, FILE_READ);

  if (!file) {
    Serial.println(
        "Failed to open test file for reading");

    return false;
  }

  Serial.println("File contents:");
  Serial.println("------------------------------");

  while (file.available()) {
    Serial.write(file.read());
  }

  Serial.println("------------------------------");

  file.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("SheepSense SD card SPI test");

  Serial.printf(
      "MOSI=D0/GPIO%d, MISO=D1/GPIO%d, "
      "SCK=D2/GPIO%d, CS=D3/GPIO%d\n",
      static_cast<int>(SD_MOSI),
      static_cast<int>(SD_MISO),
      static_cast<int>(SD_SCK),
      static_cast<int>(SD_CS));

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  SPI.begin(
      SD_SCK,
      SD_MISO,
      SD_MOSI,
      SD_CS);

  Serial.printf(
      "Attempting SD initialisation at %lu Hz...\n",
      static_cast<unsigned long>(
          SD_SPI_FREQUENCY_HZ));

  // Start at a conservative SPI frequency for troubleshooting.
  if (!SD.begin(
          SD_CS,
          SPI,
          SD_SPI_FREQUENCY_HZ)) {

    Serial.println("SD card initialisation failed");
    Serial.println();
    Serial.println("Check:");
    Serial.println("1. MOSI and MISO are not swapped");
    Serial.println("2. CS is connected to D3");
    Serial.println("3. All grounds are connected");
    Serial.println("4. The card is inserted");
    Serial.println("5. The card is FAT32 formatted");

    return;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card detected");
    return;
  }

  Serial.println("SD card initialisation successful");

  print_card_information();

  if (!write_test_file()) {
    return;
  }

  if (!read_test_file()) {
    return;
  }

  Serial.println("SD read/write test passed");
}

void loop() {
  // Nothing to do here.
}
