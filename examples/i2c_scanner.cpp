#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

constexpr int I2C_SDA = D4;
constexpr int I2C_SCL = D5;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\nI2C scanner");

  Serial.printf(
      "D4=%d, D5=%d\n",
      static_cast<int>(D4),
      static_cast<int>(D5));

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);

  delay(10);

  Serial.printf(
      "Before Wire.begin: SDA=%s, SCL=%s\n",
      digitalRead(I2C_SDA) ? "HIGH" : "LOW",
      digitalRead(I2C_SCL) ? "HIGH" : "LOW");

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(100);
}

void loop() {
  int devices_found = 0;

  Serial.println("Scanning...");

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);

    const uint8_t result =
        Wire.endTransmission(true);

    if (result == 0) {
      Serial.printf(
          "Device found at 0x%02X\n",
          address);

      ++devices_found;
    }
  }

  if (devices_found == 0) {
    Serial.println("No I2C devices found");
  }

  Serial.println();
  delay(3000);
}
