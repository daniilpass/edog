#include "pca9685.h"

#include <Wire.h>

#include "config.h"

static const uint8_t MODE1 = 0x00, MODE2 = 0x01, PRESCALE = 0xFE, LED0_ON_L = 0x06, ALL_LED_ON_L = 0xFA;
static const uint8_t SLEEP = 0x10, AUTO_INCREMENT = 0x20, RESTART = 0x80, OUTDRV = 0x04;
static const uint16_t FULL_OFF = 0x1000;   // бит 4 в LEDn_OFF_H

bool Driver::write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Driver::write_led(uint8_t reg, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(reg);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  uint8_t rc = Wire.endTransmission();
  if (rc) error = "I²C: ошибка записи в PCA9685 (" + String(rc) + ")";
  return rc == 0;
}

bool Driver::begin() {
  for (int &t : ticks_) t = -1;
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
  Wire.beginTransmission(PCA_ADDRESS);
  if (Wire.endTransmission() != 0) {
    error = "PCA9685: нет ответа на 0x" + String(PCA_ADDRESS, HEX) + " (SDA " + I2C_SDA + ", SCL " + I2C_SCL + ")";
    Serial.printf("%s, работаю на заглушке\n", error.c_str());
    return false;
  }
  int prescale = (int)lround(PCA_OSCILLATOR_HZ / (4096 * PCA_FREQUENCY_HZ)) - 1;
  frequency_hz_ = PCA_OSCILLATOR_HZ / (4096.0 * (prescale + 1));
  write_reg(MODE1, SLEEP);
  write_reg(PRESCALE, prescale);
  write_reg(MODE2, OUTDRV);
  write_reg(MODE1, AUTO_INCREMENT);
  delay(5);
  write_reg(MODE1, AUTO_INCREMENT | RESTART);
  mock_ = false;
  all_off();
  Serial.printf("PCA9685 0x%02x, %.1f Гц\n", PCA_ADDRESS, frequency_hz_);
  return true;
}

void Driver::set_pulse(int channel, double us) {
  if (mock_) return;
  int ticks = (int)lround(us * frequency_hz_ * 4096 / 1000000.0);
  ticks = ticks < 0 ? 0 : ticks > 4095 ? 4095 : ticks;
  if (ticks_[channel] == ticks) return;
  if (write_led(LED0_ON_L + 4 * channel, 0, ticks)) ticks_[channel] = ticks;
}

void Driver::off(int channel) {
  if (mock_) return;
  write_led(LED0_ON_L + 4 * channel, 0, FULL_OFF);
  ticks_[channel] = -1;
}

void Driver::all_off() {
  if (mock_) return;
  write_led(ALL_LED_ON_L, 0, FULL_OFF);
  for (int &t : ticks_) t = -1;
}
