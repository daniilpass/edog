// Драйвер PCA9685 (I²C). Если платы нет на шине — работает заглушкой, чтобы портал всё равно работал.
#pragma once

#include <Arduino.h>

class Driver {
 public:
  // false — PCA9685 не ответила, дальше заглушка; error — текст для портала.
  bool begin();
  void set_pulse(int channel, double us);
  void off(int channel);
  void all_off();
  const char *kind() const { return mock_ ? "mock" : "pca9685"; }
  String error;   // пусто — нет ошибки

 private:
  bool mock_ = true;
  double frequency_hz_ = 50;
  int ticks_[16];
  bool write_reg(uint8_t reg, uint8_t value);
  bool write_led(uint8_t reg, uint16_t on, uint16_t off);
};
