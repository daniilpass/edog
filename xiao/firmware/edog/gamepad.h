// BLE-геймпад (HID over GATT) и его сопряжение — замена gamepad.py (evdev) и bluetooth.py (bluetoothctl).
//
// Все операции BLE делает одна задача: в простое она ищет сопряжённый геймпад с малой долей эфира
// (Wi-Fi и видео делят с BLE одно радио) и подключается к нему; запросы портала (поиск, сопряжение,
// забыть) ставятся ей в очередь, обработчик HTTP ждёт ответа.
#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "robot.h"

struct BluetoothError : std::runtime_error { using std::runtime_error::runtime_error; };

struct BtDevice {
  std::string mac, name;
  bool paired, connected;
};

void gamepad_begin(Robot &robot);

// Блокирующие вызовы для API; бросают BluetoothError.
std::vector<BtDevice> bt_scan(int seconds);
std::string bt_pair(const std::string &mac);
std::string bt_remove(const std::string &mac);
