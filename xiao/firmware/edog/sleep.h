// Сон: серво выключены, патруль остановлен, камера в режиме ожидания, Wi-Fi экономит (modem sleep),
// процессор на SLEEP_CPU_MHZ. Wi-Fi и Bluetooth остаются: будят из портала, приложения (POST /api/sleep,
// включение серво, запуск патруля) или любой кнопкой геймпада. Автосон — через SLEEP_AFTER_S без команд
// человека, движения и патруля.
#pragma once

#include <ArduinoJson.h>

#include "robot.h"

void sleep_begin(Robot &r);
bool sleep_active();                                    // без замка
void sleep_enter(const char *why);                      // под RobotLock
void sleep_wake(const char *why, bool stand = true);    // под RobotLock; stand — встать в стойку
void sleep_activity();                                  // под RobotLock: команда человека, отсчёт автосна заново
void sleep_state(JsonObject out);                       // под RobotLock
