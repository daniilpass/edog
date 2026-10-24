// Автономный патруль: собака ходит, ищет новые места и людей, встретив человека — подходит и
// приветствует, потом идёт дальше. Решения — по зрению (vision.h) и локальной карте вокруг собаки.
// Всё состояние патруля — под RobotLock (команды приходят из HTTP, геймпада и задачи патруля).
#pragma once

#include <ArduinoJson.h>

#include "robot.h"

void patrol_begin(Robot &r);
void patrol_autostart();               // после включения питания: серво → 5 с → кивок → патруль
bool patrol_active();                  // патруль включён (в любом состоянии, кроме «выключен»); без замка
void patrol_start();                   // под RobotLock; ValueError — не включить
void patrol_stop(const char *reason);  // под RobotLock
void patrol_toggle();                  // под RobotLock (кнопка геймпада)
void patrol_user_activity();           // человек командует из портала: пауза
void patrol_set_autostart(bool on);
void patrol_state(JsonObject out);     // под RobotLock
int patrol_headers(char *buf, size_t cap);   // строки заголовков кадра видео: состояние и карта
