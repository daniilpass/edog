// Камера платы Sense: видео MJPEG на STREAM_PORT, /stream. Свой сервер: поток держит обработчик.
#pragma once

#include <ArduinoJson.h>

void camera_begin();
// Для /api/config и состояния: есть ли камера, сенсор, порт и путь потока, кадры в секунду.
void camera_describe(JsonObject out);
void camera_state(JsonObject out);
// Отдать поток зрителю id (тот же id он передаёт в /stream?id=): текущий зритель отключается.
void camera_take(uint32_t id);
