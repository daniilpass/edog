#pragma once

#include <ArduinoJson.h>

void camera_begin();
void camera_describe(JsonObject out);
void camera_state(JsonObject out);
void camera_take(uint32_t id);   // забрать видео этому зрителю
int camera_clients();            // сколько зрителей смотрит видео

// Последний кадр JPEG новее after (номер кадра): копия в buf (ёмкость cap), время съёмки, с.
// 0 — нового кадра нет за wait_ms; иначе номер кадра.
uint32_t camera_latest(uint8_t *buf, size_t cap, size_t &len, double &t, uint32_t after, uint32_t wait_ms);
