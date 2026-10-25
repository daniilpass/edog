#include "sleep.h"

#include <WiFi.h>

#include <atomic>

#include "camera.h"
#include "config.h"
#include "patrol.h"

static Robot *robot;
static std::atomic<bool> asleep{false};
static uint32_t awake_mhz = 240;
static double last_activity = 0;   // now_s() последней команды человека или движения
static uint32_t seen_presses = 0;
static std::string reason;

bool sleep_active() { return asleep; }

void sleep_activity() { last_activity = now_s(); }

void sleep_enter(const char *why) {
  if (asleep) return;
  if (robot->updating) throw ValueError("идёт обновление прошивки");
  if (patrol_active()) patrol_stop("сон");
  // Выключение плавное (через монтажное положение); повторный вызов снял бы сигнал сразу.
  if (robot->enabled && !robot->powering_down()) robot->set_enabled(false);
  camera_sleep(true);
  WiFi.setSleep(true);
  setCpuFrequencyMhz(SLEEP_CPU_MHZ);
  asleep = true;
  reason = why;
  Serial.printf("сон: %s\n", why);
}

void sleep_wake(const char *why, bool stand) {
  if (!asleep) return;
  setCpuFrequencyMhz(awake_mhz);
  WiFi.setSleep(false);   // иначе видео и WebSocket идут рывками
  camera_sleep(false);
  asleep = false;
  reason.clear();
  last_activity = now_s();
  Serial.printf("пробуждение: %s\n", why);
  if (stand) robot->set_enabled(true);
}

static void sleep_task(void *) {
  for (;;) {
    delay(1000);
    RobotLock lock;
    Robot &r = *robot;
    bool pad = r.pad_drive[0] || r.pad_drive[1] || r.tilt[0] || r.tilt[1] || r.height_rate || r.pad_presses != seen_presses;
    seen_presses = r.pad_presses;
    double now = now_s();
    if (pad || patrol_active() || r.updating || std::isnan(robot_still_since())) last_activity = now;
    if (asleep || now - last_activity < SLEEP_AFTER_S) continue;
    try {
      sleep_enter("автосон");
    } catch (const std::exception &e) {
      Serial.printf("автосон: %s\n", e.what());
      last_activity = now;
    }
  }
}

void sleep_begin(Robot &r) {
  robot = &r;
  awake_mhz = getCpuFrequencyMhz();
  last_activity = now_s();
  xTaskCreatePinnedToCoreWithCaps(sleep_task, "sleep", 4096, nullptr, 1, nullptr, 1, MALLOC_CAP_SPIRAM);
}

void sleep_state(JsonObject out) {
  out["active"] = asleep.load();
  out["after_s"] = SLEEP_AFTER_S;
  if (asleep) out["reason"] = reason;
  else out["idle_s"] = (int)(now_s() - last_activity);
}
