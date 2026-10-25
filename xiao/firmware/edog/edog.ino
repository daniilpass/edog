/* Прошивка EDOG для Seeed XIAO ESP32-S3 Sense — замена сервиса на Raspberry Pi (rpi/firmware).

   На плате только API, портал с 3D-моделью запускается на ПК (ui/serve.py):
     порт 8080 — REST /api/… и WebSocket /ws, те же, что на Pi;
     порт 8081 — /stream, видео MJPEG с камеры.
   Серво — через PCA9685 по I²C (SDA D4, SCL D5), геймпад — BLE (HID), сопряжение из портала.
   После включения питания собака встаёт в базовую стойку, после прошивки по воздуху — возвращается
   в состояние до неё (серво, стойка, патруль). Патруль по камере (patrol.cpp, vision.cpp)
   включается из портала, приложения или геймпада: собака ищет новые места и людей, встретив человека,
   приветствует.
   Сон (sleep.cpp): кнопкой из портала или приложения и сам через минуту бездействия — серво, камера
   и патруль выключены, Wi-Fi экономит; будят из портала, приложения или кнопкой геймпада.

   Wi-Fi: если сеть не сохранена или к ней не подключиться за 15 с — плата поднимает точку доступа
   EDOG-XIAO (пароль edog1234, адрес 192.168.4.1). Подключившись, плата доступна как edog.local.
   Команды в Serial (115200): wifi <ssid> <пароль>   forget   status

   Сборка: arduino-cli compile -b esp32:esp32:XIAO_ESP32S3:PSRAM=opi xiao/firmware/edog
*/

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "api.h"
#include "camera.h"
#include "config.h"
#include "gamepad.h"
#include "patrol.h"
#include "pca9685.h"
#include "robot.h"
#include "sleep.h"
#include "vision.h"

static Driver driver;
static Robot *robot;
static bool ap_mode = false;
static bool api_ok = false;

// ---------- откат прошивки ----------
// Новая прошивка после записи по воздуху — «на проверке». Рабочей она считается, только когда API
// поднялось и проработало VALID_AFTER_MS: иначе её уже не заменить по воздуху. Не дожила за
// ROLLBACK_AFTER_MS (зависла в setup, падает по кругу) — перезагрузка, загрузчик возвращает прежнюю.

static const uint32_t VALID_AFTER_MS = 20000, ROLLBACK_AFTER_MS = 90000;
static bool on_probation = false;

// Arduino не помечает прошивку рабочей сам. extern "C": ядро объявляет слабую функцию в esp32-hal-misc.c.
extern "C" bool verifyRollbackLater() { return true; }

static void rollback_timer(void *) {
  if (!on_probation) return;
  Serial.println("Прошивка не дожила до проверки — откат на прежнюю");
  esp_ota_mark_app_invalid_rollback_and_reboot();
  esp_restart();   // откатывать некуда (прошита по USB) — хотя бы перезагрузиться
}

static void start_probation() {
  esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
  esp_err_t err = esp_ota_get_state_partition(esp_ota_get_running_partition(), &st);
  Serial.printf("Прошивка: слот %s, состояние %d (ошибка %d)\n", esp_ota_get_running_partition()->label, (int)st, (int)err);
  on_probation = err == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
  if (!on_probation) return;
  Serial.println("Прошивка на проверке: рабочей станет, когда поднимется API");
  esp_timer_create_args_t a = {};
  a.callback = rollback_timer;
  a.name = "rollback";
  esp_timer_handle_t t;
  esp_timer_create(&a, &t);
  esp_timer_start_once(t, (uint64_t)ROLLBACK_AFTER_MS * 1000);
}

static void check_probation() {
  if (!on_probation || !api_ok || millis() < VALID_AFTER_MS) return;
  on_probation = false;
  esp_ota_mark_app_valid_cancel_rollback();
  Serial.println("Прошивка проверена: API работает");
}

// ---------- контур ----------

static void loop_task(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
  TickType_t wake = xTaskGetTickCount();
  double last = now_s();
  for (;;) {
    vTaskDelayUntil(&wake, period);
    double now = now_s();
    try {
      RobotLock lock;
      robot->step(std::min(now - last, 0.1));
    } catch (const std::exception &e) {   // контур не должен падать
      Serial.printf("контур: %s\n", e.what());
    }
    last = now;
  }
}

// ---------- Wi-Fi ----------

static void start_ap() {
  ap_mode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Точка доступа %s, пароль %s, адрес %s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static void init_wifi() {
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);   // иначе видео и WebSocket идут рывками
  Preferences prefs;
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", ""), pass = prefs.getString("pass", "");
  prefs.end();

  if (ssid.isEmpty()) {
    Serial.println("Wi-Fi сеть не сохранена");
    start_ap();
  } else {
    Serial.printf("Подключаюсь к %s", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Подключено: %s (%s.local), RSSI %d dBm\n", WiFi.localIP().toString().c_str(), HOSTNAME, WiFi.RSSI());
    } else {
      Serial.println("Не подключилось");
      WiFi.disconnect();
      start_ap();
    }
  }
  WiFi.setAutoReconnect(true);
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", API_PORT);
}

static void save_wifi(const String &ssid, const String &pass) {
  Preferences prefs;
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

// ---------- Serial ----------

static void print_status() {
  if (ap_mode) Serial.printf("Wi-Fi: точка доступа %s, адрес %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  else Serial.printf("Wi-Fi: %s, адрес %s, RSSI %d dBm\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  Serial.printf("Портал: python ui/serve.py → http://localhost:8000/?dog=%s:%d\n",
                (ap_mode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), API_PORT);
  Serial.printf("Плата: %s%s%s; свободно: heap %u КБ, PSRAM %u КБ\n", driver.kind(),
                driver.error.isEmpty() ? "" : " — ", driver.error.c_str(),
                ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
}

static void handle_serial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("wifi ")) {
    String rest = line.substring(5);
    int sp = rest.lastIndexOf(' ');
    String ssid = sp > 0 ? rest.substring(0, sp) : rest, pass = sp > 0 ? rest.substring(sp + 1) : "";
    save_wifi(ssid, pass);
    Serial.printf("Сохранена сеть %s, перезагрузка\n", ssid.c_str());
    delay(200);
    ESP.restart();
  } else if (line == "forget") {
    save_wifi("", "");
    Serial.println("Сеть забыта, перезагрузка");
    delay(200);
    ESP.restart();
  } else if (line == "status") {
    print_status();
  } else if (line.length()) {
    Serial.println("Команды: wifi <ssid> <пароль> | forget | status");
  }
}

// После включения питания: базовая стойка, какой бы профиль ни был сохранён.
static void stand_up() {
  RobotLock lock;
  try {
    robot->set_gait("base");
    robot->set_enabled(true);
  } catch (const std::exception &ex) {
    Serial.printf("стойка после включения не удалась (%s)\n", ex.what());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);   // успеть открыть монитор порта
  Serial.println("\nEDOG XIAO");
  start_probation();

  robot_lock_init();
  driver.begin();
  robot = new Robot(driver);
  xTaskCreatePinnedToCore(loop_task, "robot", 8192, nullptr, 5, nullptr, 1);

  init_wifi();
  vision_begin();   // лёгкая часть: буферы и задача; модель людей грузится уже в задаче
  // API — сразу после сети: через него прошивка по воздуху, оно должно жить, даже если дальше что-то сломалось
  api_ok = api_begin(*robot);
  camera_begin();   // после Wi-Fi: сервер видео не стартует без сетевого стека
  gamepad_begin(*robot);
  patrol_begin(*robot);
  sleep_begin(*robot);
  heap_caps_malloc_extmem_enable(EXTMEM_LIMIT);
  print_status();
  // Встаёт сама после включения питания. После прошивки по воздуху — возвращается, как было до неё.
  // После сбоя или просадки питания не встаёт: её могли оставить на столе.
  if (esp_reset_reason() == ESP_RST_POWERON) stand_up();
  else api_resume_after_update();
}

void loop() {
  check_probation();
  handle_serial();
  delay(20);
}
