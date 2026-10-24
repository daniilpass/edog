/* Проверка XIAO ESP32-S3 Sense: только API — состояние платы, видео камеры, звук микрофона.
   Страница проверки на ПК: python xiao/ui/serve.py → http://localhost:8001/?board=edog-xiao.local

   Wi-Fi: если сеть не сохранена или к ней не подключиться за 15 с — плата поднимает
   точку доступа EDOG-XIAO (пароль edog1234, адрес 192.168.4.1); сеть задаётся со страницы
   (POST /api/wifi) или командой в Serial. Подключившись, плата доступна как edog-xiao.local (IP печатается в порт).

   Порт 80: GET /api/status, /api/control?framesize=&vflip=&hmirror=, /api/scan; POST /api/wifi (ssid, pass).
   Порт 81: /stream — видео MJPEG.  Порт 82: /audio — сырой PCM 16 кГц 16 бит моно.
   Команды в Serial (115200): wifi <ssid> <пароль>   forget   status

   Сборка: arduino-cli compile -b esp32:esp32:XIAO_ESP32S3:PSRAM=opi xiao/firmware/xiao_test
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <ESP_I2S.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "freertos/stream_buffer.h"

static const char *HOSTNAME = "edog-xiao";
static const char *AP_SSID = "EDOG-XIAO";
static const char *AP_PASS = "edog1234";

// Камера Sense (OV2640 / OV3660)
#define CAM_XCLK 10
#define CAM_SIOD 40
#define CAM_SIOC 39
#define CAM_Y9 48
#define CAM_Y8 11
#define CAM_Y7 12
#define CAM_Y6 14
#define CAM_Y5 16
#define CAM_Y4 18
#define CAM_Y3 17
#define CAM_Y2 15
#define CAM_VSYNC 38
#define CAM_HREF 47
#define CAM_PCLK 13

// PDM-микрофон и microSD на плате Sense
#define MIC_CLK 42
#define MIC_DATA 41
#define SD_CS 21
#define SD_SCK 7
#define SD_MISO 8
#define SD_MOSI 9

#define MIC_RATE 16000

Preferences prefs;
I2SClass i2s;
StreamBufferHandle_t audioBuf;

bool apMode = false;
String camStatus = "не проверена", sdStatus = "не проверена", micStatus = "не проверен";
volatile uint32_t frames = 0;
volatile bool audioClient = false;
volatile int micPeak = 0;  // пик за последние ~0.25 с, 0..32767

// ---------- проверки железа ----------

void initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2; c.pin_d1 = CAM_Y3; c.pin_d2 = CAM_Y4; c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6; c.pin_d5 = CAM_Y7; c.pin_d6 = CAM_Y8; c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD; c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = -1; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    camStatus = "ОШИБКА инициализации 0x" + String(err, HEX);
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  switch (s->id.PID) {
    case OV2640_PID: camStatus = "OK, OV2640"; break;
    case OV3660_PID: camStatus = "OK, OV3660"; break;
    case OV5640_PID: camStatus = "OK, OV5640"; break;
    default: camStatus = "OK, сенсор 0x" + String(s->id.PID, HEX);
  }
}

void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    sdStatus = "нет карты (или не читается)";
    return;
  }
  uint64_t mb = SD.cardSize() / (1024 * 1024);
  File f = SD.open("/edog_test.txt", FILE_WRITE);
  bool ok = f && f.print("edog");
  if (f) f.close();
  f = SD.open("/edog_test.txt");
  ok = ok && f && f.readString() == "edog";
  if (f) f.close();
  SD.remove("/edog_test.txt");
  sdStatus = String(ok ? "OK, " : "ОШИБКА записи, ") + String((uint32_t)mb) + " МБ";
}

void micTask(void *) {
  int16_t s[256];
  float dc = 0;
  int peak = 0, blocks = 0;
  for (;;) {
    size_t n = i2s.readBytes((char *)s, sizeof s) / 2;
    if (!n) { vTaskDelay(1); continue; }
    for (size_t i = 0; i < n; i++) {
      dc += (s[i] - dc) * 0.002f;  // убираем постоянную составляющую
      int y = constrain((int)(s[i] - dc), -32768, 32767);
      s[i] = y;
      if (abs(y) > peak) peak = abs(y);
    }
    if (++blocks >= MIC_RATE / 4 / 256) { micPeak = peak; peak = 0; blocks = 0; }
    if (audioClient) xStreamBufferSend(audioBuf, s, n * 2, 0);
  }
}

void initMic() {
  i2s.setPinsPdmRx(MIC_CLK, MIC_DATA);
  if (!i2s.begin(I2S_MODE_PDM_RX, MIC_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    micStatus = "ОШИБКА инициализации";
    return;
  }
  micStatus = "OK";
  audioBuf = xStreamBufferCreate(16 * 1024, 1);
  xTaskCreatePinnedToCore(micTask, "mic", 4096, nullptr, 5, nullptr, 0);
}

// ---------- Wi-Fi ----------

void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP_STA);  // STA нужен для сканирования сетей
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Точка доступа %s, пароль %s, адрес %s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

void initWiFi() {
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", ""), pass = prefs.getString("pass", "");
  prefs.end();

  if (ssid.isEmpty()) {
    Serial.println("Wi-Fi сеть не сохранена");
    startAP();
  } else {
    Serial.printf("Подключаюсь к %s", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Подключено: %s (%s.local), RSSI %d dBm\n",
                    WiFi.localIP().toString().c_str(), HOSTNAME, WiFi.RSSI());
    } else {
      Serial.println("Не подключилось");
      WiFi.disconnect();
      startAP();
    }
  }
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

void saveWiFi(const String &ssid, const String &pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

// ---------- HTTP ----------

String jsonStr(const String &s) {
  String r = "\"";
  for (char ch : s) {
    if (ch == '"' || ch == '\\') { r += '\\'; r += ch; }
    else if ((uint8_t)ch < 0x20) r += ' ';
    else r += ch;
  }
  return r + "\"";
}

String urlDecode(const char *s) {
  String r;
  for (; *s; s++) {
    if (*s == '+') r += ' ';
    else if (*s == '%' && s[1] && s[2]) {
      char hex[3] = {s[1], s[2], 0};
      r += (char)strtol(hex, nullptr, 16);
      s += 2;
    } else r += *s;
  }
  return r;
}

String formValue(const char *body, const char *key) {
  char v[200];
  if (httpd_query_key_value(body, key, v, sizeof v) != ESP_OK) return "";
  return urlDecode(v);
}

// Страница открыта с ПК (другой origin) — CORS открыт для всех.
esp_err_t sendJson(httpd_req_t *req, const String &json) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json.c_str(), json.length());
}

esp_err_t statusHandler(httpd_req_t *req) {
  static uint32_t lastFrames = 0, lastMs = 0;
  uint32_t now = millis();
  float fps = lastMs ? (frames - lastFrames) * 1000.0f / (now - lastMs) : 0;
  lastFrames = frames; lastMs = now;

  sensor_t *s = esp_camera_sensor_get();
  String j = "{";
  j += "\"mode\":" + jsonStr(apMode ? "точка доступа " + String(AP_SSID) : "клиент");
  j += ",\"ssid\":" + jsonStr(apMode ? "" : WiFi.SSID());
  j += ",\"ip\":" + jsonStr((apMode ? WiFi.softAPIP() : WiFi.localIP()).toString());
  j += ",\"rssi\":" + String(apMode ? 0 : WiFi.RSSI());
  j += ",\"mac\":" + jsonStr(WiFi.macAddress());
  j += ",\"psram\":" + String(ESP.getPsramSize() / 1024);
  j += ",\"psram_free\":" + String(ESP.getFreePsram() / 1024);
  j += ",\"heap_free\":" + String(ESP.getFreeHeap() / 1024);
  j += ",\"temp\":" + String(temperatureRead(), 1);
  j += ",\"uptime\":" + String(now / 1000);
  j += ",\"cam\":" + jsonStr(camStatus);
  j += ",\"fps\":" + String(fps, 1);
  j += ",\"framesize\":" + String(s ? s->status.framesize : 0);
  j += ",\"vflip\":" + String(s ? s->status.vflip : 0);
  j += ",\"hmirror\":" + String(s ? s->status.hmirror : 0);
  j += ",\"mic\":" + jsonStr(micStatus);
  j += ",\"mic_peak\":" + String(micPeak);
  j += ",\"sd\":" + jsonStr(sdStatus);
  j += "}";
  return sendJson(req, j);
}

esp_err_t controlHandler(httpd_req_t *req) {
  char q[100], v[16];
  sensor_t *s = esp_camera_sensor_get();
  if (!s || httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad request");
    return ESP_FAIL;
  }
  if (httpd_query_key_value(q, "framesize", v, sizeof v) == ESP_OK) s->set_framesize(s, (framesize_t)atoi(v));
  if (httpd_query_key_value(q, "vflip", v, sizeof v) == ESP_OK) s->set_vflip(s, atoi(v));
  if (httpd_query_key_value(q, "hmirror", v, sizeof v) == ESP_OK) s->set_hmirror(s, atoi(v));
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t scanHandler(httpd_req_t *req) {
  int n = WiFi.scanNetworks();
  String j = "[";
  for (int i = 0; i < n; i++) {
    if (i) j += ",";
    j += "{\"ssid\":" + jsonStr(WiFi.SSID(i)) + ",\"rssi\":" + String(WiFi.RSSI(i)) +
         ",\"open\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "true" : "false") + "}";
  }
  WiFi.scanDelete();
  return sendJson(req, j + "]");
}

esp_err_t wifiHandler(httpd_req_t *req) {
  char body[300] = {};
  int len = httpd_req_recv(req, body, sizeof body - 1);
  if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty"); return ESP_FAIL; }
  String ssid = formValue(body, "ssid"), pass = formValue(body, "pass");
  if (ssid.isEmpty()) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no ssid"); return ESP_FAIL; }
  saveWiFi(ssid, pass);
  sendJson(req, "{\"ok\":true}");
  Serial.printf("Сохранена сеть %s, перезагрузка\n", ssid.c_str());
  delay(500);
  ESP.restart();
  return ESP_OK;
}

esp_err_t streamHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[80];
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    int hl = snprintf(part, sizeof part,
                      "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, part, hl);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (r != ESP_OK) return ESP_OK;  // клиент закрыл страницу
    frames++;
  }
}

esp_err_t audioHandler(httpd_req_t *req) {
  if (!audioBuf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "mic"); return ESP_FAIL; }
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  xStreamBufferReset(audioBuf);
  audioClient = true;
  static uint8_t buf[1024];
  for (;;) {
    size_t n = xStreamBufferReceive(audioBuf, buf, sizeof buf, pdMS_TO_TICKS(500));
    if (n && httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) break;
  }
  audioClient = false;
  return ESP_OK;
}

// Каждый поток (видео, звук) держит обработчик сервера, поэтому у каждого свой сервер.
httpd_handle_t startServer(uint16_t port, std::initializer_list<std::pair<const char *, esp_err_t (*)(httpd_req_t *)>> gets,
                           std::initializer_list<std::pair<const char *, esp_err_t (*)(httpd_req_t *)>> posts = {}) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = port;
  cfg.ctrl_port = 32768 + port;
  cfg.stack_size = 8192;
  httpd_handle_t h = nullptr;
  if (httpd_start(&h, &cfg) != ESP_OK) {
    Serial.printf("Не запустился сервер на порту %u\n", port);
    return nullptr;
  }
  for (auto &g : gets) {
    httpd_uri_t u = {};
    u.uri = g.first; u.method = HTTP_GET; u.handler = g.second;
    httpd_register_uri_handler(h, &u);
  }
  for (auto &p : posts) {
    httpd_uri_t u = {};
    u.uri = p.first; u.method = HTTP_POST; u.handler = p.second;
    httpd_register_uri_handler(h, &u);
  }
  return h;
}

// ---------- Serial ----------

void printStatus() {
  Serial.printf("PSRAM %u КБ, камера: %s, микрофон: %s, SD: %s\n",
                ESP.getPsramSize() / 1024, camStatus.c_str(), micStatus.c_str(), sdStatus.c_str());
  if (apMode) Serial.printf("Wi-Fi: точка доступа %s, адрес %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  else Serial.printf("Wi-Fi: %s, адрес %s, RSSI %d dBm\n", WiFi.SSID().c_str(),
                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.startsWith("wifi ")) {
    String rest = line.substring(5);
    int sp = rest.lastIndexOf(' ');
    String ssid = sp > 0 ? rest.substring(0, sp) : rest, pass = sp > 0 ? rest.substring(sp + 1) : "";
    saveWiFi(ssid, pass);
    Serial.printf("Сохранена сеть %s, перезагрузка\n", ssid.c_str());
    delay(200);
    ESP.restart();
  } else if (line == "forget") {
    saveWiFi("", "");
    Serial.println("Сеть забыта, перезагрузка");
    delay(200);
    ESP.restart();
  } else if (line == "status") {
    printStatus();
  } else if (line.length()) {
    Serial.println("Команды: wifi <ssid> <пароль> | forget | status");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // успеть открыть монитор порта
  Serial.println("\nEDOG XIAO test");

  initCamera();
  initMic();
  initSD();
  initWiFi();

  startServer(80, {{"/api/status", statusHandler}, {"/api/control", controlHandler}, {"/api/scan", scanHandler}},
              {{"/api/wifi", wifiHandler}});
  startServer(81, {{"/stream", streamHandler}});
  startServer(82, {{"/audio", audioHandler}});
  printStatus();
}

void loop() {
  handleSerial();
  delay(20);
}
