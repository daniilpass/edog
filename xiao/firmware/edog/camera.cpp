#include "camera.h"

#include <Arduino.h>

#include <atomic>

#include "config.h"
#include "esp_camera.h"
#include "esp_http_server.h"

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

static String sensor, error;
static std::atomic<uint32_t> frames{0};
static std::atomic<int> clients{0};
// Поток один: сервер видео отдаёт его в одной задаче. Зритель — id из /stream?id=; другое устройство
// забирает поток через camera_take, текущий цикл видит чужой id и закрывает соединение.
static std::atomic<uint32_t> viewer{0};
static std::atomic<uint32_t> taken_ms{0};
static const uint32_t TAKE_HOLD_MS = 5000;   // столько поток держится за забравшим, пока тот подключается

// Кто смотрит или вот-вот подключится; 0 — поток свободен.
static uint32_t current_viewer() {
  if (clients == 0 && millis() - taken_ms > TAKE_HOLD_MS) return 0;
  return viewer;
}

void camera_take(uint32_t id) {
  taken_ms = millis();
  viewer = id;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  char query[64], value[16];
  uint32_t id = 0;   // без id (VLC и т. п.) — безымянный зритель, его тоже можно сменить
  if (httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK &&
      httpd_query_key_value(query, "id", value, sizeof value) == ESP_OK)
    id = strtoul(value, nullptr, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  uint32_t v = current_viewer();
  if (v && v != id) {   // поток только что забрало другое устройство
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "video taken by another viewer");
  }
  viewer = id;

  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const uint32_t period = 1000 / CAMERA_FPS;
  char part[80];
  clients++;
  uint32_t next = millis();
  while (viewer == id) {
    int32_t wait = (int32_t)(next - millis());
    if (wait > 0) delay(wait);
    next += period;
    if ((int32_t)(millis() - next) > (int32_t)period) next = millis();   // не догонять пропущенные кадры
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) break;
    int hl = snprintf(part, sizeof part, "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, part, hl);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (r != ESP_OK) break;   // портал закрыт
    frames++;
  }
  clients--;
  if (viewer != id) {   // забрали: закрыть соединение сразу, а не ждать таймаута у прежнего зрителя
    httpd_resp_send_chunk(req, nullptr, 0);
    httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
  }
  viewer.compare_exchange_strong(id, 0);   // ушёл сам — поток свободен; забрали — id уже чужой
  return ESP_OK;
}

void camera_begin() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_Y2; c.pin_d1 = CAM_Y3; c.pin_d2 = CAM_Y4; c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6; c.pin_d5 = CAM_Y7; c.pin_d6 = CAM_Y8; c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD; c.pin_sccb_scl = CAM_SIOC;
  c.sccb_i2c_port = 1;   // порт 0 — Wire, PCA9685
  c.pin_pwdn = -1; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = CAMERA_FRAMESIZE;
  c.jpeg_quality = CAMERA_QUALITY;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;   // свежий кадр, а не из очереди

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    error = "ошибка инициализации 0x" + String(err, HEX);
    Serial.printf("Камера: %s\n", error.c_str());
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, CAMERA_VFLIP);
  s->set_hmirror(s, CAMERA_HMIRROR);
  switch (s->id.PID) {
    case OV2640_PID: sensor = "OV2640"; break;
    case OV3660_PID: sensor = "OV3660"; break;
    case OV5640_PID: sensor = "OV5640"; break;
    default: sensor = "0x" + String(s->id.PID, HEX);
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = STREAM_PORT;
  cfg.ctrl_port = 32768 + STREAM_PORT;
  cfg.max_open_sockets = 3;
  cfg.lru_purge_enable = true;
  httpd_handle_t h = nullptr;
  if (httpd_start(&h, &cfg) != ESP_OK) {
    error = "не запустился сервер видео";
    Serial.printf("Камера: %s\n", error.c_str());
    return;
  }
  httpd_uri_t u = {};
  u.uri = "/stream";
  u.method = HTTP_GET;
  u.handler = stream_handler;
  httpd_register_uri_handler(h, &u);
  Serial.printf("Камера %s: http://<адрес>:%d/stream, %d кадр/с\n", sensor.c_str(), STREAM_PORT, CAMERA_FPS);
}

void camera_describe(JsonObject out) {
  out["ok"] = error.isEmpty() && !sensor.isEmpty();
  if (!error.isEmpty()) out["error"] = error;
  out["sensor"] = sensor;
  out["port"] = STREAM_PORT;
  out["path"] = "/stream";
  out["fps"] = CAMERA_FPS;
}

void camera_state(JsonObject out) {
  static uint32_t last_frames = 0, last_ms = 0;
  static float fps = 0;
  uint32_t now = millis();
  if (now - last_ms >= 1000) {
    fps = last_ms ? (frames - last_frames) * 1000.0f / (now - last_ms) : 0;
    last_frames = frames;
    last_ms = now;
  }
  out["fps"] = std::round(fps * 10) / 10.0;
  out["clients"] = clients.load();
  out["viewer"] = current_viewer();
}
