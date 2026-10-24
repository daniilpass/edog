#include "api.h"

#include <Update.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

#include "camera.h"
#include "gamepad.h"

static const int STATE_HZ = 20;
static const size_t MAX_BODY = 4096;

static Robot *robot;
static httpd_handle_t server;
static SemaphoreHandle_t ws_send_lock;   // кадры WebSocket в один сокет шлют задача рассылки и обработчик
static String firmware_id;              // начало SHA-256 работающей прошивки: видно, встала ли новая

struct HttpError {
  int code;
  std::string detail;
};

static const char *status_text(int code) {
  switch (code) {
    case 200: return "200 OK";
    case 204: return "204 No Content";
    case 400: return "400 Bad Request";
    case 404: return "404 Not Found";
    case 409: return "409 Conflict";
    case 422: return "422 Unprocessable Entity";
    case 502: return "502 Bad Gateway";
    case 503: return "503 Service Unavailable";
  }
  return "500 Internal Server Error";
}

// Портал открыт с ПК (другой origin), поэтому API доступно с любого origin. Авторизации нет: домашняя сеть.
static void cors(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, PATCH, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
}

static esp_err_t send_json(httpd_req_t *req, int code, const String &json) {
  httpd_resp_set_status(req, status_text(code));
  httpd_resp_set_type(req, "application/json");
  cors(req);
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t send_doc(httpd_req_t *req, int code, JsonDocument &doc) {
  String json;
  serializeJson(doc, json);
  return send_json(req, code, json);
}

static esp_err_t send_error(httpd_req_t *req, const HttpError &e) {
  JsonDocument doc;
  doc["detail"] = e.detail;
  return send_doc(req, e.code, doc);
}

static String read_body(httpd_req_t *req) {
  if (req->content_len > MAX_BODY) throw HttpError{422, "слишком большое тело запроса"};
  String body;
  char buf[256];
  size_t left = req->content_len;
  while (left) {
    int n = httpd_req_recv(req, buf, std::min(left, sizeof buf));
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0) throw HttpError{400, "тело запроса не дочитано"};
    body.concat(buf, n);
    left -= n;
  }
  return body;
}

// ---------- разбор тела (как модели pydantic в api.py) ----------

static JsonDocument parse(const String &body) {
  JsonDocument doc;
  if (body.length() && deserializeJson(doc, body)) throw HttpError{422, "тело запроса — не JSON"};
  if (!doc.isNull() && !doc.is<JsonObject>()) throw HttpError{422, "тело запроса — не объект JSON"};
  return doc;
}

// Необязательное число в пределах; nullptr — не задано.
static const double *num(JsonVariantConst v, const char *name, double &store, double lo = -INFINITY,
                         double hi = INFINITY, bool lo_open = false) {
  if (v.isNull()) return nullptr;
  if (!v.is<double>()) throw HttpError{422, strf("%s: нужно число", name)};
  store = v.as<double>();
  if (store < lo || store > hi || (lo_open && store == lo))
    throw HttpError{422, strf("%s: вне %g … %g", name, lo, hi)};
  return &store;
}

static double need(JsonVariantConst v, const char *name, double lo = -INFINITY, double hi = INFINITY) {
  double x;
  if (!num(v, name, x, lo, hi)) throw HttpError{422, strf("%s: обязательное поле", name)};
  return x;
}

static std::string choice(JsonVariantConst v, const char *name, std::initializer_list<const char *> options,
                          const char *fallback = nullptr) {
  if (v.isNull() && fallback) return fallback;
  std::string s = v.is<const char *>() ? v.as<const char *>() : "";
  for (const char *o : options) if (s == o) return s;
  std::string list;
  for (const char *o : options) list += std::string(list.empty() ? "" : ", ") + o;
  throw HttpError{422, strf("%s: одно из %s", name, list.c_str())};
}

static bool starts(const std::string &s, const char *prefix, std::string *rest = nullptr) {
  size_t n = strlen(prefix);
  if (s.compare(0, n, prefix)) return false;
  if (rest) *rest = s.substr(n);
  return true;
}

// ---------- маршруты ----------

// Ответ команды — полное состояние, как servo_call в api.py.
static void state_reply(JsonDocument &out) {
  robot->state(out);
  camera_state(out["camera"].to<JsonObject>());
}

static void route(httpd_method_t method, const std::string &path, const String &body, JsonDocument &out) {
  JsonDocument req = parse(body);
  Robot &r = *robot;
  if (method == HTTP_GET && path == "/api/config") {
    // Описание не меняется, а размах суставов профилей считается заметное время — один раз и без замка.
    static JsonDocument config;
    if (config.isNull()) {
      r.describe(config);
      camera_describe(config["camera"].to<JsonObject>());
      config["firmware"] = firmware_id;
    }
    out.set(config);
    return;
  }
  RobotLock lock;
  double a, b, c;
  std::string rest;

  if (method == HTTP_GET) {
    if (path == "/") {
      out["service"] = "edog";
      out["board"] = "XIAO ESP32-S3 Sense";
      out["firmware"] = firmware_id;
      out["api"] = "/api";
      out["ws"] = "/ws";
      out["ui"] = "портал запускается на ПК: python ui/serve.py";
      return;
    }
    if (path == "/api/state") return state_reply(out);
  } else if (method == HTTP_POST) {
    if (path == "/api/enable") {
      if (!req["enabled"].is<bool>()) throw HttpError{422, "enabled: нужно true/false"};
      r.set_enabled(req["enabled"].as<bool>());
    } else if (path == "/api/estop") {
      r.estop();
    } else if (path == "/api/mode") {
      r.set_mode(choice(req["mode"], "mode", {"stand", "manual"}));
    } else if (starts(path, "/api/pose/", &rest)) {
      r.set_pose(rest);
    } else if (starts(path, "/api/move/", &rest)) {
      r.start_move(rest);
    } else if (path == "/api/demo") {
      // Демо: ноги по кругу шагают в воздухе; сколько ног сразу — по лимиту тока.
      std::string action = choice(req["action"], "action", {"start", "stop"});
      if (action == "start") {
        double cycles = 4, laps = 3;
        num(req["cycles"], "cycles", cycles, 1, 20);
        num(req["laps"], "laps", laps, 1, 20);
        r.start_demo((int)cycles, (int)laps);
      } else {
        r.stop_demo();
      }
    } else if (path == "/api/gait") {
      // Профиль ходьбы: им шагают и ходьба, и демо. В NVS — /api/calibration/save.
      if (!req["name"].is<const char *>()) throw HttpError{422, "name: обязательное поле"};
      r.set_gait(req["name"].as<const char *>());
    } else if (path == "/api/body") {
      r.set_body(num(req["height"], "height", a), num(req["pitch"], "pitch", b), num(req["roll"], "roll", c));
    } else if (path == "/api/drive") {
      a = b = 0;
      num(req["vx"], "vx", a, -1, 1);
      num(req["turn"], "turn", b, -1, 1);
      r.set_web_drive(a, b);
    } else if (path == "/api/camera/take") {
      // Видео смотрит одно устройство: забрать поток себе, прежний зритель отключится.
      camera_take((uint32_t)need(req["id"], "id", 1, 4294967295.0));
    } else if (path == "/api/calibration/save") {
      out["saved"] = r.save_calibration();
      return;
    } else if (starts(path, "/api/servos/", &rest) && rest.size() > 5 && rest.substr(rest.size() - 5) == "/test") {
      std::string sid = rest.substr(0, rest.size() - 5);
      std::string action = choice(req["action"], "action", {"center", "ref", "sweep", "stop", "nudge"});
      if (action == "nudge") {
        a = 0;
        num(req["delta"], "delta", a, -30, 30);
        r.nudge(sid, a);
      } else {
        r.test_servo(sid, action);
      }
    } else if (starts(path, "/api/servos/", &rest) && rest.size() > 10 && rest.substr(rest.size() - 10) == "/calibrate") {
      // «сустав сейчас = joint°»: zero — сдвиг; scale — вторая точка при неподвижном монтажном положении
      std::string sid = rest.substr(0, rest.size() - 10);
      r.calibrate(sid, need(req["joint"], "joint"), choice(req["what"], "what", {"zero", "scale"}, "zero"));
    } else {
      throw HttpError{404, "Not Found"};
    }
    return state_reply(out);
  } else if (method == HTTP_PUT) {
    if (path == "/api/calibration") {   // calibration.json целиком: с Pi или сохранённый раньше
      r.import_calibration(req.as<JsonObjectConst>());
      return state_reply(out);
    }
    if (starts(path, "/api/servos/", &rest) && rest.find('/') == std::string::npos) {
      const double *angle = num(req["angle"], "angle", a), *joint = num(req["joint"], "joint", b);
      if (!angle == !joint) throw HttpError{400, "нужно ровно одно из: angle, joint"};
      r.set_servo(rest, angle, joint);
      return state_reply(out);
    }
  } else if (method == HTTP_PATCH) {
    if (starts(path, "/api/servos/", &rest) && rest.size() > 12 && rest.substr(rest.size() - 12) == "/calibration") {
      JsonObjectConst f = req.as<JsonObjectConst>();
      if (!f["installed"].isNull() && !f["installed"].is<bool>()) throw HttpError{422, "installed: нужно true/false"};
      if (!f["dir"].isNull() && !(f["dir"].is<int>() && (f["dir"] == 1 || f["dir"] == -1)))
        throw HttpError{422, "dir: -1 или 1"};
      for (const char *k : {"zero", "scale", "min_deg", "max_deg"}) num(f[k], k, a);
      r.update_calibration(rest.substr(0, rest.size() - 12), f);
    } else if (path == "/api/power") {
      // лимит тока серво: подключить можно не больше limit_a / servo_a приводов
      r.update_power(num(req["limit_a"], "limit_a", a, 0, 50), num(req["servo_a"], "servo_a", b, 0, 5, true));
    } else if (path == "/api/trim") {
      r.update_trim(num(req["pitch_deg"], "pitch_deg", a, -10, 10), num(req["roll_deg"], "roll_deg", b, -10, 10));
    } else {
      throw HttpError{404, "Not Found"};
    }
    return state_reply(out);
  }
  throw HttpError{404, "Not Found"};
}

static esp_err_t dispatch(httpd_req_t *req) {
  std::string path = req->uri;
  path = path.substr(0, path.find('?'));
  try {
    String body = read_body(req);
    JsonDocument out;
    try {
      route((httpd_method_t)req->method, path, body, out);
    } catch (const KeyError &e) {
      throw HttpError{404, std::string("нет такого: ") + e.what()};
    } catch (const ValueError &e) {
      throw HttpError{400, e.what()};
    }
    return send_doc(req, 200, out);
  } catch (const HttpError &e) {
    return send_error(req, e);
  }
}

// ---------- Bluetooth: долгие запросы отвечаются из своей задачи ----------
// Поиск идёт 8 с, сопряжение — до 30 с. Сервер HTTP однопоточный: чтобы портал (и СТОП) не ждали,
// запрос передаётся задаче и сервер свободен.

struct BtJob {
  httpd_req_t *req;
  std::string path, body;
  int seconds;
};

static void bt_task(void *arg) {
  BtJob *job = (BtJob *)arg;
  JsonDocument out;
  try {
    try {
      if (job->path == "/api/bluetooth/scan") {
        JsonArray list = out["devices"].to<JsonArray>();
        for (const BtDevice &d : bt_scan(job->seconds)) {
          JsonObject o = list.add<JsonObject>();
          o["mac"] = d.mac;
          o["name"] = d.name;
          o["paired"] = d.paired;
          o["connected"] = d.connected;
        }
      } else {
        JsonDocument req = parse(job->body.c_str());
        if (!req["mac"].is<const char *>()) throw HttpError{422, "mac: обязательное поле"};
        std::string mac = req["mac"].as<const char *>();
        out["log"] = job->path == "/api/bluetooth/pair" ? bt_pair(mac) : bt_remove(mac);
      }
    } catch (const BluetoothError &e) {
      throw HttpError{job->path == "/api/bluetooth/scan" ? 503 : 502, e.what()};
    }
    send_doc(job->req, 200, out);
  } catch (const HttpError &e) {
    send_error(job->req, e);
  }
  httpd_req_async_handler_complete(job->req);
  delete job;
  vTaskDelete(nullptr);
}

static esp_err_t bluetooth(httpd_req_t *req, const std::string &path) {
  BtJob *job = new BtJob{nullptr, path, "", 8};
  try {
    job->body = read_body(req).c_str();
  } catch (const HttpError &e) {
    delete job;
    return send_error(req, e);
  }
  char query[64], v[8];
  if (httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK &&
      httpd_query_key_value(query, "seconds", v, sizeof v) == ESP_OK)
    job->seconds = std::max(2, std::min(atoi(v), 30));
  if (httpd_req_async_handler_begin(req, &job->req) != ESP_OK) {
    delete job;
    return send_error(req, HttpError{503, "сервер занят, повторите"});
  }
  xTaskCreate(bt_task, "bt_req", 8192, job, 2, nullptr);
  return ESP_OK;
}

// ---------- прошивка по воздуху ----------
// Тело — файл .bin как есть (arduino-cli compile --export-binaries → edog.ino.bin). Пишется во второй
// слот прошивки; плата переключается на него, только если запись прошла целиком и образ цел, иначе
// остаётся старая прошивка. Только при выключенных серво: на перезагрузке контур не держит ноги.

static void restart_task(void *) {
  delay(500);   // ответ успеть отправить
  ESP.restart();
}

static esp_err_t firmware_handler(httpd_req_t *req) {
  {
    RobotLock lock;
    if (robot->enabled) return send_error(req, HttpError{409, "обновление прошивки: сначала выключите серво"});
    if (robot->updating) return send_error(req, HttpError{409, "обновление прошивки уже идёт"});
    robot->updating = true;
  }
  auto fail = [&](int code, const std::string &detail) {
    Update.abort();
    {
      RobotLock lock;
      robot->updating = false;
    }
    Serial.printf("прошивка по воздуху: %s\n", detail.c_str());
    return send_error(req, HttpError{code, detail});
  };
  size_t len = req->content_len, left = len;
  if (!len) return fail(400, "пустой файл");
  if (!Update.begin(len, U_FLASH)) return fail(400, std::string("не записать: ") + Update.errorString());
  Serial.printf("прошивка по воздуху: %u байт\n", (unsigned)len);
  std::vector<uint8_t> buf(4096);
  int timeouts = 0;
  while (left) {
    int n = httpd_req_recv(req, (char *)buf.data(), std::min(left, buf.size()));
    if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5) continue;
    if (n <= 0) return fail(400, "передача оборвалась");
    // Образ ESP32 (0xE9) с описанием приложения сразу за заголовками: не загрузчик и не merged.bin.
    if (left == len && (n < 36 || buf[0] != 0xE9 || *(uint32_t *)&buf[32] != ESP_APP_DESC_MAGIC_WORD))
      return fail(400, "это не прошивка приложения ESP32: нужен edog.ino.bin");
    if (Update.write(buf.data(), n) != (size_t)n) return fail(500, std::string("ошибка записи: ") + Update.errorString());
    left -= n;
  }
  if (!Update.end(true)) return fail(400, std::string("образ не принят: ") + Update.errorString());
  JsonDocument out;
  out["ok"] = true;
  out["size"] = len;
  out["message"] = "прошивка записана, перезагрузка";
  send_doc(req, 200, out);
  Serial.println("прошивка по воздуху записана, перезагрузка");
  xTaskCreate(restart_task, "restart", 2048, nullptr, 1, nullptr);
  return ESP_OK;
}

static esp_err_t any_handler(httpd_req_t *req) {
  std::string path = req->uri;
  path = path.substr(0, path.find('?'));
  if (path == "/api/firmware" && req->method == HTTP_POST) return firmware_handler(req);
  if (starts(path, "/api/bluetooth/")) {
    bool scan = path == "/api/bluetooth/scan" && req->method == HTTP_GET;
    bool post = (path == "/api/bluetooth/pair" || path == "/api/bluetooth/remove") && req->method == HTTP_POST;
    if (scan || post) return bluetooth(req, path);
  }
  return dispatch(req);
}

static esp_err_t options_handler(httpd_req_t *req) {   // предзапрос CORS
  httpd_resp_set_status(req, status_text(204));
  cors(req);
  httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
  return httpd_resp_send(req, nullptr, 0);
}

// ---------- WebSocket ----------
// Сервер шлёт состояние 20 раз в секунду. Клиент может слать
// {"type": "drive", "vx", "turn"} и {"type": "servo", "id", "angle"|"joint"}.

static void ws_send(httpd_req_t *req, int fd, const String &text) {
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t *)text.c_str();
  f.len = text.length();
  xSemaphoreTake(ws_send_lock, portMAX_DELAY);
  if (req) httpd_ws_send_frame(req, &f);
  else httpd_ws_send_frame_async(server, fd, &f);
  xSemaphoreGive(ws_send_lock);
}

static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) return ESP_OK;   // рукопожатие
  httpd_ws_frame_t f = {};
  esp_err_t err = httpd_ws_recv_frame(req, &f, 0);
  if (err != ESP_OK) return err;
  if (f.type != HTTPD_WS_TYPE_TEXT || !f.len || f.len > MAX_BODY) return ESP_OK;
  std::vector<uint8_t> buf(f.len + 1, 0);
  f.payload = buf.data();
  err = httpd_ws_recv_frame(req, &f, f.len);
  if (err != ESP_OK) return err;
  JsonDocument msg;
  if (deserializeJson(msg, (const char *)buf.data())) return ESP_OK;
  try {
    RobotLock lock;
    std::string type = msg["type"] | "";
    if (type == "drive") {
      robot->set_web_drive(msg["vx"] | 0.0, msg["turn"] | 0.0);
    } else if (type == "servo") {
      if (!msg["id"].is<const char *>()) throw KeyError("'id'");
      double a, j;
      const double *angle = msg["angle"].is<double>() ? &(a = msg["angle"].as<double>()) : nullptr;
      const double *joint = msg["joint"].is<double>() ? &(j = msg["joint"].as<double>()) : nullptr;
      robot->set_servo(msg["id"].as<const char *>(), angle, joint);
    }
  } catch (const std::exception &e) {
    JsonDocument out;
    out["type"] = "error";
    out["message"] = e.what();
    String text;
    serializeJson(out, text);
    ws_send(req, 0, text);
  }
  return ESP_OK;
}

static void broadcast_task(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / STATE_HZ));
    size_t n = 8;
    int fds[8];
    if (httpd_get_client_list(server, &n, fds) != ESP_OK) continue;
    std::vector<int> ws;
    for (size_t i = 0; i < n; i++)
      if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) ws.push_back(fds[i]);
    if (ws.empty()) continue;
    JsonDocument doc;
    doc["type"] = "state";
    {
      RobotLock lock;
      state_reply(doc);
    }
    String text;
    serializeJson(doc, text);
    for (int fd : ws) ws_send(nullptr, fd, text);
  }
}

// ---------- запуск ----------

void api_begin(Robot &r) {
  robot = &r;
  ws_send_lock = xSemaphoreCreateMutex();
  uint8_t sha[32];
  if (esp_partition_get_sha256(esp_ota_get_running_partition(), sha) == ESP_OK)
    for (int i = 0; i < 8; i++) firmware_id += String(sha[i] < 16 ? "0" : "") + String(sha[i], HEX);
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = API_PORT;
  cfg.ctrl_port = 32768 + API_PORT;
  cfg.stack_size = 12288;
  cfg.max_open_sockets = 8;
  cfg.lru_purge_enable = true;
  cfg.recv_wait_timeout = 10;   // прошивка по воздуху идёт крупными кусками
  cfg.uri_match_fn = httpd_uri_match_wildcard;
  if (httpd_start(&server, &cfg) != ESP_OK) {
    Serial.println("API: сервер не запустился");
    return;
  }
  httpd_uri_t ws = {};
  ws.uri = "/ws";
  ws.method = HTTP_GET;
  ws.handler = ws_handler;
  ws.is_websocket = true;
  httpd_register_uri_handler(server, &ws);
  for (httpd_method_t m : {HTTP_GET, HTTP_POST, HTTP_PUT, HTTP_PATCH}) {
    httpd_uri_t u = {};
    u.uri = "/*";
    u.method = m;
    u.handler = any_handler;
    httpd_register_uri_handler(server, &u);
  }
  httpd_uri_t opt = {};
  opt.uri = "/*";
  opt.method = HTTP_OPTIONS;
  opt.handler = options_handler;
  httpd_register_uri_handler(server, &opt);
  xTaskCreatePinnedToCore(broadcast_task, "ws_state", 8192, nullptr, 2, nullptr, 0);
  Serial.printf("API: http://<адрес>:%d/api, WebSocket /ws\n", API_PORT);
}
