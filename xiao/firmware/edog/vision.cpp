#include "vision.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>

#include <algorithm>
#include <atomic>
#include <cmath>

#include "camera.h"
#include "esp_camera.h"
#include "config.h"
#include "esp_jpeg_dec.h"
#include "jpeg_decoder.h"
#include "patrol.h"
#include "pedestrian.h"
#include "robot.h"

static const int FW = 640, FH = 480;      // кадр камеры
static const int DW = 320, DH = 240;      // распакованный кадр (JPEG с уменьшением вдвое): нейросеть
static const int AW = 160, AH = 120;      // анализ пола и закрытости
static const int GW = 80, GH = 60;        // выравнивание, расширение, миниатюры фаз
static const size_t JPEG_CAP = 160 * 1024;
static const int PHASE_BINS = 16;

// ---------- настройки (NVS «vision») ----------

// Точка калибровки пола: низ предмета на строке y стоит в mm от камеры; body — высота корпуса в этот момент.
struct FloorPoint { double y, mm, body; };

struct VisionCfg {
  double phase = VISION_PHASE;
  bool phase_both = VISION_PHASE_BOTH;
  // Выдержка на ходу: в комнатном свете автоматика держит её длинной, и пол у лап смазан в полосы — шаг
  // по полу не измерить (запись 25.10). На ходу — ручная выдержка (строки сенсора, драйвер ограничивает длиной
  // кадра) и усиление, которое само подстраивается под яркость не выше walk_gain; walk_aec 0 — автоматика.
  int walk_aec = VISION_WALK_AEC;
  int walk_gain = VISION_WALK_GAIN;
  // Камера жёстко на корпусе: высота камеры задана при высоте корпуса ref_body_mm, в другой стойке
  // (другой профиль шага) она выше или ниже ровно на разницу высот корпуса.
  double height_mm = CAMERA_HEIGHT_MM;
  double ref_body_mm = STAND_HEIGHT_MM;
  double tilt_deg = CAMERA_TILT_DEG;
  double hfov_deg = CAMERA_HFOV_DEG;   // угол обзора по горизонтали: уточняется по точкам при измеренной высоте
  bool height_measured = false;        // высота камеры измерена рулеткой — по точкам ищутся наклон и угол обзора
  std::vector<FloorPoint> points;
};

static VisionCfg cfg;
static SemaphoreHandle_t lock;   // настройки, результат анализа, статистика
struct Lock {
  Lock() { xSemaphoreTake(lock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(lock); }
};

static double focal_px() { return (FW / 2.0) / std::tan(cfg.hfov_deg / 2 * M_PI / 180); }
double vision_hfov_deg() { return cfg.hfov_deg; }
double vision_bearing_deg(double x) { return std::atan((x - FW / 2.0) / focal_px()) * 180 / M_PI; }
double vision_col_bearing_deg(int col) { return vision_bearing_deg((col + 0.5) * FW / FREE_COLS); }

// Высота камеры над полом при высоте корпуса body (под замком).
static double cam_height(double body) { return cfg.height_mm + (body > 0 ? body - cfg.ref_body_mm : 0); }

static double current_body() { return robot_body_mm(); }

// Строка кадра (0 … 480) → расстояние по полу, мм; INFINITY — выше горизонта. f — фокус в пикселях.
static double row_to_mm(double y, double h, double tilt_deg, double f) {
  double a = std::atan((y - FH / 2.0) / f) + tilt_deg * M_PI / 180;
  if (a < 0.5 * M_PI / 180) return INFINITY;
  return h / std::tan(a);
}

// Наклон, при котором низ предмета на строке y стоит на расстоянии mm (высота h известна).
static double tilt_for(double y, double mm, double h, double f) {
  return (std::atan(h / mm) - std::atan((y - FH / 2.0) / f)) * 180 / M_PI;
}

static void save_cfg() {
  JsonDocument doc;
  doc["phase"] = cfg.phase;
  doc["phase_both"] = cfg.phase_both;
  doc["walk_aec"] = cfg.walk_aec;
  doc["walk_gain"] = cfg.walk_gain;
  doc["height_mm"] = cfg.height_mm;
  doc["ref_body_mm"] = cfg.ref_body_mm;
  doc["tilt_deg"] = cfg.tilt_deg;
  doc["hfov_deg"] = cfg.hfov_deg;
  doc["height_measured"] = cfg.height_measured;
  for (const FloorPoint &p : cfg.points) {
    JsonArray a = doc["points"].add<JsonArray>();
    a.add(p.y);
    a.add(p.mm);
    a.add(p.body);
  }
  String json;
  serializeJson(doc, json);
  Preferences p;
  p.begin("edog", false);
  p.putString("vision", json);
  p.end();
}

static void load_cfg() {
  Preferences p;
  if (!p.begin("edog", true)) return;
  String json = p.getString("vision", "");
  p.end();
  JsonDocument doc;
  if (json.isEmpty() || deserializeJson(doc, json)) return;
  cfg.phase = doc["phase"] | cfg.phase;
  cfg.phase_both = doc["phase_both"] | cfg.phase_both;
  cfg.walk_aec = doc["walk_aec"] | cfg.walk_aec;
  cfg.walk_gain = doc["walk_gain"] | cfg.walk_gain;
  cfg.height_mm = doc["height_mm"] | cfg.height_mm;
  cfg.ref_body_mm = doc["ref_body_mm"] | cfg.ref_body_mm;
  cfg.tilt_deg = doc["tilt_deg"] | cfg.tilt_deg;
  cfg.hfov_deg = doc["hfov_deg"] | cfg.hfov_deg;
  cfg.height_measured = doc["height_measured"] | cfg.height_measured;
  for (JsonArrayConst a : doc["points"].as<JsonArrayConst>())
    cfg.points.push_back({a[0], a[1], a[2] | cfg.ref_body_mm});
}

// Камера по точкам калибровки (ошибка — относительная по расстоянию). Точки из другой стойки сдвигаются
// на разницу высот корпуса; высота считается при высоте корпуса первой точки.
//   высота измерена: одна точка — наклон; две и больше — наклон и угол обзора (перебор угла);
//   не измерена: одна точка — наклон; две и больше — высота и наклон (перебор высоты).
static void solve_floor() {
  if (cfg.points.empty()) return;
  double ref = cfg.points[0].body;
  cfg.height_mm = cam_height(ref);
  cfg.ref_body_mm = ref;
  auto fit = [&](double h, double f, double &tilt) {   // наклон — среднее по точкам; ошибка
    tilt = 0;
    for (const FloorPoint &p : cfg.points) tilt += tilt_for(p.y, p.mm, h + p.body - ref, f);
    tilt /= cfg.points.size();
    double err = 0;
    for (const FloorPoint &p : cfg.points) {
      double d = row_to_mm(p.y, h + p.body - ref, tilt, f);
      err += std::isfinite(d) ? std::pow((d - p.mm) / p.mm, 2) : 1e6;
    }
    return err;
  };
  if (cfg.points.size() == 1) {
    fit(cfg.height_mm, focal_px(), cfg.tilt_deg);
    return;
  }
  double best = INFINITY, tilt;
  if (cfg.height_measured) {
    for (double fov = 40; fov <= 120; fov += 0.1) {
      double err = fit(cfg.height_mm, (FW / 2.0) / std::tan(fov / 2 * M_PI / 180), tilt);
      if (err < best) {
        best = err;
        cfg.hfov_deg = fov;
        cfg.tilt_deg = tilt;
      }
    }
    return;
  }
  for (double h = 40; h <= 300; h += 1) {
    double err = fit(h, focal_px(), tilt);
    if (err < best) {
      best = err;
      cfg.height_mm = h;
      cfg.tilt_deg = tilt;
    }
  }
}

void vision_floor_point(double y, double mm) {
  if (!(0 <= y && y < FH)) throw ValueError("строка: 0 … 479");
  if (!(100 <= mm && mm <= 5000)) throw ValueError("расстояние: 100 … 5000 мм");
  double body = current_body();
  Lock l;
  cfg.points.push_back({y, mm, body > 0 ? body : cfg.ref_body_mm});
  if (cfg.points.size() > 6) cfg.points.erase(cfg.points.begin());
  solve_floor();
  save_cfg();
}

void vision_floor_reset() {
  Lock l;
  cfg.points.clear();
  cfg.height_mm = CAMERA_HEIGHT_MM;
  cfg.ref_body_mm = STAND_HEIGHT_MM;
  cfg.tilt_deg = CAMERA_TILT_DEG;
  cfg.hfov_deg = CAMERA_HFOV_DEG;
  cfg.height_measured = false;
  save_cfg();
}

void vision_configure(JsonObjectConst f) {
  double body = current_body();
  Lock l;
  VisionCfg c = cfg;
  if (!f["phase"].isNull()) {
    if (!f["phase"].is<double>() || f["phase"] < 0 || f["phase"] >= 1) throw ValueError("phase: 0 … 1");
    c.phase = f["phase"];
  }
  if (!f["phase_both"].isNull()) {
    if (!f["phase_both"].is<bool>()) throw ValueError("phase_both: true/false");
    c.phase_both = f["phase_both"];
  }
  if (!f["walk_aec"].isNull()) {
    if (!f["walk_aec"].is<int>() || f["walk_aec"] < 0 || f["walk_aec"] > 2000) throw ValueError("walk_aec: 0 (автоматика) … 2000");
    c.walk_aec = f["walk_aec"];
  }
  if (!f["walk_gain"].isNull()) {
    if (!f["walk_gain"].is<int>() || f["walk_gain"] < 0 || f["walk_gain"] > 64) throw ValueError("walk_gain: 0 … 64");
    c.walk_gain = f["walk_gain"];
  }
  if (!f["height_mm"].isNull()) {
    if (!f["height_mm"].is<double>() || f["height_mm"] < 30 || f["height_mm"] > 400) throw ValueError("height_mm: 30 … 400");
    // высота, измеренная рулеткой в текущей стойке: точки калибровки остаются, по ним — наклон и угол обзора
    c.height_mm = f["height_mm"];
    c.ref_body_mm = body > 0 ? body : c.ref_body_mm;
    c.height_measured = true;
  }
  if (!f["tilt_deg"].isNull()) {
    if (!f["tilt_deg"].is<double>() || f["tilt_deg"] < -20 || f["tilt_deg"] > 70) throw ValueError("tilt_deg: −20 … 70");
    c.tilt_deg = f["tilt_deg"];
    c.points.clear();
  }
  cfg = c;
  if (!f["height_mm"].isNull() && !cfg.points.empty()) {
    // решатель считает высоту при корпусе первой точки: перенести измеренную высоту к ней
    cfg.height_mm = cam_height(cfg.points[0].body);
    cfg.ref_body_mm = cfg.points[0].body;
    solve_floor();
  }
  save_cfg();
}

// ---------- кадры в фазе шага (задача захвата) ----------
// Фаза кадра — по отметке времени съёмки из истории контура. Из кадров одного окна (около выбранной
// фазы) берётся ближайший; он публикуется, когда окно кончилось. Первый цикл ходьбы пропускается:
// шаг ещё разгоняется.

struct Slot {
  uint8_t *buf = nullptr;
  size_t len = 0;
  double t = 0;
  Motion m;
  double dist = 0;      // до выбранной фазы, доля цикла
  long window = -1;
  bool valid = false;
};
static Slot pending, synced;
static uint32_t synced_seq = 0;
static SemaphoreHandle_t synced_lock;
static double jitter_ms = 0, synced_rate = 0;   // разброс фазы взятых кадров (среднее), кадров в секунду
static double last_synced_t = 0;

static void publish_pending() {
  if (!pending.valid) return;
  xSemaphoreTake(synced_lock, portMAX_DELAY);
  std::swap(synced.buf, pending.buf);
  synced.len = pending.len;
  synced.t = pending.t;
  synced.m = pending.m;
  synced.dist = pending.dist;
  synced_seq++;
  xSemaphoreGive(synced_lock);
  double ms = pending.dist / pending.m.gait_hz * 1000;
  jitter_ms = jitter_ms ? 0.9 * jitter_ms + 0.1 * ms : ms;
  if (last_synced_t) synced_rate = 0.8 * synced_rate + 0.2 / std::max(0.05, pending.t - last_synced_t);
  last_synced_t = pending.t;
  pending.valid = false;
}

// ---------- подбор фазы ----------
// Пока собака идёт, каждый кадр раскладывается по 16 корзинам фазы. Кадр сравнивается с кадром той же
// корзины на прошлом цикле (после выравнивания): где разница меньше — там корпус в одной и той же позе.

static double scan_until = 0;
static float bin_resid[PHASE_BINS];
static int bin_count[PHASE_BINS];
static uint8_t *bin_thumb[PHASE_BINS];
static long bin_cycle[PHASE_BINS];
static int scan_best = -1;
static uint8_t *scan_rgb, *scan_gray, *scan_work;   // 80×60×3, 80×60, буфер декодера

void vision_phase_scan(double seconds) {
  if (!(5 <= seconds && seconds <= 120)) throw ValueError("seconds: 5 … 120");
  Lock l;
  for (int i = 0; i < PHASE_BINS; i++) {
    bin_resid[i] = 0;
    bin_count[i] = 0;
    bin_cycle[i] = -10;
  }
  scan_best = -1;
  scan_until = now_s() + seconds;
}

// work — рабочий буфер декодера (JPEG_WORK байт): свой у каждой задачи, в PSRAM, а не во внутренней памяти.
static const size_t JPEG_WORK = 3200;
static bool decode(const uint8_t *jpeg, size_t len, uint8_t *out, size_t cap, esp_jpeg_image_scale_t scale, uint8_t *work) {
  esp_jpeg_image_cfg_t c = {};
  c.advanced.working_buffer = work;
  c.advanced.working_buffer_size = JPEG_WORK;
  c.indata = (uint8_t *)jpeg;
  c.indata_size = len;
  c.outbuf = out;
  c.outbuf_size = cap;
  c.out_format = JPEG_IMAGE_FORMAT_RGB888;
  c.out_scale = scale;
  esp_jpeg_image_output_t o;
  return esp_jpeg_decode(&c, &o) == ESP_OK;
}

// Кадр для анализа: esp_new_jpeg (SIMD) сразу с уменьшением VGA → 320×240; ROM-декодер (tjpgd) втрое медленнее.
static bool decode_half(const uint8_t *jpeg, size_t len, uint8_t *out) {
  jpeg_dec_config_t c = DEFAULT_JPEG_DEC_CONFIG();
  c.output_type = JPEG_PIXEL_FORMAT_RGB888;
  c.scale = {(uint16_t)DW, (uint16_t)DH};
  jpeg_dec_handle_t h;
  if (jpeg_dec_open(&c, &h) != JPEG_ERR_OK) return false;
  jpeg_dec_io_t io = {};
  io.inbuf = (uint8_t *)jpeg;
  io.inbuf_len = len;
  jpeg_dec_header_info_t info;
  bool ok = jpeg_dec_parse_header(h, &io, &info) == JPEG_ERR_OK;
  if (ok) {
    io.outbuf = out;
    ok = jpeg_dec_process(h, &io) == JPEG_ERR_OK;
  }
  jpeg_dec_close(h);
  return ok;
}

static void rgb_to_gray(const uint8_t *rgb, uint8_t *gray, int n) {
  for (int i = 0; i < n; i++) gray[i] = (rgb[3 * i] + 2 * rgb[3 * i + 1] + rgb[3 * i + 2]) >> 2;
}

// Лучший сдвиг b относительно a (a(x) ≈ b(x + dx, y + dy)) по средней разности после вычитания средней
// яркости. Возвращает среднюю разность на пиксель.
static float align(const uint8_t *a, const uint8_t *b, int w, int h, int rx, int ry, int &best_dx, int &best_dy) {
  long sa = 0, sb = 0;
  for (int i = 0; i < w * h; i++) {
    sa += a[i];
    sb += b[i];
  }
  int off = (int)((sb - sa) / (w * h));
  float best = INFINITY;
  best_dx = best_dy = 0;
  int mx = std::max(2, w / 10), my = std::max(2, h / 10);
  for (int dy = -ry; dy <= ry; dy++) {
    for (int dx = -rx; dx <= rx; dx++) {
      int x0 = std::max(mx, mx - dx), x1 = std::min(w - mx, w - mx - dx);
      int y0 = std::max(my, my - dy), y1 = std::min(h - my, h - my - dy);
      if (x1 - x0 < w / 3 || y1 - y0 < h / 2) continue;
      long sad = 0;
      for (int y = y0; y < y1; y++) {
        const uint8_t *pa = a + y * w, *pb = b + (y + dy) * w + dx;
        for (int x = x0; x < x1; x++) sad += std::abs(pa[x] - pb[x] + off);
      }
      float v = (float)sad / ((x1 - x0) * (y1 - y0));
      if (v < best) {
        best = v;
        best_dx = dx;
        best_dy = dy;
      }
    }
  }
  return best;
}

static void phase_scan_frame(const uint8_t *jpeg, size_t len, const Motion &m) {
  if (!decode(jpeg, len, scan_rgb, GW * GH * 3, JPEG_IMAGE_SCALE_1_8, scan_work)) return;
  uint8_t *gray = scan_gray;
  rgb_to_gray(scan_rgb, gray, GW * GH);
  double frac = m.cycles - std::floor(m.cycles);
  int bin = std::min(PHASE_BINS - 1, (int)(frac * PHASE_BINS));
  long cycle = (long)std::floor(m.cycles);
  if (bin_cycle[bin] == cycle - 1) {
    int dx, dy;
    float r = align(gray, bin_thumb[bin], GW, GH, 4, 3, dx, dy);
    bin_resid[bin] = bin_count[bin] ? 0.8f * bin_resid[bin] + 0.2f * r : r;
    bin_count[bin]++;
  }
  memcpy(bin_thumb[bin], gray, GW * GH);
  bin_cycle[bin] = cycle;
}

static void finish_phase_scan() {
  int best = -1;
  for (int i = 0; i < PHASE_BINS; i++)
    if (bin_count[i] >= 3 && (best < 0 || bin_resid[i] < bin_resid[best])) best = i;
  scan_best = best;
  if (best < 0) {
    Serial.println("зрение: фаза не подобрана — собака почти не шла");
    return;
  }
  cfg.phase = (best + 0.5) / PHASE_BINS;
  save_cfg();
  Serial.printf("зрение: фаза съёмки на ходу %.3f (разница %.1f)\n", cfg.phase, bin_resid[best]);
}

void vision_on_frame(const uint8_t *jpeg, size_t len, double t) {
  Motion m;
  bool walking = robot_motion_at(t, m) && m.walking && m.cycles >= 1.0;
  double phase, tol = VISION_PHASE_TOL;
  bool both, scanning;
  {
    Lock l;
    phase = cfg.phase;
    both = cfg.phase_both;
    scanning = scan_until > 0;
    if (scanning && now_s() > scan_until) {
      scan_until = 0;
      finish_phase_scan();
      scanning = false;
    }
  }
  if (!walking) {
    publish_pending();
    return;
  }
  if (scanning) phase_scan_frame(jpeg, len, m);

  int k = both ? 2 : 1;
  double u = (m.cycles - phase) * k;
  long window = (long)std::floor(u + 0.5);
  double dist = std::fabs(u - window) / k;
  if (pending.valid && pending.window != window) publish_pending();
  if (dist >= tol) {
    publish_pending();
    return;
  }
  if (pending.valid && dist >= pending.dist) return;
  if (len > JPEG_CAP) return;
  memcpy(pending.buf, jpeg, len);
  pending.len = len;
  pending.t = t;
  pending.m = m;
  pending.dist = dist;
  pending.window = window;
  pending.valid = true;
}

// ---------- анализ ----------

static Pedestrian *detector;
static uint8_t *jpeg_buf, *rgb;            // кадр JPEG, распакованный 320×240×3
static int16_t *ay, *au, *av;              // 160×120: яркость и цветность
static uint8_t *agrad;                     // 160×120: перепад яркости
static uint8_t *gray, *prev_gray;   // 80×60: сейчас и прошлый синхронный
static int16_t *prev_ay;             // 160×120: яркость прошлого синхронного — для шага по полу
static bool prev_valid = false;
static double prev_t = 0;
static Perception result;
static uint32_t seq = 0;
static double ms_decode = 0, ms_detect = 0, ms_analyze = 0, rate = 0, last_done = 0;
static float walk_resid = 0;               // обычная разница соседних синхронных кадров на ходу
static int walk_resid_n = 0, stagnant_run = 0;

struct FloorModel {
  bool ok = false;
  float y, u, v, sy, su, sv;
  int mismatch = 0;
};
static FloorModel floor_model;

static void downsample() {   // 320×240×3 → 160×120 (Y, U = R−G, V = B−G), 80×60 серый
  for (int y = 0; y < AH; y++) {
    for (int x = 0; x < AW; x++) {
      int r = 0, g = 0, b = 0;
      for (int dy = 0; dy < 2; dy++) {
        const uint8_t *p = rgb + ((2 * y + dy) * DW + 2 * x) * 3;
        r += p[0] + p[3];
        g += p[1] + p[4];
        b += p[2] + p[5];
      }
      int i = y * AW + x;
      ay[i] = (r + 2 * g + b) >> 4;
      au[i] = (r - g) >> 2;
      av[i] = (b - g) >> 2;
    }
  }
  for (int y = 0; y < AH; y++) {
    for (int x = 0; x < AW; x++) {
      int i = y * AW + x;
      int gx = x + 1 < AW ? std::abs(ay[i + 1] - ay[i]) : 0;
      int gy = y + 1 < AH ? std::abs(ay[i + AW] - ay[i]) : 0;
      agrad[i] = std::min(255, gx + gy);
    }
  }
  for (int y = 0; y < GH; y++)
    for (int x = 0; x < GW; x++) {
      int i = 2 * y * AW + 2 * x;
      gray[y * GW + x] = (ay[i] + ay[i + 1] + ay[i + AW] + ay[i + AW + 1]) >> 2;
    }
}

// Модель пола — по участку прямо перед лапами (низ кадра, середина). Обновляется плавно, пока участок
// похож на модель; если не похож долго — пол сменился (ковёр → плитка), модель берётся заново.
static void update_floor_model() {
  double s[3] = {0, 0, 0}, q[3] = {0, 0, 0};
  int n = 0;
  for (int y = AH - 12; y < AH; y++)
    for (int x = AW / 2 - 32; x < AW / 2 + 32; x++) {
      int i = y * AW + x;
      double v[3] = {(double)ay[i], (double)au[i], (double)av[i]};
      for (int k = 0; k < 3; k++) {
        s[k] += v[k];
        q[k] += v[k] * v[k];
      }
      n++;
    }
  float m[3], sd[3];
  for (int k = 0; k < 3; k++) {
    m[k] = s[k] / n;
    sd[k] = std::sqrt(std::max(0.0, q[k] / n - m[k] * m[k]));
  }
  FloorModel &f = floor_model;
  auto set = [&](float a) {
    f.y = f.y + a * (m[0] - f.y);
    f.u = f.u + a * (m[1] - f.u);
    f.v = f.v + a * (m[2] - f.v);
    f.sy = std::max(5.0f, f.sy + a * (sd[0] - f.sy));
    f.su = std::max(2.5f, f.su + a * (sd[1] - f.su));
    f.sv = std::max(2.5f, f.sv + a * (sd[2] - f.sv));
  };
  if (!f.ok) {
    f.y = m[0]; f.u = m[1]; f.v = m[2]; f.sy = sd[0]; f.su = sd[1]; f.sv = sd[2];
    set(0);
    f.ok = true;
    f.mismatch = 0;
    return;
  }
  float d = std::max({std::fabs(m[0] - f.y) / std::max(f.sy, 8.0f), std::fabs(m[1] - f.u) / std::max(f.su, 4.0f),
                      std::fabs(m[2] - f.v) / std::max(f.sv, 4.0f)});
  if (d < 3) {
    set(0.15f);
    f.mismatch = 0;
  } else if (++f.mismatch >= 2) {   // собака всегда стоит на полу: не похож — сменились пол или свет
    f.ok = false;   // на следующем кадре — заново
  }
}

static inline bool is_floor(int i) {
  const FloorModel &f = floor_model;
  return std::fabs(ay[i] - f.y) <= std::max(4 * f.sy, 22.0f) && std::fabs(au[i] - f.u) <= std::max(3 * f.su, 8.0f) &&
         std::fabs(av[i] - f.v) <= std::max(3 * f.sv, 8.0f) && agrad[i] < 40;
}

static const float PERSON_MAX_ASPECT = 0.6f;   // ширина рамки человека / высота: по записи люди 0.4–0.55
static const float FLOOR_Y_DRIFT = 45, FLOOR_C_DRIFT = 10;   // насколько дальний пол может отличаться от пола у лап
static const double COVERED_SD = 9, COVERED_SD_WALK = 5;
static int exposure_manual = -1;   // что сейчас стоит на сенсоре: 1 — ручная, 0 — авто, −1 — неизвестно
static int exposure_aec = -1, exposure_gain = -1;
static float lum = NAN;            // средняя яркость последнего кадра анализа (0…255) — подбирать выдержку
static float gain_now = -1;        // усиление на ходу: к яркости WALK_LUM, не выше walk_gain
static const float WALK_LUM = 70;   // разброс яркости кадра (0…255) ниже — кадр закрыт, камера вплотную к препятствию
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static void analyze_floor(Perception &p, double h, double tilt) {
  // однотонный кадр: мало перепадов и малый разброс яркости — стена или бок мебели вплотную
  long flat = 0;
  double s = 0, q = 0;
  for (int i = 0; i < AW * AH; i++) {
    flat += agrad[i] < 6;
    s += ay[i];
    q += (double)ay[i] * ay[i];
  }
  p.flatness = (float)flat / (AW * AH);
  double sd = std::sqrt(std::max(0.0, q / (AW * AH) - std::pow(s / (AW * AH), 2)));
  lum = (float)(s / (AW * AH));
  // Кадр закрыт: яркость почти одна по всему кадру — объектив уткнулся в стену или мебель (запись
  // 25.10: разброс 0.6–8 перед ударами, тёмный коридор — от 10, свет — от 36). Пола не видно, модель пола
  // на таком кадре не учится (иначе вся темнота — «пол до горизонта»), впереди — вплотную.
  // с короткой выдержкой на ходу кадр темнее и ровнее — порог строже
  if (sd < (exposure_manual == 1 ? COVERED_SD_WALK : COVERED_SD)) {
    p.floor_known = floor_model.ok;
    p.bottom_floor = 0;
    float blind = (float)row_to_mm(FH - 1, h, tilt, focal_px());
    for (float &f : p.free_mm) f = blind;
    p.closeness = 1;
    return;
  }
  update_floor_model();
  p.floor_known = floor_model.ok;
  // строка горизонта (в 120 строках анализа): выше неё пола не бывает
  double horizon480 = FH / 2.0 + focal_px() * std::tan(0.5 * M_PI / 180 - tilt * M_PI / 180);
  int top = std::max(0, (int)std::ceil(horizon480 / 4) + 1);
  int bottom_n = 0, bottom_floor = 0;
  for (int y = AH - 24; y < AH; y++)
    for (int x = 0; x < AW; x++) {
      bottom_n++;
      bottom_floor += floor_model.ok && is_floor(y * AW + x);
    }
  p.bottom_floor = (float)bottom_floor / bottom_n;
  int cw = AW / FREE_COLS;
  for (int c = 0; c < FREE_COLS; c++) {
    p.free_mm[c] = NAN;
    if (!floor_model.ok || top >= AH - 1) continue;
    // Пол растёт от низа вверх: строка — пол, если похожа на цвет пола строками ниже. Дальний пол светлее
    // и бледнее (свет, блики) — опорный цвет плывёт вслед, но от пола под лапами уходит недалеко.
    const FloorModel &fm = floor_model;
    float ry = fm.y, ru = fm.u, rv = fm.v;
    float ty = std::max(3 * fm.sy, 14.0f), tu = std::max(3 * fm.su, 8.0f), tv = std::max(3 * fm.sv, 8.0f);
    int run = 0, boundary = -1;
    for (int y = AH - 1; y >= top; y--) {
      int n = 0;
      float sy = 0, su = 0, sv = 0;
      for (int x = c * cw; x < (c + 1) * cw; x++) {
        int i = y * AW + x;
        if (std::fabs(ay[i] - ry) <= ty && std::fabs(ay[i] - fm.y) <= FLOOR_Y_DRIFT && std::fabs(au[i] - ru) <= tu &&
            std::fabs(av[i] - rv) <= tv && agrad[i] < 40) {
          n++;
          sy += ay[i];
          su += au[i];
          sv += av[i];
        }
      }
      if (n * 10 >= cw * 6) {
        run = 0;
        ry += 0.3f * (sy / n - ry);
        ru = clampf(ru + 0.15f * (su / n - ru), fm.u - FLOOR_C_DRIFT, fm.u + FLOOR_C_DRIFT);
        rv = clampf(rv + 0.15f * (sv / n - rv), fm.v - FLOOR_C_DRIFT, fm.v + FLOOR_C_DRIFT);
        continue;
      }
      if (++run == 3) {
        boundary = y + 2;
        break;
      }
    }
    double row = boundary < 0 ? top : boundary;
    double d = row_to_mm(row * 4 + 2, h, tilt, focal_px());
    p.free_mm[c] = (float)std::min(FREE_MAX_MM, std::isfinite(d) ? d : FREE_MAX_MM);
  }
  float flat_close = std::min(1.0f, std::max(0.0f, (p.flatness - 0.7f) / 0.25f)) * (sd < 20 ? 1.0f : 0.4f);
  float floor_close = floor_model.ok ? std::min(1.0f, std::max(0.0f, (0.75f - p.bottom_floor) / 0.6f)) : 0.0f;
  p.closeness = std::min(1.0f, 0.65f * floor_close + 0.2f * flat_close);
}

// Центр текущего кадра против прошлого, увеличенного в s раз: если увеличенный похож лучше — центр растёт.
static float expansion(int dx, int dy) {
  const float scales[] = {1.0f, 1.04f, 1.08f, 1.12f, 1.16f};
  float cx = GW / 2.0f, cy = GH / 2.0f, best = INFINITY, base = 0, best_s = 1;
  for (float s : scales) {
    long sad = 0;
    int n = 0;
    for (int y = 12; y < GH - 12; y++)
      for (int x = 16; x < GW - 16; x++) {
        int px = (int)std::lround(cx + (x - cx) / s) + dx, py = (int)std::lround(cy + (y - cy) / s) + dy;
        if (px < 0 || px >= GW || py < 0 || py >= GH) continue;
        sad += std::abs(gray[y * GW + x] - prev_gray[py * GW + px]);
        n++;
      }
    float v = n ? (float)sad / n : INFINITY;
    if (s == 1.0f) base = v;
    if (v < best) {
      best = v;
      best_s = s;
    }
  }
  if (!(best < 0.9f * base)) return 0;
  return std::min(1.0f, std::max(0.0f, (best_s - 1.02f) / 0.1f));
}

// Шаг по полу: узор пола в нижней полосе кадра на ходу вперёд съезжает вниз. Ищем, на сколько строк
// (и чуть вбок) съехала полоса с прошлого синхронного кадра; строку до и после переводим в расстояние
// по полу — разница и есть пройденное. Собака буксует — узор стоит, шаг ≈ 0. Пол без узора — NAN.
static const int STEP_Y0 = AH - 28, STEP_Y1 = AH - 6, STEP_X0 = 32, STEP_X1 = AW - 32;
static const int STEP_MAX_DY = 20, STEP_MAX_DX = 6;   // вбок — увод на ходу

static float floor_step(double h, double tilt) {
  int area = (STEP_Y1 - STEP_Y0) * (STEP_X1 - STEP_X0);
  long texture = 0, sa = 0, sb = 0;
  for (int y = STEP_Y0; y < STEP_Y1; y++)
    for (int x = STEP_X0; x < STEP_X1; x++) {
      int i = y * AW + x;
      texture += agrad[i];
      sa += ay[i];
      sb += prev_ay[i];
    }
  if (texture < 5L * area) return NAN;
  int off = (int)((sb - sa) / area);   // экспозиция между кадрами могла смениться
  float sad[STEP_MAX_DY + 1], mean = 0;
  int best = 0;
  for (int dy = 0; dy <= STEP_MAX_DY; dy++) {
    sad[dy] = INFINITY;
    for (int dx = -STEP_MAX_DX; dx <= STEP_MAX_DX; dx++) {
      long s = 0;
      for (int y = STEP_Y0; y < STEP_Y1; y++) {
        const int16_t *a = ay + y * AW, *b = prev_ay + (y - dy) * AW + dx;
        for (int x = STEP_X0; x < STEP_X1; x++) s += std::abs(a[x] - b[x] + off);
      }
      sad[dy] = std::min(sad[dy], (float)s / area);
    }
    mean += sad[dy];
    if (sad[dy] < sad[best]) best = dy;
  }
  mean /= STEP_MAX_DY + 1;
  if (best == STEP_MAX_DY || !(sad[best] < 0.6f * mean)) return NAN;   // сдвиг за пределом или совпадение не явное
  double dy = best;
  if (best > 0) {   // доля строки — по параболе через соседей
    double a = sad[best - 1], b = sad[best], c = sad[best + 1], d = a - 2 * b + c;
    if (d > 0) dy += 0.5 * (a - c) / d;
  }
  double yc = (STEP_Y0 + STEP_Y1) / 2.0, f = focal_px();
  double now_mm = row_to_mm(yc * 4 + 2, h, tilt, f), was_mm = row_to_mm((yc - dy) * 4 + 2, h, tilt, f);
  if (!std::isfinite(now_mm) || !std::isfinite(was_mm)) return NAN;
  return (float)(was_mm - now_mm);
}

static void make_thumb(Perception &p) {
  float s = 0, q = 0;
  int bw = GW / THUMB_W, bh = GH / THUMB_H;
  for (int ty = 0; ty < THUMB_H; ty++)
    for (int tx = 0; tx < THUMB_W; tx++) {
      int sum = 0;
      for (int y = 0; y < bh; y++)
        for (int x = 0; x < bw; x++) sum += gray[(ty * bh + y) * GW + tx * bw + x];
      float v = (float)sum / (bw * bh);
      p.thumb[ty * THUMB_W + tx] = v;
      s += v;
      q += v * v;
    }
  int n = THUMB_W * THUMB_H;
  float m = s / n, sd = std::sqrt(std::max(1.0f, q / n - m * m));
  for (float &v : p.thumb) v = (v - m) / sd;
}

static SemaphoreHandle_t detector_lock;   // детектор один: анализ и проверка из портала

static void detect(Perception &p, double h, double tilt) {
  if (!detector) return;
  double f = focal_px();
  xSemaphoreTake(detector_lock, portMAX_DELAY);
  std::list<dl::detect::result_t> found = detector->run(rgb, DW, DH);
  xSemaphoreGive(detector_lock);
  for (const dl::detect::result_t &r : found) {
    if (r.score < PERSON_SCORE) continue;
    Person q;
    q.x1 = r.box[0] * 2; q.y1 = r.box[1] * 2; q.x2 = r.box[2] * 2; q.y2 = r.box[3] * 2;
    // снизу человек — узкая высокая рамка (ноги, фигура); широкая — кусок обстановки (кухня, коридор, тумба)
    if (q.x2 - q.x1 > PERSON_MAX_ASPECT * (q.y2 - q.y1)) continue;
    q.score = r.score;
    q.bearing_deg = vision_bearing_deg((q.x1 + q.x2) / 2.0);
    // ступни видны (рамка не упирается в низ) — расстояние по полу; иначе по высоте рамки, если видна голова
    q.dist_mm = NAN;
    if (q.y2 < FH - 8) q.dist_mm = row_to_mm(q.y2, h, tilt, focal_px());
    if (!std::isfinite(q.dist_mm) && q.y1 > 8) q.dist_mm = PERSON_HEIGHT_MM * f / std::max(1, q.y2 - q.y1);
    if (!std::isfinite(q.dist_mm)) q.dist_mm = NAN;
    p.people.push_back(q);
  }
}

static void analyze(const uint8_t *jpeg, size_t len, double t, const Motion &m, bool is_synced) {
  double t0 = now_s();
  if (!decode_half(jpeg, len, rgb)) return;
  double t1 = now_s();
  double h, tilt;
  {
    Lock l;
    h = cam_height(m.body_mm);
    tilt = cfg.tilt_deg;
  }
  Perception p;
  p.t = t;
  p.h_mm = h;
  p.tilt_deg = tilt;
  p.synced = is_synced;
  p.vx = m.vx;
  p.turn = m.turn;
  detect(p, h, tilt);
  double t2 = now_s();
  downsample();
  analyze_floor(p, h, tilt);
  make_thumb(p);
  // усиление на ходу — к яркости WALK_LUM (по кадру, снятому с ручной выдержкой)
  if (is_synced && exposure_manual == 1 && std::isfinite(lum) && gain_now >= 0) {
    int cap;
    {
      Lock l;
      cap = cfg.walk_gain;
    }
    float k = std::sqrt(WALK_LUM / std::max(lum, 5.0f));
    gain_now = std::min((float)cap, std::max(0.0f, gain_now * std::min(1.25f, std::max(0.8f, k)) + (k > 1 ? 0.5f : 0)));
  }
  // соседние синхронные кадры (не дальше двух циклов): выравнивание, разница, поворот, расширение
  if (is_synced && prev_valid && t - prev_t < 2.5 / m.gait_hz) {
    bool turning = std::fabs(m.turn) > 0.3 && std::fabs(m.vx) < 0.2;
    int dx, dy;
    p.residual = align(gray, prev_gray, GW, GH, turning ? 30 : 6, 4, dx, dy);
    p.dt = t - prev_t;
    // на ходу прямо — тоже: из-за сборки и серво собака уводит в сторону
    if ((turning || m.vx > 0.3) && p.residual < 16) p.yaw_deg = dx * cfg.hfov_deg / GW;
    if (m.vx > 0.3 && std::fabs(m.turn) < 0.3) p.step_mm = floor_step(h, tilt);
    if (m.vx > 0.3) {
      p.expansion = expansion(dx, dy);
      p.closeness = std::min(1.0f, p.closeness + 0.25f * p.expansion);
      bool still = walk_resid_n >= 5 && p.residual < 0.4f * walk_resid && p.expansion < 0.2f;
      if (!still || walk_resid_n < 5) {
        walk_resid = walk_resid_n ? 0.9f * walk_resid + 0.1f * p.residual : p.residual;
        walk_resid_n++;
      }
      stagnant_run = still ? stagnant_run + 1 : 0;
      p.stagnant = stagnant_run >= 3;
    } else {
      stagnant_run = 0;
    }
  }
  if (is_synced) {
    memcpy(prev_gray, gray, GW * GH);
    memcpy(prev_ay, ay, AW * AH * sizeof *ay);
    prev_valid = true;
    prev_t = t;
  } else {
    prev_valid = false;
    stagnant_run = 0;
  }
  double t3 = now_s();
  Lock l;
  p.seq = ++seq;
  result = p;
  ms_decode = 0.8 * ms_decode + 0.2 * (t1 - t0) * 1000;
  ms_detect = 0.8 * ms_detect + 0.2 * (t2 - t1) * 1000;
  ms_analyze = 0.8 * ms_analyze + 0.2 * (t3 - t2) * 1000;
  if (last_done) rate = 0.8 * rate + 0.2 / std::max(0.01, t3 - last_done);
  last_done = t3;
}

// Модель людей: мелкие выделения ESP-DL (тензоры, слои) по настройке ядра идут во внутреннюю память
// (меньше 4 КБ — туда), и её не хватает API и Bluetooth. На время загрузки — всё в PSRAM.
static void load_model() {
  size_t internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  uint32_t t0 = millis();
  heap_caps_malloc_extmem_enable(1);
  detector = new Pedestrian(0.4f, 0.5f);
  heap_caps_malloc_extmem_enable(EXTMEM_LIMIT);
  Serial.printf("Зрение: модель людей загружена за %u мс; внутренняя память %u → %u КБ, PSRAM свободно %u КБ\n",
                millis() - t0, internal / 1024, heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
                ESP.getFreePsram() / 1024);
}

// Экспозиция камеры: на ходу — ручная (короткая выдержка, усиление), стоя — автоматика (set_exposure).

static void set_exposure(bool walking) {
  int aec, gain;
  {
    Lock l;
    aec = cfg.walk_aec;
    gain = cfg.walk_gain;
  }
  bool manual = walking && aec > 0;
  if (gain_now < 0 || gain_now > gain) gain_now = gain * 0.6f;
  gain = (int)std::lround(gain_now);
  if (manual == (exposure_manual == 1) && exposure_manual >= 0 && (!manual || (aec == exposure_aec && gain == exposure_gain)))
    return;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  if (manual) {
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, aec);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, gain);
  } else {
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
  }
  exposure_manual = manual;
  exposure_aec = aec;
  exposure_gain = gain;
}

static void vision_task(void *) {
  delay(3000);   // сначала пусть поднимутся API, камера и Bluetooth
  load_model();
  {   // первый прогон: сколько внутренней памяти берёт сама работа модели
    size_t before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    heap_caps_malloc_extmem_enable(1);
    memset(rgb, 0, DW * DH * 3);
    detector->run(rgb, DW, DH);
    heap_caps_malloc_extmem_enable(EXTMEM_LIMIT);
    Serial.printf("Зрение: первый прогон, внутренняя память %u → %u КБ\n", before / 1024,
                  heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
  }
  uint32_t last_seq = 0, last_synced = 0;
  for (;;) {
    bool active = patrol_active() || camera_clients() > 0;
    if (!active) {
      prev_valid = false;
      delay(200);
      continue;
    }
    Motion m;
    bool known = robot_motion_at(now_s() - 0.012, m);
    set_exposure(known && m.walking);
    if (known && m.walking) {
      // на ходу — только кадры в выбранной фазе
      xSemaphoreTake(synced_lock, portMAX_DELAY);
      bool fresh = synced_seq != last_synced;
      size_t len = synced.len;
      double t = synced.t;
      Motion sm = synced.m;
      if (fresh) memcpy(jpeg_buf, synced.buf, len);
      last_synced = synced_seq;
      xSemaphoreGive(synced_lock);
      if (!fresh) {
        delay(10);
        continue;
      }
      analyze(jpeg_buf, len, t, sm, true);
      continue;
    }
    // стоя — свежий кадр, снятый не раньше чем через VISION_SETTLE_S после остановки
    double since = robot_still_since();
    if (std::isnan(since)) {
      prev_valid = false;
      delay(20);
      continue;
    }
    size_t len;
    double t;
    uint32_t got = camera_latest(jpeg_buf, JPEG_CAP, len, t, last_seq, 200);
    if (!got) continue;
    last_seq = got;
    if (t < since + VISION_SETTLE_S) continue;
    Motion still;   // стоя: нужна высота корпуса — от неё высота камеры
    if (!robot_motion_at(t, still)) still = Motion();
    still.gait_hz = 1;
    analyze(jpeg_buf, len, t, still, false);
  }
}

// ---------- запуск и вывод ----------

void vision_begin() {
  lock = xSemaphoreCreateMutex();
  synced_lock = xSemaphoreCreateMutex();
  detector_lock = xSemaphoreCreateMutex();
  load_cfg();
  auto ps = [](size_t n) { return (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_SPIRAM); };
  pending.buf = ps(JPEG_CAP);
  synced.buf = ps(JPEG_CAP);
  jpeg_buf = ps(JPEG_CAP);
  rgb = (uint8_t *)heap_caps_aligned_alloc(16, DW * DH * 3, MALLOC_CAP_SPIRAM);   // декодер требует выравнивания
  ay = (int16_t *)ps(AW * AH * 2);
  au = (int16_t *)ps(AW * AH * 2);
  av = (int16_t *)ps(AW * AH * 2);
  agrad = ps(AW * AH);
  scan_rgb = ps(GW * GH * 3);
  scan_gray = ps(GW * GH);
  scan_work = ps(JPEG_WORK);
  gray = ps(GW * GH);
  prev_gray = ps(GW * GH);
  prev_ay = (int16_t *)ps(AW * AH * 2);
  for (auto &b : bin_thumb) b = ps(GW * GH);
  // Ядро 1, ниже контура серво: тяжёлый анализ не мешает шагу. Стек в PSRAM — внутренней памяти мало;
  // так можно, пока задача не пишет во флеш (NVS пишут HTTP и задача захвата).
  xTaskCreatePinnedToCoreWithCaps(vision_task, "vision", 12288, nullptr, 1, nullptr, 1, MALLOC_CAP_SPIRAM);
}

bool vision_latest(Perception &out) {
  Lock l;
  if (!result.seq) return false;
  out = result;
  return true;
}

static int cm(float mm) { return std::isfinite(mm) ? (int)std::lround(mm / 10) : -1; }

int vision_headers(char *buf, size_t cap) {
  Perception p;
  if (!vision_latest(p) || now_s() - p.t > 3) return 0;
  int n = snprintf(buf, cap, "X-Detect: ");
  for (size_t i = 0; i < p.people.size() && n < (int)cap - 40; i++) {
    const Person &q = p.people[i];
    n += snprintf(buf + n, cap - n, "%s%d,%d,%d,%d,%d,%d", i ? ";" : "", q.x1, q.y1, q.x2, q.y2,
                  (int)std::lround(q.score * 100), cm(q.dist_mm));
  }
  n += snprintf(buf + n, cap - n, "\r\nX-Free: ");
  for (int c = 0; c < FREE_COLS && n < (int)cap - 8; c++) n += snprintf(buf + n, cap - n, "%s%d", c ? "," : "", cm(p.free_mm[c]));
  // h, tilt, fov — геометрия камеры на этом кадре: по ним клиент переводит расстояния X-Free в строки кадра
  n += snprintf(buf + n, cap - n, "\r\nX-Vision: seq=%u;close=%.2f;synced=%d;age=%d;h=%.0f;tilt=%.1f;fov=%.0f;step=%d;exp=%s;lum=%.0f\r\n",
                p.seq, p.closeness, p.synced, (int)((now_s() - p.t) * 1000), p.h_mm, p.tilt_deg, cfg.hfov_deg,
                std::isfinite(p.step_mm) ? (int)std::lround(p.step_mm) : -1, exposure_manual == 1 ? "walk" : "auto",
                std::isfinite(lum) ? lum : -1.0f);
  return std::min(n, (int)cap);
}

void vision_state(JsonObject out) {
  Lock l;
  double now = now_s();
  out["active"] = patrol_active() || camera_clients() > 0;
  out["ms"]["decode"] = std::round(ms_decode);
  out["ms"]["detect"] = std::round(ms_detect);
  out["ms"]["analyze"] = std::round(ms_analyze);
  out["rate"] = last_done && now - last_done < 2 ? std::round(rate * 10) / 10 : 0;
  out["temp_c"] = std::round(temperatureRead());
  out["model"] = detector != nullptr;
  out["heap_kb"] = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
  out["heap_min_kb"] = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024;
  out["psram_kb"] = ESP.getFreePsram() / 1024;
  JsonObject ph = out["phase"].to<JsonObject>();
  ph["target"] = cfg.phase;
  ph["both"] = cfg.phase_both;
  JsonObject ex = out["exposure"].to<JsonObject>();
  ex["walk_aec"] = cfg.walk_aec;
  ex["walk_gain"] = cfg.walk_gain;
  ex["manual"] = exposure_manual == 1;
  if (exposure_manual == 1) ex["gain"] = exposure_gain;
  if (std::isfinite(lum)) ex["lum"] = std::lround(lum);
  ph["tol"] = VISION_PHASE_TOL;
  ph["jitter_ms"] = std::round(jitter_ms * 10) / 10;
  ph["synced_rate"] = last_synced_t && now - last_synced_t < 2 ? std::round(synced_rate * 10) / 10 : 0;
  ph["walk_resid"] = std::round(walk_resid * 10) / 10;
  JsonObject sc = ph["scan"].to<JsonObject>();
  sc["active"] = scan_until > 0;
  if (scan_until > 0) sc["left_s"] = std::max(0.0, std::round(scan_until - now));
  if (scan_best >= 0) sc["best"] = (scan_best + 0.5) / PHASE_BINS;
  JsonArray bins = sc["bins"].to<JsonArray>();
  for (int i = 0; i < PHASE_BINS; i++) {
    if (bin_count[i]) bins.add(std::round(bin_resid[i] * 10) / 10);
    else bins.add(nullptr);
  }
  JsonObject fl = out["floor"].to<JsonObject>();
  // высота камеры — в текущей стойке; ref — при какой высоте корпуса задана калибровка
  double body = current_body(), h = cam_height(body);
  fl["height_mm"] = std::round(h);
  fl["body_mm"] = std::round(body);
  fl["ref_height_mm"] = std::round(cfg.height_mm);
  fl["ref_body_mm"] = std::round(cfg.ref_body_mm);
  fl["tilt_deg"] = std::round(cfg.tilt_deg * 10) / 10;
  fl["hfov_deg"] = std::round(cfg.hfov_deg * 10) / 10;
  fl["height_measured"] = cfg.height_measured;
  fl["blind_mm"] = std::round(row_to_mm(FH - 1, h, cfg.tilt_deg, focal_px()));
  fl["model"] = floor_model.ok;
  for (const FloorPoint &p : cfg.points) {
    JsonArray a = fl["points"].add<JsonArray>();
    a.add(std::round(p.y));
    a.add(std::round(p.mm));
    a.add(std::round(p.body));
  }
  if (!result.seq) return;
  const Perception &p = result;
  JsonObject last = out["last"].to<JsonObject>();
  last["seq"] = p.seq;
  last["age_s"] = std::round((now - p.t) * 10) / 10;
  last["synced"] = p.synced;
  last["closeness"] = std::round(p.closeness * 100) / 100;
  last["bottom_floor"] = std::round(p.bottom_floor * 100) / 100;
  last["flatness"] = std::round(p.flatness * 100) / 100;
  last["expansion"] = std::round(p.expansion * 100) / 100;
  if (std::isfinite(p.residual)) last["residual"] = std::round(p.residual * 10) / 10;
  if (std::isfinite(p.yaw_deg)) last["yaw_deg"] = std::round(p.yaw_deg * 10) / 10;
  if (std::isfinite(p.step_mm)) last["step_mm"] = std::round(p.step_mm);
  last["stagnant"] = p.stagnant;
  JsonArray fr = last["free_cm"].to<JsonArray>();
  for (float f : p.free_mm) fr.add(cm(f));
  JsonArray people = last["people"].to<JsonArray>();
  for (const Person &q : p.people) {
    JsonObject o = people.add<JsonObject>();
    JsonArray b = o["box"].to<JsonArray>();
    b.add(q.x1); b.add(q.y1); b.add(q.x2); b.add(q.y2);
    o["score"] = std::round(q.score * 100) / 100;
    o["dist_cm"] = cm(q.dist_mm);
    o["bearing_deg"] = std::round(q.bearing_deg);
  }
}

// Проверка детектора на своём снимке (JPEG до 640×480): рамки в пикселях снимка.
void vision_detect_jpeg(const uint8_t *jpeg, size_t len, JsonObject out) {
  if (!detector) throw ValueError("модель людей ещё не загружена");
  uint8_t *img = (uint8_t *)heap_caps_aligned_alloc(16, DW * DH * 3, MALLOC_CAP_SPIRAM);
  if (!img) throw ValueError("нет памяти");
  double t0 = now_s();
  bool ok = decode_half(jpeg, len, img);
  double t1 = now_s();
  if (!ok) {
    heap_caps_free(img);
    throw ValueError("не JPEG или размер не 640×480");
  }
  xSemaphoreTake(detector_lock, portMAX_DELAY);
  std::list<dl::detect::result_t> found = detector->run(img, DW, DH);
  xSemaphoreGive(detector_lock);
  heap_caps_free(img);
  out["decode_ms"] = std::lround((t1 - t0) * 1000);
  out["detect_ms"] = std::lround((now_s() - t1) * 1000);
  JsonArray people = out["people"].to<JsonArray>();
  for (const dl::detect::result_t &r : found) {
    JsonObject o = people.add<JsonObject>();
    JsonArray b = o["box"].to<JsonArray>();
    for (int v : r.box) b.add(v * 2);
    o["score"] = std::round(r.score * 100) / 100;
  }
}
