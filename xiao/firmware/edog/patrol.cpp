#include "patrol.h"

#include <Preferences.h>
#include <esp_random.h>

#include <atomic>
#include <cmath>
#include <vector>

#include "config.h"
#include "vision.h"

// Состояния. ОСМОТР и ОТХОД поворачивают собаку через «поворот» — он помнит, куда идти дальше.
enum State { OFF, COUNTDOWN, START, PAUSED, WALK, SCAN, TURN, APPROACH, GREET, LEAVE, BACKUP };
static const char *const CODES[] = {"off", "countdown", "start", "paused", "walk", "scan",
                                    "turn", "approach", "greet", "leave", "backup"};
static const char *const TEXTS[] = {"выключен", "старт через", "кивок — начинаю", "пауза: управляет человек",
                                    "иду", "осматриваюсь", "поворачиваю", "человек — подхожу", "приветствую",
                                    "ухожу дальше", "отхожу назад"};

static const int SECTORS = 16;                  // карта вокруг собаки: сектора по 22.5°, 0 — нос, + — по часовой
static const double SECTOR_DEG = 360.0 / SECTORS;
static const double MAP_FORGET_S = 90.0;        // сектор без новых наблюдений забывается
static const double PERSON_FORGET_S = 8.0;
static const int MEMORY_SIZE = 100;
static const double PERSON_MAX_MM = 5000.0;

static Robot *robot;
static std::atomic<bool> active{false};
static State state = OFF;
static double since = 0;
static std::string reason = "";
static bool autostart = true;

// ---------- вспомогательное ----------

static double frand(double a, double b) { return a + (b - a) * (esp_random() / 4294967296.0); }
static int rsign() { return esp_random() & 1 ? 1 : -1; }
static double wrap(double deg) {   // −180 … 180
  deg = std::fmod(deg + 180, 360);
  return deg < 0 ? deg + 180 : deg - 180;
}
static double clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

// ---------- карта вокруг собаки ----------
// В секторе — ближайшая точка препятствия (угол, расстояние) и когда её видели. Собака поворачивает —
// точки поворачиваются, идёт вперёд — сдвигаются назад; потом раскладываются по секторам заново.

struct MapPoint {
  double deg = 0, mm = NAN, t = -1e9;
};
struct PersonPoint {
  double deg, mm, t;
};
static MapPoint sectors[SECTORS];
static std::vector<PersonPoint> people;

static int sector_of(double deg) {
  int s = (int)std::lround(wrap(deg) / SECTOR_DEG);
  return (s % SECTORS + SECTORS) % SECTORS;
}

static void map_move(double turn_deg, double forward_mm) {
  if (!turn_deg && !forward_mm) return;
  auto move = [&](double &deg, double &mm) {
    deg -= turn_deg;
    if (forward_mm && std::isfinite(mm)) {
      double a = deg * M_PI / 180;
      double x = mm * std::cos(a) - forward_mm, y = mm * std::sin(a);
      mm = std::hypot(x, y);
      deg = std::atan2(y, x) * 180 / M_PI;
    }
    deg = wrap(deg);
  };
  MapPoint moved[SECTORS];
  for (MapPoint &p : sectors) {
    if (!std::isfinite(p.mm)) continue;
    MapPoint q = p;
    move(q.deg, q.mm);
    MapPoint &dst = moved[sector_of(q.deg)];
    if (!std::isfinite(dst.mm) || q.mm < dst.mm) dst = q;
  }
  for (int i = 0; i < SECTORS; i++) sectors[i] = moved[i];
  for (PersonPoint &p : people) move(p.deg, p.mm);
}

static void map_observe(const Perception &p, double now) {
  double best[SECTORS];
  double deg[SECTORS];
  for (int i = 0; i < SECTORS; i++) best[i] = NAN;
  for (int c = 0; c < FREE_COLS; c++) {
    if (!std::isfinite(p.free_mm[c])) continue;
    double a = vision_col_bearing_deg(c);
    int s = sector_of(a);
    if (!std::isfinite(best[s]) || p.free_mm[c] < best[s]) {
      best[s] = p.free_mm[c];
      deg[s] = a;
    }
  }
  for (int i = 0; i < SECTORS; i++)
    if (std::isfinite(best[i])) sectors[i] = {deg[i], best[i], now};
  if (!p.people.empty()) {
    people.clear();
    for (const Person &q : p.people) people.push_back({q.bearing_deg, std::isfinite(q.dist_mm) ? q.dist_mm : NAN, now});
  }
}

static void map_forget(double now) {
  for (MapPoint &p : sectors)
    if (now - p.t > MAP_FORGET_S) p = MapPoint();
  for (size_t i = 0; i < people.size();)
    if (now - people[i].t > PERSON_FORGET_S) people.erase(people.begin() + i);
    else i++;
}

// ---------- память мест ----------
// Миниатюры увиденного; новизна вида — насколько он не похож на всё, что собака видела за PATROL_MEMORY_S.

struct Memory {
  float thumb[THUMB_W * THUMB_H];
  double t;
};
static std::vector<Memory> memory;

static double novelty(const float *thumb, double now) {
  double best = INFINITY;
  for (const Memory &m : memory) {
    if (now - m.t > PATROL_MEMORY_S) continue;
    double d = 0;
    for (int i = 0; i < THUMB_W * THUMB_H; i++) d += std::fabs(thumb[i] - m.thumb[i]);
    best = std::min(best, d / (THUMB_W * THUMB_H));
  }
  return std::isfinite(best) ? clamp(best / 0.9, 0, 1) : 1.0;
}

static void remember(const float *thumb, double now) {
  for (size_t i = 0; i < memory.size();)
    if (now - memory[i].t > PATROL_MEMORY_S) memory.erase(memory.begin() + i);
    else i++;
  if (memory.size() >= MEMORY_SIZE) memory.erase(memory.begin());
  Memory m;
  memcpy(m.thumb, thumb, sizeof m.thumb);
  m.t = now;
  memory.push_back(m);
}

// ---------- состояние патруля ----------

static double heading = 0;                       // курс от включения патруля, °, + — по часовой
static double turn_dps = PATROL_TURN_DPS;        // разворот на месте при turn = 1, уточняется по камере
static double last_tick = 0;
static uint32_t last_seq = 0, seen_presses = 0, seen_estops = 0;
static bool api_activity = false;
static double last_activity = 0, countdown_end = 0;
static double cmd_vx = 0, cmd_turn = 0;          // что патруль велит сейчас

// поворот
static double turn_target = 0, turn_start = 0, turn_expected = 0;
static State turn_next = WALK;
static bool turn_next_blocked = false;

// ходьба
static double seg_end = 0, walk_t0 = 0, last_remember = 0;
static bool wall_mode = false;
static double wall_until = 0;
static int close_run = 0, near_run = 0;
static double free_center = NAN, closeness = 0;

// осмотр
struct View {
  double heading, free, novelty, closeness;
};
static std::vector<double> scan_plan;
static std::vector<View> views;
static size_t scan_idx = 0;
static double scan_from = 0, view_since = 0;     // откуда пришли (курс), когда встали на вид
static bool scan_full = false, last_tight = false;
static int scans = 0;

// человек
static bool seen[3] = {false, false, false};
static int seen_i = 0;
static double ignore_until = 0;
struct Target {
  double heading, mm, t, box_h;
};
static Target target = {0, NAN, -1e9, 0};

// приветствие, отход
static int greet_step = 0;
static double greet_t = 0, backup_until = 0;
static bool backup_then_greet = false;
static int greetings = 0, stucks = 0;

static void enter(State s);

static void turn_to(double abs_heading, State next, bool blocked = false) {
  turn_target = abs_heading;
  turn_next = next;
  turn_next_blocked = blocked;
  turn_start = now_s();
  turn_expected = std::fabs(wrap(abs_heading - heading)) / turn_dps;
  enter(TURN);
}

static void start_scan(bool blocked) {
  scans++;
  scan_full = (blocked && last_tight) || scans % 3 == 0;
  scan_plan.clear();
  views.clear();
  scan_from = heading + 180;
  if (scan_full) {
    for (int i = 0; i < 8; i++) scan_plan.push_back(heading + 45 * i);
  } else {
    for (double d : {0.0, 45.0, 90.0, -45.0, -90.0}) scan_plan.push_back(heading + d);
  }
  scan_idx = 0;
  turn_to(scan_plan[0], SCAN);
}

static void backup(double seconds, bool then_greet) {
  backup_until = now_s() + seconds;
  backup_then_greet = then_greet;
  enter(BACKUP);
}

static void enter(State s) {
  state = s;
  since = now_s();
  active = s != OFF;
  double now = since;
  switch (s) {
    case WALK:
      seg_end = now + (wall_mode ? frand(15, 20) : frand(2, 6));
      walk_t0 = now;
      close_run = near_run = 0;
      free_center = NAN;
      closeness = 0;
      break;
    case SCAN:
      view_since = now;
      break;
    case GREET:
    case START:
      greet_step = 0;
      greet_t = now;
      break;
    case LEAVE:
      ignore_until = now + PATROL_IGNORE_S;
      turn_to(heading + rsign() * frand(120, 180), WALK);
      return;
    default:
      break;
  }
}

static void stop(const char *why) {
  if (state != OFF) Serial.printf("патруль: выключен (%s)\n", why);
  reason = why;
  enter(OFF);
}

bool patrol_active() { return active; }

void patrol_stop(const char *why) { stop(why); }

static void snapshot() {
  seen_presses = robot->pad_presses;
  seen_estops = robot->estops;
  api_activity = false;
}

void patrol_start() {
  if (robot->updating) throw ValueError("идёт обновление прошивки");
  if (!robot->enabled) robot->set_enabled(true);
  snapshot();
  reason = "";
  heading = 0;
  wall_mode = false;
  for (MapPoint &p : sectors) p = MapPoint();
  people.clear();
  Serial.println("патруль: включён");
  enter(START);
}

void patrol_toggle() {
  if (state != OFF) stop("выключен кнопкой");
  else {
    try {
      patrol_start();
    } catch (const std::exception &e) {
      Serial.printf("патруль: %s\n", e.what());
    }
  }
}

void patrol_user_activity() { api_activity = true; }

void patrol_set_autostart(bool on) {
  autostart = on;
  Preferences p;
  p.begin("edog", false);
  p.putBool("patrol_auto", on);
  p.end();
}

void patrol_autostart() {
  if (!autostart) return;
  RobotLock lock;
  try {
    if (!robot->enabled) robot->set_enabled(true);
  } catch (const std::exception &e) {
    Serial.printf("патруль: авто-старт не удался (%s)\n", e.what());
    return;
  }
  snapshot();
  countdown_end = now_s() + PATROL_COUNTDOWN_S;
  heading = 0;
  Serial.println("патруль: авто-старт после включения");
  enter(COUNTDOWN);
}

// ---------- шаг автомата ----------

static bool human_active() {
  Robot &r = *robot;
  bool pad = r.pad_drive[0] || r.pad_drive[1] || r.tilt[0] || r.tilt[1] || r.height_rate || r.pad_presses != seen_presses;
  bool act = pad || r.web_driving() || api_activity;
  seen_presses = r.pad_presses;
  api_activity = false;
  return act;
}

static double center_free(const Perception &p) {
  double m = NAN;
  for (int c = FREE_COLS / 2 - 2; c < FREE_COLS / 2 + 2; c++)
    if (std::isfinite(p.free_mm[c]) && !(m <= p.free_mm[c])) m = p.free_mm[c];
  return m;
}

// Новый результат зрения: карта, человек, ходьба.
static void on_perception(const Perception &p, double now) {
  map_observe(p, now);
  // человек: 2 из 3 последних анализов
  const Person *best = nullptr;
  for (const Person &q : p.people) {
    if (std::isfinite(q.dist_mm) && q.dist_mm > PERSON_MAX_MM) continue;
    if (!best || (q.y2 - q.y1) > (best->y2 - best->y1)) best = &q;
  }
  seen[seen_i] = best != nullptr;
  seen_i = (seen_i + 1) % 3;
  if (best) target = {heading + best->bearing_deg, best->dist_mm, now, (double)(best->y2 - best->y1)};
  int count = seen[0] + seen[1] + seen[2];
  bool hunting = state == WALK || state == SCAN || state == TURN;   // после приветствия — ignore_until
  if (count >= 2 && best && now >= ignore_until && hunting) {
    Serial.printf("патруль: человек, %.0f°, %.0f мм\n", wrap(target.heading - heading), target.mm);
    enter(APPROACH);
    return;
  }
  // скорость разворота — по сдвигу картинки между синхронными кадрами
  if (p.synced && std::isfinite(p.yaw_deg) && std::fabs(p.turn) > 0.3 && p.dt > 0) {
    double measured = p.yaw_deg / p.dt / p.turn;
    if (measured > 10 && measured < 200) turn_dps = clamp(0.8 * turn_dps + 0.2 * measured, 15, 150);
  }
  if (state == WALK && p.synced && p.t > walk_t0) {
    free_center = center_free(p);
    closeness = p.closeness;
    close_run = p.closeness > 0.7 ? close_run + 1 : 0;
    near_run = std::isfinite(free_center) && free_center < 250 ? near_run + 1 : 0;
    if (now - last_remember > 3) {
      remember(p.thumb, now);
      last_remember = now;
    }
    if (p.stagnant) {
      stucks++;
      Serial.println("патруль: упёрлась — назад");
      backup(1.2, false);
    } else if (p.closeness > 0.9 || close_run >= 2 || near_run >= 2) {
      Serial.printf("патруль: близко (закрытость %.2f, свободно %.0f мм)\n", p.closeness, free_center);
      if (wall_mode) turn_to(heading - frand(60, 90), WALK);
      else start_scan(true);
    }
  }
  if (state == SCAN && !p.synced && p.t > view_since + VISION_SETTLE_S && robot_still_since() < p.t) {
    double nov = novelty(p.thumb, now);
    remember(p.thumb, now);
    views.push_back({heading, center_free(p), nov, p.closeness});
    scan_idx++;
    if (scan_idx < scan_plan.size()) {
      turn_to(scan_plan[scan_idx], SCAN);
      return;
    }
    // выбор направления: свободно × ново × немного случайности, разворот назад — со штрафом
    const View *pick = nullptr;
    double best_score = -1e9;
    for (const View &v : views) {
      double f = std::isfinite(v.free) ? v.free : 600;
      if (f < 300 || v.closeness > 0.7) continue;
      double score = std::min(f, 1500.0) / 1500 * (0.4 + 0.6 * v.novelty) + frand(0, 0.15);
      if (std::fabs(wrap(v.heading - scan_from)) < 45) score -= 0.3;
      if (score > best_score) {
        best_score = score;
        pick = &v;
      }
    }
    if (!pick) {
      if (!scan_full) {
        last_tight = true;
        start_scan(true);
      } else {
        backup(1.2, false);
      }
      return;
    }
    last_tight = std::isfinite(pick->free) && pick->free < 600;
    if (scans % 4 == 0 && !wall_mode) {   // время от времени — вдоль стены, чтобы выйти в другую комнату
      wall_mode = true;
      wall_until = now + frand(15, 20);
    }
    turn_to(pick->heading, WALK);
  }
}

static void tick() {
  Robot &r = *robot;
  double now = now_s();
  double dt = last_tick ? std::min(0.2, now - last_tick) : 0;
  last_tick = now;

  // курс и путь по командам патруля (пока ведёт патруль)
  double vx = 0, turn = 0;
  {
    Motion m;
    if (robot_motion_at(now - 0.012, m) && m.walking) {
      vx = m.vx;
      turn = m.turn;
    }
  }
  double dturn = turn * turn_dps * dt;
  double speed = 2 * r.gait_cfg().stride_mm * r.gait_cfg().gait_hz * PATROL_SLIP;
  map_move(dturn, vx * speed * dt);
  heading += dturn;
  map_forget(now);

  if (state == OFF) return;
  if (r.estops != seen_estops) {
    seen_estops = r.estops;
    stop("аварийный стоп");
    return;
  }
  if (!r.enabled && !r.powering_down()) {
    stop("серво выключены");
    return;
  }
  if (r.updating) {
    stop("обновление прошивки");
    return;
  }
  if (human_active()) {
    last_activity = now;
    if (state == COUNTDOWN) {
      stop("старт отменён человеком");
      return;
    }
    if (state != PAUSED) {
      Serial.println("патруль: пауза — управляет человек");
      enter(PAUSED);
    }
  }

  Perception p;
  if (vision_latest(p) && p.seq != last_seq) {
    last_seq = p.seq;
    if (state != PAUSED && state != COUNTDOWN && state != START) on_perception(p, now);
    else map_observe(p, now);
  }

  cmd_vx = cmd_turn = 0;
  switch (state) {
    case COUNTDOWN:
      if (std::isnan(robot_still_since())) countdown_end = std::max(countdown_end, now + PATROL_COUNTDOWN_S);
      if (now >= countdown_end) enter(START);
      return;
    case PAUSED:
      if (now - last_activity >= PATROL_COUNTDOWN_S && !r.moving()) enter(START);
      return;
    case START:
      if (greet_step == 0) {   // кивок: «начинаю»
        try {
          r.start_move("nod");
        } catch (const std::exception &e) {
          stop(e.what());
          return;
        }
        greet_step = 1;
      } else if (!r.moving()) {
        start_scan(false);
      }
      break;
    case TURN: {
      double rem = wrap(turn_target - heading);
      if (std::fabs(rem) < 6 || now - turn_start > 2 * turn_expected + 2) {
        enter(turn_next);
        break;
      }
      cmd_turn = (rem > 0 ? 1 : -1) * (std::fabs(rem) > 25 ? 0.9 : 0.6);
      break;
    }
    case SCAN:
      if (now - view_since > 4) {   // зрение молчит — вид без данных
        views.push_back({heading, NAN, 0.5, 0});
        scan_idx++;
        if (scan_idx < scan_plan.size()) turn_to(scan_plan[scan_idx], SCAN);
        else turn_to(heading + rsign() * frand(45, 135), WALK);
      }
      break;
    case WALK: {
      if (wall_mode && now > wall_until) wall_mode = false;
      if (now > seg_end) {
        start_scan(false);
        break;
      }
      double f = std::isfinite(free_center) ? clamp(free_center / 800, 0.35, 1) : 0.6;
      double c = closeness > 0.4 ? clamp(1 - (closeness - 0.4) / 0.3 * 0.65, 0.35, 1) : 1;
      cmd_vx = f * c;
      cmd_turn = wall_mode ? 0.2 : 0;
      break;
    }
    case APPROACH: {
      if (now - target.t > 2.0) {
        Serial.println("патруль: человек пропал из вида");
        start_scan(false);
        break;
      }
      double rel = wrap(target.heading - heading);
      double mm = std::isfinite(target.mm) ? target.mm : 2000;
      if (std::fabs(rel) > 12) {
        cmd_turn = rel > 0 ? 0.6 : -0.6;
      } else if (mm < PATROL_TOO_CLOSE_MM) {
        backup(0.8, true);
      } else if (mm < PATROL_GREET_MM || target.box_h > 0.6 * 480) {
        enter(GREET);
      } else {
        cmd_vx = clamp((mm - PATROL_GREET_MM) / 1200 + 0.4, 0.4, 0.8);
        cmd_turn = clamp(rel / 30, -0.3, 0.3);
      }
      break;
    }
    case GREET:
      if (greet_step == 0) {
        double still = robot_still_since();
        if (!std::isnan(still) && now - still > 0.3) {
          static const char *const MOVES_GREET[] = {"joy", "wag", "bow"};
          const char *m = MOVES_GREET[esp_random() % 3];
          try {
            r.start_move(m);
            Serial.printf("патруль: приветствие — %s\n", m);
          } catch (const std::exception &e) {
            Serial.printf("патруль: %s\n", e.what());
          }
          greet_step = 1;
          greet_t = now;
          greetings++;
        }
      } else if (!r.moving() && now - greet_t > 0.6) {
        enter(LEAVE);
      }
      break;
    case BACKUP:
      if (now < backup_until) {
        cmd_vx = -1;
      } else if (backup_then_greet) {
        enter(GREET);
      } else {
        scan_plan.clear();   // после разворота — осмотр с нового места, план строится тогда
        turn_to(heading + rsign() * frand(90, 150), SCAN);
      }
      break;
    default:
      break;
  }
  // разворот после отхода ведёт в ОСМОТР без плана: план строится, когда разворот закончился
  if (state == SCAN && scan_plan.empty()) {
    start_scan(true);
    return;
  }
  r.set_patrol_drive(cmd_vx, cmd_turn);
}

static void patrol_task(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
    try {
      RobotLock lock;
      tick();
    } catch (const std::exception &e) {
      Serial.printf("патруль: %s\n", e.what());
    }
  }
}

void patrol_begin(Robot &r) {
  robot = &r;
  Preferences p;
  if (p.begin("edog", true)) {
    autostart = p.getBool("patrol_auto", true);
    p.end();
  }
  // стек в PSRAM: внутренней памяти мало, а задача патруля во флеш не пишет
  xTaskCreatePinnedToCoreWithCaps(patrol_task, "patrol", 8192, nullptr, 2, nullptr, 1, MALLOC_CAP_SPIRAM);
}

// ---------- вывод ----------

static int cm(double mm) { return std::isfinite(mm) ? (int)std::lround(mm / 10) : -1; }

static double target_deg() {   // куда собирается: относительно носа
  if (state == TURN) return wrap(turn_target - heading);
  if (state == APPROACH) return wrap(target.heading - heading);
  if (state == WALK || state == BACKUP) return state == BACKUP ? 180 : 0;
  return NAN;
}

void patrol_state(JsonObject out) {
  double now = now_s();
  out["state"] = CODES[state];
  out["text"] = TEXTS[state];
  if (!reason.empty()) out["reason"] = reason;
  out["autostart"] = autostart;
  if (state == COUNTDOWN) out["left_s"] = std::max(0.0, std::ceil(countdown_end - now));
  if (state == PAUSED) out["left_s"] = std::max(0.0, std::ceil(last_activity + PATROL_COUNTDOWN_S - now));
  out["heading_deg"] = std::lround(wrap(heading));
  out["turn_dps"] = std::lround(turn_dps);
  out["wall"] = wall_mode;
  out["greetings"] = greetings;
  out["scans"] = scans;
  out["stucks"] = stucks;
  if (now < ignore_until) out["ignore_s"] = std::ceil(ignore_until - now);
  out["vx"] = std::round(cmd_vx * 100) / 100;
  out["turn"] = std::round(cmd_turn * 100) / 100;
  JsonObject map = out["map"].to<JsonObject>();
  map["fov_deg"] = vision_hfov_deg();
  double td = target_deg();
  if (std::isfinite(td)) map["target_deg"] = std::lround(td);
  JsonArray d = map["cm"].to<JsonArray>(), age = map["age_s"].to<JsonArray>();
  for (const MapPoint &p : sectors) {
    d.add(cm(p.mm));
    if (std::isfinite(p.mm)) age.add(std::lround(now - p.t));
    else age.add(nullptr);
  }
  JsonArray pp = map["people"].to<JsonArray>();
  for (const PersonPoint &p : people) {
    JsonObject o = pp.add<JsonObject>();
    o["deg"] = std::lround(p.deg);
    o["cm"] = cm(p.mm);
  }
}

int patrol_headers(char *buf, size_t cap) {
  RobotLock lock;
  double now = now_s();
  int n = snprintf(buf, cap, "X-Patrol: state=%s;greet=%d;heading=%ld", CODES[state], greetings, std::lround(wrap(heading)));
  if (state == COUNTDOWN || state == PAUSED) {
    double end = state == COUNTDOWN ? countdown_end : last_activity + PATROL_COUNTDOWN_S;
    n += snprintf(buf + n, cap - n, ";left=%d", (int)std::max(0.0, std::ceil(end - now)));
  }
  n += snprintf(buf + n, cap - n, "\r\nX-Map: f=%d", (int)vision_hfov_deg());
  double td = target_deg();
  if (std::isfinite(td)) n += snprintf(buf + n, cap - n, ";t=%ld", std::lround(td));
  n += snprintf(buf + n, cap - n, ";d=");
  for (int i = 0; i < SECTORS && n < (int)cap - 8; i++) n += snprintf(buf + n, cap - n, "%s%d", i ? "," : "", cm(sectors[i].mm));
  n += snprintf(buf + n, cap - n, ";a=");
  for (int i = 0; i < SECTORS && n < (int)cap - 8; i++)
    n += snprintf(buf + n, cap - n, "%s%ld", i ? "," : "", std::isfinite(sectors[i].mm) ? std::lround(now - sectors[i].t) : -1L);
  n += snprintf(buf + n, cap - n, ";p=");
  for (size_t i = 0; i < people.size() && n < (int)cap - 16; i++)
    n += snprintf(buf + n, cap - n, "%s%ld:%d", i ? "," : "", std::lround(people[i].deg), cm(people[i].mm));
  n += snprintf(buf + n, cap - n, "\r\n");
  return std::min(n, (int)cap);
}
