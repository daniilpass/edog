#include "robot.h"

#include <Preferences.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdarg>
#include <functional>

static const char *const MODES[] = {"stand", "manual"};
static const char *const TEST_ACTIONS[] = {"center", "ref", "sweep", "stop"};
static const double SWEEP_SPEED_DPS = 60;
// Резкие шаги движений без ограничения скорости: цель сразу в конце хода, серво жмёт
// с полным моментом с первого мгновения (при плавной цели ошибка мала — и момент тоже).
static const double MOVE_SPEED_DPS = INFINITY;
static const double TRIM_MAX_DEG = 10.0;          // поправка наклона корпуса на огрехи сборки, °
static const double ROLL_STEP_DEG = 1.0;          // бамперы геймпада: шаг постоянной добавки к крену, °
static const double WALK_RAMP_S = 0.5;            // разгон шага из стойки и уход лап под ось бедра при ходьбе назад, с
static const double SCALE_MIN_SPAN_DEG = 10.0;    // масштаб пишется не ближе этого к монтажному положению
static const double SCALE_RANGE[2] = {0.7, 1.3};  // масштаб за этими пределами — скорее промах при калибровке
static const double POWER_DOWN_TIMEOUT_S = 5.0;   // не дошла до монтажного положения за это время — сигнал снимается всё равно
// Рысь: диагональные пары в противофазе. Фаза цикла, в которую нога отрывается от пола: FL, FR, RL, RR.
static const double TROT_START[4] = {0.0, 0.5, 0.5, 0.0};
static const char *const JOINTS[2] = {"hip", "knee"};
const char *const PAD_AXES[6] = {"drive", "turn", "pitch", "roll", "height_down", "height_up"};

std::string strf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  return buf;
}

double now_s() { return esp_timer_get_time() / 1e6; }

static double r2(double v, int digits = 2) {
  double k = std::pow(10.0, digits);
  return std::round(v * k) / k;
}

static double pmod1(double v) {   // v % 1.0 как в Python: всегда 0 … 1
  double r = std::fmod(v, 1.0);
  return r < 0 ? r + 1.0 : r;
}

static SemaphoreHandle_t robot_mutex;
void robot_lock_init() { robot_mutex = xSemaphoreCreateRecursiveMutex(); }
RobotLock::RobotLock() { xSemaphoreTakeRecursive(robot_mutex, portMAX_DELAY); }
RobotLock::~RobotLock() { xSemaphoreGiveRecursive(robot_mutex); }

// ---------- история движения ----------
// Кадр камеры приходит с отметкой времени съёмки; по ней зрение узнаёт фазу шага в момент съёмки.
// Контур пишет запись каждый шаг (100 Гц), без общего замка робота — только короткий спинлок.

static const int MOTION_LOG = 64;   // 0.64 с при 100 Гц
static Motion motion_log[MOTION_LOG];
static int motion_head = 0;
static double still_since = NAN;
static portMUX_TYPE motion_mux = portMUX_INITIALIZER_UNLOCKED;

bool robot_motion_at(double t, Motion &out) {
  bool found = false;
  double best = INFINITY;
  portENTER_CRITICAL(&motion_mux);
  for (const Motion &m : motion_log) {
    double d = std::fabs(m.t - t);
    if (m.t > 0 && d < best) {
      best = d;
      out = m;
      found = true;
    }
  }
  portEXIT_CRITICAL(&motion_mux);
  if (!found || best > 0.02) return false;
  // между записями фаза идёт равномерно
  if (out.walking) out.cycles += out.gait_hz * (t - out.t);
  out.t = t;
  return true;
}

double robot_still_since() {
  portENTER_CRITICAL(&motion_mux);
  double s = still_since;
  portEXIT_CRITICAL(&motion_mux);
  return s;
}

// ---------- шаг ----------

// Смещение лапы (вперёд, вверх) от нейтрали в системе корпуса, мм, в фазе цикла phase.
static void leg_step(int leg, double phase, double stride, double lift, double &dx, double &dz) {
  double p = pmod1(phase - TROT_START[leg]);
  if (p < 0.5) {   // перенос: лапа в воздухе идёт вперёд
    double s = 2 * p;
    dx = stride * (s - 0.5);
    dz = lift * std::sin(M_PI * s);
    return;
  }
  double s = 2 * p - 1;   // опора: лапа на земле уходит назад
  dx = stride * (0.5 - s);
  dz = 0.0;
}

static double gait_height(const GaitCfg &g, double fallback) { return std::isnan(g.height_mm) ? fallback : g.height_mm; }

// Чем профиль отличается на суставах: размах бедра и колена, °, и пиковая скорость, °/с, при шаге
// полной длины на высоте профиля, по самой быстрой ноге. limited — пик выше ограничения скорости серво.
static void gait_stats(const GaitCfg &g, JsonObject out, int samples = 400) {
  double height = gait_height(g, STAND_HEIGHT_MM);
  double swing[2] = {0, 0}, peak[2] = {0, 0};
  std::vector<double> a[2] = {std::vector<double>(samples + 1), std::vector<double>(samples + 1)};
  for (int leg = 0; leg < 4; leg++) {
    for (int i = 0; i <= samples; i++) {
      double dx, dz, x, z;
      leg_step(leg, pmod1((double)i / samples), g.stride_mm, g.lift_mm, dx, dz);
      double foot[3] = {g.foot_x_mm + dx, 0.0, dz}, hip[3] = {0, 0, 0};
      foot_in_leg(foot, hip, height, 0, 0, x, z);
      ik(x, z, FEMUR_MM, TIBIA_MM, LEG_DEFS[0].knee_dir, TIBIA_BEND_DEG, a[0][i], a[1][i]);
    }
    double dt = 1.0 / (g.gait_hz * samples);
    for (int j = 0; j < 2; j++) {
      double lo = INFINITY, hi = -INFINITY, pk = 0;
      for (int i = 0; i <= samples; i++) {
        lo = std::fmin(lo, a[j][i]);
        hi = std::fmax(hi, a[j][i]);
        if (i) pk = std::fmax(pk, std::fabs(a[j][i] - a[j][i - 1]));
      }
      swing[j] = std::fmax(swing[j], r2(hi - lo, 1));
      peak[j] = std::fmax(peak[j], std::round(pk / dt));
    }
  }
  out["hip_swing"] = swing[0];
  out["hip_peak_dps"] = (int)peak[0];
  out["knee_swing"] = swing[1];
  out["knee_peak_dps"] = (int)peak[1];
  out["speed_dps"] = MAX_SPEED_DPS;
  // корпус едет со скоростью лапы в опоре: шаг за полцикла
  out["speed_mm_s"] = r2(2 * g.stride_mm * g.gait_hz, 1);
  out["limited"] = std::fmax(peak[0], peak[1]) > MAX_SPEED_DPS;
}

// ---------- лимит тока ----------

int Power::max_servos() const {
  if (limit_a <= 0) return -1;
  return (int)std::floor(limit_a / servo_a + 1e-9);
}

void Power::validate() const {
  if (limit_a < 0) throw ValueError("лимит тока: ≥ 0 (0 — без ограничения)");
  if (servo_a <= 0) throw ValueError("ток серво: > 0");
}

void Power::check(int installed) const {
  int n = max_servos();
  if (n >= 0 && installed > n)
    throw ValueError(strf("лимит тока %g А: подключить можно не больше %d серво по %g А, а нужно %d (%.1f А)",
                          limit_a, n, servo_a, installed, installed * servo_a));
}

// ---------- демо ----------
// Ноги по кругу шагают в воздухе, по несколько полных циклов шага каждая группа. Размер группы —
// по лимиту тока: сигнал получают только серво шагающих ног. Каждая группа плавно выходит из
// монтажного положения и возвращается в него, поэтому на стыке сигнал переключается на следующую.

static const double RAMP_S = 0.4;   // выход из монтажного положения в шаг и обратно, с
static const char *const LEG_NAMES[4] = {"FL", "FR", "RL", "RR"};

static std::string pick_program(int max_servos) {
  if (max_servos < 0 || max_servos >= 8) return "all";
  if (max_servos >= 4) return "pairs";
  if (max_servos >= 2) return "solo";
  throw ValueError(strf("демо: лимит тока позволяет %d серво, а нужно хотя бы 2 (одна нога)", max_servos));
}

static double envelope(double t, double length) {   // 0 → 1 за RAMP_S в начале, 1 → 0 в конце (плавно)
  double x = std::fmin(std::fmin(t, length - t), RAMP_S) / RAMP_S;
  if (x <= 0) return 0.0;
  return x * x * (3 - 2 * x);
}

class Demo {
 public:
  using Walk = std::function<void(int, double, double &, double &)>;
  struct Segment { std::vector<int> legs; double length; };

  Demo(const std::string &program, int cycles, double cycle_s, int laps, double ref_hip, double ref_knee, Walk walk)
      : program(program), ref_hip_(ref_hip), ref_knee_(ref_knee), walk_(walk) {
    if (cycles < 1) throw ValueError("демо: на ногу хотя бы один цикл шага");
    const int ORDER[4] = {1, 3, 2, 0};   // FR, RR, RL, FL — по кругу, начиная с правой передней
    std::vector<std::vector<int>> groups;
    if (program == "solo") {
      title = "по одной ноге";
      for (int leg : ORDER) groups.push_back({leg});
    } else if (program == "pairs") {
      title = "диагональными парами";
      groups = {{1, 2}, {0, 3}};
    } else {
      title = "все ноги";
      groups = {{1, 3, 2, 0}};
    }
    auto length = [&](int n) { return n * cycle_s + 2 * RAMP_S; };
    // Одна группа на всех — шагает без пауз столько же циклов, сколько длился бы круг по ногам.
    if (groups.size() == 1) segments_.push_back({groups[0], length(cycles * 4 * laps)});
    else for (int i = 0; i < laps; i++) for (auto &g : groups) segments_.push_back({g, length(cycles)});
    for (auto &s : segments_) total_ += s.length;
  }

  bool finished() const { return index_ >= segments_.size(); }
  const std::vector<int> &legs() const {
    static const std::vector<int> none;
    return finished() ? none : segments_[index_].legs;
  }

  // Сдвинуть время на dt секунд; углы суставов (hip, knee) шагающих ног.
  void tick(double dt, std::function<void(int, double, double)> set) {
    if (finished()) return;
    const Segment &seg = segments_[index_];
    t_ += dt;
    done_ += dt;
    if (t_ >= seg.length) {
      index_++;
      t_ = 0;
      for (int leg : seg.legs) set(leg, ref_hip_, ref_knee_);
      return;
    }
    double e = envelope(t_, seg.length);
    for (int leg : seg.legs) {
      double hip, knee;
      walk_(leg, t_ - RAMP_S, hip, knee);   // полные циклы — между разгоном и торможением
      set(leg, ref_hip_ + e * (hip - ref_hip_), ref_knee_ + e * (knee - ref_knee_));
    }
  }

  void state(JsonObject out) const {
    out["program"] = program;
    out["title"] = title;
    JsonArray legs_out = out["legs"].to<JsonArray>();
    for (int leg : legs()) legs_out.add(LEG_NAMES[leg]);
    out["number"] = std::min(index_ + 1, segments_.size());
    out["numbers"] = segments_.size();
    out["progress"] = r2(std::fmin(done_ / total_, 1.0), 3);
  }

  std::string program, title;

 private:
  double ref_hip_, ref_knee_;
  Walk walk_;
  std::vector<Segment> segments_;
  size_t index_ = 0;
  double t_ = 0, total_ = 0, done_ = 0;
};

// ---------- робот ----------

Robot::Robot(Driver &driver) : driver_(driver), guard_(TIBIA_BEND_DEG, FRAME_MARGIN_MM) {
  for (const LegDef &leg : LEG_DEFS) {
    for (int j = 0; j < 2; j++) {
      ServoCfg c;
      c.leg = leg.name;
      c.joint = JOINTS[j];
      c.id = c.leg + "_" + c.joint;
      c.channel = j ? leg.knee_channel : leg.hip_channel;
      c.dir = j ? leg.knee_dir_servo : leg.hip_dir;
      c.installed = false;   // сигнал только на подключённые серво; включается в портале по мере сборки
      c.ref_deg = j ? KNEE_REF_DEG : HIP_REF_DEG;
      c.load_deg = j ? KNEE_LOAD_DEG : HIP_LOAD_DEG;
      c.range_deg = SERVO_RANGE_DEG;
      c.pulse_min_us = SERVO_PULSE_MIN_US;
      c.pulse_max_us = SERVO_PULSE_MAX_US;
      c.min_deg = SERVO_MIN_DEG;
      c.max_deg = SERVO_MAX_DEG;
      // в центре серво сустав стоит в монтажном положении; уточняется калибровкой
      c.zero = c.range_deg / 2 - c.dir * c.ref_deg;
      Servo s;
      s.cfg = c;
      s.target = s.current = c.zero;
      servos_.push_back(s);
    }
  }
  for (int i = 0; i < GAIT_COUNT; i++)
    if (!strcmp(GAITS[i].name, DEFAULT_GAIT)) gait_ = i;
  power_ = {POWER_LIMIT_A, POWER_SERVO_A};
  load_calibration();
  body_ = pose("stand");
  update_targets(0.0);
  for (Servo &s : servos_) s.current = s.target;
}

// ---------- команды ----------

Body Robot::pose(const std::string &name) const {
  // у каждого профиля шага своя стойка: на его высоте и с его выносом лап
  if (name == "stand") return {gait_height(gait(), STAND_HEIGHT_MM), 0, 0, gait().foot_x_mm};
  if (name == "sit") return {STAND_HEIGHT_MM - 10, 20};
  if (name == "lie") return {MIN_HEIGHT_MM};
  if (name == "high") return {MAX_HEIGHT_MM - 4};
  throw KeyError("'" + name + "'");
}

static const char *const BODY_POSES[] = {"stand", "sit", "lie", "high"};
// Позы, заданные углами суставов, а не корпусом: mount — монтажное положение, load — поза под нагрузкой.
static const char *const JOINT_POSES[] = {"mount", "load"};
static const char *const MOVES[] = {"jump", "joy", "nod", "wag", "bow"};

bool Robot::is_pose(const std::string &name) const {
  for (auto p : BODY_POSES) if (name == p) return true;
  for (auto p : JOINT_POSES) if (name == p) return true;
  return false;
}

bool Robot::is_move(const std::string &name) const {
  for (auto m : MOVES) if (name == m) return true;
  return false;
}

// Движения: шаги (корпус, сколько держать, с; скорость серво, °/с, или NAN — обычная).
// После движения собака встаёт в стойку.
std::vector<MoveStep> Robot::moves(const std::string &name) const {
  if (name == "jump")   // прыжок вверх: неглубоко присесть, резкий толчок, потом стойка
    return {{{STAND_HEIGHT_MM - 10}, 0.25, NAN},
            // толчок до полностью прямой ноги (IK прижмёт к досягаемости)
            {{FEMUR_MM + TIBIA_MM}, 0.12, MOVE_SPEED_DPS}};
  if (name == "joy") {   // радость: высоко ↔ стойка 4 раза, на полной скорости серво без пауз
    std::vector<MoveStep> steps;
    for (int i = 0; i < 4; i++) {
      steps.push_back({pose("high"), 0.12, MOVE_SPEED_DPS});
      steps.push_back({pose("stand"), 0.12, MOVE_SPEED_DPS});
    }
    return steps;
  }
  Body stand = pose("stand");
  auto tilted = [&](double pitch, double roll) {
    Body b = stand;
    b.pitch = pitch;
    b.roll = roll;
    return b;
  };
  if (name == "nod")   // кивок: нос вниз и обратно — «начинаю патруль»
    return {{tilted(-10, 0), 0.3, NAN}, {tilted(4, 0), 0.25, NAN}, {stand, 0.2, NAN}};
  if (name == "wag") {   // виляние: крен с борта на борт 3 раза, не больше ±7° (на 8° уже опасно)
    std::vector<MoveStep> steps;
    for (int i = 0; i < 3; i++) {
      steps.push_back({tilted(0, 7), 0.22, NAN});
      steps.push_back({tilted(0, -7), 0.22, NAN});
    }
    steps.push_back({stand, 0.2, NAN});
    return steps;
  }
  if (name == "bow")   // поклон, как у играющей собаки: нос вниз, пауза, обратно
    return {{tilted(-MAX_TILT_DEG, 0), 0.9, NAN}, {stand, 0.3, NAN}};
  throw KeyError("'" + name + "'");
}

Servo &Robot::servo(const std::string &sid) {
  for (Servo &s : servos_) if (s.cfg.id == sid) return s;
  throw KeyError("'" + sid + "'");
}

void Robot::start_move(const std::string &name) {
  std::vector<MoveStep> steps = moves(name);
  if (!enabled || !std::isnan(powering_down_)) throw ValueError("движение: сначала включите серво");
  stop_demo();
  stop_tests();
  mode_ = "stand";
  pose_name.clear();
  move_ = steps;
  move_name_ = name;
  next_move_step();
  Serial.printf("движение: %s\n", name.c_str());
}

void Robot::next_move_step() {
  if (move_.empty()) {
    set_pose("stand");
    return;
  }
  MoveStep s = move_.front();
  move_.erase(move_.begin());
  body_ = s.body;
  move_t_ = s.hold;
  move_speed_ = s.speed;
}

// Выключение из стойки плавное: суставы сначала встают в монтажное положение (ref_deg), потом
// снимается сигнал — иначе без удержания серво собака падает под своим весом. В ручном режиме —
// сразу. Включение — из монтажного положения плавно в стойку.
void Robot::set_enabled(bool on) {
  if (on && updating) throw ValueError("идёт обновление прошивки: серво не включаются");
  stop_demo();
  if (on && !std::isnan(powering_down_)) {
    powering_down_ = NAN;   // повторное включение отменяет выключение
    set_pose("stand");
    return;
  }
  if (on == enabled) return;
  if (!on && mode_ != "manual") {
    if (std::isnan(powering_down_)) {
      stop_tests();
      for (Servo &s : servos_) s.set_target(s.joint_to_servo(s.cfg.ref_deg));
      mode_ = "manual";   // цели заданы напрямую, контур их не пересчитывает
      pose_name.clear();
      powering_down_ = now_s();
    }
    return;
  }
  do_switch(on);
  if (on) set_pose("stand");
}

void Robot::do_switch(bool on) {
  powering_down_ = NAN;
  if (on && updating) throw ValueError("идёт обновление прошивки: серво не включаются");
  if (on) power_.check(installed_count());
  enabled = on;
  if (on) {
    // Реальное положение серво неизвестно. Считаем, что они в монтажном положении —
    // в нём собака остаётся после выключения; оттуда контур плавно ведёт в стойку.
    for (Servo &s : servos_) {
      s.set_target(s.joint_to_servo(s.cfg.ref_deg));
      s.current = s.target;
    }
    write();
  } else {
    driver_.all_off();
    signal_ = 0;
    roll_offset_ = 0;
  }
  Serial.printf("серво %s\n", on ? "включены" : "отключены");
}

void Robot::estop() {
  estops++;
  web_drive_[0] = web_drive_[1] = pad_drive[0] = pad_drive[1] = 0;
  patrol_drive_[0] = patrol_drive_[1] = 0;
  stop_demo();
  if (enabled) do_switch(false);
}

void Robot::set_mode(const std::string &mode) {
  if (mode != MODES[0] && mode != MODES[1]) throw ValueError("режим: ('stand', 'manual')");
  stop_demo();
  mode_ = mode;
  stop_tests();
}

void Robot::stop_tests() {
  for (Servo &s : servos_) s.queue.clear();
}

void Robot::set_pose(const std::string &name) {
  for (auto jp : JOINT_POSES) {
    if (name != jp) continue;
    stop_demo();
    stop_tests();
    for (Servo &s : servos_) s.set_target(s.joint_to_servo(name == "mount" ? s.cfg.ref_deg : s.cfg.load_deg));
    mode_ = "manual";
    pose_name = name;
    return;
  }
  Body b = pose(name);
  stop_demo();
  body_ = b;
  pose_name = name;
  stop_tests();
  if (mode_ == "manual") mode_ = "stand";
}

void Robot::set_body(const double *height, const double *pitch, const double *roll) {
  if (height) body_.height = clampd(*height, MIN_HEIGHT_MM, MAX_HEIGHT_MM);
  if (pitch) body_.pitch = clampd(*pitch, -MAX_TILT_DEG, MAX_TILT_DEG);
  if (roll) body_.roll = clampd(*roll, -MAX_ROLL_DEG, MAX_ROLL_DEG);
  stop_demo();
  pose_name.clear();
  if (mode_ == "manual") mode_ = "stand";
}

void Robot::set_web_drive(double vx, double turn) {
  web_drive_[0] = clampd(vx, -1, 1);
  web_drive_[1] = clampd(turn, -1, 1);
  web_drive_t_ = now_s();
}

void Robot::set_patrol_drive(double vx, double turn) {
  patrol_drive_[0] = clampd(vx, -1, 1);
  patrol_drive_[1] = clampd(turn, -1, 1);
  patrol_drive_t_ = now_s();
}

bool Robot::web_driving() const {
  return now_s() - web_drive_t_ < DRIVE_TIMEOUT_S && (web_drive_[0] || web_drive_[1]);
}

void Robot::leg_joints(int leg, bool current, double &hip, double &knee) {
  Servo &h = leg_servo(leg, 0), &k = leg_servo(leg, 1);
  hip = h.servo_to_joint(current ? h.current : h.target);
  knee = k.servo_to_joint(current ? k.current : k.target);
}

static int leg_index(const std::string &leg) {
  for (int i = 0; i < 4; i++) if (leg == LEG_NAMES[i]) return i;
  return 0;
}

void Robot::set_servo(const std::string &sid, const double *angle, const double *joint) {
  Servo &s = servo(sid);
  if (!angle && !joint) throw ValueError("нужно ровно одно из: angle, joint");
  double servo_deg = angle ? *angle : s.joint_to_servo(*joint);
  const ServoCfg &c = s.cfg;
  int leg = leg_index(c.leg);
  double old_hip, old_knee;
  leg_joints(leg, false, old_hip, old_knee);
  double j = s.servo_to_joint(clampd(servo_deg, c.min_deg, c.max_deg));
  double new_hip = c.joint == "hip" ? j : old_hip, new_knee = c.joint == "hip" ? old_knee : j;
  if (!guard_.allows_move(old_hip, old_knee, new_hip, new_knee))
    throw ValueError(strf("%s: колено упрётся в раму (бедро %.0f°, колено %.0f°)", c.id.c_str(), new_hip, new_knee));
  stop_demo();
  mode_ = "manual";
  pose_name.clear();   // сустав сдвинут руками — собака уже не в позе
  s.queue.clear();
  s.set_target(servo_deg);
}

void Robot::nudge(const std::string &sid, double delta) {
  double a = servo(sid).target + delta;
  set_servo(sid, &a, nullptr);
}

// Тест при сборке: center — 90° (ставить крыло), ref — монтажное положение сустава,
// sweep — медленно min → max → обратно, stop — остановить проход.
void Robot::test_servo(const std::string &sid, const std::string &action) {
  Servo &s = servo(sid);
  bool known = false;
  for (auto a : TEST_ACTIONS) known |= action == a;
  if (!known) throw ValueError("действие: ('center', 'ref', 'sweep', 'stop')");
  if (action == "stop") {
    double a = s.current;
    set_servo(sid, &a, nullptr);
  } else if (action == "center") {
    double a = s.cfg.range_deg / 2;
    set_servo(sid, &a, nullptr);
  } else if (action == "ref") {
    double j = s.cfg.ref_deg;
    set_servo(sid, nullptr, &j);
  } else if (action == "sweep") {
    if (!(enabled && s.cfg.installed)) throw ValueError("проход по ходу: включите серво и отметьте его подключённым");
    double back = s.target, a = s.cfg.min_deg;
    set_servo(sid, &a, nullptr);
    s.queue = {s.cfg.max_deg, back};
  }
}

void Robot::update_calibration(const std::string &sid, JsonObjectConst f) {
  Servo &s = servo(sid);
  ServoCfg &c = s.cfg;
  if (!f["installed"].isNull()) {
    bool v = f["installed"].as<bool>();
    if (v && !c.installed) power_.check(installed_count() + 1);
    c.installed = v;
    if (!v) {
      s.queue.clear();
      if (enabled) driver_.off(c.channel);
    }
  }
  if (!f["zero"].isNull()) c.zero = f["zero"].as<double>();
  if (!f["scale"].isNull()) {
    double v = f["scale"].as<double>();
    if (!(SCALE_RANGE[0] <= v && v <= SCALE_RANGE[1])) throw ValueError(strf("scale: %g … %g", SCALE_RANGE[0], SCALE_RANGE[1]));
    c.scale = v;
  }
  if (!f["dir"].isNull()) {
    int v = f["dir"].as<int>();
    if (v != -1 && v != 1) throw ValueError("dir: -1 или 1");
    c.dir = v;
  }
  if (!f["min_deg"].isNull()) c.min_deg = f["min_deg"].as<double>();
  if (!f["max_deg"].isNull()) c.max_deg = f["max_deg"].as<double>();
  if (!(0 <= c.min_deg && c.min_deg < c.max_deg && c.max_deg <= c.range_deg))
    throw ValueError("пределы: 0 ≤ min < max ≤ range");
  s.set_target(s.target);
}

// Текущее положение серво соответствует углу сустава joint_deg.
// zero — сдвиг, масштаб прежний (монтажное положение или колено под нагрузкой).
// scale — вторая точка (поза под нагрузкой): масштаб подбирается так, чтобы монтажное положение
// осталось на месте, а joint_deg пришёлся на текущий угол серво. Сначала zero, потом scale.
void Robot::calibrate(const std::string &sid, double joint_deg, const std::string &what) {
  Servo &s = servo(sid);
  ServoCfg &c = s.cfg;
  if (what == "zero") {
    c.zero = r2(s.current - c.dir * c.scale * joint_deg);
    return;
  }
  if (what != "scale") throw ValueError("калибровка: zero или scale");
  double span = joint_deg - c.ref_deg;
  if (std::fabs(span) < SCALE_MIN_SPAN_DEG)
    throw ValueError(strf("масштаб: сустав ближе %.0f° к монтажному положению — запишите zero", SCALE_MIN_SPAN_DEG));
  double at_ref = s.joint_to_servo(c.ref_deg);
  double scale = r2(c.dir * (s.current - at_ref) / span, 4);
  if (!(SCALE_RANGE[0] <= scale && scale <= SCALE_RANGE[1]))
    throw ValueError(strf("масштаб %.3f вне %g … %g: проверьте угол сустава и что zero записан в монтажном положении",
                          scale, SCALE_RANGE[0], SCALE_RANGE[1]));
  c.scale = scale;
  c.zero = r2(at_ref - c.dir * scale * c.ref_deg);
}

// Всё, что меняется из портала, одним JSON в NVS платы (как calibration.json на Pi):
// калибровка серво, лимит тока, профиль ходьбы, поправка наклона корпуса.
std::string Robot::save_calibration() {
  JsonDocument doc;
  for (const Servo &s : servos_) {
    JsonObject o = doc[s.cfg.id].to<JsonObject>();
    o["installed"] = s.cfg.installed;
    o["zero"] = s.cfg.zero;
    o["scale"] = s.cfg.scale;
    o["dir"] = s.cfg.dir;
    o["min_deg"] = s.cfg.min_deg;
    o["max_deg"] = s.cfg.max_deg;
  }
  doc["power"]["limit_a"] = power_.limit_a;
  doc["power"]["servo_a"] = power_.servo_a;
  doc["gait"] = gait().name;
  doc["trim"]["pitch_deg"] = trim_pitch_;
  doc["trim"]["roll_deg"] = trim_roll_;
  String json;
  serializeJson(doc, json);
  Preferences p;
  p.begin("edog", false);
  size_t n = p.putString("calibration", json);
  p.end();
  if (!n) throw ValueError("не удалось записать калибровку в NVS");
  return "NVS платы (calibration)";
}

// Калибровка в формате calibration.json: сначала проверяется целиком, потом применяется — при ошибке
// ничего не меняется. Чего в файле нет, остаётся как было.
void Robot::apply_calibration(JsonObjectConst doc) {
  if (doc.isNull()) throw ValueError("калибровка: нужен объект JSON, как calibration.json");
  auto number = [](JsonVariantConst v, double fallback, const std::string &name) {
    if (v.isNull()) return fallback;
    if (!v.is<double>()) throw ValueError(name + ": нужно число");
    return v.as<double>();
  };
  Power pw = {number(doc["power"]["limit_a"], power_.limit_a, "power.limit_a"),
              number(doc["power"]["servo_a"], power_.servo_a, "power.servo_a")};
  pw.validate();
  double trim[2] = {number(doc["trim"]["pitch_deg"], trim_pitch_, "trim.pitch_deg"),
                    number(doc["trim"]["roll_deg"], trim_roll_, "trim.roll_deg")};
  for (double t : trim)
    if (std::fabs(t) > TRIM_MAX_DEG) throw ValueError(strf("trim: не больше ±%g°", TRIM_MAX_DEG));
  int gait = gait_;
  if (!doc["gait"].isNull()) {
    std::string g = doc["gait"] | "";
    gait = -1;
    for (int i = 0; i < GAIT_COUNT; i++) if (g == GAITS[i].name) gait = i;
    if (gait < 0) throw ValueError("gait: нет профиля '" + g + "' в config.h");
  }
  std::vector<ServoCfg> cfgs;
  int installed = 0;
  for (const Servo &s : servos_) {
    ServoCfg c = s.cfg;
    JsonObjectConst o = doc[c.id];
    const std::string &id = c.id;
    if (!o.isNull()) {
      if (!o["installed"].isNull()) {
        if (!o["installed"].is<bool>()) throw ValueError(id + ".installed: нужно true/false");
        c.installed = o["installed"].as<bool>();
      }
      c.zero = number(o["zero"], c.zero, id + ".zero");
      c.scale = number(o["scale"], c.scale, id + ".scale");
      c.dir = (int)number(o["dir"], c.dir, id + ".dir");
      c.min_deg = number(o["min_deg"], c.min_deg, id + ".min_deg");
      c.max_deg = number(o["max_deg"], c.max_deg, id + ".max_deg");
      if (c.dir != -1 && c.dir != 1) throw ValueError(id + ".dir: -1 или 1");
      if (!(SCALE_RANGE[0] <= c.scale && c.scale <= SCALE_RANGE[1]))
        throw ValueError(id + strf(".scale: %g … %g", SCALE_RANGE[0], SCALE_RANGE[1]));
      if (!(0 <= c.min_deg && c.min_deg < c.max_deg && c.max_deg <= c.range_deg))
        throw ValueError(id + ": пределы 0 ≤ min < max ≤ range");
    }
    installed += c.installed;
    cfgs.push_back(c);
  }
  pw.check(installed);

  power_ = pw;
  trim_pitch_ = trim[0];
  trim_roll_ = trim[1];
  gait_ = gait;
  for (size_t i = 0; i < servos_.size(); i++) {
    servos_[i].cfg = cfgs[i];
    servos_[i].set_target(servos_[i].target);
  }
}

void Robot::load_calibration() {
  Preferences p;
  if (!p.begin("edog", true)) return;   // namespace ещё не создан — калибровки нет
  String json = p.getString("calibration", "");
  p.end();
  if (json.isEmpty()) return;
  JsonDocument doc;
  try {
    if (deserializeJson(doc, json)) throw ValueError("не JSON");
    apply_calibration(doc.as<JsonObjectConst>());
    Serial.println("калибровка загружена из NVS");
  } catch (const ValueError &e) {
    Serial.printf("калибровка в NVS не подошла (%s) — беру значения из config.h\n", e.what());
  }
}

// Калибровка из файла (calibration.json с Raspberry Pi или сохранённая раньше). Только при выключенных
// серво: новый zero сразу сдвинул бы ноги. В NVS — save_calibration().
void Robot::import_calibration(JsonObjectConst doc) {
  if (enabled || !std::isnan(powering_down_)) throw ValueError("калибровка из файла: сначала выключите серво");
  stop_demo();
  stop_tests();
  apply_calibration(doc);
  if (mode_ == "stand" && pose_name == "stand") body_ = pose("stand");   // стойка выбранного профиля
  Serial.println("калибровка загружена из файла");
}

// Поправка наклона корпуса из портала, °. В NVS — save_calibration().
void Robot::update_trim(const double *pitch_deg, const double *roll_deg) {
  for (const double *v : {pitch_deg, roll_deg})
    if (v && std::fabs(*v) > TRIM_MAX_DEG) throw ValueError(strf("поправка наклона: не больше ±%g°", TRIM_MAX_DEG));
  if (pitch_deg) trim_pitch_ = *pitch_deg;
  if (roll_deg) trim_roll_ = *roll_deg;
}

// Профиль ходьбы: им же шагает демо. Можно менять на ходу. В NVS — save_calibration().
void Robot::set_gait(const std::string &name) {
  int found = -1;
  std::string names;
  for (int i = 0; i < GAIT_COUNT; i++) {
    if (name == GAITS[i].name) found = i;
    names += std::string(i ? ", '" : "'") + GAITS[i].name + "'";
  }
  if (found < 0) throw ValueError("профили ходьбы: (" + names + ")");
  gait_ = found;
  Serial.printf("профиль ходьбы: %s\n", name.c_str());
  if (mode_ == "stand" && pose_name == "stand" && move_name_.empty() && !demo_)
    body_ = pose("stand");   // стоит — встаёт в стойку нового профиля
}

// Бамперы: крен на ROLL_STEP_DEG в сторону sign (+ — правый борт ниже), держится до выключения.
void Robot::nudge_roll(int sign) {
  roll_offset_ = clampd(roll_offset_ + sign * ROLL_STEP_DEG, -TRIM_MAX_DEG, TRIM_MAX_DEG);
  Serial.printf("добавка к крену: %+.0f°\n", roll_offset_);
}

void Robot::next_gait() { set_gait(GAITS[(gait_ + 1) % GAIT_COUNT].name); }

int Robot::installed_count() const {
  int n = 0;
  for (const Servo &s : servos_) n += s.cfg.installed;
  return n;
}

// Лимит тока из портала: проверяется по уже подключённым серво. В NVS — save_calibration().
void Robot::update_power(const double *limit_a, const double *servo_a) {
  Power p = power_;
  if (limit_a) p.limit_a = *limit_a;
  if (servo_a) p.servo_a = *servo_a;
  p.validate();
  p.check(installed_count());
  power_ = p;
}

// Демо: ноги по кругу шагают в воздухе по cycles полных циклов шага выбранного профиля, laps кругов.
// Сколько ног сразу — по лимиту тока; сигнал только на шагающие ноги (отметки «подключено» на время
// демо не действуют). В конце сигнал снимается.
void Robot::start_demo(int cycles, int laps) {
  if (updating) throw ValueError("идёт обновление прошивки: серво не включаются");
  std::string program = pick_program(power_.max_servos());
  double ref_hip = servos_[0].cfg.ref_deg, ref_knee = servos_[1].cfg.ref_deg;
  Demo *demo = new Demo(program, cycles, 1.0 / gait().gait_hz, laps, ref_hip, ref_knee,
                        [this](int leg, double t, double &hip, double &knee) { demo_step(leg, t, hip, knee); });
  stop_demo();
  stop_tests();
  if (enabled) {
    driver_.all_off();
    signal_ = 0;
  }
  for (Servo &s : servos_) {
    s.set_target(s.joint_to_servo(s.cfg.joint == "hip" ? ref_hip : ref_knee));
    s.current = s.target;
  }
  demo_ = demo;
  mode_ = "demo";
  enabled = true;
  Serial.printf("демо: %s, шаг %s, по %d циклов на ногу, кругов %d\n", program.c_str(), gait().name, cycles, laps);
}

void Robot::stop_demo() {
  // Любая команда прерывает и движение: все они проходят через stop_demo().
  move_.clear();
  move_name_.clear();
  move_speed_ = NAN;
  if (!demo_) return;
  delete demo_;
  demo_ = nullptr;
  enabled = false;
  driver_.all_off();
  signal_ = 0;
  mode_ = "manual";
  Serial.println("демо остановлено, сигнал снят");
}

// Шаг вперёд полной длины в воздухе: корпус на высоте профиля (без неё — стойки), без наклонов.
void Robot::demo_step(int leg, double t, double &hip, double &knee) const {
  const GaitCfg &g = gait();
  const LegDef &l = LEG_DEFS[leg];
  double dx, dz, x, z;
  leg_step(leg, pmod1(t * g.gait_hz), g.stride_mm, g.lift_mm, dx, dz);
  double y = (!strcmp(l.side, "left") ? 1 : -1) * BODY_WIDTH_MM / 2;
  double foot[3] = {l.x_mm + g.foot_x_mm + dx, y, dz}, hip_b[3] = {l.x_mm, y, 0.0};
  foot_in_leg(foot, hip_b, gait_height(g, STAND_HEIGHT_MM), 0, 0, x, z);
  ik(x, z, FEMUR_MM, TIBIA_MM, l.knee_dir, TIBIA_BEND_DEG, hip, knee);
}

// Кто ведёт собаку: портал (команда свежая) → геймпад (стик отклонён) → патруль (команда свежая).
void Robot::drive(double &vx, double &turn, const char **source) {
  double now = now_s();
  bool web = now - web_drive_t_ < DRIVE_TIMEOUT_S;
  bool pad = pad_drive[0] || pad_drive[1];
  bool patrol = !web && !pad && now - patrol_drive_t_ < DRIVE_TIMEOUT_S;
  vx = web ? web_drive_[0] : patrol ? patrol_drive_[0] : pad_drive[0];
  turn = web ? web_drive_[1] : patrol ? patrol_drive_[1] : pad_drive[1];
  if (source) *source = web ? "web" : patrol ? "patrol" : "gamepad";
}

// ---------- контур ----------

void Robot::feet(double dt, double xs[4], double zs[4]) {
  const GaitCfg &g = gait();
  double pitch = clampd(body_.pitch + tilt[0] * MAX_TILT_DEG, -MAX_TILT_DEG, MAX_TILT_DEG);
  double roll = clampd(body_.roll + tilt[1] * MAX_ROLL_DEG, -MAX_ROLL_DEG, MAX_ROLL_DEG);
  double half_width = BODY_WIDTH_MM / 2;
  double vx, turn;
  drive(vx, turn, nullptr);
  // Ходьба всегда начинается с начала: если стик снова толкнули, пока собака ещё не встала,
  // она сначала доходит до стойки профиля, потом шаг с фазы 0 — первыми идут FL и RR.
  if (!(std::fabs(vx) > 0.05 || std::fabs(turn) > 0.05)) {
    walking_ = false;
  } else if (!walking_ && walk_blend_ == 0.0 && settled()) {
    walking_ = true;
    phase_ = 0.0;
    cycles_ = 0.0;
  }
  bool walking = walking_;
  if (walking) {
    phase_ = pmod1(phase_ + g.gait_hz * dt);
    cycles_ += g.gait_hz * dt;
  }
  // Профиль может ходить на своей высоте и с лапами, вынесенными от оси бедра. Старт плавный,
  // шаг растёт вместе с ним. Остановка — сразу в стойку профиля.
  walk_blend_ = walking ? clampd(walk_blend_ + dt / WALK_RAMP_S, 0.0, 1.0) : 0.0;
  double w = walk_blend_;
  // Вынос лап — в стойке и при ходьбе вперёд; назад и на развороте — лапы под осью бедра.
  bool forward = !walking || vx > 0.05;
  forward_blend_ = walking ? clampd(forward_blend_ + (forward ? dt : -dt) / WALK_RAMP_S, 0.0, 1.0) : 1.0;
  double height = body_.height + w * (gait_height(g, body_.height) - body_.height);
  double walk_x = forward_blend_ * g.foot_x_mm;
  double foot_x = body_.foot_x + w * (walk_x - body_.foot_x);
  // огрехи сборки: поправка сверх пределов
  pitch += trim_pitch_;
  roll += trim_roll_ + roll_offset_;

  // Лапы задаются точками на полу (мир), потом переводятся в систему наклонённого корпуса.
  for (int i = 0; i < 4; i++) {
    const LegDef &l = LEG_DEFS[i];
    double side = !strcmp(l.side, "left") ? 1 : -1;
    double stride = walking ? g.stride_mm * clampd(vx + side * turn, -1, 1) : 0.0;
    double dx, dz;
    leg_step(i, phase_, w * stride, w * g.lift_mm, dx, dz);
    double foot[3] = {l.x_mm + foot_x + dx, side * half_width, dz};
    double hip[3] = {l.x_mm, side * half_width, 0.0};
    foot_in_leg(foot, hip, height, pitch, roll, xs[i], zs[i]);
  }
}

bool Robot::settled() const {
  for (const Servo &s : servos_) if (std::fabs(s.target - s.current) >= 1.0) return false;
  return true;
}

void Robot::update_targets(double dt) {
  if (demo_) {
    demo_->tick(dt, [this](int leg, double hip, double knee) {
      Servo &h = leg_servo(leg, 0), &k = leg_servo(leg, 1);
      h.set_target(h.joint_to_servo(hip));
      k.set_target(k.joint_to_servo(knee));
    });
    return;
  }
  double vx, turn;
  drive(vx, turn, nullptr);
  if (!move_name_.empty()) {
    move_t_ -= dt;
    if (move_t_ <= 0) next_move_step();
  }
  if (!std::isnan(powering_down_) || !move_name_.empty()) vx = turn = 0;   // без ходьбы
  if (std::fabs(vx) > 0.05 || std::fabs(turn) > 0.05) {
    if (mode_ == "manual" || pose_name != "stand") set_pose("stand");   // ходит только из стойки
  } else if (mode_ == "manual") {
    return;
  }
  if (height_rate && std::isnan(powering_down_) && move_name_.empty()) {
    // Поза при этом не меняется: из стойки с другой высотой ходить можно.
    body_.height = clampd(body_.height + height_rate * HEIGHT_SPEED_MM_S * dt, MIN_HEIGHT_MM, MAX_HEIGHT_MM);
  }
  double xs[4], zs[4];
  feet(dt, xs, zs);
  for (int i = 0; i < 4; i++) {
    double hip, knee;
    ik(xs[i], zs[i], FEMUR_MM, TIBIA_MM, LEG_DEFS[i].knee_dir, TIBIA_BEND_DEG, hip, knee);
    Servo &h = leg_servo(i, 0), &k = leg_servo(i, 1);
    h.set_target(h.joint_to_servo(hip));
    k.set_target(k.joint_to_servo(knee));
  }
}

// Сигнал только на подключённые серво: при сборке ноги появляются по одной.
// В демо — только на шагающие ноги: их число подобрано по лимиту тока.
uint32_t Robot::signal_ids() const {
  uint32_t on = 0;
  if (demo_) {
    for (int leg : demo_->legs()) on |= 3u << (leg * 2);
    return on;
  }
  for (size_t i = 0; i < servos_.size(); i++) if (servos_[i].cfg.installed) on |= 1u << i;
  return on;
}

void Robot::write() {
  uint32_t on = signal_ids();
  for (size_t i = 0; i < servos_.size(); i++) {
    const Servo &s = servos_[i];
    if (on & (1u << i)) driver_.set_pulse(s.cfg.channel, s.pulse_us(s.current));
    else if (signal_ & (1u << i)) driver_.off(s.cfg.channel);
  }
  signal_ = on;
}

void Robot::step(double dt) {
  update_targets(dt);
  double before[4][2];
  for (int i = 0; i < 4; i++) leg_joints(i, true, before[i][0], before[i][1]);
  double max_speed = !move_name_.empty() && !std::isnan(move_speed_) && move_speed_ ? move_speed_ : MAX_SPEED_DPS;
  // Суставы ноги доходят до цели одновременно: скорость каждого — по его доле хода.
  // Иначе бедро успевает раньше колена, лапа идёт в обход, и колено задевает раму.
  double lead[4];
  for (int i = 0; i < 4; i++)
    lead[i] = std::fmax(std::fabs(leg_servo(i, 0).target - leg_servo(i, 0).current),
                        std::fabs(leg_servo(i, 1).target - leg_servo(i, 1).current));
  for (size_t i = 0; i < servos_.size(); i++) {
    Servo &s = servos_[i];
    if (!enabled) {
      s.current = s.target;
      s.queue.clear();
      continue;
    }
    double delta = s.target - s.current, speed;
    if (!s.queue.empty()) speed = SWEEP_SPEED_DPS;
    else if (delta) speed = max_speed * std::fabs(delta) / lead[i / 2];
    else continue;
    s.current += clampd(delta, -speed * dt, speed * dt);
    if (!s.queue.empty() && std::fabs(s.target - s.current) < 1e-6) {
      double next = s.queue.front();
      s.queue.erase(s.queue.begin());
      s.set_target(next);
    }
  }
  guard_legs(before);
  if (demo_ && demo_->finished()) stop_demo();
  if (!std::isnan(powering_down_)) {
    bool done = true;
    for (const Servo &s : servos_) done &= std::fabs(s.target - s.current) < 0.5;
    if (done || now_s() - powering_down_ > POWER_DOWN_TIMEOUT_S) do_switch(false);
  }
  if (enabled) write();
  record_motion();
}

void Robot::record_motion() {
  Motion m;
  m.t = now_s();
  m.cycles = cycles_;
  m.gait_hz = gait().gait_hz;
  m.walking = walking_ && enabled && !demo_;
  drive(m.vx, m.turn, nullptr);
  // без сигнала серво собака лежит неподвижно — для камеры это тоже «стоит»
  m.still = !demo_ && (!enabled || (!walking_ && move_name_.empty() && std::isnan(powering_down_) && settled()));
  portENTER_CRITICAL(&motion_mux);
  motion_log[motion_head] = m;
  motion_head = (motion_head + 1) % MOTION_LOG;
  if (!m.still) still_since = NAN;
  else if (std::isnan(still_since)) still_since = m.t;
  portEXIT_CRITICAL(&motion_mux);
}

// Шаг, поднимающий узел колена выше рамы, отменяется: нога стоит, где стояла.
// Опускаться (уходить от рамы) можно всегда.
void Robot::guard_legs(const double before[4][2]) {
  for (int i = 0; i < 4; i++) {
    double hip, knee;
    leg_joints(i, true, hip, knee);
    bool blocked = !guard_.allows_move(before[i][0], before[i][1], hip, knee);
    if (blocked) {
      for (int j = 0; j < 2; j++) {
        Servo &s = leg_servo(i, j);
        s.current = s.target = s.joint_to_servo(before[i][j]);
        s.queue.clear();
      }
      if (!blocked_[i]) Serial.printf("нога %s остановлена: колено упрётся в раму\n", LEG_NAMES[i]);
    }
    blocked_[i] = blocked;
  }
}

// ---------- описание ----------

void Robot::describe(JsonDocument &doc) {
  JsonObject g = doc["geometry"].to<JsonObject>();
  g["femur_mm"] = FEMUR_MM;
  g["tibia_mm"] = TIBIA_MM;
  g["tibia_bend_deg"] = TIBIA_BEND_DEG;
  g["frame_margin_mm"] = FRAME_MARGIN_MM;
  g["body_length_mm"] = BODY_LENGTH_MM;
  g["body_width_mm"] = BODY_WIDTH_MM;
  JsonArray legs = doc["legs"].to<JsonArray>();
  for (const LegDef &l : LEG_DEFS) {
    JsonObject o = legs.add<JsonObject>();
    o["name"] = l.name;
    o["side"] = l.side;
    o["x_mm"] = l.x_mm;
    o["knee_dir"] = l.knee_dir;
  }
  JsonArray servos = doc["servos"].to<JsonArray>();
  for (const Servo &s : servos_) {
    JsonObject o = servos.add<JsonObject>();
    o["id"] = s.cfg.id;
    o["leg"] = s.cfg.leg;
    o["joint"] = s.cfg.joint;
    o["channel"] = s.cfg.channel;
  }
  JsonArray poses = doc["poses"].to<JsonArray>();
  for (auto p : BODY_POSES) poses.add(p);
  for (auto p : JOINT_POSES) poses.add(p);
  JsonArray mv = doc["moves"].to<JsonArray>();
  for (auto m : MOVES) mv.add(m);
  JsonArray gaits = doc["gaits"].to<JsonArray>();
  for (const GaitCfg &gc : GAITS) {
    JsonObject o = gaits.add<JsonObject>();
    o["name"] = gc.name;
    o["title"] = gc.title;
    o["gait_hz"] = gc.gait_hz;
    o["stride_mm"] = gc.stride_mm;
    o["lift_mm"] = gc.lift_mm;
    o["foot_x_mm"] = gc.foot_x_mm;
    if (!std::isnan(gc.height_mm)) o["height_mm"] = gc.height_mm;
    gait_stats(gc, o);
  }
  JsonObject lim = doc["limits"].to<JsonObject>();
  lim["height"].add(MIN_HEIGHT_MM);
  lim["height"].add(MAX_HEIGHT_MM);
  lim["pitch"] = MAX_TILT_DEG;
  lim["roll"] = MAX_ROLL_DEG;
  JsonObject pad = doc["gamepad"].to<JsonObject>();
  for (const Mapping &m : GAMEPAD_AXES) pad["axes"][m.key] = m.value;
  for (const Mapping &m : GAMEPAD_BUTTONS) pad["buttons"][m.key] = m.value;
  for (const Mapping &m : GAMEPAD_DPAD) pad["dpad"][m.key] = m.value;
  // плоскость рамы, мм от оси бедра (вверх +): plane — низ перекладины, limit — с запасом
  doc["frame_guard"]["plane_mm"] = r2(HULL_FRAME_BOTTOM_MM);
  doc["frame_guard"]["limit_mm"] = r2(guard_.limit);
}

void Robot::state(JsonDocument &doc) {
  double vx, turn;
  const char *source;
  drive(vx, turn, &source);
  doc["t"] = now_s();
  doc["enabled"] = enabled;
  doc["powering_down"] = !std::isnan(powering_down_);
  doc["updating"] = updating;
  doc["mode"] = mode_;
  if (pose_name.empty()) doc["pose"] = nullptr;
  else doc["pose"] = pose_name;
  if (move_name_.empty()) doc["move"] = nullptr;
  else doc["move"] = move_name_;
  JsonObject b = doc["body"].to<JsonObject>();
  b["height"] = body_.height;
  b["pitch"] = body_.pitch;
  b["roll"] = body_.roll;
  b["foot_x"] = body_.foot_x;
  doc["tilt"].add(tilt[0]);
  doc["tilt"].add(tilt[1]);
  doc["drive"]["vx"] = vx;
  doc["drive"]["turn"] = turn;
  doc["drive"]["source"] = source;
  JsonObject gp = doc["gamepad"].to<JsonObject>();
  gp["connected"] = gamepad.connected;
  if (gamepad.connected) gp["name"] = gamepad.name;
  else gp["name"] = nullptr;
  if (!gamepad.last_button.empty()) gp["last_button"] = gamepad.last_button;
  for (int i = 0; i < 6; i++) gp["axes"][PAD_AXES[i]] = r2(gamepad.axes[i]);
  doc["driver"]["kind"] = driver_.kind();
  if (driver_.error.isEmpty()) doc["driver"]["error"] = nullptr;
  else doc["driver"]["error"] = driver_.error;
  JsonArray servos = doc["servos"].to<JsonArray>();
  for (size_t i = 0; i < servos_.size(); i++) {
    const Servo &s = servos_[i];
    const ServoCfg &c = s.cfg;
    JsonObject o = servos.add<JsonObject>();
    o["id"] = c.id;
    o["leg"] = c.leg;
    o["joint"] = c.joint;
    o["channel"] = c.channel;
    o["current"] = r2(s.current);
    o["target"] = r2(s.target);
    o["joint_current"] = r2(s.servo_to_joint(s.current));
    o["joint_target"] = r2(s.servo_to_joint(s.target));
    o["pulse_us"] = (int)std::lround(s.pulse_us(s.current));
    o["installed"] = c.installed;
    o["sweeping"] = !s.queue.empty();
    o["ref_deg"] = c.ref_deg;
    o["load_deg"] = c.load_deg;
    o["zero"] = c.zero;
    o["scale"] = c.scale;
    o["dir"] = c.dir;
    o["min_deg"] = c.min_deg;
    o["max_deg"] = c.max_deg;
    o["clamped"] = s.clamped;
    o["frame_blocked"] = blocked_[i / 2];
    o["signal"] = ((signal_ >> i) & 1) != 0;
  }
  if (demo_) demo_->state(doc["demo"].to<JsonObject>());
  else doc["demo"] = nullptr;
  doc["gait"] = gait().name;
  doc["frame_guard"] = true;
  JsonObject pw = doc["power"].to<JsonObject>();
  pw["limit_a"] = power_.limit_a;
  pw["servo_a"] = power_.servo_a;
  if (power_.max_servos() < 0) pw["max_servos"] = nullptr;
  else pw["max_servos"] = power_.max_servos();
  pw["installed"] = installed_count();
  doc["trim"]["pitch_deg"] = trim_pitch_;
  doc["trim"]["roll_deg"] = trim_roll_;
  doc["roll_offset"] = roll_offset_;
}
