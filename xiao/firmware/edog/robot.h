// Состояние робота и контур управления: позы, походка, плавное движение серво, демо, калибровка.
// Порт rpi/firmware/edog/robot.py (+ demo.py, power.py) один в один.
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "config.h"
#include "kinematics.h"
#include "pca9685.h"

// Ошибки команд: ValueError → HTTP 400, KeyError → 404 (как в api.py).
struct ValueError : std::runtime_error { using std::runtime_error::runtime_error; };
struct KeyError : std::runtime_error { using std::runtime_error::runtime_error; };

std::string strf(const char *fmt, ...);

// Один замок на всё состояние робота: контур, HTTP, WebSocket и геймпад работают в разных задачах.
void robot_lock_init();
struct RobotLock {
  RobotLock();
  ~RobotLock();
};

struct ServoCfg {
  std::string id, leg, joint;
  int channel;
  bool installed;
  double ref_deg, load_deg;   // монтажное положение и калибровочная поза под нагрузкой, ° сустава
  int dir;
  double zero, min_deg, max_deg, range_deg, pulse_min_us, pulse_max_us;
  double scale = 1.0;         // градусов серво на градус сустава
};

struct Servo {
  ServoCfg cfg;
  double target = 0, current = 0;   // угол серво, °
  bool clamped = false;             // последняя цель упёрлась в предел
  std::vector<double> queue;        // тестовый проход по ходу: следующие цели

  double joint_to_servo(double j) const { return cfg.zero + cfg.dir * cfg.scale * j; }
  double servo_to_joint(double s) const { return (s - cfg.zero) * cfg.dir / cfg.scale; }
  void set_target(double servo_deg) {
    target = clampd(servo_deg, cfg.min_deg, cfg.max_deg);
    clamped = target != servo_deg;
  }
  double pulse_us(double servo_deg) const {
    return cfg.pulse_min_us + servo_deg / cfg.range_deg * (cfg.pulse_max_us - cfg.pulse_min_us);
  }
};

struct Body {
  double height;
  double pitch = 0;    // °, + — нос вверх
  double roll = 0;     // °, + — правый борт ниже
  double foot_x = 0;   // мм, лапы впереди (+) или позади (−) оси бедра
};

struct MoveStep {
  Body body;
  double hold;    // сколько держать, с
  double speed;   // скорость серво, °/с; NAN — обычная
};

struct Power {
  double limit_a, servo_a;
  int max_servos() const;   // −1 — без ограничения
  void validate() const;
  void check(int installed) const;
};

class Demo;

// Что делает робот в момент t (для кадров камеры: в какой фазе шага снят кадр).
struct Motion {
  double t = 0;
  double cycles = 0;     // фаза шага с начала ходьбы, циклы (дробная часть — фаза 0 … 1)
  double gait_hz = 0;
  bool walking = false;
  bool still = false;    // стоит неподвижно: не идёт, нет движения, серво дошли до целей
  double vx = 0, turn = 0;
  double body_mm = 0;    // высота оси бедра над полом: от неё зависит высота камеры
};
// Ближайшая к t запись контура за последние ~0.6 с; false — такой давней записи нет.
bool robot_motion_at(double t, Motion &out);
double robot_still_since();   // с какого времени стоит неподвижно; NAN — сейчас не стоит
double robot_body_mm();       // высота корпуса по последней записи контура (и когда контур ждёт замка)

// Что знает о геймпаде портал.
struct PadInfo {
  bool connected = false;
  std::string name, last_button;
  double axes[6] = {};   // drive, turn, pitch, roll, height_down, height_up
};
extern const char *const PAD_AXES[6];

class Robot {
 public:
  explicit Robot(Driver &driver);

  // команды (вызывать под RobotLock)
  void set_enabled(bool enabled);
  void estop();
  void set_mode(const std::string &mode);
  void set_pose(const std::string &name);
  void start_move(const std::string &name);
  void set_body(const double *height, const double *pitch, const double *roll);
  void set_web_drive(double vx, double turn);
  void set_patrol_drive(double vx, double turn);   // слабее портала и геймпада; гаснет без повторения
  void set_servo(const std::string &sid, const double *angle, const double *joint);
  void nudge(const std::string &sid, double delta);
  void test_servo(const std::string &sid, const std::string &action);
  void update_calibration(const std::string &sid, JsonObjectConst fields);
  void calibrate(const std::string &sid, double joint_deg, const std::string &what);
  std::string save_calibration();
  void import_calibration(JsonObjectConst doc);
  void update_trim(const double *pitch_deg, const double *roll_deg);
  void set_gait(const std::string &name);
  void next_gait();
  void nudge_roll(int sign);
  void update_power(const double *limit_a, const double *servo_a);
  void start_demo(int cycles, int laps);
  void stop_demo();

  bool is_pose(const std::string &name) const;
  bool is_move(const std::string &name) const;
  bool moving() const { return !move_name_.empty(); }   // идёт движение (прыжок, радость…)
  bool powering_down() const { return !std::isnan(powering_down_); }
  bool web_driving() const;                             // портал сейчас ведёт собаку стрелками
  const GaitCfg &gait_cfg() const { return gait(); }

  void step(double dt);
  void describe(JsonDocument &doc);
  void state(JsonDocument &doc);

  // геймпад пишет сюда напрямую, как gamepad.py
  bool enabled = false;
  bool updating = false;             // идёт запись прошивки по воздуху: серво не включаются
  std::string pose_name = "stand";   // пусто — не в позе
  double tilt[2] = {0, 0};           // pitch/roll с геймпада, доля от max_tilt
  double pad_drive[2] = {0, 0};      // vx, turn с геймпада
  double height_rate = 0;            // курки: −1 вниз … 1 вверх
  PadInfo gamepad;
  uint32_t pad_presses = 0;          // счётчик нажатий кнопок геймпада: патруль видит, что человек взялся за него
  uint32_t estops = 0;               // счётчик аварийных стопов

 private:
  Driver &driver_;
  std::vector<Servo> servos_;
  std::string mode_ = "stand";
  double roll_offset_ = 0;
  double web_drive_[2] = {0, 0};
  double web_drive_t_ = -1e9;
  double patrol_drive_[2] = {0, 0};
  double patrol_drive_t_ = -1e9;
  double phase_ = 0, walk_blend_ = 0, forward_blend_ = 1, back_blend_ = 0;
  double cycles_ = 0;                 // фаза с начала ходьбы, не сворачивается в 0 … 1
  void record_motion();
  bool walking_ = false;
  int gait_ = 0;
  Body body_;
  Power power_;
  double trim_pitch_ = 0, trim_roll_ = 0;
  Demo *demo_ = nullptr;
  uint32_t signal_ = 0;               // биты серво, на каналы которых идёт импульс
  std::vector<MoveStep> move_;
  std::string move_name_;
  double move_t_ = 0, move_speed_ = NAN;
  double powering_down_ = NAN;        // с какого времени встаёт в монтажное положение перед снятием сигнала
  FrameGuard guard_;
  bool blocked_[4] = {};

  const GaitCfg &gait() const { return GAITS[gait_]; }
  Body pose(const std::string &name) const;
  std::vector<MoveStep> moves(const std::string &name) const;
  Servo &servo(const std::string &sid);
  Servo &leg_servo(int leg, int joint) { return servos_[leg * 2 + joint]; }
  void leg_joints(int leg, bool current, double &hip, double &knee);
  void next_move_step();
  void do_switch(bool enabled);
  void stop_tests();
  int installed_count() const;
  void drive(double &vx, double &turn, const char **source);
  void feet(double dt, double xs[4], double zs[4]);
  bool settled() const;
  void update_targets(double dt);
  uint32_t signal_ids() const;
  void write();
  void guard_legs(const double before[4][2]);
  void demo_step(int leg, double t, double &hip, double &knee) const;
  void apply_calibration(JsonObjectConst doc);
  void load_calibration();
};

double now_s();
