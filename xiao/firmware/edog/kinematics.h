// Кинематика ноги из двух звеньев в сагиттальной плоскости (как rpi/firmware/edog/kinematics.py).
//
// Система координат ноги: начало — ось бедра, x — вперёд, z — вниз (мм).
// hip: 0° — бедро вертикально вниз, + — к x+.
// knee: угол между бедром и КОРПУСОМ голени: 0° — на одной линии с бедром, + — голень к x+.
// Голень изогнута: линия «ось колена → подошва» повёрнута относительно корпуса на bend_deg,
// поэтому ik учитывает bend_deg; снаружи везде угол по корпусу.
#pragma once

#include <cmath>

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
inline double deg(double r) { return r * 180.0 / M_PI; }
inline double rad(double d) { return d * M_PI / 180.0; }

// Положение лапы → (hip°, knee°). Недостижимые точки прижимаются к границе досягаемости.
inline void ik(double x, double z, double femur, double tibia, int knee_dir, double bend_deg,
               double &hip, double &knee) {
  double d = clampd(std::hypot(x, z), std::fabs(femur - tibia) + 1e-3, femur + tibia - 1e-3);
  double alpha = std::atan2(x, z);
  double gamma = std::acos(clampd((femur * femur + d * d - tibia * tibia) / (2 * femur * d), -1, 1));
  double bend = M_PI - std::acos(clampd((femur * femur + tibia * tibia - d * d) / (2 * femur * tibia), -1, 1));
  hip = deg(alpha + knee_dir * gamma);
  knee = deg(-knee_dir * bend) + bend_deg;
}

// Поворот корпус → мир: R = Ry(−pitch) · Rx(roll). Мир: x вперёд, y влево, z вверх.
// pitch + — нос вверх, roll + — правый борт ниже.
struct BodyRotation {
  double cp, sp, cr, sr;
  BodyRotation(double pitch_deg, double roll_deg)
      : cp(std::cos(rad(pitch_deg))), sp(std::sin(rad(pitch_deg))),
        cr(std::cos(rad(roll_deg))), sr(std::sin(rad(roll_deg))) {}
  void rot(double x, double y, double z, double &ox, double &oy, double &oz) const {
    double y1 = y * cr - z * sr, z1 = y * sr + z * cr;
    ox = x * cp - z1 * sp;
    oy = y1;
    oz = x * sp + z1 * cp;
  }
};

// Лапа в мире → (x, z) в плоскости ноги (x вперёд, z вниз от оси бедра).
// Пол z=0, центр корпуса в (0, 0, height), hip — ось бедра в системе корпуса.
inline void foot_in_leg(const double foot[3], const double hip_b[3], double height,
                        double pitch_deg, double roll_deg, double &x, double &z) {
  BodyRotation r(pitch_deg, roll_deg);
  double hx, hy, hz, ax, ay, az, bx, by, bz;
  r.rot(hip_b[0], hip_b[1], hip_b[2], hx, hy, hz);
  r.rot(1, 0, 0, ax, ay, az);    // ось x ноги в мире
  r.rot(0, 0, -1, bx, by, bz);   // ось z ноги (вниз) в мире
  double tx = foot[0] - hx, tz = foot[2] - (height + hz);
  double det = ax * bz - bx * az;
  x = (tx * bz - bx * tz) / det;
  z = (ax * tz - tx * az) / det;
}

// Защита от упора узла колена в раму (как collision.py): голень, вставка и серво колена
// не поднимаются выше низа рамы минус запас. Координаты: x вперёд, y вверх от оси бедра.
#include "leg_hull.h"

struct FrameGuard {
  double limit, bend;
  FrameGuard(double bend_deg, double margin_mm) : limit(HULL_FRAME_BOTTOM_MM - margin_mm), bend(bend_deg) {}

  // Самая высокая точка узла колена, мм от оси бедра (вверх +).
  double top(double hip_deg, double knee_deg) const {
    double h = rad(hip_deg), k = rad(knee_deg - bend);
    double sh = std::sin(h), ch = std::cos(h), sk = std::sin(k), ck = std::cos(k);
    double t = -INFINITY;
    for (int i = 0; i < HULL_KNEE_N; i++) {
      double x = HULL_KNEE[i][0], y = HULL_KNEE[i][1];
      double xk = x * ck - y * sk, yk = x * sk + y * ck - HULL_FEMUR_MM;
      t = std::fmax(t, xk * sh + yk * ch);
    }
    return t;
  }

  // Шаг разрешён, если нога остаётся ниже плоскости или хотя бы опускается к ней.
  bool allows_move(double old_hip, double old_knee, double new_hip, double new_knee) const {
    double t = top(new_hip, new_knee);
    return t <= limit || t <= top(old_hip, old_knee) + 1e-6;
  }
};
