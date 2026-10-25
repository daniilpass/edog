// Конфигурация EDOG для XIAO ESP32-S3 Sense. Перенесено из rpi/firmware/config/config.toml,
// правится здесь и прошивается заново. Калибровка серво, лимит тока, профиль шага и поправка
// наклона пишутся из портала в NVS платы и перекрывают значения отсюда.
#pragma once

#include <cmath>

// ---------- сеть ----------
static const char *HOSTNAME = "edog";        // edog.local
static const char *AP_SSID = "EDOG-XIAO";    // точка доступа, если сеть не задана или недоступна
static const char *AP_PASS = "edog1234";
static const int API_PORT = 8080;             // REST + WebSocket /ws (портал по умолчанию ищет edog.local:8080)
static const int STREAM_PORT = 8081;          // видео MJPEG /stream

// ---------- камера ----------
static const int CAMERA_FPS = 8;              // кадров в секунду в потоке
#define CAMERA_FRAMESIZE FRAMESIZE_VGA        // 640×480
static const int CAMERA_QUALITY = 12;         // JPEG: 0 — лучше … 63 — хуже
static const bool CAMERA_VFLIP = false;       // камера стоит вверх ногами
static const bool CAMERA_HMIRROR = true;

// ---------- память ----------
// Внутренней памяти мало (Wi-Fi, Bluetooth, камера, API), PSRAM — 8 МБ. Обычный malloc от этого размера
// идёт в PSRAM (в ядре — от 4096). Ставится в конце setup(), когда драйверы уже взяли своё.
static const size_t EXTMEM_LIMIT = 512;

// ---------- зрение ----------
// Камера как точечная: по углу обзора, высоте и наклону строка кадра переводится в расстояние по полу.
// Высота и наклон уточняются калибровкой пола из портала (хранится в NVS), здесь — начальные значения.
static const double CAMERA_HFOV_DEG = 56.0;        // угол обзора по горизонтали (OV2640, штатный объектив)
static const double CAMERA_HEIGHT_MM = 110.0;      // высота объектива над полом в стойке
static const double CAMERA_TILT_DEG = 12.0;        // наклон оптической оси вниз от горизонта
static const double VISION_PHASE = 0.0;            // фаза шага (0 … 1), в которую снимаются кадры на ходу
static const double VISION_PHASE_TOL = 0.05;       // кадр дальше этого от фазы (доля цикла) не берётся
static const bool VISION_PHASE_BOTH = false;       // брать и вторую диагональ (фаза + 0.5): кадров вдвое больше
static const double VISION_SETTLE_S = 0.3;         // стоя: кадр берётся через столько после остановки
static const float PERSON_SCORE = 0.5f;            // уверенность детектора, ниже — не человек
static const double PERSON_HEIGHT_MM = 1650.0;     // рост для оценки расстояния по высоте рамки
static const double FREE_MAX_MM = 3000.0;          // свободное расстояние дальше этого — «далеко»

// ---------- патруль ----------
static const double PATROL_IGNORE_S = 18.0;        // после приветствия людей не замечать
static const double PATROL_TURN_DPS = 45.0;        // скорость разворота на месте, °/с (уточняется по камере)
static const double PATROL_SLIP = 0.7;             // доля расчётной скорости шага, которая доходит до пола
static const double PATROL_GREET_MM = 1100.0;      // подходить к человеку до этого расстояния
static const double PATROL_TOO_CLOSE_MM = 500.0;   // ближе — сначала отойти назад
static const double PATROL_MEMORY_S = 480.0;       // зрительная память мест: столько помнит увиденное

// ---------- драйвер PCA9685 ----------
static const int I2C_SDA = 5;                 // D4
static const int I2C_SCL = 6;                 // D5
static const uint32_t I2C_HZ = 400000;
static const uint8_t PCA_ADDRESS = 0x40;
static const double PCA_FREQUENCY_HZ = 50;
static const double PCA_OSCILLATOR_HZ = 25000000;  // у разных плат 25–27 МГц; ошибку хода закрывает scale калибровки

// ---------- геометрия ----------
static const double FEMUR_MM = 50.0;          // между осями крыльев на Femur (femur_*.stl: 50 мм)
static const double TIBIA_MM = 41.7;          // ось колена → середина подошвы (tibia.stl)
static const double TIBIA_BEND_DEG = 17.7;    // изгиб голени: линия «ось колена → подошва» отклонена от корпуса голени
static const double FRAME_MARGIN_MM = 3.0;    // запас: узел колена держится ниже плоскости рамы на столько мм
static const double BODY_LENGTH_MM = 200.0;
static const double BODY_WIDTH_MM = 95.0;     // колея: расстояние между лапами левого и правого борта

// ---------- движение ----------
static const double LOOP_HZ = 100;            // чаще ШИМ 50 Гц: каждый период серво получает свежий импульс
static const double MAX_SPEED_DPS = 300;      // ограничение скорости поворота серво, °/с
static const double STAND_HEIGHT_MM = 70.0;   // высота оси бедра над полом в стойке
static const double MIN_HEIGHT_MM = 35.0;
static const double MAX_HEIGHT_MM = 88.0;
static const double HEIGHT_SPEED_MM_S = 30.0; // курки: скорость подъёма/опускания при нажатии до упора
static const double MAX_TILT_DEG = 15.0;      // тангаж: предел и для команды, и для правого стика
static const double MAX_ROLL_DEG = 8.0;       // крен: ноги без отведения, при крене лапы скользят вбок (~1.3 мм/°)
static const char *DEFAULT_GAIT = "base";     // профиль ходьбы по умолчанию; выбор из портала — в NVS
static const double DRIVE_TIMEOUT_S = 0.5;    // команда drive из веба гаснет без повторения

// Профили ходьбы (рысь): их же показывает демо.
// gait_hz — шагов в секунду; stride_mm — длина шага при полном отклонении стика; lift_mm — подъём лапы;
// foot_x_mm — лапа впереди (+) или позади (−) оси бедра; height_mm — высота оси бедра в стойке профиля
// и при ходьбе (NAN — stand_height_mm).
struct GaitCfg {
  const char *name;
  const char *title;
  double gait_hz, stride_mm, lift_mm, foot_x_mm, height_mm;
};
static const GaitCfg GAITS[] = {
  // foot_x −15: против заваливания на задние лапы; −10 мало, −20 клюёт носом
  {"base", "Базовая", 1.6, 30.0, 15.0, -15.0, NAN},
  // всё как у base, корпус чуть выше: лапа опирается на стоппер
  {"base_stopper", "Базовая на стопперах", 1.6, 30.0, 15.0, -15.0, 78.0},
};
static const int GAIT_COUNT = sizeof(GAITS) / sizeof(GAITS[0]);

// Лимит тока серво: «подключёнными» можно отметить не больше limit_a / servo_a приводов.
static const double POWER_LIMIT_A = 8.0;      // что отдаёт питание серво, А; 0 — без ограничения
static const double POWER_SERVO_A = 0.9;      // пиковый ток одного MG90S, А

// ---------- серво ----------
static const double SERVO_RANGE_DEG = 180.0;
static const double SERVO_PULSE_MIN_US = 500.0;   // импульс для 0°
static const double SERVO_PULSE_MAX_US = 2500.0;  // импульс для range_deg
static const double SERVO_MIN_DEG = 5.0;          // программные пределы угла серво
static const double SERVO_MAX_DEG = 175.0;
// Монтажное положение (ref_deg) и калибровочная поза под нагрузкой (load_deg) суставов.
static const double HIP_REF_DEG = 0.0, HIP_LOAD_DEG = -45.0;     // бедро вниз; под нагрузкой назад на 45°
static const double KNEE_REF_DEG = 90.0, KNEE_LOAD_DEG = 90.0;   // голень под прямым углом к бедру

// Ноги. x_mm — смещение оси бедра от центра корпуса (+ вперёд). knee_dir: −1 — колено смотрит назад.
// dir: знак связи «угол сустава → угол серво». Каналы PCA9685: бедро / колено.
struct LegDef {
  const char *name;
  const char *side;
  double x_mm;
  int knee_dir;
  int hip_channel, hip_dir, knee_channel, knee_dir_servo;
};
static const LegDef LEG_DEFS[] = {
  {"FL", "left", 56.2, -1, 4, -1, 5, 1},
  {"FR", "right", 56.2, -1, 8, 1, 9, -1},
  {"RL", "left", -56.2, -1, 6, -1, 7, 1},
  {"RR", "right", -56.2, -1, 10, 1, 11, -1},
};

// ---------- геймпад (BLE HID) ----------
// Кнопки и оси названы кодами Linux evdev, как на Raspberry Pi: прошивка переводит в них HID-отчёт
// геймпада так же, как ядро Linux (кнопка N → BTN_A + N − 1, X/Y/Z/Rz → ABS_*, тормоз/газ → ABS_BRAKE/GAS).
static const double GAMEPAD_DEADZONE = 0.25;  // доля хода стика от центра, которая не считается
struct Mapping { const char *key, *value; };
static const Mapping GAMEPAD_AXES[] = {       // роль → ось
  {"drive", "ABS_Y"},          // левый стик: вперёд/назад
  {"turn", "ABS_X"},           // левый стик: поворот
  {"pitch", "ABS_RZ"},         // правый стик ↕: тангаж (Xbox; у DualShock и др. — ABS_RY)
  {"roll", "ABS_Z"},           // правый стик ↔: крен (Xbox; у DualShock и др. — ABS_RX)
  {"height_down", "ABS_BRAKE"},// левый курок: опускать, пока нажат
  {"height_up", "ABS_GAS"},    // правый курок: поднимать
};
static const Mapping GAMEPAD_BUTTONS[] = {    // кнопка → действие; имена по подписям Xbox
  {"BTN_A", "stand"},
  {"BTN_B", "sit"},
  {"BTN_X", "lie"},
  {"BTN_Y", "high"},
  {"BTN_START", "torque_toggle"},  // Menu / Options
  {"BTN_SELECT", "next_gait"},     // View / Share: следующий профиль ходьбы
  {"BTN_TL", "roll_left"},         // LB / L1: крен −1°, держится до выключения серво
  {"BTN_TR", "roll_right"},        // RB / R1: крен +1°
};
static const Mapping GAMEPAD_DPAD[] = {       // крестовина → действие
  {"up", "jump"},
  {"down", "joy"},
  {"right", "patrol"},          // патруль вкл/выкл
};
