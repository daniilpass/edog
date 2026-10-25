// Зрение собаки: что видно на кадре и как далеко. Всё по одной камере, связка простых инструментов:
//   люди — нейросеть pedestrian_detect, расстояние по ступням на полу или по высоте рамки;
//   пол — цвет у лап, по 16 полосам до первого «не пола»; строка → расстояние по геометрии камеры;
//   закрытость — пол внизу пропал, кадр однотонный, центр растёт («почти упёрлись»);
//   выравнивание соседних кадров — остаточная разница (упёрлась), сдвиг (поворот), расширение.
// На ходу кадры для анализа снимаются в одной фазе шага, стоя — после остановки.
#pragma once

#include <ArduinoJson.h>

#include <vector>

static const int FREE_COLS = 16;     // полосы свободного места по ширине кадра
static const int THUMB_W = 16, THUMB_H = 12;

struct Person {
  int x1, y1, x2, y2;     // рамка в кадре 640×480
  float score;
  double dist_mm;         // NAN — не оценить
  double bearing_deg;     // + — правее носа
};

struct Perception {
  uint32_t seq = 0;       // номер анализа, 0 — ещё не было
  double t = 0;           // время съёмки кадра, с
  bool synced = false;    // снят на ходу в выбранной фазе шага
  double vx = 0, turn = 0;   // команда движения в момент съёмки
  std::vector<Person> people;
  float free_mm[FREE_COLS];  // свободно по полу в полосе; NAN — пол неизвестен
  float closeness = 0;       // 0 … 1: насколько кадр «закрыт» близким предметом
  float bottom_floor = 0;    // доля пола в нижней полосе кадра
  float flatness = 0;        // доля однотонного кадра
  float expansion = 0;       // 0 … 1: центр кадра растёт (приближаемся)
  float residual = NAN;      // разница с прошлым синхронным кадром после выравнивания, уровни яркости
  float yaw_deg = NAN;       // поворот с прошлого синхронного кадра по сдвигу картинки, + — вправо
  double dt = 0;             // время с прошлого синхронного кадра, с
  bool stagnant = false;     // на ходу вперёд кадр не меняется третий раз подряд — упёрлась
  bool floor_known = false;
  double h_mm = 0, tilt_deg = 0;   // геометрия камеры на этом кадре (высота — по высоте корпуса)
  float thumb[THUMB_W * THUMB_H];   // нормированная миниатюра для памяти мест
};

void vision_begin();
void vision_on_frame(const uint8_t *jpeg, size_t len, double t);   // из задачи захвата, на каждый кадр
bool vision_latest(Perception &out);                               // false — анализа ещё не было
int vision_headers(char *buf, size_t cap);                         // строки заголовков кадра видео
void vision_state(JsonObject out);

double vision_bearing_deg(double x640);   // направление на столбец кадра, + — правее
double vision_col_bearing_deg(int col);   // направление на середину полосы свободного места
double vision_hfov_deg();

// Команды портала. Ошибки — ValueError.
void vision_phase_scan(double seconds);                  // подобрать фазу съёмки на ходу
void vision_floor_point(double y480, double dist_mm);    // калибровка пола: низ предмета на строке y — dist мм
void vision_floor_reset();
void vision_configure(JsonObjectConst f);                // phase, phase_both, height_mm, tilt_deg
void vision_detect_jpeg(const uint8_t *jpeg, size_t len, JsonObject out);   // проверка детектора на снимке 640×480
