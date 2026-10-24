// Разбор отчётов HID геймпада: дескриптор отчёта → поля, отчёт → оси, крестовина, нажатия.
// Кнопки и оси называются так же, как их назвал бы hid-input в ядре Linux (BTN_A, ABS_RZ, …):
// тогда раскладка из config.toml работает без изменений.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

struct HidEvents {
  std::function<void(const char *code, double t)> axis;   // t: 0 … 1 по логическому диапазону оси
  std::function<void(int x, int y)> hat;                   // крестовина: −1 вверх/влево, 1 вниз/вправо
  std::function<void(const std::string &code)> press;      // кнопка нажата (было отпущено)
};

class HidDecoder {
 public:
  void parse(const uint8_t *desc, size_t n);
  void decode(uint8_t report, const uint8_t *data, size_t n, const HidEvents &ev);
  void release_all() { pressed_.clear(); }
  size_t fields() const { return fields_.size(); }

 private:
  struct Field {
    uint8_t report;
    uint32_t bit;
    uint8_t size;
    int32_t lmin, lmax;
    uint32_t usage;   // страница << 16 | usage; у массива — первая usage диапазона
    bool array;
  };
  std::vector<Field> fields_;
  std::map<uint8_t, std::map<std::string, bool>> pressed_;   // номер отчёта → кнопки, нажатые в прошлый раз
};
