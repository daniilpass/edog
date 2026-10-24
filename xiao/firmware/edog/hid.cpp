#include "hid.h"

static const uint16_t PAGE_DESKTOP = 0x01, PAGE_SIMULATION = 0x02, PAGE_BUTTON = 0x09, PAGE_CONSUMER = 0x0C;
static const uint32_t HAT = PAGE_DESKTOP << 16 | 0x39;

static const char *axis_code(uint32_t usage) {
  switch (usage) {
    case PAGE_DESKTOP << 16 | 0x30: return "ABS_X";
    case PAGE_DESKTOP << 16 | 0x31: return "ABS_Y";
    case PAGE_DESKTOP << 16 | 0x32: return "ABS_Z";
    case PAGE_DESKTOP << 16 | 0x33: return "ABS_RX";
    case PAGE_DESKTOP << 16 | 0x34: return "ABS_RY";
    case PAGE_DESKTOP << 16 | 0x35: return "ABS_RZ";
    case PAGE_DESKTOP << 16 | 0x36: return "ABS_THROTTLE";
    case PAGE_SIMULATION << 16 | 0xBA: return "ABS_RUDDER";
    case PAGE_SIMULATION << 16 | 0xBB: return "ABS_THROTTLE";
    case PAGE_SIMULATION << 16 | 0xC4: return "ABS_GAS";
    case PAGE_SIMULATION << 16 | 0xC5: return "ABS_BRAKE";
  }
  return nullptr;
}

// Кнопка N геймпада → BTN_GAMEPAD + N − 1 (имена по подписям Xbox, как выбирает gamepad.py).
static std::string key_code(uint32_t usage) {
  uint16_t page = usage >> 16, id = usage & 0xFFFF;
  if (page == PAGE_BUTTON) {
    static const char *const names[] = {"BTN_A", "BTN_B", "BTN_C", "BTN_X", "BTN_Y", "BTN_Z", "BTN_TL", "BTN_TR",
                                        "BTN_TL2", "BTN_TR2", "BTN_SELECT", "BTN_START", "BTN_MODE", "BTN_THUMBL",
                                        "BTN_THUMBR"};
    if (id >= 1 && id <= 15) return names[id - 1];
    if (id == 16) return "319";   // у кода 0x13f нет имени
    return "BTN_TRIGGER_HAPPY" + std::to_string(id - 16);
  }
  if (page == PAGE_CONSUMER) {
    if (id == 0x224) return "KEY_BACK";
    if (id == 0x223) return "KEY_HOMEPAGE";
    if (id == 0xB2) return "KEY_RECORD";
  }
  return "";
}

void HidDecoder::parse(const uint8_t *d, size_t n) {
  struct Globals {
    uint16_t page = 0;
    int32_t lmin = 0, lmax = 0;
    uint32_t size = 0, count = 0;
    uint8_t id = 0;
  };
  Globals g;
  std::vector<Globals> stack;
  std::vector<uint32_t> usages;
  uint32_t umin = 0, umax = 0;
  bool range = false;
  std::map<uint8_t, uint32_t> bits;   // номер отчёта → занято бит входных полей
  fields_.clear();
  pressed_.clear();
  auto full = [&](uint32_t u, int sz) { return sz == 4 ? u : (uint32_t)g.page << 16 | u; };
  for (size_t i = 0; i < n;) {
    uint8_t b = d[i];
    if (b == 0xFE) {   // длинный элемент
      i += (i + 1 < n ? d[i + 1] : 0) + 3;
      continue;
    }
    int sz = (b & 3) == 3 ? 4 : (b & 3);
    if (i + 1 + sz > n) break;
    uint32_t u = 0;
    for (int k = 0; k < sz; k++) u |= (uint32_t)d[i + 1 + k] << (8 * k);
    int32_t s = sz == 1 ? (int8_t)u : sz == 2 ? (int16_t)u : (int32_t)u;
    int type = (b >> 2) & 3, tag = b >> 4;
    i += 1 + sz;
    if (type == 0) {   // main
      if (tag == 0x8 && !(u & 1)) {   // Input, не константа
        uint32_t pos = bits[g.id];
        for (uint32_t k = 0; k < g.count; k++) {
          Field f = {g.id, pos + k * g.size, (uint8_t)g.size, g.lmin, g.lmax, 0, !(u & 2)};
          if (f.array) f.usage = range ? umin : usages.empty() ? 0 : usages[0];
          else if (k < usages.size()) f.usage = usages[k];
          else if (range && umin + (k - usages.size()) <= umax) f.usage = umin + (k - usages.size());
          else f.usage = usages.empty() ? 0 : usages.back();
          uint16_t page = f.usage >> 16;
          bool wanted = f.array ? page == PAGE_BUTTON || page == PAGE_CONSUMER
                                : axis_code(f.usage) || f.usage == HAT || !key_code(f.usage).empty();
          if (wanted && f.size <= 32) fields_.push_back(f);
        }
      }
      if (tag == 0x8) bits[g.id] += g.size * g.count;
      usages.clear();
      range = false;
    } else if (type == 1) {   // global
      switch (tag) {
        case 0x0: g.page = u; break;
        case 0x1: g.lmin = s; break;
        case 0x2: g.lmax = g.lmin < 0 ? s : (int32_t)u; break;
        case 0x7: g.size = u; break;
        case 0x8: g.id = u; break;
        case 0x9: g.count = u; break;
        case 0xA: stack.push_back(g); break;
        case 0xB:
          if (!stack.empty()) {
            g = stack.back();
            stack.pop_back();
          }
          break;
      }
    } else if (type == 2) {   // local
      if (tag == 0x0) usages.push_back(full(u, sz));
      else if (tag == 0x1) umin = full(u, sz), range = true;
      else if (tag == 0x2) umax = full(u, sz);
    }
  }
}

void HidDecoder::decode(uint8_t report, const uint8_t *d, size_t n, const HidEvents &ev) {
  std::map<std::string, bool> keys;   // кнопки, нажатые в этом отчёте
  for (const Field &f : fields_) {
    if (f.report != report || f.bit + f.size > n * 8) continue;
    uint32_t raw = 0;
    for (int k = 0; k < f.size; k++) {
      uint32_t b = f.bit + k;
      raw |= (uint32_t)((d[b / 8] >> (b % 8)) & 1) << k;
    }
    if (f.lmin < 0 && f.size < 32 && (raw >> (f.size - 1)) & 1) raw |= ~0u << f.size;   // знаковое
    int32_t v = (int32_t)raw;
    if (f.array) {   // массив: значение — номер usage нажатой кнопки, 0 — ничего
      if (v && v >= f.lmin && v <= f.lmax) {
        std::string code = key_code(f.usage + (v - f.lmin));
        if (!code.empty()) keys[code] = true;
      }
    } else if (f.usage == HAT) {   // 8 направлений по часовой от «вверх», вне диапазона — центр
      static const int8_t DIRS[8][2] = {{0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}};
      int idx = v - f.lmin;
      bool center = idx < 0 || idx > 7 || v > f.lmax;
      ev.hat(center ? 0 : DIRS[idx][0], center ? 0 : DIRS[idx][1]);
    } else if (const char *code = axis_code(f.usage)) {
      if (f.lmax != f.lmin) ev.axis(code, (double)(v - f.lmin) / (f.lmax - f.lmin));
    } else if (v) {
      keys[key_code(f.usage)] = true;
    }
  }
  // Нажатие — по фронту, как событие evdev со значением 1. Кнопки, пропавшие из отчёта, отпущены.
  std::map<std::string, bool> &before = pressed_[report];
  for (auto &k : keys)
    if (!before.count(k.first)) ev.press(k.first);
  before = keys;
}
