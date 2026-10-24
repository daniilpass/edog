#include "gamepad.h"

#include <NimBLEDevice.h>
#include <Preferences.h>

#include <map>

#include "hid.h"
#include "patrol.h"

// ---------- раскладка (как gamepad.py) ----------

static Robot *robot;
static double values[6];                     // drive, turn, pitch, roll, height_down, height_up
static int hat_x, hat_y;                     // крестовина: −1 вверх/влево, 1 вниз/вправо
static HidDecoder hid;

static int axis_role(const char *code) {
  for (const Mapping &m : GAMEPAD_AXES)
    if (!strcmp(m.value, code))
      for (int i = 0; i < 6; i++) if (!strcmp(PAD_AXES[i], m.key)) return i;
  return -1;
}

static const char *lookup(const Mapping *table, size_t n, const std::string &key) {
  for (size_t i = 0; i < n; i++) if (key == table[i].key) return table[i].value;
  return nullptr;
}

static double norm_stick(double v) {   // −1 … 1 с мёртвой зоной
  if (std::fabs(v) < GAMEPAD_DEADZONE) return 0.0;
  return (std::fabs(v) - GAMEPAD_DEADZONE) / (1 - GAMEPAD_DEADZONE) * (v > 0 ? 1 : -1);
}

static double norm_trigger(double v) {   // курки: 0 отпущен … 1 нажат до упора
  return v < GAMEPAD_DEADZONE ? 0.0 : (v - GAMEPAD_DEADZONE) / (1 - GAMEPAD_DEADZONE);
}

static void apply() {
  double drive = values[0], turn = values[1];
  // Левый стик: только та ось, что отклонена сильнее, — вперёд без случайного поворота.
  // Задаёт только направление: шаг всегда полный, как у стрелок портала.
  if (std::fabs(drive) >= std::fabs(turn)) {
    drive = drive ? std::copysign(1.0, drive) : 0.0;
    turn = 0.0;
  } else {
    drive = 0.0;
    turn = std::copysign(1.0, turn);
  }
  // Ось Y стика: вверх — отрицательные значения.
  robot->pad_drive[0] = -drive;
  robot->pad_drive[1] = turn;
  robot->tilt[0] = -values[2];
  robot->tilt[1] = values[3];
  robot->height_rate = values[5] - values[4];
  for (int i = 0; i < 6; i++) robot->gamepad.axes[i] = values[i];
}

static void action(const std::string &a) {
  Robot &r = *robot;
  try {
    if (r.is_pose(a)) {
      // Повторное нажатие кнопки текущей позы возвращает в стойку.
      r.set_pose(r.pose_name == a ? "stand" : a);
    } else if (r.is_move(a)) {
      r.start_move(a);
    } else if (a == "torque_toggle") {
      r.set_enabled(!r.enabled);
    } else if (a == "roll_left" || a == "roll_right") {
      r.nudge_roll(a == "roll_right" ? 1 : -1);
    } else if (a == "next_gait") {
      r.next_gait();
    } else if (a == "estop") {
      r.estop();
    } else if (a == "patrol") {
      patrol_toggle();
    } else {
      Serial.printf("неизвестное действие геймпада: %s\n", a.c_str());
    }
  } catch (const std::exception &e) {   // серво выключены, лимит тока
    Serial.printf("геймпад: %s\n", e.what());
  }
}

static void button_down(const std::string &name) {
  robot->gamepad.last_button = name;
  robot->pad_presses++;
  if (const char *a = lookup(GAMEPAD_BUTTONS, std::size(GAMEPAD_BUTTONS), name)) action(a);
}

static void dpad(const char *side) {
  std::string name = std::string("DPAD_") + side;
  for (char &c : name) c = toupper(c);
  robot->gamepad.last_button = name;
  robot->pad_presses++;
  if (const char *a = lookup(GAMEPAD_DPAD, std::size(GAMEPAD_DPAD), side)) action(a);
}

static void hat(int x, int y) {
  if (y != hat_y && y) dpad(y < 0 ? "up" : "down");
  if (x != hat_x && x) dpad(x < 0 ? "left" : "right");
  hat_x = x;
  hat_y = y;
}

static void reset_pad() {
  for (double &v : values) v = 0.0;
  hid.release_all();
  hat_x = hat_y = 0;
  robot->gamepad = PadInfo();
  apply();
}

// ---------- отчёты HID ----------

static void on_report(uint8_t report, const uint8_t *d, size_t n) {
  RobotLock lock;
  hid.decode(report, d, n, {
    [](const char *code, double t) {
      int role = axis_role(code);
      if (role >= 0) values[role] = role >= 4 ? norm_trigger(t) : norm_stick(2 * t - 1);
    },
    hat,
    button_down,
  });
  apply();
}

// ---------- BLE ----------

static const NimBLEUUID HID_SERVICE((uint16_t)0x1812), REPORT_MAP((uint16_t)0x2A4B), REPORT((uint16_t)0x2A4D),
    REPORT_REF((uint16_t)0x2908), GAP_SERVICE((uint16_t)0x1800), DEVICE_NAME((uint16_t)0x2A00);

static NimBLEClient *client;
static std::vector<std::pair<uint16_t, uint8_t>> report_ids;   // хэндл характеристики → номер отчёта
static volatile bool pad_ready;
static std::map<std::string, std::pair<uint8_t, std::string>> seen;   // MAC → (тип адреса, имя) из поиска

static std::string upper(std::string s) {
  for (char &c : s) c = toupper(c);
  return s;
}

static std::string lower(std::string s) {
  for (char &c : s) c = tolower(c);
  return s;
}

static std::string mac_of(const NimBLEAddress &a) { return upper(a.toString()); }

static std::string name_key(const std::string &mac) {   // ключ NVS: не длиннее 15 символов
  std::string k;
  for (char c : mac) if (c != ':') k += c;
  return k;
}

static std::string stored_name(const std::string &mac) {
  Preferences p;
  if (!p.begin("btnames", true)) return "";
  String n = p.getString(name_key(mac).c_str(), "");
  p.end();
  return n.c_str();
}

static void store_name(const std::string &mac, const std::string &name) {
  Preferences p;
  p.begin("btnames", false);
  if (name.empty()) p.remove(name_key(mac).c_str());
  else p.putString(name_key(mac).c_str(), name.c_str());
  p.end();
}

static bool bonded(const std::string &mac) {
  for (int i = 0; i < NimBLEDevice::getNumBonds(); i++)
    if (mac_of(NimBLEDevice::getBondedAddress(i)) == mac) return true;
  return false;
}

static bool connected_to(const std::string &mac) {
  return pad_ready && client && client->isConnected() && mac_of(client->getPeerAddress()) == mac;
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c, int reason) override {
    if (!pad_ready) return;
    pad_ready = false;
    Serial.printf("геймпад отключён (причина %d)\n", reason);
    // Потеря связи: стоп ходьбы, корпус выравнивается. Поза сохраняется.
    RobotLock lock;
    reset_pad();
  }
  // Геймпад без экрана и клавиатуры: подтверждаем без PIN.
  void onConfirmPasskey(NimBLEConnInfo &info, uint32_t pin) override { NimBLEDevice::injectConfirmPasskey(info, true); }
};
static ClientCallbacks callbacks;

// Подключиться, зашифровать (при первом подключении — сопряжение) и подписаться на отчёты HID.
// Пустая строка — успех, иначе причина.
static std::string connect_pad(const NimBLEAddress &addr, std::string name) {
  std::string mac = mac_of(addr);
  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&callbacks, false);
  }
  if (client->isConnected()) client->disconnect();
  client->setConnectTimeout(4000);
  // Интервал 7.5–15 мс (как у Linux на Pi). По умолчанию NimBLE держит 30–50 мс и на просьбу
  // геймпада ускориться подставляет их же — отчёты копятся в геймпаде, стик отзывается с лагом.
  client->setConnectionParams(6, 12, 0, 200);
  if (!client->connect(addr)) return "нет ответа";
  if (!client->secureConnection()) {
    client->disconnect();
    return "шифрование не удалось";
  }
  NimBLERemoteService *service = client->getService(HID_SERVICE);
  NimBLERemoteCharacteristic *map = service ? service->getCharacteristic(REPORT_MAP) : nullptr;
  if (!map) {
    client->disconnect();
    return "это не геймпад: нет сервиса HID";
  }
  NimBLEAttValue desc = map->readValue();
  {
    RobotLock lock;
    hid.parse(desc.data(), desc.size());
  }
  report_ids.clear();
  int subscribed = 0;
  for (NimBLERemoteCharacteristic *chr : service->getCharacteristics(true)) {
    if (chr->getUUID() != REPORT || !chr->canNotify()) continue;
    uint8_t id = 0;
    if (NimBLERemoteDescriptor *ref = chr->getDescriptor(REPORT_REF)) {
      NimBLEAttValue v = ref->readValue();
      if (v.size() >= 2 && v.data()[1] != 1) continue;   // не входной отчёт
      if (v.size() >= 1) id = v.data()[0];
    }
    report_ids.push_back({chr->getHandle(), id});
    subscribed += chr->subscribe(true, [](NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool) {
      for (auto &r : report_ids)
        if (r.first == c->getHandle()) on_report(r.second, data, len);
    });
  }
  if (!subscribed) {
    client->disconnect();
    return "геймпад не отдаёт отчёты HID";
  }
  if (NimBLERemoteService *gap = client->getService(GAP_SERVICE))
    if (NimBLERemoteCharacteristic *c = gap->getCharacteristic(DEVICE_NAME)) {
      std::string n = c->readValue();
      if (!n.empty()) name = n;
    }
  if (name.empty()) name = mac;
  store_name(mac, name);
  {
    RobotLock lock;
    reset_pad();
    robot->gamepad.connected = true;
    robot->gamepad.name = name;
  }
  pad_ready = true;
  Serial.printf("геймпад подключён: %s (%s), полей HID: %u, интервал %.2f мс\n", name.c_str(), mac.c_str(),
                (unsigned)hid.fields(), client->getConnInfo().getConnInterval() * 1.25);
  return "";
}

// ---------- задача BLE и запросы портала ----------

enum ReqKind { SCAN, PAIR, REMOVE };
struct Request {
  ReqKind kind;
  int seconds;
  std::string mac, result, error;
  std::vector<BtDevice> devices;
  SemaphoreHandle_t done;
};
static QueueHandle_t requests;

static void run_scan(int seconds, bool active) {
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(active);   // активный поиск — чтобы узнать имена
  scan->setInterval(100);
  scan->setWindow(active ? 90 : 25);   // в простое — четверть эфира, остальное Wi-Fi
  NimBLEScanResults res = scan->getResults(seconds * 1000);
  for (int i = 0; i < res.getCount(); i++) {
    const NimBLEAdvertisedDevice *d = res.getDevice(i);
    std::string mac = mac_of(d->getAddress());
    std::string name = d->haveName() ? d->getName() : seen[mac].second;
    seen[mac] = {d->getAddress().getType(), name};
  }
}

static void handle(Request &r) {
  if (r.kind == SCAN) {
    run_scan(r.seconds, true);
    NimBLEScanResults res = NimBLEDevice::getScan()->getResults();
    std::map<std::string, bool> listed;
    for (int i = 0; i < res.getCount(); i++) {
      std::string mac = mac_of(res.getDevice(i)->getAddress());
      std::string name = seen[mac].second;
      if (name.empty()) name = stored_name(mac);
      if (name.empty()) name = mac;   // безымянное — как у bluetoothctl
      r.devices.push_back({mac, name, bonded(mac), connected_to(mac)});
      listed[mac] = true;
    }
    for (int i = 0; i < NimBLEDevice::getNumBonds(); i++) {   // сопряжённые, но сейчас выключенные
      std::string mac = mac_of(NimBLEDevice::getBondedAddress(i));
      if (listed.count(mac)) continue;
      std::string name = stored_name(mac);
      r.devices.push_back({mac, name.empty() ? mac : name, true, connected_to(mac)});
    }
  } else if (r.kind == PAIR) {
    if (!seen.count(r.mac)) run_scan(8, true);   // устройство должно быть найдено поиском, иначе тип адреса неизвестен
    if (!seen.count(r.mac)) {
      r.error = "устройство не найдено. Переведите геймпад в режим сопряжения и повторите";
      return;
    }
    if (connected_to(r.mac)) {
      r.result = "уже подключён";
      return;
    }
    NimBLEAddress addr(lower(r.mac), seen[r.mac].first);
    std::string why = connect_pad(addr, seen[r.mac].second);
    if (!why.empty()) {
      r.error = "сопряжение не удалось (" + why + "). Переведите геймпад в режим сопряжения и повторите";
      return;
    }
    if (!bonded(r.mac)) {
      r.result = "подключён, но не сопряжён: после выключения геймпада подключение не восстановится";
      return;
    }
    r.result = "сопряжён и подключён";
  } else if (r.kind == REMOVE) {
    if (connected_to(r.mac)) client->disconnect();
    bool found = false;
    for (int i = 0; i < NimBLEDevice::getNumBonds(); i++) {
      NimBLEAddress a = NimBLEDevice::getBondedAddress(i);
      if (mac_of(a) == r.mac) found = NimBLEDevice::deleteBond(a);
    }
    store_name(r.mac, "");
    r.result = found ? "устройство забыто" : "устройство не было сопряжено";
  }
}

// В простое: нет геймпада, но есть сопряжённый — ищем его рекламу и подключаемся.
static void reconnect() {
  run_scan(2, false);
  NimBLEScanResults res = NimBLEDevice::getScan()->getResults();
  for (int i = 0; i < res.getCount(); i++) {
    const NimBLEAdvertisedDevice *d = res.getDevice(i);
    std::string mac = mac_of(d->getAddress());
    if (!bonded(mac)) continue;
    std::string why = connect_pad(d->getAddress(), stored_name(mac));
    if (why.empty()) return;
    Serial.printf("геймпад %s: не подключился (%s)\n", mac.c_str(), why.c_str());
  }
}

static void ble_task(void *) {
  for (;;) {
    Request *r;
    bool idle = !pad_ready && NimBLEDevice::getNumBonds() > 0;
    if (xQueueReceive(requests, &r, idle ? 0 : pdMS_TO_TICKS(500)) == pdTRUE) {
      handle(*r);
      xSemaphoreGive(r->done);
    } else if (idle) {
      reconnect();
    }
  }
}

static std::string call(Request &r) {
  r.done = xSemaphoreCreateBinary();
  Request *p = &r;
  xQueueSend(requests, &p, portMAX_DELAY);
  xSemaphoreTake(r.done, portMAX_DELAY);
  vSemaphoreDelete(r.done);
  if (!r.error.empty()) throw BluetoothError(r.error);
  return r.result;
}

static std::string check_mac(std::string mac) {
  mac = upper(mac);
  bool ok = mac.size() == 17;
  for (size_t i = 0; ok && i < 17; i++) ok = i % 3 == 2 ? mac[i] == ':' : isxdigit((unsigned char)mac[i]);
  if (!ok) throw BluetoothError("неверный MAC");
  return mac;
}

std::vector<BtDevice> bt_scan(int seconds) {
  Request r{SCAN, seconds};
  call(r);
  return r.devices;
}

std::string bt_pair(const std::string &mac) {
  Request r{PAIR, 0, check_mac(mac)};
  return call(r);
}

std::string bt_remove(const std::string &mac) {
  Request r{REMOVE, 0, check_mac(mac)};
  return call(r);
}

void gamepad_begin(Robot &r) {
  robot = &r;
  NimBLEDevice::init("EDOG");
  NimBLEDevice::setSecurityAuth(true, false, true);   // сопряжение с запоминанием, без PIN
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  requests = xQueueCreate(4, sizeof(Request *));
  {
    RobotLock lock;
    reset_pad();
  }
  xTaskCreatePinnedToCore(ble_task, "ble", 8192, nullptr, 3, nullptr, 0);
  Serial.printf("Bluetooth: сопряжённых устройств %d\n", NimBLEDevice::getNumBonds());
}
