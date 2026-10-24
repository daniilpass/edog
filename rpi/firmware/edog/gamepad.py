"""Bluetooth-геймпад через evdev. Подключение/отключение отслеживается автоматически."""

import asyncio
import logging
from math import copysign

try:
    import evdev
    from evdev import ecodes
except ImportError:  # не Linux или нет python3-evdev
    evdev = None

log = logging.getLogger(__name__)

TRIGGERS = ("height_down", "height_up")   # курки: 0 отпущен … 1 нажат до упора


class Gamepad:
    def __init__(self, robot, cfg: dict):
        self.robot = robot
        self.deadzone = cfg["deadzone"]
        self.axes = {}
        self.buttons = {}
        if evdev is None:
            return
        for role, name in cfg["axes"].items():
            self.axes[ecodes.ecodes[name]] = role
        for name, action in cfg["buttons"].items():
            self.buttons[ecodes.ecodes[name]] = action
        self.dpad = cfg["dpad"]   # up/down/left/right → действие
        self.values = {"drive": 0.0, "turn": 0.0, "pitch": 0.0, "roll": 0.0, "height_down": 0.0, "height_up": 0.0}

    def _find(self):
        for path in evdev.list_devices():
            dev = evdev.InputDevice(path)
            caps = dev.capabilities()
            if ecodes.BTN_SOUTH in caps.get(ecodes.EV_KEY, []) and ecodes.EV_ABS in caps:
                return dev
            dev.close()
        return None

    def _norm(self, dev, code: int, value: int) -> float:
        info = dev.absinfo(code)
        if info.max == info.min:
            return 0.0
        v = 2 * (value - info.min) / (info.max - info.min) - 1
        if abs(v) < self.deadzone:
            return 0.0
        return (abs(v) - self.deadzone) / (1 - self.deadzone) * (1 if v > 0 else -1)

    def _trigger(self, dev, code: int, value: int) -> float:
        info = dev.absinfo(code)
        if info.max == info.min:
            return 0.0
        v = (value - info.min) / (info.max - info.min)
        return 0.0 if v < self.deadzone else (v - self.deadzone) / (1 - self.deadzone)

    def _apply(self) -> None:
        v = self.values
        drive, turn = v["drive"], v["turn"]
        # Левый стик: только та ось, что отклонена сильнее, — вперёд без случайного поворота.
        # Задаёт только направление: шаг всегда полный, как у стрелок портала.
        if abs(drive) >= abs(turn):
            drive, turn = copysign(1.0, drive) if drive else 0.0, 0.0
        else:
            drive, turn = 0.0, copysign(1.0, turn)
        # Ось Y стика: вверх — отрицательные значения.
        self.robot.pad_drive = (-drive, turn)
        self.robot.tilt = (-v["pitch"], v["roll"])
        self.robot.height_rate = v["height_up"] - v["height_down"]
        self.robot.gamepad["axes"] = {k: round(x, 2) for k, x in v.items()}

    def _action(self, action: str) -> None:
        r = self.robot
        if action in r.pose_names():
            # Повторное нажатие кнопки текущей позы возвращает в стойку.
            r.set_pose("stand" if r.pose_name == action else action)
        elif action in r.moves():
            try:
                r.start_move(action)
            except ValueError as e:   # серво выключены
                log.warning("%s", e)
        elif action == "torque_toggle":
            try:
                r.set_enabled(not r.enabled)
            except ValueError as e:   # лимит тока: серво не включаются
                log.warning("%s", e)
        elif action in ("roll_left", "roll_right"):
            r.nudge_roll(1 if action == "roll_right" else -1)
        elif action == "next_gait":
            r.next_gait()
        elif action == "estop":
            r.estop()
        else:
            log.warning("неизвестное действие геймпада: %s", action)

    def _handle(self, dev, ev) -> None:
        if ev.type == ecodes.EV_ABS and ev.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            if ev.value:   # крестовина: −1 вверх/влево, 1 вниз/вправо
                y = ev.code == ecodes.ABS_HAT0Y
                side = ("up" if ev.value < 0 else "down") if y else ("left" if ev.value < 0 else "right")
                self.robot.gamepad["last_button"] = f"DPAD_{side.upper()}"
                if side in self.dpad:
                    self._action(self.dpad[side])
        elif ev.type == ecodes.EV_ABS and ev.code in self.axes:
            role = self.axes[ev.code]
            norm = self._trigger if role in TRIGGERS else self._norm
            self.values[role] = norm(dev, ev.code, ev.value)
            self._apply()
        elif ev.type == ecodes.EV_KEY and ev.value == 1:
            name = ecodes.BTN.get(ev.code) or ecodes.KEY.get(ev.code) or str(ev.code)
            if not isinstance(name, str):  # у кода несколько имён: берём подпись Xbox (BTN_X, а не BTN_NORTH)
                name = next((n for n in name if n in ("BTN_A", "BTN_B", "BTN_X", "BTN_Y")), name[0])
            self.robot.gamepad["last_button"] = name
            if ev.code in self.buttons:
                self._action(self.buttons[ev.code])

    def _reset(self) -> None:
        for k in self.values:
            self.values[k] = 0.0
        self.robot.gamepad = {"connected": False, "name": None}
        self._apply()

    async def run(self) -> None:
        if evdev is None:
            log.warning("evdev недоступен — геймпад отключён")
            self.robot.gamepad = {"connected": False, "name": None, "unavailable": "нет evdev (не Linux)"}
            return
        while True:
            dev = self._find()
            if dev is None:
                await asyncio.sleep(2)
                continue
            log.info("геймпад подключён: %s (%s)", dev.name, dev.path)
            self.robot.gamepad = {"connected": True, "name": dev.name, "path": dev.path}
            self._apply()
            try:
                async for ev in dev.async_read_loop():
                    self._handle(dev, ev)
            except OSError:
                log.info("геймпад отключён")
            finally:
                # Потеря связи: стоп ходьбы, корпус выравнивается. Поза сохраняется.
                self._reset()
                dev.close()
