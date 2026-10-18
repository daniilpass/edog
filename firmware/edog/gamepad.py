"""Bluetooth-геймпад через evdev. Подключение/отключение отслеживается автоматически."""

import asyncio
import logging

try:
    import evdev
    from evdev import ecodes
except ImportError:  # не Linux или нет python3-evdev
    evdev = None

log = logging.getLogger(__name__)

HEIGHT_STEP_MM = 5.0


class Gamepad:
    def __init__(self, robot, cfg: dict):
        self.robot = robot
        self.deadzone = cfg.get("deadzone", 0.12)
        self.axes = {}
        self.buttons = {}
        if evdev is None:
            return
        for role, name in cfg.get("axes", {}).items():
            self.axes[ecodes.ecodes[name]] = role
        for name, action in cfg.get("buttons", {}).items():
            self.buttons[ecodes.ecodes[name]] = action
        self.values = {"drive": 0.0, "turn": 0.0, "pitch": 0.0, "roll": 0.0}

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

    def _apply(self) -> None:
        v = self.values
        # Ось Y стика: вверх — отрицательные значения.
        self.robot.pad_drive = (-v["drive"], v["turn"])
        self.robot.tilt = (-v["pitch"], v["roll"])
        self.robot.gamepad["axes"] = {k: round(x, 2) for k, x in v.items()}

    def _action(self, action: str) -> None:
        r = self.robot
        if action in r.poses():
            r.set_pose(action)
        elif action == "walk_toggle":
            r.toggle_walk()
        elif action == "torque_toggle":
            r.set_enabled(not r.enabled)
        elif action == "estop":
            r.estop()
        else:
            log.warning("неизвестное действие геймпада: %s", action)

    def _handle(self, dev, ev) -> None:
        if ev.type == ecodes.EV_ABS and ev.code in self.axes:
            role = self.axes[ev.code]
            if role == "height":
                if ev.value:  # крестовина: -1 вверх, 1 вниз
                    self.robot.set_body(height=self.robot.body.height - ev.value * HEIGHT_STEP_MM)
            else:
                self.values[role] = self._norm(dev, ev.code, ev.value)
                self._apply()
        elif ev.type == ecodes.EV_KEY and ev.value == 1:
            name = ecodes.BTN.get(ev.code) or ecodes.KEY.get(ev.code) or str(ev.code)
            self.robot.gamepad["last_button"] = name if isinstance(name, str) else name[0]
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
