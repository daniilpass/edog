"""Состояние робота и контур управления: позы, походка, плавное движение серво."""

import asyncio
import logging
import time
from dataclasses import asdict, dataclass, replace
from math import pi, sin

from .collision import FrameGuard
from .config import Config, save_calibration, save_power
from .kinematics import foot_in_leg, ik
from .power import PowerCfg

log = logging.getLogger(__name__)

MODES = ("stand", "walk", "manual")
TEST_ACTIONS = ("center", "ref", "sweep", "stop")
SWEEP_SPEED_DPS = 60
# Рысь: диагональные пары ног в противофазе.
TROT_PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Servo:
    def __init__(self, cfg):
        self.cfg = cfg
        self.target = cfg.zero   # угол серво, °
        self.current = cfg.zero
        self.clamped = False     # последняя цель упёрлась в предел
        self.queue: list[float] = []  # тестовый проход по ходу: следующие цели

    def joint_to_servo(self, joint_deg: float) -> float:
        return self.cfg.zero + self.cfg.dir * joint_deg

    def servo_to_joint(self, servo_deg: float) -> float:
        return (servo_deg - self.cfg.zero) * self.cfg.dir

    def set_target(self, servo_deg: float) -> None:
        c = self.cfg
        self.target = clamp(servo_deg, c.min_deg, c.max_deg)
        self.clamped = self.target != servo_deg

    def pulse_us(self, servo_deg: float) -> float:
        c = self.cfg
        return c.pulse_min_us + servo_deg / c.range_deg * (c.pulse_max_us - c.pulse_min_us)

    def state(self) -> dict:
        c = self.cfg
        return {
            "id": c.id, "leg": c.leg, "joint": c.joint, "channel": c.channel,
            "current": round(self.current, 2), "target": round(self.target, 2),
            "joint_current": round(self.servo_to_joint(self.current), 2),
            "joint_target": round(self.servo_to_joint(self.target), 2),
            "pulse_us": round(self.pulse_us(self.current)),
            "installed": c.installed, "sweeping": bool(self.queue),
            "ref_deg": c.ref_deg, "zero": c.zero, "dir": c.dir, "min_deg": c.min_deg, "max_deg": c.max_deg,
            "clamped": self.clamped,
            "frame_blocked": False,   # заполняет Robot.state()
        }


@dataclass
class Body:
    height: float
    pitch: float = 0.0  # °, + — нос вверх
    roll: float = 0.0   # °, + — правый борт ниже


class Robot:
    def __init__(self, cfg: Config, driver, driver_error: str | None = None):
        self.cfg = cfg
        self.driver = driver
        self.driver_error = driver_error
        self.m = cfg.motion
        self.g = cfg.geometry
        self.legs = {leg.name: leg for leg in cfg.legs}
        self.servos = {s.id: Servo(s) for s in cfg.servos}

        self.enabled = False
        self.mode = "stand"
        self.pose_name = "stand"
        self.body = Body(height=self.m["stand_height_mm"])
        self.tilt = (0.0, 0.0)          # смещение pitch/roll с геймпада, доля от max_tilt
        self.pad_drive = (0.0, 0.0)     # vx, turn с геймпада
        self.web_drive = (0.0, 0.0)
        self.web_drive_t = 0.0
        self.phase = 0.0
        self.gamepad = {"connected": False, "name": None}
        self.power = PowerCfg.from_dict(cfg.power)

        # Узел колена не поднимается выше плоскости рамы (см. collision.py).
        hull = cfg.path.with_name("leg_hull.json")
        self.guard = None
        if hull.exists():
            self.guard = FrameGuard(hull, self.g.get("tibia_bend_deg", 0.0), self.g.get("frame_margin_mm", 3.0))
        else:
            log.warning("нет %s — защита от упора колена в раму отключена", hull)
        self.blocked = {name: False for name in self.legs}   # нога остановлена у рамы

        self._update_targets(0.0)
        for s in self.servos.values():
            s.current = s.target

    # ---------- команды ----------

    def poses(self) -> dict[str, Body]:
        m = self.m
        return {
            "stand": Body(m["stand_height_mm"]),
            "sit": Body(m["stand_height_mm"] - 10, pitch=20),
            "lie": Body(m["min_height_mm"]),
            "high": Body(m["max_height_mm"] - 4),
        }

    def set_enabled(self, enabled: bool) -> None:
        if enabled == self.enabled:
            return
        if enabled:
            self.power.check(self.installed_count())
        self.enabled = enabled
        if enabled:
            # Реальное положение серво неизвестно: при включении они встают в цель рывком.
            for s in self.servos.values():
                s.current = s.target
            self._write()
        else:
            self.driver.all_off()
        log.info("серво %s", "включены" if enabled else "отключены")

    def estop(self) -> None:
        self.web_drive = self.pad_drive = (0.0, 0.0)
        self.set_enabled(False)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"режим: {MODES}")
        self.mode = mode
        self._stop_tests()

    def _stop_tests(self) -> None:
        for s in self.servos.values():
            s.queue.clear()

    def set_pose(self, name: str) -> None:
        poses = self.poses()
        if name not in poses:
            raise KeyError(name)
        self.body = poses[name]
        self.pose_name = name
        self._stop_tests()
        if self.mode == "manual":
            self.mode = "stand"

    def toggle_walk(self) -> None:
        self.mode = "stand" if self.mode == "walk" else "walk"

    def set_body(self, height=None, pitch=None, roll=None) -> None:
        m = self.m
        if height is not None:
            self.body.height = clamp(height, m["min_height_mm"], m["max_height_mm"])
        if pitch is not None:
            self.body.pitch = clamp(pitch, -m["max_tilt_deg"], m["max_tilt_deg"])
        if roll is not None:
            max_roll = m.get("max_roll_deg", m["max_tilt_deg"])
            self.body.roll = clamp(roll, -max_roll, max_roll)
        self.pose_name = None
        if self.mode == "manual":
            self.mode = "stand"

    def set_web_drive(self, vx: float, turn: float) -> None:
        self.web_drive = (clamp(vx, -1, 1), clamp(turn, -1, 1))
        self.web_drive_t = time.monotonic()

    def _leg_joints(self, leg: str, attr: str = "target") -> tuple[float, float]:
        hip, knee = self.servos[f"{leg}_hip"], self.servos[f"{leg}_knee"]
        return hip.servo_to_joint(getattr(hip, attr)), knee.servo_to_joint(getattr(knee, attr))

    def set_servo(self, sid: str, angle: float | None = None, joint: float | None = None) -> None:
        s = self.servos[sid]
        servo_deg = angle if angle is not None else s.joint_to_servo(joint)
        if self.guard:
            c = s.cfg
            old = self._leg_joints(c.leg)
            j = s.servo_to_joint(clamp(servo_deg, c.min_deg, c.max_deg))
            new = (j, old[1]) if c.joint == "hip" else (old[0], j)
            if not self.guard.allows_move(old, new):
                raise ValueError(f"{c.id}: колено упрётся в раму (бедро {new[0]:.0f}°, колено {new[1]:.0f}°)")
        self.mode = "manual"
        s.queue.clear()
        s.set_target(servo_deg)

    def nudge(self, sid: str, delta: float) -> None:
        s = self.servos[sid]
        self.set_servo(sid, angle=s.target + delta)

    def test_servo(self, sid: str, action: str) -> None:
        """Тест при сборке: center — 90° (ставить крыло), ref — монтажное положение сустава,
        sweep — медленно min → max → обратно, stop — остановить проход."""
        s = self.servos[sid]
        if action not in TEST_ACTIONS:
            raise ValueError(f"действие: {TEST_ACTIONS}")
        if action == "stop":
            self.set_servo(sid, angle=s.current)
        elif action == "center":
            self.set_servo(sid, angle=s.cfg.range_deg / 2)
        elif action == "ref":
            self.set_servo(sid, joint=s.cfg.ref_deg)
        elif action == "sweep":
            if not (self.enabled and s.cfg.installed):
                raise ValueError("проход по ходу: включите серво и отметьте его подключённым")
            back = s.target
            self.set_servo(sid, angle=s.cfg.min_deg)
            s.queue = [s.cfg.max_deg, back]

    def update_calibration(self, sid: str, **fields) -> None:
        c = self.servos[sid].cfg
        for key, value in fields.items():
            if value is None:
                continue
            if key == "dir" and value not in (-1, 1):
                raise ValueError("dir: -1 или 1")
            if key == "installed" and value and not c.installed:
                self.power.check(self.installed_count() + 1)
            setattr(c, key, value)
            if key == "installed" and not value:
                self.servos[sid].queue.clear()
                if self.enabled:
                    self.driver.off(c.channel)
        if not 0 <= c.min_deg < c.max_deg <= c.range_deg:
            raise ValueError("пределы: 0 ≤ min < max ≤ range")
        s = self.servos[sid]
        s.set_target(s.target)

    def calibrate(self, sid: str, joint_deg: float) -> None:
        """Текущее положение серво соответствует углу сустава joint_deg."""
        s = self.servos[sid]
        s.cfg.zero = round(s.current - s.cfg.dir * joint_deg, 1)

    def save_calibration(self) -> str:
        return str(save_calibration(self.cfg, self.servos.values()))

    def installed_count(self) -> int:
        return sum(s.cfg.installed for s in self.servos.values())

    def update_power(self, limit_a: float | None = None, servo_a: float | None = None) -> None:
        """Лимит тока из портала: проверяется по уже подключённым серво и сразу сохраняется."""
        new = replace(self.power, **{k: v for k, v in (("limit_a", limit_a), ("servo_a", servo_a)) if v is not None})
        new.validate()
        new.check(self.installed_count())
        self.power = new
        save_power(self.cfg, asdict(new))

    def drive(self) -> tuple[tuple[float, float], str]:
        if time.monotonic() - self.web_drive_t < self.m["drive_timeout_s"]:
            return self.web_drive, "web"
        return self.pad_drive, "gamepad"

    # ---------- контур ----------

    def feet(self, dt: float) -> dict[str, tuple[float, float]]:
        m = self.m
        max_tilt = m["max_tilt_deg"]
        max_roll = m.get("max_roll_deg", max_tilt)
        pitch = clamp(self.body.pitch + self.tilt[0] * max_tilt, -max_tilt, max_tilt)
        roll = clamp(self.body.roll + self.tilt[1] * max_roll, -max_roll, max_roll)
        half_width = self.g["body_width_mm"] / 2
        (vx, turn), _ = self.drive()
        walking = self.mode == "walk" and (abs(vx) > 0.05 or abs(turn) > 0.05)
        if walking:
            self.phase = (self.phase + m["gait_hz"] * dt) % 1.0

        # Лапы задаются точками на полу (мир), потом переводятся в систему наклонённого корпуса.
        # Ноги двухзвенные, без отведения: боковое смещение лапы при крене не отрабатывается,
        # поэтому крен — только в пределах нескольких градусов.
        result = {}
        for leg in self.legs.values():
            side = 1 if leg.side == "left" else -1
            fx, fz = leg.x_mm, 0.0
            if walking:
                stride = m["stride_mm"] * clamp(vx + side * turn, -1, 1)
                p = (self.phase + TROT_PHASE.get(leg.name, 0.0)) % 1.0
                if p < 0.5:   # перенос: лапа в воздухе идёт вперёд
                    s = p / 0.5
                    fx += stride * (s - 0.5)
                    fz += m["lift_mm"] * sin(pi * s)
                else:         # опора: лапа на земле уходит назад
                    s = (p - 0.5) / 0.5
                    fx += stride * (0.5 - s)
            hip = (leg.x_mm, side * half_width, 0.0)
            x, z, _ = foot_in_leg((fx, side * half_width, fz), hip, self.body.height, pitch, roll)
            result[leg.name] = (x, z)
        return result

    def _update_targets(self, dt: float) -> None:
        if self.mode == "manual":
            return
        for name, (x, z) in self.feet(dt).items():
            leg = self.legs[name]
            hip, knee = ik(x, z, self.g["femur_mm"], self.g["tibia_mm"], leg.knee_dir,
                           self.g.get("tibia_bend_deg", 0.0))
            for joint, angle in (("hip", hip), ("knee", knee)):
                s = self.servos[f"{name}_{joint}"]
                s.set_target(s.joint_to_servo(angle))

    def _write(self) -> None:
        # Сигнал только на подключённые серво: при сборке ноги появляются по одной.
        for s in self.servos.values():
            if s.cfg.installed:
                self.driver.set_pulse(s.cfg.channel, s.pulse_us(s.current))

    def step(self, dt: float) -> None:
        self._update_targets(dt)
        before = {name: self._leg_joints(name, "current") for name in self.legs}
        for s in self.servos.values():
            if not self.enabled:
                s.current = s.target
                s.queue.clear()
                continue
            speed = SWEEP_SPEED_DPS if s.queue else self.m["max_speed_dps"]
            s.current += clamp(s.target - s.current, -speed * dt, speed * dt)
            if s.queue and abs(s.target - s.current) < 1e-6:
                s.set_target(s.queue.pop(0))
        self._guard_legs(before)
        if self.enabled:
            self._write()

    def _guard_legs(self, before: dict[str, tuple[float, float]]) -> None:
        """Шаг, поднимающий узел колена выше рамы, отменяется: нога стоит, где стояла.
        Опускаться (уходить от рамы) можно всегда."""
        if not self.guard:
            return
        for name in self.legs:
            new = self._leg_joints(name, "current")
            blocked = not self.guard.allows_move(before[name], new)
            if blocked:
                for joint, angle in zip(("hip", "knee"), before[name]):
                    s = self.servos[f"{name}_{joint}"]
                    s.current = s.target = s.joint_to_servo(angle)
                    s.queue.clear()
                if not self.blocked[name]:
                    log.warning("нога %s остановлена: колено упрётся в раму", name)
            self.blocked[name] = blocked

    async def run(self) -> None:
        period = 1.0 / self.m["loop_hz"]
        last = time.monotonic()
        while True:
            await asyncio.sleep(period)
            now = time.monotonic()
            try:
                self.step(min(now - last, 0.1))
            except OSError as e:  # ошибка I²C не должна ронять сервис
                self.driver_error = f"I²C: {e}"
                log.error("ошибка записи в PCA9685: %s", e)
            last = now

    def shutdown(self) -> None:
        try:
            self.driver.all_off()
        finally:
            self.driver.close()

    # ---------- описание ----------

    def describe(self) -> dict:
        return {
            "geometry": self.g,
            "legs": [asdict(leg) for leg in self.legs.values()],
            "servos": [{"id": s.cfg.id, "leg": s.cfg.leg, "joint": s.cfg.joint,
                        "channel": s.cfg.channel} for s in self.servos.values()],
            "poses": list(self.poses()),
            "limits": {"height": [self.m["min_height_mm"], self.m["max_height_mm"]],
                       "pitch": self.m["max_tilt_deg"],
                       "roll": self.m.get("max_roll_deg", self.m["max_tilt_deg"])},
            "gamepad": {"axes": self.cfg.gamepad.get("axes", {}),
                        "buttons": self.cfg.gamepad.get("buttons", {})},
            # плоскость рамы, мм от оси бедра (вверх +): plane — низ перекладины, limit — с запасом
            "frame_guard": None if not self.guard else {
                "plane_mm": round(self.guard.frame_bottom, 2), "limit_mm": round(self.guard.limit, 2)},
        }

    def state(self) -> dict:
        (vx, turn), source = self.drive()
        return {
            "t": time.time(),
            "enabled": self.enabled,
            "mode": self.mode,
            "pose": self.pose_name,
            "body": asdict(self.body),
            "tilt": self.tilt,
            "drive": {"vx": vx, "turn": turn, "source": source},
            "gamepad": self.gamepad,
            "driver": {"kind": self.driver.kind, "error": self.driver_error},
            "servos": [{**s.state(), "frame_blocked": self.blocked[s.cfg.leg]} for s in self.servos.values()],
            "frame_guard": self.guard is not None,
            "power": {**asdict(self.power), "max_servos": self.power.max_servos(),
                      "installed": self.installed_count()},
        }
