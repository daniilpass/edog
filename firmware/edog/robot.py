"""Состояние робота и контур управления: позы, походка, плавное движение серво."""

import asyncio
import logging
import time
from dataclasses import asdict, dataclass, replace
from math import cos, inf, pi, sin

from .collision import FrameGuard
from .config import JOINTS, Config, save_calibration
from .demo import Demo, pick_program
from .kinematics import foot_in_leg, ik
from .power import PowerCfg

log = logging.getLogger(__name__)

MODES = ("stand", "manual")
TEST_ACTIONS = ("center", "ref", "sweep", "stop")
SWEEP_SPEED_DPS = 60
# Резкие шаги движений без ограничения скорости: цель сразу в конце хода, серво жмёт
# с полным моментом с первого мгновения (при плавной цели ошибка мала — и момент тоже).
MOVE_SPEED_DPS = inf
TRIM_MAX_DEG = 10.0          # поправка наклона корпуса на огрехи сборки, °
ROLL_STEP_DEG = 1.0          # бамперы геймпада: шаг постоянной добавки к крену, °
WALK_RAMP_S = 0.5            # разгон шага из стойки и уход лап под ось бедра при ходьбе назад, с
POWER_DOWN_TIMEOUT_S = 5.0   # не дошла до монтажного положения за это время (нога у рамы) — сигнал снимается всё равно
# Походки (pattern в профиле): сдвиг фазы каждой ноги и доля цикла в воздухе (swing, в профиле можно своё).
PATTERNS = {
    # рысь: диагональные пары в противофазе — в каждый момент на полу только две лапы
    "trot": {"phase": {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}, "swing": 0.5},
    # шаг: ноги по одной, на полу всегда хотя бы три лапы; фазы — по расписанию crawl_schedule()
    "crawl": {"swing": 0.1},
}
LEGS = ("FL", "FR", "RL", "RR")


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def step_offset(p: float, stride: float, lift: float, swing: float = 0.5) -> tuple[float, float]:
    """Смещение лапы (вперёд, вверх), мм, в фазе шага p ∈ [0, 1); swing — доля цикла в воздухе."""
    if p < swing:   # перенос: лапа в воздухе идёт вперёд
        s = p / swing
        return stride * (s - 0.5), lift * sin(pi * s)
    s = (p - swing) / (1 - swing)   # опора: лапа на земле уходит назад
    return stride * (0.5 - s), 0.0


def gait_swing(gait: dict) -> float:
    return gait.get("swing", PATTERNS[gait.get("pattern", "trot")]["swing"])


def leg_step(gait: dict, leg: str, phase: float, stride: float, lift: float,
             sway: float = 1.0) -> tuple[float, float]:
    """Смещение лапы (вперёд, вверх) от нейтрали в системе корпуса, мм, с качанием корпуса (sway — его доля)."""
    dx, dz = step_offset((phase - leg_start(gait, leg)) % 1.0, stride, lift, gait_swing(gait))
    return dx - sway * body_sway(gait, phase)[0], dz


def leg_start(gait: dict, leg: str) -> float:
    """Фаза цикла, в которую нога отрывается от пола."""
    if gait.get("pattern", "trot") == "crawl":
        return crawl_schedule(gait)[0][leg]
    return -PATTERNS["trot"]["phase"][leg]


def swing_progress(gait: dict, leg: str, phase: float) -> float | None:
    """Доля переноса 0…1, если нога сейчас в воздухе, иначе None."""
    p = (phase - leg_start(gait, leg)) % 1.0
    swing = gait_swing(gait)
    return p / swing if p < swing else None


# Шаг. Корпус качается, чтобы центр масс был внутри треугольника опоры: вперёд-назад (sway_mm) —
# пока несёт заднюю ногу, вперёд, переднюю — назад; вбок креном (sway_roll_deg): у ног нет отведения,
# поэтому при крене стоящие лапы остаются на месте, а корпус уезжает вбок на ~высоту·sin(крен).
# Ноги одного борта переносятся подряд, с короткой паузой gap (в ней корпус переезжает только
# вперёд-назад): сначала левые RL, FL (корпус сдвинут вправо), потом правые RR, FR (влево).
# Между бортами — долгая пауза на всех четырёх лапах: крен перекладывается медленно,
# и последнюю долю паузы (CRAWL_SETTLE) корпус стоит, чтобы раскачка успела затихнуть.
CRAWL_SWAY = {"RL": (1, 1), "FL": (-1, 1), "RR": (1, -1), "FR": (-1, -1)}   # (вперёд, крен) на перенос ноги
CRAWL_SETTLE = 0.3


def crawl_schedule(gait: dict) -> tuple[dict[str, float], list[tuple[float, float, str, str]]]:
    """Начало переноса каждой ноги (доля цикла) и отрезки переездов корпуса (начало, длина, от ноги, к ноге)."""
    swing = gait_swing(gait)
    gap = gait.get("gap", swing)
    side = (1 - 4 * swing - 2 * gap) / 2      # переезд с борта на борт
    starts, moves, t = {}, [], 0.0
    for prev, move, leg in (("FR", side, "RL"), ("RL", gap, "FL"), ("FL", side, "RR"), ("RR", gap, "FR")):
        moves.append((t, move, prev, leg))
        starts[leg] = t + move
        t += move + swing
    return starts, moves


def body_sway(gait: dict, phase: float) -> tuple[float, float]:
    """Качание корпуса при шаге: (сдвиг вперёд, мм; крен, °, + — правый борт ниже, корпус уезжает вправо).
    Пока нога в воздухе, корпус стоит в своём крайнем положении для неё; к следующему переезжает,
    пока на полу все четыре лапы."""
    if gait.get("pattern", "trot") != "crawl":
        return 0.0, 0.0
    phase %= 1.0
    starts, moves = crawl_schedule(gait)
    a = b = None
    for start, length, prev, leg in moves:
        if start <= phase < start + length:
            t = min(1.0, (phase - start) / (length * (1 - CRAWL_SETTLE)))
            a, b, t = CRAWL_SWAY[prev], CRAWL_SWAY[leg], t * t * (3 - 2 * t)
            break
    if a is None:   # нога в воздухе: держим её положение
        leg = max((s, leg) for leg, s in starts.items() if s <= phase)[1]
        a = b = CRAWL_SWAY[leg]
        t = 0.0
    shift, roll = gait.get("sway_mm", 0.0), gait.get("sway_roll_deg", 0.0)
    return shift * (a[0] + t * (b[0] - a[0])), roll * (a[1] + t * (b[1] - a[1]))


def gait_stats(gait: dict, height: float, g: dict, knee_dir: int, speed_dps: float, samples: int = 400) -> dict:
    """Чем профиль отличается на суставах: размах бедра и колена, °, и пиковая скорость, °/с,
    при шаге полной длины на высоте профиля (без неё — height), по самой быстрой ноге.
    limited — пик выше ограничения скорости серво, то есть ограничитель срежет шаг
    и реальный размах будет меньше расчётного."""
    height = gait.get("height_mm", height)
    result = {}
    for leg in LEGS:
        stats = _leg_stats(gait, leg, height, g, knee_dir, samples)
        for key, value in stats.items():
            result[key] = max(result.get(key, value), value)
    result["speed_dps"] = speed_dps
    # корпус едет со скоростью лапы в опоре: шаг за (1 − swing) цикла
    result["speed_mm_s"] = round(gait["stride_mm"] * gait["gait_hz"] / (1 - gait_swing(gait)), 1)
    result["limited"] = max(result["hip_peak_dps"], result["knee_peak_dps"]) > speed_dps
    return result


def _leg_stats(gait: dict, leg: str, height: float, g: dict, knee_dir: int, samples: int) -> dict:
    angles = []
    for i in range(samples + 1):
        dx, dz = leg_step(gait, leg, i / samples % 1.0, gait["stride_mm"], gait["lift_mm"])
        x, z, _ = foot_in_leg((gait.get("foot_x_mm", 0.0) + dx, 0.0, dz), (0.0, 0.0, 0.0), height, 0.0, 0.0)
        angles.append(ik(x, z, g["femur_mm"], g["tibia_mm"], knee_dir, g.get("tibia_bend_deg", 0.0)))
    dt = 1.0 / (gait["gait_hz"] * samples)
    result = {}
    for j, joint in enumerate(JOINTS):
        series = [a[j] for a in angles]
        peak = max(abs(b[j] - a[j]) for a, b in zip(angles, angles[1:])) / dt
        result[f"{joint}_swing"] = round(max(series) - min(series), 1)
        result[f"{joint}_peak_dps"] = round(peak)
    return result


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
    foot_x: float = 0.0  # мм, лапы впереди (+) или позади (−) оси бедра


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
        self.tilt = (0.0, 0.0)          # смещение pitch/roll с геймпада, доля от max_tilt
        self.roll_offset = 0.0          # постоянная добавка к крену с бамперов, °; сбрасывается при выключении
        self.pad_drive = (0.0, 0.0)     # vx, turn с геймпада
        self.height_rate = 0.0          # курки: −1 вниз … 1 вверх, доля от height_speed_mm_s
        self.web_drive = (0.0, 0.0)
        self.web_drive_t = 0.0
        self.phase = 0.0
        self.walk_blend = 0.0           # 0 — стойка, 1 — высота и вынос лап профиля ходьбы
        self.walking = False            # идёт шаг (стик толкнут и собака успела встать в стойку)
        self.forward_blend = 1.0        # 0 — лапы под осью бедра, 1 — вынос лап профиля (в стойке и вперёд)
        self.gait_name = cfg.gait
        self.body = self.poses()["stand"]
        self.gamepad = {"connected": False, "name": None}
        self.power = PowerCfg.from_dict(cfg.power)
        self.demo: Demo | None = None
        self.signal: set[str] = set()   # серво, на каналы которых сейчас идёт импульс
        self.move: list[tuple[Body, float, float | None]] = []   # оставшиеся шаги движения
        self.move_name: str | None = None
        self.move_t = 0.0                  # сколько ещё держать текущий шаг, с
        self.move_speed: float | None = None
        self.powering_down: float | None = None   # с какого времени встаёт в монтажное положение перед снятием сигнала

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
            # у каждого профиля шага своя стойка: на его высоте и с его выносом лап
            "stand": Body(self.gait.get("height_mm", m["stand_height_mm"]), foot_x=self.gait.get("foot_x_mm", 0.0)),
            "sit": Body(m["stand_height_mm"] - 10, pitch=20),
            "lie": Body(m["min_height_mm"]),
            "high": Body(m["max_height_mm"] - 4),
        }

    def pose_names(self) -> list[str]:
        # mount — монтажное положение: задано углами суставов (ref_deg), а не корпусом
        return [*self.poses(), "mount"]

    def moves(self) -> dict[str, list[tuple[Body, float, float | None]]]:
        """Движения: шаги (корпус, сколько держать, с; скорость серво, °/с, или None — обычная).
        После движения собака встаёт в стойку."""
        m = self.m
        poses = self.poses()
        return {
            "jump": [   # прыжок вверх: неглубоко присесть, резкий толчок, потом стойка (ноги не поджимаются)
                (Body(m["stand_height_mm"] - 10), 0.25, None),
                # толчок до полностью прямой ноги (IK прижмёт к досягаемости), дальше max_height_mm
                (Body(self.g["femur_mm"] + self.g["tibia_mm"]), 0.12, MOVE_SPEED_DPS),
            ],
            # радость: высоко ↔ стойка 4 раза, на полной скорости серво без пауз
            "joy": [(replace(poses[p]), 0.12, MOVE_SPEED_DPS) for _ in range(4) for p in ("high", "stand")],
        }

    def start_move(self, name: str) -> None:
        steps = self.moves()[name]
        if not self.enabled or self.powering_down is not None:
            raise ValueError("движение: сначала включите серво")
        self.stop_demo()
        self._stop_tests()
        self.mode = "stand"
        self.pose_name = None
        self.move, self.move_name = list(steps), name
        self._next_move_step()
        log.info("движение: %s", name)

    def _next_move_step(self) -> None:
        if not self.move:
            self.set_pose("stand")
            return
        body, self.move_t, self.move_speed = self.move.pop(0)
        self.body = replace(body)

    def set_enabled(self, enabled: bool) -> None:
        """Выключение из стойки плавное: суставы сначала встают в монтажное положение (ref_deg),
        потом снимается сигнал — иначе без удержания серво собака падает под своим весом.
        В ручном режиме — сразу. Включение — из монтажного положения плавно в стойку."""
        self.stop_demo()
        if enabled and self.powering_down is not None:
            self.powering_down = None   # повторное включение отменяет выключение
            self.set_pose("stand")
            return
        if enabled == self.enabled:
            return
        if not enabled and self.mode != "manual":
            if self.powering_down is None:
                self._stop_tests()
                for s in self.servos.values():
                    s.set_target(s.joint_to_servo(s.cfg.ref_deg))
                self.mode = "manual"   # цели заданы напрямую, контур их не пересчитывает
                self.pose_name = None
                self.powering_down = time.monotonic()
            return
        self._switch(enabled)
        if enabled:
            self.set_pose("stand")

    def _switch(self, enabled: bool) -> None:
        self.powering_down = None
        if enabled:
            self.power.check(self.installed_count())
        self.enabled = enabled
        if enabled:
            # Реальное положение серво неизвестно. Считаем, что они в монтажном положении —
            # в нём собака остаётся после выключения; оттуда контур плавно ведёт в стойку.
            for s in self.servos.values():
                s.set_target(s.joint_to_servo(s.cfg.ref_deg))
                s.current = s.target
            self._write()
        else:
            self.driver.all_off()
            self.signal = set()
            self.roll_offset = 0.0
        log.info("серво %s", "включены" if enabled else "отключены")

    def estop(self) -> None:
        self.web_drive = self.pad_drive = (0.0, 0.0)
        self.stop_demo()
        if self.enabled:
            self._switch(False)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"режим: {MODES}")
        self.stop_demo()
        self.mode = mode
        self._stop_tests()

    def _stop_tests(self) -> None:
        for s in self.servos.values():
            s.queue.clear()

    def set_pose(self, name: str) -> None:
        if name == "mount":
            # как перед выключением: суставы в ref_deg; ручной режим — контур углы не пересчитывает
            self.stop_demo()
            self._stop_tests()
            for s in self.servos.values():
                s.set_target(s.joint_to_servo(s.cfg.ref_deg))
            self.mode = "manual"
            self.pose_name = name
            return
        poses = self.poses()
        if name not in poses:
            raise KeyError(name)
        self.stop_demo()
        self.body = poses[name]
        self.pose_name = name
        self._stop_tests()
        if self.mode == "manual":
            self.mode = "stand"

    def set_body(self, height=None, pitch=None, roll=None) -> None:
        m = self.m
        if height is not None:
            self.body.height = clamp(height, m["min_height_mm"], m["max_height_mm"])
        if pitch is not None:
            self.body.pitch = clamp(pitch, -m["max_tilt_deg"], m["max_tilt_deg"])
        if roll is not None:
            max_roll = m.get("max_roll_deg", m["max_tilt_deg"])
            self.body.roll = clamp(roll, -max_roll, max_roll)
        self.stop_demo()
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
        self.stop_demo()
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
        return str(save_calibration(self.cfg, self.servos.values(), asdict(self.power), self.gait_name, self.cfg.trim))

    def update_trim(self, pitch_deg: float | None = None, roll_deg: float | None = None) -> None:
        """Поправка наклона корпуса из портала, °. В файл — save_calibration()."""
        for key, value in (("pitch_deg", pitch_deg), ("roll_deg", roll_deg)):
            if value is not None:
                if abs(value) > TRIM_MAX_DEG:
                    raise ValueError(f"поправка наклона: не больше ±{TRIM_MAX_DEG}°")
                self.cfg.trim[key] = float(value)

    @property
    def gait(self) -> dict:
        return self.cfg.gaits[self.gait_name]

    def speed_limit(self, gait: dict | None = None) -> float:
        return (gait or self.gait).get("max_speed_dps", self.m["max_speed_dps"])

    def set_gait(self, name: str) -> None:
        """Профиль ходьбы: им же шагает демо. Можно менять на ходу. В файл — save_calibration()."""
        if name not in self.cfg.gaits:
            raise ValueError(f"профили ходьбы: {tuple(self.cfg.gaits)}")
        self.gait_name = name
        log.info("профиль ходьбы: %s", name)
        if self.mode == "stand" and self.pose_name == "stand" and not self.move_name and not self.demo:
            self.body = self.poses()["stand"]   # стоит — встаёт в стойку нового профиля

    def nudge_roll(self, sign: int) -> None:
        """Бамперы: крен на ROLL_STEP_DEG в сторону sign (+ — правый борт ниже), держится до выключения."""
        self.roll_offset = clamp(self.roll_offset + sign * ROLL_STEP_DEG, -TRIM_MAX_DEG, TRIM_MAX_DEG)
        log.info("добавка к крену: %+.0f°", self.roll_offset)

    def next_gait(self) -> None:
        """Следующий профиль ходьбы по кругу, медленные копии (*_slow) пропускаются."""
        names = [n for n in self.cfg.gaits if not n.endswith("_slow")]
        i = names.index(self.gait_name) if self.gait_name in names else -1
        self.set_gait(names[(i + 1) % len(names)])

    def installed_count(self) -> int:
        return sum(s.cfg.installed for s in self.servos.values())

    def update_power(self, limit_a: float | None = None, servo_a: float | None = None) -> None:
        """Лимит тока из портала: проверяется по уже подключённым серво. В файл — save_calibration()."""
        new = replace(self.power, **{k: v for k, v in (("limit_a", limit_a), ("servo_a", servo_a)) if v is not None})
        new.validate()
        new.check(self.installed_count())
        self.power = new

    def start_demo(self, cycles: int = 4, laps: int = 3) -> None:
        """Демо: ноги по кругу шагают в воздухе по cycles полных циклов шага выбранного
        профиля, laps кругов. Сколько ног
        сразу — по лимиту тока; сигнал только на шагающие ноги (отметки «подключено»
        на время демо не действуют). В конце сигнал снимается."""
        program = pick_program(self.power.max_servos())
        ref = {joint: next(s.cfg.ref_deg for s in self.servos.values() if s.cfg.joint == joint) for joint in JOINTS}
        demo = Demo(program, cycles, 1.0 / self.gait["gait_hz"], laps, ref, self._demo_step)
        self.stop_demo()
        self._stop_tests()
        if self.enabled:
            self.driver.all_off()
            self.signal = set()
        for s in self.servos.values():
            s.set_target(s.joint_to_servo(ref[s.cfg.joint]))
            s.current = s.target
        self.demo = demo
        self.mode = "demo"
        self.enabled = True
        log.info("демо: %s, шаг %s, по %d циклов на ногу, кругов %d", program, self.gait_name, cycles, laps)

    def stop_demo(self) -> None:
        # Любая команда прерывает и движение: все они проходят через stop_demo().
        self.move, self.move_name, self.move_speed = [], None, None
        if not self.demo:
            return
        self.demo = None
        self.enabled = False
        self.driver.all_off()
        self.signal = set()
        self.mode = "manual"
        log.info("демо остановлено, сигнал снят")

    def _demo_step(self, name: str, t: float) -> tuple[float, float]:
        """Шаг вперёд полной длины в воздухе: корпус на высоте профиля (без неё — стойки), без наклонов."""
        m, gait, leg = self.m, self.gait, self.legs[name]
        dx, dz = leg_step(gait, name, t * gait["gait_hz"] % 1.0, gait["stride_mm"], gait["lift_mm"])
        y = (1 if leg.side == "left" else -1) * self.g["body_width_mm"] / 2
        x, z, _ = foot_in_leg((leg.x_mm + gait.get("foot_x_mm", 0.0) + dx, y, dz), (leg.x_mm, y, 0.0),
                              gait.get("height_mm", m["stand_height_mm"]), 0.0, 0.0)
        return ik(x, z, self.g["femur_mm"], self.g["tibia_mm"], leg.knee_dir, self.g.get("tibia_bend_deg", 0.0))

    def drive(self) -> tuple[tuple[float, float], str]:
        if time.monotonic() - self.web_drive_t < self.m["drive_timeout_s"]:
            return self.web_drive, "web"
        return self.pad_drive, "gamepad"

    # ---------- контур ----------

    def feet(self, dt: float) -> dict[str, tuple[float, float]]:
        m, gait = self.m, self.gait
        max_tilt = m["max_tilt_deg"]
        max_roll = m.get("max_roll_deg", max_tilt)
        pitch = clamp(self.body.pitch + self.tilt[0] * max_tilt, -max_tilt, max_tilt)
        roll = clamp(self.body.roll + self.tilt[1] * max_roll, -max_roll, max_roll)
        half_width = self.g["body_width_mm"] / 2
        (vx, turn), _ = self.drive()
        # Ходьба всегда начинается с начала: если стик снова толкнули, пока собака ещё не встала,
        # она сначала доходит до стойки профиля (лапы опускаются, серво доезжают), потом шаг с фазы 0 —
        # первыми идут FL и RR.
        if not (abs(vx) > 0.05 or abs(turn) > 0.05):
            self.walking = False
        elif not self.walking and self.walk_blend == 0.0 and self._settled():
            self.walking, self.phase = True, 0.0
        walking = self.walking
        if walking:
            self.phase = (self.phase + gait["gait_hz"] * dt) % 1.0
        # Профиль может ходить на своей высоте и с лапами, вынесенными вперёд/назад от оси бедра
        # (height_mm, foot_x_mm): так подошва встаёт на стоппер, а не на край голени.
        # Старт плавный, шаг растёт вместе с ним — пока корпус не поднялся, лапы почти не шагают.
        # Остановка — сразу в стойку профиля (дальше только ограничение скорости серво).
        self.walk_blend = clamp(self.walk_blend + dt / WALK_RAMP_S, 0.0, 1.0) if walking else 0.0
        w = self.walk_blend
        # Вынос лап — в стойке и при ходьбе вперёд: у рыси с лапами позади оси бедра ход назад
        # на собаке почти не работает (шагает на месте). Назад и на развороте — лапы под осью бедра.
        forward = not walking or vx > 0.05
        self.forward_blend = clamp(self.forward_blend + (dt if forward else -dt) / WALK_RAMP_S, 0.0, 1.0) if walking else 1.0
        height = self.body.height + w * (gait.get("height_mm", self.body.height) - self.body.height)
        walk_x = self.forward_blend * gait.get("foot_x_mm", 0.0)
        foot_x = self.body.foot_x + w * (walk_x - self.body.foot_x)
        roll = clamp(roll + w * body_sway(gait, self.phase)[1], -max_roll, max_roll)
        # огрехи сборки: корпус, стоящий «ровно» по командам, на деле наклонён — поправка сверх пределов
        pitch += self.cfg.trim["pitch_deg"]
        roll += self.cfg.trim["roll_deg"] + self.roll_offset

        # Лапы задаются точками на полу (мир), потом переводятся в систему наклонённого корпуса.
        # Ноги двухзвенные, без отведения: боковое смещение лапы при крене не отрабатывается,
        # поэтому крен — только в пределах нескольких градусов.
        result = {}
        for leg in self.legs.values():
            side = 1 if leg.side == "left" else -1
            stride = gait["stride_mm"] * clamp(vx + side * turn, -1, 1) if walking else 0.0
            dx, dz = leg_step(gait, leg.name, self.phase, w * stride, w * gait["lift_mm"], w)
            fx, fz = leg.x_mm + foot_x + dx, dz
            hip = (leg.x_mm, side * half_width, 0.0)
            x, z, _ = foot_in_leg((fx, side * half_width, fz), hip, height, pitch, roll)
            result[leg.name] = (x, z)
        return result

    def _settled(self) -> bool:
        return all(abs(s.target - s.current) < 1.0 for s in self.servos.values())

    def _update_targets(self, dt: float) -> None:
        if self.demo:
            for name, angles in self.demo.tick(dt).items():
                for joint, angle in zip(JOINTS, angles):
                    s = self.servos[f"{name}_{joint}"]
                    s.set_target(s.joint_to_servo(angle))
            return
        (vx, turn), _ = self.drive()
        if self.move_name:
            self.move_t -= dt
            if self.move_t <= 0:
                self._next_move_step()
        if self.powering_down is not None or self.move_name:
            vx = turn = 0.0   # встаёт в монтажное положение перед выключением: без ходьбы
        if abs(vx) > 0.05 or abs(turn) > 0.05:
            if self.mode == "manual" or self.pose_name != "stand":
                self.set_pose("stand")   # ходит только из стойки: стик или стрелки переводят в неё
        elif self.mode == "manual":
            return
        if self.height_rate and self.powering_down is None and not self.move_name:
            # Поза при этом не меняется: из стойки с другой высотой ходить можно.
            m = self.m
            h = self.body.height + self.height_rate * m.get("height_speed_mm_s", 30.0) * dt
            self.body.height = clamp(h, m["min_height_mm"], m["max_height_mm"])
        for name, (x, z) in self.feet(dt).items():
            leg = self.legs[name]
            hip, knee = ik(x, z, self.g["femur_mm"], self.g["tibia_mm"], leg.knee_dir,
                           self.g.get("tibia_bend_deg", 0.0))
            for joint, angle in (("hip", hip), ("knee", knee)):
                s = self.servos[f"{name}_{joint}"]
                s.set_target(s.joint_to_servo(angle))

    def _signal_ids(self) -> set[str]:
        # Сигнал только на подключённые серво: при сборке ноги появляются по одной.
        # В демо — только на танцующие ноги: их число подобрано по лимиту тока.
        if self.demo:
            return {f"{leg}_{joint}" for leg in self.demo.legs for joint in JOINTS}
        return {sid for sid, s in self.servos.items() if s.cfg.installed}

    def _write(self) -> None:
        on = self._signal_ids()
        for sid in self.signal - on:
            self.driver.off(self.servos[sid].cfg.channel)
        for sid in on:
            s = self.servos[sid]
            self.driver.set_pulse(s.cfg.channel, s.pulse_us(s.current))
        self.signal = on

    def step(self, dt: float) -> None:
        self._update_targets(dt)
        before = {name: self._leg_joints(name, "current") for name in self.legs}
        max_speed = self.move_speed if self.move_name and self.move_speed else self.speed_limit()
        # Суставы ноги доходят до цели одновременно: скорость каждого — по его доле хода.
        # Иначе бедро успевает раньше колена, лапа идёт в обход, и колено задевает раму (стойка → сидеть).
        lead = {name: max(abs(self.servos[f"{name}_{j}"].target - self.servos[f"{name}_{j}"].current)
                          for j in JOINTS) for name in self.legs}
        for s in self.servos.values():
            if not self.enabled:
                s.current = s.target
                s.queue.clear()
                continue
            delta = s.target - s.current
            if s.queue:
                speed = SWEEP_SPEED_DPS
            elif delta:
                speed = max_speed * abs(delta) / lead[s.cfg.leg]
            else:
                continue
            s.current += clamp(delta, -speed * dt, speed * dt)
            if s.queue and abs(s.target - s.current) < 1e-6:
                s.set_target(s.queue.pop(0))
        self._guard_legs(before)
        if self.demo and self.demo.finished:
            self.stop_demo()
        if self.powering_down is not None:
            settled = all(abs(s.target - s.current) < 0.5 for s in self.servos.values())
            if settled or time.monotonic() - self.powering_down > POWER_DOWN_TIMEOUT_S:
                self._switch(False)
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
            "poses": self.pose_names(),
            "moves": list(self.moves()),
            "gaits": [{"name": name, **gait, **gait_stats(gait, self.m["stand_height_mm"], self.g,
                                                         self.cfg.legs[0].knee_dir, self.speed_limit(gait))}
                      for name, gait in self.cfg.gaits.items()],
            "limits": {"height": [self.m["min_height_mm"], self.m["max_height_mm"]],
                       "pitch": self.m["max_tilt_deg"],
                       "roll": self.m.get("max_roll_deg", self.m["max_tilt_deg"])},
            "gamepad": {"axes": self.cfg.gamepad.get("axes", {}),
                        "buttons": self.cfg.gamepad.get("buttons", {}),
                        "dpad": self.cfg.gamepad.get("dpad", {})},
            # плоскость рамы, мм от оси бедра (вверх +): plane — низ перекладины, limit — с запасом
            "frame_guard": None if not self.guard else {
                "plane_mm": round(self.guard.frame_bottom, 2), "limit_mm": round(self.guard.limit, 2)},
        }

    def state(self) -> dict:
        (vx, turn), source = self.drive()
        return {
            "t": time.time(),
            "enabled": self.enabled,
            "powering_down": self.powering_down is not None,
            "mode": self.mode,
            "pose": self.pose_name,
            "move": self.move_name,
            "body": asdict(self.body),
            "tilt": self.tilt,
            "drive": {"vx": vx, "turn": turn, "source": source},
            "gamepad": self.gamepad,
            "driver": {"kind": self.driver.kind, "error": self.driver_error},
            "servos": [{**s.state(), "frame_blocked": self.blocked[s.cfg.leg], "signal": sid in self.signal}
                       for sid, s in self.servos.items()],
            "demo": self.demo.state() if self.demo else None,
            "gait": self.gait_name,
            "frame_guard": self.guard is not None,
            "power": {**asdict(self.power), "max_servos": self.power.max_servos(),
                      "installed": self.installed_count()},
            "trim": dict(self.cfg.trim),
            "roll_offset": self.roll_offset,
        }
