"""Кинематика ноги из двух звеньев в сагиттальной плоскости.

Система координат ноги: начало — ось бедра, x — вперёд, z — вниз (мм).
hip: 0° — бедро вертикально вниз, + — к x+.
knee: 0° — голень на одной линии с бедром, + — голень поворачивается к x+.
"""

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def ik(x: float, z: float, femur: float, tibia: float, knee_dir: int = -1) -> tuple[float, float]:
    """Положение лапы → (hip°, knee°). Недостижимые точки прижимаются к границе досягаемости."""
    d = _clamp(hypot(x, z), abs(femur - tibia) + 1e-3, femur + tibia - 1e-3)
    alpha = atan2(x, z)
    gamma = acos(_clamp((femur ** 2 + d ** 2 - tibia ** 2) / (2 * femur * d), -1, 1))
    bend = pi - acos(_clamp((femur ** 2 + tibia ** 2 - d ** 2) / (2 * femur * tibia), -1, 1))
    return degrees(alpha + knee_dir * gamma), degrees(-knee_dir * bend)


def body_rotation(pitch_deg: float, roll_deg: float):
    """Поворот корпус → мир: R = Ry(−pitch) · Rx(roll).
    Мир: x вперёд, y влево, z вверх. pitch + — нос вверх, roll + — правый борт ниже."""
    p, r = radians(pitch_deg), radians(roll_deg)
    cp, sp, cr, sr = cos(p), sin(p), cos(r), sin(r)

    def rot(v):
        x, y, z = v
        y, z = y * cr - z * sr, y * sr + z * cr
        return x * cp - z * sp, y, x * sp + z * cp
    return rot


def foot_in_leg(foot_w, hip_b, height: float, pitch_deg: float, roll_deg: float) -> tuple[float, float, float]:
    """Лапа в мире → (x, z) в плоскости ноги (x вперёд, z вниз от оси бедра) + боковой увод лапы.

    Пол z=0, центр корпуса в (0, 0, height), hip_b — ось бедра в системе корпуса.
    Нога двухзвенная, без отведения: лапа всегда в плоскости ноги. Поэтому точно выдерживаются
    высота (лапа на полу) и положение вперёд-назад, а вбок лапа сдвигается сама — это смещение
    и возвращается третьим числом (мм, по миру).
    """
    rot = body_rotation(pitch_deg, roll_deg)
    hx, hy, hz = rot(hip_b)
    ax, ay, az = rot((1, 0, 0))     # ось x ноги в мире
    bx, by, bz = rot((0, 0, -1))    # ось z ноги (вниз) в мире
    tx, tz = foot_w[0] - hx, foot_w[2] - (height + hz)
    det = ax * bz - bx * az
    x = (tx * bz - bx * tz) / det
    z = (ax * tz - tx * az) / det
    lateral = hy + x * ay + z * by - foot_w[1]
    return x, z, lateral


def fk(hip_deg: float, knee_deg: float, femur: float, tibia: float):
    """(hip°, knee°) → (колено, лапа) в координатах ноги."""
    a1 = radians(hip_deg)
    a2 = a1 + radians(knee_deg)
    knee = (femur * sin(a1), femur * cos(a1))
    foot = (knee[0] + tibia * sin(a2), knee[1] + tibia * cos(a2))
    return knee, foot
