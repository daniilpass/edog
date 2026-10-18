"""Защита от упора узла колена в раму.

Боковые зазоры из CAD ненадёжны: посадочные места серво гуляют по ширине. Поэтому правило
задано в плоскости ноги: голень, вставка и серво колена не поднимаются выше низа рамы минус
запас. Бедро стоит снаружи рамы по ширине и в правило не входит.
Оболочка — config/leg_hull.json (cad/web/make_leg_hull.py).

Координаты ноги: x вперёд, y вверх, начало на оси бедра. Углы — как в kinematics:
hip + — бедро нижним концом вперёд; knee — по корпусу голени (bend учитывается здесь).
"""

import json
from math import cos, radians, sin
from pathlib import Path


class FrameGuard:
    def __init__(self, hull_path: Path, bend_deg: float = 0.0, margin_mm: float = 3.0):
        data = json.loads(Path(hull_path).read_text(encoding="utf-8"))
        self.knee_pts = [tuple(p) for p in data["knee"]]
        self.femur_mm = data["femur_mm"]
        self.frame_bottom = data["frame_bottom_mm"]
        self.limit = self.frame_bottom - margin_mm
        self.bend = bend_deg

    def top(self, hip_deg: float, knee_deg: float) -> float:
        """Самая высокая точка узла колена, мм от оси бедра (вверх +)."""
        h = radians(hip_deg)
        k = radians(knee_deg - self.bend)
        sh, ch, sk, ck = sin(h), cos(h), sin(k), cos(k)
        top = float("-inf")
        for x, y in self.knee_pts:
            # в системе колена → повернуть на колено, сдвинуть к колену, повернуть на бедро
            xk, yk = x * ck - y * sk, x * sk + y * ck - self.femur_mm
            top = max(top, xk * sh + yk * ch)
        return top

    def ok(self, hip_deg: float, knee_deg: float) -> bool:
        return self.top(hip_deg, knee_deg) <= self.limit

    def allows_move(self, old: tuple[float, float], new: tuple[float, float]) -> bool:
        """Шаг разрешён, если нога остаётся ниже плоскости или хотя бы опускается к ней."""
        t_new = self.top(*new)
        return t_new <= self.limit or t_new <= self.top(*old) + 1e-6
