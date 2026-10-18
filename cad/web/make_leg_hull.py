"""Оболочки узла колена для проверки «не заходить выше рамы» → firmware/config/leg_hull.json.

Запуск на ПК после export_web_models.py:  python cad/web/make_leg_hull.py

Посадочные места серво гуляют по ширине, поэтому боковые зазоры из CAD ненадёжны.
Вместо них правило в плоскости ноги: голень, вставка и серво колена не должны подниматься
выше плоскости низа средней перекладины рамы (минус запас). Бедро в правило не входит: оно стоит снаружи рамы
по ширине (на валу серво бедра) и упереться в неё не может.
Детали проецируются на плоскость ноги (x вперёд, y вверх), берётся их выпуклая оболочка
в системе колена (линия «колено → подошва» вниз).
Ноги зеркальны по ширине, проекция на плоскость ноги одинаковая — берём RL.
"""

import json
import struct
from pathlib import Path

WEB = Path(__file__).resolve().parent
OUT = WEB.parent.parent / "firmware" / "config" / "leg_hull.json"
# Плоскость согласована 2026-10-18: нижняя грань средней перекладины рамы (FramePlane
# в EDOG_S_Web_Assembly), Z = −3.38 в сборке; ось бедра Z = −3.90 → плоскость на 0.52 мм выше оси.
FRAME_PLANE_ABOVE_HIP_MM = -3.38 - (-3.90)


def points(name):
    data = (WEB / f"{name}.stl").read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    for i in range(n):
        o = 84 + i * 50 + 12
        for k in range(3):
            yield struct.unpack("<3f", data[o + 12 * k:o + 12 * k + 12])


def hull(pts):
    pts = sorted(set((round(x, 2), round(y, 2)) for x, y in pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return [list(p) for p in lo[:-1] + up[:-1]]


def main():
    frame_bottom = FRAME_PLANE_ABOVE_HIP_MM
    knee = [(x, y) for name in ("tibia_RL", "tibia_insert_RL", "knee_servo_RL")
            for x, y, _ in points(name)]
    manifest = json.loads((WEB / "manifest.json").read_text(encoding="utf-8"))
    data = {
        "source": "cad/web/make_leg_hull.py",
        "frame_bottom_mm": round(frame_bottom, 2),   # от оси бедра, вверх +
        "femur_mm": manifest["geometry"]["femur_mm"],
        "knee": hull(knee),
    }
    OUT.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    print(f"{OUT}: низ рамы {frame_bottom:.1f} мм, точек оболочки {len(data['knee'])}")


if __name__ == "__main__":
    main()
