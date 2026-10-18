# Экспорт 3D-моделей для веб-портала: cad/web/*.stl + manifest.json.
# Запуск в FreeCAD (консоль Python или MCP):
#   exec(open(r"C:/git/edog/cad/web/export_web_models.py", encoding="utf-8").read())
#
# Источники: корпус и серво бёдер — EDOG_T_Front_Mark.FCStd, бедро и голень — cad/custom/*.stl.
# Платы, провода, гайки и крышка в портал не выгружаются (крышка — по желанию, см. BODY_PARTS).
#
# Система координат портала (мм): x — вперёд, y — вверх, z — вправо.
# Начало «body» — центр корпуса на высоте осей бёдер.
# Детали ног выгружаются в нулевой позе в системах своих суставов:
#   hip  — начало на оси бедра (у торца вала серво), бедро висит вниз (−y);
#   knee — начало на оси колена, линия «ось колена → подошва» смотрит вниз (−y).

import json
import math
import os

import FreeCAD
import Mesh
import MeshPart
import Part
from FreeCAD import Matrix, Vector as V

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if "__file__" in dir() else r"C:\git\edog\cad"
OUT = os.path.join(ROOT, "web")
SRC = os.path.join(ROOT, "freecad", "EDOG_T_Front_Mark.FCStd")
FIX = os.path.join(ROOT, "custom")

# --- геометрия, снятая с моделей (см. docs) ---
HIP_X = {"F": 115.38, "R": 16.40}           # оси бёдер по X в сборке
SHAFT_TIP_Y = {"L": 101.42, "R": -31.50}    # торцы валов бёдер по Y
HIP_Z = 4.07
CENTER = V((HIP_X["F"] + HIP_X["R"]) / 2, 34.96, HIP_Z)
FEMUR = 50.0                                 # между отверстиями Femur_hub7
FEMUR_THICK = 9.0
HORN_GAP = 3.0                               # торец вала заходит в бедро на 3 мм
TIB_SHAFT = V(0, 11.5 - 5.2, 5.98)           # ось вала колена в файле голени
TIB_FOOT = V(0, 46.0, -6.7)                  # середина подошвы
TIB_TIP_X = -14.0                            # торец вала колена (уши на грани X=0, вал в −X)
TIB_FOOT_X = 5.1                             # середина подошвы по толщине голени
MG90S_TIP = V(16.40, -31.50, 4.06)           # торец вала MG90S_LeftFront в сборке
MG90S_EARS = V(13.05, 0, 0.10)               # центр ушей MG90S_LeftFront (X, Z)

BODY_PARTS = {
    "body.stl": (["CylinderChassis"], "#e8e2d6"),
    "frames.stl": (["ExternalMotorBlock0", "ExternalMotorBlock1",
                    "ServoInsert_0_0", "ServoInsert_0_1", "ServoInsert_1_0", "ServoInsert_1_1"], "#cfcfca"),
    "lid.stl": (["OriginalSizePlatform"], "#e8e2d6"),
}
HIP_SERVOS = {"FL": "MG90S_RightRear", "FR": "MG90S_RightFront",
              "RL": "MG90S_LeftRear", "RR": "MG90S_LeftFront"}
SERVO_COLOR = "#2b4fa8"
LEG_COLOR = {"L": "#5b92f0", "R": "#f08a3c"}


def mat(ex, ey, ez, t):
    return Matrix(ex.x, ey.x, ez.x, t.x, ex.y, ey.y, ez.y, t.y, ex.z, ey.z, ez.z, t.z, 0, 0, 0, 1)


def frame(origin, a, l):
    a = V(a); a.normalize(); l = V(l); l.normalize()
    return mat(a, l, a.cross(l), origin)


# сборка → портал: (X, Y, Z) → (X − cx, Z − cz, −(Y − cy))
TO_SCENE = Matrix(1, 0, 0, -CENTER.x, 0, 0, 1, -CENTER.z, 0, -1, 0, CENTER.y, 0, 0, 0, 1)


def to_scene(p):
    return TO_SCENE.multVec(p)


def shape_mesh(shape):
    return MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.08, AngularDeflection=0.35, Relative=False)


def write(mesh, name, m):
    mesh = mesh.copy()
    mesh.transform(m)
    if m.determinant() < 0:   # зеркало выворачивает треугольники (Mesh.Volume всегда > 0 — по нему не понять)
        mesh.flipNormals()
    mesh.write(os.path.join(OUT, name))
    return name


def leg_matrices(leg):
    """Нулевая поза: матрицы файл → сборка для бедра, голени и серво колена."""
    o = 1 if leg[1] == "L" else -1
    mirror = o < 0
    zs = -1 if mirror else 1
    y_tip = SHAFT_TIP_Y[leg[1]]
    P = V(HIP_X[leg[0]], y_tip, HIP_Z)
    y_inner = y_tip - o * HORN_GAP
    # бедро прямой стороной наружу (файл Y=9 внутрь), отверстие (0,0,0) на оси бедра
    femur = Matrix(0, 0, o, P.x, 0, -o, 0, y_inner + o * FEMUR_THICK, 1, 0, 0, P.z, 0, 0, 0, 1)

    shaft_f = V(0, TIB_SHAFT.y, zs * TIB_SHAFT.z)
    foot_f = V(0, TIB_FOOT.y, zs * TIB_FOOT.z)
    u = foot_f - shaft_f; u.normalize()
    v = V(0, -u.z, u.y)

    def img(f):
        return V(0, -o, 0) * f.x + V(0, 0, -1) * f.dot(u) + V(o, 0, 0) * f.dot(v)
    ex, ey, ez = img(V(1, 0, 0)), img(V(0, 1, 0)), img(V(0, 0, 1))
    tip_f = V(TIB_TIP_X, shaft_f.y, shaft_f.z)
    K = V(P.x, y_tip, P.z - FEMUR)
    rot = mat(ex, ey, ez, V())
    tibia = mat(ex, ey, ez, K - rot.multVec(tip_f))
    if mirror:
        tibia = tibia * Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1)

    L = V(MG90S_TIP.x - MG90S_EARS.x, 0, MG90S_TIP.z - MG90S_EARS.z)
    servo_local = frame(MG90S_TIP, V(0, -1, 0), L).inverse()
    tip_unmirrored = V(TIB_TIP_X, TIB_SHAFT.y, zs * TIB_SHAFT.z)
    knee_servo = mat(ex, ey, ez, K - rot.multVec(tip_f)) * frame(tip_unmirrored, V(-1, 0, 0), V(0, -1, 0)) * servo_local
    return P, K, femur, tibia, knee_servo


def joint_frame(point):
    """сборка → система сустава в портале (оси портала, начало в суставе)"""
    s = to_scene(point)
    return Matrix(1, 0, 0, -s.x, 0, 1, 0, -s.y, 0, 0, 1, -s.z, 0, 0, 0, 1) * TO_SCENE


def main():
    os.makedirs(OUT, exist_ok=True)
    doc = FreeCAD.openDocument(SRC, hidden=True)
    parts, hips = [], {}
    try:
        for fn, (names, color) in BODY_PARTS.items():
            shape = Part.makeCompound([doc.getObject(n).Shape for n in names])
            parts.append({"file": write(shape_mesh(shape), fn, TO_SCENE), "attach": "body", "color": color})
        ref_servo = shape_mesh(doc.getObject("MG90S_LeftFront").Shape)
        hip_servos = Part.makeCompound([doc.getObject(n).Shape for n in HIP_SERVOS.values()])
        parts.append({"file": write(shape_mesh(hip_servos), "hip_servos.stl", TO_SCENE),
                      "attach": "body", "color": SERVO_COLOR})
    finally:
        FreeCAD.closeDocument(doc.Name)

    femur_mesh = Mesh.Mesh(os.path.join(FIX, "Femur_hub7.stl"))
    tibia_mesh = Mesh.Mesh(os.path.join(FIX, "Tibia_v2_SG90_fit.stl"))
    insert_mesh = Mesh.Mesh(os.path.join(FIX, "Tibia_v2_SG90_fit_insert.stl"))
    tibia_len = None
    for leg in ("FL", "FR", "RL", "RR"):
        P, K, femur, tibia, knee_servo = leg_matrices(leg)
        hip_f, knee_f = joint_frame(P), joint_frame(K)
        color = LEG_COLOR[leg[1]]
        s = to_scene(P)
        hips[leg] = [round(s.x, 2), round(s.y, 2), round(s.z, 2)]
        parts += [
            {"file": write(femur_mesh, f"femur_{leg}.stl", hip_f * femur), "attach": "hip", "legs": [leg], "color": color},
            {"file": write(tibia_mesh, f"tibia_{leg}.stl", knee_f * tibia), "attach": "knee", "legs": [leg], "color": color},
            {"file": write(insert_mesh, f"tibia_insert_{leg}.stl", knee_f * tibia), "attach": "knee", "legs": [leg], "color": color},
            {"file": write(ref_servo, f"knee_servo_{leg}.stl", knee_f * knee_servo),
             "attach": "knee", "legs": [leg], "color": SERVO_COLOR},
        ]
        foot = (knee_f * tibia).multVec(TIB_FOOT)
        tibia_len = round(math.hypot(foot.x, foot.y), 1)

    manifest = {
        "units": "mm",
        "axes": "x вперёд, y вверх, z вправо; начало — центр корпуса на высоте осей бёдер",
        "source": "cad/web/export_web_models.py",
        "geometry": {"femur_mm": FEMUR, "tibia_mm": tibia_len,
                     "foot_inset_mm": round(TIB_FOOT_X - TIB_TIP_X, 1)},  # подошва внутрь от торца вала бедра
        "hips": hips,
        "parts": parts,
    }
    with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print("exported", len(parts), "parts to", OUT, "tibia_mm", tibia_len, "hips", hips)


main()
