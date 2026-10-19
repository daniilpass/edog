import json
from pathlib import Path

import pytest

from edog.config import load_config
from edog.kinematics import fk, foot_in_leg, ik
from edog.pca9685 import MockDriver
from edog.robot import Robot

CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.toml"


@pytest.mark.parametrize("knee_dir", [-1, 1])
@pytest.mark.parametrize("x,z", [(0, 70), (15, 60), (-20, 80), (10, 40)])
def test_ik_roundtrip(x, z, knee_dir):
    hip, knee = ik(x, z, 50, 50, knee_dir)
    _, foot = fk(hip, knee, 50, 50)
    assert foot == pytest.approx((x, z), abs=1e-6)


def test_ik_knee_direction():
    hip, knee = ik(0, 70, 50, 50, knee_dir=-1)
    knee_pos, _ = fk(hip, knee, 50, 50)
    assert knee_pos[0] < 0  # колено позади линии бедро–лапа


def test_ik_unreachable_is_clamped():
    hip, knee = ik(0, 500, 50, 50)
    assert abs(knee) < 1 and abs(hip) < 1


@pytest.fixture
def robot(tmp_path):
    cfg = load_config(CONFIG)
    cfg.path = tmp_path / "config.toml"  # калибровка пишется во временную папку
    cfg.power["limit_a"] = 0  # лимит тока проверяется отдельно
    for s in cfg.servos:
        s.installed = True
    return Robot(cfg, MockDriver())


def test_disabled_robot_writes_nothing(robot):
    robot.step(0.02)
    assert robot.driver.pulses == {}


def test_enable_writes_all_channels(robot):
    robot.set_enabled(True)
    robot.step(0.02)
    assert sorted(robot.driver.pulses) == list(range(8))
    robot.set_enabled(False)
    assert all(v is None for v in robot.driver.pulses.values())


def test_slew_limit(robot):
    robot.set_enabled(True)
    robot.set_servo("FL_hip", angle=170)
    before = robot.servos["FL_hip"].current
    robot.step(0.02)
    assert robot.servos["FL_hip"].current - before == pytest.approx(300 * 0.02)


def test_servo_limits_clamped(robot):
    robot.set_servo("FL_hip", angle=179)
    s = robot.servos["FL_hip"]
    assert s.target == 175 and s.clamped


def test_calibrate_and_save(robot):
    robot.set_servo("FR_knee", angle=100)
    robot.step(0.02)
    robot.calibrate("FR_knee", 0)
    assert robot.servos["FR_knee"].cfg.zero == 100
    path = Path(robot.save_calibration())
    assert '"zero": 100' in path.read_text(encoding="utf-8")


def test_walk_moves_feet(robot):
    robot.set_mode("walk")
    robot.set_web_drive(1, 0)
    xs = {robot.feet(0.05)["FL"][0] for _ in range(10)}
    assert len(xs) > 5


def test_web_drive_times_out(robot, monkeypatch):
    robot.set_web_drive(1, 0)
    robot.web_drive_t -= 10
    (vx, turn), source = robot.drive()
    assert (vx, turn, source) == (0, 0, "gamepad")


def test_uninstalled_servo_gets_no_signal(robot):
    robot.update_calibration("RR_knee", installed=False)
    robot.set_enabled(True)
    robot.step(0.02)
    assert 7 not in robot.driver.pulses or robot.driver.pulses[7] is None
    assert robot.driver.pulses[6] is not None


def test_sweep_goes_min_max_back(robot):
    robot.set_enabled(True)
    robot.set_servo("FL_hip", angle=90)
    for _ in range(100):
        robot.step(0.02)
    robot.test_servo("FL_hip", "sweep")
    seen = []
    for _ in range(1000):
        robot.step(0.02)
        seen.append(robot.servos["FL_hip"].current)
    assert min(seen) == 5 and max(seen) == 175
    assert seen[-1] == 90 and not robot.servos["FL_hip"].queue


def test_sweep_requires_enabled(robot):
    with pytest.raises(ValueError):
        robot.test_servo("FL_hip", "sweep")


def _world_foot(leg_xz, hip_b, height, pitch, roll):
    """Обратная проверка: (x, z) ноги → мир."""
    from edog.kinematics import body_rotation
    rot = body_rotation(pitch, roll)
    lx, lz = leg_xz[:2]
    hx, hy, hz = rot(hip_b)
    dx, dy, dz = rot((lx, 0, -lz))
    return hx + dx, hy + dy, height + hz + dz


@pytest.mark.parametrize("pitch,roll", [(0, 0), (15, 0), (-20, 0), (0, 10), (12, -8)])
def test_foot_in_leg_keeps_feet_on_floor(pitch, roll):
    hip = (49.5, 47.5, 0.0)
    foot = (49.5, 47.5, 0.0)
    x, z, lateral = foot_in_leg(foot, hip, 70, pitch, roll)
    wx, wy, wz = _world_foot((x, z), hip, 70, pitch, roll)
    assert (wx, wz) == pytest.approx((foot[0], foot[2]), abs=1e-9)  # на полу и на месте
    assert wy - foot[1] == pytest.approx(lateral, abs=1e-9)          # вбок — сколько сказано
    if roll == 0:
        assert lateral == pytest.approx(0, abs=1e-9)


def test_pitch_nose_up_lengthens_front_legs():
    front = foot_in_leg((49.5, 0, 0), (49.5, 0, 0), 70, 15, 0)
    rear = foot_in_leg((-49.5, 0, 0), (-49.5, 0, 0), 70, 15, 0)
    assert front[1] > 70 > rear[1]


def test_roll_positive_lowers_right_side():
    left = foot_in_leg((0, 47.5, 0), (0, 47.5, 0), 70, 0, 10)
    right = foot_in_leg((0, -47.5, 0), (0, -47.5, 0), 70, 0, 10)
    assert left[1] > 70 > right[1]


def test_tilted_stance_feet_are_coplanar(robot):
    """По углам суставов (FK) лапы при наклоне лежат в одной плоскости пола."""
    robot.set_body(pitch=15, roll=6)
    robot.step(0.02)
    g = robot.g
    for name, leg in robot.legs.items():
        hip = robot.servos[f"{name}_hip"]
        knee = robot.servos[f"{name}_knee"]
        _, (fx, fz) = fk(hip.servo_to_joint(hip.target), knee.servo_to_joint(knee.target),
                         g["femur_mm"], g["tibia_mm"], g["tibia_bend_deg"])
        side = 1 if leg.side == "left" else -1
        hip_b = (leg.x_mm, side * g["body_width_mm"] / 2, 0.0)
        world = _world_foot((fx, fz), hip_b, robot.body.height, 15, 6)
        assert world[2] == pytest.approx(0, abs=0.5), name


def test_api_only_with_cors(robot):
    """На Pi только API: портала и моделей нет, CORS разрешает портал с ПК."""
    from fastapi.testclient import TestClient
    from edog.api import create_app

    app, _ = create_app(robot)
    client = TestClient(app)
    r = client.get("/api/config", headers={"Origin": "http://localhost:8000"})
    assert r.status_code == 200
    assert r.headers["access-control-allow-origin"] in ("*", "http://localhost:8000")
    pre = client.options("/api/pose/sit", headers={
        "Origin": "http://localhost:8000", "Access-Control-Request-Method": "POST",
        "Access-Control-Request-Headers": "content-type"})
    assert pre.status_code == 200
    assert client.get("/index.html").status_code == 404
    assert client.get("/models/manifest.json").status_code == 404
    assert client.get("/").json()["docs"] == "/docs"


@pytest.mark.parametrize("bend", [0, 17.7, -10])
def test_ik_fk_with_tibia_bend(bend):
    """Угол колена — по корпусу голени: изгиб меняет угол, но не положение лапы."""
    hip, knee = ik(5, 65, 50, 41.7, -1, bend)
    hip0, knee0 = ik(5, 65, 50, 41.7, -1, 0)
    assert hip == pytest.approx(hip0) and knee == pytest.approx(knee0 + bend)
    _, foot = fk(hip, knee, 50, 41.7, bend)
    assert foot == pytest.approx((5, 65), abs=1e-6)


def test_mount_pose_is_servo_center(robot):
    """Посадка крыльев при серво 90°: бедро 0° (вниз), колено 90° (корпус голени ⟂ бедру)."""
    for s in robot.servos.values():
        assert s.servo_to_joint(90) == pytest.approx(s.cfg.ref_deg)


def test_all_poses_within_servo_limits(robot):
    for name in robot.poses():
        robot.set_pose(name)
        robot.step(0.02)
        clamped = [s.cfg.id for s in robot.servos.values() if s.clamped]
        assert not clamped, (name, clamped)


@pytest.fixture
def guarded(robot):
    from edog.collision import FrameGuard
    robot.guard = FrameGuard(CONFIG.with_name("leg_hull.json"), robot.g["tibia_bend_deg"], robot.g["frame_margin_mm"])
    return robot


def test_frame_guard_allows_all_poses_and_walk(guarded):
    r = guarded
    r.set_enabled(True)
    for name in r.poses():
        r.set_pose(name)
        for _ in range(100):
            r.step(0.02)
        assert not any(r.blocked.values()), name
    r.set_pose("stand"); r.set_mode("walk")
    for _ in range(200):
        r.set_web_drive(1, 0); r.step(0.02)
        assert not any(r.blocked.values())


def test_frame_guard_rejects_manual_command(guarded):
    r = guarded
    r.set_servo("RL_hip", joint=45)
    r.step(0.02)
    with pytest.raises(ValueError, match="раму"):
        r.set_servo("RL_knee", joint=150)
    r.set_servo("RL_knee", joint=60)          # вниз от рамы — можно


def test_frame_guard_stops_sweep_at_frame(guarded):
    r = guarded
    r.set_enabled(True)
    r.set_servo("RL_hip", joint=30)
    r.set_servo("RL_knee", joint=60)
    for _ in range(200):
        r.step(0.02)
    r.test_servo("RL_knee", "sweep")
    for _ in range(1000):
        r.step(0.02)
        assert r.guard.top(*r._leg_joints("RL", "current")) <= r.guard.limit + 1e-6
    knee = r.servos["RL_knee"]
    assert not knee.queue                      # проход остановлен у рамы
    assert knee.servo_to_joint(knee.current) < 140


def test_power_limit_caps_installed(robot):
    for s in robot.cfg.servos:
        s.installed = False
    robot.update_power(limit_a=3.0, servo_a=0.9)   # 3 / 0.9 → 3 серво
    for sid in ("FL_hip", "FL_knee", "FR_hip"):
        robot.update_calibration(sid, installed=True)
    with pytest.raises(ValueError, match="не больше 3"):
        robot.update_calibration("FR_knee", installed=True)
    assert not robot.servos["FR_knee"].cfg.installed
    robot.update_calibration("FL_hip", installed=False)
    robot.update_calibration("FR_knee", installed=True)
    assert robot.state()["power"] == {"limit_a": 3.0, "servo_a": 0.9, "max_servos": 3, "installed": 3}


def test_power_limit_blocks_enable_and_lowering(robot):
    with pytest.raises(ValueError):
        robot.update_power(limit_a=3.0)                # подключены все 8
    assert robot.power.limit_a == 0
    robot.cfg.power["limit_a"] = 3.0
    robot.power.limit_a = 3.0                          # например, из старого calibration.json
    with pytest.raises(ValueError):
        robot.set_enabled(True)
    assert not robot.enabled


def test_power_saved_without_servo_calibration(robot):
    robot.update_power(limit_a=8.0)
    robot.servos["FL_hip"].cfg.zero = 12.0             # не сохранено — в файл не попадает
    data = json.loads(robot.cfg.calibration_path.read_text(encoding="utf-8"))
    assert data == {"power": {"limit_a": 8.0, "servo_a": 0.9}}
    robot.save_calibration()
    data = json.loads(robot.cfg.calibration_path.read_text(encoding="utf-8"))
    assert data["power"]["limit_a"] == 8.0 and data["FL_hip"]["zero"] == 12.0
