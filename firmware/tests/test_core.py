import json
import time
from pathlib import Path

import pytest

from edog.config import _apply_calibration, load_config
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
    assert sorted(robot.driver.pulses) == sorted(s.cfg.channel for s in robot.servos.values())
    robot.set_enabled(False)
    for _ in range(500):
        robot.step(0.02)
    assert all(v is None for v in robot.driver.pulses.values())


def test_slew_limit(robot):
    robot.set_enabled(True)
    robot.set_servo("FL_hip", angle=170)
    before = robot.servos["FL_hip"].current
    robot.step(0.02)
    assert robot.servos["FL_hip"].current - before == pytest.approx(robot.speed_limit() * 0.02)


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
    r.set_pose("stand")
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


def test_power_saved_with_calibration(robot):
    robot.update_power(limit_a=8.0)
    assert not robot.cfg.calibration_path.exists()     # применено, но не сохранено
    robot.servos["FL_hip"].cfg.zero = 12.0
    robot.save_calibration()
    data = json.loads(robot.cfg.calibration_path.read_text(encoding="utf-8"))
    assert data["power"] == {"limit_a": 8.0, "servo_a": 0.9} and data["FL_hip"]["zero"] == 12.0


@pytest.mark.parametrize("gait", list(load_config(CONFIG).gaits))
@pytest.mark.parametrize("limit_a,program,legs", [(3.0, "solo", 1), (4.0, "pairs", 2), (0, "all", 4)])
def test_demo_by_power_limit(guarded, limit_a, program, legs, gait):
    r = guarded
    r.set_gait(gait)
    for s in r.cfg.servos:
        s.installed = False
    r.update_power(limit_a=limit_a, servo_a=0.9)
    r.start_demo(cycles=2, laps=2)
    assert r.state()["demo"]["program"] == program
    max_servos = r.power.max_servos() or 8
    for _ in range(20000):
        r.step(0.02)
        on = [ch for ch, us in r.driver.pulses.items() if us is not None]
        assert len(on) <= min(2 * legs, max_servos)
        assert not any(r.blocked.values()), r.state()["demo"]
        assert not any(s.clamped for s in r.servos.values())
        if not r.demo:
            break
    assert not r.demo and not r.enabled and r.mode == "manual"
    assert all(us is None for us in r.driver.pulses.values())
    for s in r.servos.values():                        # закончили в монтажном положении
        assert s.servo_to_joint(s.current) == pytest.approx(s.cfg.ref_deg, abs=1)


def test_demo_needs_two_servos(robot):
    robot.power.limit_a, robot.power.servo_a = 1.0, 0.9   # 1 серво — даже одной ноге мало
    with pytest.raises(ValueError, match="хотя бы 2"):
        robot.start_demo()
    assert not robot.demo and not robot.enabled


def test_any_command_stops_demo(robot):
    robot.start_demo()
    robot.step(0.02)
    robot.set_pose("stand")
    assert not robot.demo and not robot.enabled and robot.state()["demo"] is None


def test_gait_profiles(robot):
    gaits = {g["name"]: g for g in robot.describe()["gaits"]}
    assert robot.state()["gait"] == robot.cfg.gait == "crawl"
    assert gaits["base"]["stride_mm"] == 30.0 and gaits["base"]["lift_mm"] == 15.0   # прежняя ходьба
    assert gaits["max"]["hip_swing"] > gaits["wide"]["hip_swing"] > gaits["base"]["hip_swing"]
    with pytest.raises(ValueError):
        robot.set_gait("нет такого")
    robot.set_gait("wide")
    assert robot.speed_limit() == robot.cfg.gaits["wide"]["max_speed_dps"]
    robot.save_calibration()
    data = json.loads(robot.cfg.calibration_path.read_text(encoding="utf-8"))
    assert data["gait"] == "wide"


def test_trim(robot):
    """Поправка наклона: работает как наклон корпуса, сохраняется и читается из calibration.json."""
    robot.update_trim(roll_deg=3)
    trimmed = robot.feet(0.02)
    robot.update_trim(roll_deg=0)
    robot.set_body(roll=3)
    assert robot.feet(0.02) == pytest.approx(trimmed)
    with pytest.raises(ValueError):
        robot.update_trim(pitch_deg=20)
    robot.update_trim(pitch_deg=-1.5, roll_deg=3)
    robot.save_calibration()
    data = json.loads(robot.cfg.calibration_path.read_text(encoding="utf-8"))
    assert data["trim"] == {"pitch_deg": -1.5, "roll_deg": 3.0}
    robot.cfg.trim = {"pitch_deg": 0.0, "roll_deg": 0.0}
    _apply_calibration(robot.cfg)
    assert robot.cfg.trim == {"pitch_deg": -1.5, "roll_deg": 3.0}


def test_foot_offset_only_forward(robot):
    """Вынос лап профиля — только при ходьбе вперёд; назад лапы под осью бедра."""
    robot.set_gait("base")
    for vx, expected in ((1, robot.gait["foot_x_mm"]), (-1, 0.0)):
        for _ in range(60):
            robot.set_web_drive(vx, 0)
            robot.feet(0.02)
        x0 = robot.forward_blend * robot.gait["foot_x_mm"]
        assert x0 == pytest.approx(expected)


def test_mount_pose(robot):
    """Стойка «монтаж» — суставы в ref_deg; стик ходьбы возвращает в обычную стойку."""
    robot.set_enabled(True)
    robot.set_pose("mount")
    assert "mount" in robot.describe()["poses"] and robot.state()["pose"] == "mount"
    for _ in range(200):
        robot.step(0.02)
    for s in robot.servos.values():
        assert s.servo_to_joint(s.current) == pytest.approx(s.cfg.ref_deg)
    robot.set_web_drive(1, 0)
    robot.step(0.02)
    assert robot.mode == "stand" and robot.pose_name == "stand"


def test_crawl_keeps_three_feet_down(robot):
    """Шаг: в воздухе не больше одной лапы, каждая за цикл поднимается; корпус качается."""
    from edog.robot import body_sway, leg_step
    g = robot.cfg.gaits["crawl"]
    lifted, sway = set(), set()
    for i in range(400):
        up = [leg for leg in robot.legs if leg_step(g, leg, i / 400, g["stride_mm"], g["lift_mm"])[1] > 0]
        assert len(up) <= 1
        lifted.update(up)
        sway.add(tuple(round(v) for v in body_sway(g, i / 400)))
    assert lifted == set(robot.legs)
    a, r = round(g["sway_mm"]), round(g["sway_roll_deg"])
    assert {(a, r), (-a, r), (a, -r), (-a, -r)} <= sway


def test_gait_height_and_foot_offset(robot):
    """Профиль stopper: при ходьбе корпус плавно поднимается к height_mm, лапы уходят на foot_x_mm,
    после остановки — обратно в стойку."""
    robot.set_gait("stopper")
    g, stand = robot.gait, robot.body.height
    robot.set_web_drive(1, 0)
    feet = robot.feet(0.02)
    assert robot.walk_blend < 0.1
    for _ in range(40):
        feet = robot.feet(0.02)
    assert robot.walk_blend == 1.0
    zs = [z for _, z in feet.values()]
    assert max(zs) == pytest.approx(g["height_mm"])   # опорные лапы на полу
    robot.set_web_drive(0, 0)
    robot.web_drive_t -= 10
    for _ in range(40):
        feet = robot.feet(0.02)
    assert robot.walk_blend == 0.0
    assert all(x == pytest.approx(0) and z == pytest.approx(stand) for x, z in feet.values())


@pytest.mark.parametrize("gait", list(load_config(CONFIG).gaits))
def test_walk_with_every_gait(guarded, gait):
    r = guarded
    r.set_gait(gait)
    r.set_enabled(True)
    r.set_web_drive(1.0, 0.0)
    for _ in range(200):
        r.web_drive_t = time.monotonic()
        r.step(0.02)
        assert not any(r.blocked.values()) and not any(s.clamped for s in r.servos.values())


def test_drive_leaves_manual_mode(robot):
    robot.set_mode("manual")
    robot.set_web_drive(1, 0)
    robot._update_targets(0.02)
    assert robot.mode == "stand"


def test_gamepad_left_stick_dominant_axis(robot):
    from edog.gamepad import Gamepad
    pad = Gamepad(robot, {})
    pad.values = {"drive": -0.8, "turn": 0.3, "pitch": 0.0, "roll": 0.0, "height_down": 0.0, "height_up": 0.0}
    pad._apply()
    assert robot.pad_drive == (1.0, 0.0)
    pad.values.update(drive=-0.2, turn=0.5)
    pad._apply()
    assert robot.pad_drive == (0.0, 1.0)


def test_drive_switches_to_stand_pose(robot):
    robot.set_pose("sit")
    robot.set_web_drive(1, 0)
    robot._update_targets(0.02)
    assert robot.pose_name == "stand"


def test_power_off_goes_to_mounting_position_first(robot):
    robot.set_enabled(True)
    robot.set_enabled(False)
    assert robot.enabled and robot.powering_down is not None
    for _ in range(500):
        robot.step(0.02)
        if not robot.enabled:
            break
    assert not robot.enabled and robot.powering_down is None
    for s in robot.servos.values():
        assert abs(s.servo_to_joint(s.current) - s.cfg.ref_deg) < 1


def test_estop_cuts_signal_at_once(robot):
    robot.set_enabled(True)
    robot.estop()
    assert not robot.enabled


def test_power_on_starts_at_mounting_position_then_stands(robot):
    robot.set_pose("sit")
    robot.set_enabled(True)
    assert robot.pose_name == "stand"
    for s in robot.servos.values():
        assert abs(s.servo_to_joint(s.current) - s.cfg.ref_deg) < 1
    for _ in range(500):
        robot.step(0.02)
    assert all(abs(s.target - s.current) < 0.5 for s in robot.servos.values())


def test_trigger_changes_height_while_held(robot):
    h = robot.body.height
    robot.height_rate = -1.0
    for _ in range(10):
        robot.step(0.02)
    assert robot.body.height < h and robot.pose_name == "stand"
    robot.height_rate = 0.0
    low = robot.body.height
    robot.step(0.02)
    assert robot.body.height == low


def test_jump_ends_in_stand(robot):
    with pytest.raises(ValueError):
        robot.start_move("jump")   # серво выключены
    robot.set_enabled(True)
    robot.start_move("jump")
    assert robot.move_name == "jump" and robot.body.height < robot.m["stand_height_mm"]
    heights = []
    for _ in range(100):
        robot.step(0.02)
        heights.append(robot.body.height)
    assert max(heights) > robot.m["max_height_mm"]   # толчок до прямой ноги
    assert robot.move_name is None and robot.pose_name == "stand"


def test_pose_interrupts_move(robot):
    robot.set_enabled(True)
    robot.start_move("jump")
    robot.set_pose("sit")
    assert robot.move_name is None and robot.pose_name == "sit"



