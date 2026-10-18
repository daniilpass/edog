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
        _, (fx, fz) = fk(hip.servo_to_joint(hip.target), knee.servo_to_joint(knee.target), g["femur_mm"], g["tibia_mm"])
        side = 1 if leg.side == "left" else -1
        hip_b = (leg.x_mm, side * g["body_width_mm"] / 2, 0.0)
        world = _world_foot((fx, fz), hip_b, robot.body.height, 15, 6)
        assert world[2] == pytest.approx(0, abs=0.5), name
