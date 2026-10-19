"""Загрузка config.toml и calibration.json."""

import json
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

JOINTS = ("hip", "knee")
CALIB_FIELDS = ("installed", "zero", "dir", "min_deg", "max_deg")


@dataclass
class ServoCfg:
    id: str
    leg: str
    joint: str
    channel: int
    installed: bool = False
    ref_deg: float = 0.0
    dir: int = 1
    zero: float = 90.0
    min_deg: float = 5.0
    max_deg: float = 175.0
    range_deg: float = 180.0
    pulse_min_us: float = 500.0
    pulse_max_us: float = 2500.0


@dataclass
class LegCfg:
    name: str
    side: str
    x_mm: float
    knee_dir: int


@dataclass
class Config:
    path: Path
    server: dict
    driver: dict
    geometry: dict
    motion: dict
    gamepad: dict
    power: dict = field(default_factory=dict)   # [power] из config.toml, поверх — "power" из calibration.json
    legs: list[LegCfg] = field(default_factory=list)
    servos: list[ServoCfg] = field(default_factory=list)

    @property
    def calibration_path(self) -> Path:
        return self.path.with_name("calibration.json")


def load_config(path: str | Path) -> Config:
    path = Path(path)
    with path.open("rb") as f:
        raw = tomllib.load(f)

    defaults = raw.get("servo_defaults", {})
    joints = raw.get("joints", {})
    legs, servos = [], []
    for name, leg in raw["legs"].items():
        if leg["side"] not in ("left", "right"):
            raise ValueError(f"legs.{name}.side: ожидается left/right")
        legs.append(LegCfg(name, leg["side"], float(leg["x_mm"]), int(leg.get("knee_dir", -1))))
        for joint in JOINTS:
            params = {"ref_deg": joints.get(joint, {}).get("ref_deg", 0.0), **defaults, **leg[joint]}
            if "zero" not in params:
                params["zero"] = params.get("range_deg", 180.0) / 2 - params.get("dir", 1) * params["ref_deg"]
            servos.append(ServoCfg(id=f"{name}_{joint}", leg=name, joint=joint, **params))

    channels = [s.channel for s in servos]
    if len(set(channels)) != len(channels):
        raise ValueError(f"каналы PCA9685 повторяются: {channels}")

    cfg = Config(
        path=path,
        server=raw.get("server", {}),
        driver=raw.get("driver", {}),
        geometry=raw["geometry"],
        motion=raw["motion"],
        gamepad=raw.get("gamepad", {}),
        power=raw.get("power", {}),
        legs=legs,
        servos=servos,
    )
    _apply_calibration(cfg)
    return cfg


def _apply_calibration(cfg: Config) -> None:
    data = _read_calibration(cfg)
    cfg.power.update(data.get("power", {}))
    for s in cfg.servos:
        for key, value in data.get(s.id, {}).items():
            if key in CALIB_FIELDS:
                setattr(s, key, type(getattr(s, key))(value))


def _read_calibration(cfg: Config) -> dict:
    if not cfg.calibration_path.exists():
        return {}
    return json.loads(cfg.calibration_path.read_text(encoding="utf-8"))


def save_calibration(cfg: Config, servos) -> Path:
    data = _read_calibration(cfg)
    data.update({s.cfg.id: {k: getattr(s.cfg, k) for k in CALIB_FIELDS} for s in servos})
    return _write_calibration(cfg, data)


def save_power(cfg: Config, power: dict) -> Path:
    """Только настройки питания: несохранённая калибровка серво в файл не попадает."""
    data = _read_calibration(cfg)
    data["power"] = power
    return _write_calibration(cfg, data)


def _write_calibration(cfg: Config, data: dict) -> Path:
    tmp = cfg.calibration_path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(cfg.calibration_path)
    return cfg.calibration_path
