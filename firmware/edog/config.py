"""Загрузка config.toml и calibration.json."""

import json
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

JOINTS = ("hip", "knee")
CALIB_FIELDS = ("installed", "zero", "scale", "dir", "min_deg", "max_deg")


@dataclass
class ServoCfg:
    id: str
    leg: str
    joint: str
    channel: int
    installed: bool
    ref_deg: float
    load_deg: float         # угол сустава в калибровочной позе под нагрузкой
    dir: int
    zero: float
    min_deg: float
    max_deg: float
    range_deg: float
    pulse_min_us: float
    pulse_max_us: float
    scale: float = 1.0      # градусов серво на градус сустава: ход реального серво ≠ номинальному


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
    power: dict                 # [power] из config.toml, поверх — "power" из calibration.json
    gaits: dict[str, dict]      # профили ходьбы [gaits.*]
    gait: str                   # выбранный профиль: motion.gait, поверх — "gait" из calibration.json
    legs: list[LegCfg]
    servos: list[ServoCfg]
    # поправка наклона корпуса на огрехи сборки, °: прибавляется к тангажу и крену всегда; из calibration.json
    trim: dict = field(default_factory=lambda: {"pitch_deg": 0.0, "roll_deg": 0.0})

    @property
    def calibration_path(self) -> Path:
        return self.path.with_name("calibration.json")


def load_config(path: str | Path) -> Config:
    path = Path(path)
    with path.open("rb") as f:
        raw = tomllib.load(f)

    legs, servos = [], []
    for name, leg in raw["legs"].items():
        if leg["side"] not in ("left", "right"):
            raise ValueError(f"legs.{name}.side: ожидается left/right")
        legs.append(LegCfg(name, leg["side"], float(leg["x_mm"]), int(leg["knee_dir"])))
        for joint in JOINTS:
            params = {**raw["joints"][joint], **raw["servo_defaults"], **leg[joint]}
            # в центре серво сустав стоит в монтажном положении; уточняется калибровкой
            params["zero"] = params["range_deg"] / 2 - params["dir"] * params["ref_deg"]
            servos.append(ServoCfg(id=f"{name}_{joint}", leg=name, joint=joint, **params))

    channels = [s.channel for s in servos]
    if len(set(channels)) != len(channels):
        raise ValueError(f"каналы PCA9685 повторяются: {channels}")

    gaits = raw["gaits"]
    m = raw["motion"]
    for name, g in gaits.items():
        missing = {"title", "gait_hz", "stride_mm", "lift_mm", "foot_x_mm"} - g.keys()
        if missing:
            raise ValueError(f"gaits.{name}: нет {', '.join(sorted(missing))}")
        if not m["min_height_mm"] <= g.get("height_mm", m["min_height_mm"]) <= m["max_height_mm"]:
            raise ValueError(f"gaits.{name}.height_mm: вне motion.min_height_mm … max_height_mm")

    cfg = Config(
        path=path,
        server=raw["server"],
        driver=raw["driver"],
        geometry=raw["geometry"],
        motion=m,
        gamepad=raw["gamepad"],
        power=raw["power"],
        gaits=gaits,
        gait=m["gait"],
        legs=legs,
        servos=servos,
    )
    _apply_calibration(cfg)
    return cfg


def _apply_calibration(cfg: Config) -> None:
    data = _read_calibration(cfg)
    cfg.power.update(data.get("power", {}))
    cfg.trim.update({k: float(v) for k, v in data.get("trim", {}).items() if k in cfg.trim})
    if data.get("gait") in cfg.gaits:
        cfg.gait = data["gait"]
    if cfg.gait not in cfg.gaits:
        raise ValueError(f"motion.gait = {cfg.gait!r}: нет такого профиля, есть {tuple(cfg.gaits)}")
    for s in cfg.servos:
        for key, value in data.get(s.id, {}).items():
            if key in CALIB_FIELDS:
                setattr(s, key, type(getattr(s, key))(value))


def _read_calibration(cfg: Config) -> dict:
    if not cfg.calibration_path.exists():
        return {}
    return json.loads(cfg.calibration_path.read_text(encoding="utf-8"))


def save_calibration(cfg: Config, servos, power: dict, gait: str, trim: dict) -> Path:
    """Всё, что меняется из портала, одним файлом: калибровка серво, лимит тока, профиль ходьбы,
    поправка наклона корпуса."""
    data = {s.cfg.id: {k: getattr(s.cfg, k) for k in CALIB_FIELDS} for s in servos}
    data["power"] = power
    data["gait"] = gait
    data["trim"] = trim
    tmp = cfg.calibration_path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(cfg.calibration_path)
    return cfg.calibration_path
