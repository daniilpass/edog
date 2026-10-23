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
    installed: bool = False
    ref_deg: float = 0.0
    load_deg: float = 0.0   # угол сустава в калибровочной позе под нагрузкой
    dir: int = 1
    zero: float = 90.0
    scale: float = 1.0      # градусов серво на градус сустава: ход реального серво ≠ номинальному
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
    gaits: dict[str, dict] = field(default_factory=dict)   # профили ходьбы [gaits.*]
    gait: str = ""                              # выбранный профиль: motion.gait, поверх — "gait" из calibration.json
    # поправка наклона корпуса на огрехи сборки, °: прибавляется к тангажу и крену всегда; из calibration.json
    trim: dict = field(default_factory=lambda: {"pitch_deg": 0.0, "roll_deg": 0.0})
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
            ref = joints.get(joint, {}).get("ref_deg", 0.0)
            params = {"ref_deg": ref, "load_deg": joints.get(joint, {}).get("load_deg", ref), **defaults, **leg[joint]}
            if "zero" not in params:
                params["zero"] = (params.get("range_deg", 180.0) / 2
                                  - params.get("dir", 1) * params.get("scale", 1.0) * params["ref_deg"])
            servos.append(ServoCfg(id=f"{name}_{joint}", leg=name, joint=joint, **params))

    channels = [s.channel for s in servos]
    if len(set(channels)) != len(channels):
        raise ValueError(f"каналы PCA9685 повторяются: {channels}")

    gaits = raw.get("gaits", {})
    if not gaits:
        raise ValueError("нет ни одного профиля ходьбы [gaits.*]")
    for name, g in gaits.items():
        missing = {"gait_hz", "stride_mm", "lift_mm"} - g.keys()
        if missing:
            raise ValueError(f"gaits.{name}: нет {', '.join(sorted(missing))}")
        g.setdefault("title", name)
        if g.get("pattern", "trot") not in ("trot", "crawl"):
            raise ValueError(f"gaits.{name}.pattern: trot или crawl")
        if g.get("pattern") == "crawl":
            swing = g.get("swing", 0.1)
            if not (0 < swing and 4 * swing + 2 * g.get("gap", swing) < 1):
                raise ValueError(f"gaits.{name}: у шага 4·swing + 2·gap < 1 — иначе не остаётся паузы на четырёх лапах")
        m = raw["motion"]
        if not m["min_height_mm"] <= g.get("height_mm", m["min_height_mm"]) <= m["max_height_mm"]:
            raise ValueError(f"gaits.{name}.height_mm: вне motion.min_height_mm … max_height_mm")

    cfg = Config(
        path=path,
        server=raw.get("server", {}),
        driver=raw.get("driver", {}),
        geometry=raw["geometry"],
        motion=raw["motion"],
        gamepad=raw.get("gamepad", {}),
        power=raw.get("power", {}),
        gaits=gaits,
        gait=raw["motion"].get("gait", next(iter(gaits))),
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
