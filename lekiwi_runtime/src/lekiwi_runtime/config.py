"""Packaged defaults, with optional per-robot JSON overrides."""

import json
from pathlib import Path

DATA_DIR = Path(__file__).parent / "data"


def load_config(path: Path | None = None) -> dict:
    config = json.loads((DATA_DIR / "robot.json").read_text(encoding="utf-8"))
    config["radar"]["cfg_path"] = str(DATA_DIR / config["radar"]["cfg_path"])
    config["arm"]["calib_path"] = str(DATA_DIR / config["arm"]["calib_path"])
    if path is not None:
        path = path.expanduser().resolve()
        overrides = json.loads(path.read_text(encoding="utf-8"))
        _merge(config, overrides)
        for section, key in (("radar", "cfg_path"), ("arm", "calib_path")):
            value = config[section][key]
            if not isinstance(value, str) or not value:
                raise ValueError(f"{section}.{key} must be a file path")
            config[section][key] = str((path.parent / Path(value).expanduser()).resolve())
    for sensor in [*config["cameras"].values(), config["radar"], config["arm"], config["imu"]]:
        if type(sensor["enabled"]) is not bool:
            raise ValueError("Sensor enabled must be true or false")
    if type(config["imu"]["bus"]) is not int or config["imu"]["bus"] < 0:
        raise ValueError("imu.bus must be a nonnegative integer")
    if config["imu"]["address"] not in (0x68, 0x69) or type(config["imu"]["address"]) is not int:
        raise ValueError("imu.address must be 104 or 105")
    return config


def _merge(defaults: dict, overrides: dict, prefix: str = "") -> None:
    if not isinstance(overrides, dict):
        raise ValueError(f"{prefix or 'config'} must be an object")
    for key, value in overrides.items():
        if key not in defaults:
            raise ValueError(f"Unknown configuration key: {prefix}{key}")
        if isinstance(defaults[key], dict):
            _merge(defaults[key], value, f"{prefix}{key}.")
        else:
            defaults[key] = value
