"""Read-only STS3215 joint observations; no torque or motion commands."""

import json
from pathlib import Path
import time

from .servo import ServoBus

ARM_JOINTS = (
    ("shoulder_pan", 1), ("shoulder_lift", 2), ("elbow_flex", 3),
    ("wrist_flex", 4), ("wrist_roll", 5), ("gripper", 6),
)
PRESENT_POSITION = 56


def load_calibration(path: str) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    calibration = {}
    for name, motor_id in ARM_JOINTS:
        entry = data.get(name, data.get(f"arm_{name}"))
        if not isinstance(entry, dict):
            raise ValueError(f"Missing calibration for {name}")
        low, high = entry.get("range_min"), entry.get("range_max")
        if (type(low) is not int or type(high) is not int
                or not 0 <= low < high <= 4095
                or entry.get("id", motor_id) != motor_id
                or entry.get("drive_mode", 0) not in (0, 1)):
            raise ValueError(f"Invalid calibration for {name}")
        calibration[name] = entry
    return calibration


def raw_to_unit(raw: int, name: str, calibration: dict) -> float:
    low, high = calibration["range_min"], calibration["range_max"]
    if name == "gripper":
        percent = (min(high, max(low, raw)) - low) / (high - low) * 100
        return 100 - percent if calibration.get("drive_mode", 0) else percent
    return (raw - (low + high) / 2) * 360 / 4095


class ArmEgoReader:
    def __init__(self, config: dict, mock: bool = False, bus: ServoBus | None = None):
        self.config = config
        self.mock = mock
        self.enabled = config["enabled"]
        self.calibration = load_calibration(config["calib_path"]) if self.enabled else {}
        self.bus = bus if bus is not None else ServoBus(config)
        self.error = None

    def start(self) -> None:
        if not self.enabled or self.mock:
            return
        try:
            self.bus.connect()
            self.error = None
        except Exception as exc:
            self.error = str(exc)

    def close(self) -> None:
        self.bus.close()

    def read_ego(self) -> dict:
        names = [name for name, _ in ARM_JOINTS]
        result = {"available": False, "port": ("mock" if self.mock else self.bus.port), "joints": {}, "angles": [],
                  "joint_names": names, "raw": {}, "timestamp": None, "error": None}
        if not self.enabled:
            return {**result, "error": "disabled"}
        if self.mock:
            raw = {name: round((cal["range_min"] + cal["range_max"]) / 2)
                   for name, cal in self.calibration.items()}
            errors = []
        else:
            if self.bus.port_handler is None:
                self.start()
            if self.bus.port_handler is None:
                return {**result, "error": self.error}
            import scservo_sdk as sdk

            raw, errors = {}, []
            try:
                for name, motor_id in ARM_JOINTS:
                    for _ in range(3):
                        value, comm, error = self.bus.packet_handler.read2ByteTxRx(
                            self.bus.port_handler, motor_id, PRESENT_POSITION)
                        if comm == sdk.COMM_SUCCESS and error == 0:
                            raw[name] = int(value)
                            break
                    else:
                        errors.append(f"{name}: communication={comm}, device={error}")
            except Exception as exc:
                errors.append(str(exc))
            if errors:
                self.close()  # Reconnect on the next observation after a disconnect.
        joints = {name: round(raw_to_unit(value, name, self.calibration[name]), 3)
                  for name, value in raw.items()}
        return {**result, "available": not errors, "port": ("mock" if self.mock else self.bus.port),
                "joints": joints, "angles": [joints[n] for n in names if n in joints],
                "raw": raw, "timestamp": time.time(), "error": "; ".join(errors) or None}
