"""STS3215 calibrated joint feedback and bounded position trajectories."""

from contextlib import nullcontext
import json
import math
import sys
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


class ArmController:
    def __init__(self, config: dict, mock: bool = False, bus: ServoBus | None = None):
        self.config = config
        self.mock = mock
        self.enabled = config["enabled"]
        self.calibration = load_calibration(config["calib_path"]) if self.enabled else {}
        self.bus = bus if bus is not None else ServoBus(config)
        self.error = None
        self.last_motion = None
        self._pending = set()
        self._mock_raw = {name: round((cal["range_min"] + cal["range_max"]) / 2)
                          for name, cal in self.calibration.items()}

    def start(self) -> None:
        if not self.enabled or self.mock:
            return
        try:
            self.bus.connect()
            self.error = None
        except Exception as exc:
            self.error = str(exc)

    def close(self) -> None:
        errors = []
        try:
            if self.enabled and not self.mock and self.bus.port_handler is not None:
                for _, motor_id in ARM_JOINTS:
                    try:
                        self._write(motor_id, 40, 0, 1)
                    except Exception as exc:
                        errors.append(str(exc))
                self._pending.clear()
                if errors:
                    raise OSError("Arm torque release failed: " + "; ".join(errors))
                print("Arm torque released", file=sys.stderr, flush=True)
        finally:
            self.bus.close()

    def read_ego(self) -> dict:
        names = [name for name, _ in ARM_JOINTS]
        result = {"available": False, "port": ("mock" if self.mock else self.bus.port), "joints": {}, "angles": [],
                  "joint_names": names, "raw": {}, "timestamp": None, "error": None,
                  "limits": self.limits(), "last_motion": self.last_motion}
        if not self.enabled:
            return {**result, "error": "disabled"}
        if self.mock:
            raw = dict(self._mock_raw)
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
                self.bus.close()  # Reconnect on the next observation; shutdown releases torque.
        joints = {name: round(raw_to_unit(value, name, self.calibration[name]), 3)
                  for name, value in raw.items()}
        return {**result, "available": not errors, "port": ("mock" if self.mock else self.bus.port),
                "joints": joints, "angles": [joints[n] for n in names if n in joints],
                "raw": raw, "timestamp": time.time(), "error": "; ".join(errors) or None}

    def limits(self) -> dict:
        return {name: {"min": 0 if name == "gripper" else raw_to_unit(cal["range_min"], name, cal),
                       "max": 100 if name == "gripper" else raw_to_unit(cal["range_max"], name, cal),
                       "max_speed": 50 if name == "gripper" else 30,
                       "unit": "%" if name == "gripper" else "deg"}
                for name, cal in self.calibration.items()}

    def valid_target(self, joints, duration_ms) -> bool:
        if not self.enabled or type(duration_ms) is not int or not 100 <= duration_ms <= 1000:
            return False
        if not isinstance(joints, dict) or not joints:
            return False
        limits = self.limits()
        return all(name in limits and type(value) in (int, float) and math.isfinite(value)
                   and limits[name]["min"] <= value <= limits[name]["max"]
                   for name, value in joints.items())

    def _read(self, motor_id, address, size=2):
        import scservo_sdk as sdk
        read = self.bus.packet_handler.read1ByteTxRx if size == 1 else self.bus.packet_handler.read2ByteTxRx
        value, comm, error = read(self.bus.port_handler, motor_id, address)
        if comm != sdk.COMM_SUCCESS or error:
            raise OSError(f"Arm {motor_id} read {address}: communication={comm}, device={error}")
        return value

    def _write(self, motor_id, address, value, size=2):
        import scservo_sdk as sdk
        write = self.bus.packet_handler.write1ByteTxRx if size == 1 else self.bus.packet_handler.write2ByteTxRx
        comm, error = write(self.bus.port_handler, motor_id, address, value)
        if comm != sdk.COMM_SUCCESS or error:
            raise OSError(f"Arm {motor_id} write {address}: communication={comm}, device={error}")

    def hold(self):
        """On interruption hold measured positions, without dropping arm torque."""
        errors = []
        for motor_id in sorted(self._pending):
            try:
                self._write(motor_id, 42, self._read(motor_id, PRESENT_POSITION))
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise OSError("Arm hold failed; physical state unknown: " + "; ".join(errors))
        if self._pending:
            print("Arm holding measured position", file=sys.stderr, flush=True)
        self._pending.clear()

    def move(self, joints: dict, duration_ms: int, base_motion=None):
        if not self.valid_target(joints, duration_ms):
            raise ValueError("Invalid arm targets or duration")
        self.last_motion = None
        before = self.read_ego()
        if not before["available"]:
            raise OSError("Arm feedback unavailable: " + str(before["error"]))
        seconds = duration_ms / 1000
        limits = self.limits()
        ids = dict(ARM_JOINTS)
        goals = {}
        for name, target in joints.items():
            cal = self.calibration[name]
            current = before["raw"][name]
            if not cal["range_min"] <= current <= cal["range_max"]:
                raise ValueError(f"Current {name} is outside calibration")
            if abs(target - before["joints"][name]) > limits[name]["max_speed"] * seconds:
                raise ValueError(f"Requested {name} motion exceeds speed limit {limits[name]['max_speed']} {limits[name]['unit']}/s")
            if name == "gripper":
                percent = 100 - target if cal.get("drive_mode", 0) else target
                raw = cal["range_min"] + percent / 100 * (cal["range_max"] - cal["range_min"])
            else:
                raw = (cal["range_min"] + cal["range_max"]) / 2 + target * 4095 / 360
            goals[name] = round(raw)
        print(f"Arm motion targets={joints} duration_ms={duration_ms} mock={self.mock}", file=sys.stderr, flush=True)
        if self.mock:
            with base_motion if base_motion is not None else nullcontext():
                time.sleep(seconds)
                self._mock_raw.update(goals)
        else:
            # Read all checks before writing any goal or enabling torque.
            for name in joints:
                mid = ids[name]
                if self._read(mid, 33, 1) != 0:
                    raise OSError(f"Arm {mid} must already be in position mode")
                if self._read(mid, 63, 1) >= 65:
                    raise OSError(f"Arm {mid} temperature exceeds limit")
                offset = self._read(mid, 31)
                offset = -(offset & 0x7ff) if offset & 0x800 else offset
                if offset != self.calibration[name].get("homing_offset"):
                    raise OSError(f"Arm {mid} homing offset does not match calibration")
            try:
                for name in joints:
                    mid = ids[name]
                    self._pending.add(mid)
                    # Clear a stale goal before torque on. Limit native servo speed too.
                    self._write(mid, 42, before["raw"][name])
                    speed = (self.calibration[name]["range_max"] - self.calibration[name]["range_min"]) / 2 if name == "gripper" else 30 * 4095 / 360
                    self._write(mid, 44, 0)
                    self._write(mid, 46, max(1, round(speed)))
                    self._write(mid, 40, 1, 1)
                with base_motion if base_motion is not None else nullcontext():
                    steps = max(1, math.ceil(seconds / .02))
                    started = time.monotonic()
                    for step in range(1, steps + 1):
                        for name, goal in goals.items():
                            initial = before["raw"][name]
                            self._write(ids[name], 42, round(initial + (goal - initial) * step / steps))
                        time.sleep(max(0, started + step * seconds / steps - time.monotonic()))
                deadline = time.monotonic() + .5
                while True:
                    measured = {name: raw_to_unit(self._read(ids[name], PRESENT_POSITION), name, self.calibration[name])
                                for name in joints}
                    if all(abs(measured[name] - target) <= (5 if name == "gripper" else 2)
                           for name, target in joints.items()):
                        break
                    if time.monotonic() >= deadline:
                        raise OSError("Arm target not reached within feedback tolerance")
                    time.sleep(.05)
            except BaseException:
                self.hold()
                raise
            self._pending.clear()  # Successful trajectory keeps holding its target.
        measured = {name: raw_to_unit(self._mock_raw[name], name, self.calibration[name])
                    for name in joints} if self.mock else measured
        self.last_motion = {"targets": dict(joints), "measured": measured, "reached": True,
                            "tolerance_deg": 2, "tolerance_percent": 5, "timestamp": time.time()}
        print("Arm target reached; holding position", file=sys.stderr, flush=True)
