"""STS3215 calibrated joint feedback and bounded position trajectories."""

from contextlib import nullcontext
from hashlib import sha256
import json
import math
import sys
from pathlib import Path
import time

from .servo import ServoBus
from ..robot import ARM_MAX_DURATION_MS, BASE_MAX_DURATION_MS, PreDispatchRejected

ARM_JOINTS = (
    ("shoulder_pan", 1), ("shoulder_lift", 2), ("elbow_flex", 3),
    ("wrist_flex", 4), ("wrist_roll", 5), ("gripper", 6),
)
PRESENT_POSITION = 56


class ArmTransportError(OSError):
    """Preserve SDK transport/device status without inferring actuation state."""
    def __init__(self, operation, motor_id, register, communication, device_error):
        self.operation = operation
        self.motor_id = motor_id
        self.register = register
        self.communication = communication
        self.device_error = device_error
        super().__init__(f"Arm {motor_id} {operation} {register}: "
                         f"communication={communication}, device={device_error}")


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
        self.calibration_id = sha256(json.dumps(self.calibration, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
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
                  "limits": self.limits(), "last_motion": self.last_motion, "calibration_id": self.calibration_id,
                  "calibration_status": self.calibration_status({}, available=False)}
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
                "raw": raw, "timestamp": time.time(), "error": "; ".join(errors) or None,
                "calibration_status": self.calibration_status(raw, available=not errors)}

    def calibration_status(self, raw: dict, *, available=True) -> dict:
        """Report measured raw-range validity without changing bounds or feedback.

        Gripper percentages are clamped for the legacy observation contract, so
        raw encoder values, not those percentages, determine range validity.
        """
        missing = [name for name, _ in ARM_JOINTS if name not in raw]
        outside = [self._calibration_details(name, raw[name]) for name in self.calibration
                   if name in raw and not self.calibration[name]["range_min"] <= raw[name] <= self.calibration[name]["range_max"]]
        return {"valid": (not outside) if available and self.enabled and not missing else None,
                "out_of_range": outside, "missing_joints": missing}

    def _calibration_details(self, name, raw):
        cal, limit = self.calibration[name], self.limits()[name]
        return {"joint": name, "raw": raw, "raw_min": cal["range_min"], "raw_max": cal["range_max"],
                "current": round(raw_to_unit(raw, name, cal), 3),
                "min": limit["min"], "max": limit["max"], "unit": limit["unit"]}

    def limits(self) -> dict:
        return {name: {"min": 0 if name == "gripper" else raw_to_unit(cal["range_min"], name, cal),
                       "max": 100 if name == "gripper" else raw_to_unit(cal["range_max"], name, cal),
                       "max_speed": 50 if name == "gripper" else 30,
                       "unit": "%" if name == "gripper" else "deg"}
                for name, cal in self.calibration.items()}

    def valid_target(self, joints, duration_ms) -> bool:
        if not self.enabled or type(duration_ms) is not int or not 100 <= duration_ms <= ARM_MAX_DURATION_MS:
            return False
        if not isinstance(joints, dict) or not joints:
            return False
        limits = self.limits()
        return all(name in limits and type(value) in (int, float) and math.isfinite(value)
                   and limits[name]["min"] <= value <= limits[name]["max"]
                   for name, value in joints.items())

    def _read(self, motor_id, address, size=2):
        import scservo_sdk as sdk
        if size not in (1, 2):
            raise ValueError("Arm register reads must be one or two bytes")
        # The SDK read helpers slice without validating the full response length;
        # a short packet can even put its checksum in that data slice. Build the
        # same READ request, but validate the entire status packet before decode.
        request = [0, 0, motor_id, 4, sdk.INST_READ, address, size, 0]
        packet, comm, error = self.bus.packet_handler.txRxPacket(self.bus.port_handler, request)
        if comm != sdk.COMM_SUCCESS or error:
            raise ArmTransportError("read", motor_id, address, comm, error)
        if (not isinstance(packet, (list, tuple, bytes, bytearray)) or len(packet) != size + 6
                or any(type(value) is not int or not 0 <= value <= 255 for value in packet)
                or list(packet[:5]) != [255, 255, motor_id, size + 2, 0]
                or packet[-1] != (~sum(packet[2:-1]) & 255)):
            raise OSError(f"Arm {motor_id} read {address}: invalid response payload")
        data = packet[5:-1]
        return data[0] if size == 1 else sdk.SCS_MAKEWORD(data[0], data[1])

    def _write(self, motor_id, address, value, size=2):
        import scservo_sdk as sdk
        write = self.bus.packet_handler.write1ByteTxRx if size == 1 else self.bus.packet_handler.write2ByteTxRx
        comm, error = write(self.bus.port_handler, motor_id, address, value)
        if comm != sdk.COMM_SUCCESS or error:
            raise ArmTransportError("write", motor_id, address, comm, error)

    def _write_motion_profile(self, motor_id, address, value):
        """Verify an ambiguous profile write with reads only; never resend it.

        This is only for time/speed setup (44/46), before base motion or arm
        interpolation. Goal/torque writes must keep their uncertain outcome.
        All arm/wheel operations remain on the runtime's one serial caller;
        never force-clear SDK ownership, reopen the bus, or flush its input.
        """
        import scservo_sdk as sdk
        if address not in (44, 46) or type(value) is not int or not 0 <= value <= 65535:
            raise ValueError("Only two-byte time/speed profile settings can be verified")
        try:
            self._write(motor_id, address, value)
            return None
        except ArmTransportError as failure:
            if (failure.operation != "write" or failure.motor_id != motor_id or failure.register != address
                    or failure.communication not in (sdk.COMM_RX_TIMEOUT, sdk.COMM_RX_CORRUPT)
                    or failure.device_error != 0):
                raise
            for attempt in (1, 2):
                try:
                    actual = self._read(motor_id, address)
                except ArmTransportError as read_failure:
                    if (attempt == 2 or read_failure.operation != "read" or read_failure.motor_id != motor_id
                            or read_failure.register != address or read_failure.device_error != 0
                            or read_failure.communication not in (sdk.COMM_RX_TIMEOUT, sdk.COMM_RX_CORRUPT)):
                        raise
                    continue
                if type(actual) is not int or actual != value:
                    raise OSError(f"Arm {motor_id} profile {address} readback mismatch: "
                                  f"requested={value}, read={actual}") from failure
                recovery = {"motor_id": motor_id, "register": address, "requested": value,
                            "read_value": actual, "read_attempts": attempt,
                            "communication": failure.communication, "recovery": "verified_readback"}
                print(f"Arm profile write verified by readback: {recovery}", file=sys.stderr, flush=True)
                return recovery

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
            raise PreDispatchRejected("invalid_arm_targets", "Invalid arm targets or duration")
        if base_motion is not None and duration_ms > BASE_MAX_DURATION_MS:
            raise PreDispatchRejected("base_duration_limit", "Combined base and arm motion is limited to 1000 ms")
        self.last_motion = None
        communication_recoveries = []
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
                raise PreDispatchRejected("arm_current_out_of_calibration",
                                          f"Current {name} is outside calibration",
                                          **self._calibration_details(name, current))
            if abs(target - before["joints"][name]) > limits[name]["max_speed"] * seconds:
                raise PreDispatchRejected("arm_speed_limit",
                                          f"Requested {name} motion exceeds speed limit {limits[name]['max_speed']} {limits[name]['unit']}/s",
                                          joint=name, current=before["joints"][name], target=target,
                                          max_speed=limits[name]["max_speed"], duration_ms=duration_ms,
                                          unit=limits[name]["unit"])
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
                    raise PreDispatchRejected("arm_position_mode_required",
                                              f"Arm {mid} must already be in position mode", joint=name, motor_id=mid)
                if self._read(mid, 63, 1) >= 65:
                    raise PreDispatchRejected("arm_temperature_limit",
                                              f"Arm {mid} temperature exceeds limit", joint=name, motor_id=mid)
                offset = self._read(mid, 31)
                offset = -(offset & 0x7ff) if offset & 0x800 else offset
                if offset != self.calibration[name].get("homing_offset"):
                    raise PreDispatchRejected("arm_homing_offset_mismatch",
                                              f"Arm {mid} homing offset does not match calibration", joint=name, motor_id=mid)
            try:
                for name in joints:
                    mid = ids[name]
                    self._pending.add(mid)
                    # Clear a stale goal before torque on. Limit native servo speed too.
                    self._write(mid, 42, before["raw"][name])
                    speed = (self.calibration[name]["range_max"] - self.calibration[name]["range_min"]) / 2 if name == "gripper" else 30 * 4095 / 360
                    for address, value in ((44, 0), (46, max(1, round(speed)))):
                        recovery = self._write_motion_profile(mid, address, value)
                        if recovery is not None:
                            communication_recoveries.append(recovery)
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
                            "tolerance_deg": 2, "tolerance_percent": 5, "timestamp": time.time(),
                            "communication_recoveries": communication_recoveries}
        print("Arm target reached; holding position", file=sys.stderr, flush=True)
