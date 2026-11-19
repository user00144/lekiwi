"""STS3215 calibrated joint feedback and bounded position trajectories."""

from contextlib import nullcontext
from copy import deepcopy
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
P_COEFFICIENT = 21
EEPROM_LOCK = 55
# Position P gain is set on the five arm joints only; the gripper keeps its own gain.
GAIN_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ARM_FEEDBACK_TIMEOUT_S = 2.0
ARM_CONTROL_REVISION = 'arm-gain-v8'  # v8: post-miss hold accepts a stable load sag up to the warning limit
ARM_WARNING_MAX_ERROR_DEG = 10.0
HOLD_STABILITY_INTERVAL_S = .2
HOLD_STABILITY_SAMPLES = 3
HOLD_STABILITY_TIMEOUT_S = 2.0
# In-motion monitor (opt-in per arm-only command). The 20 ms interpolation loop reads the
# gripper every 3 steps (60 ms) and the commanded joints every 6 steps (120 ms, telemetry).
MONITOR_FIELDS = frozenset({'gripper_min_percent'})
MONITOR_CONDITIONS = frozenset({'gripper_below_min'})
MONITOR_GRIPPER_EVERY_STEPS = 3
MONITOR_TELEMETRY_EVERY_STEPS = 6
MONITOR_CONSECUTIVE = 2  # consecutive low gripper samples before a stop (one noisy read never stops)


def monitor_problem(monitor, joints, base_motion=None):
    """Why an in-motion monitor cannot be used with this command (None when valid)."""
    if not isinstance(monitor, dict) or set(monitor) - MONITOR_FIELDS:
        return 'unknown_monitor_fields'
    if base_motion is not None or not isinstance(joints, dict) or not joints or 'gripper' in joints:
        return 'monitor_requires_arm_only_motion_without_gripper_target'
    threshold = monitor.get('gripper_min_percent')
    if threshold is not None and (type(threshold) not in (int, float) or not math.isfinite(threshold)
                                  or not 0 < threshold < 100):
        return 'invalid_gripper_min_percent'
    return None


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


class ArmTargetNotReached(OSError):
    """Endpoint feedback captured before an error handler changes the goal."""
    def __init__(self, feedback):
        self.feedback = deepcopy(feedback)
        details = []
        for name in feedback['failed_joints']:
            unit = feedback['units'][name]
            details.append(f"{name}: target={feedback['targets'][name]:.3f} {unit}, "
                           f"measured={feedback['measured'][name]:.3f} {unit}, "
                           f"error={feedback['errors'][name]:+.3f} {unit}, "
                           f"tolerance={feedback['tolerances'][name]:g} {unit}")
        super().__init__("Arm target not reached within feedback tolerance: " + "; ".join(details))


class ArmMonitorStopped(Exception):
    """A requested in-motion condition broke: the arm was held where it was and the hold verified.

    This is a known, measured stop (never target arrival). Any failure to hold or to verify the
    hold raises an ordinary error instead, keeping its uncertain outcome.
    """
    def __init__(self, feedback):
        self.feedback = deepcopy(feedback)
        stop = feedback['monitor_stop']
        super().__init__(f"Arm stopped by monitor {stop['condition']} at {stop['progress_fraction']:.0%} of the "
                         f"motion: {stop}")


class _MonitorTrip(Exception):
    """Internal: leaves the interpolation loop so the shared handler holds the arm."""


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
        gain = config.get("p_coefficient")
        if gain is not None and (type(gain) is not int or not 1 <= gain <= 254):
            raise ValueError("arm.p_coefficient must be an integer from 1 to 254, or null")
        self.p_coefficient = gain
        self.gain_status = {"configured": gain, "joints": {}, "error": None, "applied_at": None}
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
            self._apply_gains()
        except Exception as exc:
            self.error = str(exc)

    def _apply_gains(self) -> None:
        """Set the configured position P gain and verify it by readback.

        Only register 21 is written; torque, goals and the EEPROM lock are left
        unchanged. A failure is reported and later arm moves are rejected before
        dispatch, so a stale gain never silently drives the arm.
        """
        if self.p_coefficient is None:
            return
        ids, joints, errors = dict(ARM_JOINTS), {}, []
        for name in GAIN_JOINTS:
            try:
                before = self._read(ids[name], P_COEFFICIENT, 1)
                if before != self.p_coefficient:
                    self._write(ids[name], P_COEFFICIENT, self.p_coefficient, 1)
                after = self._read(ids[name], P_COEFFICIENT, 1)
                joints[name] = {"before": before, "after": after,
                                "eeprom_lock": self._read(ids[name], EEPROM_LOCK, 1)}
                if after != self.p_coefficient:
                    errors.append(f"{name}: P readback {after} != {self.p_coefficient}")
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        self.gain_status = {"configured": self.p_coefficient, "joints": joints,
                            "error": "; ".join(errors) or None, "applied_at": time.time()}
        print(f"Arm P gain status: {self.gain_status}", file=sys.stderr, flush=True)

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
                  "control": {"revision": ARM_CONTROL_REVISION, "max_duration_ms": ARM_MAX_DURATION_MS,
                              "feedback_timeout_s": ARM_FEEDBACK_TIMEOUT_S, "tolerance_deg": 2,
                              "tolerance_percent": 5, "warning_max_error_deg": ARM_WARNING_MAX_ERROR_DEG,
                              "hold_stability_timeout_s": HOLD_STABILITY_TIMEOUT_S,
                              "p_coefficient": deepcopy(self.gain_status)},
                  "calibration_status": self.calibration_status({}, available=False),
                  "communication_retries": []}
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
                        result['communication_retries'].append({
                            'joint': name, 'register': PRESENT_POSITION,
                            'communication': comm, 'device_error': error})
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
        errors, held_goals = [], {}
        names = {motor_id: name for name, motor_id in ARM_JOINTS}
        for motor_id in sorted(self._pending):
            try:
                measured = self._read(motor_id, PRESENT_POSITION)
                self._write(motor_id, 42, measured)
                held_goals[names[motor_id]] = measured
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise OSError("Arm hold failed; physical state unknown: " + "; ".join(errors))
        if self._pending:
            print("Arm holding measured position", file=sys.stderr, flush=True)
        self._pending.clear()
        return held_goals

    def verify_hold(self, held_goals: dict, *, certificate=None, passive_reference=None, goal_tolerance_deg=2.):
        """Read-only certificate for a calibrated, healthy and stable measured hold.

        The initial hold writes only the interrupted command's pending joints.
        This verification reads all six joints without enabling any idle motor
        or changing a goal. An uncommanded, unpowered gripper is checked as
        passive feedback, not as a powered position hold.

        ``goal_tolerance_deg``: allowed arm-joint distance from the written hold goal. A hold written
        after a missed target under load (e.g. a held cube) settles again below its new goal by about
        the same gravity sag, so that path passes up to ARM_WARNING_MAX_ERROR_DEG; stability (drift
        <= 0.5 deg over >= 0.2 s), health, calibration and the 2 deg reference check are unchanged.
        A re-check of a certificate uses the tolerance recorded in it.
        """
        names = {name for name, _ in ARM_JOINTS}
        if certificate is not None and isinstance(certificate, dict):
            goal_tolerance_deg = certificate.get('goal_tolerance_deg', 2.)
        if (type(goal_tolerance_deg) not in (int, float) or not math.isfinite(goal_tolerance_deg)
                or not 2. <= goal_tolerance_deg <= ARM_WARNING_MAX_ERROR_DEG):
            raise OSError('Invalid arm hold goal tolerance')
        goal_tolerance_deg = float(goal_tolerance_deg)
        if self.mock or not self.enabled or self.bus.port_handler is None:
            raise OSError('Arm hold requires acknowledged servo feedback')
        if (not isinstance(held_goals, dict) or not held_goals or not set(held_goals) <= names
                or any(type(raw) is not int or not 0 <= raw <= 4095 for raw in held_goals.values())):
            raise OSError('Invalid measured arm hold goals')
        commanded = set(held_goals)
        expected_passive = None
        passive_anchor = {}
        reference = None
        if certificate is not None:
            if (not isinstance(certificate, dict) or certificate.get('verified') is not True
                    or certificate.get('revision') != ARM_CONTROL_REVISION
                    or certificate.get('calibration_id') != self.calibration_id
                    or set(held_goals) != names or held_goals != certificate.get('held_goal_raw')):
                raise OSError('Arm hold certificate or calibration changed')
            reference = certificate.get('measured')
            if (not isinstance(reference, dict) or set(reference) != names
                    or any(type(value) not in (int, float) or not math.isfinite(value) for value in reference.values())):
                raise OSError('Arm hold certificate has invalid measured positions')
            commanded_list, passive_list = certificate.get('commanded_joints'), certificate.get('passive_joints')
            if (not isinstance(commanded_list, list) or not commanded_list
                    or any(not isinstance(name, str) or name not in names for name in commanded_list)
                    or len(set(commanded_list)) != len(commanded_list)
                    or not isinstance(passive_list, list) or passive_list not in ([], ['gripper'])):
                raise OSError('Arm hold certificate has invalid command scope')
            commanded, expected_passive = set(commanded_list), set(passive_list)
            passive_anchor = certificate.get('passive_reference')
            if (commanded & expected_passive or not isinstance(passive_anchor, dict)
                    or set(passive_anchor) != expected_passive
                    or any(type(value) not in (int, float) or not math.isfinite(value)
                           or not 0 <= value <= 100 for value in passive_anchor.values())):
                raise OSError('Arm hold certificate has invalid passive reference')
            passive_anchor = dict(passive_anchor)
        expected = dict(held_goals)
        samples, stamps, health = [], [], {}
        verification_started = time.monotonic()
        deadline = verification_started + HOLD_STABILITY_TIMEOUT_S
        drift_limits = {name: 2. if name == 'gripper' else .5 for name in names}
        while True:
            if stamps:
                # The interval is measured between completed samples, not assumed
                # from servo-read latency. Allow a newly written hold to settle,
                # but never repeat a write or relax the stability criterion.
                due = stamps[-1] + HOLD_STABILITY_INTERVAL_S / (HOLD_STABILITY_SAMPLES - 1)
                time.sleep(max(0., min(due, deadline) - time.monotonic()))
                if time.monotonic() >= deadline:
                    raise OSError('Arm hold position is not stable within settling timeout')
            measured_raw, passive = {}, set()
            for name, motor_id in ARM_JOINTS:
                if time.monotonic() >= deadline:
                    raise OSError('Arm hold position is not stable within settling timeout')
                goal = self._read(motor_id, 42)
                if name not in expected:
                    expected[name] = goal
                if type(goal) is not int or not 0 <= goal <= 4095 or goal != expected[name]:
                    raise OSError(f'{name}: arm hold goal readback changed')
                torque, mode = self._read(motor_id, 40, 1), self._read(motor_id, 33, 1)
                status, temperature = self._read(motor_id, 65, 1), self._read(motor_id, 63, 1)
                encoded_offset = self._read(motor_id, 31)
                offset = -(encoded_offset & 0x7ff) if encoded_offset & 0x800 else encoded_offset
                is_passive = name == 'gripper' and name not in commanded and torque == 0
                if is_passive:
                    passive.add(name)
                if (torque != 1 and not is_passive) or mode != 0 or status != 0 or temperature >= 65:
                    raise OSError(f'{name}: arm hold health not verified '
                                  f'(torque={torque}, mode={mode}, status={status}, temperature={temperature})')
                if offset != self.calibration[name].get('homing_offset'):
                    raise OSError(f'{name}: arm hold homing offset changed')
                if not is_passive and not self.calibration[name]['range_min'] <= goal <= self.calibration[name]['range_max']:
                    raise OSError(f'{name}: arm hold goal outside calibration')
                health[name] = {'torque_enabled': torque == 1, 'passive': is_passive, 'mode': mode, 'status': status,
                                'temperature_c': temperature, 'homing_offset': offset}
                measured_raw[name] = self._read(motor_id, PRESENT_POSITION)
            if expected_passive is None:
                expected_passive = passive
                if passive:
                    anchor = passive_reference.get('gripper') if isinstance(passive_reference, dict) else None
                    if type(anchor) not in (int, float) or not math.isfinite(anchor) or not 0 <= anchor <= 100:
                        raise OSError('Passive gripper requires its pre-motion measured position')
                    passive_anchor['gripper'] = anchor
            elif passive != expected_passive:
                raise OSError('Arm hold torque state changed during verification')
            if self.calibration_status(measured_raw).get('valid') is not True:
                raise OSError('Arm hold encoder feedback outside calibration')
            measured = {name: raw_to_unit(raw, name, self.calibration[name]) for name, raw in measured_raw.items()}
            goals = {name: raw_to_unit(raw, name, self.calibration[name]) for name, raw in expected.items()}
            for name in names:
                tolerance = 5. if name == 'gripper' else 2.
                goal_tolerance = 5. if name == 'gripper' else goal_tolerance_deg
                if name not in passive and abs(measured[name] - goals[name]) > goal_tolerance:
                    raise OSError(f'{name}: arm hold did not reach measured goal '
                                  f'(error {measured[name] - goals[name]:+.2f}, tolerance {goal_tolerance:g})')
                if name in passive and abs(measured[name] - passive_anchor[name]) > 2.:
                    raise OSError('Passive gripper moved from its pre-motion measured position')
                if reference is not None and abs(measured[name] - reference[name]) > tolerance:
                    raise OSError(f'{name}: arm moved away from hold certificate')
            samples.append(measured)
            stamps.append(time.monotonic())
            if stamps[-1] > deadline:
                raise OSError('Arm hold position is not stable within settling timeout')
            samples, stamps = samples[-HOLD_STABILITY_SAMPLES:], stamps[-HOLD_STABILITY_SAMPLES:]
            interval = stamps[-1] - stamps[0]
            if len(samples) == HOLD_STABILITY_SAMPLES and interval + 1e-9 >= HOLD_STABILITY_INTERVAL_S:
                drift = {name: max(sample[name] for sample in samples) - min(sample[name] for sample in samples)
                         for name in names}
                if all(drift[name] <= drift_limits[name] for name in names):
                    break
        return {'verified': True, 'revision': ARM_CONTROL_REVISION, 'calibration_id': self.calibration_id,
                'commanded_joints': sorted(commanded), 'passive_joints': sorted(expected_passive),
                'passive_reference': passive_anchor,
                'timestamp': time.time(), 'held_goal_raw': expected, 'goals': goals,
                'measured_raw': measured_raw, 'measured': samples[-1], 'health': health,
                'stability': {'sample_count': len(samples), 'interval_s': interval,
                              'max_drift': drift, 'limits': drift_limits,
                              'settling_elapsed_s': stamps[-1] - verification_started,
                              'settling_timeout_s': HOLD_STABILITY_TIMEOUT_S},
                'tolerances': {name: 5. if name == 'gripper' else 2. for name in names},
                'goal_tolerance_deg': goal_tolerance_deg}

    def reconcile_hold(self, certificate):
        """Recheck an existing hold using reads only; never replay a movement."""
        goals = certificate.get('held_goal_raw') if isinstance(certificate, dict) else None
        return self.verify_hold(goals, certificate=certificate)

    def _warning_eligibility(self, feedback, before, base_motion):
        """A small acknowledged endpoint miss may continue after a verified hold."""
        if 'gripper' in feedback['targets']:
            return False, 'gripper_command_not_warning_eligible'
        if base_motion is not None:
            return False, 'base_motion_not_warning_eligible'
        if before.get('communication_retries') != [] or feedback['communication_recoveries']:
            return False, 'communication_not_fully_acknowledged'
        if self.calibration_status(before['raw']).get('valid') is not True:
            return False, 'initial_calibration_invalid'
        if any(not self.calibration[name]['range_min'] <= feedback['measured_raw'][name]
                   <= self.calibration[name]['range_max'] for name in feedback['targets']):
            return False, 'endpoint_calibration_invalid'
        if (not feedback['failed_joints'] or not feedback['errors']
                or any(not math.isfinite(error) or abs(error) > ARM_WARNING_MAX_ERROR_DEG
                       for error in feedback['errors'].values())):
            return False, 'endpoint_error_exceeds_warning_limit'
        return True, 'hold_verification_required'

    def _recovery_eligibility(self, feedback, before, base_motion):
        """Return conservative endpoint-only recovery evidence before hold checks."""
        if 'gripper' in feedback['targets']:
            return False, 'gripper_command_not_recoverable', {}
        if base_motion is not None:
            return False, 'base_motion_not_recoverable', {}
        if (before.get('communication_retries') != [] or feedback['communication_recoveries']):
            return False, 'communication_not_fully_acknowledged', {}
        if self.calibration_status(before['raw']).get('valid') is not True:
            return False, 'initial_calibration_invalid', {}
        progress = {}
        for name in feedback['failed_joints']:
            initial = before['joints'][name]
            requested_delta = feedback['targets'][name] - initial
            remaining_error = abs(feedback['errors'][name])
            reduction = abs(requested_delta) - remaining_error
            required = max(1., .2 * abs(requested_delta))
            before_target = requested_delta * (feedback['targets'][name] - feedback['measured'][name]) > 0
            progress[name] = {'initial': initial, 'requested_delta': requested_delta,
                              'error_reduction': reduction, 'required_progress': required,
                              'remaining_error': remaining_error, 'before_target': before_target,
                              'eligible': before_target and reduction >= required}
        if not progress or not all(item['eligible'] for item in progress.values()):
            return False, 'insufficient_joint_progress', progress
        return True, 'hold_verification_required', progress

    def _monitor_sample(self, watch, step, steps, started, goals, before):
        """Read-only checks between interpolation writes; a read failure never stops the arm.

        A gripper stop needs MONITOR_CONSECUTIVE successive low readings; a failed read resets
        the count. Following error (commanded interpolation minus measured) is telemetry only.
        """
        ids = dict(ARM_JOINTS)
        threshold = watch['gripper_min_percent']
        if threshold is not None and step % MONITOR_GRIPPER_EVERY_STEPS == 0:
            try:
                raw = self._read(ids['gripper'], PRESENT_POSITION)
            except OSError:
                watch['read_errors'] += 1
                watch['low'] = 0
            else:
                percent = raw_to_unit(raw, 'gripper', self.calibration['gripper'])
                watch['gripper_samples'] += 1
                seen = watch['gripper_min_seen']
                watch['gripper_min_seen'] = round(percent if seen is None else min(seen, percent), 3)
                watch['low'] = watch['low'] + 1 if percent < threshold else 0
                if watch['low'] >= MONITOR_CONSECUTIVE:
                    watch['stop'] = {'condition': 'gripper_below_min', 'gripper_percent': round(percent, 3),
                                     'gripper_min_percent': threshold, 'consecutive_samples': watch['low'],
                                     'elapsed_s': round(time.monotonic() - started, 3), 'step': step,
                                     'steps': steps, 'progress_fraction': round(step / steps, 3)}
                    raise _MonitorTrip()
        if step % MONITOR_TELEMETRY_EVERY_STEPS == 0:
            lags = watch['max_following_error_deg']
            for name, goal in goals.items():
                try:
                    measured = self._read(ids[name], PRESENT_POSITION)
                except OSError:
                    watch['read_errors'] += 1
                    continue
                initial = before['raw'][name]
                lag = abs(measured - (initial + (goal - initial) * step / steps)) * 360 / 4095
                lags[name] = round(max(lags.get(name, 0.), lag), 2)
            watch['telemetry_samples'] += 1

    @staticmethod
    def _monitor_summary(watch):
        return {key: deepcopy(watch[key]) for key in (
            'gripper_min_percent', 'gripper_samples', 'gripper_min_seen', 'telemetry_samples',
            'max_following_error_deg', 'read_errors')}

    def move(self, joints: dict, duration_ms: int, base_motion=None, monitor=None):
        if not self.valid_target(joints, duration_ms):
            raise PreDispatchRejected("invalid_arm_targets", "Invalid arm targets or duration")
        if base_motion is not None and duration_ms > BASE_MAX_DURATION_MS:
            raise PreDispatchRejected("base_duration_limit", "Combined base and arm motion is limited to 1000 ms")
        if monitor is not None:
            problem = monitor_problem(monitor, joints, base_motion)
            if problem:
                raise PreDispatchRejected("invalid_arm_monitor", f"Invalid in-motion monitor: {problem}")
        watch = None if monitor is None else {
            'gripper_min_percent': monitor.get('gripper_min_percent'), 'gripper_samples': 0,
            'gripper_min_seen': None, 'telemetry_samples': 0, 'max_following_error_deg': {},
            'read_errors': 0, 'low': 0, 'stop': None}
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
                if name in GAIN_JOINTS and self.p_coefficient is not None:
                    gain = self._read(mid, P_COEFFICIENT, 1)
                    if gain != self.p_coefficient:
                        raise PreDispatchRejected("arm_gain_mismatch",
                                                  f"Arm {mid} P gain {gain} is not the configured {self.p_coefficient}",
                                                  joint=name, motor_id=mid, measured=gain, configured=self.p_coefficient)
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
                        if watch is not None and step < steps:
                            self._monitor_sample(watch, step, steps, started, goals, before)
                        time.sleep(max(0, started + step * seconds / steps - time.monotonic()))
                settling_started = time.monotonic()
                deadline = settling_started + ARM_FEEDBACK_TIMEOUT_S
                while True:
                    measured_raw = {name: self._read(ids[name], PRESENT_POSITION) for name in joints}
                    measured = {name: raw_to_unit(raw, name, self.calibration[name])
                                for name, raw in measured_raw.items()}
                    if all(abs(measured[name] - target) <= (5 if name == "gripper" else 2)
                           for name, target in joints.items()):
                        break
                    sampled_at = time.monotonic()
                    if sampled_at >= deadline:
                        # Preserve the endpoint reading before hold() replaces
                        # the goal with another measured position. This is a
                        # failed motion record, never a completion certificate.
                        errors = {name: measured[name] - target for name, target in joints.items()}
                        tolerances = {name: 5 if name == "gripper" else 2 for name in joints}
                        self.last_motion = {
                            'targets': dict(joints), 'target_raw': dict(goals),
                            'initial_joints': deepcopy(before['joints']), 'initial_raw': deepcopy(before['raw']),
                            'measured': dict(measured), 'measured_raw': dict(measured_raw),
                            'errors': errors, 'tolerances': tolerances,
                            'units': {name: '%' if name == 'gripper' else 'deg' for name in joints},
                            'failed_joints': [name for name in joints if abs(errors[name]) > tolerances[name]],
                            'reached': False, 'outcome': 'target_not_reached', 'measurement_phase': 'before_hold',
                            'duration_ms': duration_ms, 'interpolation_elapsed_s': settling_started - started,
                            'settling_elapsed_s': sampled_at - settling_started,
                            'feedback_timeout_s': ARM_FEEDBACK_TIMEOUT_S,
                            'tolerance_deg': 2, 'tolerance_percent': 5, 'timestamp': time.time(),
                            'communication_recoveries': deepcopy(communication_recoveries),
                            'recovery_eligible': False, 'recovery_reason': 'hold_not_verified', 'hold': None,
                            'warning_eligible': False, 'warning_reason': 'hold_not_verified',
                            'warning_max_error_deg': ARM_WARNING_MAX_ERROR_DEG,
                        }
                        raise ArmTargetNotReached(self.last_motion)
                    # Retain the final goal while checking feedback. No goal,
                    # profile or torque writes are repeated during settling.
                    time.sleep(min(.05, max(0, deadline - sampled_at)))
            except BaseException as failure:
                try:
                    held_goals = self.hold()
                except BaseException as hold_failure:
                    if type(failure) is ArmTargetNotReached:
                        self.last_motion.update(recovery_eligible=False, recovery_reason='hold_failed',
                                                warning_eligible=False, warning_reason='hold_failed',
                                                hold_error=str(hold_failure))
                        failure.feedback = deepcopy(self.last_motion)
                    raise
                if type(failure) is _MonitorTrip:
                    # Requested stop: goals now hold the measured pose. Only a verified hold is a
                    # known outcome; anything else keeps the ordinary uncertain failure.
                    self.last_motion = {
                        'targets': dict(joints), 'target_raw': dict(goals),
                        'initial_joints': deepcopy(before['joints']), 'initial_raw': deepcopy(before['raw']),
                        'held_goal_raw': dict(held_goals), 'reached': False, 'outcome': 'monitor_stop',
                        'monitor_stop': deepcopy(watch['stop']), 'monitor': self._monitor_summary(watch),
                        'duration_ms': duration_ms, 'interpolation_elapsed_s': time.monotonic() - started,
                        'communication_recoveries': deepcopy(communication_recoveries),
                        'timestamp': time.time(), 'hold': None,
                    }
                    try:
                        certificate = self.verify_hold(held_goals, passive_reference=before['joints'],
                                            goal_tolerance_deg=ARM_WARNING_MAX_ERROR_DEG)
                    except Exception as verification_failure:
                        self.last_motion['hold_error'] = str(verification_failure)
                        raise OSError('Arm monitor stop hold not verified; physical state unknown: '
                                      f'{verification_failure}') from verification_failure
                    self.last_motion.update(hold=certificate,
                                            measured={name: certificate['measured'][name] for name in joints})
                    print(f"Arm stopped by monitor: {watch['stop']}", file=sys.stderr, flush=True)
                    raise ArmMonitorStopped(self.last_motion) from None
                if type(failure) is ArmTargetNotReached:
                    eligible, reason, progress = self._recovery_eligibility(self.last_motion, before, base_motion)
                    warning_eligible, warning_reason = self._warning_eligibility(self.last_motion, before, base_motion)
                    self.last_motion.update(recovery_reason=reason, progress=progress, warning_reason=warning_reason)
                    if eligible or warning_eligible:
                        try:
                            certificate = self.verify_hold(held_goals, passive_reference=before['joints'],
                                            goal_tolerance_deg=ARM_WARNING_MAX_ERROR_DEG)
                        except Exception as verification_failure:
                            self.last_motion.update(recovery_reason='hold_verification_failed',
                                                    warning_reason='hold_verification_failed',
                                                    hold_error=str(verification_failure))
                        else:
                            hold_target_errors = {
                                name: certificate['measured'][name] - target
                                for name, target in self.last_motion['targets'].items()
                            }
                            if warning_eligible and any(
                                    not math.isfinite(error) or abs(error) > ARM_WARNING_MAX_ERROR_DEG
                                    for error in hold_target_errors.values()):
                                warning_eligible = False
                                warning_reason = 'held_position_error_exceeds_warning_limit'
                            self.last_motion.update(recovery_eligible=eligible,
                                                    recovery_reason='verified_measured_hold' if eligible else reason,
                                                    warning_eligible=warning_eligible,
                                                    warning_reason='verified_measured_hold' if warning_eligible else warning_reason,
                                                    hold_target_errors=hold_target_errors,
                                                    hold=certificate)
                    failure.feedback = deepcopy(self.last_motion)
                raise
            self._pending.clear()  # Successful trajectory keeps holding its target.
        measured = {name: raw_to_unit(self._mock_raw[name], name, self.calibration[name])
                    for name in joints} if self.mock else measured
        self.last_motion = {"targets": dict(joints), "measured": measured, "reached": True,
                            "tolerance_deg": 2, "tolerance_percent": 5, "timestamp": time.time(),
                            "communication_recoveries": communication_recoveries}
        if watch is not None:
            self.last_motion["monitor"] = ({"simulated": True} if self.mock else
                                           {**self._monitor_summary(watch),
                                            "interpolation_elapsed_s": round(settling_started - started, 3),
                                            "planned_s": seconds})
        print("Arm target reached; holding position", file=sys.stderr, flush=True)
