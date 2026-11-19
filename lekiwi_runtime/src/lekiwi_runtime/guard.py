"""Opt-in observation leases and at-most-once execution for one runtime process.

This is an execution/observation contract, not a collision or localization system.
All clocks and sensor timestamps are checked on the robot host.
"""

from copy import deepcopy
import json
import math
import time
from uuid import uuid4

from .drivers.arm import MONITOR_CONDITIONS, monitor_problem
from .robot import ARM_MAX_DURATION_MS, BASE_MAX_DURATION_MS, PreDispatchRejected


GUARD_FIELDS = frozenset({"observation_id", "runtime_id", "action_id", "reason"})
CAPABILITIES = {"guarded_execution": 1, "arm_reconciliation": 1, "arm_motion_warning": 1,
                "arm_motion_monitor": 1}
MAX_ARM_RECONCILIATIONS = 2
RECOVERY_MAX_JOINT_DELTA_DEG = 40.0
RECOVERY_MIN_DURATION_MS = 5000
ARM_WARNING_MAX_ERROR_DEG = 10.0
MAX_OBSERVATION_BYTES = 2 * 1024 * 1024
JOINT_NAMES = frozenset({"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"})


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _fresh(sensor, max_age_s, now):
    if not isinstance(sensor, dict) or sensor.get("available") is not True or sensor.get("error"):
        return False
    stamp = sensor.get("timestamp")
    if not _number(stamp) or not -.1 <= now - stamp <= max_age_s:
        return False
    age = sensor.get("age_s")
    return age is None or (_number(age) and 0 <= age <= max_age_s)


class ExecutionGuard:
    def __init__(self, robot, *, max_age_s=60.0, post_timeout_s=2.0,
                 max_actions=128, max_cache_bytes=32 * 1024 * 1024):
        self.robot = robot
        self.runtime_id = str(uuid4())
        self.max_age_s = max_age_s
        self.post_timeout_s = post_timeout_s
        self.max_actions = max_actions
        self.max_cache_bytes = max_cache_bytes
        self._baseline = None
        self._cache = {}
        self._cache_bytes = 0
        self._halt_reason = None
        self._arm_failure = None
        self._recovery_constraints = None
        self._reconciliations = 0
        self._preflight_joints = None

    def invalidate(self):
        self._baseline = None

    def halt(self, reason="uncertain_execution"):
        self.invalidate()
        self._halt_reason = reason
        if reason != "arm_reconciliation_required":
            self._arm_failure = None

    @property
    def legacy_block_reason(self):
        return self._halt_reason or ("arm_recovery_constrained" if self._recovery_constraints else None)

    def _remaining_reconciliations(self):
        return max(0, MAX_ARM_RECONCILIATIONS - self._reconciliations)

    def _recovery(self):
        failure = self._arm_failure
        return {"eligible": bool(failure and self._halt_reason == "arm_reconciliation_required"
                                 and self._remaining_reconciliations()
                                 and time.monotonic() - failure["created"] <= self.max_age_s),
                "failed_action_id": failure["command"]["action_id"] if failure else None,
                "runtime_id": self.runtime_id,
                "attempts_remaining": self._remaining_reconciliations()}

    def _context(self, reason=None):
        baseline = self._baseline
        reason = self._halt_reason or reason
        if not reason and baseline is None:
            reason = "observe_required"
        if not reason and time.monotonic() - baseline["created"] > self.max_age_s:
            reason = "observation_expired"
        if not reason and not baseline["valid_cameras"]:
            reason = "no_fresh_camera"
        if not reason and len(self._cache) >= self.max_actions:
            reason = "action_cache_full"
        context = {
            "runtime_id": self.runtime_id,
            "observation_id": baseline["id"] if baseline else None,
            "movement_allowed": reason is None,
            "max_age_s": self.max_age_s,
        }
        if reason:
            context["reason"] = reason
        if self._recovery_constraints:
            context["recovery_constraints"] = {
                "max_joint_delta_deg": RECOVERY_MAX_JOINT_DELTA_DEG,
                "min_duration_ms": RECOVERY_MIN_DURATION_MS,
                "remaining": self._remaining_reconciliations(),
                "failed_action_id": self._recovery_constraints["action_id"],
                "runtime_id": self.runtime_id,
            }
        return context

    @staticmethod
    def _arm_only(command):
        joints = command.get("joints")
        return (command.get("action") == "move" and isinstance(joints, dict) and bool(joints)
                and set(joints) <= JOINT_NAMES - {"gripper"}
                and all(_number(value) for value in joints.values())
                and all(command.get(axis, 0) == 0 for axis in ("x", "y", "theta")))

    @staticmethod
    def _hold_problem(certificate):
        if not isinstance(certificate, dict) or certificate.get("verified") is not True:
            return "arm_hold_unverified"
        if not isinstance(certificate.get("calibration_id"), str) or not certificate["calibration_id"]:
            return "arm_hold_calibration_invalid"
        if not _number(certificate.get("timestamp")):
            return "arm_hold_timestamp_invalid"
        measured = certificate.get("measured")
        if (not isinstance(measured, dict) or set(measured) != JOINT_NAMES
                or not all(_number(value) for value in measured.values())):
            return "arm_hold_measurements_invalid"
        goals = certificate.get("held_goal_raw")
        if (not isinstance(goals, dict) or set(goals) != JOINT_NAMES
                or any(type(value) is not int for value in goals.values())):
            return "arm_hold_goals_invalid"
        return None

    def _record_incomplete(self, command, execution):
        motion = execution.get("arm_motion")
        hold = motion.get("hold") if isinstance(motion, dict) else None
        if (not self._arm_only(command) or execution.get("command_sent") is not True
                or execution.get("arm_reached") is not False
                or execution.get("arm_hold_verified") is not True
                or not isinstance(motion, dict) or motion.get("recovery_eligible") is not True
                or self._hold_problem(hold)
                or not callable(getattr(self.robot, "reconcile_arm_hold", None))):
            raise ValueError("Invalid incomplete arm execution result")
        if not -.1 <= time.time() - hold["timestamp"] <= 1.0:
            raise ValueError("Incomplete arm hold certificate is stale")
        # Retain the original command and certificate independently of the
        # immutable action journal, even when the caller modifies its response.
        self._arm_failure = {"command": deepcopy(command), "hold": deepcopy(hold),
                             "created": time.monotonic(), "finished_at": time.time()}
        self.halt("arm_reconciliation_required")
        execution["motion_finished_at"] = self._arm_failure["finished_at"]

    def _validate_warning(self, command, execution, calibration_id):
        """Accept a measured, bounded miss without claiming target arrival."""
        motion, warning = execution.get("arm_motion"), execution.get("warning")
        hold = motion.get("hold") if isinstance(motion, dict) else None
        bound = warning.get("max_error_deg") if isinstance(warning, dict) else None
        if (not self._arm_only(command) or execution.get("command_sent") is not True
                or execution.get("arm_reached") is not False
                or execution.get("arm_hold_verified") is not True
                or not isinstance(warning, dict) or warning.get("code") != "arm_target_not_reached"
                or not _number(bound) or not 0 < bound <= ARM_WARNING_MAX_ERROR_DEG
                or not isinstance(motion, dict) or motion.get("targets") != command["joints"]
                or motion.get("reached") is not False or motion.get("warning_eligible") is not True
                or motion.get("communication_recoveries") != [] or self._hold_problem(hold)
                or hold.get("calibration_id") != calibration_id
                or not -.1 <= time.time() - hold["timestamp"] <= 1.0):
            raise ValueError("Invalid arm motion warning evidence")
        measured, errors = motion.get("measured"), motion.get("errors")
        if (not isinstance(measured, dict) or set(measured) != set(command["joints"])
                or not isinstance(errors, dict) or set(errors) != set(command["joints"])):
            raise ValueError("Missing arm warning measurements")
        for name, target in command["joints"].items():
            if (not _number(measured[name]) or not _number(errors[name])
                    or abs(errors[name]) > bound
                    or abs(errors[name] - (measured[name] - target)) > 1e-6
                    or abs(hold["measured"][name] - target) > bound):
                raise ValueError("Arm warning exceeds its measured error bound")

    def _validate_stopped(self, command, execution, calibration_id):
        """Accept a requested in-motion monitor stop only with a fresh verified measured hold."""
        motion, stop = execution.get("arm_motion"), execution.get("monitor_stop")
        hold = motion.get("hold") if isinstance(motion, dict) else None
        monitor = command.get("monitor")
        if (not self._arm_only(command) or not isinstance(monitor, dict)
                or execution.get("command_sent") is not True or execution.get("arm_reached") is not False
                or execution.get("arm_hold_verified") is not True
                or not isinstance(motion, dict) or motion.get("targets") != command["joints"]
                or motion.get("reached") is not False or motion.get("outcome") != "monitor_stop"
                or not isinstance(stop, dict) or stop != motion.get("monitor_stop")
                or stop.get("condition") not in MONITOR_CONDITIONS
                or monitor.get("gripper_min_percent") is None
                or stop.get("gripper_min_percent") != monitor["gripper_min_percent"]
                or self._hold_problem(hold) or hold.get("calibration_id") != calibration_id
                or not -.1 <= time.time() - hold["timestamp"] <= 1.0):
            raise ValueError("Invalid arm monitor stop evidence")

    @staticmethod
    def _validate_stop_observation(response, execution):
        """After a monitor stop the fresh state must still match the verified hold."""
        ego = response["observation"].get("ego")
        hold = execution["arm_motion"]["hold"]
        joints = ego.get("joints") if isinstance(ego, dict) else None
        if (not _fresh(ego, 1., time.time())
                or ego["timestamp"] <= execution["motion_finished_at"]
                or ego.get("calibration_id") != hold["calibration_id"]
                or (ego.get("calibration_status") or {}).get("valid") is not True
                or not isinstance(joints, dict) or not JOINT_NAMES <= set(joints)
                or any(not _number(joints[name])
                       or abs(joints[name] - hold["measured"][name]) > (2. if name == "gripper" else .5)
                       for name in JOINT_NAMES)):
            raise ValueError("Arm state changed after monitor stop hold verification")

    @staticmethod
    def _validate_warning_observation(response, command, execution):
        observation = response["observation"]
        ego = observation.get("ego")
        hold = execution["arm_motion"]["hold"]
        joints = ego.get("joints") if isinstance(ego, dict) else None
        if (not _fresh(ego, 1., time.time())
                or ego["timestamp"] <= execution["motion_finished_at"]
                or ego.get("calibration_id") != hold["calibration_id"]
                or (ego.get("calibration_status") or {}).get("valid") is not True
                or not isinstance(joints, dict) or not JOINT_NAMES <= set(joints)
                or any(not _number(joints[name])
                       or abs(joints[name] - hold["measured"][name]) > (2. if name == "gripper" else .5)
                       for name in JOINT_NAMES)
                or any(abs(joints[name] - target) > execution["warning"]["max_error_deg"]
                       for name, target in command["joints"].items())):
            raise ValueError("Arm state changed after warning hold verification")

    def reconcile(self, request):
        """Recheck a known, verified hold without sending any motor command."""
        def reject(reason):
            return {"status": "rejected", "error": reason,
                    "capabilities": dict(CAPABILITIES), "action_context": self._context(),
                    "recovery": self._recovery()}

        if set(request) - {"op", "request_id", "runtime_id", "failed_action_id"}:
            return reject("unknown_reconcile_fields")
        if any(not isinstance(request.get(key), str) or not request[key].strip()
               or len(request[key]) > 128 for key in ("runtime_id", "failed_action_id")):
            return reject("reconcile_identity_required")
        if request["runtime_id"] != self.runtime_id:
            return reject("runtime_mismatch")
        failure = self._arm_failure
        if self._halt_reason != "arm_reconciliation_required" or failure is None:
            return reject("arm_reconciliation_not_available")
        if request["failed_action_id"] != failure["command"]["action_id"]:
            return reject("failed_action_mismatch")
        if time.monotonic() - failure["created"] > self.max_age_s:
            return reject("arm_reconciliation_expired")
        if not self._remaining_reconciliations():
            return reject("arm_reconciliation_limit")
        if len(self._cache) >= self.max_actions or self._cache_bytes >= self.max_cache_bytes:
            self.halt("action_cache_full")
            return reject("action_cache_full")

        self.invalidate()
        self._reconciliations += 1
        try:
            certificate = self.robot.reconcile_arm_hold(deepcopy(failure["hold"]))
            problem = self._hold_problem(certificate)
            if problem:
                raise ValueError(problem)
            if certificate["calibration_id"] != failure["hold"]["calibration_id"]:
                raise ValueError("arm_calibration_changed")
            if certificate["held_goal_raw"] != failure["hold"]["held_goal_raw"]:
                raise ValueError("arm_hold_goals_changed")
            if not -.1 <= time.time() - certificate["timestamp"] <= 1.0:
                raise ValueError("arm_hold_feedback_stale")
        except Exception as exc:
            # A failed hardware verification has no reset path. In particular,
            # transport errors cannot be reclassified by trying another ID.
            self.halt()
            return reject(f"arm_hold_verification_failed: {str(exc)[:1024]}")

        try:
            verified_at = max(failure["finished_at"], certificate["timestamp"])
            # Camera workers publish asynchronously. Wait for a genuinely new
            # frame with the same bounded read-only polling used after motion.
            response = self._post_observe(failure["command"], verified_at)
            raw = response["observation"]
            ego = raw.get("ego")
            if not _fresh(ego, 1.0, time.time()) or ego["timestamp"] <= verified_at:
                raise ValueError("arm_feedback_unavailable_or_stale")
            if (ego.get("calibration_id") != certificate["calibration_id"]
                    or not isinstance(ego.get("calibration_status"), dict)
                    or ego["calibration_status"].get("valid") is not True):
                raise ValueError("arm_calibration_changed_or_invalid")
            joints = ego.get("joints")
            if not isinstance(joints, dict) or any(
                    not _number(joints.get(name))
                    or abs(joints[name] - value) > (2 if name == "gripper" else .5)
                    for name, value in certificate["measured"].items()):
                raise ValueError("arm_changed_after_hold_verification")
            # The observation was adopted while halted. Only the fully checked
            # ego state below that camera barrier can open admission.
        except Exception as exc:
            self.invalidate()
            return reject(f"arm_reconciliation_observation_failed: {str(exc)[:1024]}")

        self._recovery_constraints = deepcopy(failure["command"])
        self._arm_failure = None
        self._halt_reason = None
        response.update(status="ok", action_context=self._context(), recovery={
            "reconciled": True, "failed_action_id": request["failed_action_id"],
            "runtime_id": self.runtime_id,
            "attempts_remaining": self._remaining_reconciliations(),
            "hold": deepcopy(certificate),
        })
        return response

    @staticmethod
    def _camera_problem(observation, name, *, after=None):
        cameras = observation.get("cameras")
        camera = cameras.get(name) if isinstance(cameras, dict) else None
        if not _fresh(camera, 1.0, time.time()) or not isinstance(camera.get("data"), str) or not camera["data"]:
            return f"{name}_camera_unavailable_or_stale"
        if after is not None and camera["timestamp"] <= after:
            return f"{name}_camera_not_after_action"
        return None

    def observe(self):
        self.invalidate()  # Failed refresh must not leave an older lease usable.
        return self._adopt(self.robot.observe(), allow_unleased=True)

    def _adopt(self, raw, *, allow_unleased=False):
        if not isinstance(raw, dict):
            raise ValueError("Robot observation must be an object")
        if len(json.dumps(raw, allow_nan=False).encode()) > MAX_OBSERVATION_BYTES:
            if allow_unleased:
                # Legacy observers may use larger configured images. Deliver
                # their data without copying or retaining it as a guarded lease.
                observation_id = str(uuid4())
                return {
                    "observation": {**raw, "observation_id": observation_id, "runtime_id": self.runtime_id},
                    "capabilities": dict(CAPABILITIES),
                    "action_context": {**self._context("observation_too_large"), "observation_id": observation_id},
                }
            raise ValueError("Guarded observation exceeds size limit")
        observation = deepcopy(raw)
        observation_id = str(uuid4())
        observation.update(observation_id=observation_id, runtime_id=self.runtime_id)
        valid_cameras = [name for name in ("front", "wrist")
                         if self._camera_problem(observation, name) is None]
        old_ego = observation.get("ego")
        self._baseline = {"id": observation_id, "created": time.monotonic(),
                          "observation": observation, "valid_cameras": valid_cameras,
                          "ego_fresh": _fresh(old_ego, 1.0, time.time())}
        return {"observation": observation, "capabilities": dict(CAPABILITIES),
                "action_context": self._context()}

    @staticmethod
    def _required_cameras(command):
        names = []
        if command.get("action") == "move":
            if any(command.get(axis, 0) for axis in ("x", "y", "theta")):
                names.append("front")
            if command.get("joints") is not None:
                names.append("wrist")
        return names

    def _preflight(self, command):
        self._preflight_joints = None
        if self._halt_reason:
            return self._halt_reason
        baseline = self._baseline
        if command["runtime_id"] != self.runtime_id:
            return "runtime_mismatch"
        if baseline is None or command["observation_id"] != baseline["id"]:
            return "observation_mismatch"
        if time.monotonic() - baseline["created"] > self.max_age_s:
            self.invalidate()
            return "observation_expired"
        if not baseline["valid_cameras"]:
            return "no_fresh_camera"
        if self._recovery_constraints and command.get("action") != "wait":
            if not self._arm_only(command):
                return "arm_recovery_requires_arm_only"
            if command["duration_ms"] < RECOVERY_MIN_DURATION_MS:
                return "arm_recovery_duration_too_short"
            failed = self._recovery_constraints
            if command["joints"] == failed["joints"] and command["duration_ms"] == failed["duration_ms"]:
                return "arm_recovery_command_not_revised"

        before = baseline["observation"]
        try:
            current = self.robot.observe()
        except Exception as exc:
            self.invalidate()
            return f"preflight_observation_failed: {str(exc)[:1024]}"
        if not isinstance(current, dict):
            self.invalidate()
            return "preflight_observation_invalid"
        for name in self._required_cameras(command):
            problem = (f"{name}_camera_unavailable_or_stale" if name not in baseline["valid_cameras"]
                       else self._camera_problem(current, name))
            if problem:
                self.invalidate()
                return problem

        # Check all measured joints, even for base motion: the arm may have moved
        # since the image the agent used. Disabled/unavailable arms are not a
        # prerequisite for a wheels-only robot.
        old_ego = before.get("ego") if isinstance(before.get("ego"), dict) else {}
        ego = current.get("ego") if isinstance(current.get("ego"), dict) else {}
        needs_arm = command.get("joints") is not None
        if needs_arm or old_ego.get("available") is True:
            if not _fresh(ego, 1.0, time.time()) or not baseline["ego_fresh"]:
                self.invalidate()
                return "arm_feedback_unavailable_or_stale"
            old_joints, joints = old_ego.get("joints"), ego.get("joints")
            if not isinstance(old_joints, dict) or not old_joints or not isinstance(joints, dict):
                self.invalidate()
                return "arm_feedback_invalid"
            for name, old in old_joints.items():
                value = joints.get(name)
                if not _number(old) or not _number(value) or abs(value - old) > (5 if name == "gripper" else 2):
                    self.invalidate()
                    return "arm_state_changed"
            if needs_arm:
                limits = ego.get("limits", {})
                if not isinstance(limits, dict) or any(name not in limits for name in command["joints"]):
                    self.invalidate()
                    return "arm_calibration_unavailable"
                for name, target in command["joints"].items():
                    limit = limits[name]
                    if not isinstance(limit, dict) or not all(_number(limit.get(key)) for key in ("min", "max", "max_speed")):
                        return "arm_calibration_invalid"
                    if not limit["min"] <= target <= limit["max"]:
                        return "arm_target_out_of_range"
                    value = joints.get(name)
                    if not _number(value) or abs(target - value) > limit["max_speed"] * command["duration_ms"] / 1000:
                        return "arm_speed_limit"
                    if self._recovery_constraints and (
                            abs(target - value) > RECOVERY_MAX_JOINT_DELTA_DEG
                            or not _number(old_joints.get(name))
                            or abs(target - old_joints[name]) > RECOVERY_MAX_JOINT_DELTA_DEG):
                        return "arm_recovery_joint_delta_limit"

        # IMU is supplemental. Never treat integrated displacement as verified
        # base pose, and do not require an IMU on robots configured without it.
        if any(command.get(axis, 0) for axis in ("x", "y", "theta")):
            imu = current.get("imu") if isinstance(current.get("imu"), dict) else {}
            if imu.get("available") is True and not _fresh(imu, .25, time.time()):
                self.invalidate()
                return "imu_stale"
        if self._recovery_constraints and needs_arm:
            self._preflight_joints = deepcopy(ego["joints"])
        return None

    def _post_observe(self, command, finished_at, fallback_cameras=()):
        required = self._required_cameras(command)
        if not required:
            required = list(fallback_cameras)
        deadline = time.monotonic() + self.post_timeout_s
        problem = "post_observation_unavailable"
        while True:
            try:
                observation = self.robot.observe()
                if not isinstance(observation, dict):
                    raise ValueError("Robot observation must be an object")
                problems = [self._camera_problem(observation, name, after=finished_at) for name in required]
                problem = next((value for value in problems if value), None)
                if problem is None:
                    observation = deepcopy(observation)
                    # Never return a pre-action image as if it showed the result.
                    cameras = observation.get("cameras", {})
                    if not isinstance(cameras, dict):
                        raise ValueError("Camera observations must be an object")
                    for name, camera in cameras.items():
                        if isinstance(camera, dict) and camera.get("available") and self._camera_problem(observation, name, after=finished_at):
                            camera.update(available=False, data=None, error="no_fresh_post_action_frame")
                    return self._adopt(observation)
            except Exception as exc:
                problem = f"post_observation_failed: {str(exc)[:1024]}"
            if time.monotonic() >= deadline:
                raise RuntimeError(problem)
            time.sleep(min(.02, max(0, deadline - time.monotonic())))

    @staticmethod
    def _execution(command, outcome, command_sent=None):
        return {"action_id": command.get("action_id"), "outcome": outcome,
                "command_sent": False if command.get("action") == "wait" else command_sent,
                "arm_reached": None, "base_stop_command_sent": None,
                "base_stop_verified": None}

    def _reject(self, command, reason):
        return {"status": "rejected", "error": reason,
                "execution": self._execution(command, "rejected", False),
                "capabilities": dict(CAPABILITIES), "action_context": self._context()}

    @staticmethod
    def _metadata_problem(command):
        for name in GUARD_FIELDS:
            limit = 4096 if name == "reason" else 128
            if not isinstance(command.get(name), str) or not command[name].strip() or len(command[name]) > limit:
                return "guard_fields_required"
        return None

    @staticmethod
    def _validate(command):
        allowed = GUARD_FIELDS | {"request_id", "op", "action", "x", "y", "theta", "joints", "duration_ms", "monitor"}
        if set(command) - allowed:
            return "unknown_command_fields"
        if command.get("action") not in ("move", "wait"):
            return "unsupported_action"
        duration = command.get("duration_ms")
        values = [command.get(axis, 0) for axis in ("x", "y", "theta")]
        joints = command.get("joints")
        arm_only = (command["action"] == "move" and isinstance(joints, dict) and bool(joints)
                    and all(_number(value) and value == 0 for value in values))
        maximum = ARM_MAX_DURATION_MS if arm_only else BASE_MAX_DURATION_MS
        if type(duration) is not int or not (0 if command["action"] == "wait" else 1) <= duration <= maximum:
            return "invalid_duration"
        if not all(_number(value) for value in values):
            return "invalid_velocity"
        if math.hypot(*values[:2]) > .4 or abs(values[2]) > 150:
            return "velocity_limit"
        if command["action"] == "wait" and (joints is not None or any(values)):
            return "wait_cannot_move"
        if joints is not None and (not isinstance(joints, dict) or not joints or len(joints) > 6
                                   or duration < 100 or not all(k in JOINT_NAMES and _number(v) for k, v in joints.items())):
            return "invalid_joints"
        if "monitor" in command and (not arm_only or monitor_problem(command["monitor"], joints) is not None):
            return "invalid_monitor"
        return None

    def execute(self, command):
        """Reserve action identity before side effects; never evict old identities."""
        problem = self._metadata_problem(command)
        if problem:
            return self._reject(command, problem)
        identity = command["action_id"]
        try:
            fingerprint = json.dumps({key: value for key, value in command.items() if key != "request_id"},
                                     sort_keys=True, separators=(",", ":"))
        except (TypeError, ValueError):
            return self._reject(command, "invalid_command_encoding")
        if len(fingerprint.encode()) > 16384:
            return self._reject(command, "command_too_large")
        if identity in self._cache:
            previous, response = self._cache[identity]
            if previous != fingerprint:
                return self._reject(command, "action_id_conflict")
            return {**deepcopy(response), "replayed": True}
        # Count and total encoded response bytes are bounded. A full journal is
        # never recycled: unknown old action identities cannot execute again.
        if len(self._cache) >= self.max_actions or self._cache_bytes >= self.max_cache_bytes:
            self._halt_reason = "action_cache_full"
            return self._reject(command, self._halt_reason)
        pending = {"status": "error", "error": "execution_in_progress",
                   "execution": self._execution(command, "uncertain_execution"),
                   "capabilities": dict(CAPABILITIES), "action_context": self._context("execution_in_progress")}
        self._cache[identity] = (fingerprint, pending)
        problem = self._validate(command) or self._preflight(command)
        if problem:
            response = self._reject(command, problem)
        else:
            post_cameras = tuple(self._baseline["valid_cameras"])
            calibration_id = (self._baseline["observation"].get("ego") or {}).get("calibration_id")
            preflight_joints = self._preflight_joints
            self.invalidate()  # The observation is single-use, including on failure.
            try:
                execute = getattr(self.robot, "execute_guarded", None)
                if execute is None:
                    accepted = self.robot.execute(command)
                    execution = self._execution(command, "completed" if accepted else "rejected")
                    if not accepted:
                        execution["command_sent"] = False
                else:
                    execution = execute(command)
                    if not isinstance(execution, dict) or execution.get("outcome") not in (
                            "completed", "rejected", "incomplete", "warning", "stopped"):
                        raise ValueError("Invalid guarded execution result")
                    if len(json.dumps(execution, allow_nan=False).encode()) > 64 * 1024:
                        raise ValueError("Guarded execution result exceeds size limit")
                    execution = {**self._execution(command, execution["outcome"]), **execution,
                                 "action_id": identity}
                    if execution["outcome"] == "incomplete":
                        self._record_incomplete(command, execution)
                    elif execution["outcome"] == "warning":
                        self._validate_warning(command, execution, calibration_id)
                    elif execution["outcome"] == "stopped":
                        self._validate_stopped(command, execution, calibration_id)
                response = {"status": execution["outcome"], "execution": execution,
                            "capabilities": dict(CAPABILITIES), "action_context": self._context()}
                if execution["outcome"] == "incomplete":
                    response["recovery"] = self._recovery()
            except PreDispatchRejected as exc:
                # Only an explicit driver guarantee that no motor write has
                # happened is recoverable. The consumed lease still requires
                # a new observation and this action ID keeps its rejection.
                response = self._reject(command, str(exc)[:1024])
                response.update(error_code=exc.code, error_details=exc.details)
            except Exception as exc:
                self.halt()
                response = {"status": "error", "error": str(exc)[:1024],
                            "execution": self._execution(command, "uncertain_execution"),
                            "capabilities": dict(CAPABILITIES), "action_context": self._context()}
            else:
                if response["status"] in ("completed", "warning", "stopped"):
                    warning = response["status"] == "warning"
                    stopped = response["status"] == "stopped"
                    finished_at = time.time()
                    response["execution"]["motion_finished_at"] = finished_at
                    try:
                        response.update(self._post_observe(command, finished_at, post_cameras))
                        if warning or stopped:
                            try:
                                if warning:
                                    self._validate_warning_observation(response, command, execution)
                                else:
                                    self._validate_stop_observation(response, execution)
                            except Exception:
                                self.halt()
                                raise
                        if (self._recovery_constraints and self._arm_only(command)
                                and execution.get("command_sent") is True
                                and execution.get("arm_reached") is True):
                            post_ego = response["observation"].get("ego", {})
                            post_joints = post_ego.get("joints", {}) if isinstance(post_ego, dict) else {}
                            progress = bool(
                                _fresh(post_ego, 1.0, time.time())
                                and post_ego["timestamp"] > finished_at
                                and isinstance(preflight_joints, dict) and isinstance(post_joints, dict)
                                and any(_number(preflight_joints.get(name)) and _number(post_joints.get(name))
                                        and abs(target - preflight_joints[name]) - abs(target - post_joints[name]) >= 1.0
                                        for name, target in command["joints"].items()))
                            response["execution"]["arm_recovery_progress_verified"] = progress
                            if progress:
                                self._recovery_constraints = None
                            response["action_context"] = self._context()
                    except Exception as exc:
                        self.invalidate()
                        response.update(status=("warning_without_observation" if warning else
                                                "stopped_without_observation" if stopped else
                                                "completed_without_observation"),
                                        observation_error=str(exc)[:1024],
                                        action_context=self._context())

        encoded = json.dumps(response, allow_nan=False, separators=(",", ":"))
        size = len(fingerprint.encode()) + len(encoded.encode())
        if self._cache_bytes + size > self.max_cache_bytes:
            # Execution may already have completed. Keep its outcome and identity
            # but return no oversized images, then close admission permanently.
            self._halt_reason = "action_cache_full"
            self.invalidate()
            response.pop("observation", None)
            if response["status"] == "completed":
                response["status"] = "completed_without_observation"
            elif response["status"] == "warning":
                response["status"] = "warning_without_observation"
            elif response["status"] == "stopped":
                response["status"] = "stopped_without_observation"
            response.update(observation_error="action_cache_full", action_context=self._context())
            size = len(fingerprint.encode()) + len(json.dumps(response).encode())
        self._cache[identity] = (fingerprint, deepcopy(response))
        self._cache_bytes += size
        return response
