"""Opt-in observation leases and at-most-once execution for one runtime process.

This is an execution/observation contract, not a collision or localization system.
All clocks and sensor timestamps are checked on the robot host.
"""

from copy import deepcopy
import json
import math
import time
from uuid import uuid4


GUARD_FIELDS = frozenset({"observation_id", "runtime_id", "action_id", "reason"})
CAPABILITIES = {"guarded_execution": 1}
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

    def invalidate(self):
        self._baseline = None

    def halt(self, reason="uncertain_execution"):
        self.invalidate()
        self._halt_reason = reason

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
        return context

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

        # IMU is supplemental. Never treat integrated displacement as verified
        # base pose, and do not require an IMU on robots configured without it.
        if any(command.get(axis, 0) for axis in ("x", "y", "theta")):
            imu = current.get("imu") if isinstance(current.get("imu"), dict) else {}
            if imu.get("available") is True and not _fresh(imu, .25, time.time()):
                self.invalidate()
                return "imu_stale"
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
        allowed = GUARD_FIELDS | {"request_id", "op", "action", "x", "y", "theta", "joints", "duration_ms"}
        if set(command) - allowed:
            return "unknown_command_fields"
        if command.get("action") not in ("move", "wait"):
            return "unsupported_action"
        duration = command.get("duration_ms")
        if type(duration) is not int or not (0 if command["action"] == "wait" else 1) <= duration <= 1000:
            return "invalid_duration"
        values = [command.get(axis, 0) for axis in ("x", "y", "theta")]
        if not all(_number(value) for value in values):
            return "invalid_velocity"
        if math.hypot(*values[:2]) > .4 or abs(values[2]) > 150:
            return "velocity_limit"
        joints = command.get("joints")
        if command["action"] == "wait" and (joints is not None or any(values)):
            return "wait_cannot_move"
        if joints is not None and (not isinstance(joints, dict) or not joints or len(joints) > 6
                                   or duration < 100 or not all(k in JOINT_NAMES and _number(v) for k, v in joints.items())):
            return "invalid_joints"
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
                    if not isinstance(execution, dict) or execution.get("outcome") not in ("completed", "rejected"):
                        raise ValueError("Invalid guarded execution result")
                    execution = {**self._execution(command, execution["outcome"]), **execution,
                                 "action_id": identity}
                response = {"status": execution["outcome"], "execution": execution,
                            "capabilities": dict(CAPABILITIES), "action_context": self._context()}
            except Exception as exc:
                self._halt_reason = "uncertain_execution"
                response = {"status": "error", "error": str(exc)[:1024],
                            "execution": self._execution(command, "uncertain_execution"),
                            "capabilities": dict(CAPABILITIES), "action_context": self._context()}
            else:
                if response["status"] == "completed":
                    finished_at = time.time()
                    response["execution"]["motion_finished_at"] = finished_at
                    try:
                        response.update(self._post_observe(command, finished_at, post_cameras))
                    except Exception as exc:
                        self.invalidate()
                        response.update(status="completed_without_observation", observation_error=str(exc)[:1024],
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
            response.update(observation_error="action_cache_full", action_context=self._context())
            size = len(fingerprint.encode()) + len(json.dumps(response).encode())
        self._cache[identity] = (fingerprint, deepcopy(response))
        self._cache_bytes += size
        return response
