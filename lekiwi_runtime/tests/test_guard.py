from copy import deepcopy
import time
import unittest
from unittest.mock import patch

from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.guard import MAX_OBSERVATION_BYTES
from lekiwi_runtime.orin import OrinRobot


class ObservedRobot:
    """No device access: fresh sensor timestamps, with controllable failures."""
    def __init__(self):
        self.executions = []
        self.joints = {"wrist_roll": 0.0, "gripper": 50.0}
        self.camera_errors = {}
        self.camera_ages = {}
        self.post_frozen = False
        self.execute_error = None
        self.observe_error = None
        self.imu = {"available": False, "error": "disabled"}

    def observe(self):
        if self.observe_error:
            raise RuntimeError(self.observe_error)
        stamp = time.time()
        cameras = {}
        for name in ("front", "wrist"):
            age = self.camera_ages.get(name, 0)
            if self.post_frozen and self.executions:
                age = 5
            cameras[name] = {"available": True, "timestamp": stamp - age,
                             "age_s": age, "data": "jpeg", "error": self.camera_errors.get(name)}
        return {"timestamp": stamp, "cameras": cameras, "imu": dict(self.imu),
                "radar": {"available": False, "error": "disabled"},
                "ego": {"available": True, "timestamp": stamp, "error": None,
                        "joints": dict(self.joints),
                        "limits": {name: {"min": -90, "max": 100, "max_speed": 50 if name == "gripper" else 30}
                                   for name in self.joints}}}

    def execute(self, command):
        self.executions.append(deepcopy(command))
        if self.execute_error:
            raise RuntimeError(self.execute_error)
        self.joints.update(command.get("joints", {}))
        return True


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.robot = ObservedRobot()
        self.client = RobotClient(self.robot, "test", "inproc://unused")
        self.client.guard.post_timeout_s = .025

    def observe(self):
        return self.client.handle({"op": "observe", "request_id": "obs-rpc"})

    def command(self, observation=None, **overrides):
        observation = observation or self.observe()
        context = observation["action_context"]
        return {"op": "execute", "request_id": "exec-rpc", "action_id": "action-1",
                "runtime_id": context["runtime_id"], "observation_id": context["observation_id"],
                "reason": "A short forward step", "action": "move", "x": .05, "y": 0,
                "theta": 0, "duration_ms": 100, **overrides}

    def test_observation_identity_and_capability(self):
        response = self.observe()
        self.assertEqual(response["capabilities"], {"guarded_execution": 1, "arm_reconciliation": 1})
        for name in ("runtime_id", "observation_id"):
            self.assertEqual(response["observation"][name], response["action_context"][name])
        self.assertTrue(response["action_context"]["movement_allowed"])
        self.assertEqual(response["action_context"]["max_age_s"], 60)

    def test_large_legacy_observation_is_delivered_without_guarded_lease(self):
        raw = self.robot.observe()
        raw["cameras"]["front"]["data"] = "x" * (MAX_OBSERVATION_BYTES + 1)
        self.robot.observe = lambda: raw
        response = self.observe()
        self.assertEqual(response["status"], "ok")
        self.assertIs(response["observation"]["cameras"], raw["cameras"])
        self.assertGreater(len(response["observation"]["cameras"]["front"]["data"]), MAX_OBSERVATION_BYTES)
        self.assertFalse(response["action_context"]["movement_allowed"])
        self.assertEqual(response["action_context"]["reason"], "observation_too_large")
        self.assertEqual(response["action_context"]["observation_id"], response["observation"]["observation_id"])
        self.assertIsNone(self.client.guard._baseline)
        self.assertEqual(self.client.guard._cache, {})
        rejected = self.client.handle(self.command(response))
        self.assertEqual(rejected["status"], "rejected")
        self.assertFalse(rejected["execution"]["command_sent"])
        self.assertFalse(self.robot.executions)
        self.assertNotIn("observation", rejected)

    def test_nonfinite_observation_still_fails_closed(self):
        self.observe()
        raw = self.robot.observe()
        raw["ego"]["joints"]["wrist_roll"] = float("nan")
        self.robot.observe = lambda: raw
        response = self.observe()
        self.assertEqual(response["status"], "error")
        self.assertFalse(response["action_context"]["movement_allowed"])
        self.assertIsNone(self.client.guard._baseline)

    def test_identical_retry_replays_result_and_echoes_new_rpc_id(self):
        command = self.command()
        first = self.client.handle(command)
        second = self.client.handle({**command, "request_id": "retry-rpc"})
        self.assertEqual(first["status"], "completed")
        self.assertEqual(second["request_id"], "retry-rpc")
        self.assertTrue(second["replayed"])
        self.assertEqual(second["execution"], first["execution"])
        self.assertEqual(len(self.robot.executions), 1)
        self.assertNotEqual(first["action_context"]["observation_id"], command["observation_id"])
        conflict = self.client.handle({**command, "x": .1})
        self.assertEqual(conflict["error"], "action_id_conflict")
        self.assertEqual(len(self.robot.executions), 1)

    def test_consumed_and_superseded_observations_never_execute(self):
        command = self.command()
        self.client.handle(command)
        response = self.client.handle({**command, "action_id": "action-2"})
        self.assertEqual(response["error"], "observation_mismatch")
        self.assertEqual(len(self.robot.executions), 1)
        newer = self.command(action_id="action-3")
        self.observe()
        self.assertEqual(self.client.handle(newer)["error"], "observation_mismatch")

    def test_expiry_uses_runtime_monotonic_age(self):
        command = self.command()
        self.client.guard._baseline["created"] -= 61
        self.assertEqual(self.client.handle(command)["error"], "observation_expired")
        self.assertFalse(self.robot.executions)

    def test_image_observed_seconds_ago_is_valid_within_lease(self):
        command = self.command()
        baseline = self.client.guard._baseline
        baseline["created"] -= 5
        for camera in baseline["observation"]["cameras"].values():
            camera["timestamp"] -= 5
        self.assertEqual(self.client.handle(command)["status"], "completed")

    def test_restart_identity_rejects_old_commands(self):
        command = self.command()
        self.client = RobotClient(self.robot, "test", "inproc://unused")
        self.observe()
        self.assertEqual(self.client.handle(command)["error"], "runtime_mismatch")
        self.assertFalse(self.robot.executions)

    def test_partial_guard_envelope_cannot_fall_back_to_legacy(self):
        request = {"op": "execute", "request_id": "rpc", "action": "wait", "duration_ms": 0,
                   "action_id": "missing-other-fields"}
        self.assertEqual(self.client.handle(request)["error"], "guard_fields_required")
        self.assertFalse(self.robot.executions)

    def test_rejected_action_identity_cannot_be_repurposed(self):
        command = self.command(duration_ms=2000)
        self.assertEqual(self.client.handle(command)["error"], "invalid_duration")
        self.assertTrue(self.client.handle(command)["replayed"])
        self.assertEqual(self.client.handle({**command, "duration_ms": 100})["error"], "action_id_conflict")
        self.assertFalse(self.robot.executions)

    def test_legacy_execution_invalidates_guard_lease(self):
        command = self.command()
        legacy = self.client.handle({"op": "execute", "request_id": "legacy", "action": "wait", "duration_ms": 0})
        self.assertEqual(legacy["status"], "completed")
        self.assertNotIn("execution", legacy)
        self.assertEqual(self.client.handle(command)["error"], "observation_mismatch")
        self.assertEqual(len(self.robot.executions), 1)

    def test_changed_arm_and_gripper_state_reject_before_execution(self):
        for joint, delta in (("wrist_roll", 2.01), ("gripper", 5.01)):
            with self.subTest(joint=joint):
                command = self.command(action_id=joint)
                self.robot.joints[joint] += delta
                self.assertEqual(self.client.handle(command)["error"], "arm_state_changed")
                self.assertFalse(self.robot.executions)

    def test_sensor_requirements_are_action_specific(self):
        self.robot.camera_errors["wrist"] = "disabled"
        self.assertEqual(self.client.handle(self.command())["status"], "completed")
        arm = self.command(action_id="arm", x=0, joints={"wrist_roll": 1})
        self.assertEqual(self.client.handle(arm)["error"], "wrist_camera_unavailable_or_stale")
        self.robot.camera_errors = {"front": "disabled"}
        arm = self.command(action_id="arm2", x=0, joints={"wrist_roll": 1})
        self.assertEqual(self.client.handle(arm)["status"], "completed")

    def test_stale_current_camera_and_available_imu_are_rejected(self):
        command = self.command()
        self.robot.camera_ages["front"] = 2
        self.assertEqual(self.client.handle(command)["error"], "front_camera_unavailable_or_stale")
        self.robot.camera_ages.clear()
        command = self.command(action_id="imu-stale")
        self.robot.imu = {"available": True, "timestamp": time.time() - 1, "age_s": 1}
        self.assertEqual(self.client.handle(command)["error"], "imu_stale")
        self.assertFalse(self.robot.executions)

    def test_execution_exception_halts_and_is_never_retried(self):
        command = self.command()
        self.robot.execute_error = "partial serial write"
        first = self.client.handle(command)
        self.assertEqual(first["status"], "error")
        self.assertIsNone(first["execution"]["command_sent"])
        self.assertEqual(first["execution"]["outcome"], "uncertain_execution")
        self.assertFalse(first["action_context"]["movement_allowed"])
        self.assertTrue(self.client.handle(command)["replayed"])
        fresh = self.command(action_id="action-2")
        self.assertEqual(self.client.handle(fresh)["error"], "uncertain_execution")
        self.assertEqual(len(self.robot.executions), 1)

    def test_known_arm_range_and_speed_failures_are_safe_rejections(self):
        for target, expected in ((101, "arm_target_out_of_range"), (10, "arm_speed_limit")):
            command = self.command(action_id=str(target), x=0, joints={"wrist_roll": target})
            response = self.client.handle(command)
            self.assertEqual(response["error"], expected)
            self.assertFalse(response["execution"]["command_sent"])
        self.assertFalse(self.robot.executions)
        self.assertTrue(self.observe()["action_context"]["movement_allowed"])

    def test_legacy_execution_error_also_halts_later_guarded_motion(self):
        self.robot.execute_error = "partial serial write"
        response = self.client.handle({"op": "execute", "request_id": "legacy", "action": "move"})
        self.assertEqual(response["status"], "error")
        self.robot.execute_error = None
        command = self.command()
        self.assertEqual(self.client.handle(command)["error"], "uncertain_execution")
        self.assertEqual(len(self.robot.executions), 1)

    def test_post_observation_failure_preserves_completion_and_consumes_lease(self):
        command = self.command()
        self.robot.post_frozen = True
        first = self.client.handle(command)
        self.assertEqual(first["status"], "completed_without_observation")
        self.assertEqual(first["execution"]["outcome"], "completed")
        self.assertNotIn("observation", first)
        self.assertFalse(first["action_context"]["movement_allowed"])
        self.assertTrue(self.client.handle(command)["replayed"])
        self.assertEqual(len(self.robot.executions), 1)
        self.assertEqual(self.client.handle({**command, "action_id": "action-2"})["error"], "observation_mismatch")
        self.robot.post_frozen = False
        self.assertTrue(self.observe()["action_context"]["movement_allowed"])

    def test_old_irrelevant_image_is_never_returned_as_post_action_image(self):
        command = self.command()
        self.robot.camera_ages["wrist"] = 2
        response = self.client.handle(command)
        self.assertEqual(response["status"], "completed")
        self.assertIsNone(response["observation"]["cameras"]["wrist"]["data"])
        self.assertFalse(response["observation"]["cameras"]["wrist"]["available"])

    def test_bounded_cache_fails_closed_without_eviction(self):
        self.client.guard.max_actions = 1
        command = self.command()
        self.client.handle(command)
        self.assertTrue(self.client.handle(command)["replayed"])
        fresh = self.command(action_id="new")
        self.assertEqual(self.client.handle(fresh)["error"], "action_cache_full")
        self.assertTrue(self.client.handle(command)["replayed"])
        self.assertEqual(len(self.robot.executions), 1)

    def test_wait_never_claims_motor_command(self):
        command = self.command(action="wait", x=0, duration_ms=0)
        result = self.client.handle(command)
        self.assertEqual(result["status"], "completed")
        self.assertFalse(result["execution"]["command_sent"])


class StructuredDriverResultTest(unittest.TestCase):
    def test_orin_preserves_bool_api_and_reports_base_limits(self):
        robot = OrinRobot(mock=True)
        with patch("time.sleep"):
            result = robot.execute_guarded({"action": "move", "x": .1, "duration_ms": 100})
        self.assertEqual(result["outcome"], "completed")
        self.assertTrue(result["command_sent"])
        self.assertTrue(result["base_stop_command_sent"])
        self.assertFalse(result["base_stop_verified"])
        self.assertIsNone(result["arm_reached"])

    def test_arm_result_comes_from_feedback(self):
        robot = OrinRobot(mock=True)
        with patch("time.sleep"):
            result = robot.execute_guarded({"action": "move", "joints": {"wrist_roll": 1}, "duration_ms": 100})
        self.assertTrue(result["arm_reached"])
        self.assertAlmostEqual(result["arm_motion"]["measured"]["wrist_roll"], 1, delta=.1)
        self.assertIsNone(result["base_stop_command_sent"])


if __name__ == "__main__":
    unittest.main()
