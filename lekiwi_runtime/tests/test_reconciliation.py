"""Reconciliation admission tests with memory-only sensors and motor counters."""
from copy import deepcopy
import time
import unittest

from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.guard import JOINT_NAMES
from test_guard import ObservedRobot


class ReconciliationRobot(ObservedRobot):
    def __init__(self):
        super().__init__()
        self.joints = dict.fromkeys(JOINT_NAMES, 0.)
        self.joints["gripper"] = 50.
        self.incomplete = True
        self.reconciliations = 0
        self.hold_error = None
        self.execution_mutator = None
        self.certificate_mutator = None
        self.observation_mutator = None
        self.old_frames = 0

    def observe(self):
        observation = super().observe()
        observation["ego"].update(calibration_id="calibration-1", calibration_status={"valid": True})
        if self.old_frames:
            self.old_frames -= 1
            observation["cameras"]["wrist"]["timestamp"] -= .1
        if self.observation_mutator:
            self.observation_mutator(observation)
        return observation

    def certificate(self):
        return {"verified": True, "timestamp": time.time(), "calibration_id": "calibration-1",
                "held_goal_raw": dict.fromkeys(JOINT_NAMES, 2048), "measured": dict(self.joints)}

    def execute_guarded(self, command):
        self.executions.append(deepcopy(command))
        if self.execute_error:
            raise OSError(self.execute_error)
        if self.incomplete and command["action"] == "move":
            for name, target in command["joints"].items():
                self.joints[name] = (self.joints[name] + target) / 2
            result = {"outcome": "incomplete", "command_sent": True, "arm_reached": False,
                      "arm_hold_verified": True,
                      "arm_motion": {"recovery_eligible": True, "hold": self.certificate()}}
        else:
            self.joints.update(command.get("joints", {}))
            result = {"outcome": "completed", "command_sent": command["action"] == "move",
                      "arm_reached": True if command.get("joints") else None}
        if self.execution_mutator:
            self.execution_mutator(result)
        return result

    def reconcile_arm_hold(self, certificate):
        self.reconciliations += 1
        if self.hold_error:
            raise OSError(self.hold_error)
        result = self.certificate()
        if self.certificate_mutator:
            self.certificate_mutator(result)
        return result


class ReconciliationTest(unittest.TestCase):
    def setUp(self):
        self.robot = ReconciliationRobot()
        self.client = RobotClient(self.robot, "memory", "inproc://unused")
        self.guard = self.client.guard
        self.guard.post_timeout_s = .025
        self.sequence = 0

    def observe(self):
        return self.client.handle({"op": "observe", "request_id": "observation"})

    def command(self, observation=None, **changes):
        self.sequence += 1
        context = (observation or self.observe())["action_context"]
        return {"request_id": "rpc", "op": "execute", "action": "move",
                "action_id": f"action-{self.sequence}", "runtime_id": self.guard.runtime_id,
                "observation_id": context["observation_id"], "reason": "Memory-only arm movement",
                "joints": {"wrist_roll": self.robot.joints["wrist_roll"] + 6},
                "duration_ms": 1000, **changes}

    def make_incomplete(self):
        self.failed_command = self.command()
        response = self.client.handle(self.failed_command)
        self.assertEqual(response["status"], "incomplete", response)
        self.failed_response = deepcopy(response)
        return response

    def reconcile(self, **changes):
        return self.client.handle({"op": "reconcile", "request_id": "reconcile-rpc",
                                   "runtime_id": self.guard.runtime_id,
                                   "failed_action_id": self.failed_command["action_id"], **changes})

    def test_read_only_reconciliation_issues_new_constrained_lease(self):
        before = self.make_incomplete()
        response = self.reconcile()
        self.assertEqual(response["status"], "ok", response)
        self.assertTrue(response["recovery"]["reconciled"])
        self.assertTrue(response["recovery"]["hold"]["verified"])
        self.assertEqual(response["recovery"]["attempts_remaining"], 1)
        context = response["action_context"]
        self.assertTrue(context["movement_allowed"])
        self.assertEqual(context["recovery_constraints"],
                         {"max_joint_delta_deg": 3., "min_duration_ms": 3000, "remaining": 1,
                          "failed_action_id": self.failed_command["action_id"], "runtime_id": self.guard.runtime_id})
        self.assertNotEqual(context["observation_id"], self.failed_command["observation_id"])
        self.assertEqual(len(self.robot.executions), 1)
        self.assertEqual(self.robot.reconciliations, 1)
        replay = self.client.handle(self.failed_command)
        self.assertTrue(replay.pop("replayed"))
        self.assertEqual(replay, before)
        self.assertEqual(self.reconcile()["error"], "arm_reconciliation_not_available")
        self.assertEqual(self.robot.reconciliations, 1)

    def test_reconcile_waits_for_first_frame_after_verified_hold(self):
        self.make_incomplete()
        self.robot.old_frames = 1
        response = self.reconcile()
        self.assertEqual(response["status"], "ok", response)
        self.assertGreater(response["observation"]["cameras"]["wrist"]["timestamp"],
                           response["recovery"]["hold"]["timestamp"])
        self.assertEqual(self.robot.reconciliations, 1)

    def test_identity_extra_fields_and_stale_failure_never_verify_or_execute(self):
        self.make_incomplete()
        for changes, reason in (({"runtime_id": "other"}, "runtime_mismatch"),
                                ({"failed_action_id": "other"}, "failed_action_mismatch"),
                                ({"joints": {"wrist_roll": 9}}, "unknown_reconcile_fields")):
            with self.subTest(reason=reason):
                self.assertEqual(self.reconcile(**changes)["error"], reason)
        self.guard._arm_failure["created"] -= 61
        self.assertEqual(self.reconcile()["error"], "arm_reconciliation_expired")
        self.assertFalse(self.reconcile()["recovery"]["eligible"])
        self.assertEqual(self.robot.reconciliations, 0)
        self.assertEqual(len(self.robot.executions), 1)

    def test_unknown_faults_and_manual_halts_have_no_reconciliation(self):
        command = self.command()
        self.failed_command = command
        self.robot.execute_error = "serial read failed"
        self.assertEqual(self.client.handle(command)["execution"]["outcome"], "uncertain_execution")
        for reason in ("uncertain_execution", "action_cache_full"):
            self.guard.halt(reason)
            self.assertEqual(self.reconcile()["error"], "arm_reconciliation_not_available")
            self.assertFalse(self.observe()["action_context"]["movement_allowed"])
        self.assertEqual(self.robot.reconciliations, 0)

    def test_malformed_incomplete_halts_and_retains_action_identity(self):
        cases = {
            "hold_false": lambda packet: packet["arm_motion"]["hold"].update(verified=False),
            "sent_unknown": lambda packet: packet.update(command_sent=None),
            "not_eligible": lambda packet: packet["arm_motion"].update(recovery_eligible=False),
            "stale_hold": lambda packet: packet["arm_motion"]["hold"].update(timestamp=time.time() - 2),
            "nan": lambda packet: packet["arm_motion"].update(error=float("nan")),
            "non_json": lambda packet: packet["arm_motion"].update(error=object()),
        }
        for name, mutate in cases.items():
            with self.subTest(name=name):
                self.setUp()
                self.robot.execution_mutator = mutate
                command = self.command()
                response = self.client.handle(command)
                self.assertEqual(response["execution"]["outcome"], "uncertain_execution", response)
                self.assertFalse(response["action_context"]["movement_allowed"])
                self.assertTrue(self.client.handle(command)["replayed"])
                self.assertEqual(len(self.robot.executions), 1)

    def test_base_or_gripper_cannot_be_reclassified_incomplete(self):
        for changes in ({"x": .1}, {"joints": {"gripper": 55.}}):
            with self.subTest(changes=changes):
                self.setUp()
                response = self.client.handle(self.command(**changes))
                self.assertEqual(response["execution"]["outcome"], "uncertain_execution")

    def test_failed_hold_or_changed_certificate_never_unhalts(self):
        mutations = (
            lambda cert: cert.update(verified=False),
            lambda cert: cert.update(calibration_id="other"),
            lambda cert: cert["held_goal_raw"].update(wrist_roll=2049),
            lambda cert: cert.update(timestamp=time.time() - 2),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.setUp()
                self.make_incomplete()
                self.robot.certificate_mutator = mutate
                response = self.reconcile()
                self.assertEqual(response["status"], "rejected")
                self.assertEqual(response["action_context"]["reason"], "uncertain_execution")
                self.assertFalse(response["recovery"]["eligible"])
        self.setUp()
        self.make_incomplete()
        self.robot.hold_error = "serial timeout"
        self.assertEqual(self.reconcile()["action_context"]["reason"], "uncertain_execution")
        self.assertEqual(self.reconcile()["error"], "arm_reconciliation_not_available")

    def test_stale_camera_changed_calibration_or_drift_never_issues_lease(self):
        mutations = (
            lambda obs: obs["cameras"]["wrist"].update(timestamp=time.time() - 2),
            lambda obs: obs["ego"].update(timestamp=time.time() - 2),
            lambda obs: obs["ego"].update(calibration_id="other"),
            lambda obs: obs["ego"]["calibration_status"].update(valid=False),
            lambda obs: obs["ego"]["joints"].update(wrist_roll=4.),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.setUp()
                self.make_incomplete()
                self.robot.observation_mutator = mutate
                response = self.reconcile()
                self.assertEqual(response["status"], "rejected", response)
                self.assertFalse(response["action_context"]["movement_allowed"])
                self.assertIsNone(response["action_context"]["observation_id"])
                self.assertEqual(len(self.robot.executions), 1)

    def test_constraints_survive_wait_observe_and_bad_proposals_until_success(self):
        self.make_incomplete()
        self.assertEqual(self.reconcile()["status"], "ok")
        proposals = (
            ({"joints": {"wrist_roll": 5}, "duration_ms": 2999}, "arm_recovery_duration_too_short"),
            ({"joints": {"wrist_roll": 6.01}, "duration_ms": 3000}, "arm_recovery_joint_delta_limit"),
            ({"joints": {"gripper": 51}, "duration_ms": 3000}, "arm_recovery_requires_arm_only"),
            ({"joints": {"wrist_roll": 5}, "x": .1, "duration_ms": 1000}, "arm_recovery_requires_arm_only"),
        )
        for changes, reason in proposals:
            with self.subTest(reason=reason):
                response = self.client.handle(self.command(**changes))
                self.assertEqual(response["error"], reason)
                self.assertIn("recovery_constraints", self.observe()["action_context"])
        self.robot.incomplete = False
        wait = self.command(action="wait", joints=None, duration_ms=0)
        self.assertEqual(self.client.handle(wait)["status"], "completed")
        self.assertIn("recovery_constraints", self.observe()["action_context"])
        result = self.client.handle(self.command(joints={"wrist_roll": 5.5}, duration_ms=3000))
        self.assertEqual(result["status"], "completed", result)
        self.assertNotIn("recovery_constraints", result["action_context"])
        self.assertEqual(len(self.robot.executions), 3)

    def test_same_command_rejected_but_duration_revision_is_supported(self):
        self.failed_command = self.command(duration_ms=3000)
        self.assertEqual(self.client.handle(self.failed_command)["status"], "incomplete")
        self.assertEqual(self.reconcile()["status"], "ok")
        result = self.client.handle(self.command(joints={"wrist_roll": 6}, duration_ms=3000))
        self.assertEqual(result["error"], "arm_recovery_command_not_revised")
        self.robot.incomplete = False
        revised = self.client.handle(self.command(joints={"wrist_roll": 6}, duration_ms=4000))
        self.assertEqual(revised["status"], "completed", revised)

    def test_legacy_cannot_bypass_any_halt_or_recovery_constraint(self):
        legacy = {"op": "execute", "request_id": "legacy", "action": "wait", "duration_ms": 0}
        self.make_incomplete()
        self.assertEqual(self.client.handle(legacy)["error"], "arm_reconciliation_required")
        self.assertEqual(self.reconcile()["status"], "ok")
        self.assertEqual(self.client.handle(legacy)["error"], "arm_recovery_constrained")
        for reason in ("uncertain_execution", "action_cache_full"):
            self.guard.halt(reason)
            self.assertEqual(self.client.handle(legacy)["error"], reason)
        self.assertEqual(len(self.robot.executions), 1)

    def test_only_two_reconciliations_per_runtime_even_after_successful_moves(self):
        for expected_remaining in (1, 0):
            self.robot.incomplete = True
            self.make_incomplete()
            response = self.reconcile()
            self.assertEqual(response["status"], "ok", response)
            self.assertEqual(response["recovery"]["attempts_remaining"], expected_remaining)
            self.robot.incomplete = False
            result = self.client.handle(self.command(
                joints={"wrist_roll": self.robot.joints["wrist_roll"] + 1}, duration_ms=3000))
            self.assertEqual(result["status"], "completed", result)
        self.robot.incomplete = True
        failure = self.make_incomplete()
        self.assertFalse(failure["recovery"]["eligible"])
        self.assertEqual(self.reconcile()["error"], "arm_reconciliation_limit")
        self.assertFalse(self.observe()["action_context"]["movement_allowed"])
        self.assertEqual(self.robot.reconciliations, 2)

    def test_failed_camera_attempts_also_consume_the_bounded_budget(self):
        self.make_incomplete()
        self.robot.camera_ages["wrist"] = 5
        for remaining in (1, 0):
            response = self.reconcile()
            self.assertEqual(response["status"], "rejected")
            self.assertEqual(response["recovery"]["attempts_remaining"], remaining)
        self.robot.camera_ages.clear()
        self.assertEqual(self.reconcile()["error"], "arm_reconciliation_limit")
        self.assertEqual(self.robot.reconciliations, 2)

    def test_failed_post_action_observation_does_not_clear_constraint(self):
        self.make_incomplete()
        self.assertEqual(self.reconcile()["status"], "ok")
        self.robot.incomplete = False
        command = self.command(joints={"wrist_roll": 5}, duration_ms=3000)
        original = self.robot.execute_guarded

        def execute(value):
            result = original(value)
            self.robot.post_frozen = True
            return result

        self.robot.execute_guarded = execute
        result = self.client.handle(command)
        self.assertEqual(result["status"], "completed_without_observation")
        self.assertIn("recovery_constraints", result["action_context"])
        self.robot.post_frozen = False
        self.assertIn("recovery_constraints", self.observe()["action_context"])


if __name__ == "__main__":
    unittest.main()
