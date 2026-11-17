"""Measured hold reconciliation on a fake servo bus and fake clock only."""
from contextlib import ExitStack
from copy import deepcopy
import json
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import (
    ARM_CONTROL_REVISION, ARM_JOINTS, ArmController, ArmTargetNotReached, ArmTransportError,
)
from lekiwi_runtime.orin import OrinRobot
from test_arm_feedback import FakeClock
from test_profile_readback import MemoryPackets, status_packet


class ArmReconcileTest(unittest.TestCase):
    def make_arm(self, progress=None):
        bus = SimpleNamespace(port='memory', port_handler=object(), close=Mock())
        arm = ArmController({**load_config()['arm'], 'p_coefficient': None}, bus=bus)
        packets = MemoryPackets(arm)
        packets.follow_goals = False
        packets.read2ByteTxRx = lambda port, mid, address: (packets.raw[packets.names[mid]], 0, 0)
        bus.packet_handler = packets
        # Simulate an already powered robot. Verification never enables these
        # other joints; absent health/readback evidence must remain a failure.
        for name, mid in ARM_JOINTS:
            packets.registers[mid, 42] = packets.raw[name]
            packets.registers[mid, 40] = 1
        clock = FakeClock()
        initial = dict(packets.raw)
        progress = {'wrist_roll': 4.} if progress is None else progress
        def advance():
            if clock.value >= 11. - 1e-9:
                for name, degrees in progress.items():
                    packets.raw[name] = initial[name] + round(degrees * 4095 / 360)
        clock.on_sleep = advance
        return arm, packets, clock

    def clock_context(self, clock):
        stack = ExitStack()
        stack.enter_context(patch('time.monotonic', side_effect=clock.monotonic))
        stack.enter_context(patch('time.sleep', side_effect=clock.sleep))
        return stack

    def fail_move(self, arm, clock, joints=None, base_motion=None):
        with self.clock_context(clock), self.assertRaises(ArmTargetNotReached) as raised:
            arm.move(joints or {'wrist_roll': 10.}, 1000, base_motion=base_motion)
        self.assertEqual(raised.exception.feedback, arm.last_motion)
        return raised.exception.feedback

    def test_partial_progress_preserves_endpoint_and_certifies_only_measured_hold(self):
        arm, packets, clock = self.make_arm()
        feedback = self.fail_move(arm, clock)
        self.assertTrue(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'verified_measured_hold')
        self.assertFalse(feedback['reached'])
        self.assertEqual(feedback['measurement_phase'], 'before_hold')
        self.assertEqual(feedback['targets'], {'wrist_roll': 10.})
        self.assertAlmostEqual(feedback['measured']['wrist_roll'], 4., delta=.1)
        self.assertAlmostEqual(feedback['initial_joints']['wrist_roll'], 0., delta=.1)
        certificate = feedback['hold']
        self.assertTrue(certificate['verified'])
        self.assertEqual(certificate['revision'], ARM_CONTROL_REVISION)
        self.assertEqual(certificate['calibration_id'], arm.calibration_id)
        self.assertEqual(set(certificate['held_goal_raw']), {name for name, _ in ARM_JOINTS})
        self.assertEqual(certificate['held_goal_raw']['wrist_roll'], feedback['measured_raw']['wrist_roll'])
        self.assertNotEqual(certificate['held_goal_raw']['wrist_roll'], feedback['target_raw']['wrist_roll'])
        self.assertEqual(certificate['stability']['sample_count'], 3)
        self.assertGreaterEqual(certificate['stability']['interval_s'], .2 - 1e-9)
        self.assertTrue(all(value == 0 for value in certificate['stability']['max_drift'].values()))
        self.assertTrue(all(mid == 5 for mid, _, _, _ in packets.writes))
        # 1 stale-goal clear, 50 interpolation goals, 1 measured hold. No replay.
        self.assertEqual(sum(address == 42 for _, address, _, _ in packets.writes), 52)
        self.assertEqual(sum(address == 40 for _, address, _, _ in packets.writes), 1)
        self.assertFalse(arm._pending)
        self.assertTrue(feedback['progress']['wrist_roll']['before_target'])
        json.dumps(feedback, allow_nan=False)

    def test_orin_incomplete_and_reconciliation_have_no_new_goal_writes(self):
        arm, packets, clock = self.make_arm()
        robot = OrinRobot(mock=True)
        robot.arm = arm
        with self.clock_context(clock):
            result = robot.execute_guarded({'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 1000})
            writes = list(packets.writes)
            verified = robot.reconcile_arm_hold(result['arm_motion']['hold'])
        self.assertEqual(result['outcome'], 'incomplete')
        self.assertIs(result['command_sent'], True)
        self.assertIs(result['arm_reached'], False)
        self.assertIs(result['arm_hold_verified'], True)
        self.assertTrue(verified['verified'])
        self.assertEqual(packets.writes, writes)
        result['arm_motion']['measured']['wrist_roll'] = -99
        self.assertNotEqual(result['arm_motion'], arm.last_motion)

    def test_stall_small_progress_reverse_and_overshoot_are_never_recoverable(self):
        for target, progress in ((10., 0.), (3.5, .8), (20., 2.), (10., -4.), (6., 9.), (10., 22.)):
            with self.subTest(target=target, progress=progress):
                arm, packets, clock = self.make_arm({'wrist_roll': progress})
                feedback = self.fail_move(arm, clock, {'wrist_roll': target})
                self.assertFalse(feedback['recovery_eligible'])
                self.assertEqual(feedback['recovery_reason'], 'insufficient_joint_progress')
                self.assertIsNone(feedback['hold'])
                self.assertEqual(sum(address == 42 for _, address, _, _ in packets.writes), 52)

    def test_every_failed_joint_must_make_progress(self):
        arm, packets, clock = self.make_arm({'wrist_roll': 4., 'elbow_flex': 0.})
        feedback = self.fail_move(arm, clock, {'wrist_roll': 10., 'elbow_flex': 10.})
        self.assertFalse(feedback['recovery_eligible'])
        self.assertTrue(feedback['progress']['wrist_roll']['eligible'])
        self.assertFalse(feedback['progress']['elbow_flex']['eligible'])

    def test_negative_direction_progress_is_supported(self):
        arm, packets, clock = self.make_arm({'wrist_roll': -4.})
        feedback = self.fail_move(arm, clock, {'wrist_roll': -10.})
        self.assertTrue(feedback['recovery_eligible'])
        self.assertTrue(feedback['progress']['wrist_roll']['before_target'])

    def test_gripper_and_combined_base_requests_are_not_recoverable(self):
        arm, packets, clock = self.make_arm()
        feedback = self.fail_move(arm, clock, {'wrist_roll': 10., 'gripper': 70.})
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'gripper_command_not_recoverable')
        arm, packets, clock = self.make_arm()
        feedback = self.fail_move(arm, clock, base_motion=MagicMock())
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'base_motion_not_recoverable')

    def test_profile_ack_recovery_or_initial_read_retry_disqualifies_recovery(self):
        arm, packets, clock = self.make_arm()
        packets.write_faults[46] = [(-6, 0, True)]
        feedback = self.fail_move(arm, clock)
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'communication_not_fully_acknowledged')
        arm, packets, clock = self.make_arm()
        read = packets.read2ByteTxRx
        first = True
        def retrying_read(port, mid, address):
            nonlocal first
            if first:
                first = False
                return 0, -6, 0
            return read(port, mid, address)
        packets.read2ByteTxRx = retrying_read
        feedback = self.fail_move(arm, clock)
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'communication_not_fully_acknowledged')

    def test_all_six_health_mode_temperature_offset_and_calibration_are_checked(self):
        for address, value in ((40, 0), (33, 1), (65, 4), (63, 65), (31, 2047)):
            with self.subTest(address=address):
                arm, packets, clock = self.make_arm()
                original = arm._read
                # An uncommanded joint can still invalidate the all-joint hold.
                arm._read = lambda mid, addr, size=2: value if mid == 2 and addr == address else original(mid, addr, size)
                feedback = self.fail_move(arm, clock)
                self.assertFalse(feedback['recovery_eligible'])
                self.assertEqual(feedback['recovery_reason'], 'hold_verification_failed')
        arm, packets, clock = self.make_arm()
        packets.raw['elbow_flex'] = arm.calibration['elbow_flex']['range_max'] + 1
        feedback = self.fail_move(arm, clock)
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'initial_calibration_invalid')

    def test_goal_readback_transport_and_hold_ack_failure_keep_uncertainty(self):
        for failure in ('goal_mismatch', 'read_error', 'hold_write_error'):
            with self.subTest(failure=failure):
                arm, packets, clock = self.make_arm()
                if failure == 'goal_mismatch':
                    packets.read_faults[42] = [(status_packet(1, [0, 0]), 0, 0)]
                elif failure == 'read_error':
                    packets.read_faults[65] = [(None, -6, 0)]
                else:
                    original_write = packets.write
                    def failed_hold(port, mid, address, value, size):
                        if address == 42 and clock.value >= 13. - 1e-9:
                            packets.writes.append((mid, address, value, size))
                            return -6, 0
                        return original_write(port, mid, address, value, size)
                    packets.write = failed_hold
                robot = OrinRobot(mock=True)
                robot.arm = arm
                with self.clock_context(clock), self.assertRaises(OSError):
                    robot.execute_guarded({'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 1000})
                self.assertFalse(arm.last_motion['recovery_eligible'])
                self.assertIn(arm.last_motion['recovery_reason'], ('hold_verification_failed', 'hold_failed'))
                self.assertIsNone(arm.last_motion['hold'])

    def test_moving_hold_is_rejected_despite_being_within_endpoint_tolerance(self):
        arm, packets, clock = self.make_arm()
        advance = clock.on_sleep
        def drift():
            advance()
            if clock.value >= 13.1 - 1e-9:
                packets.raw['wrist_roll'] += 8  # .703 degrees, below arrival tolerance but not stable.
        clock.on_sleep = drift
        feedback = self.fail_move(arm, clock)
        self.assertFalse(feedback['recovery_eligible'])
        self.assertEqual(feedback['recovery_reason'], 'hold_verification_failed')
        self.assertIn('not stable', feedback['hold_error'])

    def test_read_only_reconcile_rejects_changed_calibration_goal_health_offset_and_drift(self):
        for change in ('calibration', 'goal', 'health', 'offset', 'drift', 'malformed'):
            with self.subTest(change=change):
                arm, packets, clock = self.make_arm()
                feedback = self.fail_move(arm, clock)
                certificate = deepcopy(feedback['hold'])
                self.assertTrue(certificate['verified'])
                clock.on_sleep = lambda: None
                if change == 'calibration':
                    certificate['calibration_id'] = 'different'
                elif change == 'goal':
                    packets.registers[5, 42] += 1
                elif change == 'health':
                    packets.registers[6, 40] = 0
                elif change == 'offset':
                    original = arm._read
                    arm._read = lambda mid, address, size=2: 2047 if address == 31 else original(mid, address, size)
                elif change == 'drift':
                    # Shift the certificate rather than the goal so the fresh
                    # encoder is stable and at its goal but not at the old pose.
                    certificate['measured']['wrist_roll'] -= 2.1
                else:
                    certificate['measured']['wrist_roll'] = float('nan')
                writes = list(packets.writes)
                with self.clock_context(clock), self.assertRaises(OSError):
                    arm.reconcile_hold(certificate)
                self.assertEqual(packets.writes, writes)

    def test_generic_transport_failure_cannot_become_incomplete(self):
        robot = OrinRobot(mock=True)
        robot.arm.move = Mock(side_effect=ArmTransportError('write', 5, 42, -6, 0))
        with self.assertRaises(ArmTransportError):
            robot.execute_guarded({'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 1000})


if __name__ == '__main__':
    unittest.main()
