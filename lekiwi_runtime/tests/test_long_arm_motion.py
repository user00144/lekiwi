"""Longer arm interpolation must never extend the base motion window."""
import unittest
from unittest.mock import Mock, patch
import test_arm_motion

from lekiwi_runtime.guard import ExecutionGuard
from lekiwi_runtime.orin import OrinRobot
from lekiwi_runtime.robot import PreDispatchRejected
from test_guard import ObservedRobot
from lekiwi_runtime.client import RobotClient


class LongArmMotionTest(unittest.TestCase):
    def test_mock_arm_only_ten_seconds_keeps_other_joints_and_wheels_unchanged(self):
        robot = OrinRobot(mock=True)
        robot.wheels = Mock()
        before = robot.arm.read_ego()
        with patch('time.sleep') as sleep:
            result = robot.execute_guarded({'action': 'move', 'joints': {'wrist_roll': 100.},
                                            'duration_ms': 10000})
        sleep.assert_called_once_with(10.)
        self.assertEqual(result['outcome'], 'completed')
        self.assertTrue(result['arm_reached'])
        self.assertIsNone(result['base_stop_command_sent'])
        robot.wheels.move.assert_not_called()
        robot.wheels.driving.assert_not_called()
        after = robot.arm.read_ego()
        self.assertAlmostEqual(after['joints']['wrist_roll'], 100., delta=.1)
        for name in before['joints']:
            if name != 'wrist_roll':
                self.assertEqual(after['raw'][name], before['raw'][name])

    def test_long_live_interpolation_keeps_speed_cap_and_feedback(self):
        # Fake register bus: exercises the real driver without opening a device.
        arm = test_arm_motion.ArmMotionTest().make_live()
        with patch('time.sleep'):
            arm.move({'wrist_roll': 60.}, 3000)
        goal_writes = [call for call in arm._write.call_args_list if call.args[1] == 42]
        self.assertEqual(len(goal_writes), 151)  # Initial hold + 150 steps at 50 Hz.
        self.assertTrue(arm.last_motion['reached'])
        self.assertFalse(arm._pending)
        arm._write.assert_any_call(5, 46, round(30 * 4095 / 360))
        self.assertAlmostEqual(arm.last_motion['measured']['wrist_roll'], 60., delta=.1)

    def test_long_motion_still_rejects_speed_and_combined_base_before_writes(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        with self.assertRaisesRegex(PreDispatchRejected, 'speed'):
            arm.move({'wrist_roll': 70.}, 2000)
        arm._write.assert_not_called()
        base = Mock()
        with self.assertRaisesRegex(PreDispatchRejected, '1000'):
            arm.move({'wrist_roll': 20.}, 2000, base_motion=base)
        arm._write.assert_not_called()
        base.assert_not_called()
        arm.read_ego.assert_called_once()  # Combined rejection happens before feedback.

    def test_long_invalid_or_base_commands_never_reach_driver(self):
        robot = OrinRobot(mock=True)
        robot.arm.move = Mock()
        robot.wheels = Mock()
        base = {'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 2000}
        cases = [
            {**base, axis: value} for axis, value in [('x', .01), ('y', -.01), ('theta', .01)]
        ] + [
            {**base, 'duration_ms': value} for value in [99, 10001, True, 2000.5]
        ] + [
            {**base, 'joints': value} for value in [{}, None, {'unknown': 0}, {'wrist_roll': True}]
        ] + [
            {**base, 'x': value} for value in [True, float('nan'), float('inf'), '0']
        ] + [{'action': 'wait', 'duration_ms': 2000}]
        for command in cases:
            with self.subTest(command=command):
                self.assertFalse(robot.execute(command))
        robot.arm.move.assert_not_called()
        robot.wheels.move.assert_not_called()
        robot.wheels.driving.assert_not_called()

    def test_guard_long_arm_retains_lease_speed_and_idempotency(self):
        robot = ObservedRobot()
        client = RobotClient(robot, 'test', 'inproc://unused')
        observed = client.handle({'op': 'observe', 'request_id': 'observe'})
        command = {'op': 'execute', 'request_id': 'execute', 'action': 'move',
                   'duration_ms': 3000, 'joints': {'wrist_roll': 60.},
                   'runtime_id': observed['action_context']['runtime_id'],
                   'observation_id': observed['action_context']['observation_id'],
                   'action_id': 'long-arm', 'reason': 'Move the arm slowly'}
        result = client.handle(command)
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(len(robot.executions), 1)
        retry = client.handle({**command, 'request_id': 'retry'})
        self.assertTrue(retry['replayed'])
        self.assertEqual(len(robot.executions), 1)
        fresh = client.handle({'op': 'observe', 'request_id': 'observe-again'})
        too_fast = {**command, 'action_id': 'too-fast', 'duration_ms': 2000,
                    'joints': {'wrist_roll': -60.},
                    'observation_id': fresh['action_context']['observation_id']}
        rejected = client.handle(too_fast)
        self.assertEqual(rejected['error'], 'arm_speed_limit')
        self.assertFalse(rejected['execution']['command_sent'])
        self.assertEqual(len(robot.executions), 1)

    def test_guard_duration_applies_only_to_nonempty_zero_base_arm_moves(self):
        command = {'action': 'move', 'joints': {'wrist_roll': 1.}, 'duration_ms': 10000}
        self.assertIsNone(ExecutionGuard._validate(command))
        self.assertIsNone(ExecutionGuard._validate({**command, 'x': 0, 'y': 0., 'theta': -0.}))
        for change in ({'joints': {}}, {'joints': None}, {'x': .001}, {'theta': .001},
                       {'action': 'wait'}, {'duration_ms': 10001}, {'duration_ms': True}):
            with self.subTest(change=change):
                self.assertEqual(ExecutionGuard._validate({**command, **change}), 'invalid_duration')


if __name__ == '__main__':
    unittest.main()
