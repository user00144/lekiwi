"""Certify recoverable rejections at the real driver's no-write boundary."""
from copy import deepcopy
import time
import unittest
from unittest.mock import MagicMock, Mock, patch

import test_arm_motion
from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.drivers.arm import raw_to_unit
from lekiwi_runtime.orin import OrinRobot
from lekiwi_runtime.robot import PreDispatchRejected


class PreDispatchTest(unittest.TestCase):
    def setUp(self):
        # Actual Orin/arm command paths, synthetic feedback only; no start/ports.
        self.robot = OrinRobot(mock=True)
        self.robot.wheels.driving = MagicMock()
        self.robot.observe = self.fresh_observation
        self.client = RobotClient(self.robot, 'test', 'inproc://unused')

    def fresh_observation(self):
        stamp = time.time()
        arm = self.robot.arm
        ego = arm.read_ego()
        ego.update(timestamp=stamp, error=None, limits=arm.limits(),
                   calibration_status=arm.calibration_status(ego['raw']))
        return {'timestamp': stamp, 'ego': ego,
                'cameras': {name: {'available': True, 'timestamp': stamp, 'age_s': 0,
                                   'data': 'synthetic-jpeg', 'error': None}
                            for name in ('front', 'wrist')},
                'radar': {'available': False}, 'imu': {'available': False}}

    def observe(self):
        return self.client.handle({'op': 'observe', 'request_id': 'observe-rpc'})

    def command(self, observation=None, **fields):
        context = (observation or self.observe())['action_context']
        return {'op': 'execute', 'request_id': 'execute-rpc', 'action': 'move',
                'action_id': 'first-action', 'reason': 'Inspect before moving',
                'observation_id': context['observation_id'], 'runtime_id': context['runtime_id'],
                'x': .05, 'y': 0, 'theta': 0, 'duration_ms': 500,
                'joints': {'wrist_roll': 5}, **fields}

    def elbow_outside(self):
        arm = self.robot.arm
        raw = arm.calibration['elbow_flex']['range_max'] + 38
        arm._mock_raw['elbow_flex'] = raw
        return raw

    def test_current_range_rejection_is_cached_and_new_observation_can_replan(self):
        raw = self.elbow_outside()
        arm = self.robot.arm
        calibration = deepcopy(arm.calibration)
        target = arm.limits()['elbow_flex']['max'] - 7
        command = self.command(joints={'elbow_flex': target})
        result = self.client.handle(command)
        self.assertEqual(result['status'], 'rejected')
        self.assertEqual(result['error'], 'Current elbow_flex is outside calibration')
        self.assertEqual(result['error_code'], 'arm_current_out_of_calibration')
        self.assertEqual(result['error_details']['raw'], raw)
        self.assertEqual(result['execution']['outcome'], 'rejected')
        self.assertIs(result['execution']['command_sent'], False)
        self.assertEqual(result['execution']['action_id'], command['action_id'])
        self.assertIsNone(result['execution']['arm_reached'])
        self.assertIsNone(result['execution']['base_stop_command_sent'])
        self.assertEqual(result['action_context']['reason'], 'observe_required')
        self.assertIsNone(self.client.guard._halt_reason)
        self.assertIsNone(arm.last_motion)
        self.robot.wheels.driving.return_value.__enter__.assert_not_called()
        # The old ID remains a rejection, and a new ID cannot reuse its lease.
        replay = self.client.handle({**command, 'request_id': 'retry-rpc'})
        self.assertTrue(replay['replayed'])
        self.assertEqual(replay['execution'], result['execution'])
        self.assertEqual(self.client.handle({**command, 'action_id': 'old-lease'})['error'],
                         'observation_mismatch')
        refreshed = self.observe()
        self.assertTrue(refreshed['action_context']['movement_allowed'])
        self.assertFalse(refreshed['observation']['ego']['calibration_status']['valid'])
        revised = self.command(refreshed, action_id='revised-wrist', x=0)
        with patch('time.sleep'):
            completed = self.client.handle(revised)
        self.assertEqual(completed['status'], 'completed')
        self.assertTrue(completed['execution']['arm_reached'])
        self.assertEqual(arm._mock_raw['elbow_flex'], raw)
        self.assertEqual(arm.calibration, calibration)

    def test_changed_elbow_target_does_not_bypass_current_range_validation(self):
        raw = self.elbow_outside()
        maximum = self.robot.arm.limits()['elbow_flex']['max']
        for index, target in enumerate((maximum - 7, maximum - 8)):
            result = self.client.handle(self.command(action_id=f'target-{index}',
                                                     joints={'elbow_flex': target}))
            self.assertEqual(result['error_code'], 'arm_current_out_of_calibration')
            self.assertFalse(result['execution']['command_sent'])
        self.robot.wheels.driving.return_value.__enter__.assert_not_called()
        self.assertEqual(self.robot.arm._mock_raw['elbow_flex'], raw)

    def test_outside_calibration_reports_raw_gripper_even_when_percent_is_clamped(self):
        arm = self.robot.arm
        self.assertTrue(arm.read_ego()['calibration_status']['valid'])
        self.elbow_outside()
        arm._mock_raw['gripper'] = arm.calibration['gripper']['range_max'] + 1
        ego = arm.read_ego()
        self.assertTrue(ego['available'])  # The encoder reading itself succeeded.
        self.assertFalse(ego['calibration_status']['valid'])
        details = {item['joint']: item for item in ego['calibration_status']['out_of_range']}
        self.assertEqual(set(details), {'elbow_flex', 'gripper'})
        self.assertGreater(details['elbow_flex']['current'], details['elbow_flex']['max'])
        self.assertGreater(details['gripper']['raw'], details['gripper']['raw_max'])
        self.assertTrue(0 <= ego['joints']['gripper'] <= 100)
        self.assertEqual(details['gripper']['unit'], '%')
        missing = arm.calibration_status({}, available=False)
        self.assertIsNone(missing['valid'])
        self.assertIn('elbow_flex', missing['missing_joints'])

    def test_live_raw_range_failure_precedes_every_write_and_base_entry(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        raw = arm.calibration['elbow_flex']['range_max'] + 38
        before = arm.read_ego.return_value
        before['raw']['elbow_flex'] = raw
        before['joints']['elbow_flex'] = raw_to_unit(raw, 'elbow_flex', arm.calibration['elbow_flex'])
        motion = MagicMock()
        with self.assertRaises(PreDispatchRejected) as raised:
            arm.move({'wrist_roll': 5, 'elbow_flex': arm.limits()['elbow_flex']['max'] - 7},
                     500, base_motion=motion)
        self.assertEqual(raised.exception.code, 'arm_current_out_of_calibration')
        arm._write.assert_not_called()
        motion.__enter__.assert_not_called()

    def test_device_state_validation_is_typed_before_writes(self):
        cases = [(33, 1, 'arm_position_mode_required'),
                 (63, 65, 'arm_temperature_limit'),
                 (31, 0x7ff, 'arm_homing_offset_mismatch')]
        for address, value, expected in cases:
            with self.subTest(expected=expected):
                arm = test_arm_motion.ArmMotionTest().make_live()
                original = arm._read.side_effect
                arm._read.side_effect = lambda mid, addr, size=2: value if addr == address else original(mid, addr, size)
                motion = MagicMock()
                with self.assertRaises(PreDispatchRejected) as raised:
                    arm.move({'wrist_roll': 5}, 500, base_motion=motion)
                self.assertEqual(raised.exception.code, expected)
                arm._write.assert_not_called()
                motion.__enter__.assert_not_called()

    def test_untyped_value_error_is_never_inferred_safe_from_message(self):
        self.robot.arm.move = Mock(side_effect=ValueError('Current elbow_flex is outside calibration'))
        result = self.client.handle(self.command())
        self.assertEqual(result['status'], 'error')
        self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
        self.assertIsNone(result['execution']['command_sent'])
        self.assertEqual(self.observe()['action_context']['reason'], 'uncertain_execution')

    def test_read_communication_failure_is_still_uncertain(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        arm._read.side_effect = OSError('serial read failed')
        self.robot.arm = arm
        result = self.client.handle(self.command())
        arm._write.assert_not_called()
        self.assertEqual(result['status'], 'error')
        self.assertIsNone(result['execution']['command_sent'])
        self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
        self.assertEqual(self.observe()['action_context']['reason'], 'uncertain_execution')

    def test_error_after_first_serial_write_keeps_uncertainty_and_no_retry(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        original = arm._write.side_effect
        count = 0

        def write(mid, address, value, size=2):
            nonlocal count
            count += 1
            original(mid, address, value, size)
            if count == 1:
                raise ValueError('driver failed after first write')

        arm._write.side_effect = write
        self.robot.arm = arm
        command = self.command()
        result = self.client.handle(command)
        writes = arm._write.call_count
        self.assertGreater(writes, 0)
        self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
        self.assertIsNone(result['execution']['command_sent'])
        self.assertTrue(self.client.handle(command)['replayed'])
        revised = self.client.handle(self.command(action_id='after-error'))
        self.assertEqual(revised['error'], 'uncertain_execution')
        self.assertEqual(arm._write.call_count, writes)

    def test_legacy_predispatch_rejection_does_not_halt_guarded_followup(self):
        self.elbow_outside()
        response = self.client.handle({'op': 'execute', 'request_id': 'legacy', 'action': 'move',
                                       'joints': {'elbow_flex': self.robot.arm.limits()['elbow_flex']['max'] - 7},
                                       'duration_ms': 500})
        self.assertEqual(response['status'], 'rejected')
        self.assertEqual(response['error_code'], 'arm_current_out_of_calibration')
        self.assertNotIn('execution', response)
        self.assertTrue(self.observe()['action_context']['movement_allowed'])
