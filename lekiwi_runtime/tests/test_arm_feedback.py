"""Endpoint settling and failure telemetry use a memory bus, never hardware."""
from copy import deepcopy
import json
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import (
    ARM_FEEDBACK_TIMEOUT_S, ArmController, ArmTargetNotReached, ArmTransportError, raw_to_unit,
)
from lekiwi_runtime.orin import OrinRobot
import test_profile_readback


class FakeClock:
    def __init__(self):
        self.value = 10.
        self.on_sleep = lambda: None

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds
        self.on_sleep()


class ArmFeedbackTest(unittest.TestCase):
    def make_arm(self):
        bus = SimpleNamespace(port='memory', port_handler=object(), close=Mock())
        arm = ArmController({**load_config()['arm'], 'p_coefficient': None}, bus=bus)
        packets = test_profile_readback.MemoryPackets(arm)
        packets.follow_goals = False
        packets.read2ByteTxRx = lambda port, mid, address: (packets.raw[packets.names[mid]], 0, 0)
        bus.packet_handler = packets
        return arm, packets, FakeClock()

    def test_servo_can_settle_after_half_second_without_resending_goal(self):
        arm, packets, clock = self.make_arm()
        writes = []
        write = packets.write

        def timed_write(port, mid, address, value, size):
            writes.append((clock.value, mid, address))
            return write(port, mid, address, value, size)

        packets.write = timed_write
        # Interpolation lasts 1 s. The motor reaches the last goal 0.75 s later.
        clock.on_sleep = lambda: packets.raw.update(wrist_roll=packets.registers[5, 42]) if clock.value >= 11.75 else None
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            arm.move({'wrist_roll': 10.}, 1000)
        self.assertGreaterEqual(clock.value, 11.75)
        self.assertLess(clock.value, 13.)
        self.assertTrue(arm.last_motion['reached'])
        self.assertEqual(len([item for item in writes if item[2] == 42]), 51)
        self.assertTrue(all(stamp <= 11. for stamp, _, _ in writes))
        self.assertFalse(arm._pending)

    def test_failure_preserves_measurement_before_hold_in_fresh_ego(self):
        arm, packets, clock = self.make_arm()
        initial_raw = dict(packets.raw)
        original_hold = arm.hold

        def hold_with_changed_measurement():
            # A later encoder sample differs from the endpoint failure sample.
            self.assertFalse(arm.last_motion['reached'])
            packets.raw['wrist_roll'] += 30
            packets.raw['gripper'] += 40
            original_hold()

        arm.hold = hold_with_changed_measurement
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            with self.assertRaises(ArmTargetNotReached) as raised:
                arm.move({'wrist_roll': 10., 'gripper': 70.}, 1000)
        feedback = arm.last_motion
        self.assertEqual(feedback, raised.exception.feedback)
        self.assertFalse(feedback['reached'])
        self.assertEqual(feedback['outcome'], 'target_not_reached')
        self.assertEqual(feedback['measurement_phase'], 'before_hold')
        self.assertEqual(feedback['failed_joints'], ['wrist_roll', 'gripper'])
        self.assertEqual(feedback['tolerances'], {'wrist_roll': 2, 'gripper': 5})
        self.assertEqual(feedback['units'], {'wrist_roll': 'deg', 'gripper': '%'})
        self.assertAlmostEqual(feedback['interpolation_elapsed_s'], 1.)
        self.assertAlmostEqual(feedback['settling_elapsed_s'], ARM_FEEDBACK_TIMEOUT_S)
        self.assertEqual(feedback['feedback_timeout_s'], 2.)
        for name, target in feedback['targets'].items():
            self.assertEqual(feedback['measured_raw'][name], initial_raw[name])
            measured = raw_to_unit(initial_raw[name], name, arm.calibration[name])
            self.assertAlmostEqual(feedback['measured'][name], measured)
            self.assertAlmostEqual(feedback['errors'][name], measured - target)
            self.assertIn(name + ': target=', str(raised.exception))
        self.assertIn('measured=', str(raised.exception))
        self.assertIn('tolerance=', str(raised.exception))
        self.assertFalse(arm._pending)
        fresh = arm.read_ego()
        self.assertTrue(fresh['available'])
        self.assertEqual(fresh['control']['revision'], 'verified-arm-reconcile-v3')
        self.assertEqual(fresh['control']['feedback_timeout_s'], 2.)
        self.assertEqual(fresh['control']['max_duration_ms'], 20000)
        self.assertEqual(fresh['last_motion'], feedback)
        self.assertNotEqual(fresh['raw']['wrist_roll'], feedback['measured_raw']['wrist_roll'])
        json.dumps(feedback, allow_nan=False)
        # Exception payload is an independent snapshot of the failure reading.
        snapshot = deepcopy(raised.exception.feedback)
        feedback['measured']['wrist_roll'] = -99
        self.assertEqual(raised.exception.feedback, snapshot)

    def test_transport_failure_is_not_misreported_as_endpoint_failure(self):
        arm, packets, clock = self.make_arm()
        packets.read_faults[56] = [([], -6, 0)]
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            with self.assertRaises(ArmTransportError):
                arm.move({'wrist_roll': 10.}, 1000)
        self.assertIsNone(arm.last_motion)
        self.assertFalse(arm._pending)

    def test_timeout_still_halts_guard_and_replays_failure_without_writes(self):
        arm, packets, clock = self.make_arm()
        robot = OrinRobot(mock=True)
        robot.arm = arm
        robot.observe = lambda: {
            'ego': arm.read_ego(),
            'cameras': {name: {'available': True, 'data': 'fake-jpeg', 'error': None,
                               'timestamp': time.time(), 'age_s': 0} for name in ('front', 'wrist')},
            'imu': {'available': False}, 'radar': {'available': False},
        }
        client = RobotClient(robot, 'memory', 'inproc://unused')
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            observed = client.handle({'op': 'observe', 'request_id': 'observe'})
            command = {'op': 'execute', 'request_id': 'execute', 'action_id': 'failed-arm',
                       'observation_id': observed['action_context']['observation_id'],
                       'runtime_id': observed['action_context']['runtime_id'], 'reason': 'Memory test',
                       'action': 'move', 'duration_ms': 1000, 'joints': {'wrist_roll': 10.}}
            result = client.handle(command)
            writes = list(packets.writes)
            retry = client.handle({**command, 'request_id': 'retry'})
            fresh = client.handle({'op': 'observe', 'request_id': 'after-failure'})
        self.assertEqual(result['status'], 'error')
        self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
        self.assertIn('wrist_roll: target=', result['error'])
        self.assertTrue(retry['replayed'])
        self.assertEqual(packets.writes, writes)
        self.assertFalse(fresh['action_context']['movement_allowed'])
        self.assertEqual(fresh['action_context']['reason'], 'uncertain_execution')
        self.assertFalse(fresh['observation']['ego']['last_motion']['reached'])


if __name__ == '__main__':
    unittest.main()
