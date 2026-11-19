"""In-motion arm monitor on a fake servo bus and fake clock: stop, hold, verify; never stop on noise."""
from contextlib import ExitStack
from copy import deepcopy
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import ARM_JOINTS, ArmController, ArmMonitorStopped
from lekiwi_runtime.guard import CAPABILITIES, JOINT_NAMES, ExecutionGuard
from lekiwi_runtime.orin import OrinRobot
from lekiwi_runtime.robot import PreDispatchRejected
from test_arm_feedback import FakeClock
from test_profile_readback import MemoryPackets
from test_reconciliation import ReconciliationRobot

HELD, EMPTY, THRESHOLD = 27.7, 26.2, 27.0  # measured on kiwi-01: cube between the jaws vs closed empty


def percent_raw(arm, percent):
    cal = arm.calibration['gripper']
    share = (100 - percent if cal.get('drive_mode', 0) else percent) / 100
    return round(cal['range_min'] + share * (cal['range_max'] - cal['range_min']))


class ArmMonitorTest(unittest.TestCase):
    def make_arm(self):
        bus = SimpleNamespace(port='memory', port_handler=object(), close=Mock())
        arm = ArmController({**load_config()['arm'], 'p_coefficient': None}, bus=bus)
        packets = MemoryPackets(arm)  # joints follow written goals
        packets.read2ByteTxRx = lambda port, mid, address: (packets.raw[packets.names[mid]], 0, 0)
        bus.packet_handler = packets
        packets.raw['gripper'] = percent_raw(arm, HELD)
        for name, mid in ARM_JOINTS:  # an already powered arm holding a closed gripper
            packets.registers[mid, 40] = 1
            packets.registers[mid, 42] = packets.raw[name]
        packets.registers[6, 42] = percent_raw(arm, 26.0)
        return arm, packets, FakeClock()

    def run_move(self, arm, clock, monitor, joints=None):
        with ExitStack() as stack:
            stack.enter_context(patch('time.monotonic', side_effect=clock.monotonic))
            stack.enter_context(patch('time.sleep', side_effect=clock.sleep))
            return arm.move(joints or {'wrist_roll': 10.}, 1000, monitor=monitor)

    def goal_writes(self, packets, mid=5):
        return sum(1 for m, address, _, _ in packets.writes if m == mid and address == 42)

    def test_object_lost_mid_motion_stops_holds_and_verifies(self):
        arm, packets, clock = self.make_arm()
        clock.on_sleep = lambda: packets.raw.update(gripper=percent_raw(arm, EMPTY)) if clock.value >= 10.4 else None
        with self.assertRaises(ArmMonitorStopped) as raised:
            self.run_move(arm, clock, {'gripper_min_percent': THRESHOLD})
        feedback = raised.exception.feedback
        self.assertEqual(feedback, arm.last_motion)
        stop = feedback['monitor_stop']
        self.assertEqual(stop['condition'], 'gripper_below_min')
        self.assertEqual(stop['consecutive_samples'], 2)
        self.assertLess(stop['gripper_percent'], THRESHOLD)
        self.assertLess(stop['progress_fraction'], .6)  # stopped well before the end of the motion
        self.assertFalse(feedback['reached'])
        self.assertTrue(feedback['hold']['verified'])
        self.assertEqual(feedback['hold']['calibration_id'], arm.calibration_id)
        # 1 stale-goal clear + interpolation goals up to the stop + 1 measured hold; no replay.
        self.assertEqual(self.goal_writes(packets), 1 + stop['step'] + 1)
        self.assertFalse(arm._pending)
        self.assertFalse(any(address == 42 for mid, address, _, _ in packets.writes if mid == 6))  # gripper untouched

    def test_held_object_completes_with_telemetry(self):
        arm, packets, clock = self.make_arm()
        self.run_move(arm, clock, {'gripper_min_percent': THRESHOLD})
        motion = arm.last_motion
        self.assertTrue(motion['reached'])
        telemetry = motion['monitor']
        self.assertGreater(telemetry['gripper_samples'], 10)
        self.assertAlmostEqual(telemetry['gripper_min_seen'], HELD, delta=.1)
        self.assertIn('wrist_roll', telemetry['max_following_error_deg'])
        self.assertEqual(telemetry['read_errors'], 0)
        self.assertEqual(self.goal_writes(packets), 51)

    def test_one_low_reading_or_failed_reads_never_stop(self):
        arm, packets, clock = self.make_arm()
        dips = iter([True])
        def dip():  # a single low sample at ~0.4 s, then the object reads as held again
            low = clock.value >= 10.4 and next(dips, False)
            packets.raw['gripper'] = percent_raw(arm, EMPTY if low else HELD)
        clock.on_sleep = dip
        packets.read_faults[56] = [(None, -3001, 0)] * 2  # two gripper read timeouts
        self.run_move(arm, clock, {'gripper_min_percent': THRESHOLD})
        self.assertTrue(arm.last_motion['reached'])
        self.assertEqual(arm.last_motion['monitor']['read_errors'], 2)

    def test_telemetry_only_monitor_never_stops(self):
        arm, packets, clock = self.make_arm()
        packets.raw['gripper'] = percent_raw(arm, EMPTY)
        self.run_move(arm, clock, {'gripper_min_percent': None})
        self.assertTrue(arm.last_motion['reached'])
        self.assertEqual(arm.last_motion['monitor']['gripper_samples'], 0)
        self.assertGreater(arm.last_motion['monitor']['telemetry_samples'], 0)

    def test_invalid_monitor_is_rejected_before_any_write(self):
        arm, packets, clock = self.make_arm()
        for monitor, joints in (({'gripper_min_percent': 27.}, {'gripper': 30.}),
                                ({'unknown': 1}, None), ({'gripper_min_percent': True}, None),
                                ({'gripper_min_percent': 0}, None), ({'gripper_min_percent': float('nan')}, None)):
            with self.subTest(monitor=monitor, joints=joints), self.assertRaises(PreDispatchRejected):
                self.run_move(arm, clock, monitor, joints)
        self.assertEqual(packets.writes, [])

    def test_orin_reports_a_verified_stop_as_stopped(self):
        robot = OrinRobot(mock=True)
        arm, packets, clock = self.make_arm()
        robot.arm, robot.wheels = arm, Mock()
        clock.on_sleep = lambda: packets.raw.update(gripper=percent_raw(arm, EMPTY)) if clock.value >= 10.4 else None
        command = {'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 1000,
                   'monitor': {'gripper_min_percent': THRESHOLD}}
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            result = robot.execute_guarded(command)
        self.assertEqual(result['outcome'], 'stopped')
        self.assertTrue(result['command_sent'])
        self.assertFalse(result['arm_reached'])
        self.assertTrue(result['arm_hold_verified'])
        self.assertEqual(result['monitor_stop'], arm.last_motion['monitor_stop'])
        for bad in ({**command, 'x': .1}, {**command, 'monitor': None},
                    {'action': 'wait', 'duration_ms': 10, 'monitor': {}}):
            with self.subTest(command=bad):
                self.assertFalse(robot.execute(bad))


class StoppingRobot(ReconciliationRobot):
    """Memory-only robot whose arm moves stop by monitor with a verified hold."""
    def __init__(self):
        super().__init__()
        self.incomplete = False
        self.stop = True

    def execute_guarded(self, command):
        if not (self.stop and (command.get('monitor') or {}).get('gripper_min_percent') is not None):
            return super().execute_guarded(command)
        self.executions.append(deepcopy(command))
        for name, target in command['joints'].items():
            self.joints[name] = (self.joints[name] + target) / 2
        stop = {'condition': 'gripper_below_min', 'gripper_percent': 26.2,
                'gripper_min_percent': command['monitor']['gripper_min_percent'], 'consecutive_samples': 2,
                'elapsed_s': .48, 'step': 24, 'steps': 50, 'progress_fraction': .48}
        result = {'outcome': 'stopped', 'command_sent': True, 'arm_reached': False, 'arm_hold_verified': True,
                  'monitor_stop': stop, 'arm_motion': {'targets': dict(command['joints']), 'reached': False,
                                                       'outcome': 'monitor_stop', 'monitor_stop': dict(stop),
                                                       'hold': self.certificate()}}
        if self.execution_mutator:
            self.execution_mutator(result)
        return result


class GuardMonitorTest(unittest.TestCase):
    def setUp(self):
        self.robot = StoppingRobot()
        self.client = RobotClient(self.robot, 'memory', 'inproc://unused')
        self.guard = self.client.guard
        self.guard.post_timeout_s = .025

    def command(self, **changes):
        context = self.client.handle({'op': 'observe', 'request_id': 'observation'})['action_context']
        return {'request_id': 'rpc', 'op': 'execute', 'action': 'move', 'action_id': f'action-{time.monotonic_ns()}',
                'runtime_id': self.guard.runtime_id, 'observation_id': context['observation_id'],
                'reason': 'Carry the held cube', 'joints': {'wrist_roll': 6.}, 'duration_ms': 1000,
                'monitor': {'gripper_min_percent': THRESHOLD}, **changes}

    def test_capability_is_advertised(self):
        self.assertEqual(CAPABILITIES.get('arm_motion_monitor'), 1)

    def test_verified_stop_returns_a_fresh_observation_and_keeps_admission_open(self):
        response = self.client.handle(self.command())
        self.assertEqual(response['status'], 'stopped', response)
        self.assertEqual(response['execution']['monitor_stop']['condition'], 'gripper_below_min')
        self.assertIn('observation', response)
        self.assertTrue(response['action_context']['movement_allowed'])
        self.assertIsNone(self.guard.legacy_block_reason)
        follow = self.client.handle(self.command(joints={'wrist_roll': 4.}, monitor={'gripper_min_percent': None}))
        self.assertEqual(follow['status'], 'completed', follow)

    def test_inconsistent_stop_evidence_halts_as_uncertain(self):
        for mutate in (lambda r: r['monitor_stop'].update(condition='made_up'),
                       lambda r: r['arm_motion']['hold'].update(verified=False),
                       lambda r: r['arm_motion']['hold'].update(timestamp=time.time() - 5),
                       lambda r: r.update(arm_reached=True)):
            with self.subTest(mutate=mutate):
                self.setUp()
                self.robot.execution_mutator = mutate
                response = self.client.handle(self.command())
                self.assertEqual(response['status'], 'error', response)
                self.assertEqual(response['execution']['outcome'], 'uncertain_execution')
                self.assertEqual(self.guard.legacy_block_reason, 'uncertain_execution')

    def test_state_change_after_the_stop_halts(self):
        observe = self.robot.observe
        def drifted():
            observation = observe()
            if self.robot.executions:
                observation['ego']['joints']['wrist_roll'] += 3.
            return observation
        self.robot.observe = drifted
        response = self.client.handle(self.command())
        self.assertEqual(response['status'], 'stopped_without_observation', response)
        self.assertEqual(self.guard.legacy_block_reason, 'uncertain_execution')

    def test_monitor_is_only_valid_on_arm_only_moves(self):
        base = {'action': 'move', 'joints': {'wrist_roll': 1.}, 'duration_ms': 1000,
                'monitor': {'gripper_min_percent': THRESHOLD}}
        self.assertIsNone(ExecutionGuard._validate(base))
        self.assertIsNone(ExecutionGuard._validate({**base, 'monitor': {'gripper_min_percent': None}}))
        for change in ({'x': .1}, {'joints': {'gripper': 30.}}, {'monitor': None}, {'monitor': {'other': 1}},
                       {'monitor': {'gripper_min_percent': 120}}, {'action': 'wait', 'joints': None}):
            with self.subTest(change=change):
                self.assertIn(ExecutionGuard._validate({**base, **change}),
                              ('invalid_monitor', 'invalid_duration', 'wait_cannot_move'))
        self.assertEqual(set(JOINT_NAMES), {name for name, _ in ARM_JOINTS})


if __name__ == '__main__':
    unittest.main()


class LoadSagHoldTest(unittest.TestCase):
    """A held cube makes the elbow settle ~2.2 deg short of every goal, including the hold goal itself."""
    SAG_DEG = 2.2

    def make_sagging_arm(self):
        bus = SimpleNamespace(port='memory', port_handler=object(), close=Mock())
        arm = ArmController({**load_config()['arm'], 'p_coefficient': None}, bus=bus)
        packets = MemoryPackets(arm)
        packets.follow_goals = False
        packets.read2ByteTxRx = lambda port, mid, address: (packets.raw[packets.names[mid]], 0, 0)
        bus.packet_handler = packets
        for name, mid in ARM_JOINTS:
            packets.registers[mid, 40] = 1
            packets.registers[mid, 42] = packets.raw[name]
        sag = round(self.SAG_DEG * 4095 / 360)
        write = packets.write

        def sagging_write(port, mid, address, value, size):
            result = write(port, mid, address, value, size)
            if address == 42 and packets.names[mid] == 'wrist_roll':
                packets.raw['wrist_roll'] = value - sag  # gravity: the joint settles short of its goal
            return result
        packets.write = sagging_write
        return arm, packets, FakeClock()

    def test_post_miss_hold_accepts_a_stable_sag_and_reports_a_warning(self):
        robot = OrinRobot(mock=True)
        arm, packets, clock = self.make_sagging_arm()
        robot.arm, robot.wheels = arm, Mock()
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            result = robot.execute_guarded({'action': 'move', 'joints': {'wrist_roll': 10.}, 'duration_ms': 1000})
        self.assertEqual(result['outcome'], 'warning', result)
        hold = result['arm_motion']['hold']
        self.assertTrue(hold['verified'])
        self.assertEqual(hold['goal_tolerance_deg'], 10.)
        self.assertAlmostEqual(result['arm_motion']['hold_target_errors']['wrist_roll'], -2 * self.SAG_DEG, delta=.2)
        # A later read-only re-check uses the tolerance recorded in the certificate.
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            self.assertTrue(arm.reconcile_hold(hold)['verified'])

    def test_normal_hold_check_keeps_two_degrees(self):
        arm, packets, clock = self.make_sagging_arm()
        held = {'wrist_roll': packets.raw['wrist_roll']}
        arm._pending.add(5)
        arm._write(5, 42, held['wrist_roll'])  # goal written; the joint settles 2.2 deg short of it
        with patch('time.monotonic', side_effect=clock.monotonic), patch('time.sleep', side_effect=clock.sleep):
            with self.assertRaisesRegex(OSError, 'did not reach measured goal'):
                arm.verify_hold(held, passive_reference={'gripper': 50.})
            with self.assertRaisesRegex(OSError, 'goal tolerance'):
                arm.verify_hold(held, goal_tolerance_deg=12.)
            certificate = arm.verify_hold(held, goal_tolerance_deg=10.)
        self.assertEqual(certificate['goal_tolerance_deg'], 10.)
