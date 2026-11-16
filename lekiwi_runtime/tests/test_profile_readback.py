"""Readback recovery uses a memory servo protocol, never physical devices."""
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import ARM_JOINTS, ArmController, ArmTransportError, raw_to_unit
from lekiwi_runtime.orin import OrinRobot


def status_packet(motor_id, data=(), error=0):
    packet = [255, 255, motor_id, len(data) + 2, error, *data]
    return packet + [(~sum(packet[2:]) & 255)]


class MemoryPackets:
    def __init__(self, arm):
        self.arm = arm
        self.names = {mid: name for name, mid in ARM_JOINTS}
        self.raw = dict(arm._mock_raw)
        self.registers = {}
        self.writes, self.reads = [], []
        self.write_faults, self.read_faults = {}, {}
        self.follow_goals = True

    def write(self, port, mid, address, value, size):
        self.writes.append((mid, address, value, size))
        faults = self.write_faults.get(address, [])
        comm, error, apply = faults.pop(0) if faults else (0, 0, True)
        if apply:
            self.registers[mid, address] = value
            if address == 42 and self.follow_goals:
                self.raw[self.names[mid]] = value
        return comm, error

    def write1ByteTxRx(self, port, mid, address, value):
        return self.write(port, mid, address, value, 1)

    def write2ByteTxRx(self, port, mid, address, value):
        return self.write(port, mid, address, value, 2)

    def txRxPacket(self, port, request):
        assert len(request) == 8 and request[3:5] == [4, 2]
        mid, address, size = request[2], request[5], request[6]
        self.reads.append((mid, address, size))
        faults = self.read_faults.get(address, [])
        if faults:
            return faults.pop(0)
        name = self.names[mid]
        if address == 33:
            value = 0
        elif address == 63:
            value = 30
        elif address == 31:
            offset = self.arm.calibration[name]['homing_offset']
            value = abs(offset) | (0x800 if offset < 0 else 0)
        elif address == 56:
            value = self.raw[name]
        else:
            value = self.registers.get((mid, address), 0)
        data = [value & 255] if size == 1 else [value & 255, value >> 8]
        return status_packet(mid, data), 0, 0


class ProfileReadbackTest(unittest.TestCase):
    def make_arm(self):
        bus = SimpleNamespace(port='memory', port_handler=SimpleNamespace(is_using=False))
        arm = ArmController(load_config()['arm'], bus=bus)
        bus.packet_handler = packets = MemoryPackets(arm)

        def read_ego():
            return {'available': True, 'error': None, 'timestamp': time.time(), 'raw': dict(packets.raw),
                    'joints': {name: raw_to_unit(value, name, arm.calibration[name])
                               for name, value in packets.raw.items()}, 'limits': arm.limits()}

        arm.read_ego = read_ego
        return arm, packets

    def make_guarded(self):
        arm, packets = self.make_arm()
        robot = OrinRobot(mock=True)
        robot.arm = arm

        def observe():
            return {'ego': arm.read_ego(),
                    'cameras': {name: {'available': True, 'data': 'synthetic-jpeg', 'error': None,
                                       'timestamp': time.time(), 'age_s': 0}
                                for name in ('front', 'wrist')},
                    'imu': {'available': False}, 'radar': {'available': False}}

        robot.observe = observe
        client = RobotClient(robot, 'memory', 'inproc://unused')
        observed = client.handle({'op': 'observe', 'request_id': 'observe'})
        context = observed['action_context']
        command = {'op': 'execute', 'request_id': 'execute', 'action_id': 'action', 'reason': 'Mock verification',
                   'runtime_id': context['runtime_id'], 'observation_id': context['observation_id'],
                   'action': 'move', 'duration_ms': 200, 'x': 0, 'y': 0, 'theta': 0,
                   'joints': {'wrist_roll': 5}}
        return arm, packets, client, command

    def test_only_exact_profile_readback_recovers_lost_or_corrupt_ack_without_resend(self):
        for address, value in ((44, 0), (46, 341)):
            for communication in (-6, -7):
                with self.subTest(address=address, communication=communication):
                    arm, packets = self.make_arm()
                    packets.write_faults[address] = [(communication, 0, True)]
                    recovery = arm._write_motion_profile(2, address, value)
                    self.assertEqual(recovery, {'motor_id': 2, 'register': address, 'requested': value,
                                                'read_value': value, 'read_attempts': 1,
                                                'communication': communication, 'recovery': 'verified_readback'})
                    self.assertEqual(packets.writes, [(2, address, value, 2)])
                    self.assertEqual(packets.reads, [(2, address, 2)])

    def test_read_attempts_are_bounded_and_mismatch_is_never_rewritten(self):
        arm, packets = self.make_arm()
        packets.write_faults[46] = [(-6, 0, True)]
        packets.read_faults[46] = [(None, -6, 0)]
        self.assertEqual(arm._write_motion_profile(2, 46, 341)['read_attempts'], 2)
        self.assertEqual(len(packets.writes), 1)
        for replies in ([(None, -6, 0), (None, -7, 0)], [(status_packet(2, [0, 0]), 0, 0)]):
            with self.subTest(replies=replies):
                arm, packets = self.make_arm()
                packets.write_faults[46] = [(-6, 0, True)]
                attempts = len(replies)
                packets.read_faults[46] = list(replies)
                with self.assertRaises(OSError):
                    arm._write_motion_profile(2, 46, 341)
                self.assertEqual(len(packets.writes), 1)
                self.assertEqual(len(packets.reads), attempts)

    def test_device_fault_generic_error_busy_and_tx_failure_are_not_recovered(self):
        for communication, device in ((0, 1), (-6, 1), (-2, 0), (-1, 0)):
            with self.subTest(communication=communication, device=device):
                arm, packets = self.make_arm()
                packets.write_faults[46] = [(communication, device, False)]
                arm.bus.port_handler.is_using = True
                with self.assertRaises(ArmTransportError):
                    arm._write_motion_profile(2, 46, 341)
                self.assertEqual(packets.reads, [])
                self.assertTrue(arm.bus.port_handler.is_using)  # No forced SDK unlock.
        arm, packets = self.make_arm()
        arm._write = Mock(side_effect=OSError('unclassified transport failure'))
        with self.assertRaisesRegex(OSError, 'unclassified'):
            arm._write_motion_profile(2, 46, 341)
        self.assertEqual(packets.reads, [])

    def test_readback_device_fault_or_invalid_scalar_is_not_retried(self):
        arm, packets = self.make_arm()
        packets.write_faults[44] = [(-6, 0, True)]
        packets.read_faults[44] = [(status_packet(2, [0, 0], error=1), 0, 1)]
        with self.assertRaises(ArmTransportError):
            arm._write_motion_profile(2, 44, 0)
        self.assertEqual(len(packets.reads), 1)
        arm._write = Mock(side_effect=ArmTransportError('write', 2, 44, -6, 0))
        arm._read = Mock(return_value=False)  # False must not equal integer setting zero.
        with self.assertRaisesRegex(OSError, 'mismatch'):
            arm._write_motion_profile(2, 44, 0)
        arm._read.assert_called_once()

    def test_strict_read_packet_validation_rejects_late_ack_and_wrong_length(self):
        valid = status_packet(2, [85, 1])
        malformed = [status_packet(2), status_packet(2, [85]), status_packet(3, [85, 1]),
                     valid[:-1] + [0], valid[:5] + [True, 1, valid[-1]], None]
        for packet in malformed:
            with self.subTest(packet=packet):
                arm, packets = self.make_arm()
                packets.read_faults[46] = [(packet, 0, 0)]
                with self.assertRaisesRegex(OSError, 'invalid response payload'):
                    arm._read(2, 46)
        arm, packets = self.make_arm()
        packets.read_faults[46] = [(valid, 0, 0)]
        self.assertEqual(arm._read(2, 46), 341)
        packets.read_faults[33] = [(status_packet(2), 0, 0)]
        with self.assertRaisesRegex(OSError, 'invalid response payload'):
            arm._read(2, 33, 1)

    def test_goal_and_torque_are_never_accepted_by_profile_recovery(self):
        for address in (40, 42):
            arm, packets = self.make_arm()
            with self.assertRaises(ValueError):
                arm._write_motion_profile(2, address, 1)
            self.assertEqual(packets.writes, [])

    def test_guard_completes_once_after_verified_profile_and_measured_endpoint(self):
        arm, packets, client, command = self.make_guarded()
        packets.write_faults[46] = [(-6, 0, True)]
        with patch('time.sleep'):
            result = client.handle(command)
        self.assertEqual(result['status'], 'completed')
        self.assertTrue(result['execution']['arm_reached'])
        self.assertTrue(result['execution']['command_sent'])
        motion = result['execution']['arm_motion']
        self.assertAlmostEqual(motion['measured']['wrist_roll'], 5, delta=.1)
        self.assertEqual(motion['communication_recoveries'][0]['register'], 46)
        self.assertEqual(sum(address == 46 for _, address, _, _ in packets.writes), 1)
        writes = list(packets.writes)
        replay = client.handle({**command, 'request_id': 'retry'})
        self.assertTrue(replay['replayed'])
        self.assertEqual(replay['execution'], result['execution'])
        self.assertEqual(packets.writes, writes)

    def test_guard_mismatch_or_failed_reads_remains_uncertain_and_halted(self):
        for replies in ([(status_packet(5, [0, 0]), 0, 0)], [(None, -6, 0), (None, -6, 0)]):
            arm, packets, client, command = self.make_guarded()
            packets.write_faults[46] = [(-6, 0, True)]
            packets.read_faults[46] = list(replies)
            result = client.handle(command)
            self.assertEqual(result['status'], 'error')
            self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
            self.assertIsNone(result['execution']['command_sent'])
            self.assertIsNone(arm.last_motion)
            self.assertFalse(any(address == 40 for _, address, _, _ in packets.writes))
            writes = list(packets.writes)
            self.assertTrue(client.handle(command)['replayed'])
            observation = client.handle({'op': 'observe', 'request_id': 'refresh'})
            self.assertEqual(observation['action_context']['reason'], 'uncertain_execution')
            self.assertEqual(packets.writes, writes)

    def test_goal_or_torque_ack_failure_still_halts_without_profile_readback(self):
        for address in (42, 40):
            arm, packets, client, command = self.make_guarded()
            packets.write_faults[address] = [(-6, 0, True)]
            result = client.handle(command)
            self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
            self.assertIsNone(result['execution']['command_sent'])
            self.assertFalse(any(register in (44, 46) for _, register, _ in packets.reads))
            self.assertIsNone(arm.last_motion)

    def test_verified_profile_does_not_replace_final_encoder_arrival_check(self):
        arm, packets, client, command = self.make_guarded()
        packets.write_faults[46] = [(-6, 0, True)]
        packets.follow_goals = False
        clock = iter(i * .05 for i in range(1000))
        with patch('time.sleep'), patch('time.monotonic', side_effect=lambda: next(clock)):
            result = client.handle(command)
        self.assertEqual(result['execution']['outcome'], 'uncertain_execution')
        self.assertIn('not reached', result['error'])
        self.assertFalse(arm.last_motion['reached'])
        self.assertEqual(arm.last_motion['outcome'], 'target_not_reached')
        self.assertEqual(arm.last_motion['failed_joints'], ['wrist_roll'])
        self.assertEqual(arm.last_motion['communication_recoveries'][0]['recovery'], 'verified_readback')
