import math
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.drivers.wheels import WheelDrive, body_to_wheel_raw, valid_motion
from lekiwi_runtime.orin import OrinRobot


class WheelTest(unittest.TestCase):
    def drive(self):
        bus = Mock()
        bus.packet_handler.ping.return_value = (0, 0, 0)
        bus.packet_handler.write1ByteTxRx.return_value = (0, 0)
        bus.packet_handler.write2ByteTxRx.return_value = (0, 0)
        return WheelDrive(bus)

    def test_kinematics(self):
        self.assertEqual(body_to_wheel_raw(0, 0, 0), {7: 0, 8: 0, 9: 0})
        forward = body_to_wheel_raw(.25, 0, 0)
        backward = body_to_wheel_raw(-.25, 0, 0)
        self.assertEqual(forward[8], 0)
        self.assertTrue(forward[7] & 0x8000)
        self.assertFalse(forward[9] & 0x8000)
        for mid in (7, 9):
            self.assertEqual(forward[mid] ^ backward[mid], 0x8000)
        rotation = body_to_wheel_raw(0, 0, 100)
        self.assertEqual(len(set(rotation.values())), 1)
        self.assertLess(rotation[7], 0x8000)
        self.assertTrue(all((v & 0x7fff) <= 3000 for v in body_to_wheel_raw(.4, .4, 150).values()))

    def test_invalid_commands_never_move(self):
        robot = OrinRobot(mock=True)
        robot.wheels = Mock()
        valid = dict(action='move', x=.1, y=0, theta=0, duration_ms=100)
        for key, value in [('x', math.nan), ('y', math.inf), ('theta', True),
                           ('x', .41), ('theta', 151), ('duration_ms', True),
                           ('duration_ms', 0), ('duration_ms', 1001)]:
            self.assertFalse(robot.execute({**valid, key: value}))
        self.assertFalse(valid_motion({**valid, 'x': .4, 'y': .4}))
        robot.wheels.move.assert_not_called()
        self.assertTrue(robot.execute(valid))
        robot.wheels.move.assert_called_once_with(.1, 0, 0, 100)

    def assert_stopped(self, drive):
        bus = drive.bus
        for motor in (7, 8, 9):
            bus.packet_handler.write2ByteTxRx.assert_any_call(bus.port_handler, motor, 46, 0)
            bus.packet_handler.write1ByteTxRx.assert_any_call(bus.port_handler, motor, 40, 0)
        self.assertFalse(drive._active)

    def test_success_and_transmit_error_stop_all_wheels(self):
        for status in (0, -1):
            drive = self.drive()
            with patch('scservo_sdk.GroupSyncWrite') as writer, patch('time.sleep'):
                writer.return_value.txPacket.return_value = status
                if status:
                    with self.assertRaises(OSError):
                        drive.move(.1, 0, 0, 100)
                else:
                    drive.move(.1, 0, 0, 100)
                self.assert_stopped(drive)
                self.assertEqual(writer.return_value.addParam.call_count, 3)

    def test_interruption_stops_and_releases(self):
        drive = self.drive()
        with patch('scservo_sdk.GroupSyncWrite') as writer, patch('time.sleep', side_effect=KeyboardInterrupt):
            writer.return_value.txPacket.return_value = 0
            with self.assertRaises(KeyboardInterrupt):
                drive.move(.1, 0, 0, 100)
        self.assert_stopped(drive)

    def test_partial_setup_failure_stops_every_wheel(self):
        drive = self.drive()
        original = drive._write
        def fail_mode(mid, addr, value, size=1):
            if mid == 8 and addr == 33:
                raise OSError('setup failed')
            return original(mid, addr, value, size)
        with patch.object(drive, '_write', side_effect=fail_mode):
            with self.assertRaises(OSError):
                drive.move(.1, 0, 0, 100)
        self.assert_stopped(drive)

    def test_failed_stop_attempts_remaining_wheels(self):
        drive = self.drive()
        drive._active = True
        drive.bus.packet_handler.write2ByteTxRx.return_value = (-1, 0)
        with self.assertRaisesRegex(OSError, 'physical state unknown'):
            drive.close()
        self.assertEqual(drive.bus.packet_handler.write2ByteTxRx.call_count, 3)
        self.assertTrue(drive._active)

    def test_shutdown_releases_even_when_no_motion_was_started(self):
        drive = self.drive()
        self.assertFalse(drive._active)
        drive.close(release_all=True)
        self.assert_stopped(drive)
