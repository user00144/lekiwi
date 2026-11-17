"""No-hardware regression tests for keyboard release and motion expiry."""

import csv
import importlib.util
import json
import math
import multiprocessing as mp
from pathlib import Path
import socket
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import teleop_slam as t


class FakeRoot:
    def __init__(self):
        self.jobs = {}
        self.index = 0

    def after_idle(self, callback):
        self.index += 1
        self.jobs[self.index] = callback
        return self.index

    def after_cancel(self, job):
        self.jobs.pop(job, None)

    def flush(self):
        jobs, self.jobs = self.jobs, {}
        for callback in jobs.values():
            callback()

    def focus_displayof(self):
        return None


class TeleopTest(unittest.TestCase):
    def command(self, velocity, at=10):
        return json.dumps({"kind": "velocity", "at": at, "velocity": velocity}).encode()

    def test_holonomic_keys_and_diagonal_limit(self):
        self.assertEqual(t.key_velocity({"w"}), (.06, 0, 0))
        self.assertEqual(t.key_velocity({"s"}), (-.06, 0, 0))
        self.assertEqual(t.key_velocity({"a"}), (0, .06, 0))
        self.assertEqual(t.key_velocity({"d"}), (0, -.06, 0))
        self.assertEqual(t.key_velocity({"q"}), (0, 0, 12))
        self.assertEqual(t.key_velocity({"e"}), (0, 0, -12))
        self.assertEqual(t.key_velocity({"r"}), t.ZERO)
        self.assertNotIn("r", t.KEYS)
        self.assertEqual(t.key_velocity(set("wasdqe")), t.ZERO)
        x, y, yaw = t.key_velocity(set("waq"))
        self.assertAlmostEqual(math.hypot(x, y), .06)
        self.assertEqual(yaw, 12)

    def test_stale_malformed_and_unbounded_messages_stop(self):
        good = self.command([.06, 0, 0])
        self.assertEqual(t.decode_command(good, 10.1), (.06, 0, 0))
        self.assertEqual(t.decode_command(good, 10.31), t.ZERO)
        self.assertEqual(t.decode_command(good, 9), t.ZERO)
        for velocity in ([math.nan, 0, 0], [True, 0, 0], [.16, 0, 0], [0, 0, 31], [0, 0]):
            self.assertEqual(t.decode_command(self.command(velocity), 10), t.ZERO)
        for bad in (b"garbage", b"[]", b"{}", b"null"):
            self.assertEqual(t.decode_command(bad, 10), t.ZERO)

    def app(self):
        app = t.TeleopApp.__new__(t.TeleopApp)
        app.root = FakeRoot()
        app.args = SimpleNamespace(speed=.06, turn=12)
        app.keys, app.pending_release = set(), {}
        app.armed, app.closing = True, False
        app.arm_button, app.message, app.send = Mock(), Mock(), Mock()
        return app

    def test_x11_repeat_does_not_release_but_real_release_stops(self):
        app = self.app()
        event = SimpleNamespace(keysym="w")
        app.key_press(event)
        app.key_release(event)
        app.key_press(event)
        app.root.flush()
        self.assertEqual(app.keys, {"w"})
        app.key_release(event)
        app.root.flush()
        self.assertFalse(app.keys)
        app.send.assert_called_with(t.ZERO)

    def test_focus_loss_and_space_disarm_immediately(self):
        for method in (lambda a: a.focus_check(), lambda a: a.key_press(SimpleNamespace(keysym="space"))):
            app = self.app()
            app.keys = {"w", "q"}
            method(app)
            self.assertFalse(app.armed)
            self.assertFalse(app.keys)
            app.send.assert_called_with(t.ZERO)

    def test_slam_progress_expiry_and_lost(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tracking.csv"
            health = t.SlamHealth()
            self.assertFalse(health.update(path, 0))
            path.write_text("timestamp_s,state,state_name,track_ms,loop_fps,keypoints,pose_valid,big_map_change\n1,1,NOT_INITIALIZED,20,25,100,0,0\n")
            self.assertTrue(health.update(path, 1))
            self.assertFalse(health.update(path, 4))
            with path.open("a") as out:
                out.write("2,2,OK,20,25,100,1,0\n")
            self.assertTrue(health.update(path, 5))
            with path.open("a") as out:
                out.write("3,4,LOST,20,25,100,0,0\n")
            self.assertFalse(health.update(path, 6))

    def drive(self):
        bus = Mock()
        bus.packet_handler.ping.return_value = (0, 0, 0)
        bus.packet_handler.write1ByteTxRx.return_value = (0, 0)
        bus.packet_handler.write2ByteTxRx.return_value = (0, 0)
        return t.ContinuousWheels(bus)

    def test_config_bus_settings_and_port_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.json"
            path.write_text(json.dumps({"arm": {"port": "/dev/reference-board", "baudrate": 500000}}))
            port, baudrate = t.servo_settings(path)
            self.assertEqual(port, Path("/dev/reference-board"))
            self.assertEqual(baudrate, 500000)
            self.assertEqual(t.servo_settings(path, Path("/dev/override")), (Path("/dev/override"), 500000))

    def test_motor_packets_match_test_moving_reference(self):
        # Import definitions only: test_moving.main() and sensors are not run.
        reference_path = t.HERE.parent / "test_moving.py"
        spec = importlib.util.spec_from_file_location("_teleop_moving_reference", reference_path)
        reference = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = reference
        try:
            spec.loader.exec_module(reference)
            for velocity in ((.06, 0, 0), (-.06, 0, 0), (0, .06, 0),
                             (0, -.06, 0), (0, 0, 12), (0, 0, -12), (.03, .04, -8)):
                drive = self.drive()
                reference_drive = self.drive()
                base = reference.Base(reference_drive, dry_run=False)
                with patch("scservo_sdk.GroupSyncWrite") as group:
                    group.return_value.txPacket.return_value = 0
                    base.send(*velocity)
                    expected = group.return_value.addParam.call_args_list.copy()
                    group.return_value.addParam.reset_mock()
                    drive.set_velocity(velocity)
                    self.assertEqual(group.return_value.addParam.call_args_list, expected)
                    base.stop()
                    drive.set_velocity(t.ZERO)
                for method in ("write1ByteTxRx", "write2ByteTxRx"):
                    expected_writes = [call.args[1:] for call in getattr(reference_drive.bus.packet_handler, method).call_args_list]
                    actual_writes = [call.args[1:] for call in getattr(drive.bus.packet_handler, method).call_args_list]
                    self.assertEqual(actual_writes, expected_writes)
        finally:
            sys.modules.pop(spec.name, None)

    def test_continuous_updates_do_not_cycle_torque_or_touch_arm(self):
        drive = self.drive()
        with patch("scservo_sdk.GroupSyncWrite") as group:
            group.return_value.txPacket.return_value = 0
            drive.set_velocity((.03, 0, 0))
            setup_calls = drive.bus.packet_handler.write1ByteTxRx.call_count
            drive.set_velocity((0, .03, 12))
            self.assertEqual(drive.bus.packet_handler.write1ByteTxRx.call_count, setup_calls)
            self.assertEqual(group.return_value.txPacket.call_count, 2)
            drive.set_velocity(t.ZERO)
        for motor in (7, 8, 9):
            drive.bus.packet_handler.write2ByteTxRx.assert_any_call(drive.bus.port_handler, motor, 46, 0)
            drive.bus.packet_handler.write1ByteTxRx.assert_any_call(drive.bus.port_handler, motor, 40, 0)
        for call in drive.bus.packet_handler.write1ByteTxRx.call_args_list:
            self.assertIn(call.args[1], (7, 8, 9))

    def test_transmit_failure_attempts_stop_on_every_wheel(self):
        drive = self.drive()
        with patch("scservo_sdk.GroupSyncWrite") as group:
            group.return_value.txPacket.return_value = -1
            with self.assertRaises(OSError):
                drive.set_velocity((.03, 0, 0))
        for motor in (7, 8, 9):
            drive.bus.packet_handler.write2ByteTxRx.assert_any_call(drive.bus.port_handler, motor, 46, 0)

    def test_motor_process_stops_when_gui_heartbeat_disappears(self):
        # A real subprocess, but mocked wheels: no Serial or camera is opened.
        context = mp.get_context("spawn")
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        parent.setblocking(False)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "commands.csv"
            process = context.Process(target=t.motor_worker, args=(child, None, True, str(path)))
            process.start()
            child.close()
            try:
                deadline = time.monotonic() + 5
                while not path.exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(path.exists())
                for _ in range(5):
                    t.send_packet(parent, {"kind": "velocity", "at": time.monotonic(), "velocity": [.06, 0, 0]})
                    time.sleep(.05)
                # Simulate a frozen GUI: no more commands are sent.
                time.sleep(.55)
                with path.open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertTrue(any(float(r["vx_m_s"]) > 0 for r in rows))
                self.assertTrue(all(float(rows[-1][key]) == 0 for key in ("vx_m_s", "vy_m_s", "theta_deg_s")))
            finally:
                t.send_packet(parent, {"kind": "quit"})
                process.join(3)
                parent.close()
            self.assertFalse(process.is_alive())
            self.assertEqual(process.exitcode, 0)


if __name__ == "__main__":
    unittest.main()
