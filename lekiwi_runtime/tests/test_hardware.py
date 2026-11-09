import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.config import DATA_DIR, load_config
from lekiwi_runtime.drivers.arm import ArmController, load_calibration, raw_to_unit
from lekiwi_runtime.drivers import radar_protocol as protocol
from lekiwi_runtime.orin import OrinRobot


def radar_packet(points=((1., 2., 3., 4.),), snr=True):
    payload = b"".join(struct.pack("<4f", *point) for point in points)
    tlvs = struct.pack("<II", 1, len(payload)) + payload
    if snr:
        side = b"".join(struct.pack("<2h", 185, 20) for _ in points)
        tlvs += struct.pack("<II", 7, len(side)) + side
    size = ((40 + len(tlvs) + 31) // 32) * 32
    header = struct.pack("<8s8I", protocol.MAGIC, 1, size, 0, 42, 0,
                         len(points), 2 if snr else 1, 0)
    return (header + tlvs).ljust(size, b"\0")


class RadarTest(unittest.TestCase):
    def test_points_snr_and_padding(self):
        packet = radar_packet()
        self.assertTrue(protocol.packet_is_valid(packet))
        frame, count, tlvs = protocol.parse_header(packet)
        self.assertEqual(frame, 42)
        points, snr = protocol.parse_packet(packet, count, tlvs)
        self.assertEqual(points, [(1., 2., 3., 4.)])
        self.assertEqual(snr, [18.5])

    def test_missing_side_info_and_empty_frame(self):
        for points in ((), ((1., 2., 3., 4.),)):
            packet = radar_packet(points, snr=False)
            self.assertTrue(protocol.packet_is_valid(packet))
            _, count, tlvs = protocol.parse_header(packet)
            decoded, snr = protocol.parse_packet(packet, count, tlvs)
            self.assertEqual(decoded, list(points))
            self.assertEqual(snr, [None] * len(points))

    def test_corrupt_lengths_and_truncation_are_rejected(self):
        packet = radar_packet()
        for length in range(len(packet)):
            self.assertFalse(protocol.packet_is_valid(packet[:length]))
        for offset, value in ((12, 999), (32, 99), (44, 999), (44, 8)):
            corrupted = bytearray(packet)
            struct.pack_into("<I", corrupted, offset, value)
            self.assertFalse(protocol.packet_is_valid(corrupted))


class ConfigurationTest(unittest.TestCase):
    def test_defaults_and_relative_overrides(self):
        config = load_config()
        self.assertTrue(Path(config["radar"]["cfg_path"]).is_file())
        self.assertTrue(Path(config["arm"]["calib_path"]).is_file())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.json'
            path.write_text(json.dumps({"arm": {"calib_path": "calibration.json"},
                                        "cameras": {"front": {"device": "/dev/video8"}}}))
            loaded = load_config(path)
            self.assertEqual(loaded["arm"]["calib_path"], str(path.parent / 'calibration.json'))
            self.assertEqual(loaded["cameras"]["wrist"], config["cameras"]["wrist"])
            self.assertEqual(loaded["radar"]["cfg_path"], config["radar"]["cfg_path"])
            path.write_text('{"arm": {"potr": "wrong"}}')
            with self.assertRaises(ValueError):
                load_config(path)

    def test_calibrated_units(self):
        cal = {"range_min": 1000, "range_max": 3000, "drive_mode": 1}
        self.assertEqual(raw_to_unit(2000, "shoulder_pan", cal), 0)
        self.assertEqual(raw_to_unit(1000, "gripper", cal), 100)
        self.assertEqual(raw_to_unit(4000, "gripper", cal), 0)
        self.assertEqual(len(load_calibration(str(DATA_DIR / 'follower.json'))), 6)


class HardwareTest(unittest.TestCase):
    def test_complete_mock_observation_and_cleanup(self):
        with OrinRobot(mock=True) as robot:
            observation = robot.observe()
            self.assertTrue(observation["simulated"])
            for camera in observation["cameras"].values():
                self.assertTrue(camera["available"], camera["error"])
                self.assertTrue(camera["data"])
            self.assertTrue(observation["radar"]["available"])
            self.assertTrue(observation["ego"]["available"])
            self.assertEqual(len(observation["ego"]["joints"]), 6)
            self.assertTrue(observation["imu"]["available"])
            self.assertTrue(observation["imu"]["calibrated"])
            json.dumps(observation, allow_nan=False)
            self.assertTrue(robot.execute({"action": "wait", "duration_ms": 0}))
            for duration in (True, -1, 1001, 1.5, None):
                self.assertFalse(robot.execute({"action": "wait", "duration_ms": duration}))
            self.assertFalse(robot.execute({"action": "move", "duration_ms": 0}))
        for sensor in [*robot.cameras.values(), robot.radar]:
            self.assertFalse(sensor._thread.is_alive())

    def test_disabled_devices_do_not_open_hardware(self):
        config = load_config()
        for sensor in [*config["cameras"].values(), config["radar"], config["arm"], config["imu"]]:
            sensor["enabled"] = False
        with patch('serial.Serial', side_effect=AssertionError('opened hardware')):
            with OrinRobot(config) as robot:
                observation = robot.observe()
                self.assertFalse(observation["radar"]["available"])
                self.assertFalse(observation["ego"]["available"])
                self.assertFalse(observation["cameras"]["front"]["available"])

    def test_partial_startup_closes_all_devices(self):
        robot = OrinRobot(mock=True)
        first, broken = Mock(), Mock()
        broken.start.side_effect = RuntimeError('startup failed')
        robot._sensors = [first, broken]
        with self.assertRaisesRegex(RuntimeError, 'startup failed'):
            robot.__enter__()
        first.close.assert_called_once()
        broken.close.assert_called_once()

    def test_servo_device_errors_are_not_success(self):
        arm = ArmController(load_config()["arm"])
        arm.bus.port_handler, arm.bus.packet_handler = Mock(), Mock()
        bus = arm.bus.port_handler
        arm.bus.packet_handler.read2ByteTxRx.return_value = (2000, 0, 1)
        result = arm.read_ego()
        self.assertFalse(result["available"])
        self.assertEqual(result["joints"], {})
        bus.closePort.assert_called_once()
        self.assertIsNone(arm.bus.port_handler)

    def test_failed_baudrate_releases_port(self):
        arm = ArmController(load_config()["arm"])
        with patch('scservo_sdk.PortHandler') as handler:
            handler.return_value.openPort.return_value = True
            handler.return_value.setBaudRate.return_value = False
            with self.assertRaises(OSError):
                arm.bus._open('/dev/fake')
            handler.return_value.closePort.assert_called_once()
