from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from lekiwi_runtime.calibrate import RangeCapture, build_calibration, run_calibration, save_calibration
from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import ARM_JOINTS, load_calibration


def captured(low=900, high=3100):
    result = RangeCapture()
    for value in (low, low, low, 2000, 2000, 2000, high, high, high):
        result.add(value)
    return result


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.original = load_calibration(self.config['arm']['calib_path'])
        self.offsets = {name: item['homing_offset'] for name, item in self.original.items()}

    def fake_arm(self):
        arm = Mock()
        names = {mid: name for name, mid in ARM_JOINTS}

        def read(mid, addr, size=2):
            if addr in (33, 40):
                return 0
            if addr == 31:
                offset = self.offsets[names[mid]]
                return abs(offset) | (0x800 if offset < 0 else 0)
            raise AssertionError((mid, addr, size))

        arm._read.side_effect = read
        return arm

    def test_range_median_ignores_single_sample_spike_and_applies_margin(self):
        capture = RangeCapture()
        for value in (1000, 1000, 1000, 900, 1000, 1000, 2000, 2000, 2000, 3000, 3000, 3000):
            capture.add(value)
        self.assertEqual(capture.limits(10), (1010, 2990))

    def test_no_motion_insufficient_samples_and_encoder_wrap_are_refused(self):
        for readings in ([2000] * 6, [1000, 3000]):
            capture = RangeCapture()
            for raw in readings:
                capture.add(raw)
            with self.assertRaises(ValueError):
                capture.limits()
        capture = RangeCapture()
        capture.add(4090)
        with self.assertRaisesRegex(ValueError, '경계'):
            capture.add(5)
        for value in (True, -1, 4096, 123.4):
            with self.assertRaises(ValueError):
                RangeCapture().add(value)

    def test_partial_update_preserves_other_ranges_and_gripper_direction(self):
        offsets = {**self.offsets, 'elbow_flex': 1122}
        result = build_calibration(self.original, {'elbow_flex': captured()}, offsets, 10)
        self.assertEqual(result['elbow_flex']['range_min'], 910)
        self.assertEqual(result['elbow_flex']['range_max'], 3090)
        self.assertEqual(result['elbow_flex']['homing_offset'], 1122)
        for name in self.original:
            if name != 'elbow_flex':
                self.assertEqual(result[name], self.original[name])
        self.assertEqual(self.original['elbow_flex']['homing_offset'], self.offsets['elbow_flex'])
        with self.assertRaisesRegex(ValueError, '측정해야'):
            build_calibration(self.original, {'elbow_flex': captured()},
                              {**offsets, 'shoulder_pan': 0})

    def test_save_validates_before_replacing_and_keeps_exact_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'follower.json'
            previous = json.dumps(self.original, indent=4).encode()
            path.write_bytes(previous)
            invalid = deepcopy(self.original)
            invalid['elbow_flex']['range_min'] = 4096
            with self.assertRaises(ValueError):
                save_calibration(path, invalid, expected_previous=previous)
            self.assertEqual(path.read_bytes(), previous)
            result = build_calibration(self.original, {'elbow_flex': captured()}, self.offsets)
            backup = save_calibration(path, result, expected_previous=previous)
            self.assertEqual(backup.read_bytes(), previous)
            self.assertEqual(load_calibration(str(path)), result)
            with self.assertRaisesRegex(RuntimeError, '변경'):
                save_calibration(path, self.original, expected_previous=previous)
            self.assertEqual(load_calibration(str(path)), result)

    def test_interactive_workflow_only_disables_arm_torque_and_saves_confirmed_ranges(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'new.json'
            arm = self.fake_arm()
            with patch('builtins.print'):
                run_calibration(self.config, output, ['elbow_flex'], 0,
                                arm_factory=lambda cfg: arm, prompt=Mock(side_effect=['', 'save']),
                                capture_fn=lambda *args: captured())
            result = load_calibration(str(output))
            self.assertEqual((result['elbow_flex']['range_min'], result['elbow_flex']['range_max']), (900, 3100))
            self.assertEqual([call.args for call in arm._write.call_args_list],
                             [(mid, 40, 0, 1) for _, mid in ARM_JOINTS])
            arm.move.assert_not_called()
            arm.bus.close.assert_called_once()

    def test_cancel_and_read_failure_never_save_or_reenable_torque(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'new.json'
            for failure in (KeyboardInterrupt(), OSError('read timeout')):
                arm = self.fake_arm()
                with patch('builtins.print'), self.assertRaises(type(failure)):
                    run_calibration(self.config, output, ['elbow_flex'], 0,
                                    arm_factory=lambda cfg: arm, prompt=lambda _: '',
                                    capture_fn=Mock(side_effect=failure))
                self.assertFalse(output.exists())
                self.assertTrue(all(call.args[1:] == (40, 0, 1) for call in arm._write.call_args_list))
                arm.bus.close.assert_called_once()
            arm = self.fake_arm()
            with patch('builtins.print'):
                run_calibration(self.config, output, ['elbow_flex'], 0, arm_factory=lambda cfg: arm,
                                prompt=Mock(side_effect=['', 'cancel']), capture_fn=lambda *args: captured())
            self.assertFalse(output.exists())

    def test_startup_cancel_never_opens_port(self):
        factory = Mock()
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            self.assertIsNone(run_calibration(self.config, Path(directory) / 'new.json', ['elbow_flex'], 0,
                                              arm_factory=factory, prompt=lambda _: 'q'))
        factory.assert_not_called()

    def test_release_failure_prevents_capture_and_save(self):
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            output = Path(directory) / 'new.json'
            arm, capture = self.fake_arm(), Mock()
            arm._write.side_effect = OSError('ACK timeout')
            with self.assertRaisesRegex(OSError, '손으로 움직이지'):
                run_calibration(self.config, output, ['elbow_flex'], 0, arm_factory=lambda cfg: arm,
                                prompt=lambda _: '', capture_fn=capture)
            capture.assert_not_called()
            self.assertFalse(output.exists())
            arm.bus.close.assert_called_once()

    def test_offset_change_during_measurement_prevents_save(self):
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            output = Path(directory) / 'new.json'
            arm = self.fake_arm()
            def capture(*args):
                self.offsets['gripper'] -= 1
                return captured()
            with self.assertRaisesRegex(RuntimeError, '영점이 변경'):
                run_calibration(self.config, output, ['elbow_flex'], 0, arm_factory=lambda cfg: arm,
                                prompt=lambda _: '', capture_fn=capture)
            self.assertFalse(output.exists())
            arm.bus.close.assert_called_once()
