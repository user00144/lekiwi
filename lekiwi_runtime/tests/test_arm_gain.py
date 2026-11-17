"""Position P gain is applied with readback and enforced before arm dispatch."""
import unittest

from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import ARM_JOINTS, GAIN_JOINTS, P_COEFFICIENT, ArmController
from lekiwi_runtime.robot import PreDispatchRejected


class FakeRegisters:
    def __init__(self, arm, gain):
        self.ids = dict(ARM_JOINTS)
        self.values = {}
        for name, motor_id in ARM_JOINTS:
            offset = arm.calibration[name]['homing_offset']
            self.values[(motor_id, 31)] = (-offset | 0x800) if offset < 0 else offset
            self.values[(motor_id, 33)] = 0
            self.values[(motor_id, 63)] = 30
            self.values[(motor_id, 55)] = 1
            self.values[(motor_id, P_COEFFICIENT)] = gain
        self.writes = []
        self.ignore_gain_writes = False

    def read(self, motor_id, address, size=2):
        return self.values[(motor_id, address)]

    def write(self, motor_id, address, value, size=2):
        self.writes.append((motor_id, address, value, size))
        if not (address == P_COEFFICIENT and self.ignore_gain_writes):
            self.values[(motor_id, address)] = value


def controller(gain=32):
    arm = ArmController({**load_config()['arm'], 'p_coefficient': gain})
    arm.bus.port_handler = object()
    return arm


class ArmGainTest(unittest.TestCase):
    def test_apply_writes_only_arm_joints_and_reports_before_after(self):
        arm = controller()
        registers = FakeRegisters(arm, 16)
        arm._read, arm._write = registers.read, registers.write
        arm._apply_gains()
        self.assertEqual(registers.writes, [(registers.ids[name], P_COEFFICIENT, 32, 1) for name in GAIN_JOINTS])
        self.assertIsNone(arm.gain_status['error'])
        self.assertEqual(set(arm.gain_status['joints']), set(GAIN_JOINTS))
        self.assertTrue(all(entry == {'before': 16, 'after': 32, 'eeprom_lock': 1}
                            for entry in arm.gain_status['joints'].values()))
        arm.mock = True  # Report the recorded status without another bus read.
        self.assertEqual(arm.read_ego()['control']['p_coefficient'], arm.gain_status)

    def test_matching_gain_is_not_rewritten_and_failed_readback_is_reported(self):
        arm = controller()
        registers = FakeRegisters(arm, 32)
        arm._read, arm._write = registers.read, registers.write
        arm._apply_gains()
        self.assertEqual(registers.writes, [])
        registers = FakeRegisters(arm, 16)
        registers.ignore_gain_writes = True
        arm._read, arm._write = registers.read, registers.write
        arm._apply_gains()
        self.assertIn('P readback 16 != 32', arm.gain_status['error'])

    def test_move_rejects_mismatched_gain_before_any_write(self):
        arm = controller()
        registers = FakeRegisters(arm, 16)
        arm._read, arm._write = registers.read, registers.write
        raw = {name: (cal['range_min'] + cal['range_max']) // 2 for name, cal in arm.calibration.items()}
        arm.read_ego = lambda: {'available': True, 'raw': raw, 'error': None,
                                'joints': {name: 0. for name in raw}}
        with self.assertRaises(PreDispatchRejected) as caught:
            arm.move({'elbow_flex': 1.}, 1000)
        self.assertEqual(caught.exception.code, 'arm_gain_mismatch')
        self.assertEqual(registers.writes, [])

    def test_invalid_or_disabled_gain_config(self):
        for invalid in (0, 255, 32., '32', True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                controller(invalid)
        arm = controller(None)
        arm._read = arm._write = None  # Disabled gain never touches the bus.
        arm._apply_gains()
        self.assertEqual(arm.gain_status['configured'], None)


if __name__ == '__main__':
    unittest.main()
