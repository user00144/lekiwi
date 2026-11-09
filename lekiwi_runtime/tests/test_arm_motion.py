import unittest
from unittest.mock import Mock, patch
from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.arm import ArmController, raw_to_unit, ARM_JOINTS
from lekiwi_runtime.orin import OrinRobot


class ArmMotionTest(unittest.TestCase):
    def make_live(self):
        arm=ArmController(load_config()['arm'],bus=Mock())
        raw=dict(arm._mock_raw)
        initial={'available':True,'raw':dict(raw),'joints':{
            n:raw_to_unit(v,n,arm.calibration[n]) for n,v in raw.items()}}
        arm.read_ego=Mock(return_value=initial)
        names={mid:n for n,mid in ARM_JOINTS}
        def read(mid,addr,size=2):
            n=names[mid]
            if addr==56: return raw[n]
            if addr==33: return 0
            if addr==63: return 30
            if addr==31:
                val=arm.calibration[n]['homing_offset']
                return abs(val)|(0x800 if val<0 else 0)
            raise AssertionError(addr)
        def write(mid,addr,value,size=2):
            if addr==42: raw[names[mid]]=value
        arm._read=Mock(side_effect=read)
        arm._write=Mock(side_effect=write)
        return arm

    def test_mock_partial_target_persists_and_verifies(self):
        robot=OrinRobot(mock=True)
        before=robot.arm.read_ego()
        with patch('time.sleep'):
            self.assertTrue(robot.execute({'action':'move','joints':{'wrist_roll':10,'gripper':60},'duration_ms':1000}))
        after=robot.arm.read_ego()
        self.assertAlmostEqual(after['joints']['wrist_roll'],10,delta=.1)
        self.assertAlmostEqual(after['joints']['gripper'],60,delta=.1)
        self.assertEqual(after['raw']['elbow_flex'],before['raw']['elbow_flex'])
        self.assertTrue(after['last_motion']['reached'])

    def test_invalid_targets_and_excessive_speed_do_not_write(self):
        arm=self.make_live()
        for targets in ({'unknown':0},{'gripper':101},{'wrist_roll':float('nan')},{'wrist_roll':True}):
            self.assertFalse(arm.valid_target(targets,500))
        with self.assertRaisesRegex(ValueError,'speed'):
            arm.move({'wrist_roll':40},1000)
        arm._write.assert_not_called()

    def test_live_write_only_selected_joint_and_hold_torque(self):
        arm=self.make_live()
        with patch('time.sleep'):
            arm.move({'wrist_roll':10},1000)
        self.assertTrue(arm.last_motion['reached'])
        self.assertTrue(all(c.args[0]==5 for c in arm._write.call_args_list))
        self.assertFalse(any(c.args[1:3]==(40,0) for c in arm._write.call_args_list))
        self.assertEqual(arm._write.call_args_list[0].args[1],42)
        self.assertFalse(arm._pending)

    def test_wrong_mode_rejects_before_writes(self):
        arm=self.make_live()
        original=arm._read.side_effect
        arm._read.side_effect=lambda mid,addr,size=2: 1 if addr==33 else original(mid,addr,size)
        with self.assertRaisesRegex(OSError,'position mode'):
            arm.move({'wrist_roll':10},1000)
        arm._write.assert_not_called()

    def test_interruption_holds_measured_position(self):
        arm=self.make_live()
        with patch('time.sleep',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                arm.move({'wrist_roll':10},1000)
        self.assertFalse(arm._pending)
        self.assertEqual(arm._write.call_args_list[-1].args[1],42)
        self.assertFalse(any(c.args[1:3]==(40,0) for c in arm._write.call_args_list))

    def test_feedback_timeout_is_not_completion(self):
        arm=self.make_live()
        arm._write.side_effect=None  # Unresponsive physical joint.
        clock=iter(i*.1 for i in range(1000))
        with patch('time.sleep'), patch('time.monotonic',side_effect=lambda: next(clock)):
            with self.assertRaisesRegex(OSError,'not reached'):
                arm.move({'wrist_roll':10},1000)
        self.assertIsNone(arm.last_motion)
