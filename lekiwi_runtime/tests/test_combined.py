from contextlib import contextmanager
import signal
import unittest
from unittest.mock import MagicMock, Mock, patch
import test_arm_motion
from lekiwi_runtime import main
from lekiwi_runtime.orin import OrinRobot


class CombinedTest(unittest.TestCase):
    def test_base_active_during_arm_trajectory_and_stopped_before_feedback(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        events = []
        active = False
        @contextmanager
        def driving():
            nonlocal active
            active = True
            events.append('start')
            try:
                yield
            finally:
                active = False
                events.append('stop')
        original = arm._write.side_effect
        def write(mid, addr, value, size=2):
            if addr == 42 and active:
                events.append('arm')
            return original(mid, addr, value, size)
        arm._write.side_effect = write
        with patch('time.sleep'):
            arm.move({'wrist_roll':10},1000,base_motion=driving())
        self.assertEqual(events[0],'start')
        self.assertEqual(events[-1],'stop')
        self.assertEqual(events.count('arm'),50)
        self.assertTrue(arm.last_motion['reached'])

    def test_preflight_failure_never_starts_base(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        motion = MagicMock()
        with self.assertRaises(ValueError):
            arm.move({'wrist_roll':40},1000,base_motion=motion)
        motion.__enter__.assert_not_called()

    def test_combined_dispatch_and_invalid_base_rejection(self):
        robot = OrinRobot(mock=True)
        robot.wheels.driving = Mock()
        robot.arm.move = Mock()
        cmd = dict(action='move',x=.1,y=0,theta=0,joints={'wrist_roll':10},duration_ms=1000)
        self.assertFalse(robot.execute({**cmd,'x':1}))
        robot.arm.move.assert_not_called()
        self.assertTrue(robot.execute(cmd))
        robot.arm.move.assert_called_once_with(cmd['joints'],1000,base_motion=robot.wheels.driving.return_value)

    def test_release_attempts_all_arm_motors_even_after_failure(self):
        arm = test_arm_motion.ArmMotionTest().make_live()
        arm._write.side_effect = lambda mid,*args: (_ for _ in ()).throw(OSError('failed')) if mid==2 else None
        with self.assertRaisesRegex(OSError,'release failed'):
            arm.close()
        self.assertEqual([c.args for c in arm._write.call_args_list],[(i,40,0,1) for i in range(1,7)])
        arm.bus.close.assert_called_once()

    def test_ctrl_c_releases_arm_and_wheels(self):
        robot = OrinRobot(mock=True)
        robot.arm = test_arm_motion.ArmMotionTest().make_live()
        robot._sensors = [robot.arm]
        robot.wheels = Mock()
        previous = signal.getsignal(signal.SIGINT)
        with patch('sys.argv',['lekiwi-runtime','--mock']), patch('lekiwi_runtime.orin.OrinRobot',return_value=robot), \
             patch.object(OrinRobot,'__enter__',return_value=robot), \
             patch('lekiwi_runtime.client.RobotClient.run',side_effect=lambda: signal.getsignal(signal.SIGINT)(signal.SIGINT,None)):
            main()
        robot.wheels.close.assert_called_once_with(release_all=True)
        for mid in range(1,7):
            robot.arm._write.assert_any_call(mid,40,0,1)
        robot.arm.bus.close.assert_called_once()
        self.assertEqual(signal.getsignal(signal.SIGINT),previous)
