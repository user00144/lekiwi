import math
import struct
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
from lekiwi_runtime.drivers.bmi160 import BMI160
from lekiwi_runtime.drivers.imu import GRAVITY, ImuStream, InertialEstimate


def sample(accel=(0,0,GRAVITY), gyro=(0,0,0)):
    return dict(accel_m_s2=list(accel), gyro_deg_s=list(gyro), saturated=False)


class ImuTest(unittest.TestCase):
    def test_decode_units_and_ready(self):
        bus = Mock()
        bus.read_reg.return_value = 0
        imu = BMI160(bus,105)
        self.assertIsNone(imu.read())
        bus.read_reg.return_value = 0xC0
        bus.read_regs.side_effect = [struct.pack('<6h',656,0,-656,8192,0,-8192), struct.pack('<h',512)]
        result = imu.read()
        self.assertEqual(result['accel_m_s2'],[GRAVITY,0,-GRAVITY])
        self.assertEqual(result['gyro_deg_s'],[10,0,-10])
        self.assertEqual(result['temperature_c'],24)

    def test_stationary_tilt_and_bias_removal(self):
        data = sample((0, GRAVITY/math.sqrt(2), GRAVITY/math.sqrt(2)), (0,0,1))
        estimator = InertialEstimate([data]*100,0)
        for i in range(1,101):
            result=estimator.update(data,i*.01)
        np.testing.assert_allclose(result['displacement_m'],[0,0,0],atol=1e-12)
        np.testing.assert_allclose(result['linear_accel_m_s2'],[0,0,0],atol=1e-12)
        self.assertAlmostEqual(result['orientation_deg'][0],45)

    def test_acceleration_integrates_and_coasts_without_false_zero_velocity(self):
        estimator=InertialEstimate([sample()]*100,0)
        for i in range(1,101):
            result=estimator.update(sample((1,0,GRAVITY)),i*.01)
        self.assertAlmostEqual(result['velocity_m_s'][0],1,delta=.01)
        self.assertAlmostEqual(result['displacement_m'][0],.5,delta=.01)
        for i in range(101,201):
            result=estimator.update(sample(),i*.01)
        self.assertAlmostEqual(result['velocity_m_s'][0],1,delta=.01)
        self.assertAlmostEqual(result['displacement_m'][0],1.5,delta=.02)

    def test_rotation_compensates_gravity(self):
        estimator=InertialEstimate([sample()]*100,0)
        for i in range(1,101):
            angle=math.radians(i*.3)
            result=estimator.update(sample((0,GRAVITY*math.sin(angle),GRAVITY*math.cos(angle)),(30,0,0)),i*.01)
        self.assertAlmostEqual(result['orientation_deg'][0],30,places=6)
        np.testing.assert_allclose(result['displacement_m'],[0,0,0],atol=1e-10)

    def test_gap_invalidates_instead_of_extrapolating(self):
        estimator=InertialEstimate([sample()]*100,0)
        result=estimator.update(sample(),.2)
        self.assertFalse(result['estimate_valid'])
        self.assertIsNone(result['displacement_m'])
        self.assertFalse(estimator.update(sample(),.21)['estimate_valid'])

    def test_moving_calibration_and_saturation(self):
        with self.assertRaises(ValueError):
            InertialEstimate([sample((i%2,0,GRAVITY)) for i in range(100)],0)
        estimator=InertialEstimate([sample()]*100,0)
        self.assertFalse(estimator.update({**sample(),'saturated':True},.01)['estimate_valid'])

    def test_mock_lifecycle_stale_and_disabled(self):
        stream=ImuStream({'enabled':True,'bus':7,'address':105},mock=True)
        stream.start()
        try:
            for _ in range(100):
                if stream.snapshot().get('estimate_valid'): break
                time.sleep(.005)
            self.assertTrue(stream.snapshot()['estimate_valid'])
            with patch('lekiwi_runtime.drivers.imu.time.monotonic',return_value=time.monotonic()+1):
                stale=stream.snapshot()
                self.assertFalse(stale['available'])
                self.assertIsNone(stale['displacement_m'])
        finally:
            stream.close()
        self.assertFalse(stream._thread.is_alive())
        disabled=ImuStream({'enabled':False})
        disabled.start()
        self.assertFalse(disabled.snapshot()['available'])
        self.assertTrue(disabled.ready)
