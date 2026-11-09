"""Continuous IMU capture and drift-prone inertial displacement estimation."""
from copy import deepcopy
import math
import sys
import threading
import time
import uuid

import numpy as np

from .bmi160 import BMI160, LinuxI2CBus

GRAVITY = 9.80665


class InertialEstimate:
    """Startup-gravity-aligned frame; initial velocity and relative yaw are zero.

    Do not infer zero velocity from low acceleration: constant-speed travel is
    indistinguishable from rest with this sensor alone.
    """
    def __init__(self, samples, timestamp):
        accel = np.array([s['accel_m_s2'] for s in samples], dtype=float)
        gyro = np.array([s['gyro_deg_s'] for s in samples], dtype=float)
        if (not np.isfinite(accel).all() or not np.isfinite(gyro).all()
                or any(s.get('saturated') for s in samples)
                or np.max(np.std(accel, axis=0)) > .15
                or np.max(np.std(gyro, axis=0)) > .5
                or np.linalg.norm(gyro.mean(axis=0)) > 5
                or abs(np.linalg.norm(accel.mean(axis=0)) - GRAVITY) > .5):
            raise ValueError('IMU calibration requires the robot to remain stationary; restart runtime')
        mean = accel.mean(axis=0)
        norm = np.linalg.norm(mean)
        roll = math.atan2(mean[1], mean[2])
        pitch = math.atan2(-mean[0], math.hypot(mean[1], mean[2]))
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        self.rotation = np.array([[cp, sp*sr, sp*cr], [0, cr, -sr], [-sp, cp*sr, cp*cr]])
        # A single static pose cannot resolve all three accelerometer biases.
        self.accel_bias = mean * (1 - GRAVITY / norm)
        self.gyro_bias = gyro.mean(axis=0)
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.previous_accel = np.zeros(3)
        self.previous_time = timestamp
        self.valid = True
        self.error = None

    def update(self, sample, timestamp):
        dt = timestamp - self.previous_time
        self.previous_time = timestamp
        accel = np.asarray(sample['accel_m_s2'], dtype=float)
        gyro = np.asarray(sample['gyro_deg_s'], dtype=float) - self.gyro_bias
        if not np.isfinite(accel).all() or not np.isfinite(gyro).all():
            raise ValueError('Non-finite IMU sample')
        if dt <= 0 or dt > .1 or sample.get('saturated'):
            self.valid = False
            self.error = 'Sample gap, invalid timestamp or saturation; restart to reset estimate'
        linear = self.rotation @ (accel - self.accel_bias) - [0, 0, GRAVITY]
        if self.valid:
            omega = np.radians(gyro) * dt
            angle = np.linalg.norm(omega)
            if angle > 1e-12:
                x, y, z = omega / angle
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                self.rotation = self.rotation @ (np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew))
            linear = self.rotation @ (accel - self.accel_bias) - [0, 0, GRAVITY]
            average_accel = (self.previous_accel + linear) / 2
            self.position += self.velocity * dt + .5 * average_accel * dt**2
            self.velocity += average_accel * dt
            self.previous_accel = linear
        r = self.rotation
        angles = [math.atan2(r[2,1],r[2,2]), math.asin(float(np.clip(-r[2,0],-1,1))), math.atan2(r[1,0],r[0,0])]
        return dict(orientation_deg=[math.degrees(v) for v in angles],
                    linear_accel_m_s2=linear.tolist(), gyro_corrected_deg_s=gyro.tolist(),
                    velocity_m_s=self.velocity.tolist() if self.valid else None,
                    displacement_m=self.position.tolist() if self.valid else None,
                    estimate_valid=self.valid, estimate_error=self.error)


class ImuStream:
    def __init__(self, config, mock=False):
        self.config, self.mock = config, mock
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._sample_time = None
        self._state = dict(available=False, calibrated=False, error='Not started',
                           estimate_valid=False, displacement_m=None, velocity_m_s=None,
                           frame='startup_gravity_aligned_sensor', simulated=mock,
                           estimate_method='inertial_double_integration',
                           estimate_note='Relative estimate only; drift accumulates. Initial rest assumed; no wheel/vision correction.')

    @property
    def ready(self):
        return self._ready.is_set()

    def start(self):
        if not self.config['enabled']:
            self._state['error'] = 'disabled'
            self._ready.set()
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self._thread = threading.Thread(target=self._run, name='imu', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        with self._lock:
            self._state.update(available=False, estimate_valid=False, displacement_m=None, velocity_m_s=None, error='IMU stopped')

    def snapshot(self):
        with self._lock:
            state = deepcopy(self._state)
            age = None if self._sample_time is None else time.monotonic() - self._sample_time
        state['age_s'] = None if age is None else round(age, 4)
        if age is not None and age > .25:
            state.update(available=False, estimate_valid=False, displacement_m=None,
                         velocity_m_s=None, error='Stale IMU sample')
        return state

    def _run(self):
        bus = None
        try:
            print('IMU starting; keep robot stationary during calibration', file=sys.stderr, flush=True)
            if self.mock:
                def read():
                    return dict(accel_m_s2=[0,0,GRAVITY], gyro_deg_s=[0,0,0], temperature_c=23, saturated=False)
            else:
                bus = LinuxI2CBus(self.config['bus'])
                bus.open()
                sensor = BMI160(bus, self.config['address'])
                sensor.initialize()
                read = sensor.read
            samples, estimator = [], None
            last_received = time.monotonic()
            while not self._stop.is_set():
                sample = read()
                now = time.monotonic()
                if sample is None:
                    if now - last_received > .5:
                        raise OSError('IMU data-ready timeout')
                    self._stop.wait(.002)
                    continue
                last_received = now
                if estimator is None:
                    samples.append(sample)
                    if len(samples) >= (1 if self.mock else 100):
                        estimator = InertialEstimate(samples, now)
                        with self._lock:
                            self._state.update(calibrated=True, origin_id=uuid.uuid4().hex,
                                               origin_timestamp=time.time())
                        print('IMU calibration complete; displacement origin set', file=sys.stderr, flush=True)
                    data = dict(estimate_valid=False, displacement_m=None, velocity_m_s=None)
                else:
                    data = estimator.update(sample, now)
                with self._lock:
                    self._sample_time = now
                    self._state.update(sample, **data, available=True, timestamp=time.time(),
                                       error=None if estimator else 'Calibrating; keep robot stationary')
                if estimator:
                    self._ready.set()
                self._stop.wait(.01 if self.mock else .002)
        except Exception as exc:
            with self._lock:
                self._state.update(available=False, estimate_valid=False, displacement_m=None,
                                   velocity_m_s=None, error=str(exc))
            self._ready.set()
            print(f'IMU failed: {exc}', file=sys.stderr, flush=True)
        finally:
            if bus:
                bus.close()
