"""Numerical backend regression tests; no robot/network/serial access."""
from concurrent.futures import ThreadPoolExecutor
import json
import unittest
from unittest.mock import patch

import numpy as np
import osqp

from harness.kinematics import ARM_JOINTS, CartesianController, DEFAULT_URDF, RobotGeometry


def joints(values):
    return dict(zip(ARM_JOINTS, values))


class PinocchioBackendTest(unittest.TestCase):
    def setUp(self):
        self.geometry = RobotGeometry(DEFAULT_URDF)

    def test_fk_preserves_independent_urdf_reference_positions(self):
        # Frozen references from the previous URDF matrix-product implementation;
        # these detect changed joint order, radian/degree errors, and wrong frames.
        references = [
            ([0, 0, 0, 0, 0], [.39136147022015794, -.00000921206311457187, .22646971024033286]),
            ([0, -15, 35, -20, 0], [.35491516020895275, -.00000932843880485247, .18342868753748573]),
            ([-3.077, -99.78, 90.33, 70.418, -4.44], [.15595532491831596, .006897762144006808, .009312544121469576]),
            ([20, -40, 35, 50, -25], [.2476412384371982, -.07247261260424986, .12016483339019683]),
        ]
        for values, expected in references:
            with self.subTest(values=values):
                result = self.geometry.forward(joints(values))
                np.testing.assert_allclose(result['xyz_m'], expected, atol=1e-12)
                self.assertEqual(result['kinematics_library'], 'pinocchio')
                points = self.geometry.joint_points(joints(values))
                self.assertEqual([point['name'] for point in points], ['base_link', *ARM_JOINTS, 'gripper_frame_link'])
                np.testing.assert_allclose(points[-1]['xyz_m'], expected, atol=1e-12)
        self.assertNotIn('gripper', self.geometry.forward(joints([0] * 5))['joints_deg'])

    def test_pinocchio_analytic_position_and_approach_derivatives(self):
        for values in ([0, 0, 0, 0, 0], [20, -40, 35, 50, -25]):
            q = np.deg2rad(values)
            transform, jacobian = self.geometry._kinematics(q, jacobian=True)
            axis_jacobian = np.column_stack([np.cross(jacobian[3:, i], transform[:3, 2]) for i in range(5)])
            for i in range(5):
                dq = np.zeros(5)
                dq[i] = 1e-6
                plus = self.geometry._transform(q + dq)
                minus = self.geometry._transform(q - dq)
                np.testing.assert_allclose(jacobian[:3, i], (plus[:3, 3] - minus[:3, 3]) / 2e-6, atol=1e-9)
                np.testing.assert_allclose(axis_jacobian[:, i], (plus[:3, 2] - minus[:3, 2]) / 2e-6, atol=1e-9)

    def test_geometry_is_safe_to_share_between_preview_workers(self):
        values = [joints([i * 3., -25., 20., 15., -i]) for i in range(12)]
        expected = [self.geometry.forward(value)['xyz_m'] for value in values]
        with ThreadPoolExecutor(max_workers=4) as pool:
            observed = list(pool.map(self.geometry.forward, values * 4))
        for i, result in enumerate(observed):
            np.testing.assert_allclose(result['xyz_m'], expected[i % len(values)], atol=1e-12)

    def test_fk_reports_invalid_measured_seed_without_clipping(self):
        seed = joints([0, -101, 0, 0, 0])
        fk = self.geometry.forward(seed)
        self.assertEqual(fk['joints_deg']['shoulder_lift'], -101.)
        self.assertIn('shoulder_lift', fk['urdf_limit_violations'])
        with patch.object(osqp.OSQP, 'setup', side_effect=AssertionError('invalid state must not optimize')):
            with self.assertRaisesRegex(ValueError, '자동으로 잘라'):
                self.geometry.solve_position(fk['xyz_m'], seed)


class OsqpEndpointTest(unittest.TestCase):
    def setUp(self):
        self.geometry = RobotGeometry(DEFAULT_URDF)

    def test_real_osqp_is_used_with_analytic_jacobians(self):
        seed = joints([0, -15, 35, -20, 0])
        target_pose = self.geometry.forward(joints([4, -20, 40, -12, 4]))
        axis = np.array(target_pose['transform'])[:3, 2].tolist()
        actual_solve = osqp.OSQP.solve
        calls = []

        def tracking_solve(instance, *args, **kwargs):
            calls.append(1)
            return actual_solve(instance, *args, **kwargs)

        with patch.object(osqp.OSQP, 'solve', tracking_solve), patch(
                'scipy.optimize.least_squares', side_effect=AssertionError('old IK backend must not run')):
            result = self.geometry.solve_position(target_pose['xyz_m'], seed, approach_axis=axis)
        self.assertTrue(result['success'], result)
        self.assertTrue(calls)
        self.assertEqual(result['solver'], 'pinocchio+osqp')
        self.assertEqual(result['jacobian'], 'pinocchio.LOCAL_WORLD_ALIGNED.analytic')
        self.assertLessEqual(result['residual_m'], .002)
        self.assertLessEqual(result['orientation_error_deg'], 5.)
        self.assertEqual(sum(attempt['qp_solves'] for attempt in result['attempts']), len(calls))
        self.assertFalse(result['executable'])
        self.assertFalse(result['collision_checked'])
        json.dumps(result, allow_nan=False)

    def test_reported_absolute_target_preserves_duration_box_behavior(self):
        seed = joints([-3.077, -99.78, 90.33, 70.418, -4.44])
        result = self.geometry.solve_position([.2, 0., .05], seed,
            {name: [value - 30., value + 30.] for name, value in seed.items()})
        self.assertTrue(result['success'], result)
        self.assertLessEqual(result['residual_m'], .002)
        shorter = self.geometry.solve_position([.2, 0., .05], seed,
            {name: [value - 15., value + 15.] for name, value in seed.items()})
        self.assertFalse(shorter['success'], shorter)
        self.assertGreater(shorter['residual_m'], .002)
        for result in (result, shorter):
            for name, value in result['best_candidate_joints_deg'].items():
                low, high = result['effective_limits_deg'][name]
                self.assertGreaterEqual(value, low - 1e-9)
                self.assertLessEqual(value, high + 1e-9)

    def test_current_grasp_reaches_absolute_target_when_more_time_is_allowed(self):
        # Frozen measured arm values; this fixture makes no robot connection.
        seed = joints([-2.813, 49.231, 6.549, 36.396, -4.352])
        maxima = [120.17582417582418, 106.02197802197803, 95.07692307692308,
                  101.97802197802197, 180.]
        packet = {'observation': {'ego': {
            'available': True, 'calibration_id': 'synthetic-grasp-reference',
            'calibration_status': {'valid': True},
            'joints': {**seed, 'gripper': 29.827},
            'limits': {name: {'min': -maximum, 'max': maximum, 'unit': 'deg', 'max_speed': 30.}
                       for name, maximum in zip(ARM_JOINTS, maxima)},
        }}}
        controller = CartesianController()
        target = {'frame': 'arm_base', 'mode': 'absolute', 'xyz_m': [.2, 0., .05]}
        short = controller.solve(packet, target, 1000)
        self.assertFalse(short['success'])
        self.assertGreater(short['residual_m'], .05)
        for duration in (2000, 3000, 10000):
            with self.subTest(duration_ms=duration):
                result = controller.solve(packet, target, duration)
                self.assertTrue(result['success'], result)
                self.assertLessEqual(result['residual_m'], .002)
                self.assertFalse(result['executable'])  # Time does not verify physical mapping.
                self.assertIsNone(result['joints'])
                self.assertNotIn('gripper', result['solution_runtime_joints_deg'])
                for name, value in result['solution_runtime_joints_deg'].items():
                    self.assertLessEqual(abs(value - seed[name]), 30. * duration / 1000. + 1e-9)
                    maximum = maxima[ARM_JOINTS.index(name)]
                    self.assertGreaterEqual(value, -maximum - 1e-9)
                    self.assertLessEqual(value, maximum + 1e-9)
        for invalid in (True, 99, 10000.5, 10001):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                controller.solve(packet, target, invalid)

    def test_locked_joint_stays_locked_while_other_joints_solve(self):
        seed = joints([0, -15, 35, -20, 0])
        target = self.geometry.forward(joints([0, -17, 39, -25, 0]))['xyz_m']
        limits = {name: [value - 10., value + 10.] for name, value in seed.items()}
        limits['shoulder_pan'] = [0., 0.]
        limits['wrist_roll'] = [0., 0.]
        result = self.geometry.solve_position(target, seed, limits)
        self.assertTrue(result['success'], result)
        self.assertEqual(result['solution_joints_deg']['shoulder_pan'], 0.)
        self.assertEqual(result['solution_joints_deg']['wrist_roll'], 0.)

    def test_unreachable_target_is_finite_bounded_and_budgeted(self):
        seed = joints([0, -15, 35, -20, 0])
        result = self.geometry.solve_position([1., 1., 1.], seed)
        self.assertFalse(result['success'])
        self.assertIsNone(result['solution_joints_deg'])
        self.assertLessEqual(len(result['attempts']), 4)
        self.assertLessEqual(result['residual_calls'], 401)
        for attempt in result['attempts']:
            self.assertLessEqual(attempt['nfev'], 100)
            self.assertLessEqual(attempt['qp_solves'], 100)
        for name, value in result['best_candidate_joints_deg'].items():
            low, high = result['effective_limits_deg'][name]
            self.assertGreaterEqual(value, low - 1e-9)
            self.assertLessEqual(value, high + 1e-9)
        json.dumps(result, allow_nan=False)

    def test_nonfinite_or_nonconverged_qp_is_not_accepted(self):
        from types import SimpleNamespace
        seed = joints([0, -15, 35, -20, 0])
        target = self.geometry.forward(joints([4, -20, 40, -12, 4]))['xyz_m']
        for status_val, status, step in [(2, 'solved inaccurate', np.zeros(5)),
                                         (1, 'solved', np.full(5, float('nan'))),
                                         (1, 'solved', np.full(5, 20.))]:
            response = SimpleNamespace(info=SimpleNamespace(status_val=status_val, status=status, iter=10), x=step)
            with self.subTest(status=status, step=step), patch.object(osqp.OSQP, 'solve', return_value=response):
                result = self.geometry.solve_position(target, seed)
            self.assertFalse(result['success'])
            self.assertTrue(all(attempt['stop_reason'].startswith('qp_') for attempt in result['attempts']))


if __name__ == '__main__':
    unittest.main()
