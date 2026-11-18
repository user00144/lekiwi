"""Read-only measured-pose correspondences never become motion permissions."""
from copy import deepcopy
import json
import unittest
from unittest.mock import patch

import numpy as np

from harness.gui_preview import MODEL_SEED
from harness.kinematics import ARM_JOINTS, DEFAULT_URDF, RobotGeometry
from harness.pose_reference import build_pose_reference


def observation():
    # Live grasp snapshot from 2026-11-16, with images deliberately omitted.
    joints = dict(zip(ARM_JOINTS, [-2.813, 49.231, 6.549, 36.396, -4.352]))
    ranges = [120.175824, 106.021978, 95.076923, 101.978022, 180.]
    return {
        'status': 'ok', 'action_context': {'movement_allowed': True},
        'observation': {
            'observation_id': 'grasp-observation', 'runtime_id': 'test-runtime',
            'timestamp': 1794831043.8512113, 'simulated': False,
            'ego': {
                'available': True, 'joints': {**joints, 'gripper': 29.827},
                'raw': dict(zip((*ARM_JOINTS, 'gripper'), [2017, 2562, 2088, 2399, 1998, 2986])),
                'timestamp': 1794831043.8669837, 'error': None,
                'calibration_id': 'test-calibration', 'calibration_status': {'valid': True},
                'limits': {name: {'min': -bound, 'max': bound, 'max_speed': 30., 'unit': 'deg'}
                           for name, bound in zip(ARM_JOINTS, ranges)},
            },
        },
    }


def configuration():
    return {
        'version': 1, 'hardware_alignment_verified': True, 'calibration_id': 'test-calibration',
        'model_hash': RobotGeometry(DEFAULT_URDF).model_hash,
        'joint_mapping': {name: {'sign': 1, 'offset_deg': 0.} for name in ARM_JOINTS},
    }


class PoseReferenceTest(unittest.TestCase):
    def test_measured_grasp_default_and_zero_correspond_with_real_ik(self):
        packet = observation()
        before = deepcopy(packet)
        result = build_pose_reference(packet)
        self.assertTrue(result['reference_valid'])
        self.assertEqual(result['kinematics_library'], 'pinocchio')
        self.assertEqual(result['poses']['default']['urdf_joints_deg'], MODEL_SEED)
        self.assertEqual(result['poses']['zero']['urdf_joints_deg'], dict.fromkeys(ARM_JOINTS, 0.))
        current = result['poses']['current']
        np.testing.assert_allclose(current['xyz_m'], [.23876184898, .01041400442, -.09853227248], atol=1e-10)
        self.assertEqual(result['provenance']['raw']['gripper'], 2986)
        self.assertEqual(result['provenance']['gripper'], {'value': 29.827, 'unit': '%', 'used_in_arm_ik': False})
        self.assertEqual(result['provenance']['observation_id'], 'grasp-observation')
        self.assertEqual(result['provenance']['runtime_id'], 'test-runtime')
        np.testing.assert_allclose(result['delta']['current_minus_default_xyz_m'],
                                   np.array(current['xyz_m']) - result['poses']['default']['xyz_m'])
        geometry = RobotGeometry(DEFAULT_URDF)
        for check in result['checks'].values():
            self.assertTrue(check['success'], check)
            self.assertFalse(check['skipped'])
            self.assertEqual(check['solver'], 'pinocchio+osqp')
            self.assertLess(check['residual_m'], .002)
            achieved = geometry.forward(check['solution_joints_deg'])['xyz_m']
            self.assertLess(np.linalg.norm(np.array(achieved) - check['target_xyz_m']), .002)
            for name, value in check['solution_joints_deg'].items():
                low, high = result['effective_urdf_limits_deg'][name]
                self.assertGreaterEqual(value, low - 1e-9)
                self.assertLessEqual(value, high + 1e-9)
        round_trip = result['checks']['current_round_trip']
        np.testing.assert_allclose(list(round_trip['solution_joints_deg'].values()),
                                   list(current['urdf_joints_deg'].values()), atol=1e-10)
        self.assertIn('hardware_alignment_unverified', result['reason_codes'])
        self.assertFalse(result['scope']['calibration_estimated'])
        self.assertFalse(result['scope']['cube_centre_measured'])
        self.assertEqual(packet, before)
        json.dumps(result, allow_nan=False)

    def test_verified_config_and_movement_allowed_still_never_authorize_reference(self):
        packet, config = observation(), configuration()
        original_config = deepcopy(config)
        result = build_pose_reference(packet, config)
        self.assertTrue(result['reference_valid'])
        self.assertTrue(result['mapping_was_verified'])
        for value in (result, *result['checks'].values()):
            self.assertFalse(value['executable'])
            self.assertFalse(value['executed'])
            self.assertFalse(value['hardware_alignment_verified'])
            self.assertTrue(value['preview_only'])
        self.assertEqual(config, original_config)

    def test_full_range_geometry_is_explicitly_independent_of_speed_and_time(self):
        packet = observation()
        for limit in packet['observation']['ego']['limits'].values():
            limit['max_speed'] = .000001
        result = build_pose_reference(packet)
        self.assertTrue(result['checks']['current_to_default']['success'])
        self.assertTrue(result['checks']['default_to_current']['success'])
        self.assertFalse(result['scope']['speed_time_constrained'])
        for check in result['checks'].values():
            self.assertFalse(check['speed_time_constrained'])
            self.assertIsNone(check['joints'])

    def test_mapping_applies_to_both_measured_seed_and_calibrated_ranges(self):
        packet, config = observation(), configuration()
        config['joint_mapping']['shoulder_pan'] = {'sign': -1, 'offset_deg': 5.}
        result = build_pose_reference(packet, config)
        self.assertAlmostEqual(result['poses']['current']['urdf_joints_deg']['shoulder_pan'], 7.813)
        current_check = result['checks']['current_round_trip']
        self.assertTrue(current_check['success'])
        self.assertAlmostEqual(current_check['solution_runtime_joints_deg']['shoulder_pan'], -2.813)
        urdf_range = RobotGeometry(DEFAULT_URDF).forward(MODEL_SEED)['urdf_limits_deg']['shoulder_pan']
        self.assertEqual(result['effective_urdf_limits_deg']['shoulder_pan'],
                         [max(-120.175824 + 5., urdf_range[0]), min(120.175824 + 5., urdf_range[1])])

    def test_invalid_measured_states_never_enter_ik_or_clip_seed(self):
        cases = []
        packet = observation()
        packet['observation']['ego']['joints']['shoulder_lift'] = -110.5  # >10 deg past URDF -100
        cases.append((packet, 'urdf_seed_out_of_range'))
        packet = observation()
        packet['observation']['ego']['joints']['elbow_flex'] = 100.
        cases.append((packet, 'runtime_seed_out_of_range'))
        packet = observation()
        packet['observation']['ego']['calibration_status']['valid'] = False
        cases.append((packet, 'calibration_invalid'))
        packet = observation()
        packet['observation']['ego']['calibration_id'] = None
        cases.append((packet, 'calibration_id_missing'))
        packet = observation()
        packet['status'] = 'error'
        cases.append((packet, 'observation_packet_error'))
        packet = observation()
        packet['observation']['ego']['error'] = 'Communication failure'
        cases.append((packet, 'feedback_error'))
        for packet, reason in cases:
            with self.subTest(reason=reason), patch.object(RobotGeometry, 'solve_position') as solve:
                result = build_pose_reference(packet)
                solve.assert_not_called()
                self.assertFalse(result['reference_valid'])
                self.assertIn(reason, result['reason_codes'])
                self.assertEqual(result['poses']['current']['runtime_joints_deg'],
                                 {name: packet['observation']['ego']['joints'][name] for name in ARM_JOINTS})
                self.assertTrue(all(check['skipped'] and not check['success'] for check in result['checks'].values()))

    def test_mismatched_calibration_is_not_a_trusted_reference(self):
        config = configuration()
        config['calibration_id'] = 'other-calibration'
        with patch.object(RobotGeometry, 'solve_position') as solve:
            result = build_pose_reference(observation(), config)
            solve.assert_not_called()
        self.assertFalse(result['reference_valid'])
        self.assertIn('calibration_id_mismatch', result['reason_codes'])

    def test_default_seed_outside_calibrated_range_is_reported_not_clipped(self):
        packet = observation()
        ego = packet['observation']['ego']
        for name in ARM_JOINTS:
            ego['limits'][name].update(min=ego['joints'][name] - 1., max=ego['joints'][name] + 1.)
        result = build_pose_reference(packet)
        self.assertTrue(result['reference_valid'])
        self.assertTrue(result['checks']['current_round_trip']['success'])
        check = result['checks']['default_to_current']
        self.assertFalse(check['success'])
        self.assertTrue(check['skipped'])
        self.assertIn('ik_invalid_seed_or_bounds', check['reason_codes'])
        self.assertEqual(check['seed_joints_deg'], MODEL_SEED)

    def test_unavailable_or_nonfinite_feedback_preserves_model_preview_and_json_safety(self):
        packet = observation()
        packet['observation']['ego']['joints']['shoulder_pan'] = float('nan')
        packet['observation']['ego']['raw']['shoulder_pan'] = float('inf')
        for value in (None, {}, packet):
            with self.subTest(value=type(value).__name__), patch.object(RobotGeometry, 'solve_position') as solve:
                result = build_pose_reference(value)
                solve.assert_not_called()
                self.assertFalse(result['reference_valid'])
                self.assertIsNone(result['poses']['current'])
                self.assertIsNone(result['delta'])
                self.assertEqual(result['poses']['default']['urdf_joints_deg'], MODEL_SEED)
                json.dumps(result, allow_nan=False)

    def test_recorded_snapshot_remains_usable_only_as_numeric_reference(self):
        packet = observation()
        packet['source'] = {'recorded': True, 'run_id': 'grasp'}
        result = build_pose_reference(packet)
        self.assertTrue(result['reference_valid'])
        self.assertIn('recorded_observation', result['reason_codes'])
        self.assertEqual(result['provenance']['source'], packet['source'])
        self.assertFalse(result['executable'])


if __name__ == '__main__':
    unittest.main()
