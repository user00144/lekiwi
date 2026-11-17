"""Pure numerical geometry checks: no model, network, serial port or robot."""
from copy import deepcopy
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from harness.kinematics import ARM_JOINTS, CartesianController, DEFAULT_URDF, RobotGeometry, main


CALIBRATION_ID = 'sha256:synthetic-test-calibration'
MODEL_HASH = RobotGeometry(DEFAULT_URDF).model_hash


def config(*, verified=True):
    return {'version': 1, 'hardware_alignment_verified': verified,
            'calibration_id': CALIBRATION_ID if verified else None,
            'model_hash': MODEL_HASH if verified else None,
            'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}}


def packet(joints=None):
    return {'observation': {'ego': {
        'available': True, 'calibration_id': CALIBRATION_ID,
        'calibration_status': {'valid': True, 'out_of_range': [], 'missing_joints': []},
        'joints': {**dict.fromkeys(ARM_JOINTS, 0.), 'gripper': 50., **(joints or {})},
        'limits': {**{name: {'min': -90., 'max': 90., 'unit': 'deg', 'max_speed': 30.}
                     for name in ARM_JOINTS},
                   'gripper': {'min': 0., 'max': 100., 'unit': '%', 'max_speed': 50.}},
    }}}


def request(xyz, mode='absolute', axis=None):
    return {'frame': 'arm_base', 'mode': mode, 'xyz_m': xyz, 'approach_axis': axis}


class CartesianReadinessTest(unittest.TestCase):
    def test_default_identity_is_preview_even_for_successful_ik(self):
        controller = CartesianController()
        state = controller.evaluate(packet())
        self.assertTrue(state['available'])
        self.assertFalse(state['executable'])
        self.assertEqual(state['frame'], 'arm_base')
        self.assertEqual(state['tip_frame'], 'gripper_frame_link')
        self.assertIn('hardware_alignment_unverified', state['reason_codes'])
        result = controller.solve(packet(), request(state['current_xyz_m']), 500)
        self.assertTrue(result['success'])
        self.assertFalse(result['executable'])
        self.assertIsNone(result['joints'])
        self.assertEqual(set(result['solution_runtime_joints_deg']), set(ARM_JOINTS))
        self.assertFalse(result['collision_checked'])
        self.assertFalse(result['cartesian_straight_line'])
        json.dumps(result, allow_nan=False)

    def test_verified_mapping_overrides_preview_metadata_and_keeps_fingerprint(self):
        controller = CartesianController(config())
        state = controller.evaluate(packet())
        self.assertTrue(state['executable'])
        self.assertTrue(state['fk']['executable'])
        self.assertTrue(state['fk']['hardware_alignment_verified'])
        self.assertEqual(state['fk']['frame'], 'arm_base')
        result = controller.solve(packet(), request(state['current_xyz_m']), 500)
        self.assertTrue(result['success'])
        self.assertTrue(result['executable'])
        self.assertEqual(result['joints'], result['solution_runtime_joints_deg'])
        self.assertEqual(result['calibration_id'], CALIBRATION_ID)
        self.assertEqual(result['verified_calibration_id'], CALIBRATION_ID)
        self.assertEqual(result['model_hash'], controller.geometry.model_hash)
        self.assertEqual(result['verified_model_hash'], MODEL_HASH)
        self.assertEqual(result['config_version'], 1)
        self.assertEqual(result['joint_mapping'], config()['joint_mapping'])
        self.assertEqual(result['mapping_id'], controller.evaluate(packet())['mapping_id'])
        self.assertEqual(result['reasons'], [])

    def test_mapping_fingerprint_tracks_sign_offsets_and_returned_mapping_is_detached(self):
        first = CartesianController(config())
        changed = config()
        changed['joint_mapping']['wrist_roll']['sign'] = -1
        second = CartesianController(changed)
        self.assertNotEqual(first.mapping_id, second.mapping_id)
        state = first.evaluate(packet())
        state['joint_mapping']['wrist_roll']['sign'] = -1
        self.assertEqual(first.evaluate(packet())['joint_mapping']['wrist_roll']['sign'], 1)

    def test_changed_or_missing_calibration_revokes_execution(self):
        controller = CartesianController(config())
        for observed, code in [('changed-calibration', 'calibration_id_mismatch'),
                               (None, 'calibration_id_missing')]:
            with self.subTest(observed=observed):
                observed_packet = packet()
                observed_packet['observation']['ego']['calibration_id'] = observed
                state = controller.evaluate(observed_packet)
                self.assertFalse(state['executable'])
                self.assertIn(code, state['reason_codes'])
                result = controller.solve(observed_packet, request(state['current_xyz_m']), 500)
                self.assertTrue(result['success'])
                self.assertFalse(result['executable'])
                self.assertIsNone(result['joints'])

    def test_changed_urdf_revokes_execution_even_with_same_calibration_and_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            changed_model = Path(directory) / 'changed.urdf'
            # Even a syntactically valid replacement requires a new explicit model binding.
            changed_model.write_bytes(DEFAULT_URDF.read_bytes() + b'\n<!-- changed model file -->\n')
            controller = CartesianController(config(), urdf_path=changed_model)
            state = controller.evaluate(packet())
            self.assertTrue(state['available'])
            self.assertFalse(state['executable'])
            self.assertFalse(state['fk']['executable'])
            self.assertIn('model_hash_mismatch', state['reason_codes'])
            self.assertEqual(state['verified_model_hash'], MODEL_HASH)
            self.assertNotEqual(state['model_hash'], MODEL_HASH)
            self.assertNotEqual(state['mapping_id'], CartesianController(config()).mapping_id)
            result = controller.solve(packet(), request([0., 0., 0.], 'relative'), 500)
            self.assertTrue(result['success'])
            self.assertFalse(result['executable'])
            self.assertIsNone(result['joints'])
            self.assertIn('model_hash_mismatch', result['reason_codes'])

    def test_model_hash_required_only_for_verified_config_and_validated_when_supplied(self):
        for verified in (False, True):
            for malformed in ('g' * 64, 'a' * 63, 'a' * 65, True, 123):
                value = config(verified=verified)
                value['model_hash'] = malformed
                with self.subTest(verified=verified, malformed=malformed), self.assertRaisesRegex(ValueError, 'model_hash'):
                    CartesianController(value)
        for missing in (False, True):
            value = config()
            if missing:
                del value['model_hash']
            else:
                value['model_hash'] = None
            with self.subTest(missing=missing), self.assertRaisesRegex(ValueError, 'requires its model_hash'):
                CartesianController(value)
            value['hardware_alignment_verified'] = False
            self.assertFalse(CartesianController(value).evaluate(packet())['executable'])
        upper_hash = config()
        upper_hash['model_hash'] = MODEL_HASH.upper()
        self.assertTrue(CartesianController(upper_hash).evaluate(packet())['executable'])

    def test_recorded_observation_is_never_executable(self):
        controller = CartesianController(config())
        observed = packet()
        observed['source'] = {'recorded': True}
        result = controller.solve(observed, request([0., 0., 0.], mode='relative'), 500)
        self.assertTrue(result['success'])
        self.assertFalse(result['executable'])
        self.assertIn('recorded_observation', result['reason_codes'])

    def test_unavailable_or_malformed_feedback_does_not_authorize_ik(self):
        examples = [None, {}, {'observation': {}}]
        for change in ('unavailable', 'missing_joint', 'bad_joint', 'missing_limits',
                       'bad_unit', 'zero_speed', 'reversed_limits'):
            observed = packet()
            ego = observed['observation']['ego']
            if change == 'unavailable': ego['available'] = False
            elif change == 'missing_joint': del ego['joints']['wrist_roll']
            elif change == 'bad_joint': ego['joints']['wrist_roll'] = True
            elif change == 'missing_limits': del ego['limits']
            elif change == 'bad_unit': ego['limits']['wrist_roll']['unit'] = 'rad'
            elif change == 'zero_speed': ego['limits']['wrist_roll']['max_speed'] = 0
            elif change == 'reversed_limits': ego['limits']['wrist_roll']['max'] = -100
            examples.append(observed)
        controller = CartesianController(config())
        for observed in examples:
            with self.subTest(observed=observed):
                result = controller.solve(observed, request([.2, 0., .2]), 500)
                self.assertFalse(result['available'])
                self.assertFalse(result['success'])
                self.assertFalse(result['executable'])
                self.assertIsNone(result['joints'])

    def test_bad_seed_and_invalid_raw_calibration_are_not_clipped(self):
        controller = CartesianController(config())
        examples = []
        outside_runtime = packet({'elbow_flex': 91.})
        examples.append((outside_runtime, 'runtime_seed_out_of_range'))
        outside_urdf = packet({'shoulder_pan': 115.})
        outside_urdf['observation']['ego']['limits']['shoulder_pan']['max'] = 120
        examples.append((outside_urdf, 'urdf_seed_out_of_range'))
        for valid in (False, None):
            bad_cal = packet()
            bad_cal['observation']['ego']['calibration_status']['valid'] = valid
            examples.append((bad_cal, 'calibration_invalid'))
        with patch.object(controller.geometry, 'solve_position', side_effect=AssertionError('solver must not run')):
            for observed, code in examples:
                with self.subTest(code=code):
                    result = controller.solve(observed, request([0., 0., 0.], 'relative'), 500)
                    self.assertFalse(result['success'])
                    self.assertFalse(result['executable'])
                    self.assertIn(code, result['reason_codes'])

    def test_configuration_validation_and_file_loading(self):
        invalid = []
        for kind in ('verified_without_id', 'unknown', 'version_bool', 'missing_mapping',
                     'missing_joint', 'sign_bool', 'sign_zero', 'nan_offset', 'wrong_frame'):
            value = config()
            if kind == 'verified_without_id': value['calibration_id'] = None
            elif kind == 'unknown': value['allow_any_robot'] = True
            elif kind == 'version_bool': value['version'] = True
            elif kind == 'missing_mapping': del value['joint_mapping']
            elif kind == 'missing_joint': del value['joint_mapping']['elbow_flex']
            elif kind == 'sign_bool': value['joint_mapping']['elbow_flex']['sign'] = True
            elif kind == 'sign_zero': value['joint_mapping']['elbow_flex']['sign'] = 0
            elif kind == 'nan_offset': value['joint_mapping']['elbow_flex']['offset_deg'] = float('nan')
            elif kind == 'wrong_frame': value['frame'] = 'camera'
            invalid.append(value)
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                CartesianController(value)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'cartesian.json'
            path.write_text(json.dumps(config()))
            self.assertTrue(CartesianController(path).evaluate(packet())['executable'])
        sample = DEFAULT_URDF.with_name('cartesian.example.json')
        self.assertFalse(CartesianController(sample).evaluate(packet())['executable'])


class CartesianSolveTest(unittest.TestCase):
    def test_nonidentity_sign_offset_mapping_round_trip_and_relative_target(self):
        value = config()
        for i, name in enumerate(ARM_JOINTS):
            value['joint_mapping'][name] = {'sign': -1 if i % 2 else 1, 'offset_deg': (i - 2) * 3.}
        controller = CartesianController(value)
        initial = dict(zip(ARM_JOINTS, [4., -5., 6., -7., 8.]))
        observed = packet(initial)
        desired = np.array([5., -7., 8., -9., 7.])
        target_urdf = desired * controller.signs + controller.offsets
        target = controller.geometry.forward(dict(zip(ARM_JOINTS, target_urdf.tolist())))
        start = controller.evaluate(observed)['current_xyz_m']
        relative = (np.array(target['xyz_m']) - np.array(start)).tolist()
        # The local +Z axis adds two constraints without requesting a full 6D pose.
        axis = np.array(target['transform'])[:3, 2].tolist()
        result = controller.solve(observed, request(relative, 'relative', axis), 1000)
        self.assertTrue(result['success'], result)
        self.assertTrue(result['executable'], result)
        self.assertLessEqual(result['residual_m'], .002)
        self.assertLessEqual(result['orientation_error_deg'], 5.)
        self.assertTrue(result['orientation_checked'])
        np.testing.assert_allclose(result['target_xyz_m'], target['xyz_m'], atol=1e-12)
        solved_runtime = np.array([result['joints'][name] for name in ARM_JOINTS])
        solved_urdf = np.array([result['solution_urdf_joints_deg'][name] for name in ARM_JOINTS])
        np.testing.assert_allclose(solved_urdf, solved_runtime * controller.signs + controller.offsets, atol=1e-10)
        forward = controller.geometry.forward(result['solution_urdf_joints_deg'])
        self.assertLessEqual(np.linalg.norm(np.array(forward['xyz_m']) - np.array(target['xyz_m'])), .002)
        self.assertNotIn('gripper', result['joints'])

    def test_speed_limits_are_part_of_optimization_not_a_posthoc_rejection(self):
        controller = CartesianController(config())
        observed = packet()
        # With all joints capped to 0.1 deg/s no solution can cover a large displacement.
        for limit in observed['observation']['ego']['limits'].values():
            limit['max_speed'] = .1
        result = controller.solve(observed, request([0., 0., -.03], 'relative'), 100)
        self.assertFalse(result['success'])
        self.assertFalse(result['executable'])
        self.assertIn('ik_no_solution', result['reason_codes'])
        for name in ARM_JOINTS:
            np.testing.assert_allclose(result['effective_limits_deg'][name], [-.01, .01], atol=1e-12)
        self.assertLessEqual(len(result['attempts']), 4)
        self.assertTrue(all(attempt['nfev'] <= 100 for attempt in result['attempts']))
        self.assertLessEqual(result['residual_calls'], 4 * 100 * 6)

    def test_sign_reversal_transforms_asymmetric_runtime_and_speed_bounds(self):
        value = config()
        value['joint_mapping']['elbow_flex'] = {'sign': -1, 'offset_deg': 15.}
        controller = CartesianController(value)
        observed = packet({'elbow_flex': 10.})
        observed['observation']['ego']['limits']['elbow_flex'].update(min=8., max=13., max_speed=4.)
        result = controller.solve(observed, request([0., 0., 0.], 'relative'), 500)
        self.assertTrue(result['executable'])
        np.testing.assert_allclose(result['effective_runtime_limits_deg']['elbow_flex'], [8., 12.])
        np.testing.assert_allclose(result['effective_limits_deg']['elbow_flex'], [3., 7.])
        self.assertAlmostEqual(result['joints']['elbow_flex'], 10.)

    def test_urdf_hardware_limit_intersection_preserved_after_mapping(self):
        value = config()
        value['joint_mapping']['shoulder_pan'] = {'sign': -1, 'offset_deg': 100.}
        controller = CartesianController(value)
        result = controller.solve(packet(), request([0., 0., 0.], 'relative'), 1000)
        self.assertTrue(result['executable'])
        upper = controller.geometry.forward(dict.fromkeys(ARM_JOINTS, 0.))['urdf_limits_deg']['shoulder_pan'][1]
        self.assertAlmostEqual(result['effective_limits_deg']['shoulder_pan'][1], upper)
        self.assertGreaterEqual(result['joints']['shoulder_pan'], 100. - upper - 1e-9)

    def test_unreachable_goal_returns_bounded_failed_preview(self):
        controller = CartesianController(config())
        result = controller.solve(packet(), request([1., 1., 1.]), 1000)
        self.assertFalse(result['success'])
        self.assertFalse(result['executable'])
        self.assertIsNone(result['joints'])
        self.assertIsNone(result['solution_runtime_joints_deg'])
        self.assertIn('ik_no_solution', result['reason_codes'])
        self.assertLessEqual(len(result['attempts']), 4)
        json.dumps(result, allow_nan=False)

    def test_malformed_requests_fail_before_optimization(self):
        controller = CartesianController(config())
        inputs = [(request([0., 0., 0.]), duration) for duration in (True, 0, 99, 20001, 100.5, None)]
        for xyz in ([True, 0, 0], ['.1', 0, 0], [float('nan'), 0, 0], [0, 0], [3., 0, 0]):
            inputs.append((request(xyz), 500))
        for axis in ([0, 0, 0], [0, float('inf'), 1], [0, 1]):
            inputs.append((request([0, 0, 0], axis=axis), 500))
        for changed in ({'frame': 'camera'}, {'mode': 'delta'}, {'rpy': [0, 0, 0]}):
            inputs.append(({**request([0, 0, 0]), **changed}, 500))
        for value, duration in inputs:
            with self.subTest(value=value, duration=duration), self.assertRaises(ValueError):
                controller.solve(packet(), value, duration)

    def test_original_observation_and_request_are_not_mutated(self):
        controller = CartesianController(config())
        observed = packet()
        target = request([0., 0., 0.], 'relative')
        before = deepcopy((observed, target))
        controller.solve(observed, target, 500)
        self.assertEqual((observed, target), before)


class GeometryKernelTest(unittest.TestCase):
    def test_fully_locked_seed_is_computed_without_optimizer(self):
        geometry = RobotGeometry(DEFAULT_URDF)
        seed = dict.fromkeys(ARM_JOINTS, 0.)
        target = geometry.forward(seed)['xyz_m']
        with patch('scipy.optimize.least_squares', side_effect=AssertionError('locked seed must not optimize')):
            result = geometry.solve_position(target, seed, {name: [0., 0.] for name in ARM_JOINTS})
        self.assertTrue(result['success'])
        self.assertEqual(result['attempts'], [])
        self.assertFalse(result['executable'])

    def test_seed_is_never_clipped_and_disjoint_limits_are_rejected(self):
        geometry = RobotGeometry(DEFAULT_URDF)
        seed = dict.fromkeys(ARM_JOINTS, 0.)
        for bounds in ([1., 2.], [500., 600.]):
            limits = {name: [-30., 30.] for name in ARM_JOINTS}
            limits['elbow_flex'] = bounds
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                geometry.solve_position([.2, 0, .2], seed, limits)


class PreviewCommandTest(unittest.TestCase):
    def test_saved_observation_remains_preview_even_with_verified_config(self):
        with tempfile.TemporaryDirectory() as directory:
            observation_path = Path(directory) / 'observation.json'
            config_path = Path(directory) / 'mapping.json'
            observation_path.write_text(json.dumps(packet()))
            config_path.write_text(json.dumps(config()))
            output = io.StringIO()
            with redirect_stdout(output):
                code = main(['--observation', str(observation_path), '--config', str(config_path),
                             '--xyz', '0', '0', '0', '--mode', 'relative', '--duration-ms', '500'])
            result = json.loads(output.getvalue())
            self.assertEqual(code, 0)
            self.assertTrue(result['success'])
            self.assertTrue(result['preview_only'])
            self.assertFalse(result['executable'])
            self.assertIsNone(result['joints'])
            self.assertIn('recorded_observation', result['reason_codes'])
            self.assertEqual(json.loads(observation_path.read_text()), packet())

    def test_unreachable_goal_returns_json_and_nonzero_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'observation.json'
            path.write_text(json.dumps(packet()))
            output = io.StringIO()
            with redirect_stdout(output):
                code = main(['--observation', str(path), '--xyz', '1', '1', '1'])
            self.assertEqual(code, 1)
            self.assertFalse(json.loads(output.getvalue())['success'])
