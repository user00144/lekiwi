"""Preview API stays usable without a planner, robot, or verified mapping."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import AsyncMock

import httpx
import numpy as np

from harness.episode import EpisodeLimits, RunRecord
from harness.gui import GuiController, create_app
from harness.gui_preview import MODEL_SEED
from harness.kinematics import ARM_JOINTS, DEFAULT_URDF, RobotGeometry


TARGET = {'frame': 'arm_base', 'mode': 'relative', 'xyz_m': [0, 0, .01], 'approach_axis': None}


class GuiPreviewTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.episode = AsyncMock()
        self.controller = GuiController(runs_dir=self.temp.name, episode_fn=self.episode)
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(self.controller)),
                                       base_url='http://127.0.0.1:8765')

    async def asyncTearDown(self):
        self.episode.assert_not_called()
        self.assertIsNone(self.controller.task)
        await self.client.aclose()
        self.temp.cleanup()

    def record(self, joints=None, oid='obs-1', limits=None):
        record = RunRecord(Path(self.temp.name) / 'saved', 'robot', 'saved goal', EpisodeLimits())
        packet = {'observation': {'observation_id': oid, 'ego': {
            'available': True, 'calibration_id': 'test-calibration',
            'calibration_status': {'valid': True}, 'joints': joints or dict(MODEL_SEED),
            'limits': limits or {name: {'min': -150., 'max': 150., 'max_speed': 40., 'unit': 'deg'}
                       for name in ARM_JOINTS}}}}
        record.observation(packet, [])
        return record

    async def preview(self, payload):
        response = await self.client.post('/api/kinematics/preview', json=payload)
        self.assertEqual(response.status_code, 200, response.text)
        value = response.json()
        self.assertTrue(value['preview_only'])
        self.assertFalse(value['executable'])
        if value['result']:
            self.assertFalse(value['result']['executable'])
            self.assertIsNone(value['result']['joints'])
        return value

    async def test_model_fk_and_relative_ik_work_without_configuration_or_runs(self):
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['capabilities']['xyz_preview'], 1)
        fk = await self.preview({'source': 'model'})
        result = await self.preview({'source': 'model', 'target': TARGET})
        self.assertTrue(result['result']['success'])
        self.assertLess(result['result']['residual_m'], .002)
        np.testing.assert_allclose(result['result']['target_xyz_m'],
                                   np.array(fk['state']['current_xyz_m']) + [0, 0, .01])
        np.testing.assert_allclose(result['current_points'][-1]['xyz_m'], fk['state']['current_xyz_m'])
        np.testing.assert_allclose(result['solution_points'][-1]['xyz_m'], result['result']['achieved_xyz_m'])
        self.assertEqual(list(Path(self.temp.name).iterdir()), [])

    async def test_absolute_goal_uses_editable_model_seed(self):
        joints = {**MODEL_SEED, 'shoulder_pan': 25.}
        target = RobotGeometry(DEFAULT_URDF).forward(joints)['xyz_m']
        value = await self.preview({'source': 'model', 'joints_deg': joints,
                                    'target': {**TARGET, 'mode': 'absolute', 'xyz_m': target}})
        self.assertTrue(value['result']['success'])
        self.assertAlmostEqual(value['result']['residual_m'], 0.)

    async def test_unreachable_goal_keeps_current_plot_without_valid_solution(self):
        value = await self.preview({'source': 'model', 'target': {**TARGET, 'xyz_m': [1., 0, 0]}})
        self.assertFalse(value['result']['success'])
        self.assertTrue(value['current_points'])
        self.assertEqual(value['solution_points'], [])
        self.assertTrue(value['result']['reasons'])

    async def test_recorded_ik_uses_pinned_observation_and_does_not_modify_history(self):
        record = self.record()
        record.observation({'observation': {'observation_id': 'new-unavailable', 'ego': {'available': False}}}, [])
        before = record.events.read_bytes()
        value = await self.preview({'source': 'recorded', 'run_id': 'saved',
                                    'observation_id': 'obs-1', 'target': TARGET})
        self.assertTrue(value['result']['success'])
        self.assertEqual(value['source']['observation_id'], 'obs-1')
        self.assertIn('recorded_observation', value['state']['reason_codes'])
        self.assertIn('hardware_alignment_unverified', value['state']['reason_codes'])
        self.assertEqual(record.events.read_bytes(), before)

    async def test_verified_config_still_cannot_make_disk_preview_executable(self):
        self.record()
        config = Path(self.temp.name) / 'mapping.json'
        config.write_text(json.dumps({'version': 1, 'hardware_alignment_verified': True,
                                      'calibration_id': 'test-calibration',
                                      'model_hash': RobotGeometry(DEFAULT_URDF).model_hash,
                                      'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}}))
        self.controller.kinematics_config = config
        value = await self.preview({'source': 'recorded', 'run_id': 'saved', 'target': TARGET})
        self.assertTrue(value['result']['success'])
        self.assertFalse(value['state']['executable'])
        self.assertFalse(value['state']['fk']['executable'])

    async def test_outside_urdf_seed_displays_fk_and_reason_without_clamping(self):
        joints = {**MODEL_SEED, 'shoulder_lift': -100.484}
        self.record(joints)
        for source in ({'source': 'recorded', 'run_id': 'saved'}, {'source': 'model', 'joints_deg': joints}):
            value = await self.preview({**source, 'target': TARGET})
            self.assertTrue(value['state']['available'])
            self.assertTrue(value['current_points'])
            self.assertFalse(value['result']['success'])
            self.assertEqual(value['state']['fk']['joints_deg']['shoulder_lift'], -100.484)
            self.assertIn('shoulder_lift', value['state']['fk']['urdf_limit_violations'])

    async def test_duration_failure_explained_without_changing_request_or_authorizing_motion(self):
        joints = dict(zip(ARM_JOINTS, [-3.077, -99.78, 90.33, 70.418, -4.44]))
        ranges = [120.175824, 106.021978, 95.076923, 101.978022, 180.]
        limits = {name: {'min': -bound, 'max': bound, 'max_speed': 30., 'unit': 'deg'}
                  for name, bound in zip(ARM_JOINTS, ranges)}
        record = self.record(joints, limits=limits)
        before = record.events.read_bytes()
        payload = {'source': 'recorded', 'run_id': 'saved', 'duration_ms': 500,
                   'target': {**TARGET, 'mode': 'absolute', 'xyz_m': [.2, 0., .05]}}
        value = await self.preview(payload)
        failed = value['result']
        self.assertFalse(failed['success'])
        self.assertEqual(failed['duration_ms'], 500)
        self.assertAlmostEqual(failed['residual_m'], .00263059, places=6)
        check = failed['duration_check']
        self.assertTrue(check['success'])
        self.assertEqual(check['checked_duration_ms'], 1000)
        self.assertFalse(check['executable'])
        self.assertEqual(value['solution_points'], [])
        self.assertIsNone(failed['solution_runtime_joints_deg'])
        slower = await self.preview({**payload, 'duration_ms': 1000})
        self.assertTrue(slower['result']['success'])
        self.assertLess(slower['result']['residual_m'], 1e-6)
        self.assertIn('hardware_alignment_unverified', slower['result']['reason_codes'])
        self.assertNotIn('duration_check', slower['result'])
        self.assertEqual(record.events.read_bytes(), before)

    async def test_unreachable_goal_does_not_suggest_duration_as_a_fix(self):
        self.record()
        value = await self.preview({'source':'recorded', 'run_id':'saved', 'duration_ms':500,
                                    'target':{**TARGET, 'mode':'absolute', 'xyz_m':[1.,0.,0.]}})
        self.assertFalse(value['result']['success'])
        self.assertFalse(value['result']['duration_check']['success'])
        self.assertFalse(value['result']['executable'])

    async def test_unavailable_feedback_gives_clear_empty_preview(self):
        record = self.record()
        record.observation({'observation': {'observation_id': 'missing', 'ego': {'available': False}}}, [])
        value = await self.preview({'source': 'recorded', 'run_id': 'saved', 'target': TARGET})
        self.assertFalse(value['state']['available'])
        self.assertTrue(value['state']['reasons'])
        self.assertEqual(value['current_points'], [])

    async def test_strict_payload_rejects_override_and_bad_numbers(self):
        cases = [{'source': 'model', 'execute': True}, {'source': 'model', 'config': '/tmp/mapping.json'},
                 {'source': 'model', 'run_id': 'saved'}, {'source': 'recorded'},
                 {'source': 'recorded', 'run_id': 'saved', 'joints_deg': MODEL_SEED},
                 {'source': 'model', 'joints_deg': {'shoulder_pan': 0}},
                 {'source': 'model', 'joints_deg': {**MODEL_SEED, 'shoulder_pan': True}},
                 {'source': 'model', 'duration_ms': True}, {'source': 'model', 'duration_ms': 1001},
                 {'source': 'model', 'target': {**TARGET, 'xyz_m': [1, 2]}},
                 {'source': 'model', 'target': {**TARGET, 'frame': 'camera'}}]
        for payload in cases:
            response = await self.client.post('/api/kinematics/preview', json=payload)
            self.assertEqual(response.status_code, 400, response.text)

    async def test_missing_records_and_symlink_paths_never_load_arbitrary_files(self):
        self.record()
        (Path(self.temp.name) / 'alias').symlink_to(Path(self.temp.name) / 'saved', target_is_directory=True)
        for run_id in ('no-such-run', '../saved', 'alias'):
            response = await self.client.post('/api/kinematics/preview', json={'source': 'recorded', 'run_id': run_id})
            self.assertEqual(response.status_code, 404)
        response = await self.client.post('/api/kinematics/preview', json={
            'source': 'recorded', 'run_id': 'saved', 'observation_id': 'missing'})
        self.assertEqual(response.status_code, 400)

    async def test_same_origin_and_body_guards_apply_to_preview(self):
        response = await self.client.post('/api/kinematics/preview', json={'source': 'model'},
                                          headers={'Origin': 'https://example.com'})
        self.assertEqual(response.status_code, 403)
        response = await self.client.post('/api/kinematics/preview', content='{}')
        self.assertEqual(response.status_code, 400)

    async def test_parallel_solver_requests_are_bounded(self):
        async with self.controller.preview_lock:
            response = await self.client.post('/api/kinematics/preview', json={'source': 'model', 'target': TARGET})
        self.assertEqual(response.status_code, 409)
