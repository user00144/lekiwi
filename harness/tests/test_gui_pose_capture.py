"""A reference capture reads once, remains inspection-only, and reserves the session."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

import httpx

from harness.gui import GuiController, create_app
from test_episode_cartesian import ArmTools


class PoseCaptureTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.episode = AsyncMock()
        self.controller = GuiController(runs_dir=self.folder.name, episode_fn=self.episode)
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(self.controller)),
                                       base_url='http://127.0.0.1:8765')
        self.addAsyncCleanup(self.client.aclose)
        self.transport = AsyncMock()
        self.transport.request.return_value = ArmTools().packet()
        self.patcher = patch('harness.gui.ServerClient', return_value=self.transport)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    async def test_capture_once_then_reopen_comparison_without_robot_io(self):
        response = await self.client.post('/api/kinematics/capture', json={})
        self.assertEqual(response.status_code, 200, response.text)
        saved = response.json()
        self.assertFalse(saved['executed'])
        self.assertTrue(saved['pose_reference']['reference_valid'])
        run_id = saved['run_id']
        run = (await self.client.get('/api/runs/' + run_id)).json()
        self.assertEqual(run['mode'], 'reference')
        self.assertEqual(run['result']['action_attempts'], 0)
        self.assertEqual(run['status'], 'reference_saved')
        preview = (await self.client.post('/api/kinematics/preview', json={
            'source': 'recorded', 'run_id': run_id})).json()
        self.assertEqual(preview['pose_reference']['provenance']['observation_id'],
                         preview['source']['observation_id'])
        self.assertFalse(preview['state']['executable'])
        self.transport.request.assert_awaited_once_with('kiwi-01', 'observe')
        self.episode.assert_not_called()

    async def test_other_capture_agent_and_manual_cannot_start_during_read(self):
        entered, release = asyncio.Event(), asyncio.Event()
        async def slow(*args):
            entered.set()
            await release.wait()
            return ArmTools().packet()
        self.transport.request.side_effect = slow
        capture = asyncio.create_task(self.client.post('/api/kinematics/capture', json={}))
        await entered.wait()
        try:
            self.assertTrue((await self.client.get('/api/state')).json()['pose_capture_busy'])
            for path, payload in [('/api/kinematics/capture', {}),
                ('/api/manual-xyz/prepare', {'robot_id': 'kiwi-01', 'delta_m': [0., 0., .005]}),
                ('/api/runs', {'robot_id': 'kiwi-01', 'instruction': 'move', 'execute': True, 'model': None,
                              'limits': {'max_actions': 1, 'max_steps': 1, 'max_seconds': 30.}})]:
                response = await self.client.post(path, json=payload)
                self.assertEqual(response.status_code, 409, f'{path}: {response.text}')
        finally:
            release.set()
            await capture
        self.transport.request.assert_awaited_once()
        self.episode.assert_not_called()
        self.assertFalse(self.controller.reference_busy)

    async def test_timeout_releases_capture_and_does_not_create_success_record(self):
        self.transport.request.side_effect = TimeoutError('read timed out')
        response = await self.client.post('/api/kinematics/capture', json={})
        self.assertIn(response.status_code, (400, 503))
        self.assertFalse(self.controller.reference_busy)
        self.assertEqual(list(Path(self.folder.name).iterdir()), [])

    async def test_reference_with_different_mapping_is_not_shown_as_current(self):
        saved = (await self.client.post('/api/kinematics/capture', json={})).json()
        path = Path(self.folder.name) / saved['run_id'] / 'pose_reference.json'
        reference = json.loads(path.read_text())
        reference['mapping_id'] = 'different'
        path.write_text(json.dumps(reference))
        response = await self.client.post('/api/kinematics/preview', json={
            'source': 'recorded', 'run_id': saved['run_id']})
        self.assertNotIn('pose_reference', response.json())

    async def test_origin_payload_and_demo_checks_never_contact_robot(self):
        response = await self.client.post('/api/kinematics/capture', json={},
                                          headers={'Origin': 'https://example.com'})
        self.assertEqual(response.status_code, 403)
        response = await self.client.post('/api/kinematics/capture', json={'execute': True})
        self.assertEqual(response.status_code, 400)
        self.controller.mode = 'demo'
        response = await self.client.post('/api/kinematics/capture', json={})
        self.assertEqual(response.status_code, 400)
        self.transport.request.assert_not_called()
