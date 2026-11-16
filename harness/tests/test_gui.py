import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import httpx

from harness.episode import EpisodeLimits, RunRecord, atomic_json
from harness.gui import GuiController, create_app, read_events


REQUEST = {'instruction': 'Inspect the object', 'robot_id': 'kiwi-test', 'model': None,
           'limits': {'max_actions': 2, 'max_steps': 5, 'max_seconds': 30}, 'execute': True}


def packet(oid):
    return {'status': 'ok', 'observation': {'observation_id': oid, 'runtime_id': 'runtime',
                                           'ego': {'joints': {'wrist_roll': 3}}, 'cameras': {}}}


class RecordingEpisode:
    """Disk-only fake, including a cancellable cleanup phase; no model or robot I/O."""
    def __init__(self):
        self.calls = []
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.cleaning = asyncio.Event()
        self.cleanup_release = asyncio.Event()
        self.hold_cleanup = False

    async def __call__(self, server, model, robot_id, instruction, *, run_dir, limits, kinematics_config=None):
        self.calls.append({'server': server, 'model': model, 'robot_id': robot_id,
                           'instruction': instruction, 'run_dir': run_dir, 'limits': limits,
                           'kinematics_config': kinematics_config})
        record = RunRecord(run_dir, robot_id, instruction, limits)
        record.event('startup_started', {'component': 'planner'})
        record.observation(packet('obs-before'), [('front', '/9j/2Q==', 'image/jpeg')])
        record.event('planning_started', {'step': 1, 'observation_id': 'obs-before'})
        self.started.set()
        try:
            await self.release.wait()
            record.event('decision', {'tool': 'task_complete', 'reason': 'Visible evidence',
                                      'plan': ['Inspect'], 'expected': 'Task complete',
                                      'feedback': 'progress', 'evidence': 'Object retained'})
            record.observation(packet('obs-after'), [('front', '/9j/2Q==', 'image/jpeg')])
            record.event('motion_feedback', {'before_observation_id': 'obs-before',
                                             'after_observation_id': 'obs-after'})
            result = {'status': 'completion_claim', 'physical_success_verified': False}
        except asyncio.CancelledError:
            self.cleaning.set()
            if self.hold_cleanup:
                await self.cleanup_release.wait()
            result = {'status': 'uncertain_execution', 'pending_action_id': 'action-in-flight',
                      'physical_success_verified': False}
            atomic_json(Path(run_dir) / 'result.json', result)
            record.event('episode_finished', result)
            raise
        atomic_json(Path(run_dir) / 'result.json', result)
        record.event('episode_finished', result)
        return result


class GuiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.episode = RecordingEpisode()
        self.controller = GuiController('ipc://unused', 'kiwi-01', self.root / 'runs',
                                        episode_fn=self.episode)
        self.static = self.root / 'static'
        self.static.mkdir()
        for name, content in [('console.html', '<h1>LeKiwi console</h1>'),
                              ('console.css', 'body{color:black}'), ('console.js', 'console.log("UI");')]:
            (self.static / name).write_text(content)
        self.app = create_app(self.controller, static_dir=self.static)
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app),
                                       base_url='http://127.0.0.1:8765')

    async def asyncTearDown(self):
        self.episode.cleanup_release.set()
        await self.controller.close()
        await self.client.aclose()
        self.temp.cleanup()

    async def start(self, payload=None):
        response = await self.client.post('/api/runs', json=payload or REQUEST)
        self.assertEqual(response.status_code, 202, response.text)
        await asyncio.wait_for(self.episode.started.wait(), 1)
        return response.json()['id']

    async def finish(self):
        self.episode.release.set()
        await self.controller.task
        await asyncio.sleep(0)

    async def test_default_state_assets_and_no_observation_polling(self):
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['mode'], 'live')
        self.assertIsNone(state['active_run_id'])
        self.assertEqual(state['config']['limits']['max_actions'], 20)
        for path in ('/', '/static/console.css', '/static/console.js'):
            response = await self.client.get(path)
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.headers['x-content-type-options'], 'nosniff')
        for _ in range(3):
            await self.client.get('/api/state')
        self.assertEqual(self.episode.calls, [])

    async def test_start_calls_existing_episode_function_with_bound_request(self):
        request = {**REQUEST, 'model': 'test-model'}
        run_id = await self.start(request)
        call = self.episode.calls[0]
        self.assertEqual(call['server'], 'ipc://unused')
        self.assertEqual(call['model'], 'test-model')
        self.assertEqual(call['robot_id'], REQUEST['robot_id'])
        self.assertEqual(call['limits'], EpisodeLimits(2, 5, 30))
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['active_run_id'], run_id)
        self.assertEqual(state['active']['phase'], 'planning')
        self.assertEqual(state['active']['request']['instruction'], REQUEST['instruction'])
        self.assertEqual(len(self.episode.calls), 1)
        await self.finish()
        persisted = json.loads((self.controller.runs_dir / run_id / 'gui_request.json').read_text())
        self.assertEqual(persisted['model'], 'test-model')

    async def test_null_model_uses_gui_launch_default(self):
        self.controller.model = 'configured-model'
        run_id = await self.start()
        self.assertEqual(self.episode.calls[0]['model'], 'configured-model')
        self.assertEqual(self.controller.run_view(run_id)['request']['model'], 'configured-model')

    async def test_kinematics_path_is_launch_only_and_forwarded_to_existing_episode(self):
        config = self.root / 'verified-mapping.json'
        config.write_text('{}')
        self.controller.kinematics_config = config
        run_id = await self.start()
        self.assertEqual(self.episode.calls[0]['kinematics_config'], config)
        state = (await self.client.get('/api/state')).json()
        self.assertTrue(state['config']['kinematics_configured'])
        self.assertNotIn('kinematics_config', state['config'])
        self.assertEqual(state['active_run_id'], run_id)

    async def test_single_active_until_cancelled_task_cleanup_finishes(self):
        self.episode.hold_cleanup = True
        run_id = await self.start()
        self.assertEqual((await self.client.post('/api/runs', json=REQUEST)).status_code, 409)
        response = await self.client.post(f'/api/runs/{run_id}/stop', json={})
        self.assertEqual(response.status_code, 202)
        await asyncio.wait_for(self.episode.cleaning.wait(), 1)
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['active']['status'], 'stopping')
        self.assertTrue(state['active']['active'])
        self.assertEqual((await self.client.post('/api/runs', json=REQUEST)).status_code, 409)
        # An idempotent second stop must not interrupt cleanup again.
        self.assertEqual((await self.client.post(f'/api/runs/{run_id}/stop', json={})).status_code, 202)
        self.episode.cleanup_release.set()
        await self.controller.task
        await asyncio.sleep(0)
        state = (await self.client.get('/api/state')).json()
        self.assertIsNone(state['active_run_id'])
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['result']['status'], 'uncertain_execution')
        self.assertEqual(view['result']['pending_action_id'], 'action-in-flight')
        self.assertEqual(len(self.episode.calls), 1)

    async def test_prestartup_cancel_creates_inspection_only_record(self):
        view = await self.controller.start(REQUEST)
        await self.controller.stop(view['id'])
        with self.assertRaises(asyncio.CancelledError):
            await self.controller.task
        await asyncio.sleep(0)
        saved = self.controller.run_view(view['id'])
        self.assertEqual(saved['status'], 'interrupted')
        self.assertFalse(saved['result']['automatic_resume'])
        self.assertEqual(self.episode.calls, [])

    async def test_current_before_images_and_public_decision_history(self):
        run_id = await self.start()
        await self.finish()
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['status'], 'completion_claim')
        self.assertFalse(view['active'])
        self.assertFalse(view['result']['physical_success_verified'])
        self.assertEqual(view['current_observation']['observation_id'], 'obs-after')
        self.assertEqual(view['before_observation']['observation_id'], 'obs-before')
        decision = next(event['data'] for event in view['events'] if event['event'] == 'decision')
        self.assertEqual(decision['reason'], 'Visible evidence')
        image = view['current_observation']['images'][0]
        self.assertEqual(image['label'], 'front')
        response = await self.client.get(image['url'])
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers['content-type'], 'image/jpeg')
        self.assertEqual(response.content, b'\xff\xd8\xff\xd9')

    async def test_restarted_gui_never_resumes_running_or_unknown_history(self):
        run_id = '20261116T000000Z-cli'
        record = RunRecord(self.controller.runs_dir / run_id, 'cli-robot', 'CLI goal', EpisodeLimits())
        record.event('planning_started', {'step': 1})
        restarted = GuiController('unused', 'kiwi', self.controller.runs_dir, episode_fn=self.episode)
        state = restarted.snapshot()
        self.assertIsNone(state['active_run_id'])
        self.assertEqual(state['runs'][0]['status'], 'interrupted_unknown')
        self.assertEqual(restarted.run_view(run_id)['status'], 'interrupted_unknown')
        self.assertEqual(read_events(record.events)[0]['event'], 'planning_started')
        self.assertEqual(self.episode.calls, [])

    async def test_history_keeps_demo_mode_separate_from_live_gui(self):
        run_id = 'demo-history'
        record = RunRecord(self.controller.runs_dir / run_id, 'demo', 'synthetic', EpisodeLimits())
        metadata = json.loads((record.path / 'run.json').read_text())
        atomic_json(record.path / 'run.json', {**metadata, 'mode': 'demo'})
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['mode'], 'demo')
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['mode'], 'live')
        self.assertEqual(state['runs'][0]['mode'], 'demo')

    async def test_poll_loads_only_latest_and_last_motion_before_observations(self):
        run_id = 'many-observations'
        record = RunRecord(self.controller.runs_dir / run_id, 'robot', 'goal', EpisodeLimits())
        for number in range(8):
            record.observation(packet(f'obs-{number}'), [])
        record.event('motion_feedback', {'before_observation_id': 'obs-2',
                                         'after_observation_id': 'obs-3'})
        with patch.object(self.controller, '_observation', wraps=self.controller._observation) as load:
            view = self.controller.run_view(run_id)
        self.assertEqual(load.call_count, 2)
        self.assertEqual(view['current_observation']['observation_id'], 'obs-7')
        self.assertEqual(view['before_observation']['observation_id'], 'obs-2')

    async def test_current_kinematics_comes_from_durable_events_without_sensor_polling(self):
        run_id = 'kinematics-history'
        record = RunRecord(self.controller.runs_dir / run_id, 'robot', 'goal', EpisodeLimits())
        record.observation(packet('obs-fk'), [])
        state = {'observation_id': 'obs-fk', 'available': True, 'executable': False,
                 'current_xyz_m': [.1, .2, .3], 'frame': 'arm_base'}
        record.event('kinematics_state', state)
        view = self.controller.run_view(run_id)
        self.assertEqual(view['kinematics'], state)
        self.assertEqual(view['current_observation']['observation_id'], state['observation_id'])
        self.assertEqual(self.episode.calls, [])

    async def test_fallback_retains_unconfirmed_or_mismatched_action(self):
        cases = [
            ('mismatch', 'completed', 'completed', True, 'other-action', True),
            ('unknown', 'error', 'uncertain_execution', None, 'pending-action', True),
            ('incomplete', 'completed', 'completed', None, 'pending-action', True),
            ('complete', 'completed', 'completed', True, 'pending-action', False),
            ('rejected', 'rejected', 'rejected', False, 'pending-action', False),
        ]
        for name, status, outcome, sent, action_id, uncertain in cases:
            run_id = 'fallback-' + name
            record = RunRecord(self.controller.runs_dir / run_id, 'robot', 'goal', EpisodeLimits())
            self.controller.requests[run_id] = dict(REQUEST)
            record.event('action_prepared', {'action_id': 'pending-action'})
            record.event('action_result', {'action_id': 'pending-action', 'packet': {
                'status': status, 'execution': {'action_id': action_id, 'outcome': outcome,
                                                'command_sent': sent},
            }})
            self.controller._fallback_result(run_id, 'interrupted', 'cleanup interrupted')
            result = self.controller.run_view(run_id)['result']
            self.assertEqual(result['status'], 'uncertain_execution' if uncertain else 'interrupted', name)
            self.assertEqual(result['pending_action_id'], 'pending-action' if uncertain else None, name)

    async def test_host_origin_and_json_guards_prevent_cross_site_starts(self):
        for headers in ({'Origin': 'https://untrusted.example'}, {'Host': 'untrusted.example'},
                        {'Origin': 'http://localhost:9999'}):
            response = await self.client.post('/api/runs', json=REQUEST, headers=headers)
            self.assertEqual(response.status_code, 403)
        self.assertEqual((await self.client.get('/api/state', headers={'Host': 'evil.example'})).status_code, 403)
        self.assertEqual((await self.client.post('/api/runs', content='{}')).status_code, 400)
        self.assertEqual((await self.client.post('/api/runs', content='x' * 17000,
                                                headers={'Content-Type': 'application/json'})).status_code, 400)
        self.assertEqual(self.episode.calls, [])

    async def test_invalid_settings_or_missing_execution_intent_cannot_start(self):
        payloads = [
            {**REQUEST, 'execute': False}, {**REQUEST, 'execute': 1},
            {key: value for key, value in REQUEST.items() if key != 'execute'},
            {**REQUEST, 'instruction': '   '}, {**REQUEST, 'robot_id': 'bad\nrobot'},
            {**REQUEST, 'limits': {**REQUEST['limits'], 'max_actions': True}},
            {**REQUEST, 'limits': {**REQUEST['limits'], 'max_steps': 201}},
            {**REQUEST, 'limits': {**REQUEST['limits'], 'max_seconds': 3601}},
            {**REQUEST, 'server': 'tcp://attacker:5556'},
            {**REQUEST, 'kinematics_config': '/another/mapping.json'},
        ]
        for payload in payloads:
            response = await self.client.post('/api/runs', json=payload)
            self.assertEqual(response.status_code, 400, response.text)
        self.assertEqual(self.episode.calls, [])

    async def test_run_file_routes_reject_traversal_and_symlinks(self):
        run_id = await self.start()
        await self.finish()
        directory = self.controller.runs_dir / run_id
        outside = self.root / 'secret.json'
        outside.write_text('{"secret":true}')
        (directory / 'observations' / '0001' / 'image-99.jpg').symlink_to(outside)
        (self.controller.runs_dir / 'alias').symlink_to(directory, target_is_directory=True)
        denied = [f'/api/runs/{run_id}/files/run.json',
                  f'/api/runs/{run_id}/files/observations/0001/image-99.jpg',
                  f'/api/runs/{run_id}/files/%2e%2e/secret.json',
                  '/api/runs/alias', '/api/runs/no-such-run']
        for path in denied:
            self.assertEqual((await self.client.get(path)).status_code, 404, path)
        good = f'/api/runs/{run_id}/files/observations/0001/observation.json'
        self.assertEqual((await self.client.get(good)).status_code, 200)

    async def test_partial_jsonl_is_ignored_until_complete(self):
        run_id = await self.start()
        events = self.controller.runs_dir / run_id / 'events.jsonl'
        with events.open('a') as stream:
            stream.write('{"event":"decision","data":{"reason":"partial"}')
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertFalse(any(event.get('event') == 'decision' for event in view['events']))
        with events.open('a') as stream:
            stream.write('}\n')
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['events'][-1]['data']['reason'], 'partial')

    async def test_stop_unknown_run_never_touches_current_task(self):
        await self.start()
        response = await self.client.post('/api/runs/another/stop', json={})
        self.assertEqual(response.status_code, 409)
        self.assertFalse(self.controller.stopping)
        self.assertFalse(self.controller.task.done())

    async def test_shutdown_cancels_owned_episode_and_preserves_uncertainty(self):
        run_id = await self.start()
        await self.controller.close()
        await asyncio.sleep(0)
        self.assertEqual(self.controller.run_view(run_id)['result']['status'], 'uncertain_execution')
        self.assertEqual(len(self.episode.calls), 1)

    async def test_episode_exception_is_recorded_without_retry(self):
        async def failed(*args, **kwargs):
            raise RuntimeError('planner failed')
        self.controller.episode_fn = failed
        view = await self.controller.start(REQUEST)
        await self.controller.task
        await asyncio.sleep(0)
        result = self.controller.run_view(view['id'])['result']
        self.assertEqual(result['status'], 'gui_error')
        self.assertIn('planner failed', result['detail'])
        self.assertFalse(result['automatic_resume'])
