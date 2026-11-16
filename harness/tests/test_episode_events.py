"""Live run records are sufficient for a read-only GUI; no extra robot calls."""
import asyncio
import base64
from contextlib import asynccontextmanager
from datetime import datetime
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from harness.episode import EpisodeRunner
from harness.episode_runtime import run_episode
from harness.tools import RobotTools


JPEG = '/9j/2Q=='


def decision(context, tool='blocked'):
    return {
        'robot_id': context['robot_id'],
        'observation_id': context['current']['observation']['observation_id'],
        'tool': tool, 'plan': ['Inspect the scene', 'Confirm the visible result'],
        'reason': 'The current front and wrist views support waiting.',
        'expected': 'A fresh observation with no requested motor motion.',
        'feedback': 'initial', 'evidence': 'The object is visible in both images.',
        'command': ({'action': 'wait', 'x': 0., 'y': 0., 'theta': 0.,
                     'duration_ms': 1, 'joints': None} if tool == 'execute' else None),
    }


class MemoryTools:
    def __init__(self):
        self.sequence = 0
        self.observe_calls = 0
        self.execute_calls = []
        self.action_entered = asyncio.Event()
        self.action_release = asyncio.Event()
        self.action_release.set()
        self.result_status = self.execution_outcome = 'completed'
        self.command_sent = False
        self.result_action_id = None

    def packet(self):
        self.sequence += 1
        oid = f'obs-{self.sequence}'
        return {
            'status': 'ok', 'capabilities': {'guarded_execution': 1},
            'action_context': {'observation_id': oid, 'runtime_id': 'runtime',
                               'movement_allowed': True, 'max_age_s': 60},
            'observation': {
                'observation_id': oid, 'runtime_id': 'runtime', 'ego': {'joints': {}},
                'cameras': {name: {'available': True, 'encoding': 'jpeg_base64',
                                   'data': JPEG, 'timestamp': float(self.sequence)}
                            for name in ('front', 'wrist')},
            },
        }

    async def observe(self, robot_id):
        self.observe_calls += 1
        return RobotTools._content(self.packet())

    async def execute(self, robot_id, **arguments):
        self.execute_calls.append(arguments)
        self.action_entered.set()
        await self.action_release.wait()
        packet = self.packet()
        packet.update(status=self.result_status, execution={
            'action_id': self.result_action_id or arguments['action_id'], 'outcome': self.execution_outcome,
            'command_sent': self.command_sent, 'arm_reached': None,
            'base_stop_command_sent': None, 'base_stop_verified': None,
        })
        if self.result_status == 'error':
            packet['error'] = 'Arm 2 write 46: communication=-6, device=0'
        return RobotTools._content(packet)


class EpisodeEventsTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'run'
        self.tools = MemoryTools()

    def events(self):
        return [json.loads(line) for line in (self.path / 'events.jsonl').read_text().splitlines()]

    async def finish_task(self, task):
        if not task.done():
            task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    async def test_planning_is_visible_while_pending_and_images_are_ready(self):
        entered, release = asyncio.Event(), asyncio.Event()

        async def plan(context, images):
            entered.set()
            await release.wait()
            return decision(context)

        runner = EpisodeRunner(SimpleNamespace(decide=plan), self.tools, 'kiwi', 'Inspect', self.path)
        task = asyncio.create_task(runner.run())
        self.addAsyncCleanup(self.finish_task, task)
        await asyncio.wait_for(entered.wait(), timeout=1)
        events = self.events()
        self.assertEqual([event['event'] for event in events],
                         ['observe_started', 'observation', 'observe_finished', 'planning_started'])
        self.assertEqual(events[-1]['data']['step'], 1)
        self.assertEqual(events[-1]['data']['observations'], 1)
        self.assertEqual(events[-1]['data']['observation_id'], 'obs-1')
        self.assertEqual(json.loads((self.path / 'result.json').read_text())['status'], 'running')
        images = events[1]['data']['images']
        self.assertEqual([image['label'] for image in images], ['front', 'wrist'])
        manifest = json.loads((self.path / 'observations/0001/images.json').read_text())
        self.assertEqual(manifest, {'observation_id': 'obs-1', 'images': images})
        for image in images:
            self.assertFalse(Path(image['path']).is_absolute())
            self.assertEqual(image['mime_type'], 'image/jpeg')
            self.assertEqual((self.path / image['path']).read_bytes(), base64.b64decode(JPEG))
        # Reading the files never creates another observation or motor command.
        self.assertEqual(self.tools.observe_calls, 1)
        self.assertEqual(self.tools.execute_calls, [])
        release.set()
        result = await asyncio.wait_for(task, timeout=1)
        self.assertEqual(result['status'], 'blocked')
        events = self.events()
        self.assertEqual([event['seq'] for event in events], list(range(1, len(events) + 1)))
        elapsed = [event['elapsed_s'] for event in events]
        self.assertEqual(elapsed, sorted(elapsed))
        for event in events:
            self.assertIsNotNone(datetime.fromisoformat(event['at']).tzinfo)
        public = next(event['data'] for event in events if event['event'] == 'decision')
        self.assertEqual(set(public), {'robot_id', 'observation_id', 'tool', 'plan', 'reason',
                                      'expected', 'feedback', 'evidence', 'command'})
        finished = next(event['data'] for event in events if event['event'] == 'planning_finished')
        self.assertEqual(finished['status'], 'completed')
        self.assertEqual(finished['steps'], 1)
        self.assertGreaterEqual(finished['duration_s'], 0)

    async def test_action_progress_links_decision_result_and_feedback(self):
        choices = iter(('execute', 'blocked'))

        async def plan(context, images):
            return decision(context, next(choices))

        runner = EpisodeRunner(SimpleNamespace(decide=plan), self.tools, 'kiwi', 'Inspect', self.path)
        result = await runner.run()
        self.assertEqual(result['status'], 'blocked')
        events = self.events()
        by_name = {event['event']: event for event in events}
        action_id = self.tools.execute_calls[0]['action_id']
        for name in ('action_prepared', 'action_started', 'action_result', 'action_finished'):
            self.assertEqual(by_name[name]['data']['action_id'], action_id)
        started, finished = by_name['action_started'], by_name['action_finished']
        self.assertEqual(started['data']['observation_id'], 'obs-1')
        self.assertEqual(started['data']['action_attempts'], 1)
        self.assertEqual(finished['data']['status'], 'completed')
        self.assertEqual(finished['data']['transport_status'], 'completed')
        self.assertEqual(finished['data']['result_status'], 'completed')
        self.assertLess(started['seq'], by_name['action_result']['seq'])
        self.assertLess(by_name['action_result']['seq'], finished['seq'])
        self.assertGreaterEqual(finished['data']['duration_s'], 0)
        self.assertEqual(by_name['motion_feedback']['data']['after_observation_id'], 'obs-2')
        self.assertEqual(len(self.tools.execute_calls), 1)
        self.assertEqual(self.tools.observe_calls, 1)

    async def test_received_robot_error_is_not_reported_as_action_completed(self):
        self.tools.result_status = 'error'
        self.tools.execution_outcome = 'uncertain_execution'
        self.tools.command_sent = None
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda c, _: decision(c, 'execute')))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        finished = next(event['data'] for event in self.events() if event['event'] == 'action_finished')
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(finished['status'], 'error')
        self.assertEqual(finished['transport_status'], 'completed')
        self.assertEqual(finished['result_status'], 'error')
        self.assertEqual(finished['execution_outcome'], 'uncertain_execution')
        self.assertIn('communication=-6', finished['result_error'])
        self.assertEqual(len(self.tools.execute_calls), 1)
        self.assertEqual(planner.decide.await_count, 1)

    async def test_mismatched_completed_result_is_action_error(self):
        self.tools.result_action_id = 'different-action'
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda c, _: decision(c, 'execute')))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        finished = next(event['data'] for event in self.events() if event['event'] == 'action_finished')
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(finished['status'], 'error')
        self.assertEqual(finished['transport_status'], 'completed')
        self.assertEqual(finished['result_status'], 'completed')
        self.assertEqual(finished['error'], 'uncertain_execution')

    async def test_safe_rejection_is_distinct_from_transport_failure(self):
        self.tools.result_status = self.tools.execution_outcome = 'rejected'
        choices = iter(('execute', 'blocked'))
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda c, _: decision(c, next(choices))))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        finished = next(event['data'] for event in self.events() if event['event'] == 'action_finished')
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(finished['status'], 'rejected')
        self.assertEqual(finished['transport_status'], 'completed')
        self.assertFalse(finished['command_sent'])
        self.assertEqual(planner.decide.await_count, 2)

    async def test_completed_without_observation_is_explicit_in_action_finish(self):
        self.tools.result_status = 'completed_without_observation'
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda c, _: decision(c, 'execute')))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        finished = next(event['data'] for event in self.events() if event['event'] == 'action_finished')
        self.assertEqual(result['status'], 'post_motion_observation_failed')
        self.assertEqual(finished['status'], 'completed_without_observation')
        self.assertEqual(finished['transport_status'], 'completed')

    async def test_transport_exception_has_no_successful_response_status(self):
        self.tools.execute = AsyncMock(side_effect=OSError('connection closed'))
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda c, _: decision(c, 'execute')))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        finished = next(event['data'] for event in self.events() if event['event'] == 'action_finished')
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(finished['status'], 'error')
        self.assertEqual(finished['transport_status'], 'error')
        self.assertNotIn('result_status', finished)

    async def test_cancelled_action_closes_phase_and_preserves_uncertainty(self):
        self.tools.action_release.clear()

        async def plan(context, images):
            return decision(context, 'execute')

        runner = EpisodeRunner(SimpleNamespace(decide=plan), self.tools, 'kiwi', 'Inspect', self.path)
        task = asyncio.create_task(runner.run())
        self.addAsyncCleanup(self.finish_task, task)
        await asyncio.wait_for(self.tools.action_entered.wait(), timeout=1)
        self.assertEqual(self.events()[-1]['event'], 'action_started')
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        events = self.events()
        finished = next(event['data'] for event in events if event['event'] == 'action_finished')
        self.assertEqual(finished['status'], 'cancelled')
        self.assertEqual(finished['transport_status'], 'cancelled')
        self.assertEqual(finished['error'], 'CancelledError')
        self.assertNotIn('action_result', [event['event'] for event in events])
        result = json.loads((self.path / 'result.json').read_text())
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(result['pending_action_id'], self.tools.execute_calls[0]['action_id'])
        self.assertEqual(len(self.tools.execute_calls), 1)

    async def test_invalid_planner_response_finishes_phase_without_public_decision(self):
        planner = SimpleNamespace(decide=AsyncMock(return_value={'unexpected': 'not a decision'}))
        result = await EpisodeRunner(planner, self.tools, 'kiwi', 'Inspect', self.path).run()
        self.assertEqual(result['status'], 'invalid_decision')
        events = self.events()
        finished = next(event['data'] for event in events if event['event'] == 'planning_finished')
        self.assertEqual(finished['status'], 'error')
        self.assertEqual(finished['error'], 'invalid_decision')
        self.assertNotIn('decision', [event['event'] for event in events])
        self.assertEqual(self.tools.execute_calls, [])

    async def test_cancelled_startup_has_record_before_first_observation(self):
        entered = asyncio.Event()

        async def enter():
            entered.set()
            await asyncio.Event().wait()

        sdk = SimpleNamespace(__aenter__=enter, close=AsyncMock())
        with patch('harness.episode_runtime.AsyncCodex', return_value=sdk):
            task = asyncio.create_task(run_episode('unused', 'model', 'kiwi', 'Inspect', run_dir=self.path))
            self.addAsyncCleanup(self.finish_task, task)
            await asyncio.wait_for(entered.wait(), timeout=1)
            event = self.events()[-1]
            self.assertEqual(event['event'], 'startup_started')
            self.assertEqual(event['data']['component'], 'planner')
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
        events = self.events()
        self.assertEqual(events[-2]['event'], 'startup_finished')
        self.assertEqual(events[-2]['data']['status'], 'cancelled')
        result = json.loads((self.path / 'result.json').read_text())
        self.assertEqual(result['status'], 'interrupted')
        self.assertEqual(result['observations'], 0)
        self.assertEqual(result['action_attempts'], 0)
        sdk.close.assert_awaited_once()

    async def test_transport_connection_failure_finishes_startup_phase(self):
        sdk = SimpleNamespace(__aenter__=AsyncMock(), close=AsyncMock(),
                              thread_start=AsyncMock(return_value=SimpleNamespace(id='thread')))

        @asynccontextmanager
        async def failed_transport(params):
            raise RuntimeError('connection failed')
            yield

        with patch('harness.episode_runtime.AsyncCodex', return_value=sdk), \
                patch('harness.episode_runtime.stdio_client', failed_transport):
            result = await run_episode('unused', 'model', 'kiwi', 'Inspect', run_dir=self.path)
        self.assertEqual(result['status'], 'startup_error')
        transport = [event for event in self.events() if event['data'].get('component') == 'transport']
        self.assertEqual([event['event'] for event in transport], ['startup_started', 'startup_finished'])
        self.assertEqual(transport[-1]['data']['status'], 'error')
        self.assertEqual(transport[-1]['data']['error'], 'RuntimeError')
        self.assertEqual(result['observations'], 0)
