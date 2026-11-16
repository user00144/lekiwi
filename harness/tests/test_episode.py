import asyncio
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from harness.episode import EpisodeLimits, EpisodeRunner
from harness.episode_runtime import CodexPlanner, isolated_planner_config, run_episode
from harness.tools import RobotTools


class FakeTools:
    def __init__(self):
        self.sequence = 0
        self.calls = []
        self.joint = 0.
        self.behavior = 'completed'
        self.capability = True
        self.recorded = False

    def packet(self):
        self.sequence += 1
        oid = f'obs-{self.sequence}'
        return {'status': 'ok', 'capabilities': {'guarded_execution': 1 if self.capability else 0},
                'source': {'recorded': self.recorded},
                'action_context': {'observation_id': oid, 'runtime_id': 'runtime',
                                   'movement_allowed': True, 'max_age_s': 10},
                'observation': {'observation_id': oid, 'runtime_id': 'runtime',
                                'ego': {'joints': {'wrist_roll': self.joint}},
                                'cameras': {name: {'available': True, 'encoding': 'jpeg_base64',
                                                  'data': '/9j/2Q==', 'timestamp': 2.}
                                            for name in ('front', 'wrist')}}}

    async def observe(self, robot_id):
        return RobotTools._content(self.packet())

    async def execute(self, robot_id, **args):
        self.calls.append({'robot_id': robot_id, **args})
        if self.behavior == 'timeout':
            await asyncio.sleep(10)
        if self.behavior != 'no_progress':
            self.joint = (args.get('joints') or {}).get('wrist_roll', self.joint)
        packet = self.packet()
        packet.update(status='completed', execution={
            'action_id': args['action_id'], 'outcome': 'completed', 'command_sent': True,
            'arm_reached': self.behavior != 'no_progress', 'motion_finished_at': 1.,
            'base_stop_command_sent': True, 'base_stop_verified': False})
        if self.behavior == 'rejected':
            packet['status'] = packet['execution']['outcome'] = 'rejected'
            packet['execution']['command_sent'] = False
        elif self.behavior == 'missing_observation':
            packet['status'] = 'completed_without_observation'
            del packet['observation']
        elif self.behavior == 'stale_camera':
            packet['observation']['cameras']['wrist']['timestamp'] = 1.
        elif self.behavior == 'mismatched_id':
            packet['execution']['action_id'] = 'wrong'
        elif self.behavior == 'runtime_changed':
            packet['observation']['runtime_id'] = packet['action_context']['runtime_id'] = 'new'
        return RobotTools._content(packet)


def decision(context, tool='blocked', *, target=3., observation_id=None, robot_id=None):
    command = None
    if tool == 'execute':
        command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 100,
                   'joints': {name: target if name == 'wrist_roll' else None for name in
                              ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')}}
    return {'robot_id': robot_id or context['robot_id'],
            'observation_id': observation_id or context['current']['observation']['observation_id'],
            'tool': tool, 'plan': ['Inspect result'], 'reason': 'Current views support this step.',
            'expected': 'Joint changes by the requested amount', 'feedback': 'uncertain',
            'evidence': 'Current wrist image shows the object retained.', 'command': command}


class ScriptedPlanner:
    def __init__(self, *choices):
        self.choices = iter(choices)
        self.contexts = []
        self.images = []

    async def decide(self, context, images):
        self.contexts.append(context)
        self.images.append(images)
        choice = next(self.choices, 'blocked')
        return choice(context) if callable(choice) else decision(context, choice)


class EpisodeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'run'
        self.tools = FakeTools()

    async def asyncTearDown(self):
        self.temp.cleanup()

    async def run_episode(self, planner, **limits):
        self.runner = EpisodeRunner(planner, self.tools, 'kiwi', 'Pick up the object', self.path,
                                    EpisodeLimits(**limits))
        return await self.runner.run()

    async def test_one_action_feedback_and_two_fresh_completion_claims(self):
        planner = ScriptedPlanner('execute', 'task_complete', 'task_complete')
        result = await self.run_episode(planner)
        self.assertEqual(result['status'], 'completion_claim')
        self.assertFalse(result['physical_success_verified'])
        self.assertEqual(len(self.tools.calls), 1)
        self.assertEqual(result['observations'], 3)
        self.assertNotEqual(result['completion_claim']['observation_id'],
                            result['completion_claim']['first_claim']['observation_id'])
        self.assertEqual(planner.contexts[1]['pending_motion_feedback']['joint_delta'], {'wrist_roll': 3.})
        self.assertTrue(any(label.startswith('BEFORE LAST MOTION') for label, _, _ in planner.images[1]))
        sent = self.tools.calls[0]
        self.assertEqual(sent['observation_id'], 'obs-1')
        events = [json.loads(x) for x in (self.path / 'events.jsonl').read_text().splitlines()]
        prepared = next(x['data'] for x in events if x['event'] == 'action_prepared')
        self.assertEqual(prepared['action_id'], sent['action_id'])
        self.assertEqual(json.loads((self.path / 'result.json').read_text())['status'], 'completion_claim')

    async def test_capability_required_no_downgrade(self):
        self.tools.capability = False
        result = await self.run_episode(ScriptedPlanner('execute'))
        self.assertEqual(result['status'], 'guarded_execution_unavailable')
        self.assertEqual(self.tools.calls, [])

    async def test_recorded_observation_cannot_authorize_motion(self):
        self.tools.recorded = True
        result = await self.run_episode(ScriptedPlanner('execute'))
        self.assertEqual(result['status'], 'recorded_observation')
        self.assertEqual(self.tools.calls, [])

    async def test_robot_bound_to_episode(self):
        result = await self.run_episode(ScriptedPlanner(lambda c: decision(c, 'execute', robot_id='other')))
        self.assertEqual(result['status'], 'invalid_decision')
        self.assertEqual(self.tools.calls, [])

    async def test_malformed_model_command_halts_before_dispatch(self):
        def invalid(context):
            output = decision(context, 'execute')
            output['command']['duration_ms'] = True
            return output
        result = await self.run_episode(ScriptedPlanner(invalid))
        self.assertEqual(result['status'], 'invalid_decision')
        self.assertEqual(self.tools.calls, [])

    async def test_stale_decision_reobserves_without_sending(self):
        planner = ScriptedPlanner(lambda c: decision(c, 'execute', observation_id='old'), 'blocked')
        result = await self.run_episode(planner)
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(result['observations'], 2)
        self.assertEqual(self.tools.calls, [])

    async def test_action_budget_checked_before_second_send(self):
        result = await self.run_episode(ScriptedPlanner('execute', lambda c: decision(c, 'execute', target=6.)),
                                        max_actions=1)
        self.assertEqual(result['status'], 'budget_actions')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_decision_budget_includes_observe_and_completion(self):
        result = await self.run_episode(ScriptedPlanner('task_complete', 'task_complete'), max_steps=1)
        self.assertEqual(result['status'], 'budget_steps')
        self.assertIsNone(result['completion_claim'])
        self.assertEqual(result['observations'], 2)

    async def test_time_budget_interrupts_planning_before_motion(self):
        planner = SimpleNamespace(decide=AsyncMock(side_effect=lambda *_: asyncio.sleep(10)))
        async def slow(*args):
            await asyncio.sleep(10)
        planner.decide = slow
        result = await self.run_episode(planner, max_seconds=.03)
        self.assertEqual(result['status'], 'budget_seconds')
        self.assertEqual(self.tools.calls, [])

    async def test_uncertain_timeout_is_never_replayed(self):
        self.tools.behavior = 'timeout'
        result = await self.run_episode(ScriptedPlanner('execute', 'execute'), max_seconds=.2)
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(len(self.tools.calls), 1)
        self.assertEqual(result['pending_action_id'], self.tools.calls[0]['action_id'])

    async def test_cancellation_keeps_uncertain_action_record(self):
        self.tools.behavior = 'timeout'
        task = asyncio.create_task(self.run_episode(ScriptedPlanner('execute')))
        for _ in range(100):
            if self.tools.calls:
                break
            await asyncio.sleep(.001)
        self.assertEqual(len(self.tools.calls), 1)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        saved = json.loads((self.path / 'result.json').read_text())
        self.assertEqual(saved['status'], 'uncertain_execution')
        self.assertEqual(saved['pending_action_id'], self.tools.calls[0]['action_id'])

    async def test_success_without_fresh_post_observation_halts(self):
        self.tools.behavior = 'missing_observation'
        result = await self.run_episode(ScriptedPlanner('execute', 'execute'))
        self.assertEqual(result['status'], 'post_motion_observation_failed')
        self.assertIsNone(result['pending_action_id'])
        self.assertEqual(len(self.tools.calls), 1)

    async def test_postmotion_barrier_checks_camera_time_not_just_new_id(self):
        self.tools.behavior = 'stale_camera'
        result = await self.run_episode(ScriptedPlanner('execute', 'execute'))
        self.assertEqual(result['status'], 'post_motion_observation_failed')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_mismatched_action_result_is_unknown(self):
        self.tools.behavior = 'mismatched_id'
        result = await self.run_episode(ScriptedPlanner('execute'))
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(result['pending_action_id'], self.tools.calls[0]['action_id'])

    async def test_runtime_restart_stops_episode(self):
        self.tools.behavior = 'runtime_changed'
        result = await self.run_episode(ScriptedPlanner('execute', 'execute'))
        self.assertEqual(result['status'], 'runtime_changed')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_safe_rejection_can_reobserve_and_choose_different_action(self):
        self.tools.behavior = 'rejected'
        def revised(context):
            self.tools.behavior = 'completed'
            return decision(context, 'execute', target=6.)
        planner = ScriptedPlanner('execute', revised, 'blocked')
        result = await self.run_episode(planner)
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(len(self.tools.calls), 2)
        self.assertNotEqual(self.tools.calls[0]['observation_id'], self.tools.calls[1]['observation_id'])

    async def test_unchanged_rejected_command_is_not_repeated(self):
        self.tools.behavior = 'rejected'
        result = await self.run_episode(ScriptedPlanner('execute', 'execute'))
        self.assertEqual(result['status'], 'repeated_failed_action')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_no_measured_joint_progress_blocks_identical_retry(self):
        self.tools.behavior = 'no_progress'
        result = await self.run_episode(ScriptedPlanner(lambda c: decision(c, 'execute', target=6.),
                                                       lambda c: decision(c, 'execute', target=6.)))
        self.assertEqual(result['status'], 'repeated_failed_action')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_existing_run_is_inspection_only_never_auto_resumed(self):
        await self.run_episode(ScriptedPlanner('blocked'))
        with self.assertRaises(FileExistsError):
            EpisodeRunner(ScriptedPlanner('execute'), self.tools, 'kiwi', 'task', self.path)
        self.assertEqual(self.tools.calls, [])

    async def test_snapshot_execution_rejected_before_sdk_start(self):
        with self.assertRaisesRegex(ValueError, 'Snapshot'):
            await run_episode('unused', None, 'kiwi', 'task', snapshot=Path('unused'))

    async def test_startup_failure_is_recorded_before_any_motor_connection(self):
        sdk = SimpleNamespace(__aenter__=AsyncMock(side_effect=RuntimeError('startup failed')), close=AsyncMock())
        with patch('harness.episode_runtime.AsyncCodex', return_value=sdk):
            result = await run_episode('unused', None, 'kiwi', 'task', run_dir=self.path)
        self.assertEqual(result['status'], 'startup_error')
        self.assertEqual(result['action_attempts'], 0)
        self.assertEqual(json.loads((self.path / 'result.json').read_text())['status'], 'startup_error')
        sdk.close.assert_awaited_once()


class PlannerTest(unittest.IsolatedAsyncioTestCase):
    async def test_isolated_home_reuses_only_auth_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            original = Path(directory)
            (original / 'auth.json').write_text('{}')
            (original / 'config.toml').write_text('[mcp_servers.other]\ncommand="dangerous"')
            with patch.dict('os.environ', {'CODEX_HOME': directory}):
                async with isolated_planner_config() as (config, cwd):
                    isolated = Path(config.env['CODEX_HOME'])
                    self.assertTrue((isolated / 'auth.json').is_symlink())
                    self.assertEqual((isolated / 'auth.json').resolve(), original / 'auth.json')
                    self.assertFalse((isolated / 'config.toml').exists())
                    self.assertNotEqual(cwd, original)
                    self.assertIn('features.shell_tool=false', config.config_overrides)
                    self.assertIn('mcp_servers={}', config.config_overrides)
                self.assertFalse(isolated.exists())

    async def test_cancelled_planner_requests_turn_interrupt(self):
        async def wait_forever():
            await asyncio.sleep(10)
        turn = SimpleNamespace(run=wait_forever, interrupt=AsyncMock())
        thread = SimpleNamespace(turn=AsyncMock(return_value=turn))
        planner = CodexPlanner(thread)
        with self.assertRaises(TimeoutError):
            async with asyncio.timeout(.01):
                await planner.decide({'task': 'test'}, [])
        turn.interrupt.assert_awaited_once()

    async def test_decision_payload_uses_untrusted_sensor_message(self):
        from openai_codex import ExternalMessage
        turn = SimpleNamespace(run=AsyncMock(return_value=SimpleNamespace(
            status=SimpleNamespace(value='completed'), items=[], final_response='{}')))
        thread = SimpleNamespace(turn=AsyncMock(return_value=turn))
        await CodexPlanner(thread).decide({'task': 'test'}, [('CURRENT front', '/9j/2Q==', 'image/jpeg')])
        message = thread.turn.call_args.args[0]
        self.assertIsInstance(message, ExternalMessage)
        self.assertEqual(message.content[-1]['type'], 'input_image')
        self.assertIn('output_schema', thread.turn.call_args.kwargs)
