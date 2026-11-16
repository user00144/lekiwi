"""A real relay/MCP/mock-runtime round trip for pre-dispatch rejection recovery."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from harness.client import ServerClient
from harness.episode import EpisodeLimits, EpisodeRunner
from harness.episode_runtime import MCPRobotTools

ROOT = Path(__file__).resolve().parents[2]
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


class RecoveryPlanner:
    def __init__(self):
        self.contexts = []

    async def decide(self, context, images):
        self.contexts.append(context)
        step = len(self.contexts)
        command = None
        if step <= 2:
            joint, target = ('elbow_flex', 88.) if step == 1 else ('wrist_roll', 5.)
            command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 500,
                       'joints': {name: target if name == joint else None for name in JOINTS}}
        return {'robot_id': context['robot_id'],
                'observation_id': context['current']['observation']['observation_id'],
                'tool': 'execute' if command else 'task_complete', 'command': command,
                'plan': ['Inspect rejection', 'Choose a different mock joint'],
                'reason': 'Deterministic mock regression; no physical robot is connected.',
                'expected': 'Only the second, valid mock joint command changes state.',
                'feedback': 'initial' if step == 1 else 'progress',
                'evidence': 'Current mock joint feedback'}


class RecoveryIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_calibration_rejection_replans_through_mcp_without_halting_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            robots, api = f'ipc://{folder}/robots', f'ipc://{folder}/api'
            env = {k: v for k, v in os.environ.items() if k != 'VIRTUAL_ENV'}
            processes = []
            try:
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'server'), 'server',
                    '--bind', robots, '--api', api,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
                # Reproduce the user's measured raw value using simulated hardware only.
                script = '''
import sys
from lekiwi_runtime.orin import OrinRobot
from lekiwi_runtime.client import RobotClient
with OrinRobot(mock=True) as robot:
    robot.arm._mock_raw['elbow_flex'] = 3133
    RobotClient(robot, 'recovery-mock', sys.argv[1]).run()
'''
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'lekiwi_runtime'), 'python', '-c', script, robots,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
                client = ServerClient(api, timeout=5)
                for _ in range(100):
                    response = await client.request('recovery-mock', 'observe')
                    if response.get('status') == 'ok':
                        break
                    await asyncio.sleep(.02)
                self.assertEqual(response['status'], 'ok', response)
                params = StdioServerParameters(command=sys.executable, args=['-m', 'harness', '--server', api])
                planner = RecoveryPlanner()
                run_dir = Path(folder) / 'episode'
                async with stdio_client(params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        runner = EpisodeRunner(planner, MCPRobotTools(session), 'recovery-mock',
                                               'Verify bounded recovery using simulated hardware', run_dir,
                                               EpisodeLimits(max_actions=3, max_steps=5, max_seconds=20))
                        result = await runner.run()
                self.assertEqual(result['status'], 'completion_claim', result)
                self.assertEqual(result['action_attempts'], 2)
                self.assertIsNone(result['pending_action_id'])
                rejected = planner.contexts[1]['previous_result']
                self.assertEqual(rejected['status'], 'rejected')
                self.assertIs(rejected['execution']['command_sent'], False)
                self.assertIn('Current elbow_flex is outside calibration', rejected['error'])
                first_obs = planner.contexts[0]['current']['observation']
                recovery_obs = planner.contexts[1]['current']['observation']
                self.assertNotEqual(first_obs['observation_id'], recovery_obs['observation_id'])
                self.assertEqual(first_obs['runtime_id'], recovery_obs['runtime_id'])
                self.assertEqual(recovery_obs['ego']['raw']['elbow_flex'], 3133)
                final = planner.contexts[-1]['current']['observation']['ego']
                self.assertEqual(final['raw']['elbow_flex'], 3133)
                self.assertAlmostEqual(final['joints']['wrist_roll'], 5., delta=.1)
                events = [json.loads(line) for line in (run_dir / 'events.jsonl').read_text().splitlines()]
                outcomes = [e['data']['packet']['status'] for e in events if e['event'] == 'action_result']
                self.assertEqual(outcomes, ['rejected', 'completed'])
            finally:
                for process in reversed(processes):
                    process.terminate()
                    try:
                        process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.communicate()
