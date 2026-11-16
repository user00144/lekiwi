"""Real IK → MCP → relay → mock runtime, without model or physical hardware."""
import asyncio
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from harness.client import ServerClient
from harness.episode import EpisodeLimits, EpisodeRunner, MCPRobotTools
from harness.kinematics import ARM_JOINTS, CartesianController

ROOT = Path(__file__).resolve().parents[2]


class CartesianIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_xyz_reaches_encoder_verified_endpoint_through_existing_guard(self):
        with tempfile.TemporaryDirectory() as folder:
            robots, api = f'ipc://{folder}/robots', f'ipc://{folder}/api'
            env = {key: value for key, value in os.environ.items() if key != 'VIRTUAL_ENV'}
            processes = []
            try:
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'server'), 'server', '--bind', robots, '--api', api,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'lekiwi_runtime'), 'lekiwi-runtime',
                    '--mock', '--robot-id', 'xyz-mock', '--server', robots,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
                client = ServerClient(api, timeout=5)
                for _ in range(100):
                    initial = await client.request('xyz-mock', 'observe')
                    if initial.get('status') == 'ok':
                        break
                    await asyncio.sleep(.02)
                self.assertEqual(initial['status'], 'ok', initial)
                ego = initial['observation']['ego']
                config = {'version': 1, 'hardware_alignment_verified': True,
                          'calibration_id': ego['calibration_id'],
                          'model_hash': CartesianController().geometry.model_hash,
                          'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}}
                # Verification is synthetic and applies only to this isolated mock robot.
                cartesian = CartesianController(config)
                desired = {name: ego['joints'][name] for name in ARM_JOINTS}
                desired['shoulder_pan'] += 5
                target = cartesian.geometry.forward(desired)['xyz_m']
                contexts = []

                class Planner:
                    async def decide(self, context, images):
                        contexts.append(context)
                        command = None
                        if len(contexts) == 1:
                            command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 500,
                                       'joints': None, 'cartesian': {'frame': 'arm_base', 'mode': 'absolute',
                                                                   'xyz_m': target, 'approach_axis': None}}
                        return {'robot_id': context['robot_id'],
                                'observation_id': context['current']['observation']['observation_id'],
                                'tool': 'execute' if command else 'task_complete', 'command': command,
                                'plan': ['Solve an XYZ endpoint on a simulated arm'],
                                'reason': 'Synthetic regression without physical hardware.',
                                'expected': 'Measured mock endpoint matches the IK target.',
                                'feedback': 'initial' if command else 'progress', 'evidence': 'Mock encoders'}

                params = StdioServerParameters(command=sys.executable, args=['-m', 'harness', '--server', api])
                run_dir = Path(folder) / 'episode'
                async with stdio_client(params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        runner = EpisodeRunner(Planner(), MCPRobotTools(session), 'xyz-mock',
                                               'Synthetic XYZ endpoint', run_dir,
                                               EpisodeLimits(max_actions=2, max_steps=4, max_seconds=30),
                                               cartesian=cartesian)
                        result = await runner.run()
                self.assertEqual(result['status'], 'completion_claim', result)
                self.assertEqual(result['action_attempts'], 1)
                after = contexts[-1]['current']['observation']['ego']
                actual = cartesian.geometry.forward({name: after['joints'][name] for name in ARM_JOINTS})['xyz_m']
                self.assertLess(math.dist(actual, target), .002)
                self.assertEqual(after['raw']['gripper'], ego['raw']['gripper'])
                self.assertTrue(contexts[1]['previous_result']['execution']['arm_reached'])
                events = [json.loads(line) for line in (run_dir / 'events.jsonl').read_text().splitlines()]
                prepared = [event['data'] for event in events if event['event'] == 'action_prepared']
                self.assertEqual(len(prepared), 1)
                self.assertNotIn('cartesian', prepared[0])
                self.assertEqual(set(prepared[0]['joints']), set(ARM_JOINTS))
                solved = [event['data'] for event in events if event['event'] == 'ik_result']
                self.assertEqual(len(solved), 1)
                self.assertTrue(solved[0]['executable'])
                self.assertEqual(solved[0]['target_xyz_m'], target)
            finally:
                for process in reversed(processes):
                    process.terminate()
                    try:
                        process.communicate(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.communicate()
