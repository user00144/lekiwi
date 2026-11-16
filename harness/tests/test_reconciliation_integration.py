"""Real IPC and MCP recovery flow with an in-memory robot, never hardware."""

import asyncio
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from harness.client import ServerClient
from harness.episode import EpisodeLimits, EpisodeRunner, MCPRobotTools


ROOT = Path(__file__).resolve().parents[2]
FAKE_RUNTIME = r'''
from copy import deepcopy
import json, os, time
from pathlib import Path
from lekiwi_runtime.client import RobotClient

NAMES = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
LOG = Path(os.environ['LEKIWI_RECONCILE_TEST_LOG'])
def log(event, **data):
    with LOG.open('a') as output: output.write(json.dumps({'event':event, **data})+'\n')

class MemoryRobot:
    def __init__(self):
        self.joints = dict.fromkeys(NAMES, 0.)
        self.joints['gripper'] = 50.
        self.executions = 0
    def observe(self):
        stamp = time.time()
        return {'timestamp':stamp, 'simulated':True,
                'cameras':{name:{'available':True,'timestamp':stamp,'age_s':0.,'error':None,
                                  'encoding':'jpeg_base64','data':'/9j/2Q=='} for name in ('front','wrist')},
                'ego':{'available':True,'timestamp':stamp,'error':None,'joints':dict(self.joints),
                       'raw':dict.fromkeys(NAMES,2048),'calibration_id':'integration-calibration',
                       'calibration_status':{'valid':True,'out_of_range':[],'missing_joints':[]},
                       'limits':{name:{'min':0 if name=='gripper' else -100,'max':100,
                                       'max_speed':50 if name=='gripper' else 30,'unit':'%' if name=='gripper' else 'deg'}
                                 for name in NAMES}},
                'radar':{'available':False},'imu':{'available':False}}
    def certificate(self):
        return {'verified':True,'revision':'verified-arm-reconcile-v3',
                'calibration_id':'integration-calibration','timestamp':time.time(),
                'held_goal_raw':dict.fromkeys(NAMES,2048),'goals':dict(self.joints),
                'measured_raw':dict.fromkeys(NAMES,2048),'measured':dict(self.joints),
                'health':{name:{'torque_enabled':True,'mode':0,'status':0,'temperature_c':25} for name in NAMES},
                'stability':{'sample_count':3,'interval_s':.21,'max_drift':dict.fromkeys(NAMES,0.),
                             'limits':{name:2. if name=='gripper' else .5 for name in NAMES}},
                'tolerances':{name:5. if name=='gripper' else 2. for name in NAMES}}
    def execute_guarded(self, command):
        log('execute',command=command)
        self.executions += 1
        if self.executions == 1:
            before = dict(self.joints)
            self.joints['wrist_roll'] = 3.
            motion = {'outcome':'target_not_reached','reached':False,
                      'targets':command['joints'],'initial_joints':before,
                      'measured':{'wrist_roll':3.},'errors':{'wrist_roll':-3.},
                      'failed_joints':['wrist_roll'],'tolerances':{'wrist_roll':2.},
                      'units':{'wrist_roll':'deg'},'timestamp':time.time(),
                      'duration_ms':command['duration_ms'],'communication_recoveries':[],
                      'recovery_eligible':True,'recovery_reason':'verified_measured_hold',
                      'progress':{'wrist_roll':{'initial':0.,'requested_delta':6.,'error_reduction':3.,
                                              'required_progress':1.2,'remaining_error':3.,'eligible':True}},
                      'hold':self.certificate()}
            return {'outcome':'incomplete','command_sent':True,'arm_reached':False,
                    'arm_hold_verified':True,'arm_motion':motion,'simulated':True,
                    'error':'Mock target not reached; measured hold verified'}
        self.joints.update(command.get('joints',{}))
        return {'outcome':'completed','command_sent':True,'arm_reached':True,
                'arm_motion':{'reached':True,'measured':dict(self.joints)},'simulated':True}
    def reconcile_arm_hold(self, certificate):
        assert certificate['verified'] is True
        assert certificate['measured'] == self.joints
        log('reconcile_read_only')
        return self.certificate()
    def execute(self, command):
        raise AssertionError('No legacy dispatch in reconciliation test')

RobotClient(MemoryRobot(),'memory',os.environ['LEKIWI_RECONCILE_TEST_ROBOT_ENDPOINT']).run()
'''


class ReconciliationIntegrationTest(unittest.IsolatedAsyncioTestCase):
    @asynccontextmanager
    async def stack(self):
        with tempfile.TemporaryDirectory(prefix='lekiwi-reconcile-') as directory:
            folder = Path(directory)
            robots, api = f'ipc://{folder}/robots', f'ipc://{folder}/api'
            command_log = folder / 'commands.jsonl'
            command_log.touch()
            environment = {key: value for key, value in os.environ.items() if key != 'VIRTUAL_ENV'}
            environment.update(PYTHONPATH=os.pathsep.join(str(ROOT / name / 'src') for name in ('server', 'lekiwi_runtime')),
                               LEKIWI_RECONCILE_TEST_LOG=str(command_log),
                               LEKIWI_RECONCILE_TEST_ROBOT_ENDPOINT=robots)
            processes, logs = [], []
            try:
                for name, args in (
                    ('server', [str(ROOT / 'server/main.py'), '--bind', robots, '--api', api, '--timeout', '3']),
                    ('lekiwi_runtime', ['-c', FAKE_RUNTIME]),
                ):
                    log = (folder / f'{name}.stderr').open('w+')
                    logs.append(log)
                    processes.append(subprocess.Popen([str(ROOT / name / '.venv/bin/python'), *args],
                                                      env=environment, stdout=subprocess.DEVNULL, stderr=log))
                client = ServerClient(api, timeout=4)
                deadline = time.monotonic() + 10
                while True:
                    if any(process.poll() is not None for process in processes):
                        for log in logs:
                            log.seek(0)
                        self.fail('Isolated mock stack exited: ' + '\n'.join(log.read() for log in logs))
                    try:
                        if (await client.request('memory', 'observe')).get('status') == 'ok':
                            break
                    except TimeoutError:
                        pass
                    if time.monotonic() > deadline:
                        self.fail('Isolated mock stack did not start')
                    await asyncio.sleep(.02)
                yield client, command_log
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.terminate()
                    try:
                        await asyncio.to_thread(process.wait, timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        await asyncio.to_thread(process.wait)
                for log in logs:
                    log.close()

    async def test_mcp_episode_reconciles_once_and_sends_revised_action(self):
        async with self.stack() as (client, command_log):
            contexts = []

            class Planner:
                async def decide(self, context, images):
                    contexts.append(context)
                    observed = context['current']['observation']
                    command = None
                    if len(contexts) <= 2:
                        joints = dict.fromkeys(('shoulder_pan', 'shoulder_lift', 'elbow_flex',
                                                'wrist_flex', 'wrist_roll', 'gripper'))
                        joints['wrist_roll'] = 6. if len(contexts) == 1 else observed['ego']['joints']['wrist_roll'] + 2.5
                        command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0.,
                                   'duration_ms': 1000 if len(contexts) == 1 else 3000,
                                   'joints': joints, 'cartesian': None}
                    return {'robot_id': 'memory', 'observation_id': observed['observation_id'],
                            'tool': 'execute' if command else 'task_complete',
                            'plan': ['Inspect actual partial movement', 'Use a smaller revised target'],
                            'reason': 'Current simulated feedback supports the revised bounded movement',
                            'expected': 'Wrist moves to the smaller target after verified hold',
                            'feedback': 'initial' if len(contexts) == 1 else 'progress',
                            'evidence': 'Inspect measured joint feedback and current simulated images',
                            'command': command}

            parameters = StdioServerParameters(command=str(ROOT / 'harness/.venv/bin/python'),
                                                args=[str(ROOT / 'harness/main.py'), '--server', client.endpoint],
                                                env={'PYTHONPATH': str(ROOT / 'harness/src')})
            run_dir = command_log.parent / 'episode'
            async with stdio_client(parameters) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    runner = EpisodeRunner(Planner(), MCPRobotTools(session), 'memory',
                                           'Continue from known partial mock movement', run_dir,
                                           EpisodeLimits(max_actions=3, max_steps=5, max_seconds=20))
                    result = await runner.run()
            self.assertEqual(result['status'], 'completion_claim', result)
            self.assertEqual(result['action_attempts'], 2)
            self.assertFalse(result['physical_success_verified'])
            calls = [json.loads(line) for line in command_log.read_text().splitlines()]
            self.assertEqual([call['event'] for call in calls], ['execute', 'reconcile_read_only', 'execute'])
            first, second = calls[0]['command'], calls[2]['command']
            self.assertNotEqual(first['action_id'], second['action_id'])
            self.assertNotEqual(first['observation_id'], second['observation_id'])
            self.assertEqual(first['joints']['wrist_roll'], 6.)
            self.assertEqual(second['joints']['wrist_roll'], 5.5)
            self.assertEqual(second['duration_ms'], 3000)
            self.assertTrue(contexts[1]['motion_failure_feedback'])
            self.assertEqual(contexts[1]['current']['observation']['ego']['joints']['wrist_roll'], 3.)
            self.assertIsNone(contexts[1]['completion_candidate'])
            records = [json.loads(line) for line in (run_dir / 'events.jsonl').read_text().splitlines()]
            results = [record['data']['packet'] for record in records if record['event'] == 'action_result']
            self.assertEqual([packet['status'] for packet in results], ['incomplete', 'completed'])
            self.assertFalse(results[0]['execution']['arm_reached'])


if __name__ == '__main__':
    unittest.main()
