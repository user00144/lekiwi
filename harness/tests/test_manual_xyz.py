"""One-shot operator moves: real IK, guarded dispatch, no hardware connection."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import httpx
from harness.gui import GuiController, create_app
from harness.kinematics import ARM_JOINTS, DEFAULT_URDF
from harness.manual_xyz import ExecuteInput, ManualBusy, ManualXYZ, PrepareInput
from test_episode_cartesian import ArmTools, configuration


def prepare_input(**changes):
    return PrepareInput.model_validate(dict(robot_id='xyz-mock', delta_m=[0., 0., .005], **changes))


def execute_input(token, **changes):
    return ExecuteInput.model_validate({'token': token, 'execute': True, 'alignment_confirmed': True,
                                       'clearance_confirmed': True, **changes})


class ManualTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.tools = ArmTools()
        self.now = [0.]
        self.manual = ManualXYZ(self.folder.name, 'unused', tools=self.tools, clock=lambda: self.now[0])
        self.addAsyncCleanup(self.manual.close)

    async def test_review_then_exactly_one_guarded_move_and_post_observation(self):
        view = await self.manual.prepare(prepare_input())
        self.assertTrue(view['ready'])
        self.assertTrue(view['requires_alignment_confirmation'])
        self.assertEqual(self.tools.calls, [])
        self.assertEqual(self.tools.sequence, 1)
        expected = deepcopy(view['result']['solution_runtime_joints_deg'])
        token = view['token']
        with self.assertRaises(ValueError):
            self.manual.execute(execute_input(token, alignment_confirmed=False))
        self.manual.execute(execute_input(token))
        with self.assertRaises(ManualBusy):
            self.manual.execute(execute_input(token))
        await self.manual.task
        with self.assertRaises(ValueError):
            self.manual.execute(execute_input(token))
        self.assertEqual(len(self.tools.calls), 1)
        call = self.tools.calls[0]
        self.assertEqual(call['joints'], expected)
        self.assertEqual(call['observation_id'], view['observation_id'])
        self.assertEqual(call['runtime_id'], 'runtime')
        self.assertEqual((call['x'], call['y'], call['theta']), (0, 0, 0))
        self.assertEqual(self.tools.joints['gripper'], 50)
        self.assertEqual(self.manual.last['status'], 'completed')
        self.assertEqual(self.manual.last['observations'], 2)
        self.assertFalse(self.manual.last['physical_success_verified'])
        events = [json.loads(line) for line in (Path(self.folder.name) / view['run_id'] / 'events.jsonl').read_text().splitlines()]
        confirmation = next(e['data'] for e in events if e['event'] == 'manual_operator_confirmation')
        self.assertEqual(confirmation['scope'], 'this_one_move_only')
        self.assertIsNone(self.manual.config)
        again = await self.manual.prepare(prepare_input())
        self.assertTrue(again['requires_alignment_confirmation'])

    async def test_expired_cancelled_and_replaced_tokens_never_send(self):
        first = await self.manual.prepare(prepare_input())
        self.now[0] = 31
        self.assertFalse(self.manual.reserved())
        self.assertEqual(self.manual.last['status'], 'expired')
        with self.assertRaises(ValueError): self.manual.execute(execute_input(first['token']))
        second = await self.manual.prepare(prepare_input())
        third = await self.manual.prepare(prepare_input())
        with self.assertRaises(ValueError): self.manual.execute(execute_input(second['token']))
        self.manual.cancel()
        with self.assertRaises(ValueError): self.manual.execute(execute_input(third['token']))
        self.assertEqual(self.tools.calls, [])

    async def test_preparing_reserves_session_before_network_yields(self):
        started, release = asyncio.Event(), asyncio.Event()
        observe = self.tools.observe
        async def slow(robot_id):
            started.set(); await release.wait()
            return await observe(robot_id)
        self.tools.observe = slow
        task = asyncio.create_task(self.manual.prepare(prepare_input()))
        await started.wait()
        self.assertTrue(self.manual.reserved())
        with self.assertRaises(ManualBusy): await self.manual.prepare(prepare_input())
        with self.assertRaises(ManualBusy): self.manual.cancel()
        release.set(); await task

    async def test_unknown_motion_is_not_replayed(self):
        view = await self.manual.prepare(prepare_input())
        self.tools.uncertain = True
        self.manual.execute(execute_input(view['token']))
        await self.manual.task
        self.assertEqual(self.manual.last['status'], 'uncertain_execution')
        self.assertEqual(len(self.tools.calls), 1)
        self.assertIsNotNone(self.manual.last['pending_action_id'])
        with self.assertRaises(ValueError): self.manual.execute(execute_input(view['token']))

    async def test_missing_arm_confirmation_is_not_success(self):
        original_execute = self.tools.execute
        async def missing_feedback(robot_id, **command):
            content = await original_execute(robot_id, **command)
            response = json.loads(content[0].text)
            response['execution']['arm_reached'] = None
            content[0].text = json.dumps(response)
            return content
        self.tools.execute = missing_feedback
        view = await self.manual.prepare(prepare_input())
        self.manual.execute(execute_input(view['token']))
        await self.manual.task
        self.assertEqual(self.manual.last['status'], 'uncertain_execution')
        self.assertEqual(len(self.tools.calls), 1)

    async def test_invalid_calibration_and_out_of_range_seed_cannot_be_confirmed_away(self):
        self.tools.calibration_id = None
        view = await self.manual.prepare(prepare_input())
        self.assertFalse(view['ready'])
        self.assertIsNone(view['token'])
        self.tools.calibration_id = 'a' * 64
        self.tools.joints['elbow_flex'] = 170.
        view = await self.manual.prepare(prepare_input())
        self.assertFalse(view['ready'])
        self.assertFalse(view['result']['success'])
        self.assertEqual(self.tools.calls, [])

    async def test_verified_config_still_requires_matching_calibration(self):
        self.manual.config = configuration()
        view = await self.manual.prepare(prepare_input())
        self.assertTrue(view['ready'])
        self.assertFalse(view['requires_alignment_confirmation'])
        self.tools.calibration_id = 'b' * 64
        view = await self.manual.prepare(prepare_input())
        self.assertFalse(view['ready'])
        self.assertIn('calibration_id_mismatch', view['state']['reason_codes'])

    async def test_joint_change_cap_even_for_small_xyz(self):
        from harness.kinematics import CartesianController
        solve = CartesianController.solve
        def large(controller, *args):
            result = solve(controller, *args)
            result['solution_runtime_joints_deg']['shoulder_pan'] = 3.01
            return result
        with patch.object(CartesianController, 'solve', large):
            view = await self.manual.prepare(prepare_input())
        self.assertFalse(view['ready'])
        self.assertIsNone(view['token'])
        self.assertTrue(any('3°' in reason for reason in view['reasons']))
        self.assertEqual(self.tools.calls, [])

    def test_strict_input_bounds(self):
        for delta in ([0, 0, 0], [0.,0.,.002], [.01,.01,0.], [True,0.,0.], [float('nan'),0.,0.], [0.,0.,.011]):
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                PrepareInput.model_validate({'robot_id':'mock', 'delta_m':delta})
        for change in ({'execute':1}, {'clearance_confirmed':False}, {'alignment_confirmed':'true'}, {'server':'tcp://elsewhere'}):
            with self.subTest(change=change), self.assertRaises(ValueError): execute_input('a'*32, **change)
        with self.assertRaises(ValueError): prepare_input(joint_mapping={})


class ManualHTTPTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.controller = GuiController(runs_dir=self.temp.name, server='unused')
        self.controller.manual.tools = ArmTools()
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(self.controller)), base_url='http://127.0.0.1:8765')
        self.addAsyncCleanup(self.client.aclose); self.addAsyncCleanup(self.controller.close)

    async def test_state_reads_no_robot_and_reservation_excludes_agent(self):
        for _ in range(3): await self.client.get('/api/state')
        self.assertEqual(self.controller.manual.tools.sequence, 0)
        response = await self.client.post('/api/manual-xyz/prepare', json=prepare_input().model_dump())
        self.assertEqual(response.status_code, 200, response.text)
        self.assertTrue(response.json()['ready'])
        run_id = response.json()['run_id']
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['runs'][0]['status'], 'prepared')
        self.assertTrue(state['runs'][0]['active'])
        self.assertEqual((await self.client.get('/api/runs/' + run_id)).json()['status'], 'prepared')
        response = await self.client.post('/api/runs', json={'instruction':'test','robot_id':'mock','execute':True,'model':None,'limits':{'max_actions':1,'max_steps':2,'max_seconds':30.}})
        self.assertEqual(response.status_code, 409)
        self.assertEqual((await self.client.post('/api/manual-xyz/cancel', json={})).status_code, 200)
        self.assertEqual(self.controller.manual.tools.calls, [])

    async def test_agent_and_demo_and_foreign_origin_cannot_move(self):
        self.controller.task = asyncio.create_task(asyncio.sleep(10))
        response = await self.client.post('/api/manual-xyz/prepare', json=prepare_input().model_dump())
        self.assertEqual(response.status_code, 409)
        self.controller.task.cancel()
        await asyncio.gather(self.controller.task, return_exceptions=True)
        self.controller.task = None
        self.controller.mode = 'demo'
        self.assertEqual((await self.client.post('/api/manual-xyz/prepare', json=prepare_input().model_dump())).status_code, 400)
        self.controller.mode = 'live'
        response = await self.client.post('/api/manual-xyz/prepare', json=prepare_input().model_dump(), headers={'origin':'https://evil.example'})
        self.assertEqual(response.status_code, 403)
        self.assertEqual(self.controller.manual.tools.sequence, 0)

    async def test_served_urdf_meshes_and_local_modules_are_complete(self):
        urdf = await self.client.get('/models/so101_new_calib.urdf')
        self.assertEqual(urdf.status_code, 200)
        self.assertEqual(urdf.content, DEFAULT_URDF.read_bytes())
        for name in {e.attrib['filename'] for e in ET.fromstring(urdf.content).iter('mesh')}:
            response = await self.client.get('/models/' + name)
            self.assertEqual(response.status_code, 200, name)
            data = response.content
            self.assertGreater(len(data), 100)
            self.assertEqual(len(data), 84 + 50 * struct.unpack_from('<I', data, 80)[0], name)
        for name in ('urdf-viewer.js','vendor/three/build/three.module.js','vendor/three/build/three.core.js', 'vendor/urdf-loader/src/URDFLoader.js'):
            response = await self.client.get('/static/' + name)
            self.assertEqual(response.status_code, 200, name)
            self.assertIn('javascript', response.headers['content-type'])
        for path in ('/models/cartesian.example.json','/models/..%2Fgui.py','/static/vendor/..%2F..%2Fconsole.js','/static/vendor/three/LICENSE'):
            self.assertEqual((await self.client.get(path)).status_code, 404, path)
        manifest = json.loads((DEFAULT_URDF.parent/'mesh-sources.json').read_text())
        self.assertTrue(manifest)


if __name__ == '__main__': unittest.main()
