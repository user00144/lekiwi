"""Tests for SLAM-free visual operation and ASPIRE policy.py execution in the streamlined GUI."""
from __future__ import annotations

import asyncio
from pathlib import Path
import tempfile
import unittest

import httpx

from harness.episode import RunRecord, atomic_json
from harness.gui import GuiController, create_app


def _obs_packet(oid: str) -> dict:
    return {
        'status': 'ok',
        'observation': {
            'observation_id': oid,
            'runtime_id': 'rt-visual',
            'ego': {
                'available': True,
                'joints': {
                    'shoulder_pan': 0.0,
                    'shoulder_lift': -12.0,
                    'elbow_flex': 30.0,
                    'wrist_flex': -18.0,
                    'wrist_roll': 2.0,
                    'gripper': 55.0,
                },
            },
            'cameras': {'front': {'available': True}, 'wrist': {'available': True}},
        },
    }


class VisualStageEpisodeFake:
    """Fake stage episode runner writing real stage observations without network or SLAM."""

    def __init__(self, *, hold_until_cancelled: bool = False):
        self.calls: list[dict] = []
        self.hold_until_cancelled = hold_until_cancelled
        self.stage_entered = asyncio.Event()

    async def __call__(self, server, model, robot_id, instruction, *, run_dir, limits, kinematics_config=None):
        self.calls.append({
            'server': server,
            'model': model,
            'robot_id': robot_id,
            'instruction': instruction,
            'run_dir': Path(run_dir),
            'limits': limits,
            'kinematics_config': kinematics_config,
        })
        record = RunRecord(run_dir, robot_id, instruction, limits)
        record.observation(_obs_packet('obs-stage-1'), [('front', '/9j/2Q==', 'image/jpeg')])
        record.event('decision', {
            'tool': 'execute',
            'reason': 'Visual target centered in front camera',
            'plan': ['Approach visually', 'Grasp and lift'],
            'expected': 'Gripper closes around mug',
            'feedback': 'progress',
            'evidence': 'Front + wrist views clear',
        })
        if self.hold_until_cancelled:
            record.event('action_prepared', {
                'action_id': 'act-inflight-1',
                'joints': {'shoulder_lift': -10.0, 'gripper': 20.0},
            })
            # Signal cross-thread safely via file / flag
            self.stage_entered.set()
            try:
                await asyncio.sleep(30)
            except asyncio.CancelledError:
                res = {
                    'status': 'uncertain_execution',
                    'detail': 'Cancelled while stage action was in flight',
                    'pending_action_id': 'act-inflight-1',
                    'physical_success_verified': False,
                }
                atomic_json(Path(run_dir) / 'result.json', res)
                record.event('episode_finished', res)
                raise

        record.event('action_prepared', {
            'action_id': 'act-ok-1',
            'joints': {'shoulder_lift': -10.0, 'gripper': 20.0},
        })
        record.event('action_result', {
            'action_id': 'act-ok-1',
            'packet': {'status': 'completed', 'execution': {'action_id': 'act-ok-1', 'outcome': 'completed', 'command_sent': True}},
        })
        record.observation(_obs_packet('obs-stage-2'), [('front', '/9j/2Q==', 'image/jpeg'), ('wrist', '/9j/2Q==', 'image/jpeg')])
        record.event('motion_feedback', {'before_observation_id': 'obs-stage-1', 'after_observation_id': 'obs-stage-2'})
        res = {
            'status': 'completed',
            'detail': 'Two-observation visual completion verified',
            'steps': 3,
            'action_attempts': 1,
            'observations': 2,
            'reconciliation_attempts': 0,
            'completion_claim': {'evidence': 'Mug lifted clearly in wrist camera'},
            'physical_success_verified': False,
        }
        atomic_json(Path(run_dir) / 'result.json', res)
        record.event('episode_finished', res)
        return res


class GuiPolicyPipelineTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.fake_episode = VisualStageEpisodeFake()
        self.controller = GuiController(
            server='ipc://unused',
            robot_id='kiwi-01',
            runs_dir=self.root / 'runs',
            episode_fn=self.fake_episode,
        )
        self.app = create_app(self.controller)
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=self.app),
            base_url='http://127.0.0.1:8765',
        )

    async def asyncTearDown(self):
        await self.controller.close()
        await self.client.aclose()
        self.temp.cleanup()

    async def test_default_gui_state_is_slam_free_visual_mode_and_serves_streamlined_console(self):
        state = (await self.client.get('/api/state')).json()
        self.assertEqual(state['mode'], 'live')
        self.assertFalse(state['config']['slam_configured'])
        self.assertEqual(state['config']['navigation_mode'], 'visual_only')
        self.assertEqual(state['capabilities']['policy_run'], 1)

        html = (await self.client.get('/')).text
        self.assertIn('SLAM OFF · Visual-Only', html)
        self.assertIn('Policy.py', html)
        self.assertNotIn('xyz-manual-tab', html)

    async def test_live_slam_free_policy_executes_stages_and_serves_stage_camera_images(self):
        policy_src = '''report = {"stage": "start"}
pose = get_pose()
report["slam_active"] = pose["confidence"] > 0.0
if not report["slam_active"]:
    obs = inspect_scene("Locate mug in front camera")
    if obs["ok"]:
        grasp = act("Approach visually and grasp mug")
        report["grasp_ok"] = grasp["ok"]
        print("grasped:", grasp["summary"])
        if grasp["ok"]:
            result = report
            finish("Visual grasp succeeded without SLAM")
result = report
'''
        res = await self.client.post('/api/policy/run', json={
            'instruction': 'Grasp mug visually without SLAM',
            'policy_source': policy_src,
            'robot_id': 'kiwi-01',
            'model': 'test-vlm',
            'backend': 'live',
            'execute': True,
        })
        self.assertEqual(res.status_code, 202, res.text)
        run_id = res.json()['id']
        await self.controller.task
        await asyncio.sleep(0)

        self.assertEqual(len(self.fake_episode.calls), 2)
        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['mode'], 'policy')
        self.assertEqual(view['status'], 'completed')
        self.assertFalse(view['policy']['slam_enabled'])
        self.assertFalse(view['policy']['report']['slam_active'])
        self.assertTrue(view['policy']['report']['grasp_ok'])
        self.assertTrue(view['policy']['outcome']['task_success'])
        self.assertTrue(view['policy']['outcome']['navigation_reached'])
        self.assertIn('>>>', view['policy']['active_highlighted'])
        self.assertIn('grasped:', view['policy']['stdout'])

        # Stage camera images must surface on the main run view and be retrievable via /files/
        self.assertEqual(view['current_observation']['observation_id'], 'obs-stage-2')
        self.assertEqual(view['before_observation']['observation_id'], 'obs-stage-1')
        img_url = view['current_observation']['images'][0]['url']
        img_res = await self.client.get(img_url)
        self.assertEqual(img_res.status_code, 200)
        self.assertEqual(img_res.content, b'\xff\xd8\xff\xd9')

        # Non-observation stage files (e.g. stages/.../result.json) must still be denied
        stage_name = view['policy']['stages'][-1]['name']
        denied = await self.client.get(f'/api/runs/{run_id}/files/stages/{stage_name}/result.json')
        self.assertEqual(denied.status_code, 404)

        # Earlier stage can be selected explicitly via ?stage=<name> with full reasoning & events
        first_stage = view['policy']['stages'][0]['name']
        first_view = (await self.client.get(f'/api/runs/{run_id}?stage={first_stage}')).json()
        self.assertEqual(first_view['policy']['selected_stage'], first_stage)
        self.assertTrue(any(ev['event'] == 'decision' for ev in first_view['events']))
        self.assertEqual((await self.client.get(f'/api/runs/{run_id}?stage=../evil')).status_code, 404)

    async def test_policy_sim_backend_works_even_in_demo_gui(self):
        demo_ctrl = GuiController(runs_dir=self.root / 'demo_runs', demo=True)
        demo_app = create_app(demo_ctrl)
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=demo_app), base_url='http://127.0.0.1:8765') as client:
            live_attempt = await client.post('/api/policy/run', json={
                'instruction': 'Reject live in demo',
                'policy_source': 'result = {}\n',
                'robot_id': 'kiwi-01',
                'backend': 'live',
                'execute': True,
            })
            self.assertEqual(live_attempt.status_code, 400)

            sim_res = await client.post('/api/policy/run', json={
                'instruction': 'Dry-run sim policy',
                'policy_source': (
                    'pose = relocalize()\n'
                    'nav = navigate_to("2F", 4.0, 1.5, 0.0)\n'
                    'grasp = act("Grasp red ceramic mug")\n'
                    'result = {"ok": grasp["ok"]}\n'
                    'if grasp["ok"]:\n'
                    '    finish("Sim fetch ok")\n'
                ),
                'robot_id': 'kiwi-01',
                'backend': 'sim',
                'execute': True,
            })
            self.assertEqual(sim_res.status_code, 202, sim_res.text)
            await demo_ctrl.task
            await asyncio.sleep(0)
            view = (await client.get(f"/api/runs/{sim_res.json()['id']}")).json()
            self.assertEqual(view['status'], 'completed')
            self.assertTrue(view['policy']['outcome']['task_success'])
        await demo_ctrl.close()

    async def test_invalid_policy_syntax_or_forbidden_imports_rejected_upfront(self):
        for bad_code in ('import os\nresult = {}\n', 'for _ in range(2):\n    pass\nresult = {}\n', '   '):
            res = await self.client.post('/api/policy/run', json={
                'instruction': 'Bad policy',
                'policy_source': bad_code,
                'robot_id': 'kiwi-01',
                'backend': 'sim',
                'execute': True,
            })
            self.assertEqual(res.status_code, 400)
        self.assertEqual(list(self.controller.runs_dir.iterdir()), [])

    async def test_stop_cancels_inflight_policy_stage_and_preserves_uncertain_execution(self):
        slow_fake = VisualStageEpisodeFake(hold_until_cancelled=True)
        self.controller.episode_fn = slow_fake
        policy_src = '''report = {"ran_second_block": False}
act("Grasp target")
report["ran_second_block"] = True
result = report
'''
        res = await self.client.post('/api/policy/run', json={
            'instruction': 'Stop mid-stage test',
            'policy_source': policy_src,
            'robot_id': 'kiwi-01',
            'backend': 'live',
            'execute': True,
        })
        self.assertEqual(res.status_code, 202)
        run_id = res.json()['id']

        # Wait until worker thread enters the stage and logs `action_prepared`
        for _ in range(50):
            if len(slow_fake.calls) > 0:
                break
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.05)

        stop_res = await self.client.post(f'/api/runs/{run_id}/stop', json={})
        self.assertEqual(stop_res.status_code, 202)
        await self.controller.task
        await asyncio.sleep(0)

        view = (await self.client.get(f'/api/runs/{run_id}')).json()
        self.assertEqual(view['status'], 'uncertain_execution')
        self.assertFalse(view['policy']['report']['ran_second_block'])


if __name__ == '__main__':
    unittest.main()

