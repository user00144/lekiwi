"""policy.py stage results, stage history, host skills and SLAM-free memory (no model, no hardware)."""
from __future__ import annotations

import asyncio
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from harness.campaign import (
    CampaignPolicyRunner,
    LiveInnerActionExecutor,
    LiveLocalizationProvider,
    LiveNavigationExecutor,
    PoseEstimate,
    SemanticMemoryStore,
    SimObject,
    SimulatedCampaignWorld,
    TopologicalMap,
    compose_stage_instruction,
    episode_stage_result,
    skill_stage_result,
)
from harness.locator import LocateAnswer
from harness.skill_session import drive_chunks, run_skill_session

ROOT = Path(__file__).resolve().parents[2]
CAMERA_CONFIG = ROOT / 'outputs' / 'kinematics' / 'camera-front-kiwi-01.json'


def sim_runner(world, *, skills=True, localization=None, store=None):
    return CampaignPolicyRunner(
        task_language='pick the silver cube',
        robot_id='kiwi-01',
        mode='sim',
        semantic_memory=store or SemanticMemoryStore(),
        topology=TopologicalMap(),
        localization=localization or world,
        navigation_executor=world,
        action_executor=world,
        skill_executor=world if skills else None,
    )


def cube_world(x_m=0.62, y_m=0.07):
    return SimulatedCampaignWorld(
        pose=PoseEstimate(floor='local', x_m=0.0, y_m=0.0, confidence=0.0, relocalized=False),
        objects={'cube': SimObject('cube', 'silver cube', floor='local', x_m=x_m, y_m=y_m)},
    )


def trace(path):
    return [json.loads(line) for line in (path / 'trace' / 'events.jsonl').read_text().splitlines() if line]


APPROACH_AND_PICK = '''report = {}
loc = locate("silver cube")
for drive_idx in range(4):
    if not loc["ok"] or loc["reachable"]:
        break
    drive(forward_m=max(-0.3, min(0.3, loc["xyz_m"][0] - 0.27)), left_m=loc["xyz_m"][1])
    loc = locate("silver cube")
if loc["ok"] and loc["reachable"]:
    grasp = pick(loc["xyz_m"])
    report["grasp_ok"] = grasp["ok"]
    if grasp["ok"]:
        result = report
        finish("picked", xyz_m=loc["xyz_m"])
result = report
'''


class StageResultTests(unittest.TestCase):
    def test_completion_claim_is_success_and_other_statuses_are_not(self):
        with tempfile.TemporaryDirectory() as tmp:
            claim = episode_stage_result(
                {'status': 'completion_claim', 'completion_claim': {'evidence': 'cube between jaws'}, 'steps': 4},
                Path(tmp))
            self.assertTrue(claim['ok'])
            self.assertEqual(claim['status'], 'completed')
            self.assertEqual(claim['summary'], 'cube between jaws')
            for status in ('blocked', 'budget_steps', 'uncertain_execution', 'invalid_decision'):
                result = episode_stage_result({'status': status, 'detail': 'x'}, Path(tmp))
                self.assertFalse(result['ok'])
                self.assertEqual(result['status'], status)

    def test_instruction_without_history_keeps_the_original_format(self):
        self.assertEqual(compose_stage_instruction('act', 'grasp cube', '>>> 1 | act()'),
                         'Active stage (act): grasp cube\n\nPolicy context (`>>>` marks active call):\n>>> 1 | act()')
        text = compose_stage_instruction('act', 'grasp cube', 'ctx', [
            {'tool': 'locate', 'request': 'cube', 'status': 'completed', 'summary': 'cube at [0.27, 0, -0.09]'}])
        self.assertIn('Earlier robot stages', text)
        self.assertIn("locate 'cube' -> completed: cube at [0.27, 0, -0.09]", text)

    def test_skill_stage_result_flattens_payload(self):
        with tempfile.TemporaryDirectory() as tmp:
            res = skill_stage_result({'status': 'completed', 'skill': 'locate', 'action_attempts': 0,
                                      'skill_result': {'found': True, 'label': 'cube', 'xyz_m': [.27, 0, -.09],
                                                       'horizontal_distance_m': .33, 'reachable': True}},
                                     Path(tmp))
            self.assertTrue(res['ok'])
            self.assertEqual(res['xyz_m'], [.27, 0, -.09])
            self.assertIn('reachable=True', res['summary'])
            failed = skill_stage_result({'status': 'skill_failed', 'skill': 'pick', 'detail': 'after lift: empty',
                                         'skill_result': {'error': 'after lift: empty'}}, Path(tmp))
            self.assertFalse(failed['ok'])
            self.assertEqual(failed['summary'], 'after lift: empty')


class LiveExecutorTests(unittest.TestCase):
    """The CLI live executors must accept EpisodeRunner's `completion_claim` as stage success."""

    def fake_episode(self, status='completion_claim'):
        calls = []

        async def fake(server, model, robot_id, instruction, *, run_dir, limits, kinematics_config=None):
            calls.append({'instruction': instruction, 'run_dir': str(run_dir), 'model': model})
            Path(run_dir).mkdir(parents=True, exist_ok=True)
            return {'status': status, 'steps': 3, 'detail': '',
                    'completion_claim': {'evidence': 'verified on two fresh observations'}}
        return fake, calls

    def test_cli_live_policy_finishes_after_completion_claims(self):
        fake, calls = self.fake_episode()
        plan_calls = []

        async def plan_act(server, model, robot_id, instruction, *, run_dir, limits, kinematics_config=None):
            plan_calls.append(instruction)
            Path(run_dir).mkdir(parents=True, exist_ok=True)
            return {'status': 'completion_claim', 'completion_claim': {'evidence': 'cube lifted'}}

        with tempfile.TemporaryDirectory() as tmp, mock.patch('harness.episode_runtime.run_episode', fake):
            runner = CampaignPolicyRunner(
                task_language='pick cube', robot_id='kiwi-01', mode='live',
                semantic_memory=SemanticMemoryStore(), topology=TopologicalMap(),
                localization=LiveLocalizationProvider(None),
                navigation_executor=LiveNavigationExecutor(server='tcp://x', model=None,
                                                           localization=LiveLocalizationProvider(None)),
                action_executor=LiveInnerActionExecutor(server='tcp://x', model=None, act_episode_fn=plan_act),
            )
            outcome = runner.run_policy_source(
                'obs = inspect_scene("is the cube visible?")\n'
                'grasp = act("grasp the cube")\n'
                'if obs["ok"] and grasp["ok"]:\n'
                '    finish("grasped")\n', Path(tmp) / 'attempt')
        self.assertEqual(outcome.termination_reason, 'completed')
        self.assertTrue(outcome.task_success)
        self.assertEqual(len(calls), 1, 'inspect_scene keeps the step loop')
        self.assertEqual(len(plan_calls), 1, 'act() uses the configured plan-act episode')
        self.assertIn('Earlier robot stages', plan_calls[0])
        self.assertIn('verified on two fresh observations', plan_calls[0])

    def test_cli_live_navigation_counts_completion_claim_legs(self):
        fake, calls = self.fake_episode()
        with tempfile.TemporaryDirectory() as tmp, mock.patch('harness.episode_runtime.run_episode', fake):
            pose = Path(tmp) / 'pose.json'
            pose.write_text(json.dumps({'floor': '1F', 'x_m': 0, 'y_m': 0, 'confidence': .9, 'relocalized': True}))
            topo = TopologicalMap()
            topo.add_node('a', floor='1F', x_m=0, y_m=0)
            topo.add_node('b', floor='1F', x_m=2, y_m=0)
            topo.add_edge('a', 'b', via='drive')
            localization = LiveLocalizationProvider(pose)
            nav = LiveNavigationExecutor(server='tcp://x', model=None, localization=localization)
            route = topo.plan_route('1F', 0, 0, '1F', 2, 0)
            result = nav.execute_route('kiwi-01', route, Path(tmp) / 'nav')
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['legs_completed'], len(route.legs))

    def test_failed_inner_stage_is_not_success(self):
        fake, _ = self.fake_episode('budget_steps')
        with tempfile.TemporaryDirectory() as tmp, mock.patch('harness.episode_runtime.run_episode', fake):
            executor = LiveInnerActionExecutor(server='tcp://x', model=None)
            from harness.episode import EpisodeLimits
            res = executor.run_subgoal(robot_id='kiwi-01', kind='act', subgoal='grasp', highlighted_policy='',
                                       limits=EpisodeLimits(), stage_dir=Path(tmp) / 's')
        self.assertFalse(res['ok'])
        self.assertEqual(res['status'], 'budget_steps')


class MemoryTests(unittest.TestCase):
    def test_in_memory_stores_do_not_share_records(self):
        first = SemanticMemoryStore()
        first.upsert('red mug', floor='1F', x_m=1, y_m=2)
        self.assertEqual(SemanticMemoryStore().count(), 0)
        self.assertEqual(first.count(), 1)

    def test_slam_free_save_memory_is_marked_unlocalized(self):
        store = SemanticMemoryStore()
        world = cube_world()
        runner = sim_runner(world, localization=LiveLocalizationProvider(None), store=store)
        with tempfile.TemporaryDirectory() as tmp:
            runner.run_policy_source(
                'a = save_memory("silver cube near sofa")\n'
                'b = save_memory("silver cube at measured spot", x_m=1.0, y_m=0.5)\n'
                'result = {"a": a["record"]["localized"], "b": b["record"]["localized"]}\n', Path(tmp) / 'm')
            saved = json.loads((Path(tmp) / 'm' / 'source' / 'policy-result.json').read_text())['result']
        self.assertEqual(saved, {'a': False, 'b': True})
        hits = {hit.text: hit.localized for hit in store.search('silver cube', top_k=5)}
        self.assertEqual(hits, {'silver cube near sofa': False, 'silver cube at measured spot': True})


class HostSkillPolicyTests(unittest.TestCase):
    def test_locate_drive_pick_policy_succeeds_in_sim(self):
        world = cube_world()
        with tempfile.TemporaryDirectory() as tmp:
            outcome = sim_runner(world).run_policy_source(APPROACH_AND_PICK, Path(tmp) / 'a')
            tools = [row['tool'] for row in trace(Path(tmp) / 'a')]
            stages = sorted(path.name for path in (Path(tmp) / 'a' / 'stages').iterdir())
        self.assertTrue(outcome.task_success, outcome)
        self.assertTrue(outcome.navigation_reached)
        self.assertEqual(tools, ['locate', 'drive', 'locate', 'drive', 'locate', 'pick', 'finish'])
        self.assertTrue(stages[0].endswith('_locate') and stages[-1].endswith('_pick'))
        self.assertTrue(world.objects['cube'].held)

    def test_pick_on_air_fails_and_place_releases_held_object(self):
        world = cube_world(x_m=0.27, y_m=0.0)
        with tempfile.TemporaryDirectory() as tmp:
            outcome = sim_runner(world).run_policy_source(
                'miss = pick([0.27, 0.06, -0.09])\n'
                'hit = pick([0.27, 0.0, -0.09])\n'
                'drop = place([0.25, 0.0, -0.09])\n'
                'result = {"miss": miss["ok"], "hit": hit["ok"], "drop": drop["ok"]}\n', Path(tmp) / 'p')
            saved = json.loads((Path(tmp) / 'p' / 'source' / 'policy-result.json').read_text())['result']
        self.assertTrue(outcome.program_ok)
        self.assertEqual(saved, {'miss': False, 'hit': True, 'drop': True})
        self.assertFalse(world.objects['cube'].held)

    def test_invalid_arguments_and_missing_skills_return_errors_without_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = cube_world()
            sim_runner(world).run_policy_source(
                'a = pick([1, 2])\n'
                'b = drive(forward_m=0.8)\n'
                'c = drive()\n'
                'd = locate("")\n'
                'e = place([0.9, 0.5, 0.0])\n'
                'result = [a["status"], b["status"], c["status"], d["status"], e["status"]]\n', Path(tmp) / 'i')
            saved = json.loads((Path(tmp) / 'i' / 'source' / 'policy-result.json').read_text())['result']
            self.assertEqual(saved, ['invalid_arguments'] * 5)
            self.assertFalse((Path(tmp) / 'i' / 'stages').exists() and any((Path(tmp) / 'i' / 'stages').iterdir()))

            outcome = sim_runner(cube_world(), skills=False).run_policy_source(
                'loc = locate("silver cube")\nresult = {"status": loc["status"]}\n', Path(tmp) / 'n')
            saved = json.loads((Path(tmp) / 'n' / 'source' / 'policy-result.json').read_text())['result']
        self.assertEqual(saved, {'status': 'skills_unavailable'})
        self.assertTrue(outcome.program_ok)

    def test_uncertain_skill_execution_halts_the_policy(self):
        world = cube_world(x_m=0.27, y_m=0.0)
        world.force_uncertain_on_tool = 'pick'
        with tempfile.TemporaryDirectory() as tmp:
            outcome = sim_runner(world).run_policy_source(
                'grasp = pick([0.27, 0.0, -0.09])\nprint("must not run")\nfinish("no")\n', Path(tmp) / 'u')
            stdout = (Path(tmp) / 'u' / 'logs' / 'policy.stdout').read_text()
        self.assertEqual(outcome.termination_reason, 'uncertain_execution')
        self.assertFalse(outcome.task_success)
        self.assertEqual(stdout, '')


class SkillHelperTests(unittest.TestCase):
    def test_drive_chunks_respect_runtime_limit_and_total_motion(self):
        chunks = drive_chunks(0.25, 0.0, 0.0)
        self.assertEqual(len(chunks), 3)
        self.assertTrue(all(1 <= chunk['duration_ms'] <= 1000 for chunk in chunks))
        self.assertAlmostEqual(sum(c['x'] * c['duration_ms'] / 1000 for c in chunks), 0.25, places=3)
        turn = drive_chunks(0.0, 0.0, -45.0)
        self.assertEqual(len(turn), 2)
        self.assertAlmostEqual(sum(c['theta'] * c['duration_ms'] / 1000 for c in turn), -45.0, places=1)
        both = drive_chunks(0.1, 0.1, 30.0)
        self.assertEqual([c['theta'] == 0 for c in both], [True, True, False])
        self.assertTrue(all(math.hypot(c['x'], c['y']) <= 0.4 for c in both))
        self.assertEqual(drive_chunks(0.0, 0.0, 0.0), [])

    def test_locate_answer_requires_a_box_only_when_found(self):
        with self.assertRaises(ValueError):
            LocateAnswer.model_validate({'found': True, 'bbox_px': None, 'confidence': 'high', 'note': ''})
        answer = LocateAnswer.model_validate({'found': False, 'bbox_px': [1., 2., 3., 4.], 'confidence': 'low',
                                              'note': 'not visible'})
        self.assertIsNone(answer.bbox_px)


class SkillSessionIntegrationTest(unittest.IsolatedAsyncioTestCase):
    """drive / locate / home_arm through MCP -> relay -> mock runtime, with a fake locator box."""

    async def test_skill_session_through_existing_guard(self):
        from harness.client import ServerClient
        from harness.kinematics import ARM_JOINTS, CartesianController

        with tempfile.TemporaryDirectory() as folder:
            robots, api = f'ipc://{folder}/robots', f'ipc://{folder}/api'
            env = {key: value for key, value in os.environ.items() if key != 'VIRTUAL_ENV'}
            processes = []
            try:
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'server'), 'server', '--bind', robots, '--api', api,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                processes.append(subprocess.Popen([
                    'uv', 'run', '--project', str(ROOT / 'lekiwi_runtime'), 'lekiwi-runtime',
                    '--mock', '--robot-id', 'skill-mock', '--server', robots,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                client = ServerClient(api, timeout=5)
                for _ in range(100):
                    initial = await client.request('skill-mock', 'observe')
                    if initial.get('status') == 'ok':
                        break
                    await asyncio.sleep(.02)
                self.assertEqual(initial['status'], 'ok', initial)
                ego = initial['observation']['ego']
                cartesian_config = {'version': 1, 'hardware_alignment_verified': True,
                                    'calibration_id': ego['calibration_id'],
                                    'model_hash': CartesianController().geometry.model_hash,
                                    'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}}
                cartesian = CartesianController(cartesian_config)
                home_fk = cartesian.geometry.forward({name: ego['joints'][name] for name in ARM_JOINTS})
                skills_config = {
                    'version': 1, 'calibration_id': ego['calibration_id'],
                    'home': {'joints_deg': {name: ego['joints'][name] for name in ARM_JOINTS},
                             'xyz_m': home_fk['xyz_m'], 'approach_axis': [row[2] for row in home_fk['transform'][:3]]},
                    'grasp': {'approach_axis': [-.117, .008, -.993], 'reference_xyz_m': [.25, 0, -.09],
                              'floor_grasp_z_m': -.09, 'hover_m': .04},
                    'gripper': {'open_percent': 54.6, 'close_percent': 26., 'duration_ms': 1500,
                                'held_margin_percent': 1.},
                    'motion': {'speed_margin': 1.4, 'min_duration_ms': 2000, 'max_duration_ms': 20000},
                }
                boxes = []

                async def fake_locate(label, image_b64, mime, size):
                    boxes.append((label, mime, size))
                    return LocateAnswer.model_validate({'found': True, 'bbox_px': [300., 320., 340., 360.],
                                                        'confidence': 'high', 'note': 'synthetic box'})

                common = dict(kinematics_config=cartesian_config, skills_config=skills_config,
                              camera_config=CAMERA_CONFIG, locate_fn=fake_locate)
                drive_dir = Path(folder) / '0001_L001_drive'
                raw = await run_skill_session(api, None, 'skill-mock', 'drive',
                                              {'forward_m': 0.15, 'left_m': 0.0, 'turn_deg': 0.0},
                                              run_dir=drive_dir, **common)
                drove = skill_stage_result(raw, drive_dir)
                self.assertTrue(drove['ok'], drove)
                self.assertEqual(drove['base_commands'], 2)
                self.assertEqual(drove['action_attempts'], 2)
                self.assertFalse(drove['arm_moved'])

                locate_dir = Path(folder) / '0002_L002_locate'
                raw = await run_skill_session(api, None, 'skill-mock', 'locate', {'label': 'silver cube'},
                                              run_dir=locate_dir, **common)
                located = skill_stage_result(raw, locate_dir)
                self.assertTrue(located['ok'], located)
                self.assertTrue(located['found'])
                self.assertEqual(len(located['xyz_m']), 3)
                self.assertIsInstance(located['reachable'], bool)
                self.assertEqual(located['action_attempts'], 0)
                self.assertEqual(boxes[0][0], 'silver cube')
                events = [json.loads(line)['event'] for line in (locate_dir / 'events.jsonl').read_text().splitlines()]
                self.assertIn('locate_answer', events)

                home_dir = Path(folder) / '0003_L003_home_arm'
                raw = await run_skill_session(api, None, 'skill-mock', 'home_arm', {}, run_dir=home_dir, **common)
                homed = skill_stage_result(raw, home_dir)
                self.assertTrue(homed['ok'], homed)
                self.assertTrue((home_dir / 'result.json').is_file())
            finally:
                for process in processes:
                    process.terminate()
                for process in processes:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()


if __name__ == '__main__':
    unittest.main()
