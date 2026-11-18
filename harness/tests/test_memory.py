"""Verified lesson memory and host-measured body corrections (no model, no hardware)."""
from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from harness.memory import (
    EmbodimentMemory,
    LessonMemory,
    MemoryTracker,
    failure_signature,
    import_experience,
    select_lessons,
)

ROOT = Path(__file__).resolve().parents[2]
CAMERA_CONFIG = ROOT / 'outputs' / 'kinematics' / 'camera-front-kiwi-01.json'


class LessonMemoryTests(unittest.TestCase):
    def test_hypothesis_is_verified_only_by_later_successful_episodes(self):
        with tempfile.TemporaryDirectory() as tmp:
            memory = LessonMemory(tmp, 'kiwi-01')
            lesson = memory.propose('pick at x>0.29 slipped twice; approach to x<=0.28 first',
                                    skill='pick', signature='object_slipped', run='run-a')
            self.assertEqual(memory.get(lesson)['status'], 'hypothesis')
            self.assertEqual(memory.credit({lesson}, set(), 'completion_claim', 'run-a'), {},
                             'the episode that wrote a lesson cannot confirm it')
            self.assertEqual(memory.credit({lesson}, set(), 'completion_claim', 'run-b'), {lesson: 'support'})
            self.assertEqual(memory.get(lesson)['status'], 'hypothesis')
            memory.credit({lesson}, set(), 'completion_claim', 'run-c')
            self.assertEqual(memory.get(lesson)['status'], 'verified')
            self.assertEqual(memory.credit({lesson}, set(), 'uncertain_execution', 'run-d'), {},
                             'infrastructure stops say nothing about a lesson')
            memory.save()
            reloaded = LessonMemory(tmp, 'kiwi-01')
            self.assertEqual(reloaded.recall()[0]['id'], lesson)
            self.assertEqual(reloaded.recall()[0]['status'], 'verified')

    def test_failed_application_and_failed_episode_contradict_until_retired(self):
        with tempfile.TemporaryDirectory() as tmp:
            memory = LessonMemory(tmp, 'kiwi-01')
            lesson = memory.propose('drive 0.1 m/s for 2 s when the cube is beyond 0.4 m', skill='drive',
                                    signature='ok', run='run-a')
            self.assertEqual(memory.credit({lesson}, {lesson}, 'completion_claim', 'run-b'), {lesson: 'contradict'},
                             'an action that applied the lesson failed')
            self.assertEqual(memory.credit({lesson}, set(), 'blocked', 'run-c'), {lesson: 'contradict'})
            self.assertEqual(memory.get(lesson)['status'], 'retired')
            self.assertNotIn(lesson, [item['id'] for item in memory.recall()])

    def test_similar_lessons_merge_and_count_reproposals(self):
        with tempfile.TemporaryDirectory() as tmp:
            memory = LessonMemory(tmp, 'kiwi-01')
            first = memory.propose('같은 좌표로 반복하지 말고 팔을 치운 뒤 위치를 재측정한다.', skill='pick',
                                   signature='object_slipped', run='run-a')
            again = memory.propose('같은 좌표로 반복하지 말고 팔을 치운 뒤 큐브 위치를 재측정한다.', skill='pick',
                                   signature='object_slipped', run='run-b')
            other = memory.propose('같은 좌표로 반복하지 말고 팔을 치운 뒤 위치를 재측정한다.', skill='drive',
                                   signature='ok', run='run-b')
            self.assertEqual(first, again)
            self.assertNotEqual(first, other)
            self.assertEqual(memory.get(first)['proposals'], 2)

    def test_episode_lesson_selection_prefers_conclusions_and_failures(self):
        chosen = select_lessons([
            {'text': 'routine progress note number one', 'skill': 'drive', 'signature': 'ok'},
            {'text': 'pick slipped at x=0.293; approach closer first', 'skill': 'pick', 'signature': 'object_slipped'},
            {'text': 'staged lift kept the cube; use it after slips', 'skill': 'attempt', 'signature': 'done', 'final': True},
            {'text': 'routine progress note number one!', 'skill': 'drive', 'signature': 'ok'},
            {'text': '', 'skill': 'drive', 'signature': 'ok'},
        ], limit=3)
        self.assertEqual([item['skill'] for item in chosen], ['attempt', 'pick', 'drive'])

    def test_failure_signatures(self):
        self.assertEqual(failure_signature('pick', 'ok'), 'ok')
        self.assertEqual(failure_signature('pick', 'failed', 'after lift: the object is no longer between the jaws'),
                         'object_slipped')
        self.assertEqual(failure_signature('pick', 'failed', 'x', {'ik_reason_codes': ['urdf_seed_out_of_range']}),
                         'ik:urdf_seed_out_of_range')


class EmbodimentTests(unittest.TestCase):
    def test_drive_gain_from_clean_locate_drive_locate_chains(self):
        with tempfile.TemporaryDirectory() as tmp:
            body = EmbodimentMemory(tmp, 'kiwi-01')
            tracker = MemoryTracker(body, run='run-a')
            tracker.after_locate('cube', [0.40, 0.02, -0.09], 0.004)
            tracker.after_drive(0.1, 0.0, 0.0, 1000)
            measured = tracker.after_locate('cube', [0.305, 0.021, -0.09], 0.003)['drive_measurement']
            self.assertTrue(measured['accepted'], measured)
            self.assertAlmostEqual(measured['gain'], 0.95, places=2)

            tracker.after_drive(0.1, 0.0, 0.0, 1000)
            tracker.after_arm_skill('move_to', ok=True)  # may have touched the object: no sample
            self.assertEqual(tracker.after_locate('cube', [0.20, 0.02, -0.09], 0.003), {})
            tracker.after_drive(0.0, 0.0, 30.0, 1000)  # rotation: no sample
            self.assertEqual(tracker.after_locate('cube', [0.25, 0.05, -0.09], 0.003), {})
            tracker.after_drive(0.05, 0.0, 0.0, 1000)
            far = tracker.after_locate('cube', [0.62, 0.0, -0.09], 0.05)['drive_measurement']
            self.assertFalse(far['accepted'])

            for before, after in (([0.45, 0, -.09], [0.35, 0, -.09]), ([0.33, 0, -.09], [0.28, 0, -.09])):
                tracker.after_locate('cube', before, 0.003)
                tracker.after_drive(0.1 if before[0] == 0.45 else 0.05, 0.0, 0.0, 1000)
                tracker.after_locate('cube', after, 0.003)
            gain = body.drive_gain()
            self.assertTrue(gain['ready'])
            self.assertEqual(gain['samples'], 3)
            body.save()
            self.assertEqual(EmbodimentMemory(tmp, 'kiwi-01').drive_gain()['samples'], 3)

    def test_locate_offset_from_place_then_locate(self):
        with tempfile.TemporaryDirectory() as tmp:
            body = EmbodimentMemory(tmp, 'kiwi-01')
            for index in range(3):
                tracker = MemoryTracker(body, run=f'run-{index}', camera='cam-a')
                located = {'cube': {'xyz_m': [0.27, 0.0, -0.09]}}
                tracker.after_arm_skill('pick', ok=True, target_xyz=[0.27, 0.005, -0.09], located=located)
                tracker.after_arm_skill('place', ok=True, release_xyz=[0.26, 0.01, -0.09])
                sample = tracker.after_locate('cube', [0.252, 0.013, -0.09], 0.003)['locate_measurement']
                self.assertTrue(sample['accepted'])
            offset = body.locate_offset('cam-a')
            self.assertTrue(offset['ready'])
            self.assertEqual(offset['offset_m'], [0.008, -0.003])
            self.assertFalse(body.locate_offset('cam-b')['ready'], 'offsets belong to one camera calibration')

    def test_import_experience_seeds_hypotheses_and_drive_samples(self):
        rows = [
            {'kind': 'action', 'tool': 'locate', 'status': 'ok', 'args': {'label': 'cube'},
             'result': {'xyz_m': [0.40, 0.0, -0.09]}, 'lesson': ''},
            {'kind': 'action', 'tool': 'drive', 'status': 'ok',
             'args': {'drive': {'x': 0.1, 'y': 0.0, 'theta': 0.0, 'duration_ms': 1000}}, 'lesson': 'start lesson text'},
            {'kind': 'action', 'tool': 'locate', 'status': 'ok', 'args': {'label': 'cube'},
             'result': {'xyz_m': [0.30, 0.0, -0.09]}, 'lesson': 'the drive moved the cube 0.10 m closer'},
            {'kind': 'action', 'tool': 'pick', 'status': 'failed', 'error': 'after lift: the object is no longer between the jaws',
             'args': {'xyz_m': [0.30, 0.0, -0.09]}, 'lesson': ''},
            {'kind': 'action', 'tool': 'home', 'status': 'ok', 'args': {},
             'lesson': 'after a slip at x=0.30 approach closer before picking'},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'experience.jsonl'
            path.write_text('\n'.join(json.dumps({**row, 'robot_id': 'kiwi-01', 'run': 'runs/a'}) for row in rows))
            counts = import_experience(path, Path(tmp) / 'memory', 'kiwi-01')
            lessons = LessonMemory(Path(tmp) / 'memory', 'kiwi-01')
        self.assertEqual(counts['drive_samples'], 1)
        by_text = {lesson['text']: (lesson['skill'], lesson['signature']) for lesson in lessons.lessons}
        self.assertEqual(by_text['after a slip at x=0.30 approach closer before picking'], ('pick', 'object_slipped'))
        self.assertEqual(by_text['the drive moved the cube 0.10 m closer'], ('drive', 'ok'))
        self.assertTrue(all(lesson['status'] == 'hypothesis' for lesson in lessons.lessons))


class PlanActMemoryIntegrationTest(unittest.IsolatedAsyncioTestCase):
    """plan -> locate -> drive -> locate -> done -> verify through MCP -> relay -> mock runtime."""

    async def test_episode_credits_applied_lesson_and_measures_drive(self):
        from mcp import ClientSession, StdioServerParameters
        from mcp.client.stdio import stdio_client

        from harness.client import ServerClient
        from harness.episode import EpisodeLimits, EpisodeRunner, MCPRobotTools
        from harness.kinematics import ARM_JOINTS, CartesianController
        from harness.perception import CameraModel
        from harness.plan_loop import PlanActRunner
        from harness.skills import Skills

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
                    '--mock', '--robot-id', 'memory-mock', '--server', robots,
                ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                client = ServerClient(api, timeout=5)
                for _ in range(100):
                    initial = await client.request('memory-mock', 'observe')
                    if initial.get('status') == 'ok':
                        break
                    await asyncio.sleep(.02)
                self.assertEqual(initial['status'], 'ok', initial)
                ego = initial['observation']['ego']
                cartesian = CartesianController({
                    'version': 1, 'hardware_alignment_verified': True, 'calibration_id': ego['calibration_id'],
                    'model_hash': CartesianController().geometry.model_hash,
                    'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}})
                home_fk = cartesian.geometry.forward({name: ego['joints'][name] for name in ARM_JOINTS})
                camera = CameraModel(CAMERA_CONFIG)

                def box_for(x_m):
                    u, v = camera.project([x_m, 0.0, camera.grasp_plane_z])
                    return [u - 15, v, u + 15, v + 30]  # feature = bbox top centre

                memory_dir = Path(folder) / 'memory'
                lessons = LessonMemory(memory_dir, 'memory-mock')
                old = lessons.propose('locate again after every drive before picking', skill='drive',
                                      signature='ok', run='earlier-run')
                lessons.save()
                lessons = LessonMemory(memory_dir, 'memory-mock')
                body = EmbodimentMemory(memory_dir, 'memory-mock')
                seen = []

                def base(context, tool, **fields):
                    values = {'label': None, 'bbox_px': None, 'xyz_m': None, 'approach': None,
                              'gripper_percent': None, 'drive': None, 'lesson': '', 'lessons_applied': []}
                    values.update(fields)
                    return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                            'tool': tool, 'reason': f'scripted {tool}', 'expected': '', **values}

                class Planner:
                    async def plan(self, context, images):
                        seen.append(context['memory']['lessons'])
                        return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                                'tool': 'plan', 'reason': 'scripted plan', 'expected_outcome': 'cube located close',
                                'lessons_used': 'relocate after driving', 'lessons_applied': [old, 'L999'],
                                'plan': [{'step': 'locate, drive, locate', 'skill': 'locate', 'success_check': 'x<0.35'}]}

                    async def act(self, context, images):
                        step = len([item for item in context['history']])
                        if step == 0:
                            return base(context, 'locate', label='cube', bbox_px=box_for(0.40))
                        if step == 1:
                            return base(context, 'drive', drive={'x': 0.1, 'y': 0.0, 'theta': 0.0, 'duration_ms': 1000},
                                        lessons_applied=[old])
                        if step == 2:
                            return base(context, 'locate', label='cube', bbox_px=box_for(0.305))
                        return base(context, 'done', lesson='a 0.1 m/s 1 s drive closes about 0.095 m of distance')

                    async def verify(self, context, images):
                        return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                                'tool': 'confirm', 'reason': 'scripted confirm', 'evidence': 'scripted', 'lesson': ''}

                skills_config = {
                    'version': 1, 'calibration_id': ego['calibration_id'],
                    'home': {'joints_deg': {name: ego['joints'][name] for name in ARM_JOINTS},
                             'xyz_m': home_fk['xyz_m'], 'approach_axis': [row[2] for row in home_fk['transform'][:3]]},
                    'grasp': {'approach_axis': [-.117, .008, -.993], 'reference_xyz_m': [.25, 0, -.09],
                              'floor_grasp_z_m': -.09, 'hover_m': .04},
                    'gripper': {'open_percent': 54.6, 'close_percent': 26., 'duration_ms': 1500, 'held_margin_percent': 1.},
                    'motion': {'speed_margin': 1.4, 'min_duration_ms': 2000, 'max_duration_ms': 20000},
                }
                params = StdioServerParameters(command=sys.executable, args=['-m', 'harness', '--server', api])
                async with stdio_client(params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        runner = EpisodeRunner(None, MCPRobotTools(session), 'memory-mock', 'find the cube',
                                               Path(folder) / 'episode',
                                               EpisodeLimits(max_actions=6, max_steps=10, max_seconds=60),
                                               cartesian=cartesian)
                        skills = Skills(runner, skills_config, reason='memory integration test')
                        result = await PlanActRunner(runner, Planner(), skills, camera, None, max_replans=0,
                                                     lessons=lessons, embodiment=body).run()

                self.assertEqual(result['status'], 'completion_claim', result)
                self.assertEqual(seen[0][0]['id'], old)
                self.assertEqual(result['memory']['credited'], {old: 'support'}, 'unknown id L999 is ignored')
                saved_body = EmbodimentMemory(memory_dir, 'memory-mock').drive_gain()
                self.assertEqual(saved_body['samples'], 1)
                self.assertAlmostEqual(saved_body['gain'], 0.95, delta=0.03)
                stored = LessonMemory(memory_dir, 'memory-mock')
                self.assertEqual(stored.get(old)['support'], 1)
                self.assertIn('a 0.1 m/s 1 s drive closes about 0.095 m of distance', [l['text'] for l in stored.lessons])
                events = [json.loads(line) for line in (Path(folder) / 'episode' / 'events.jsonl').read_text().splitlines()]
                locate_results = [e['data'] for e in events if e['event'] == 'skill_result' and e['data']['tool'] == 'locate']
                self.assertTrue(locate_results[1]['result']['drive_measurement']['accepted'])
                decisions = [e['data'] for e in events if e['event'] == 'decision']
                self.assertEqual([item['id'] for item in decisions[0]['lessons_applied']], [old])
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
