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
    FailureLedger,
    MemoryTracker,
    SkillLibrary,
    failure_signature,
    import_experience,
)

ROOT = Path(__file__).resolve().parents[2]
CAMERA_CONFIG = ROOT / 'outputs' / 'kinematics' / 'camera-front-kiwi-01.json'


def act(tool, status='ok', error=None, **result):
    args = {'locate': {'label': 'cube', 'bbox_px': [1, 2, 3, 4]}, 'drive': {'drive': {'x': 0.1, 'y': 0.0, 'theta': 0.0, 'duration_ms': 1000}},
            'pick': {'xyz_m': [0.29, 0.0, -0.09]}, 'home': {}, 'move_to': {'xyz_m': [0.28, 0.0, -0.05], 'approach': 'grasp'},
            'gripper': {'gripper_percent': 54.6}}.get(tool, {})
    return {'tool': tool, 'args': args, 'status': status, 'error': error, 'result': result}


VERIFIED = {'verify_confirmed': True, 'executed_skills': True, 'still_holding': True, 'passed': True}


class SkillLibraryTests(unittest.TestCase):
    def test_only_verified_episodes_are_admitted(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = SkillLibrary(tmp, 'kiwi-01')
            actions = [act('locate', xyz_m=[0.4, 0, -0.09], reachable=False), act('drive'), act('pick')]
            skipped = library.admit_episode(task='pick up the cube', run='run-a', actions=actions,
                                            verification={'verify_confirmed': False, 'passed': False})
            self.assertIn('skipped', skipped)
            self.assertEqual(library.entries, [])
            added = library.admit_episode(task='pick up the cube', run='run-b', actions=actions, verification=VERIFIED,
                                          strategy='approach to x<=0.28 first', labels=['cube'])
            self.assertEqual(added, {'added': ['S001'], 'confirmed': []})
            entry = library.entries[0]
            self.assertEqual((entry['kind'], entry['topic'], entry['shape']), ('workflow', 'episodic-workflows.md',
                                                                                'locate>drive>pick'))
            self.assertIn("locate('cube') -> xyz [0.4, 0, -0.09], reachable=False", entry['snippet'])
            library.save()
            text = (Path(tmp) / 'episodic-workflows.md').read_text()
            for field in ('**Trigger:**', '**Snippet (executed skill calls, host-recorded):**', '**Provenance:**',
                          '**Strategy (verifier note):** approach to x<=0.28 first'):
                self.assertIn(field, text)
            self.assertEqual(SkillLibrary(tmp, 'kiwi-01').entries[0]['id'], 'S001')

    def test_same_workflow_is_confirmed_not_duplicated_and_reuse_is_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = SkillLibrary(tmp, 'kiwi-01')
            actions = [act('locate'), act('drive'), act('locate'), act('pick')]
            library.admit_episode(task='pick up the silver cube', run='run-a', actions=actions, verification=VERIFIED)
            again = library.admit_episode(task='pick up the silver cube please', run='run-b', actions=actions,
                                          verification=VERIFIED)
            self.assertEqual(again, {'added': [], 'confirmed': ['S001']})
            self.assertEqual(library.record_use({'S001'}, run='run-c', status='blocked', verified=False),
                             {'S001': 'failed_reuse'})
            self.assertEqual(library.record_use({'S001'}, run='run-d', status='uncertain_execution', verified=False), {},
                             'infrastructure stops record nothing')
            self.assertEqual(library.record_use({'S001'}, run='run-a', status='completion_claim', verified=True), {},
                             'the source run cannot confirm its own entry')
            view = library.recall('pick up the silver cube')[0]
            self.assertEqual((view['confirmed'], view['failed_reuse'], view['stale']), (1, 1, False))

    def test_failure_followed_by_success_becomes_a_repair_entry(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = SkillLibrary(tmp, 'kiwi-01')
            actions = [act('locate'), act('pick', 'failed', 'after lift: the object is no longer between the jaws'),
                       act('home'), act('locate'), act('drive'), act('locate'), act('pick')]
            result = library.admit_episode(task='pick up the cube', run='run-a', actions=actions, verification=VERIFIED)
            kinds = {entry['kind']: entry for entry in library.entries}
            self.assertEqual(set(kinds), {'workflow', 'repair'})
            repair = kinds['repair']
            self.assertEqual((repair['topic'], repair['failed_tool'], repair['signature'], repair['shape']),
                             ('mobile-grasp.md', 'pick', 'object_slipped', 'home>locate>drive>locate>pick'))
            self.assertIn('failed', repair['contrast'])
            self.assertEqual(len(result['added']), 2)
            self.assertNotIn('failed', ' '.join(kinds['workflow']['snippet'][:1]))

    def test_failure_never_resolved_is_not_a_repair_and_frozen_library_is_read_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = SkillLibrary(tmp, 'kiwi-01')
            library.admit_episode(task='locate the cube', run='run-a', actions=[act('locate'), act('pick', 'failed', 'x')],
                                  verification=VERIFIED)
            self.assertEqual([entry['kind'] for entry in library.entries], ['workflow'])
            frozen = SkillLibrary(tmp, 'kiwi-01', frozen=True)
            self.assertIn('skipped', frozen.admit_episode(task='t', run='run-b', actions=[act('locate')],
                                                          verification=VERIFIED))
            self.assertEqual(frozen.record_use({'S001'}, run='run-c', status='blocked', verified=False), {})

    def test_failure_signatures(self):
        self.assertEqual(failure_signature('pick', 'ok'), 'ok')
        self.assertEqual(failure_signature('pick', 'failed', 'after lift: the object is no longer between the jaws'),
                         'object_slipped')
        self.assertEqual(failure_signature('pick', 'failed', 'x', {'ik_reason_codes': ['urdf_seed_out_of_range']}),
                         'ik:urdf_seed_out_of_range')


class FailureLedgerTests(unittest.TestCase):
    def test_task_level_failure_is_recorded_and_repeated_approach_is_eliminated(self):
        with tempfile.TemporaryDirectory() as tmp:
            ledger = FailureLedger(tmp, 'kiwi-01')
            release = [act('locate'), act('pick'), {'tool': 'move_to', 'args': {'xyz_m': [0.29, 0.0, 0.141]}, 'status': 'ok'},
                       {'tool': 'gripper', 'args': {'gripper_percent': 54.6}, 'status': 'ok'}]
            first = ledger.record_attempt(task='place the cube into the cup', run='run-a', attempt=1, actions=release,
                                          reason='both releases missed the cup')
            self.assertEqual(first, ('F001', 'tried'))
            record = ledger.records[0]
            self.assertEqual((record['level'], record['signature'], record['tail'], record['target']),
                             ('task', 'goal_not_achieved', 'locate>pick>move_to>gripper', [0.28, 0.0, 0.14]))
            self.assertEqual(ledger.record_attempt(task='place the cube into the cup', run='run-a', attempt=1,
                                                   actions=release, reason='same attempt again'), ('F001', 'tried'),
                             'one attempt is counted once')
            self.assertEqual(ledger.record_attempt(task='place the cube into the cup!', run='run-b', attempt=1,
                                                   actions=release, reason='missed again'), ('F001', 'eliminated'))
            lower = release[:2] + [{'tool': 'move_to', 'args': {'xyz_m': [0.29, 0.0, 0.08]}, 'status': 'ok'}, release[3]]
            self.assertEqual(ledger.record_attempt(task='place the cube into the cup', run='run-c', attempt=1,
                                                   actions=lower, reason='bounced off the rim'), ('F002', 'tried'),
                             'a different release height is a different approach')
            self.assertIsNone(ledger.record_attempt(task='x', run='run-d', attempt=1, actions=[act('observe')],
                                                    reason='nothing executed'))

    def test_planner_named_approach_groups_different_call_sequences(self):
        with tempfile.TemporaryDirectory() as tmp:
            ledger = FailureLedger(tmp, 'kiwi-01')
            one = [act('locate'), act('pick'), {'tool': 'move_to', 'args': {'xyz_m': [0.25, 0.0, 0.06]}, 'status': 'ok'}]
            two = [act('locate'), act('drive'), act('locate'), act('pick'), act('drive'),
                   {'tool': 'move_to', 'args': {'xyz_m': [0.22, 0.03, 0.08]}, 'status': 'ok'}]
            first = ledger.record_attempt(task='place the cube into the cup', run='run-a', attempt=1, actions=one,
                                          reason='cup tipped', approach='advance holding the cube toward the cup -> tipped it')
            self.assertEqual(first, ('F001', 'tried'))
            self.assertEqual(ledger.record_attempt(task='place the cube into the cup', run='run-b', attempt=1, actions=two,
                                                   reason='cup tipped again', approach='same advance, slower',
                                                   same_as='F001'), ('F001', 'eliminated'))
            self.assertEqual(ledger.records[0]['approach'], 'advance holding the cube toward the cup -> tipped it')
            self.assertEqual([f['approach'] for f in ledger.records[0]['failures']],
                             ['advance holding the cube toward the cup -> tipped it', 'same advance, slower'])

    def test_blocked_resolved_recall_and_markdown(self):
        with tempfile.TemporaryDirectory() as tmp:
            ledger = FailureLedger(tmp, 'kiwi-01')
            blocked = [act('locate'), {'tool': 'move_to', 'args': {'xyz_m': [0.22, 0.0, 0.085]}, 'status': 'failed',
                                       'error': 'move_to: IK not executable', 'detail': {'ik_reason_codes': ['ik_no_solution']}}]
            self.assertEqual(ledger.record_attempt(task='place the cube into the cup', run='run-a', attempt=1,
                                                   actions=blocked, reason='raised pose unreachable'), ('F001', 'blocked'))
            ledger.record_attempt(task='open the drawer', run='run-a', attempt=2, actions=[act('locate')], reason='no handle')
            self.assertEqual([item['id'] for item in ledger.recall('place the cube into the cup')], ['F001'])
            self.assertEqual(ledger.resolve('place the cube into the cup', 'run-b'), ['F001'])
            self.assertEqual(ledger.recall('place the cube into the cup')[0]['status'], 'resolved')
            ledger.save()
            text = (Path(tmp) / 'failure-ledger.md').read_text()
            self.assertIn('F001 · resolved', text)
            self.assertIn('ik:ik_no_solution', text)
            frozen = FailureLedger(tmp, 'kiwi-01', frozen=True)
            self.assertIsNone(frozen.record_attempt(task='t', run='r', attempt=1, actions=[act('locate')], reason='x'))
            self.assertEqual(frozen.resolve('open the drawer', 'run-c'), [])


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

    def test_import_experience_seeds_drive_samples(self):
        rows = [
            {'kind': 'action', 'tool': 'locate', 'status': 'ok', 'args': {'label': 'cube'},
             'result': {'xyz_m': [0.40, 0.0, -0.09]}},
            {'kind': 'action', 'tool': 'drive', 'status': 'ok',
             'args': {'drive': {'x': 0.1, 'y': 0.0, 'theta': 0.0, 'duration_ms': 1000}}},
            {'kind': 'action', 'tool': 'locate', 'status': 'ok', 'args': {'label': 'cube'},
             'result': {'xyz_m': [0.30, 0.0, -0.09]}},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'experience.jsonl'
            path.write_text('\n'.join(json.dumps({**row, 'robot_id': 'kiwi-01', 'run': 'runs/a'}) for row in rows))
            self.assertEqual(import_experience(path, Path(tmp) / 'memory', 'kiwi-01'), {'drive_samples': 1})
            self.assertEqual(EmbodimentMemory(Path(tmp) / 'memory', 'kiwi-01').drive_gain()['samples'], 1)


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
                body = EmbodimentMemory(memory_dir, 'memory-mock')
                seen, seen_ledger, planning_ledgers = [], [], []

                def base(context, tool, **fields):
                    values = {'label': None, 'bbox_px': None, 'xyz_m': None, 'approach': None,
                              'gripper_percent': None, 'drive': None, 'lesson': '', 'skills_applied': [],
                              'failed_approach': '', 'same_as_ledger': None}
                    values.update(fields)
                    return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                            'tool': tool, 'reason': f'scripted {tool}', 'expected': '', **values}

                reviews = []

                class Planner:
                    def __init__(self, applied, fail=False, test=None):
                        self.applied, self.fail, self.test = applied, fail, test

                    async def plan(self, context, images):
                        seen.append(context['memory']['skill_library'])
                        planning_ledgers.append(context['memory']['failure_ledger'])
                        return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                                'tool': 'plan', 'reason': 'scripted plan', 'expected_outcome': 'cube located close',
                                'library_used': 'follow the verified workflow', 'skills_applied': self.applied,
                                'hypothesis_tested': self.test,
                                'hypothesis_change': 'drive 0.1 m closer before judging reach' if self.test else '',
                                'plan': [{'step': 'locate, drive, locate', 'skill': 'locate', 'success_check': 'x<0.35'}]}

                    async def review(self, context, images):
                        reviews.append((context, [label for label, _, _ in images]))
                        return {'robot_id': context['robot_id'], 'failed_step_index': 0, 'failure_level': 'planning',
                                'failure_type': 'wrong_plan', 'confidence': 'medium',
                                'step_checks': [{'index': 0, 'expected': 'cube within reach', 'observed': 'x=0.40 m',
                                                 'met': 'no'}],
                                'hypothesis': 'gave up from 0.40 m without driving closer; reach is x<0.30',
                                'evidence': 'locate x=0.40 at step 0', 'next_try': 'drive 0.1 m forward, then locate',
                                'prediction': 'second locate reads x about 0.30', 'approach': 'locate far, give up',
                                'same_as_ledger': None}

                    async def act(self, context, images):
                        step = len(context['history'])
                        if step == 0:
                            return base(context, 'locate', label='cube', bbox_px=box_for(0.40))
                        if step == 1 and self.fail:
                            seen_ledger.append(context['failure_ledger'])
                            return base(context, 'failed', reason='scripted: cube judged out of reach',
                                        failed_approach='locate from far away, then give up')
                        if step == 1:
                            return base(context, 'drive', drive={'x': 0.1, 'y': 0.0, 'theta': 0.0, 'duration_ms': 1000})
                        if step == 2:
                            return base(context, 'locate', label='cube', bbox_px=box_for(0.305))
                        return base(context, 'done', lesson='a 0.1 m/s 1 s drive closes about 0.095 m of distance')

                    async def verify(self, context, images):
                        return {'robot_id': context['robot_id'], 'observation_id': context['observation_id'],
                                'tool': 'confirm', 'reason': 'scripted confirm', 'evidence': 'scripted', 'lesson': '',
                                'failed_approach': '', 'same_as_ledger': None}

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
                results = []
                async with stdio_client(params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        for index, (applied, fail, test) in enumerate(((['S999'], True, None), (['S999'], False, 'F001'),
                                                                       (['S001'], False, None))):
                            runner = EpisodeRunner(None, MCPRobotTools(session), 'memory-mock', 'find the cube',
                                                   Path(folder) / f'episode-{index}',
                                                   EpisodeLimits(max_actions=6, max_steps=10, max_seconds=60),
                                                   cartesian=cartesian)
                            skills = Skills(runner, skills_config, reason='memory integration test')
                            library = SkillLibrary(memory_dir / 'skill-library', 'memory-mock')
                            ledger = FailureLedger(memory_dir, 'memory-mock')
                            results.append(await PlanActRunner(runner, Planner(applied, fail, test), skills, camera, None,
                                                               max_replans=0, library=library, embodiment=body,
                                                               ledger=ledger).run())

                failed, first, second = results
                self.assertEqual(failed['status'], 'replan_budget', failed)
                self.assertFalse(failed['memory']['verification']['passed'])
                self.assertIn('skipped', failed['memory']['library'])
                self.assertEqual(failed['memory']['ledger'], {'recorded': [['F001', 'tried']], 'resolved': []})
                self.assertEqual(planning_ledgers[0], [])
                self.assertEqual([item['id'] for item in planning_ledgers[1]], ['F001'],
                                 'the next episode plans with the failed approach in view')
                self.assertEqual(planning_ledgers[1][0]['last_reason'], 'scripted: cube judged out of reach')
                # REVIEW turn: steps with intent and measurements, host facts, key frames; stored as hypothesis
                review_context, review_images = reviews[0]
                self.assertEqual(len(reviews), 1, 'one review per failed attempt, none repeated after the stop')
                self.assertEqual(review_context['phase'], 'REVIEW')
                self.assertEqual(review_context['steps'][0]['tool'], 'locate')
                self.assertIn('expected', review_context['steps'][0])
                self.assertEqual(review_context['host_facts']['calls'], 1)
                self.assertIn('ATTEMPT START: front', review_images)
                self.assertEqual(failed['memory']['hypotheses'][0]['id'], 'H1')
                ledgered = planning_ledgers[1][0]
                self.assertEqual(ledgered['approach'], 'locate far, give up', 'the review names the approach')
                self.assertEqual(ledgered['failure_type'], 'wrong_plan')
                self.assertEqual(ledgered['next_try'], 'drive 0.1 m forward, then locate')
                self.assertEqual(ledgered['hypothesis_status'], 'untested')
                stored_ledger = FailureLedger(memory_dir, 'memory-mock')
                f001 = stored_ledger.records[0]
                self.assertEqual(f001['hypothesis_status'], 'supported', 'episode 2 tested F001 and succeeded')
                self.assertEqual(f001['hypothesis_tests'][0]['change'], 'drive 0.1 m closer before judging reach')
                self.assertIn('Why (hypothesis, supported)', (memory_dir / 'failure-ledger.md').read_text())
                self.assertEqual(first['memory']['ledger']['resolved'], ['F001'])
                seen = seen[1:]
                self.assertEqual(first['status'], 'completion_claim', first)
                self.assertEqual(seen[0], [], 'empty library on the first episode')
                self.assertTrue(first['memory']['verification']['passed'], first['memory'])
                self.assertEqual(first['memory']['library'], {'added': ['S001'], 'confirmed': []})
                self.assertEqual(first['memory']['reuse'], {}, 'unknown id S999 is ignored')
                self.assertEqual(seen[1][0]['id'], 'S001')
                self.assertEqual(second['memory']['reuse'], {'S001': 'confirmed'})
                stored = SkillLibrary(memory_dir / 'skill-library', 'memory-mock')
                entry = stored.entries[0]
                self.assertEqual(len(stored.entries), 1, 'the same verified workflow is confirmed, not duplicated')
                self.assertEqual(len(entry['confirmed_in']), 1)
                self.assertIn('Fix found by testing F001 (wrong_plan)', entry['strategy'], 'skill discovery keeps why')
                self.assertIn('a 0.1 m/s 1 s drive closes about 0.095 m of distance', entry['strategy'])
                self.assertTrue(entry['snippet'][1].startswith('drive(x=0.1 m/s'))
                self.assertIn('S001', (memory_dir / 'skill-library' / 'episodic-workflows.md').read_text())
                saved_body = EmbodimentMemory(memory_dir, 'memory-mock').drive_gain()
                self.assertEqual(saved_body['samples'], 2)
                self.assertAlmostEqual(saved_body['gain'], 0.95, delta=0.03)
                events = [json.loads(line) for line in (Path(folder) / 'episode-2' / 'events.jsonl').read_text().splitlines()]
                decisions = [e['data'] for e in events if e['event'] == 'decision']
                self.assertEqual([item['id'] for item in decisions[0]['skills_applied']], ['S001'])
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
