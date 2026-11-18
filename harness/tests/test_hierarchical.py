"""Unit and integration tests for the ASPIRE-style LeKiwi campaign harness."""
from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout

from harness.campaign import (
    CampaignPolicyRunner,
    LiveLocalizationProvider,
    PolicyValidationError,
    PoseEstimate,
    SemanticMemoryStore,
    SimObject,
    SimulatedCampaignWorld,
    TopologicalMap,
    build_default_sim_topology,
    init_skill_library,
    main as campaign_cli_main,
    sanitize_policy_result,
    validate_and_compile_policy,
)


HARNESS_ROOT = Path(__file__).resolve().parents[1]
SKILLS_DIR = HARNESS_ROOT / '.claude' / 'skills' / 'lekiwi-mobile-manip'
SCORE_SCRIPT = SKILLS_DIR / 'score_campaign.py'


def _load_score_module():
    import sys
    prev = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec = importlib.util.spec_from_file_location('score_campaign', SCORE_SCRIPT)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.dont_write_bytecode = prev


class SemanticMemoryTests(unittest.TestCase):
    def test_persistent_upsert_search_and_mark_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            db_dir = Path(tmp) / 'chroma'
            store = SemanticMemoryStore(persist_dir=db_dir)
            hit_2f = store.upsert(
                'red ceramic mug on kitchen counter',
                floor='2F',
                x_m=4.2,
                y_m=1.5,
                yaw_deg=90.0,
                room='pantry',
                tags=('mug', 'red'),
            )
            store.upsert(
                'blue vitamin bottle on lobby shelf',
                floor='1F',
                x_m=1.1,
                y_m=0.4,
                yaw_deg=0.0,
                room='lobby',
                tags=('bottle', 'blue'),
            )

            reloaded = SemanticMemoryStore(persist_dir=db_dir)
            self.assertEqual(reloaded.count(), 2)
            hits = reloaded.search('red mug', top_k=2)
            self.assertGreaterEqual(len(hits), 1)
            self.assertEqual(hits[0].record_id, hit_2f.record_id)
            self.assertEqual(hits[0].floor, '2F')

            floor_filtered = reloaded.search('red mug', floor='1F', top_k=2)
            self.assertTrue(all(h.floor == '1F' for h in floor_filtered))

            updated = reloaded.mark_absent(hit_2f.record_id, note='moved_by_user')
            self.assertEqual(updated, 1)
            after_absent = reloaded.search('red ceramic mug', floor='2F', top_k=2)
            self.assertEqual(after_absent, [])
            with_absent = reloaded.search('red ceramic mug', floor='2F', include_absent=True, top_k=2)
            self.assertEqual(len(with_absent), 1)
            self.assertEqual(with_absent[0].status, 'absent')


class TopologicalMapTests(unittest.TestCase):
    def test_multi_floor_route_planning_and_persistence(self):
        topo = build_default_sim_topology()
        route = topo.plan_route('1F', 0.4, 0.5, '2F', 4.2, 1.5, target_yaw_deg=45.0)
        self.assertTrue(route.ok)
        self.assertEqual(route.floors_traversed, ('1F', '2F'))
        self.assertTrue(any(leg.via == 'ramp' for leg in route.legs))
        self.assertAlmostEqual(route.legs[-1].x_m, 4.2)
        self.assertAlmostEqual(route.legs[-1].yaw_deg, 45.0)

        pantry = topo.find_place('kitchen')
        self.assertIsNotNone(pantry)
        assert pantry is not None
        self.assertEqual(pantry.floor, '2F')

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'topology.json'
            topo.save(path)
            loaded = TopologicalMap.load(path)
            route2 = loaded.plan_route('1F', 0.4, 0.5, '2F', 4.2, 1.5)
            self.assertTrue(route2.ok)
            self.assertEqual(route2.floors_traversed, ('1F', '2F'))

    def test_ingest_trajectory_spacing(self):
        topo = TopologicalMap()
        pts = [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 90.0)]
        created = topo.ingest_trajectory('1F', pts, spacing_m=0.75, prefix='hall')
        self.assertEqual(len(created), 3)


class PolicySandboxValidationTests(unittest.TestCase):
    def test_allows_math_and_rejects_forbidden_syntax(self):
        valid = 'import math\nx = math.hypot(3.0, 4.0)\nresult = {"dist": x}\n'
        compiled = validate_and_compile_policy(valid)
        self.assertIsNotNone(compiled)

        invalid_cases = [
            'import os\nresult = {}\n',
            'import math as m\nresult = {}\n',
            'from math import hypot\nresult = {}\n',
            'for _ in range(3):\n    pass\nresult = {}\n',
            'x = (1).__class__\nresult = {}\n',
            'try:\n    x = 1\nexcept Exception:\n    x = 2\nresult = {}\n',
        ]
        for bad_src in invalid_cases:
            with self.subTest(bad_src=bad_src):
                with self.assertRaises(PolicyValidationError):
                    validate_and_compile_policy(bad_src)

    def test_sanitize_policy_result_strips_privileged_substrings(self):
        raw = {
            'stage': 'done',
            'grasp_ok': True,
            'grasp_success': True,
            'ground_truth_pose': [1, 2, 3],
            'nested': {'reward_delta': 1.0, 'distance_m': 0.35},
        }
        cleaned = sanitize_policy_result(raw)
        self.assertEqual(cleaned, {'stage': 'done', 'grasp_ok': True, 'nested': {'distance_m': 0.35}})


class CampaignRunnerTests(unittest.TestCase):
    def test_multi_floor_stale_memory_recovery_and_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            store = SemanticMemoryStore()
            # Seed a stale memory at (1.0, 1.0) on 2F and actual mug at (4.2, 1.5) on 2F.
            stale = store.upsert('red ceramic mug on counter', floor='2F', x_m=1.0, y_m=1.0, room='old_spot')
            store.upsert('red ceramic mug in pantry', floor='2F', x_m=4.1, y_m=1.5, room='pantry')

            topo = build_default_sim_topology()
            world = SimulatedCampaignWorld.default_two_floor_world()
            runner = CampaignPolicyRunner(
                task_language='Bring the red ceramic mug from 2F to 1F lobby',
                robot_id='kiwi-01',
                mode='sim',
                semantic_memory=store,
                topology=topo,
                localization=world,
                navigation_executor=world,
                action_executor=world,
                goal_checker=lambda _task: (
                    not world.objects['red_mug'].held and world.objects['red_mug'].floor == '1F'
                ),
            )

            policy = '''import math

report = {"stage": "start"}

# Code block 1: relocalize start floor and query spatial memory.
pose = get_pose()
if not pose["relocalized"]:
    pose = relocalize()
mem = search_memory("red ceramic mug", top_k=3)
report["hits_initial"] = len(mem["hits"])
print("initial hits:", len(mem["hits"]))

# Code block 2: search candidate hits, invalidating stale coordinates if mug not visible.
found_spot = False
for idx in range(len(mem["hits"])):
    cand = mem["hits"][idx]
    if not found_spot:
        nav = navigate_to(cand["floor"], cand["x_m"], cand["y_m"], cand["yaw_deg"])
        if nav["ok"]:
            obs = inspect_scene("Is the red ceramic mug visible here?")
            if "red ceramic mug" in obs["summary"]:
                found_spot = True
                report["found_at"] = cand["room"]
            else:
                mark_absent(cand["record_id"], note="not_visible_on_arrival")

# Code block 3: grasp mug, return to 1F lobby, and place it.
if found_spot:
    grasp = act("Grasp and lift the red ceramic mug")
    report["grasp_ok"] = grasp["ok"]
    if grasp["ok"]:
        back = navigate_to_place("lobby", floor="1F")
        report["return_ok"] = back["ok"]
        if back["ok"]:
            placed = act("Place and release the red ceramic mug on the lobby table")
            report["place_ok"] = placed["ok"]
            if placed["ok"]:
                save_memory("red ceramic mug on 1F lobby table", room="lobby", tags=["mug"])
                result = report
                finish("Delivered red ceramic mug to 1F lobby")

result = report
'''
            attempt_dir = root / 'attempt_001'
            outcome = runner.run_policy_source(policy, attempt_dir)
            self.assertTrue(outcome.program_ok)
            self.assertTrue(outcome.task_success)
            self.assertTrue(outcome.navigation_reached)
            self.assertEqual(outcome.termination_reason, 'completed')

            # Check stale entry was marked absent and new 1F memory was stored.
            active_2f = store.search('red ceramic mug', floor='2F')
            self.assertTrue(all(h.record_id != stale.record_id for h in active_2f))
            active_1f = store.search('red ceramic mug', floor='1F')
            self.assertGreaterEqual(len(active_1f), 1)

            # Check ASPIRE artifact files.
            self.assertTrue((attempt_dir / 'outcome.json').is_file())
            self.assertTrue((attempt_dir / 'source' / 'policy.py').is_file())
            self.assertTrue((attempt_dir / 'source' / 'policy-result.json').is_file())
            self.assertIn('initial hits:', (attempt_dir / 'logs' / 'policy.stdout').read_text(encoding='utf-8'))

            events = [
                json.loads(line)
                for line in (attempt_dir / 'trace' / 'events.jsonl').read_text(encoding='utf-8').splitlines()
                if line.strip()
            ]
            self.assertGreaterEqual(len(events), 6)
            self.assertTrue(any('>>>' in ev['highlighted_policy'] for ev in events))

    def test_uncertain_execution_aborts_immediately_without_running_later_lines(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = SimulatedCampaignWorld.default_two_floor_world()
            world.pose = PoseEstimate(floor='1F', x_m=2.2, y_m=0.8, confidence=0.99, relocalized=True)
            world.force_uncertain_on_tool = 'act'
            runner = CampaignPolicyRunner(
                task_language='Pick up blue vitamin bottle',
                robot_id='kiwi-01',
                mode='sim',
                semantic_memory=SemanticMemoryStore(),
                topology=build_default_sim_topology(),
                localization=world,
                navigation_executor=world,
                action_executor=world,
            )
            policy = '''report = {"after_uncertain": False}
act("Pick up the blue vitamin bottle")
report["after_uncertain"] = True
print("THIS MUST NEVER PRINT")
result = report
'''
            out_dir = Path(tmp) / 'attempt_uncertain'
            outcome = runner.run_policy_source(policy, out_dir)
            self.assertFalse(outcome.program_ok)
            self.assertFalse(outcome.task_success)
            self.assertEqual(outcome.termination_reason, 'uncertain_execution')
            saved_res = json.loads((out_dir / 'source' / 'policy-result.json').read_text(encoding='utf-8'))
            self.assertFalse(saved_res['result']['after_uncertain'])
            self.assertEqual((out_dir / 'logs' / 'policy.stdout').read_text(encoding='utf-8'), '')

    def test_loop_tick_budget_stops_infinite_loop(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = SimulatedCampaignWorld.default_two_floor_world()
            runner = CampaignPolicyRunner(
                task_language='Loop test',
                robot_id='kiwi-01',
                mode='sim',
                semantic_memory=SemanticMemoryStore(),
                topology=TopologicalMap(),
                localization=world,
                navigation_executor=world,
                action_executor=world,
                max_loop_ticks=25,
            )
            outcome = runner.run_policy_source('n = 0\nwhile True:\n    n = n + 1\nresult = {"n": n}\n', Path(tmp) / 'loop')
            self.assertFalse(outcome.program_ok)
            self.assertEqual(outcome.termination_reason, 'loop_limit_exceeded')

    def test_unverified_finish_with_zero_robot_stages_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = SimulatedCampaignWorld.default_two_floor_world()
            runner = CampaignPolicyRunner(
                task_language='Pick up mug',
                robot_id='kiwi-01',
                mode='sim',
                semantic_memory=SemanticMemoryStore(),
                topology=TopologicalMap(),
                localization=world,
                navigation_executor=world,
                action_executor=world,
            )
            outcome = runner.run_policy_source('report = {}\nresult = report\nfinish("instant")\n', Path(tmp) / 'fake')
            self.assertTrue(outcome.program_ok)
            self.assertFalse(outcome.task_success)
            self.assertEqual(outcome.termination_reason, 'unverified_finish')

    def test_slam_free_visual_mode_branches_on_zero_confidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            loc = LiveLocalizationProvider(pose_file=None)
            world = SimulatedCampaignWorld(
                pose=PoseEstimate(floor='local', x_m=0.0, y_m=0.0, confidence=0.0, relocalized=False),
                objects={'mug': SimObject('mug', 'red ceramic mug', floor='local', x_m=0.45, y_m=0.0)},
            )
            runner = CampaignPolicyRunner(
                task_language='Pick up the red ceramic mug visually',
                robot_id='kiwi-01',
                mode='live',
                semantic_memory=SemanticMemoryStore(),
                topology=TopologicalMap(),
                localization=loc,
                navigation_executor=world,
                action_executor=world,
            )
            policy = '''report = {}
pose = get_pose()
report["slam_active"] = pose["confidence"] > 0.0
if not report["slam_active"]:
    obs = inspect_scene("Locate the red ceramic mug in front camera")
    if obs["ok"]:
        grasp = act("Approach visually and grasp the red ceramic mug")
        report["grasp_ok"] = grasp["ok"]
        if grasp["ok"]:
            result = report
            finish("Grasped mug in SLAM-free visual mode")
result = report
'''
            outcome = runner.run_policy_source(policy, Path(tmp) / 'visual_only')
            self.assertTrue(outcome.program_ok)
            self.assertTrue(outcome.task_success)
            self.assertTrue(outcome.navigation_reached)


class SkillLibraryAndScoringTests(unittest.TestCase):
    def test_init_skill_library_creates_headings_only_templates_without_body_leakage(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / 'skill-library'
            written = init_skill_library(dest)
            names = sorted(p.name for p in written)
            self.assertEqual(
                names,
                [
                    'episodic-workflows.md',
                    'lekiwi-operations.md',
                    'mobile-grasp.md',
                    'navigation.md',
                    'search-and-memory.md',
                    'time-budget.md',
                ],
            )
            for file_path in written:
                lines = [ln for ln in file_path.read_text(encoding='utf-8').splitlines() if ln.strip()]
                self.assertGreaterEqual(len(lines), 2, f'{file_path.name} should have headings')
                for ln in lines:
                    self.assertTrue(
                        ln.startswith('# ') or ln.startswith('## '),
                        f'Non-heading line leaked into {file_path.name}: {ln!r}',
                    )

    def test_score_campaign_evaluates_only_final_policy_hash(self):
        score_mod = _load_score_module()
        with tempfile.TemporaryDirectory() as tmp:
            camp = Path(tmp) / 'lekiwi'
            task_dir = camp / 'fetch_mug' / 'stage2'

            def write_attempt(trial: str, attempt: str, policy_src: str, *, success: bool, reached: bool) -> None:
                adir = task_dir / trial / 'attempts' / attempt
                (adir / 'source').mkdir(parents=True, exist_ok=True)
                (adir / 'source' / 'policy.py').write_text(policy_src, encoding='utf-8')
                (adir / 'outcome.json').write_text(
                    json.dumps(
                        {
                            'program_ok': True,
                            'task_success': success,
                            'navigation_reached': reached,
                            'termination_reason': 'completed' if success else 'end_of_policy',
                        }
                    ),
                    encoding='utf-8',
                )

            # trial_01: block 1 (dev), block 1+2 (dev), final policy (success) + 1 repeat (failed) -> SUCCESS=yes, NAV=yes
            write_attempt('trial_01', 'attempt_001', '# block 1\nresult = {}\n', success=False, reached=False)
            write_attempt('trial_01', 'attempt_002', '# block 1+2\nresult = {}\n', success=False, reached=True)
            final_p1 = '# block 1+2+3\nresult = {"done": True}\n'
            write_attempt('trial_01', 'attempt_003', final_p1, success=True, reached=True)
            write_attempt('trial_01', 'attempt_004', final_p1, success=False, reached=True)

            # trial_02: reached & engaged arm, but grasp never held -> SUCCESS=no, NAV=yes
            write_attempt('trial_02', 'attempt_001', '# block 1\nresult = {}\n', success=False, reached=False)
            write_attempt('trial_02', 'attempt_002', '# block 1+2\nresult = {}\n', success=False, reached=True)

            # VOIDED trial must be ignored completely
            write_attempt('VOIDED_trial_03', 'attempt_001', 'result = {}\n', success=False, reached=False)

            res = score_mod.score_task(camp / 'fetch_mug', 'stage2')
            self.assertIsNotNone(res)
            assert res is not None
            self.assertEqual(res['trials'], 2)
            self.assertEqual(res['success'], 1)
            self.assertEqual(res['navigated'], 2)

            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = score_mod.main([str(camp), '--stage', 'stage2', '--per-trial'])
            self.assertEqual(rc, 0)
            output = buf.getvalue()
            self.assertIn('SUCCESS      1/2  = 50%', output)
            self.assertIn('NAVIGATION   2/2  = 100%', output)

    def test_cli_run_and_init_library_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            lib_dir = root / 'lib'
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc_init = campaign_cli_main(['init-library', '--dest', str(lib_dir)])
            self.assertEqual(rc_init, 0)
            self.assertTrue((lib_dir / 'episodic-workflows.md').is_file())

            policy_file = root / 'policy.py'
            policy_file.write_text(
                'pose = relocalize()\n'
                'nav = navigate_to("2F", 4.0, 1.5, 0.0)\n'
                'grasp = act("Grasp red ceramic mug")\n'
                'result = {"ok": grasp["ok"]}\n'
                'if grasp["ok"]:\n'
                '    finish("Picked up mug in sim")\n',
                encoding='utf-8',
            )
            out_dir = root / 'attempt_cli'
            buf_run = io.StringIO()
            with redirect_stdout(buf_run):
                rc_run = campaign_cli_main(
                    [
                        'run',
                        '--policy',
                        str(policy_file),
                        '--output-dir',
                        str(out_dir),
                        '--mode',
                        'sim',
                        '--seed-sim-memory',
                    ]
                )
            self.assertEqual(rc_run, 0)
            parsed = json.loads(buf_run.getvalue())
            self.assertTrue(parsed['task_success'])
            self.assertTrue(parsed['navigation_reached'])


if __name__ == '__main__':
    unittest.main()
