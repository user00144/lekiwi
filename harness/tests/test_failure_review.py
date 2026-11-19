"""Failure review → hypothesis → test: host checks around the REVIEW turn and the ledger lifecycle."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from harness.memory import FailureLedger, ReflectionMemory
from harness.plan_loop import PlanActRunner


def review_answer(**changes):
    answer = {'robot_id': 'kiwi', 'failed_step_index': 3, 'failure_type': 'release_misaligned', 'confidence': 'medium',
              'inspection': {'verdict': 'error', 'finding': 'took the cup footprint for the mouth; depth not visible',
                             'fix': 'drive 5 cm forward and check the mouth with the wrist camera from above'},
              'execution': {'verdict': 'ok', 'finding': 'release opened as commanded', 'fix': ''},
              'planning': {'verdict': 'unclear', 'finding': 'released without a depth check', 'fix': 'add a depth check'},
              'hypothesis': 'released 4 cm above the rim at the located centre; the cup centre was 2 cm off',
              'evidence': 'AFTER step 3 place: cube beside the cup', 'next_try': 'relocate the cup from 0.25 m, release at z+0.02',
              'prediction': 'cube inside the cup on the wrist image', 'approach': 'place over the cup centre from z 0.14',
              'same_as_ledger': None}
    answer.update(changes)
    return answer


REFLECTION = {'robot_id': 'kiwi', 'title': 'Released over the rim three times; never re-located the cup up close',
              'what_happened': 'Three attempts carried the cube to the cup and released it 4 cm above the rim.',
              'why_failed': 'The cup was located from 0.40 m, where locate is 2 cm off; every release used that xyz.',
              'regret': 'I should have driven to 0.25 m and re-located the cup before the first release.',
              'lessons': ['When the container is farther than 0.30 m, drive closer and locate it again before placing.'],
              'next_hypothesis': 'A cup located from 0.25 m is accurate enough to release at rim + 0.02 m.',
              'evidence': 'AFTER place frames: cube beside the cup each time', 'confidence': 'medium'}


class ScriptedReviewer:
    def __init__(self, **changes):
        self.calls, self.reflect_calls, self.changes = [], [], changes

    async def review(self, context, images):
        self.calls.append((context, images))
        return review_answer(**self.changes)

    async def reflect(self, context, images):
        self.reflect_calls.append((context, images))
        return dict(REFLECTION)


def make_runner(planner, ledger=None):
    """A PlanActRunner with only what the review path uses (no robot, no Codex)."""
    runner = PlanActRunner.__new__(PlanActRunner)
    events = []
    runner.runner = SimpleNamespace(robot_id='kiwi', packet=None, record=SimpleNamespace(
        path=Path('/runs/test-run'), event=lambda kind, data: events.append((kind, data))))
    runner.planner, runner.ledger, runner.experience = planner, ledger, None
    runner.frames = [{'index': None, 'tool': 'start', 'images': [('front', 'F0', 'image/jpeg')]},
                     {'index': 1, 'tool': 'locate', 'images': [('front', 'F0', 'image/jpeg')]},
                     {'index': 3, 'tool': 'place', 'images': [('front', 'F3', 'image/jpeg'), ('wrist', 'W3', 'image/jpeg')]}]
    runner.hypotheses, runner.attempts, runner.current, runner.ledger_seen = [], [], None, []
    runner.reflections, runner.reflections_seen, runner.reflection, runner.settled = None, [], None, False
    runner._context = lambda phase, **extra: {'phase': phase, **extra}
    runner._ledger_view = lambda compact=False: []
    runner._images = lambda: [('CURRENT front', 'F3', 'image/jpeg'), ('CURRENT wrist', 'WC', 'image/jpeg')]
    return runner, events


def attempt(actions, **extra):
    return {'attempt': 1, 'plan': [], 'actions': actions, 'hypothesis_tested': None, 'hypothesis_change': '', **extra}


LOCATE_CUP = {'tool': 'locate', 'args': {'label': 'cup'}, 'status': 'ok', 'result': {'xyz_m': [.26, .05, -.09]}}
ACTIONS = [
    {'tool': 'locate', 'args': {'label': 'cube'}, 'status': 'ok', 'result': {'xyz_m': [.25, 0., -.09]}},
    LOCATE_CUP,
    {'tool': 'pick', 'args': {'xyz_m': [.25, 0., -.09]}, 'status': 'ok',
     'result': {'gripper_after_close': 27.7, 'object_between_jaws_likely': True}},
    {'tool': 'place', 'args': {'xyz_m': [.26, .05, .05]}, 'status': 'ok', 'result': {'release_fk_xyz_m': [.26, .05, .05]}},
    {**LOCATE_CUP, 'result': {'xyz_m': [.30, .07, -.09]}},
]


class ReviewTest(unittest.IsolatedAsyncioTestCase):
    async def test_review_becomes_a_hypothesis_with_host_facts_and_key_frames(self):
        planner = ScriptedReviewer()
        runner, events = make_runner(planner)
        summary = attempt(list(ACTIONS))
        review = await runner.review(summary, {'verdict': 'failed', 'reason': 'cube beside the cup'}, counted=False)
        self.assertEqual(review['id'], 'H1')
        self.assertEqual(review['status'], 'untested')
        self.assertEqual(review['aspects_in_error'], ['inspection'])
        self.assertEqual(review['failure_level'], 'inspection')
        self.assertEqual(runner._hypotheses_view()[0]['fixes'],
                         {'inspection': 'drive 5 cm forward and check the mouth with the wrist camera from above',
                          'planning': 'add a depth check'}, 'fixes of aspects that were not ok go to PLANNING')
        self.assertEqual(review['failed_step'], {'index': 3, 'tool': 'place', 'phase': None})
        self.assertIs(summary['review'], review)
        context, images = planner.calls[0]
        moved = context['host_facts']['objects_moved_without_base_motion']
        self.assertEqual([(m['label'], m['from_step'], m['to_step']) for m in moved], [('cup', 1, 4)],
                         'the cup moved between two locates without base motion')
        self.assertEqual(moved[0]['skills_between'], ['pick', 'place'])
        labels = [label for label, _, _ in images]
        self.assertEqual(labels, ['ATTEMPT START: front', 'AFTER step 3 place: front', 'AFTER step 3 place: wrist',
                                  'CURRENT wrist'], 'start + scene-changing frames + current, no duplicates')
        self.assertEqual(events[-1][1]['phase'], 'review')

    async def test_host_measurements_bound_the_review(self):
        failed_pick = [ACTIONS[0], {'tool': 'pick', 'args': {'xyz_m': [.25, 0., -.09]}, 'status': 'failed',
                                    'error': 'after lift: the object is no longer between the jaws',
                                    'result': {'failed_step': 'after lift'}}, ACTIONS[3]]
        runner, _ = make_runner(ScriptedReviewer(failed_step_index=2, failure_type='release_misaligned'))
        review = await runner.review(attempt(failed_pick), {'verdict': 'failed'}, counted=False)
        self.assertEqual(review['failed_step_index'], 1, 'a later step cannot be the first failure')
        self.assertTrue(review['host_notes'])
        self.assertEqual(review['host_type_hint'], 'grasp_missed')
        self.assertFalse(review['agrees_with_host'])
        runner, _ = make_runner(ScriptedReviewer(failed_step_index=9))
        review = await runner.review(attempt(list(ACTIONS)), {'verdict': 'failed'}, counted=False)
        self.assertIsNone(review['failed_step_index'])

    async def test_monitor_stop_hint_and_planner_without_review(self):
        carried = [ACTIONS[0], {'tool': 'place', 'status': 'failed', 'error': 'place hover: stopped mid-motion',
                                'result': {'failed_step': 'place hover', 'monitor_stop': {'progress_fraction': .48}}}]
        runner, _ = make_runner(ScriptedReviewer(failure_type='grip_lost', failed_step_index=1))
        review = await runner.review(attempt(carried), {'verdict': 'failed'}, counted=False)
        self.assertEqual(review['host_type_hint'], 'grip_lost')
        self.assertTrue(review['agrees_with_host'])
        self.assertEqual(review['failed_step']['phase'], 'place hover')
        runner, _ = make_runner(SimpleNamespace())
        self.assertIsNone(await runner.review(attempt(carried), {'verdict': 'failed'}, counted=False))

    async def test_episode_reflection_only_for_real_failures(self):
        failed, passed = {'passed': False}, {'passed': True}
        planner = ScriptedReviewer()
        runner, events = make_runner(planner)
        runner.current = attempt(list(ACTIONS))
        for reason, verification in (('uncertain_execution', failed), ('interrupted', failed),
                                      ('completion_claim', passed)):
            self.assertIsNone(await runner._reflect_episode(reason, '', verification))
        self.assertEqual(planner.reflect_calls, [])
        reflection = await runner._reflect_episode('budget_steps', 'step budget used', failed)
        self.assertEqual(reflection['title'], REFLECTION['title'])
        context, _ = planner.reflect_calls[0]
        self.assertEqual(context['phase'], 'REFLECT')
        self.assertEqual(context['attempts'][-1]['verdict'], 'incomplete', 'the attempt the stop cut short is included')
        self.assertEqual(context['attempts'][-1]['steps'][3]['tool'], 'place')
        self.assertEqual(context['outcome']['status'], 'budget_steps')
        self.assertEqual(events[-1][1]['phase'], 'reflect')
        verified_claim_failed = await make_runner(ScriptedReviewer())[0]._reflect_episode(
            'completion_claim', '', failed)
        self.assertIsNone(verified_claim_failed, 'no executed steps -> nothing to reflect on')
        runner, _ = make_runner(SimpleNamespace())
        runner.current = attempt(list(ACTIONS))
        self.assertIsNone(await runner._reflect_episode('budget_steps', '', failed), 'planner without reflect')

    async def test_tests_settle_episode_and_ledger_hypotheses(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            runner, events = make_runner(ScriptedReviewer(), ledger)
            first = attempt(list(ACTIONS))
            await runner.review(first, {'verdict': 'failed'}, counted=False)
            second = attempt(list(ACTIONS), attempt=2, hypothesis_tested='H1', hypothesis_change='release at z+0.02')
            self.assertEqual(runner._settle_test(second, 'not_supported'), 'H1')
            self.assertEqual(runner.hypotheses[0]['status'], 'not_supported')
            self.assertEqual(runner.hypotheses[0]['tests'][0]['change'], 'release at z+0.02')
            record_id, _ = ledger.record_attempt(task='put the cube in the cup', run='r1', attempt=1, actions=ACTIONS,
                                                 reason='beside the cup', review=runner.hypotheses[0])
            later = attempt(list(ACTIONS), hypothesis_tested=record_id, hypothesis_change='relocate from 0.25 m')
            self.assertEqual(runner._settle_test(later, 'supported'), record_id)
            self.assertIsNone(runner._settle_test(attempt([], hypothesis_tested='F999'), 'supported'))
            self.assertEqual([kind for kind, _ in events].count('hypothesis_test'), 2)


class SharingAcrossEpisodesTest(unittest.IsolatedAsyncioTestCase):
    async def test_in_episode_test_result_reaches_the_ledger_record(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            runner, _ = make_runner(ScriptedReviewer(), ledger)
            runner.runner.instruction, runner.located = 'put the cube in the cup', {}
            first = {**attempt(list(ACTIONS)), 'verdict': 'failed', 'reason': 'beside the cup'}
            await runner.review(first, {'verdict': 'failed'}, counted=False)
            second = {**attempt(list(ACTIONS), attempt=2, hypothesis_tested='H1', hypothesis_change='z+0.02'),
                      'verdict': 'failed', 'reason': 'beside the cup again'}
            runner._settle_test(second, 'not_supported')
            runner.attempts = [first, second]
            recorded = runner._record_failures('replan_budget', '', 'run-1')
            record = next(r for r in ledger.records if r['id'] == recorded[0][0])
            self.assertEqual(record['hypothesis_status'], 'not_supported', 'H1 failed its test in attempt 2')
            self.assertEqual(record['hypothesis_tests'][0]['change'], 'z+0.02')

    def test_new_hypotheses_are_not_crowded_out_by_old_records(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            task = 'Inspect the front and place the cube into the cup'
            for index in range(8):  # older records without a hypothesis (as imported before reviews existed)
                ledger.record_attempt(task='Inspect front of camera and place the cube into the cup', run=f'old-{index}',
                                      attempt=1, actions=[{**ACTIONS[3], 'args': {'xyz_m': [.2 + index / 50, 0, .05]}}],
                                      reason='old failure', approach=f'old approach {index}')
            review = {**review_answer(), 'failed_step': None}
            new_id, _ = ledger.record_attempt(task=task, run='new', attempt=1, actions=ACTIONS, reason='new failure',
                                              review=review)
            tested_id, _ = ledger.record_attempt(task=task, run='new', attempt=2, actions=ACTIONS[:2], reason='x',
                                                 review={**review, 'approach': 'another approach'})
            ledger.record_test(tested_id, run='new', attempt=3, result='not_supported')
            recalled = [item['id'] for item in ledger.recall(task)]
            self.assertEqual(recalled[:2], [new_id, tested_id], 'open hypothesis first, then the refuted one')
            self.assertEqual(len(recalled), 6)


class ReflectionMemoryTest(unittest.TestCase):
    def test_markdown_is_the_memory_and_is_read_back(self):
        with tempfile.TemporaryDirectory() as folder:
            memory = ReflectionMemory(Path(folder), 'kiwi-01')
            task = 'Inspect the front and place the cube into the cup'
            title = memory.append(run='/runs/r1', task=task, outcome='budget_steps', reflection=REFLECTION,
                                  hypotheses=[{'id': 'H1', 'failure_type': 'release_misaligned', 'status': 'not_supported',
                                               'hypothesis': 'released too high'}])
            self.assertEqual(title, REFLECTION['title'])
            memory.append(run='/runs/r2', task='drive to the door', outcome='blocked',
                          reflection={**REFLECTION, 'title': 'Door task'})
            text = memory.path.read_text()
            self.assertTrue(text.startswith('# Episode reflections (kiwi-01)'))
            self.assertIn('**Regret.** I should have driven to 0.25 m', text)
            self.assertIn('- When the container is farther than 0.30 m', text)
            self.assertIn('H1 release_misaligned — not_supported', text)
            recalled = memory.recall('Inspect front of camera and place the cube into the cup')
            self.assertEqual([entry['title'] for entry in recalled], [REFLECTION['title']], 'only similar tasks')
            self.assertNotIn('<!-- reflection', recalled[0]['text'])
            self.assertIn('**Why it failed.**', recalled[0]['text'])
            # People may edit the file: an added note and a hand-written entry are read back too.
            memory.path.write_text(text.replace('**Regret.**', '**Regret.** (operator: the cup was also tilted)') +
                                   '\n## Hand note: place the cup on the mat first\n\nThe carpet makes the cup tip.\n')
            entries = memory.entries()
            self.assertIn('operator: the cup was also tilted', entries[0]['text'])
            self.assertEqual(entries[-1]['task'], 'Hand note: place the cup on the mat first')
            frozen = ReflectionMemory(Path(folder), 'kiwi-01', frozen=True)
            self.assertIsNone(frozen.append(run='r3', task=task, outcome='x', reflection=REFLECTION))


class LedgerHypothesisTest(unittest.TestCase):
    def test_review_fields_tests_and_markdown(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            review = {**review_answer(), 'failed_step': {'index': 3, 'tool': 'place', 'phase': None},
                      'agrees_with_host': True, 'host_type_hint': None, 'aspects_in_error': ['inspection']}
            record_id, status = ledger.record_attempt(task='put the cube in the cup', run='r1', attempt=1,
                                                      actions=ACTIONS, reason='beside the cup', review=review)
            record = ledger.records[0]
            self.assertEqual(record['approach'], 'place over the cup centre from z 0.14', 'review names the approach')
            self.assertEqual(record['hypothesis_status'], 'untested')
            view = ledger.view(record, compact=True)
            self.assertEqual(view['failure_type'], 'release_misaligned')
            self.assertEqual(view['aspects_in_error'], ['inspection'])
            self.assertEqual(view['failure_types'], {'release_misaligned': 1})
            self.assertEqual(view['next_try'], 'relocate the cup from 0.25 m, release at z+0.02')
            self.assertTrue(ledger.record_test(record_id, run='r2', attempt=1, result='not_supported', change='z+0.02'))
            self.assertFalse(ledger.record_test('F404', run='r2', attempt=1, result='supported'))
            self.assertEqual(ledger.view(record)['hypothesis_tests'], [{'result': 'not_supported', 'change': 'z+0.02'}])
            ledger.save()
            text = (Path(folder) / 'failure-ledger.md').read_text()
            self.assertIn('**Why (hypothesis, not_supported):** release_misaligned', text)
            self.assertIn('- inspection: **error** — took the cup footprint for the mouth', text)
            self.assertIn('fix: drive 5 cm forward', text)
            self.assertIn('Tested in r2 attempt 1: not_supported', text)
            frozen = FailureLedger(Path(folder), 'kiwi', frozen=True)
            self.assertFalse(frozen.record_test(record_id, run='r3', attempt=1, result='supported'))


if __name__ == '__main__':
    unittest.main()


class RecallPlanner:
    """Draft -> RECALL -> PLAN_REVISE, scripted."""
    def __init__(self, picks):
        self.contexts, self.picks = [], picks

    async def plan(self, context, images):
        self.contexts.append(context)
        revising = context['phase'] == 'PLAN_REVISE'
        return {'robot_id': 'kiwi', 'observation_id': 'obs', 'tool': 'plan', 'reason': 'scripted',
                'expected_outcome': 'cube in the cup', 'library_used': '', 'skills_applied': [],
                'hypothesis_tested': 'F001' if revising else None, 'hypothesis_change': '',
                'revised_because': 'step 2: F001 missed the mouth in depth -> wrist check before release' if revising else '',
                'plan': [{'step': 'pick the cube', 'skill': 'pick', 'success_check': 'held'},
                         {'step': 'release over the cup', 'skill': 'place', 'success_check': 'cube inside'}]}

    async def recall(self, context, images):
        self.contexts.append(context)
        return {'robot_id': 'kiwi', 'note': 'release failures apply to step 1', 'selected': self.picks}


class PlanRecallTest(unittest.IsolatedAsyncioTestCase):
    def runner_for(self, planner, folder, with_memory=True):
        ledger = FailureLedger(Path(folder), 'kiwi')
        reflections = ReflectionMemory(Path(folder), 'kiwi')
        if with_memory:
            ledger.record_attempt(task='a different wording: drop the block in the mug', run='r0', attempt=1,
                                  actions=ACTIONS, reason='beside', review={**review_answer(), 'failed_step': None})
            reflections.append(run='r0', task='drop the block in the mug', outcome='budget_steps', reflection=REFLECTION)
        runner, events = make_runner(planner, ledger)
        runner.reflections, runner.library, runner.experience, runner.recalled, runner.applied = reflections, None, None, [], set()

        async def fresh():
            return None

        async def decide(kind, model, context, *, soft=False, images=None):
            return model.model_validate(await getattr(planner, kind)(context, []))
        runner._fresh, runner._decide = fresh, decide
        return runner, events

    async def test_draft_then_plan_conditioned_recall_then_revision(self):
        planner = RecallPlanner([{'id': 'F001', 'step': 1, 'why': 'same release over a container failed in depth'},
                                 {'id': 'R1', 'step': None, 'why': 'lesson: relocate the container up close'},
                                 {'id': 'F999', 'step': 0, 'why': 'not a real id'}])
        with tempfile.TemporaryDirectory() as folder:
            runner, events = self.runner_for(planner, folder)
            plan = await runner.planning()
        self.assertEqual([c['phase'] for c in planner.contexts], ['PLANNING', 'RECALL', 'PLAN_REVISE'])
        recall_context, revise_context = planner.contexts[1:]
        self.assertNotIn('failure_ledger', planner.contexts[0]['memory'], 'the draft is written without long-term memory')
        self.assertEqual(recall_context['draft_plan']['plan'][1]['skill'], 'place')
        self.assertEqual([item['id'] for item in recall_context['catalog']['failure_ledger']], ['F001'],
                         'the whole catalog, not a similarity-filtered subset (task wording differs)')
        recalled = revise_context['memory']['recalled']
        self.assertEqual([item['id'] for item in recalled], ['F001', 'R1'], 'unknown ids are dropped')
        self.assertEqual(recalled[0]['step'], 1)
        self.assertIn('hypothesis', recalled[0]['content'])
        self.assertIn('**Regret.**', recalled[1]['content'])
        self.assertEqual(plan.hypothesis_tested, 'F001', 'a recalled ledger hypothesis can be tested')
        self.assertIn('step 2: F001', plan.revised_because)
        phases = [data.get('phase') for kind, data in events if kind == 'decision']
        self.assertEqual(phases, ['draft', 'recall', 'planning'])
        recall_event = next(data for kind, data in events if kind == 'decision' and data.get('phase') == 'recall')
        self.assertEqual(recall_event['unknown_ids'], ['F999'])

    async def test_without_memory_the_draft_is_the_plan(self):
        planner = RecallPlanner([])
        with tempfile.TemporaryDirectory() as folder:
            runner, _ = self.runner_for(planner, folder, with_memory=False)
            await runner.planning()
        self.assertEqual([c['phase'] for c in planner.contexts], ['PLANNING'])
