"""Failure review → hypothesis → test: host checks around the REVIEW turn and the ledger lifecycle."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from harness.memory import FailureLedger
from harness.plan_loop import PlanActRunner


def review_answer(**changes):
    answer = {'robot_id': 'kiwi', 'failed_step_index': 3, 'failure_level': 'execution', 'failure_type': 'release_misaligned',
              'confidence': 'medium', 'step_checks': [{'index': 2, 'expected': 'cube held', 'observed': 'wrist shows cube',
                                                       'met': 'yes'}],
              'hypothesis': 'released 4 cm above the rim at the located centre; the cup centre was 2 cm off',
              'evidence': 'AFTER step 3 place: cube beside the cup', 'next_try': 'relocate the cup from 0.25 m, release at z+0.02',
              'prediction': 'cube inside the cup on the wrist image', 'approach': 'place over the cup centre from z 0.14',
              'same_as_ledger': None}
    answer.update(changes)
    return answer


class ScriptedReviewer:
    def __init__(self, **changes):
        self.calls, self.changes = [], changes

    async def review(self, context, images):
        self.calls.append((context, images))
        return review_answer(**self.changes)


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

    async def test_review_after_stop_runs_once_and_never_for_infrastructure_stops(self):
        planner = ScriptedReviewer()
        runner, _ = make_runner(planner)
        runner.current = attempt(list(ACTIONS))
        await runner._review_after_stop('uncertain_execution', 'transport')
        self.assertEqual(planner.calls, [])
        await runner._review_after_stop('budget_steps', '')
        await runner._review_after_stop('budget_steps', '')
        self.assertEqual(len(planner.calls), 1)
        self.assertEqual(planner.calls[0][0]['ending']['verdict'], 'stopped')
        self.assertTrue(runner.current['review']['after_stop'])

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


class LedgerHypothesisTest(unittest.TestCase):
    def test_review_fields_tests_and_markdown(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            review = {**review_answer(), 'failed_step': {'index': 3, 'tool': 'place', 'phase': None},
                      'agrees_with_host': True, 'host_type_hint': None}
            record_id, status = ledger.record_attempt(task='put the cube in the cup', run='r1', attempt=1,
                                                      actions=ACTIONS, reason='beside the cup', review=review)
            record = ledger.records[0]
            self.assertEqual(record['approach'], 'place over the cup centre from z 0.14', 'review names the approach')
            self.assertEqual(record['hypothesis_status'], 'untested')
            view = ledger.view(record, compact=True)
            self.assertEqual(view['failure_type'], 'release_misaligned')
            self.assertEqual(view['failure_types'], {'release_misaligned': 1})
            self.assertEqual(view['next_try'], 'relocate the cup from 0.25 m, release at z+0.02')
            self.assertTrue(ledger.record_test(record_id, run='r2', attempt=1, result='not_supported', change='z+0.02'))
            self.assertFalse(ledger.record_test('F404', run='r2', attempt=1, result='supported'))
            self.assertEqual(ledger.view(record)['hypothesis_tests'], [{'result': 'not_supported', 'change': 'z+0.02'}])
            ledger.save()
            text = (Path(folder) / 'failure-ledger.md').read_text()
            self.assertIn('**Why (hypothesis, not_supported):** release_misaligned', text)
            self.assertIn('| 2 | cube held | wrist shows cube | yes |', text)
            self.assertIn('Tested in r2 attempt 1: not_supported', text)
            frozen = FailureLedger(Path(folder), 'kiwi', frozen=True)
            self.assertFalse(frozen.record_test(record_id, run='r3', attempt=1, result='supported'))


if __name__ == '__main__':
    unittest.main()
