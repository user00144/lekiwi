"""XYZ decisions use the same guarded action loop, never a second motor path."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from harness.episode import Command, EpisodeLimits, EpisodeRunner
from harness.episode_runtime import planner_decision_schema
from harness.kinematics import ARM_JOINTS, CartesianController
from harness.tools import RobotTools


CALIBRATION_ID = 'a' * 64


def configuration(verified=True):
    return {'version': 1, 'hardware_alignment_verified': verified,
            'calibration_id': CALIBRATION_ID if verified else None,
            'model_hash': CartesianController().geometry.model_hash if verified else None,
            'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS}}


def decide(context, tool='blocked', *, cartesian=None, joints=None):
    return {'robot_id': context['robot_id'],
            'observation_id': context['current']['observation']['observation_id'],
            'tool': tool, 'plan': ['Inspect the current pose', 'Move one bounded endpoint'],
            'reason': 'The current scene supports this bounded step.',
            'expected': 'The gripper endpoint moves toward the specified arm-base target.',
            'feedback': 'progress', 'evidence': 'Current front and wrist views show the requested state.',
            'command': ({'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 1000,
                         'joints': joints, 'cartesian': cartesian} if tool == 'execute' else None)}


def xyz(target, mode='absolute'):
    return {'frame': 'arm_base', 'mode': mode, 'xyz_m': target, 'approach_axis': None}


class Planner:
    def __init__(self, *choices):
        self.choices, self.contexts = iter(choices), []

    async def decide(self, context, images):
        self.contexts.append(deepcopy(context))
        choice = next(self.choices, 'blocked')
        return choice(context) if callable(choice) else decide(context, choice)


class ArmTools:
    def __init__(self):
        self.joints = {name: 0. for name in ARM_JOINTS}
        self.joints['gripper'] = 50.
        self.sequence, self.calls = 0, []
        self.max_age_s, self.calibration_id = 60., CALIBRATION_ID
        self.uncertain = False

    def packet(self):
        self.sequence += 1
        oid = f'obs-{self.sequence}'
        return {'status': 'ok', 'capabilities': {'guarded_execution': 1},
                'action_context': {'observation_id': oid, 'runtime_id': 'runtime',
                                   'movement_allowed': True, 'max_age_s': self.max_age_s},
                'observation': {'observation_id': oid, 'runtime_id': 'runtime',
                                'ego': {'available': True, 'joints': dict(self.joints),
                                        'calibration_id': self.calibration_id,
                                        'calibration_status': {'valid': True},
                                        'limits': {name: {'min': -90., 'max': 90., 'max_speed': 30., 'unit': 'deg'}
                                                   for name in ARM_JOINTS}},
                                'cameras': {name: {'available': True, 'encoding': 'jpeg_base64',
                                                  'data': '/9j/2Q==', 'timestamp': 2.}
                                            for name in ('front', 'wrist')}}}

    async def observe(self, robot_id):
        return RobotTools._content(self.packet())

    async def execute(self, robot_id, **command):
        self.calls.append({'robot_id': robot_id, **command})
        packet = self.packet()
        if not self.uncertain:
            self.joints.update(command.get('joints') or {})
            packet['observation']['ego']['joints'] = dict(self.joints)
        packet.update(status='error' if self.uncertain else 'completed', execution={
            'action_id': command['action_id'],
            'outcome': 'uncertain_execution' if self.uncertain else 'completed',
            'command_sent': None if self.uncertain else True,
            'arm_reached': None if self.uncertain else True, 'motion_finished_at': 1.,
        })
        return RobotTools._content(packet)


class CountingController(CartesianController):
    def __init__(self, config=None):
        super().__init__(config)
        self.solves = []

    def solve(self, packet, request, duration_ms):
        self.solves.append((deepcopy(packet), deepcopy(request), duration_ms))
        return super().solve(packet, request, duration_ms)


class CartesianEpisodeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'run'
        self.tools = ArmTools()

    async def run_case(self, planner, controller, *, limits=None, clock=None):
        options = {'clock': clock} if clock is not None else {}
        runner = EpisodeRunner(planner, self.tools, 'kiwi', 'Move the gripper endpoint', self.path,
                               limits=limits, cartesian=controller, **options)
        return await runner.run()

    def events(self):
        return [json.loads(line) for line in (self.path / 'events.jsonl').read_text().splitlines()]

    async def test_three_second_cartesian_move_uses_same_guarded_loop_and_budget(self):
        controller = CountingController(configuration())
        target_joints = {name: 60. if name == 'shoulder_pan' else 0. for name in ARM_JOINTS}
        target = controller.geometry.forward(target_joints)['xyz_m']

        def longer(context):
            value = decide(context, 'execute', cartesian=xyz(target))
            value['command']['duration_ms'] = 3000
            return value

        result = await self.run_case(Planner(longer, 'task_complete', 'task_complete'), controller)
        self.assertEqual(result['status'], 'completion_claim')
        self.assertEqual(len(self.tools.calls), 1)
        self.assertEqual(self.tools.calls[0]['duration_ms'], 3000)
        self.assertEqual(controller.solves[0][2], 3000)
        self.assertNotIn('cartesian', self.tools.calls[0])
        self.assertEqual(set(self.tools.calls[0]['joints']), set(ARM_JOINTS))
        self.assertTrue(self.tools.calls[0]['action_id'])
        self.assertTrue(self.tools.calls[0]['observation_id'])

    async def test_longer_arm_duration_still_requires_remaining_episode_budget(self):
        def longer(context):
            value = decide(context, 'execute', cartesian=xyz([0., 0., 0.], 'relative'))
            value['command']['duration_ms'] = 3000
            return value

        result = await self.run_case(Planner(longer), CountingController(configuration()),
                                    limits=EpisodeLimits(max_seconds=2.))
        self.assertEqual(result['status'], 'budget_seconds')
        self.assertEqual(self.tools.calls, [])

    async def test_real_ik_endpoint_sends_only_guarded_joint_command_then_reobserves(self):
        controller = CountingController(configuration())
        pose = {name: 4. if name == 'shoulder_pan' else 0. for name in ARM_JOINTS}
        target = controller.geometry.forward(pose)['xyz_m']
        planner = Planner(lambda c: decide(c, 'execute', cartesian=xyz(target)),
                          'task_complete', 'task_complete')
        result = await self.run_case(planner, controller)
        self.assertEqual(result['status'], 'completion_claim')
        self.assertEqual(len(self.tools.calls), 1)
        sent = self.tools.calls[0]
        self.assertEqual(set(sent['joints']), set(ARM_JOINTS))
        self.assertNotIn('cartesian', sent)
        self.assertEqual((sent['x'], sent['y'], sent['theta']), (0., 0., 0.))
        self.assertEqual(sent['observation_id'], planner.contexts[0]['current']['observation']['observation_id'])
        self.assertEqual(self.tools.joints['gripper'], 50.)
        solved = next(event['data'] for event in self.events() if event['event'] == 'ik_result')
        self.assertTrue(solved['success'] and solved['executable'])
        self.assertEqual(solved['joints'], sent['joints'])
        self.assertLessEqual(solved['residual_m'], .002)
        self.assertEqual(planner.contexts[1]['previous_ik_result']['observation_id'], sent['observation_id'])
        self.assertIn('pending_motion_feedback', planner.contexts[1])
        states = [event['data'] for event in self.events() if event['event'] == 'kinematics_state']
        self.assertEqual(len(states), 3)
        self.assertTrue(states[-1]['executable'])
        self.assertNotEqual(states[0]['observation_id'], states[-1]['observation_id'])
        self.assertEqual(planner.contexts[1]['cartesian']['current_xyz_m'], states[1]['current_xyz_m'])

    async def test_relative_request_is_based_on_latest_measured_fk(self):
        controller = CountingController(configuration())
        self.tools.joints['shoulder_pan'] = 12.
        planner = Planner(lambda c: decide(c, 'execute', cartesian=xyz([0., 0., 0.], 'relative')), 'blocked')
        await self.run_case(planner, controller)
        solved = next(event['data'] for event in self.events() if event['event'] == 'ik_result')
        self.assertEqual(solved['target_xyz_m'], planner.contexts[0]['cartesian']['current_xyz_m'])
        self.assertEqual(controller.solves[0][0]['observation']['ego']['joints']['shoulder_pan'], 12.)
        self.assertEqual(len(self.tools.calls), 1)

    async def test_unverified_numeric_preview_never_dispatches_and_repeated_request_is_bounded(self):
        controller = CountingController()
        request = lambda c: decide(c, 'execute', cartesian=xyz([0., 0., 0.], 'relative'))
        planner = Planner(request, request, request)
        result = await self.run_case(planner, controller)
        self.assertEqual(result['status'], 'repeated_failed_action')
        self.assertEqual(self.tools.calls, [])
        self.assertEqual(len(controller.solves), 1)
        self.assertEqual(result['action_attempts'], 0)
        solved = next(event['data'] for event in self.events() if event['event'] == 'ik_result')
        self.assertTrue(solved['success'])
        self.assertFalse(solved['executable'])
        self.assertIsNone(solved['joints'])
        self.assertFalse(planner.contexts[1]['rejection_feedback']['command_sent'])

    async def test_calibration_identity_mismatch_never_dispatches(self):
        controller = CountingController(configuration())
        self.tools.calibration_id = 'b' * 64
        planner = Planner(lambda c: decide(c, 'execute', cartesian=xyz([0., 0., 0.], 'relative')), 'blocked')
        result = await self.run_case(planner, controller)
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(self.tools.calls, [])
        self.assertIn('calibration_id_mismatch', planner.contexts[1]['previous_ik_result']['reason_codes'])

    async def test_solver_failure_can_reobserve_and_choose_ordinary_joint_action(self):
        controller = CountingController(configuration())
        def fail(*args):
            raise ValueError('No feasible endpoint within this step')
        controller.solve = fail
        joints = {name: 1. if name == 'wrist_roll' else None for name in (*ARM_JOINTS, 'gripper')}
        planner = Planner(lambda c: decide(c, 'execute', cartesian=xyz([0., 0., .02], 'relative')),
                          lambda c: decide(c, 'execute', joints=joints), 'task_complete', 'task_complete')
        result = await self.run_case(planner, controller)
        self.assertEqual(result['status'], 'completion_claim')
        self.assertEqual(len(self.tools.calls), 1)
        self.assertEqual(self.tools.calls[0]['joints'], {'wrist_roll': 1.})
        self.assertEqual(planner.contexts[1]['rejection_feedback']['error_code'], 'ik_error')
        self.assertNotEqual(planner.contexts[0]['current']['observation']['observation_id'],
                            planner.contexts[1]['current']['observation']['observation_id'])

    async def test_unknown_execution_after_xyz_never_retries(self):
        self.tools.uncertain = True
        controller = CountingController(configuration())
        request = lambda c: decide(c, 'execute', cartesian=xyz([0., 0., 0.], 'relative'))
        planner = Planner(request, request)
        result = await self.run_case(planner, controller)
        self.assertEqual(result['status'], 'uncertain_execution')
        self.assertEqual(len(controller.solves), 1)
        self.assertEqual(len(self.tools.calls), 1)
        self.assertEqual(result['pending_action_id'], self.tools.calls[0]['action_id'])

    async def test_expired_observation_after_solver_is_not_dispatched(self):
        now = [0.]
        self.tools.max_age_s = .1
        controller = CountingController(configuration())
        solve = controller.solve
        def slow(packet, request, duration):
            result = solve(packet, request, duration)
            now[0] = .2
            return result
        controller.solve = slow
        planner = Planner(lambda c: decide(c, 'execute', cartesian=xyz([0., 0., 0.], 'relative')), 'blocked')
        result = await self.run_case(planner, controller, clock=lambda: now[0])
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(self.tools.calls, [])
        self.assertEqual(result['observations'], 2)
        self.assertFalse(any(event['event'] == 'action_prepared' for event in self.events()))


class CartesianSchemaTest(unittest.TestCase):
    def test_legacy_joint_command_remains_valid_but_planner_requires_explicit_mode(self):
        legacy = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 500, 'joints': None}
        self.assertIsNone(Command.model_validate(legacy).cartesian)
        schema = planner_decision_schema()['$defs']['Command']
        self.assertIn('cartesian', schema['required'])
        self.assertNotIn('default', schema['properties']['cartesian'])

    def test_cartesian_cannot_mix_base_joints_gripper_or_wait(self):
        base = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 500,
                'joints': None, 'cartesian': xyz([.1, .2, .3])}
        joints = {name: None for name in (*ARM_JOINTS, 'gripper')}
        for change in ({'action': 'wait'}, {'x': .1}, {'theta': 1.}, {'joints': joints}, {'duration_ms': 50}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                Command.model_validate({**base, **change})
        with self.assertRaises(ValueError):
            Command.model_validate(base).wire()
        for target in ([1., 2.], [True, 0., 0.], [float('nan'), 0., 0.]):
            with self.subTest(target=target), self.assertRaises(ValueError):
                Command.model_validate({**base, 'cartesian': xyz(target)})
