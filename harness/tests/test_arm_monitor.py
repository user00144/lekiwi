"""Host side of the runtime in-motion monitor: wire command, stop acceptance, skills and ledger."""
from pathlib import Path
import tempfile
import time
import unittest

from harness.episode import Command, EpisodeLimits, EpisodeRunner, EpisodeStopped
from harness.episode_runtime import planner_decision_schema
from harness.memory import FailureLedger
from harness.skills import SkillFailed, Skills
from harness.tools import RobotTools, build_command

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
CONFIG = {
    'version': 1, 'calibration_id': None,
    'home': {'joints_deg': dict.fromkeys(JOINTS[:5], 0.), 'xyz_m': [.15, 0., .0], 'approach_axis': [.45, 0., -.89]},
    'grasp': {'approach_axis': [-.117, .008, -.993], 'reference_xyz_m': [.25, 0, -.09],
              'floor_grasp_z_m': -.09, 'hover_m': .04},
    'gripper': {'open_percent': 54.6, 'close_percent': 26., 'duration_ms': 1500, 'held_margin_percent': 1.},
    'motion': {'speed_margin': 1.4, 'min_duration_ms': 2000, 'max_duration_ms': 20000},
}


class MonitorTools:
    """Guarded runtime fake: a move with a gripper threshold stops half-way with a verified hold."""

    def __init__(self, gripper=27.7, monitor_capability=True):
        self.sequence, self.calls, self.mutate = 0, [], None
        self.joints = dict.fromkeys(JOINTS, 0.)
        self.joints['gripper'] = gripper
        self.capabilities = {'guarded_execution': 1, 'arm_reconciliation': 1, 'arm_motion_warning': 1}
        if monitor_capability:
            self.capabilities['arm_motion_monitor'] = 1

    def packet(self):
        self.sequence += 1
        oid, now = f'obs-{self.sequence}', time.time()
        return {'status': 'ok', 'capabilities': dict(self.capabilities),
                'action_context': {'observation_id': oid, 'runtime_id': 'runtime', 'movement_allowed': True,
                                   'max_age_s': 60},
                'observation': {'observation_id': oid, 'runtime_id': 'runtime',
                                'ego': {'available': True, 'error': None, 'timestamp': now, 'raw': {},
                                        'joints': dict(self.joints), 'calibration_id': 'cal-1',
                                        'calibration_status': {'valid': True}},
                                'cameras': {name: {'available': True, 'encoding': 'jpeg_base64', 'data': '/9j/2Q==',
                                                   'timestamp': now} for name in ('front', 'wrist')}}}

    async def observe(self, robot_id):
        return RobotTools._content(self.packet())

    async def execute(self, robot_id, **args):
        self.calls.append(args)
        targets = args.get('joints') or {}
        threshold = (args.get('monitor') or {}).get('gripper_min_percent')
        telemetry = {'gripper_min_seen': None, 'max_following_error_deg': {'wrist_roll': 1.4}}
        if threshold is None:
            self.joints.update(targets)
            packet = self.packet()
            packet.update(status='completed', execution={
                'action_id': args['action_id'], 'outcome': 'completed', 'command_sent': True, 'arm_reached': True,
                'motion_finished_at': time.time() - .01, 'base_stop_command_sent': None, 'base_stop_verified': None,
                'arm_motion': {'reached': True, 'monitor': telemetry}})
            return RobotTools._content(packet)
        for name, target in targets.items():
            self.joints[name] = (self.joints[name] + target) / 2
        self.joints['gripper'] = 26.2
        stop = {'condition': 'gripper_below_min', 'gripper_percent': 26.2, 'gripper_min_percent': threshold,
                'consecutive_samples': 2, 'elapsed_s': .96, 'step': 48, 'steps': 100, 'progress_fraction': .48}
        hold = {'verified': True, 'calibration_id': 'cal-1', 'timestamp': time.time() - .02,
                'measured': dict(self.joints), 'held_goal_raw': dict.fromkeys(JOINTS, 2048)}
        finished = time.time() - .01
        packet = self.packet()
        packet.update(status='stopped', execution={
            'action_id': args['action_id'], 'outcome': 'stopped', 'command_sent': True, 'arm_reached': False,
            'arm_hold_verified': True, 'motion_finished_at': finished, 'monitor_stop': stop,
            'base_stop_command_sent': None, 'base_stop_verified': None,
            'arm_motion': {'targets': dict(targets), 'reached': False, 'outcome': 'monitor_stop',
                           'monitor_stop': dict(stop), 'hold': hold, 'monitor': telemetry}})
        if self.mutate:
            self.mutate(packet)
        return RobotTools._content(packet)


class WireTest(unittest.TestCase):
    def test_monitor_only_on_arm_only_moves_without_gripper(self):
        command = build_command('move', 0, 0, 0, 2000, {'wrist_roll': 5.}, {'gripper_min_percent': 27.})
        self.assertEqual(command['monitor'], {'gripper_min_percent': 27.})
        self.assertEqual(build_command('move', 0, 0, 0, 2000, {'wrist_roll': 5.}, {})['monitor'],
                         {'gripper_min_percent': None})
        self.assertNotIn('monitor', build_command('move', 0, 0, 0, 2000, {'wrist_roll': 5.}))
        for args in (('move', 0, 0, 0, 2000, {'gripper': 30.}, {'gripper_min_percent': 27.}),
                     ('move', .1, 0, 0, 500, {'wrist_roll': 5.}, {'gripper_min_percent': 27.}),
                     ('move', .1, 0, 0, 500, None, {'gripper_min_percent': 27.}),
                     ('wait', 0, 0, 0, 500, None, {}),
                     ('move', 0, 0, 0, 2000, {'wrist_roll': 5.}, {'gripper_min_percent': 0}),
                     ('move', 0, 0, 0, 2000, {'wrist_roll': 5.}, {'gripper_min_percent': True}),
                     ('move', 0, 0, 0, 2000, {'wrist_roll': 5.}, {'other': 1.})):
            with self.subTest(args=args), self.assertRaises(ValueError):
                build_command(*args)

    def test_cartesian_request_carries_monitor_and_model_schema_hides_it(self):
        command = Command.model_validate({
            'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 2000, 'joints': None,
            'cartesian': {'frame': 'arm_base', 'mode': 'absolute', 'xyz_m': [.2, 0., 0.], 'approach_axis': None},
            'monitor': {'gripper_min_percent': 27.}})
        self.assertEqual(command.request()['monitor'], {'gripper_min_percent': 27.})
        schema = planner_decision_schema()
        self.assertNotIn('monitor', schema['$defs']['Command']['properties'])
        self.assertNotIn('ArmMonitor', schema['$defs'])


class MonitorStopTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()

    async def asyncTearDown(self):
        self.temp.cleanup()

    async def skills(self, tools):
        self.run_dir = Path(tempfile.mkdtemp(dir=self.temp.name)) / 'run'
        runner = EpisodeRunner(None, tools, 'kiwi', 'put the cube in the cup', self.run_dir,
                               EpisodeLimits(max_actions=6, max_steps=10, max_seconds=60))
        await runner.observe()
        skills = Skills(runner, CONFIG, reason='monitor test')
        runner.cartesian = type('FK', (), {'evaluate': staticmethod(lambda packet: {'current_xyz_m': [.2, 0., .05]})})()
        return runner, skills

    def carry(self, skills, monitor):
        command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': 2000,
                   'joints': {name: 10. if name == 'wrist_roll' else None for name in JOINTS}, 'cartesian': None}
        if monitor is not None:
            command['monitor'] = monitor
        return skills.send(command, 'place hover')

    async def test_monitor_only_while_holding_and_only_when_the_runtime_supports_it(self):
        _, skills = await self.skills(MonitorTools(gripper=27.7))
        self.assertEqual(skills.monitor_for_move(), {'gripper_min_percent': 27.})
        _, skills = await self.skills(MonitorTools(gripper=26.2))
        self.assertEqual(skills.monitor_for_move(), {'gripper_min_percent': None})
        _, skills = await self.skills(MonitorTools(monitor_capability=False))
        self.assertIsNone(skills.monitor_for_move())
        runner, skills = await self.skills(MonitorTools())
        runner.recovery_constraints = {'max_joint_delta_deg': 3.}
        self.assertIsNone(skills.monitor_for_move())

    async def test_runtime_stop_becomes_a_skill_failure_with_where_it_broke(self):
        tools = MonitorTools()
        runner, skills = await self.skills(tools)
        with self.assertRaises(SkillFailed) as raised:
            await self.carry(skills, skills.monitor_for_move())
        detail = raised.exception.detail
        self.assertEqual(detail['failed_step'], 'place hover')
        self.assertEqual(detail['monitor_stop']['progress_fraction'], .48)
        self.assertIn('no longer between the jaws', str(raised.exception))
        self.assertEqual(tools.calls[0]['monitor'], {'gripper_min_percent': 27.})
        self.assertEqual(runner.pending_feedback['monitor_stop']['condition'], 'gripper_below_min')
        self.assertIn('do not assume it is still held', runner.pending_feedback['guidance'])
        self.assertEqual(runner.packet['observation']['ego']['joints']['gripper'], 26.2)  # fresh post-stop state
        events = (self.run_dir / 'events.jsonl').read_text()
        self.assertIn('"monitor_stop"', events)

    async def test_telemetry_only_move_completes(self):
        _, skills = await self.skills(MonitorTools(gripper=26.2))
        entry = await self.carry(skills, skills.monitor_for_move())
        self.assertEqual(entry['status'], 'completed')
        self.assertEqual(entry['monitor']['max_following_error_deg'], {'wrist_roll': 1.4})

    async def test_unverifiable_stop_is_an_uncertain_execution(self):
        for mutate in (lambda p: p['capabilities'].pop('arm_motion_monitor'),
                       lambda p: p['execution']['arm_motion']['hold'].update(verified=False),
                       lambda p: p['execution']['monitor_stop'].update(condition='made_up'),
                       lambda p: p['observation']['ego']['joints'].update(wrist_roll=9.)):
            with self.subTest(mutate=mutate):
                tools = MonitorTools()
                tools.mutate = mutate
                _, skills = await self.skills(tools)
                with self.assertRaises(EpisodeStopped) as raised:
                    await self.carry(skills, skills.monitor_for_move())
                self.assertIn(raised.exception.reason, ('uncertain_execution', 'post_motion_observation_failed'))


class LedgerTest(unittest.TestCase):
    def test_in_motion_stop_is_recorded_with_the_failed_step(self):
        with tempfile.TemporaryDirectory() as folder:
            ledger = FailureLedger(Path(folder), 'kiwi')
            stop = {'condition': 'gripper_below_min', 'progress_fraction': .48, 'elapsed_s': .96,
                    'gripper_percent': 26.2, 'gripper_min_percent': 27.}
            actions = [{'tool': 'pick', 'args': {'xyz_m': [.25, 0, -.09]}, 'status': 'ok', 'result': {}},
                       {'tool': 'place', 'args': {'xyz_m': [.22, .05, .02]}, 'status': 'failed',
                        'error': 'place hover: stopped mid-motion by the in-motion monitor',
                        'result': {'failed_step': 'place hover', 'monitor_stop': stop}}]
            record_id, status = ledger.record_attempt(task='put the cube in the cup', run='run-1', attempt=1,
                                                      actions=actions, reason='cube dropped while carrying')
            view = ledger.view(next(r for r in ledger.records if r['id'] == record_id))
            self.assertEqual(status, 'tried')
            self.assertIn("place object_slipped at step 'place hover'", view['failure'])
            self.assertIn('stopped the arm in motion at 48%', view['failure'])
            ledger.save()
            self.assertIn('in motion at 48%', (Path(folder) / 'failure-ledger.md').read_text())


if __name__ == '__main__':
    unittest.main()
