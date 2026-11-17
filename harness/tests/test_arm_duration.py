"""Longer arm moves retain the short base-motion and wait limits."""
import json
import unittest
from unittest.mock import AsyncMock

from mcp.server.fastmcp.exceptions import ToolError
from mcp.server.fastmcp.tools.base import Tool

from harness.client import ServerClient
from harness.episode import Command
from harness.episode_runtime import planner_decision_schema
from harness.tools import RobotTools, build_command


def command(*, duration=3000, action='move', x=0., y=0., theta=0., joint='wrist_roll', cartesian=False):
    joints = {name: (30. if name == joint else None) for name in
              ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')}
    return {
        'action': action, 'x': x, 'y': y, 'theta': theta, 'duration_ms': duration,
        'joints': None if joint is None or cartesian else joints,
        'cartesian': ({'frame': 'arm_base', 'mode': 'absolute', 'xyz_m': [.2, 0., .05],
                       'approach_axis': None} if cartesian else None),
    }


class ArmDurationSchemaTest(unittest.TestCase):
    def test_arm_and_gripper_only_allow_up_to_twenty_seconds_without_duration_clamping(self):
        for duration in (100, 1000, 1001, 3000, 10000, 20000):
            for joint in ('wrist_roll', 'gripper'):
                with self.subTest(duration=duration, joint=joint):
                    value = Command.model_validate(command(duration=duration, joint=joint)).wire()
                    self.assertEqual(value['duration_ms'], duration)
                    self.assertEqual(value['joints'], {joint: 30.})
                    self.assertEqual(build_command('move', 0., 0., 0., duration, {joint: 30.}), value)

    def test_cartesian_accepts_longer_arm_only_duration_and_still_requires_resolution(self):
        for duration in (100, 1000, 3000, 10000, 20000):
            value = Command.model_validate(command(duration=duration, cartesian=True))
            self.assertEqual(value.request()['duration_ms'], duration)
            with self.assertRaisesRegex(ValueError, 'resolved by the host'):
                value.wire()
        schema = planner_decision_schema()['$defs']['Command']['properties']['duration_ms']
        self.assertEqual(schema["maximum"], 20000)

    def test_base_combined_wait_and_empty_move_retain_one_second_limit(self):
        cases = [command(joint=None), command(action='wait', joint=None)]
        for axis in ('x', 'y', 'theta'):
            cases.extend([command(**{axis: .1}, joint=None), command(**{axis: .1})])
        for payload in cases:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    Command.model_validate(payload)
                joints = ({k: v for k, v in payload['joints'].items() if v is not None}
                          if payload['joints'] else None)
                with self.assertRaises(ValueError):
                    build_command(payload['action'], payload['x'], payload['y'], payload['theta'], 3000, joints)
                accepted = Command.model_validate({**payload, 'duration_ms': 1000})
                self.assertEqual(accepted.wire()['duration_ms'], 1000)

    def test_no_duration_type_or_empty_target_bypass(self):
        for duration in (True, 3000., '3000', 0, 99, 20001):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                Command.model_validate(command(duration=duration))
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                build_command('move', 0., 0., 0., duration, {'wrist_roll': 30.})
        for joints in ({}, {'wrist_roll': None}, {'wrist_roll': True}, {'unknown': 0.}):
            with self.subTest(joints=joints), self.assertRaises(ValueError):
                build_command('move', 0., 0., 0., 3000, joints)
        payload = command()
        payload['joints'] = dict.fromkeys(payload['joints'])
        with self.assertRaises(ValueError):
            Command.model_validate(payload)
        for axis in ('x', 'y', 'theta'):
            with self.subTest(axis=axis), self.assertRaises(ValueError):
                Command.model_validate(command(cartesian=True, **{axis: .1}))

    def test_client_default_outlasts_thirty_five_second_broker(self):
        self.assertGreater(ServerClient("inproc://unused").timeout, 35.)
        self.assertEqual(ServerClient('inproc://unused', timeout=2.).timeout, 2.)


class ArmDurationToolTest(unittest.IsolatedAsyncioTestCase):
    async def test_mcp_accepts_guarded_long_arm_and_rejects_long_base_before_dispatch(self):
        client = AsyncMock()
        client.request.return_value = {'status': 'completed'}
        tool = Tool.from_function(RobotTools(client).execute, structured_output=False)
        self.assertEqual(tool.parameters['properties']["duration_ms"]["maximum"], 20000)
        fields = {'observation_id': 'observation', 'runtime_id': 'runtime',
                  'action_id': 'action', 'reason': 'Measured arm target with sufficient motion time'}
        content = await tool.run({'robot_id': 'test', 'joints': {'wrist_roll': 90.},
                                  'duration_ms': 3000, **fields})
        self.assertEqual(json.loads(content[0].text)['status'], 'completed')
        client.request.assert_awaited_once_with('test', 'execute', command={
            'action': 'move', 'x': 0, 'y': 0, 'theta': 0, 'duration_ms': 3000,
            'joints': {'wrist_roll': 90.}, **fields})
        client.request.reset_mock()
        for payload in ({'x': .1}, {'x': .1, 'joints': {'wrist_roll': 90.}}, {'action': 'wait'}, {}):
            with self.subTest(payload=payload), self.assertRaises(ToolError):
                await tool.run({'robot_id': 'test', 'duration_ms': 3000, **payload})
            client.request.assert_not_awaited()


if __name__ == '__main__':
    unittest.main()
