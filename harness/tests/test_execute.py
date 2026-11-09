import json
from pathlib import Path
import unittest
from unittest.mock import AsyncMock
from harness.client import SnapshotClient
from harness.tools import RobotTools


class ExecuteTest(unittest.IsolatedAsyncioTestCase):
    async def test_payload_and_post_motion_observation(self):
        client = AsyncMock()
        client.request.return_value = {'status': 'completed', 'observation': {'simulated': True}}
        content = await RobotTools(client).execute('kiwi', .2, 0, 30, 100)
        client.request.assert_awaited_once_with('kiwi', 'execute', command={
            'action': 'move', 'x': .2, 'y': 0, 'theta': 30, 'duration_ms': 100})
        self.assertTrue(json.loads(content[0].text)['observation']['simulated'])

    async def test_invalid_input_and_snapshot(self):
        client = AsyncMock()
        with self.assertRaises(ValueError):
            await RobotTools(client).execute('kiwi', .4, .4, 0, 100)
        client.request.assert_not_called()
        with self.assertRaises(ValueError):
            await RobotTools(SnapshotClient(Path('/unused'))).execute('kiwi', .1, 0, 0, 100)

    async def test_no_automatic_retry_on_error(self):
        client = AsyncMock()
        client.request.return_value = {'status': 'error', 'error': 'outcome unknown'}
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            await RobotTools(client).execute('kiwi', .1, 0, 0, 100)
        self.assertEqual(client.request.await_count, 1)

    async def test_arm_payload_and_rejection(self):
        client=AsyncMock()
        client.request.return_value={'status':'completed','observation':{'ego':{'last_motion':{'reached':True}}}}
        await RobotTools(client).execute('kiwi',action='move_arm',joints={'gripper':60},duration_ms=1000)
        client.request.assert_awaited_once_with('kiwi','execute',command={
            'action':'move_arm','joints':{'gripper':60},'duration_ms':1000})
        for joints in ({'unknown':0},{'gripper':200},{'wrist_roll':True}):
            with self.assertRaises(ValueError):
                await RobotTools(client).execute('kiwi',action='move_arm',joints=joints)
        with self.assertRaises(ValueError):
            await RobotTools(client).execute('kiwi',x=.1,action='move_arm',joints={'gripper':60})
