"""Broker timing with a simulated transport only; no robot process is contacted."""
import asyncio
import json
import unittest
from unittest.mock import AsyncMock

from server.router import RobotServer


class ResponseTimeoutTest(unittest.IsolatedAsyncioTestCase):
    async def test_twenty_second_arm_with_feedback_is_not_timed_out(self):
        server = RobotServer()
        # A fake socket returns the correlated response after the old
        # 20 s deadline. This includes a little post-trajectory feedback latency.
        timers = []

        async def send(frames, **kwargs):
            request = json.loads(frames[1])
            self.assertEqual(request['duration_ms'], 20000)
            future = server._pending[(frames[0], request['request_id'])]
            timers.append(asyncio.get_running_loop().call_later(
                20.05, future.set_result, {'request_id': request['request_id'], 'status': 'completed'}))

        server._socket = AsyncMock()
        server._socket.send_multipart.side_effect = send
        try:
            response = await server.execute('mock', {'action': 'move', 'duration_ms': 20000,
                                                     'joints': {'wrist_roll': 30.}})
            self.assertEqual(response['status'], 'completed')
            server._socket.send_multipart.assert_awaited_once()
            self.assertEqual(server._pending, {})
            self.assertFalse(server._locks['mock'].locked())
        finally:
            for timer in timers:
                timer.cancel()

    async def test_explicit_short_timeout_cleans_up_without_retry(self):
        server = RobotServer(timeout=.01)
        server._socket = AsyncMock()
        with self.assertRaises(TimeoutError):
            await server.execute('mock', {'action': 'move', 'duration_ms': 10000,
                                          'joints': {'wrist_roll': 30.}})
        server._socket.send_multipart.assert_awaited_once()
        self.assertEqual(server._pending, {})
        self.assertFalse(server._locks['mock'].locked())


if __name__ == '__main__':
    unittest.main()
