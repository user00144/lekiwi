"""Strict read-only reconciliation routing with fake transport objects."""
import json
import unittest
from unittest.mock import AsyncMock

from server.router import RobotServer
from server.service import ServerService


class ReconcileRoutingTest(unittest.IsolatedAsyncioTestCase):
    async def test_router_forwards_only_the_original_failure_identity(self):
        server = RobotServer()
        server._request = AsyncMock(return_value={"status": "ok"})
        self.assertEqual(await server.reconcile("robot", "runtime", "failed"), {"status": "ok"})
        server._request.assert_awaited_once_with(
            "robot", {"op": "reconcile", "runtime_id": "runtime", "failed_action_id": "failed"})

    async def test_invalid_identity_never_reaches_transport(self):
        server = RobotServer()
        server._request = AsyncMock()
        for runtime, action in (("", "a"), (None, "a"), ("r", " " ), ("r", "a" * 129)):
            with self.subTest(runtime=runtime, action=action):
                with self.assertRaises(ValueError):
                    await server.reconcile("robot", runtime, action)
        server._request.assert_not_awaited()

    async def test_service_routes_read_only_identity_and_rejects_motion_fields(self):
        server, socket = AsyncMock(), AsyncMock()
        server.reconcile.return_value = {"status": "ok", "recovery": {"reconciled": True}}
        service = ServerService(server, "inproc://unused")
        request = {"op": "reconcile", "robot_id": "robot", "runtime_id": "runtime", "failed_action_id": "failed"}
        await service._reply(socket, [b"caller", b""], json.dumps(request).encode())
        server.reconcile.assert_awaited_once_with("robot", "runtime", "failed")
        self.assertEqual(json.loads(socket.send_multipart.call_args.args[0][-1]), server.reconcile.return_value)
        server.reconcile.reset_mock()
        for extra in ({"command": {}}, {"joints": {"wrist_roll": 0}}, {"force": True}):
            await service._reply(socket, [b"caller", b""], json.dumps({**request, **extra}).encode())
            response = json.loads(socket.send_multipart.call_args.args[0][-1])
            self.assertEqual(response["status"], "error")
        server.reconcile.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
