import asyncio

import zmq
import zmq.asyncio


class ServerClient:
    def __init__(self, endpoint: str, timeout: float = 15):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.endpoint = endpoint
        self.timeout = timeout

    async def request(self, robot_id: str, op: str, **payload) -> dict:
        with zmq.asyncio.Context() as context, context.socket(zmq.REQ) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.connect(self.endpoint)
            async with asyncio.timeout(self.timeout):
                await socket.send_json({"robot_id": robot_id, "op": op, **payload})
                return await socket.recv_json()


class SnapshotClient:
    def __init__(self, path):
        self.path = path

    async def request(self, robot_id: str, op: str) -> dict:
        import json

        if op != "observe":
            raise ValueError("Snapshots only support observe")
        schema = self.path.with_name("schema.json")
        if schema.exists() and json.loads(schema.read_text())["identity"] != robot_id:
            raise ValueError("Snapshot robot_id does not match")
        response = json.loads(self.path.read_text())
        response["source"] = {"recorded": True, "path": str(self.path)}
        return response
