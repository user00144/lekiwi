import sys

import asyncio

import zmq
import zmq.asyncio

from .router import RobotServer


class ServerService:
    """Local request interface for callers such as the MCP harness."""

    def __init__(self, robot_server: RobotServer, endpoint: str):
        self.robot_server = robot_server
        self.endpoint = endpoint

    async def _reply(self, socket, address: list[bytes], body: bytes):
        import json

        try:
            request = json.loads(body)
            robot_id = request["robot_id"]
            if not isinstance(robot_id, str) or not robot_id:
                raise ValueError("robot_id must be a nonempty string")
            if request["op"] == "observe":
                response = await self.robot_server.observe(robot_id)
            elif request["op"] == "execute":
                response = await self.robot_server.execute(robot_id, request["command"])
            else:
                raise ValueError("Unknown operation")
        except TimeoutError:
            response = {"status": "error", "error": "Robot timeout; execution outcome unknown. Do not retry blindly."}
        except Exception as exc:
            print(f'API request failed: {exc}', file=sys.stderr, flush=True)
            response = {"status": "error", "error": str(exc)}
        await socket.send_multipart([*address, json.dumps(response).encode()])

    async def run(self):
        with zmq.asyncio.Context() as context, context.socket(zmq.ROUTER) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.bind(self.endpoint)
            print(f'Request API listening endpoint={self.endpoint}', file=sys.stderr, flush=True)
            tasks = set()
            try:
                while True:
                    frames = await socket.recv_multipart()
                    if len(frames) != 3 or frames[1] != b"":
                        continue
                    task = asyncio.create_task(self._reply(socket, frames[:2], frames[2]))
                    tasks.add(task)
                    task.add_done_callback(tasks.discard)
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
