import asyncio
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
import zmq
import zmq.asyncio

from harness.client import ServerClient

ROOT = Path(__file__).resolve().parents[2]


class IntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_server_routing_and_mcp_tools(self):
        with tempfile.TemporaryDirectory() as folder:
            robots, api = f"ipc://{folder}/robots", f"ipc://{folder}/api"
            process = subprocess.Popen([
                "uv", "run", "--project", str(ROOT / "server"), "server",
                "--bind", robots, "--api", api, "--timeout", "0.2",
            ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                env={k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"})
            context = zmq.asyncio.Context()
            sockets = []
            tasks = []
            try:
                for identity in ("a", "b"):
                    socket = context.socket(zmq.DEALER)
                    socket.setsockopt(zmq.LINGER, 0)
                    socket.setsockopt(zmq.IDENTITY, identity.encode())
                    socket.connect(robots)
                    await socket.send_json({"type": "hello"})
                    sockets.append(socket)

                async def robot(socket, identity):
                    while True:
                        request = await socket.recv_json()
                        self.assertIsInstance(request["request_id"], str)
                        if request.get("action") == "timeout":
                            continue
                        status = "ok" if request["op"] == "observe" else "completed"
                        if request.get("action") == "bad":
                            status = "rejected"
                        await socket.send_json({"request_id": request["request_id"], "status": status,
                                                "observation": {"robot": identity, "cameras": {
                                                    "front": {"available": True, "encoding": "jpeg_base64", "data": "/9j/2Q=="}
                                                }}})

                tasks = [asyncio.create_task(robot(s, name)) for s, name in zip(sockets, ("a", "b"))]
                client = ServerClient(api, timeout=3)
                # Wait for asynchronous DEALER routing to become available.
                for _ in range(100):
                    result = await client.request("a", "observe")
                    if result["status"] == "ok":
                        break
                    await asyncio.sleep(.01)
                self.assertEqual(result["observation"]["robot"], "a")
                stalled, other = await asyncio.gather(
                    client.request("a", "execute", command={"action": "timeout"}),
                    client.request("b", "observe"),
                )
                self.assertEqual(stalled["status"], "error")
                self.assertEqual(other["observation"]["robot"], "b")
                rejected = await client.request("a", "execute", command={"op": "observe"})
                self.assertEqual(rejected["status"], "error")

                params = StdioServerParameters(command="uv", args=[
                    "run", "--project", str(ROOT / "harness"), "harness",
                    "--server", api,
                ])
                async with stdio_client(params) as (read, write):
                    async with ClientSession(read, write) as session:
                        await session.initialize()
                        listed = await session.list_tools()
                        self.assertEqual({t.name for t in listed.tools}, {"observe"})
                        result = await session.call_tool("observe", {"robot_id": "b"})
                        self.assertFalse(result.isError)
                        self.assertEqual(json.loads(result.content[0].text)["observation"]["robot"], "b")
                        self.assertEqual(result.content[2].type, "image")
                        self.assertIsNone(result.structuredContent)
                        self.assertNotIn("data", json.loads(result.content[0].text)["observation"]["cameras"]["front"])
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                for socket in sockets:
                    socket.close()
                context.term()
                process.terminate()
                try:
                    _, stderr = process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    _, stderr = process.communicate()
                if process.returncode not in (0, -15, 143):
                    self.fail(stderr.decode())
