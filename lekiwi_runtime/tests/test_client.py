import unittest
from lekiwi_runtime import RobotClient


class FakeRobot:
    def __init__(self):
        self.executed = False

    def observe(self):
        return {"executed": self.executed}

    def execute(self, command):
        self.executed = command.get("action") == "wait"
        return self.executed


class ClientTest(unittest.TestCase):
    def test_completion_observation_and_rejection(self):
        client = RobotClient(FakeRobot(), "test", "inproc://unused")
        request = {"request_id": "1", "op": "execute", "action": "wait"}
        self.assertEqual(client.handle(request), {
            "request_id": "1", "status": "completed", "observation": {"executed": True},
        })
        self.assertEqual(client.handle({**request, "action": "unknown"})["status"], "rejected")
        self.assertEqual(client.handle([])["status"], "rejected")
        self.assertEqual(client.handle({**request, "op": "observation"})["status"], "rejected")

    def test_hardware_error_is_not_reported_as_completion(self):
        class BrokenRobot(FakeRobot):
            def observe(self):
                raise RuntimeError("sensor failure")

        client = RobotClient(BrokenRobot(), "test", "inproc://unused")
        response = client.handle({"request_id": "2", "op": "execute", "action": "wait"})
        self.assertEqual(response["status"], "error")
        self.assertEqual(response["error"], "sensor failure")


class TransportTest(unittest.TestCase):
    def test_dealer_registration_and_completion_reply(self):
        import json
        import subprocess
        import sys
        import tempfile
        import zmq

        with tempfile.TemporaryDirectory() as folder, zmq.Context() as context:
            endpoint = f"ipc://{folder}/router"
            with context.socket(zmq.ROUTER) as socket:
                socket.setsockopt(zmq.LINGER, 0)
                socket.bind(endpoint)
                source = (
                    "from lekiwi_runtime import RobotClient\n"
                    "class Robot:\n"
                    " def observe(self): return {'test': True}\n"
                    " def execute(self, command): return command['action'] == 'wait'\n"
                    f"RobotClient(Robot(), 'test', {endpoint!r}).run()\n"
                )
                process = subprocess.Popen([sys.executable, "-c", source])
                try:
                    self.assertTrue(socket.poll(3000))
                    identity, body = socket.recv_multipart()
                    self.assertEqual(identity, b"test")
                    self.assertEqual(json.loads(body), {"type": "hello"})
                    socket.send_multipart([identity, b'{"request_id":"1","op":"execute","action":"wait"}'])
                    self.assertTrue(socket.poll(3000))
                    identity, body = socket.recv_multipart()
                    self.assertEqual(json.loads(body), {
                        "request_id": "1", "status": "completed", "observation": {"test": True},
                    })
                finally:
                    process.terminate()
                    process.wait(timeout=3)
