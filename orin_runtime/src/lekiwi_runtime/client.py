import zmq

from .robot import Robot


class RobotClient:
    def __init__(self, robot: Robot, robot_id: str, endpoint: str):
        self.robot = robot
        self.robot_id = robot_id
        self.endpoint = endpoint

    def handle(self, request: dict) -> dict:
        request_id = request.get("request_id") if isinstance(request, dict) else None
        reply = {"request_id": request_id, "status": "rejected"}
        if not isinstance(request_id, str):
            return reply
        try:
            if request.get("op") == "observe":
                reply.update(status="ok", observation=self.robot.observe())
            elif request.get("op") == "execute" and self.robot.execute(request):
                reply.update(status="completed", observation=self.robot.observe())
        except Exception as exc:
            reply.update(status="error", error=str(exc))
        return reply

    def run(self):
        with zmq.Context() as context, context.socket(zmq.DEALER) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.IDENTITY, self.robot_id.encode())
            socket.connect(self.endpoint)
            socket.send_json({"type": "hello"})
            while True:
                try:
                    request = socket.recv_json()
                except (ValueError, UnicodeDecodeError):
                    socket.send_json({"request_id": None, "status": "rejected"})
                    continue
                socket.send_json(self.handle(request))
