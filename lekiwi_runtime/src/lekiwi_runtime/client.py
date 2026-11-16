import sys


import zmq

from .robot import PreDispatchRejected, Robot
from .guard import CAPABILITIES, GUARD_FIELDS, ExecutionGuard


class RobotClient:
    def __init__(self, robot: Robot, robot_id: str, endpoint: str):
        self.robot = robot
        self.robot_id = robot_id
        self.endpoint = endpoint
        self.guard = ExecutionGuard(robot)

    def handle(self, request: dict) -> dict:
        request_id = request.get("request_id") if isinstance(request, dict) else None
        reply = {"request_id": request_id, "status": "rejected"}
        if not isinstance(request_id, str):
            return reply
        if request.get("op") == "execute":
            print(f"Execution started robot={self.robot_id} request={request_id} action={request.get('action')}", file=sys.stderr, flush=True)
        try:
            if request.get("op") == "observe":
                reply.update(status="ok", **self.guard.observe())
            elif request.get("op") == "execute":
                if GUARD_FIELDS.intersection(request):
                    reply.update(self.guard.execute(request))
                else:
                    # Legacy clients retain the old wire format, but a legacy
                    # command always invalidates any guarded observation lease.
                    self.guard.invalidate()
                    try:
                        accepted = self.robot.execute(request)
                    except PreDispatchRejected as exc:
                        # This known no-write rejection must not poison later
                        # guarded leases. Legacy replies keep their small shape.
                        reply.update(status="rejected", error=str(exc),
                                     error_code=exc.code, error_details=exc.details)
                        accepted = False
                    except BaseException:
                        self.guard.halt()
                        raise
                    if accepted:
                        reply.update(status="completed", observation=self.robot.observe())
        except Exception as exc:
            print(f'Request failed robot={self.robot_id} request={request_id} error={exc}', file=sys.stderr, flush=True)
            reply.update(status="error", error=str(exc))
            if request.get("op") == "observe":
                reply.update(capabilities=dict(CAPABILITIES), action_context=self.guard._context())
        if request.get("op") == "execute":
            print(f"Execution finished robot={self.robot_id} request={request_id} status={reply['status']}", file=sys.stderr, flush=True)
        return reply

    def run(self):
        with zmq.Context() as context, context.socket(zmq.DEALER) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.IDENTITY, self.robot_id.encode())
            print(f'Connecting to server robot={self.robot_id} endpoint={self.endpoint}', file=sys.stderr, flush=True)
            socket.connect(self.endpoint)
            socket.send_json({"type": "hello"})
            while True:
                try:
                    request = socket.recv_json()
                except (ValueError, UnicodeDecodeError):
                    print(f'Rejected malformed JSON request robot={self.robot_id}', file=sys.stderr, flush=True)
                    socket.send_json({"request_id": None, "status": "rejected"})
                    continue
                socket.send_json(self.handle(request))
