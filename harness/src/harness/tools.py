from copy import deepcopy
import json
import math
from typing import Annotated, Literal

from pydantic import Field

from mcp.types import ImageContent, TextContent

from .client import ServerClient, SnapshotClient


class RobotTools:
    def __init__(self, client: ServerClient | SnapshotClient):
        self.client = client

    async def observe(self, robot_id: str) -> list[TextContent | ImageContent]:
        """Get current cameras, radar and joints for one robot."""
        response = await self.client.request(robot_id, "observe")
        if response.get("status") != "ok" or not isinstance(response.get("observation"), dict):
            raise RuntimeError(f"Observation unavailable: {response}")
        return self._content(response)

    async def execute(self, robot_id: str,
                      x: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)] = 0,
                      y: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)] = 0,
                      theta: Annotated[float, Field(strict=True, ge=-150, le=150, allow_inf_nan=False)] = 0,
                      duration_ms: Annotated[int, Field(strict=True, ge=1, le=1000)] = 500,
                      action: Literal["move", "move_arm", "wait"] = "move",
                      joints: dict[str, Annotated[float, Field(strict=True, allow_inf_nan=False)]] | None = None,
                      ) -> list[TextContent | ImageContent]:
        """Execute one bounded action. move: x forward/y left m/s, theta CCW deg/s.
        move_arm: absolute joint degrees, gripper 0..100%; omitted joints unchanged.
        Arm duration 100..1000ms; speed <=30deg/s or 50%/s, calibrated limits from observe.
        Arm completion includes joint feedback; torque remains on to hold posture.
        Timeout means outcome unknown; do not blindly retry.
        """
        if isinstance(self.client, SnapshotClient):
            raise ValueError("Snapshots cannot execute robot commands")
        if type(duration_ms) is not int or not 1 <= duration_ms <= 1000:
            raise ValueError("duration_ms must be 1..1000")
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, theta)):
            raise ValueError("Velocity must be finite")
        command = {"action": action, "duration_ms": duration_ms}
        if action == "move_arm":
            names = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
            if (any((x, y, theta)) or duration_ms < 100 or not isinstance(joints, dict) or not joints
                    or not all(k in names and type(v) in (int, float) and math.isfinite(v)
                               and (0 <= v <= 100 if k == "gripper" else -180 <= v <= 180)
                               for k, v in joints.items())):
                raise ValueError("Invalid arm targets; use absolute degrees/gripper percent and zero base velocity")
            command["joints"] = joints
        elif action == "move":
            if joints is not None or math.hypot(x, y) > .4 or abs(theta) > 150:
                raise ValueError("Expected hypot(x,y)<=0.4 m/s, abs(theta)<=150 deg/s and no arm targets")
            command.update(x=x, y=y, theta=theta)
        elif action == "wait":
            if joints is not None or any((x, y, theta)):
                raise ValueError("Wait cannot contain motion targets")
        else:
            raise ValueError("Unsupported action")
        response = await self.client.request(robot_id, "execute", command=command)
        if response.get("status") != "completed":
            raise RuntimeError(f"Execution not confirmed; do not retry blindly: {response}")
        return self._content(response)

    @staticmethod
    def _content(response: dict) -> list[TextContent | ImageContent]:
        metadata = deepcopy(response)
        images = []
        for name, camera in metadata.get("observation", {}).get("cameras", {}).items():
            if camera.get("available") and camera.get("encoding") == "jpeg_base64":
                data = camera.pop("data")
                images.extend([TextContent(type="text", text=name),
                               ImageContent(type="image", data=data, mimeType="image/jpeg")])
        return [TextContent(type="text", text=json.dumps(metadata, ensure_ascii=False)), *images]
