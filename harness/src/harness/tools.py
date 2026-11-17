from copy import deepcopy
import json
import math
from typing import Annotated, Literal

from pydantic import Field

from mcp.types import ImageContent, TextContent

from .client import ServerClient, SnapshotClient


BASE_MAX_DURATION_MS = 1000
ARM_MIN_DURATION_MS = 100
ARM_MAX_DURATION_MS = 20000


def build_command(action, x, y, theta, duration_ms, joints=None):
    """Validate the wire command, including callers outside the MCP schema."""
    if not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, theta)):
        raise ValueError("Velocity must be finite")
    arm_only = action == 'move' and isinstance(joints, dict) and bool(joints) and not any((x, y, theta))
    maximum = ARM_MAX_DURATION_MS if arm_only else BASE_MAX_DURATION_MS
    if type(duration_ms) is not int or not 1 <= duration_ms <= maximum:
        raise ValueError(f"duration_ms must be 1..{maximum}; only arm-only targets allow more than 1000ms")
    command = {"action": action, "duration_ms": duration_ms}
    if action == "move":
        if math.hypot(x, y) > .4 or abs(theta) > 150:
            raise ValueError("Expected hypot(x,y)<=0.4 m/s and abs(theta)<=150 deg/s")
        command.update(x=x, y=y, theta=theta)
        if joints is not None:
            names = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
            if (duration_ms < ARM_MIN_DURATION_MS or not isinstance(joints, dict) or not joints
                    or not all(k in names and type(v) in (int, float) and math.isfinite(v)
                               and (0 <= v <= 100 if k == "gripper" else -180 <= v <= 180)
                               for k, v in joints.items())):
                raise ValueError("Invalid arm targets or duration")
            command["joints"] = joints
    elif action == "wait":
        if joints is not None or any((x, y, theta)):
            raise ValueError("Wait cannot contain motion targets")
    else:
        raise ValueError("Unsupported action")
    return command


class RobotTools:
    def __init__(self, client: ServerClient | SnapshotClient):
        self.client = client

    async def observe(self, robot_id: str) -> list[TextContent | ImageContent]:
        """Get current cameras, radar, joints and IMU for one robot.
        IMU includes acceleration, angular velocity, orientation and drift-prone relative displacement.
        Use displacement only when available and estimate_valid; it is not exact position."""
        response = await self.client.request(robot_id, "observe")
        if response.get("status") != "ok" or not isinstance(response.get("observation"), dict):
            raise RuntimeError(f"Observation unavailable: {response}")
        return self._content(response)

    async def reconcile(self, robot_id: str, runtime_id: str, failed_action_id: str) -> list[TextContent | ImageContent]:
        """Read and verify a runtime-certified held arm after known incomplete motion.

        This sends no motor target. Only the host may request it for the exact
        failed action and runtime; it cannot recover unknown execution outcomes.
        """
        if isinstance(self.client, SnapshotClient):
            raise ValueError('Snapshots cannot reconcile robot state')
        if not all(isinstance(value, str) and value.strip() for value in (robot_id, runtime_id, failed_action_id)):
            raise ValueError('Reconciliation requires robot_id, runtime_id and failed_action_id')
        response = await self.client.request(robot_id, 'reconcile', runtime_id=runtime_id,
                                             failed_action_id=failed_action_id)
        return self._content(response)

    async def execute(self, robot_id: str,
                      x: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)] = 0,
                      y: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)] = 0,
                      theta: Annotated[float, Field(strict=True, ge=-150, le=150, allow_inf_nan=False)] = 0,
                      duration_ms: Annotated[int, Field(strict=True, ge=1, le=ARM_MAX_DURATION_MS)] = 500,
                      action: Literal["move", "wait"] = "move",
                      joints: dict[str, Annotated[float, Field(strict=True, allow_inf_nan=False)]] | None = None,
                      observation_id: str | None = None,
                      runtime_id: str | None = None,
                      action_id: str | None = None,
                      reason: str | None = None,
                      ) -> list[TextContent | ImageContent]:
        """Execute one bounded action. move: x forward/y left m/s, theta CCW deg/s.
        move accepts optional joints: absolute degrees, gripper 0..100%; omitted joints unchanged.
        Omitted base velocities default to zero. Supply both to move base and arm together.
        wait pauses without motion targets.
        Arm-only duration 100..20000ms when x=y=theta=0; base/wait/combined duration <=1000ms.
        Choose enough arm time for the joint distance; speed <=30deg/s or 50%/s, calibrated limits from observe.
        Arm completion includes joint feedback; torque remains on to hold posture.
        Timeout means outcome unknown; do not blindly retry.
        Guarded episode calls must supply observation_id, runtime_id, action_id and reason together.
        """
        if isinstance(self.client, SnapshotClient):
            raise ValueError("Snapshots cannot execute robot commands")
        command = build_command(action, x, y, theta, duration_ms, joints)
        metadata = dict(observation_id=observation_id, runtime_id=runtime_id, action_id=action_id, reason=reason)
        guarded = any(value is not None for value in metadata.values())
        if guarded:
            if not all(isinstance(value, str) and value.strip() for value in metadata.values()):
                raise ValueError("All guarded execution fields must be nonempty strings")
            command.update(metadata)
        response = await self.client.request(robot_id, "execute", command=command)
        if not guarded and response.get("status") != "completed":
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
