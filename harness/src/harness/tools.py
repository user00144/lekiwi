from copy import deepcopy
import json
import math
from typing import Annotated

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
                      x: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)],
                      y: Annotated[float, Field(strict=True, ge=-0.4, le=0.4, allow_inf_nan=False)],
                      theta: Annotated[float, Field(strict=True, ge=-150, le=150, allow_inf_nan=False)],
                      duration_ms: Annotated[int, Field(strict=True, ge=1, le=1000)]) -> list[TextContent | ImageContent]:
        """Move for 1..1000 ms, then stop. x forward/y left in m/s (speed <=0.4);
        theta counterclockwise in deg/s (absolute <=150). Returns post-stop observation.
        A timeout means outcome unknown: observe before deciding; never blindly retry.
        """
        if isinstance(self.client, SnapshotClient):
            raise ValueError("Snapshots cannot execute robot commands")
        if (type(duration_ms) is not int or not 1 <= duration_ms <= 1000
                or not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, theta))
                or math.hypot(x, y) > 0.4 or abs(theta) > 150):
            raise ValueError("Expected finite velocity: hypot(x,y)<=0.4 m/s, abs(theta)<=150 deg/s, duration_ms=1..1000")
        response = await self.client.request(robot_id, "execute", command={
            "action": "move", "x": x, "y": y, "theta": theta, "duration_ms": duration_ms,
        })
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
