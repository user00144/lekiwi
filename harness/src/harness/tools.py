from copy import deepcopy
import json

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
