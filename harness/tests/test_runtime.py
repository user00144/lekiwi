from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from harness.runtime import CodexRuntime


class RuntimeTest(unittest.TestCase):
    def run_with(self, events):
        sdk = MagicMock()
        thread = sdk.__enter__.return_value.thread_start.return_value
        thread.run.return_value = SimpleNamespace(
            status=SimpleNamespace(value="completed"),
            items=[SimpleNamespace(model_dump=lambda **_: event) for event in events],
            final_response='{"robot_id":"a","command":{"action":"wait","duration_ms":100},"reason":"test"}',
        )
        with patch("harness.runtime.Codex", return_value=sdk):
            return CodexRuntime().run("a", "wait")

    def test_requires_successful_mcp_observation(self):
        with self.assertRaisesRegex(RuntimeError, "No successful MCP observation"):
            self.run_with([])
        with self.assertRaisesRegex(RuntimeError, "No successful MCP observation"):
            self.run_with([{"type": "mcpToolCall", "server": "lekiwi", "tool": "observe",
                            "status": "failed", "arguments": {"robot_id": "a"}}])

    def test_returns_control_output_after_observation(self):
        result = self.run_with([{
            "type": "mcpToolCall", "server": "lekiwi", "tool": "observe", "status": "completed",
            "arguments": {"robot_id": "a"},
            "result": {"content": [{"type": "text", "text": '{"status":"ok","observation":{}}'}]},
        }])
        self.assertEqual(result["command"], {"action": "wait", "duration_ms": 100})


class SnapshotTest(unittest.IsolatedAsyncioTestCase):
    async def test_snapshot_via_observe_and_identity_check(self):
        import json
        from pathlib import Path
        import tempfile
        from harness.client import SnapshotClient
        from harness.tools import RobotTools

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "frame-00.json"
            path.write_text(json.dumps({"status": "ok", "observation": {"timestamp": 123}}))
            path.with_name("schema.json").write_text('{"identity":"kiwi-01"}')
            tools = RobotTools(SnapshotClient(path))
            result = await tools.observe("kiwi-01")
            data = json.loads(result[0].text)
            self.assertTrue(data["source"]["recorded"])
            self.assertEqual(data["observation"]["timestamp"], 123)
            with self.assertRaises(ValueError):
                await tools.observe("kiwi-02")
