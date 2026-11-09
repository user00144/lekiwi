import json
import sys
from pathlib import Path

from openai_codex import Codex, CodexConfig, Sandbox


class CodexRuntime:
    def __init__(self, server: str = "tcp://127.0.0.1:5556", model: str | None = None, snapshot: Path | None = None):
        self.server = server
        self.model = model
        self.snapshot = snapshot.resolve() if snapshot else None

    def run(self, robot_id: str, instruction: str) -> dict:
        args = ["-m", "harness", "--server", self.server]
        if self.snapshot:
            args.extend(["--snapshot", str(self.snapshot)])
        config = CodexConfig(config_overrides=(
            f'mcp_servers.lekiwi.command={json.dumps(sys.executable)}',
            'mcp_servers.lekiwi.args=' + json.dumps(args),
            'mcp_servers.lekiwi.enabled_tools=["observe"]',
        ))
        schema = {
            "type": "object", "additionalProperties": False,
            "required": ["robot_id", "command", "reason"],
            "properties": {
                "robot_id": {"type": "string", "enum": [robot_id]},
                "command": {
                    "type": "object", "additionalProperties": False,
                    "required": ["action", "duration_ms"],
                    "properties": {
                        "action": {"type": "string", "enum": ["wait"]},
                        "duration_ms": {"type": "integer", "minimum": 0, "maximum": 1000},
                    },
                },
                "reason": {"type": "string"},
            },
        }
        with Codex(config) as codex:
            thread = codex.thread_start(
                model=self.model, sandbox=Sandbox.read_only, ephemeral=True,
                developer_instructions=(
                    "Call lekiwi.observe for the requested robot before deciding. "
                    "Use the current images, radar and joints to propose one control command. "
                    "This proposal-only mode supports wait (duration_ms 0..1000); movement execution is available separately through MCP execute. "
                    "If the instruction needs unsupported movement, output wait with duration_ms=0 "
                    "and explain the limitation. If observation fails, do not invent sensor data. "
                    "If source.recorded is true, explicitly say the observation is recorded, not live. "
                    "Output JSON only. Never execute a robot command, use shell, or modify files."
                ),
            )
            result = thread.run(
                json.dumps({"robot_id": robot_id, "instruction": instruction}, ensure_ascii=False),
                output_schema=schema,
            )
        if result.status.value != "completed":
            raise RuntimeError(f"Codex turn did not complete: {result.status}")
        observed = False
        for item in result.items:
            event = item.model_dump(mode="json")
            if (event.get("type") == "mcpToolCall" and event.get("server") == "lekiwi"
                    and event.get("tool") == "observe" and event.get("status") == "completed"
                    and event.get("arguments", {}).get("robot_id") == robot_id):
                for content in (event.get("result") or {}).get("content", []):
                    if content.get("type") == "text":
                        try:
                            observed |= json.loads(content["text"]).get("status") == "ok"
                        except (ValueError, AttributeError):
                            pass
        if not observed:
            raise RuntimeError("No successful MCP observation; control output discarded")
        return json.loads(result.final_response)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("instruction")
    parser.add_argument("--robot-id", default="kiwi-01")
    parser.add_argument("--server", default="tcp://127.0.0.1:5556")
    parser.add_argument("--model")
    parser.add_argument("--snapshot", type=Path)
    args = parser.parse_args()
    result = CodexRuntime(args.server, args.model, args.snapshot).run(args.robot_id, args.instruction)
    print(json.dumps(result, ensure_ascii=False, indent=2))
