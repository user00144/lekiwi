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

    def run_episode(self, robot_id: str, instruction: str, *, run_dir=None,
                    max_actions=20, max_steps=40, max_seconds=300, kinematics_config=None, plan_act=None):
        """Explicit execution mode; the legacy run() remains proposal-only."""
        import asyncio
        from .episode import EpisodeLimits
        from .episode_runtime import run_episode

        if plan_act is not None:
            from .plan_loop import run_plan_episode
            return asyncio.run(run_plan_episode(
                self.server, self.model, robot_id, instruction, snapshot=self.snapshot, run_dir=run_dir,
                limits=EpisodeLimits(max_actions, max_steps, max_seconds), kinematics_config=kinematics_config,
                **plan_act))
        return asyncio.run(run_episode(
            self.server, self.model, robot_id, instruction, snapshot=self.snapshot,
            run_dir=run_dir, limits=EpisodeLimits(max_actions, max_steps, max_seconds),
            kinematics_config=kinematics_config,
        ))


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("instruction")
    parser.add_argument("--robot-id", default="kiwi-01")
    parser.add_argument("--server", default="tcp://127.0.0.1:5556")
    parser.add_argument("--model")
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--execute", action="store_true", help="Run a bounded observe/decide/execute episode")
    parser.add_argument("--max-actions", type=int, help="default 20 (step) / 120 (plan-act)")
    parser.add_argument("--max-steps", type=int, help="default 40 (step) / 60 (plan-act)")
    parser.add_argument("--max-seconds", type=float, help="default 300 (step) / 1200 (plan-act)")
    parser.add_argument("--run-dir", type=Path, help="New inspection-only run directory; never resumes motors")
    parser.add_argument("--kinematics-config", type=Path, help="Verified local joint mapping for guarded XYZ execution")
    parser.add_argument("--loop", choices=("auto", "step", "plan-act"), default="auto",
                        help="auto (default): plan-act when --skills-config is given, else step. step: one small action "
                             "per decision; plan-act: Planning → big-skill Action Loop → Verify/replan with robot memory")
    parser.add_argument("--skills-config", type=Path, help="plan-act: home/grasp/gripper skill settings")
    parser.add_argument("--camera-config", type=Path, help="plan-act: calibrated front camera for locate (optional)")
    parser.add_argument("--experience", type=Path, help="plan-act: experience JSONL (default: runs/experience.jsonl)")
    parser.add_argument("--max-replans", type=int, default=3)
    parser.add_argument("--freeze-skill-library", action="store_true",
                        help="plan-act: read the skill library but never write it (evaluation)")
    args = parser.parse_args()
    if args.execute and args.snapshot:
        parser.error("--execute cannot be combined with --snapshot")
    if args.kinematics_config and not args.execute:
        parser.error("--kinematics-config requires --execute")
    if args.loop == "auto":
        args.loop = "plan-act" if args.skills_config else "step"
    defaults = (120, 60, 1200.0) if args.loop == "plan-act" else (20, 40, 300.0)
    args.max_actions = args.max_actions if args.max_actions is not None else defaults[0]
    args.max_steps = args.max_steps if args.max_steps is not None else defaults[1]
    args.max_seconds = args.max_seconds if args.max_seconds is not None else defaults[2]
    runtime = CodexRuntime(args.server, args.model, args.snapshot)
    if args.execute:
        plan_act = None
        if args.loop == "plan-act":
            if not args.kinematics_config or not args.skills_config:
                parser.error("--loop plan-act needs --kinematics-config and --skills-config")
            plan_act = {"skills_config": args.skills_config, "camera_config": args.camera_config,
                        "experience_path": args.experience, "max_replans": args.max_replans,
                        "freeze_library": args.freeze_skill_library}
        result = runtime.run_episode(args.robot_id, args.instruction, run_dir=args.run_dir,
                                     max_actions=args.max_actions, max_steps=args.max_steps,
                                     max_seconds=args.max_seconds, kinematics_config=args.kinematics_config,
                                     plan_act=plan_act)
    else:
        result = runtime.run(args.robot_id, args.instruction)
    print(json.dumps(result, ensure_ascii=False, indent=2))
