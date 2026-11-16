"""Real IPC relay and mock runtime; no model, network, or physical robot calls."""

import asyncio
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from harness.client import ServerClient
from harness.episode import EpisodeLimits, EpisodeRunner, MCPRobotTools
from harness.tools import RobotTools


ROOT = Path(__file__).resolve().parents[2]

# Instrument only the mock driver's entry point, so a replay cannot pass merely
# because sending the same absolute target twice happens to leave the same pose.
MOCK_RUNTIME = """
import json
import os
from pathlib import Path
from lekiwi_runtime import main
from lekiwi_runtime.client import RobotClient
from lekiwi_runtime.orin import OrinRobot

original_execute = OrinRobot.execute
original_init = RobotClient.__init__

def counted_execute(self, command):
    assert self.mock, 'Integration tests must never use real hardware'
    with Path(os.environ['LEKIWI_TEST_COMMAND_LOG']).open('a') as log:
        log.write(json.dumps(command) + '\\n')
    return original_execute(self, command)

def configured_init(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    self.guard.max_age_s = float(os.environ['LEKIWI_TEST_MAX_AGE'])

OrinRobot.execute = counted_execute
RobotClient.__init__ = configured_init
main()
"""


def metadata(content):
    return json.loads(content[0].text)


def guard_fields(observed, action_id="action-1"):
    observation = observed["observation"]
    return {
        "observation_id": observation["observation_id"],
        "runtime_id": observation["runtime_id"],
        "action_id": action_id,
        "reason": "Verify one bounded action using the current mock observation",
    }


class GuardedIntegrationTest(unittest.IsolatedAsyncioTestCase):
    @asynccontextmanager
    async def stack(self, max_age_s=60):
        with tempfile.TemporaryDirectory(prefix="lekiwi-guard-") as folder:
            folder = Path(folder)
            robots, api = f"ipc://{folder}/robots", f"ipc://{folder}/api"
            command_log = folder / "commands.jsonl"
            command_log.touch()
            environment = {key: value for key, value in os.environ.items() if key != "VIRTUAL_ENV"}
            environment.update(
                PYTHONPATH=os.pathsep.join(str(ROOT / name / "src") for name in ("server", "lekiwi_runtime")),
                LEKIWI_TEST_COMMAND_LOG=str(command_log),
                LEKIWI_TEST_MAX_AGE=str(max_age_s),
            )
            processes = []
            logs = []
            try:
                for name, args in (
                    ("server", [str(ROOT / "server/main.py"), "--bind", robots, "--api", api, "--timeout", "2"]),
                    ("lekiwi_runtime", ["-c", MOCK_RUNTIME, "--server", robots, "--robot-id", "mock", "--mock"]),
                ):
                    log = (folder / f"{name}.stderr").open("w+")
                    logs.append(log)
                    processes.append(subprocess.Popen(
                        [str(ROOT / name / ".venv/bin/python"), *args], env=environment,
                        stdout=subprocess.DEVNULL, stderr=log,
                    ))
                client = ServerClient(api, timeout=3)
                deadline = time.monotonic() + 10
                while True:
                    stopped = [process for process in processes if process.poll() is not None]
                    if stopped:
                        for log in logs:
                            log.seek(0)
                        self.fail("IPC process exited: " + "\n".join(log.read() for log in logs))
                    try:
                        observed = await client.request("mock", "observe")
                        if observed.get("status") == "ok":
                            break
                    except TimeoutError:
                        pass
                    if time.monotonic() >= deadline:
                        self.fail("Mock runtime did not become available")
                    await asyncio.sleep(.02)
                yield client, RobotTools(client), command_log
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.terminate()
                    try:
                        await asyncio.to_thread(process.wait, timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        await asyncio.to_thread(process.wait)
                for log in logs:
                    log.close()

    async def test_guarded_arm_action_returns_measured_state_and_new_camera(self):
        async with self.stack() as (_, tools, command_log):
            observed = metadata(await tools.observe("mock"))
            self.assertEqual(observed["capabilities"]["guarded_execution"], 1)
            self.assertTrue(observed["action_context"]["movement_allowed"])
            before = observed["observation"]
            guard = guard_fields(observed)
            target = before["ego"]["joints"]["wrist_roll"] + 2
            content = await tools.execute("mock", joints={"wrist_roll": target}, duration_ms=200, **guard)
            result = metadata(content)
            self.assertEqual(result["status"], "completed")
            self.assertTrue(result["execution"]["arm_reached"])
            self.assertEqual(result["execution"]["action_id"], guard["action_id"])
            after = result["observation"]
            self.assertAlmostEqual(after["ego"]["joints"]["wrist_roll"], target, delta=.1)
            self.assertNotEqual(after["observation_id"], before["observation_id"])
            self.assertEqual(after["runtime_id"], before["runtime_id"])
            self.assertGreater(after["cameras"]["wrist"]["timestamp"], result["execution"]["motion_finished_at"])
            self.assertGreater(after["cameras"]["wrist"]["timestamp"], before["cameras"]["wrist"]["timestamp"])
            self.assertTrue(any(item.type == "image" for item in content))
            calls = [json.loads(line) for line in command_log.read_text().splitlines()]
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0]["observation_id"], before["observation_id"])
            self.assertNotEqual(calls[0]["request_id"], calls[0]["observation_id"])
            self.assertNotEqual(calls[0]["request_id"], calls[0]["action_id"])
            self.assertEqual(calls[0]["request_id"], result["request_id"])

    async def test_replayed_action_is_not_executed_again_and_changed_payload_is_rejected(self):
        async with self.stack() as (client, tools, command_log):
            observed = metadata(await tools.observe("mock"))
            command = {"action": "move", "duration_ms": 200, "joints": {"wrist_roll": 2},
                       "x": 0, "y": 0, "theta": 0, **guard_fields(observed)}
            first = await client.request("mock", "execute", command=command)
            second = await client.request("mock", "execute", command=command)
            self.assertEqual(first["status"], "completed")
            self.assertEqual(second["status"], "completed")
            self.assertTrue(second["replayed"])
            self.assertNotEqual(second["request_id"], first["request_id"])
            self.assertEqual(second["observation"], first["observation"])
            self.assertEqual(second["execution"], first["execution"])
            changed = await client.request("mock", "execute", command={**command, "joints": {"wrist_roll": 3}})
            self.assertEqual(changed["status"], "rejected")
            self.assertEqual(changed["error"], "action_id_conflict")
            self.assertEqual(len(command_log.read_text().splitlines()), 1)

    async def test_stale_observation_rejected_and_legacy_tools_remain_available(self):
        async with self.stack() as (client, tools, command_log):
            old = metadata(await tools.observe("mock"))
            current = metadata(await tools.observe("mock"))
            self.assertNotEqual(current["observation"]["observation_id"], old["observation"]["observation_id"])
            rejected = await client.request("mock", "execute", command={
                "action": "wait", "duration_ms": 1, **guard_fields(old),
            })
            self.assertEqual(rejected["status"], "rejected")
            self.assertEqual(rejected["error"], "observation_mismatch")
            self.assertEqual(command_log.read_text(), "")
            legacy = metadata(await tools.execute("mock", action="wait", duration_ms=1))
            self.assertEqual(legacy["status"], "completed")
            self.assertTrue(legacy["observation"]["simulated"])
            # A legacy action invalidates a token held by an independent planner.
            invalidated = await client.request("mock", "execute", command={
                "action": "wait", "duration_ms": 1, **guard_fields(current, "after-legacy"),
            })
            self.assertEqual(invalidated["status"], "rejected")
            self.assertEqual(invalidated["error"], "observation_mismatch")
            self.assertEqual(len(command_log.read_text().splitlines()), 1)

    async def test_expired_observation_never_reaches_driver(self):
        async with self.stack(max_age_s=.1) as (client, tools, command_log):
            observed = metadata(await tools.observe("mock"))
            await asyncio.sleep(.15)
            result = await client.request("mock", "execute", command={
                "action": "wait", "duration_ms": 1, **guard_fields(observed),
            })
            self.assertEqual(result["status"], "rejected")
            self.assertEqual(result["error"], "observation_expired")
            self.assertEqual(command_log.read_text(), "")

    async def test_scripted_episode_uses_mcp_feedback_then_rechecks_completion(self):
        async with self.stack() as (client, _, command_log):
            contexts, image_sets = [], []

            class ScriptedPlanner:
                async def decide(self, context, images):
                    contexts.append(context)
                    image_sets.append(images)
                    observation = context["current"]["observation"]
                    command = None
                    if len(contexts) == 1:
                        targets = dict.fromkeys(("shoulder_pan", "shoulder_lift", "elbow_flex",
                                                "wrist_flex", "wrist_roll", "gripper"))
                        targets["wrist_roll"] = observation["ego"]["joints"]["wrist_roll"] + 3
                        command = {"action": "move", "x": 0.0, "y": 0.0, "theta": 0.0,
                                   "duration_ms": 200, "joints": targets}
                    return {
                        "robot_id": "mock", "observation_id": observation["observation_id"],
                        "tool": "execute" if command else "task_complete",
                        "plan": ["Move one mock joint", "Check measured endpoint"],
                        "reason": "Use current mock camera and encoder feedback",
                        "expected": "The measured wrist roll increases by three degrees",
                        "feedback": "initial" if command else "progress",
                        "evidence": "The measured endpoint matches the requested mock target",
                        "command": command,
                    }

            parameters = StdioServerParameters(
                command=str(ROOT / "harness/.venv/bin/python"),
                args=[str(ROOT / "harness/main.py"), "--server", client.endpoint],
                env={"PYTHONPATH": str(ROOT / "harness/src")},
            )
            run_dir = command_log.parent / "episode"
            async with stdio_client(parameters) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    runner = EpisodeRunner(ScriptedPlanner(), MCPRobotTools(session), "mock",
                                           "Move the mock wrist once", run_dir,
                                           EpisodeLimits(max_actions=2, max_steps=4, max_seconds=15))
                    report = await runner.run()
            self.assertEqual(report["status"], "completion_claim", report)
            self.assertEqual(report["action_attempts"], 1)
            self.assertEqual(report["steps"], 3)
            self.assertEqual(report["observations"], 3)
            self.assertFalse(report["physical_success_verified"])
            self.assertEqual(len(command_log.read_text().splitlines()), 1)
            feedback = contexts[1]["pending_motion_feedback"]
            self.assertAlmostEqual(feedback["joint_delta"]["wrist_roll"], 3, delta=.1)
            self.assertNotEqual(feedback["before_observation_id"], feedback["after_observation_id"])
            self.assertTrue(any(label.startswith("BEFORE LAST MOTION") for label, _, _ in image_sets[1]))
            self.assertIsNotNone(contexts[2]["completion_candidate"])
            self.assertNotEqual(contexts[1]["current"]["observation"]["observation_id"],
                                contexts[2]["current"]["observation"]["observation_id"])
            self.assertEqual(json.loads((run_dir / "result.json").read_text()), report)
            events = [json.loads(line)["event"] for line in (run_dir / "events.jsonl").read_text().splitlines()]
            self.assertLess(events.index("action_prepared"), events.index("action_result"))
            # Each camera has its own capture thread. For this arm-only command
            # the wrist may produce its post-motion frame before the front;
            # the guard must omit that older front image, not save six images
            # by reusing a pre-action frame. Verify the exact declared files
            # and frame barrier instead of assuming synchronized cameras.
            directories = sorted((run_dir / "observations").iterdir())
            self.assertEqual(len(directories), 3)
            for directory, context in zip(directories, contexts):
                stored = json.loads((directory / "observation.json").read_text())
                self.assertEqual(stored["observation"]["observation_id"],
                                 context["current"]["observation"]["observation_id"])
                manifest = json.loads((directory / "images.json").read_text())
                expected = {name for name, camera in stored["observation"]["cameras"].items()
                            if camera["available"] and camera["encoding"] == "jpeg_base64"}
                self.assertIn("wrist", expected)
                images = manifest["images"]
                self.assertEqual(len(images), len(expected))
                self.assertEqual({image["label"] for image in images}, expected)
                self.assertEqual(set(directory.glob("*.jpg")),
                                 {run_dir / image["path"] for image in images})
                for image in images:
                    self.assertGreater((run_dir / image["path"]).stat().st_size, 0)
            post = contexts[1]["current"]
            barrier = post["execution"]["motion_finished_at"]
            post_images = json.loads((directories[1] / "images.json").read_text())["images"]
            self.assertEqual({image["label"] for image in post_images},
                             {label.removeprefix("CURRENT ") for label, _, _ in image_sets[1]
                              if label.startswith("CURRENT ")})
            for image in post_images:
                self.assertGreater(post["observation"]["cameras"][image["label"]]["timestamp"], barrier)
            front = post["observation"]["cameras"]["front"]
            if not front["available"]:
                self.assertEqual(front["error"], "no_fresh_post_action_frame")
                self.assertIsNone(front["data"])
                self.assertNotIn("front", {image["label"] for image in post_images})


if __name__ == "__main__":
    unittest.main()
