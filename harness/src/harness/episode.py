"""One host-owned decision/action/observation loop; never replay motor commands."""
from __future__ import annotations

import asyncio
import base64
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
from typing import Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .tools import build_command


class JointTargets(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    shoulder_pan: float | None
    shoulder_lift: float | None
    elbow_flex: float | None
    wrist_flex: float | None
    wrist_roll: float | None
    gripper: float | None


class Command(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    action: Literal['move', 'wait']
    x: float
    y: float
    theta: float
    duration_ms: int
    joints: JointTargets | None

    def wire(self):
        joints = self.joints.model_dump(exclude_none=True) if self.joints else None
        return build_command(self.action, self.x, self.y, self.theta, self.duration_ms, joints)


class Decision(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    robot_id: str
    observation_id: str
    tool: Literal['observe', 'execute', 'task_complete', 'blocked']
    plan: list[str] = Field(max_length=8)
    reason: str = Field(min_length=1, max_length=2000)
    expected: str = Field(max_length=1000)
    feedback: Literal['initial', 'progress', 'stalled', 'uncertain']
    evidence: str = Field(max_length=2000)
    command: Command | None

    @model_validator(mode='after')
    def command_matches_tool(self):
        if (self.tool == 'execute') != (self.command is not None):
            raise ValueError('Only execute requires a command')
        if not self.reason.strip():
            raise ValueError('reason cannot be blank')
        if self.tool == 'task_complete' and not self.evidence.strip():
            raise ValueError('Completion requires visible/measured evidence')
        return self


@dataclass(frozen=True)
class EpisodeLimits:
    max_actions: int = 20
    max_steps: int = 40
    max_seconds: float = 300

    def __post_init__(self):
        if any(type(v) is not int or v <= 0 for v in (self.max_actions, self.max_steps)):
            raise ValueError('Action and decision budgets must be positive integers')
        if type(self.max_seconds) not in (int, float) or not math.isfinite(self.max_seconds) or self.max_seconds <= 0:
            raise ValueError('Time budget must be positive and finite')


class EpisodeStopped(Exception):
    def __init__(self, reason, detail=''):
        super().__init__(detail or reason)
        self.reason, self.detail = reason, detail


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class RunRecord:
    def __init__(self, path, robot_id, instruction, limits):
        self.path = Path(path).resolve()
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'observations').mkdir()
        self.sequence = 0
        self.events = self.path / 'events.jsonl'
        atomic_json(self.path / 'run.json', {
            'version': 1, 'robot_id': robot_id, 'instruction': instruction,
            'started_at': datetime.now(timezone.utc).isoformat(), 'limits': asdict(limits),
            'mode': 'execute', 'automatic_resume': False,
        })
        atomic_json(self.path / 'result.json', {'status': 'running', 'automatic_resume': False})

    def event(self, kind, data):
        with self.events.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'at': datetime.now(timezone.utc).isoformat(), 'event': kind,
                                     'data': data}, ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())

    def observation(self, response, images):
        self.sequence += 1
        directory = self.path / 'observations' / f'{self.sequence:04d}'
        directory.mkdir()
        atomic_json(directory / 'observation.json', response)
        for index, (_, data, mime) in enumerate(images):
            suffix = '.jpg' if mime == 'image/jpeg' else '.png'
            (directory / f'image-{index:02d}{suffix}').write_bytes(base64.b64decode(data, validate=True))
        self.event('observation', {'path': str(directory.relative_to(self.path)),
                                  'observation_id': response['observation']['observation_id']})


def decode_content(content):
    """Decode the existing MCP text + labelled JPEG response, preserving images."""
    if not isinstance(content, (tuple, list)) or not content:
        raise ValueError('Missing MCP content')
    packet = json.loads(content[0].text)
    if not isinstance(packet, dict):
        raise ValueError('MCP packet must be an object')
    images, label = [], 'camera'
    for item in content[1:]:
        if item.type == 'text':
            label = item.text
        elif item.type == 'image':
            images.append((label, item.data, item.mimeType))
    return packet, images


class MCPRobotTools:
    """Host-side adapter: the model never owns the MCP session."""
    def __init__(self, session):
        self.session = session

    async def _call(self, name, arguments):
        response = await self.session.call_tool(name, arguments)
        if response.isError:
            raise RuntimeError(f'MCP {name} failed: {response.content}')
        return response.content

    async def observe(self, robot_id):
        return await self._call('observe', {'robot_id': robot_id})

    async def execute(self, robot_id, **arguments):
        return await self._call('execute', {'robot_id': robot_id, **arguments})


class EpisodeRunner:
    """Testable loop: planner.decide(context, images), and observe/execute tools.

    Budgets and observation identities are checked by the host. A record is inspection-only: a
    previously written action is never automatically replayed after restart.
    """
    def __init__(self, planner, tools, robot_id, instruction, run_dir, limits=None, *, clock=time.monotonic):
        self.planner, self.tools = planner, tools
        self.robot_id, self.instruction = robot_id, instruction
        self.limits = limits or EpisodeLimits()
        self.clock = clock
        self.record = RunRecord(run_dir, robot_id, instruction, self.limits)
        self.actions = self.steps = 0
        self.started = self.clock()
        self.packet = None
        self.images = []
        self.received_at = None
        self.runtime_id = None
        self.previous_decision = self.previous_result = None
        self.pending_feedback = None
        self.before_images = []
        self.completion_candidate = None
        self.claim = None
        self.in_flight = None
        self.rejected_signature = None

    def remaining(self):
        return self.limits.max_seconds - (self.clock() - self.started)

    def check_time(self):
        if self.remaining() <= 0:
            raise EpisodeStopped('budget_seconds')

    async def bounded(self, call):
        self.check_time()
        async with asyncio.timeout(self.remaining()):
            return await call()

    def _accept_observation(self, packet, images, *, previous_id=None):
        if (packet.get('source') or {}).get('recorded'):
            raise EpisodeStopped('recorded_observation', 'Recorded observations cannot authorize execution')
        capability = (packet.get('capabilities') or {}).get('guarded_execution')
        if type(capability) is not int or capability != 1:
            raise EpisodeStopped('guarded_execution_unavailable', 'Runtime does not advertise guarded_execution: 1')
        obs, context = packet.get('observation'), packet.get('action_context')
        if not isinstance(obs, dict) or not isinstance(context, dict):
            raise EpisodeStopped('observation_invalid', 'Observation and action_context are required')
        observation_id, runtime_id = obs.get('observation_id'), obs.get('runtime_id')
        if (not isinstance(observation_id, str) or not observation_id or
                not isinstance(runtime_id, str) or not runtime_id or
                context.get('observation_id') != observation_id or context.get('runtime_id') != runtime_id):
            raise EpisodeStopped('observation_invalid', 'Observation identity mismatch')
        if self.runtime_id is not None and runtime_id != self.runtime_id:
            raise EpisodeStopped('runtime_changed', 'Runtime restarted during this episode')
        age = context.get('max_age_s')
        if (context.get('movement_allowed') is not True or type(age) not in (int, float) or
                not math.isfinite(age) or age <= 0 or not images):
            raise EpisodeStopped('observation_invalid', 'Fresh usable camera/state observations are required')
        if previous_id is not None and observation_id == previous_id:
            raise EpisodeStopped('observation_invalid', 'A new observation is required')
        self.runtime_id = runtime_id
        self.packet, self.images, self.received_at = packet, images, self.clock()
        self.record.observation(packet, images)

    def _fresh(self):
        return (self.packet is not None and
                self.clock() - self.received_at < self.packet['action_context']['max_age_s'])

    async def observe(self):
        previous_id = self.packet['observation']['observation_id'] if self.packet else None
        packet, images = decode_content(await self.bounded(lambda: self.tools.observe(self.robot_id)))
        if packet.get('status') != 'ok':
            raise EpisodeStopped('observation_failed', str(packet.get('error', packet.get('status'))))
        self._accept_observation(packet, images, previous_id=previous_id)

    def _context(self):
        return {
            'robot_id': self.robot_id, 'task': self.instruction, 'current': self.packet,
            'previous_decision': self.previous_decision, 'previous_result': self.previous_result,
            'pending_motion_feedback': self.pending_feedback,
            'completion_candidate': self.completion_candidate,
            'budget': {'actions_remaining': self.limits.max_actions - self.actions,
                       'decisions_remaining': self.limits.max_steps - self.steps,
                       'seconds_remaining': max(0, round(self.remaining(), 2))},
        }

    async def run(self):
        reason, detail = 'host_error', ''
        try:
            await self.observe()
            while True:
                self.check_time()
                if self.steps >= self.limits.max_steps:
                    raise EpisodeStopped('budget_steps')
                if not self._fresh():
                    self.completion_candidate = None
                    await self.observe()
                context = self._context()
                image_input = [('CURRENT ' + name, data, mime) for name, data, mime in self.images]
                image_input += [('BEFORE LAST MOTION ' + name, data, mime)
                                for name, data, mime in self.before_images]
                raw = await self.bounded(lambda: self.planner.decide(context, image_input))
                self.steps += 1
                try:
                    decision = Decision.model_validate(raw)
                    if decision.command is not None:
                        decision.command.wire()
                except ValueError as exc:
                    raise EpisodeStopped('invalid_decision', str(exc)) from exc
                self.record.event('decision', decision.model_dump())
                self.check_time()
                if decision.robot_id != self.robot_id:
                    raise EpisodeStopped('invalid_decision', 'Decision targets another robot')
                if decision.observation_id != self.packet['observation']['observation_id'] or not self._fresh():
                    self.record.event('decision_rejected', {'reason': 'stale_observation'})
                    self.previous_result = {'status': 'rejected', 'reason': 'stale_observation; inspect new images'}
                    self.completion_candidate = None
                    await self.observe()
                    continue
                self.previous_decision = decision.model_dump()
                if decision.tool == 'blocked':
                    raise EpisodeStopped('blocked', decision.reason)
                if decision.tool == 'observe':
                    self.completion_candidate = None
                    await self.observe()
                elif decision.tool == 'task_complete':
                    if self.completion_candidate is not None:
                        self.claim = {'reason': decision.reason, 'evidence': decision.evidence,
                                      'observation_id': decision.observation_id,
                                      'first_claim': self.completion_candidate,
                                      'physical_success_verified': False}
                        raise EpisodeStopped('completion_claim', decision.reason)
                    self.completion_candidate = {'reason': decision.reason, 'evidence': decision.evidence,
                                                 'observation_id': decision.observation_id}
                    self.record.event('completion_candidate', self.completion_candidate)
                    await self.observe()
                else:
                    self.completion_candidate = None
                    await self.execute(decision)
        except EpisodeStopped as exc:
            reason, detail = exc.reason, exc.detail
        except TimeoutError:
            reason = 'uncertain_execution' if self.in_flight else 'budget_seconds'
            detail = 'Deadline expired; no action is retried'
        except asyncio.CancelledError:
            reason = 'uncertain_execution' if self.in_flight else 'interrupted'
            detail = 'Execution interrupted; no automatic motor resume'
            raise
        except Exception as exc:
            reason = 'uncertain_execution' if self.in_flight else 'host_error'
            detail = f'{type(exc).__name__}: {exc}'
        finally:
            result = self.finish(reason, detail)
        return result

    def finish(self, reason, detail=''):
        result = {'status': reason, 'detail': detail, 'robot_id': self.robot_id,
                  'task': self.instruction, 'steps': self.steps, 'action_attempts': self.actions,
                  'observations': self.record.sequence, 'elapsed_s': round(self.clock() - self.started, 3),
                  'completion_claim': self.claim, 'physical_success_verified': False,
                  'pending_action_id': self.in_flight, 'automatic_resume': False,
                  'run_dir': str(self.record.path)}
        self.record.event('episode_finished', result)
        atomic_json(self.record.path / 'result.json', result)
        return result

    async def execute(self, decision):
        if self.actions >= self.limits.max_actions:
            raise EpisodeStopped('budget_actions')
        command = decision.command.wire()
        signature = json.dumps(command, sort_keys=True)
        if signature == self.rejected_signature:
            raise EpisodeStopped('repeated_failed_action', 'Change the rejected/failed command after inspecting feedback')
        self.check_time()
        if command['duration_ms'] / 1000 >= self.remaining():
            raise EpisodeStopped('budget_seconds', 'Insufficient time for the requested action')
        before = self.packet
        action_id = uuid4().hex
        arguments = {**command, 'observation_id': decision.observation_id, 'runtime_id': self.runtime_id,
                     'action_id': action_id, 'reason': decision.reason}
        # Journal before dispatch; an interrupted record never authorizes a replay.
        self.record.event('action_prepared', {'robot_id': self.robot_id, **arguments})
        self.actions += 1
        self.in_flight = action_id
        packet, images = decode_content(await self.bounded(lambda: self.tools.execute(self.robot_id, **arguments)))
        self.record.event('action_result', {'action_id': action_id, 'packet': packet})
        execution = packet.get('execution') or {}
        if execution.get('action_id') != action_id:
            raise EpisodeStopped('uncertain_execution', 'Missing or mismatched execution action_id')
        status, sent = packet.get('status'), execution.get('command_sent')
        if status == 'rejected' and sent is False and execution.get('outcome') == 'rejected':
            self.in_flight = None
            self.previous_result = packet
            self.rejected_signature = signature
            await self.observe()
            return
        if status == 'completed_without_observation' and execution.get('outcome') == 'completed':
            self.in_flight = None
            raise EpisodeStopped('post_motion_observation_failed', 'Action completed; fresh observation unavailable')
        if status != 'completed' or type(sent) is not bool or execution.get('outcome') != 'completed':
            raise EpisodeStopped('uncertain_execution', f'Execution {status}: observation or result not confirmed')
        self.in_flight = None
        self.previous_result = packet
        self.before_images = self.images
        self._check_post_motion(packet, command, execution)
        self._accept_observation(packet, images, previous_id=decision.observation_id)
        self.pending_feedback = self._feedback(before, self.packet, command, execution)
        self.record.event('motion_feedback', self.pending_feedback)
        self.rejected_signature = signature if self.pending_feedback['no_joint_progress'] else None

    @staticmethod
    def _check_post_motion(packet, command, execution):
        if command['action'] != 'move':
            return
        cameras = (packet.get('observation') or {}).get('cameras') or {}
        required = []
        if any(command.get(axis, 0) for axis in ('x', 'y', 'theta')):
            required.append('front')
        if command.get('joints'):
            required.append('wrist')
        barrier = execution.get('motion_finished_at')
        if required and (type(barrier) not in (int, float) or not math.isfinite(barrier) or barrier <= 0):
            raise EpisodeStopped('post_motion_observation_failed', 'Missing runtime motion timestamp')
        for name in required:
            camera = cameras.get(name) or {}
            stamp = camera.get('timestamp')
            if (camera.get('available') is not True or type(stamp) not in (int, float) or
                    not math.isfinite(stamp) or stamp <= barrier):
                raise EpisodeStopped('post_motion_observation_failed', f'{name} image is not newer than motion completion')

    @staticmethod
    def _feedback(before, after, command, execution):
        old = before['observation'].get('ego', {}).get('joints', {})
        new = after['observation'].get('ego', {}).get('joints', {})
        targets = command.get('joints') or {}
        delta = {key: new[key] - old[key] for key in targets
                 if type(old.get(key)) in (float, int) and type(new.get(key)) in (float, int)}
        needs_motion = any(abs(value - old.get(key, value)) > (5 if key == 'gripper' else 2)
                           for key, value in targets.items())
        no_progress = bool(targets and needs_motion and len(delta) == len(targets)
                           and all(abs(value) < .1 for value in delta.values()))
        return {'execution': execution, 'joint_delta': delta, 'no_joint_progress': no_progress,
                'before_observation_id': before['observation']['observation_id'],
                'after_observation_id': after['observation']['observation_id'],
                'note': 'Compare current and before-motion images. Base command duration is not measured travel; '
                        'joint arrival and gripper closure do not prove a grasp. IMU displacement can drift.'}
