"""One host-owned decision/action/observation loop; never replay motor commands."""
from __future__ import annotations

import asyncio
import base64
from contextlib import asynccontextmanager
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

from .tools import ARM_MAX_DURATION_MS, ARM_MIN_DURATION_MS, build_command


MAX_ARM_RECONCILIATIONS = 2
MAX_ARM_WARNING_ERROR_DEG = 5.
RECOVERY_ARM_JOINTS = {'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'}


class JointTargets(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    shoulder_pan: float | None
    shoulder_lift: float | None
    elbow_flex: float | None
    wrist_flex: float | None
    wrist_roll: float | None
    gripper: float | None


class CartesianTarget(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    frame: Literal['arm_base']
    mode: Literal['absolute', 'relative']
    xyz_m: list[float] = Field(min_length=3, max_length=3)
    approach_axis: list[float] | None = Field(min_length=3, max_length=3)


class Command(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    action: Literal['move', 'wait']
    x: float
    y: float
    theta: float
    duration_ms: int = Field(ge=1, le=ARM_MAX_DURATION_MS,
                             description='Arm-only targets: 100..10000ms; base, wait or combined motion: at most 1000ms')
    joints: JointTargets | None
    cartesian: CartesianTarget | None = None

    @model_validator(mode='after')
    def motion_duration_matches_command(self):
        if self.cartesian is not None and (self.action != 'move' or self.joints is not None
                                          or any((self.x, self.y, self.theta))
                                          or not ARM_MIN_DURATION_MS <= self.duration_ms <= ARM_MAX_DURATION_MS):
            raise ValueError('Cartesian move requires zero base velocity, joints=null and duration_ms=100..10000')
        if self.cartesian is None:
            joints = self.joints.model_dump(exclude_none=True) if self.joints else None
            build_command(self.action, self.x, self.y, self.theta, self.duration_ms, joints)
        return self

    def request(self):
        if self.cartesian is not None:
            return {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': self.duration_ms,
                    'cartesian': self.cartesian.model_dump()}
        return self.wire()

    def wire(self):
        if self.cartesian is not None:
            raise ValueError('Cartesian targets must be resolved by the host before motor dispatch')
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
        self.event_sequence = 0
        self.started_monotonic = time.monotonic()
        self.events = self.path / 'events.jsonl'
        atomic_json(self.path / 'run.json', {
            'version': 1, 'robot_id': robot_id, 'instruction': instruction,
            'started_at': datetime.now(timezone.utc).isoformat(), 'limits': asdict(limits),
            'mode': 'execute', 'automatic_resume': False,
        })
        atomic_json(self.path / 'result.json', {'status': 'running', 'automatic_resume': False})

    def event(self, kind, data):
        self.event_sequence += 1
        with self.events.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'at': datetime.now(timezone.utc).isoformat(),
                                     'seq': self.event_sequence,
                                     'elapsed_s': round(time.monotonic() - self.started_monotonic, 3),
                                     'event': kind,
                                     'data': data}, ensure_ascii=False, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())

    def observation(self, response, images):
        self.sequence += 1
        directory = self.path / 'observations' / f'{self.sequence:04d}'
        directory.mkdir()
        atomic_json(directory / 'observation.json', response)
        saved_images = []
        for index, (label, data, mime) in enumerate(images):
            suffix = '.jpg' if mime == 'image/jpeg' else '.png'
            image_path = directory / f'image-{index:02d}{suffix}'
            with image_path.open('wb') as stream:
                stream.write(base64.b64decode(data, validate=True))
                stream.flush()
                os.fsync(stream.fileno())
            saved_images.append({'label': label, 'path': str(image_path.relative_to(self.path)),
                                 'mime_type': mime})
        observation_id = response['observation']['observation_id']
        atomic_json(directory / 'images.json', {'observation_id': observation_id, 'images': saved_images})
        self.event('observation', {'path': str(directory.relative_to(self.path)),
                                  'observation_id': observation_id, 'images': saved_images})


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

    async def reconcile(self, robot_id, *, runtime_id, failed_action_id):
        return await self._call('reconcile', {'robot_id': robot_id, 'runtime_id': runtime_id,
                                             'failed_action_id': failed_action_id})


class EpisodeRunner:
    """Testable loop: planner.decide(context, images), and observe/execute tools.

    Budgets and observation identities are checked by the host. A record is inspection-only: a
    previously written action is never automatically replayed after restart.
    """
    def __init__(self, planner, tools, robot_id, instruction, run_dir, limits=None, *, clock=time.monotonic,
                 cartesian=None, allow_reconciliation=True):
        self.planner, self.tools = planner, tools
        self.cartesian = cartesian
        self.geometry = {'available': False, 'executable': False, 'frame': 'arm_base',
                         'reasons': ['Cartesian controller is unavailable'],
                         'reason_codes': ['cartesian_unavailable']}
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
        self.previous_ik_result = None
        self.pending_feedback = None
        self.before_images = []
        self.completion_candidate = None
        self.claim = None
        self.in_flight = None
        self.rejected_signature = None
        self.rejected_commands = {}
        self.calibration_rejections = {}
        self.rejection_feedback = None
        self.repeat_block_count = 0
        self.allow_reconciliation = allow_reconciliation
        self.reconciliation_attempts = 0
        self.reconciled_action_ids = set()
        self.seen_observation_ids = set()
        self.motion_failure_feedback = None
        self.recovery_constraints = None
        self.failed_motion_targets = set()

    def remaining(self):
        return self.limits.max_seconds - (self.clock() - self.started)

    def check_time(self):
        if self.remaining() <= 0:
            raise EpisodeStopped('budget_seconds')

    async def bounded(self, call):
        self.check_time()
        async with asyncio.timeout(self.remaining()):
            return await call()

    @asynccontextmanager
    async def phase(self, name, **metadata):
        """Persist public lifecycle progress without observing or calling the model.

        The yielded dict can add result metadata, never model reasoning streams.
        A file reader can show an active phase before its awaited work finishes.
        """
        started = self.clock()
        data = dict(metadata)
        self.record.event(name + '_started', {
            **data, 'steps': self.steps, 'action_attempts': self.actions,
            'observations': self.record.sequence,
        })
        status = 'completed'
        try:
            yield data
        except BaseException as exc:
            status = 'cancelled' if isinstance(exc, asyncio.CancelledError) else 'error'
            data['error'] = exc.reason if isinstance(exc, EpisodeStopped) else type(exc).__name__
            raise
        finally:
            if name == 'action':
                # A finished RPC is not proof of a successful robot action. Keep
                # transport lifecycle separate from the validated runtime result.
                if data.get('transport_status') == 'pending':
                    data['transport_status'] = status
                if status == 'completed':
                    status = data.get('result_status', status)
            self.record.event(name + '_finished', {
                **data, 'status': status, 'duration_s': round(max(0, self.clock() - started), 3),
                'steps': self.steps, 'action_attempts': self.actions,
                'observations': self.record.sequence,
            })

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
        incoming_constraints = context.get('recovery_constraints')
        if incoming_constraints is not None:
            recovery_capability = (packet.get('capabilities') or {}).get('arm_reconciliation')
            if (type(recovery_capability) is not int or recovery_capability != 1
                    or not self._valid_recovery_constraints(incoming_constraints)
                    or incoming_constraints.get('runtime_id', runtime_id) != runtime_id):
                raise EpisodeStopped('reconciliation_failed', 'Invalid runtime recovery constraints')
            if self.recovery_constraints is None:
                self.recovery_constraints = dict(incoming_constraints)
                self.reconciliation_attempts = max(self.reconciliation_attempts,
                                                   MAX_ARM_RECONCILIATIONS - incoming_constraints['remaining'])
                self.motion_failure_feedback = {
                    'resolved': False, 'failed_action_id': incoming_constraints.get('failed_action_id'),
                    'runtime_id': runtime_id, 'failed_command': None, 'arm_motion': {},
                    'recovery_constraints': dict(incoming_constraints),
                    'measured_joints': (obs.get('ego') or {}).get('joints'),
                    'guidance': 'The runtime retains an unresolved prior arm motion. Reinspect the current pose '
                                'and propose a small revised arm action; do not claim completion.',
                }
                self.record.event('motion_failure_feedback', self.motion_failure_feedback)
        if self.recovery_constraints is not None:
            finished_recovery = (packet.get('status') == 'completed'
                                 and (packet.get('execution') or {}).get('arm_reached') is True)
            if not finished_recovery and incoming_constraints != self.recovery_constraints:
                raise EpisodeStopped('reconciliation_failed', 'Recovery constraints changed or disappeared before a verified move')
        if observation_id in self.seen_observation_ids or previous_id is not None and observation_id == previous_id:
            raise EpisodeStopped('observation_invalid', 'A new observation is required')
        self.seen_observation_ids.add(observation_id)
        self.runtime_id = runtime_id
        self.packet, self.images, self.received_at = packet, images, self.clock()
        self._clear_recovered_calibration()
        self.record.observation(packet, images)
        if self.cartesian is not None:
            try:
                self.geometry = self.cartesian.evaluate(packet)
            except Exception as exc:
                self.geometry = {'available': False, 'executable': False, 'frame': 'arm_base',
                                 'reasons': [f'{type(exc).__name__}: {exc}'], 'reason_codes': ['fk_error']}
            self.record.event('kinematics_state', {**self.geometry, 'observation_id': observation_id})

    def _clear_recovered_calibration(self):
        """A fresh encoder sample may clear a restriction; target changes never do."""
        ego = self.packet['observation'].get('ego') or {}
        if ego.get('available') is not True:
            return
        raw = ego.get('raw') or {}
        recovered = set()
        for joint, failure in list(self.calibration_rejections.items()):
            detail = failure['error_details']
            values = (raw.get(joint), detail.get('raw_min'), detail.get('raw_max'))
            if (all(type(value) in (int, float) and math.isfinite(value) for value in values)
                    and values[1] <= values[0] <= values[2]):
                del self.calibration_rejections[joint]
                recovered.add(joint)
        for signature, failure in list(self.rejected_commands.items()):
            if (failure.get('error_code') == 'arm_current_out_of_calibration'
                    and failure['error_details'].get('joint') in recovered):
                del self.rejected_commands[signature]

    def _fresh(self):
        return (self.packet is not None and
                self.clock() - self.received_at < self.packet['action_context']['max_age_s'])

    async def observe(self):
        previous_id = self.packet['observation']['observation_id'] if self.packet else None
        async with self.phase('observe', previous_observation_id=previous_id) as phase:
            packet, images = decode_content(await self.bounded(lambda: self.tools.observe(self.robot_id)))
            if packet.get('status') != 'ok':
                raise EpisodeStopped('observation_failed', str(packet.get('error', packet.get('status'))))
            self._accept_observation(packet, images, previous_id=previous_id)
            phase['observation_id'] = self.packet['observation']['observation_id']

    def _context(self):
        return {
            'robot_id': self.robot_id, 'task': self.instruction, 'current': self.packet,
            'previous_decision': self.previous_decision, 'previous_result': self.previous_result,
            'pending_motion_feedback': self.pending_feedback,
            'motion_failure_feedback': self.motion_failure_feedback,
            'recovery_constraints': self.recovery_constraints,
            'rejection_feedback': ({**self.rejection_feedback,
                                    'current_observation_id': self.packet['observation']['observation_id']}
                                   if self.rejection_feedback else None),
            'calibration_restrictions': {joint: failure['error_details']
                                         for joint, failure in self.calibration_rejections.items()},
            'cartesian': self.geometry,
            'previous_ik_result': self.previous_ik_result,
            'completion_candidate': self.completion_candidate,
            'budget': {'actions_remaining': self.limits.max_actions - self.actions,
                       'decisions_remaining': self.limits.max_steps - self.steps,
                       'arm_reconciliations_remaining': MAX_ARM_RECONCILIATIONS - self.reconciliation_attempts,
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
                async with self.phase('planning', step=self.steps + 1,
                                      observation_id=self.packet['observation']['observation_id']):
                    raw = await self.bounded(lambda: self.planner.decide(context, image_input))
                    self.steps += 1
                    try:
                        decision = Decision.model_validate(raw)
                        if decision.command is not None:
                            decision.command.request()
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
                    if self.motion_failure_feedback and self.motion_failure_feedback.get('resolved') is not True:
                        self.completion_candidate = None
                        self.record.event('decision_rejected', {
                            'reason': 'incomplete_motion_requires_replan',
                            'detail': 'Partial arm motion is not completion. Inspect measured state and plan a revised action.',
                        })
                        await self.observe()
                        continue
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
                  'rejection_feedback': self.rejection_feedback,
                  'motion_failure_feedback': self.motion_failure_feedback,
                  'reconciliation_attempts': self.reconciliation_attempts,
                  'pending_action_id': self.in_flight, 'automatic_resume': False,
                  'run_dir': str(self.record.path)}
        self.record.event('episode_finished', result)
        atomic_json(self.record.path / 'result.json', result)
        return result

    async def execute(self, decision):
        if self.actions >= self.limits.max_actions:
            raise EpisodeStopped('budget_actions')
        command = decision.command.request()
        requested_command = command
        if not await self._recovery_command_allowed(command, resolved=False):
            return
        if decision.command.cartesian is not None:
            request_signature = json.dumps(command, sort_keys=True)
            if request_signature in self.rejected_commands:
                await self._reject_repeated(command, self.rejected_commands[request_signature], decision.observation_id)
                return
            command = await self._resolve_cartesian(decision, command)
            if command is None:
                return
        if not await self._recovery_command_allowed(command, resolved=True):
            return
        signature = json.dumps(command, sort_keys=True)
        if signature == self.rejected_signature:
            raise EpisodeStopped('repeated_failed_action', 'Change the rejected/failed command after inspecting feedback')
        failure = next((failure for joint, failure in self.calibration_rejections.items()
                        if joint in (command.get('joints') or {})), None)
        failure = failure or self.rejected_commands.get(signature)
        if failure is not None:
            await self._reject_repeated(command, failure, decision.observation_id)
            return
        self.check_time()
        if command['duration_ms'] / 1000 >= self.remaining():
            raise EpisodeStopped('budget_seconds', 'Insufficient time for the requested action')
        if not self._fresh():
            self.record.event('decision_rejected', {'reason': 'observation_expired_before_dispatch'})
            self.previous_result = {'status': 'rejected', 'reason': 'Observation expired while preparing the action'}
            await self.observe()
            return
        before = self.packet
        action_id = uuid4().hex
        arguments = {**command, 'observation_id': decision.observation_id, 'runtime_id': self.runtime_id,
                     'action_id': action_id, 'reason': decision.reason}
        # Journal before dispatch; an interrupted record never authorizes a replay.
        self.record.event('action_prepared', {'robot_id': self.robot_id, **arguments})
        self.actions += 1
        self.in_flight = action_id
        async with self.phase('action', action_id=action_id, observation_id=decision.observation_id,
                              action_attempt=self.actions, transport_status='pending') as phase:
            content = await self.bounded(lambda: self.tools.execute(self.robot_id, **arguments))
            phase['transport_status'] = 'completed'
            packet, images = decode_content(content)
            self.record.event('action_result', {'action_id': action_id, 'packet': packet})
            phase['result_status'] = packet.get('status')
            phase['result_error'] = packet.get('error') or packet.get('observation_error')
            execution = packet.get('execution')
            if not isinstance(execution, dict) or execution.get('action_id') != action_id:
                raise EpisodeStopped('uncertain_execution', 'Missing or mismatched execution action_id')
            status, sent = packet.get('status'), execution.get('command_sent')
            phase['execution_outcome'], phase['command_sent'] = execution.get('outcome'), sent
            safe_rejection = status == 'rejected' and sent is False and execution.get('outcome') == 'rejected'
            confirmed = (status in ('completed', 'completed_without_observation') and type(sent) is bool
                         and execution.get('outcome') == 'completed')
            incomplete = status == 'incomplete' and self._verified_incomplete(packet, action_id, command)
            warning = (status in ('warning', 'warning_without_observation')
                       and self._verified_motion_warning(packet, action_id, command))
            if not safe_rejection and not confirmed and not incomplete and not warning:
                raise EpisodeStopped('uncertain_execution', f'Execution {status}: observation or result not confirmed')
            if confirmed and self.recovery_constraints is not None and execution.get('arm_reached') is not True:
                raise EpisodeStopped('uncertain_execution', 'Revised arm motion has no confirmed joint arrival')
        if incomplete:
            await self._reconcile_incomplete(packet, before, command, requested_command, action_id)
            return
        if warning:
            self._accept_motion_warning(packet, images, before, command, requested_command, action_id)
            return
        if status == 'rejected' and sent is False and execution.get('outcome') == 'rejected':
            self.in_flight = None
            self.previous_result = packet
            details = packet.get('error_details')
            details = details if isinstance(details, dict) else {}
            self.rejection_feedback = {
                'source': 'runtime', 'action_id': action_id, 'command': command,
                'error': packet.get('error', 'Runtime rejected the command before dispatch'),
                'error_code': packet.get('error_code'), 'error_details': details,
                'command_sent': False, 'failed_observation_id': decision.observation_id,
                'current_observation_id': None, 'repeat_block_count': self.repeat_block_count,
                'guidance': 'No command was sent. Reinspect current observations and revise the failed '
                            'command within the existing limits. Do not repeat it unchanged or relax '
                            'calibration/speed limits. Use blocked if no feasible alternative remains.',
            }
            self.rejected_commands[signature] = self.rejection_feedback
            joint = details.get('joint')
            if packet.get('error_code') == 'arm_current_out_of_calibration' and isinstance(joint, str):
                self.calibration_rejections[joint] = self.rejection_feedback
            await self._reobserve_rejection()
            return
        if status == 'completed_without_observation' and execution.get('outcome') == 'completed':
            self.in_flight = None
            raise EpisodeStopped('post_motion_observation_failed', 'Action completed; fresh observation unavailable')
        self.in_flight = None
        self.previous_result = packet
        self.before_images = self.images
        self._check_post_motion(packet, command, execution)
        self._accept_observation(packet, images, previous_id=decision.observation_id)
        self.pending_feedback = self._feedback(before, self.packet, command, execution)
        self.record.event('motion_feedback', self.pending_feedback)
        if self.recovery_constraints is not None:
            retained = packet['action_context'].get('recovery_constraints')
            if retained is not None:
                self.recovery_constraints = dict(retained)
                self.motion_failure_feedback = {
                    **self.motion_failure_feedback, 'resolved': False,
                    'current_observation_id': packet['observation']['observation_id'],
                    'measured_joints': (packet['observation'].get('ego') or {}).get('joints'),
                    'last_revised_action_id': action_id, 'recovery_constraints': dict(retained),
                    'guidance': 'The revised endpoint was reached but the runtime has not verified enough '
                                'target-directed progress. Keep the small arm-only constraints and replan; '
                                'do not claim the prior failure resolved.',
                }
                self.record.event('motion_failure_feedback', self.motion_failure_feedback)
                self.record.event('motion_replanning', self.motion_failure_feedback)
            else:
                if execution.get('arm_recovery_progress_verified') is not True:
                    raise EpisodeStopped('reconciliation_failed', 'Runtime released recovery without verified measured progress')
                self.recovery_constraints = None
                self.motion_failure_feedback = {**self.motion_failure_feedback, 'resolved': True,
                                                'resolved_by_action_id': action_id,
                                                'resolution': 'Revised arm target reached with verified measured progress and a fresh observation'}
                self.record.event('motion_recovery_completed', self.motion_failure_feedback)
        self.rejected_signature = signature if self.pending_feedback['no_joint_progress'] else None
        if command['action'] == 'move' and not self.pending_feedback['no_joint_progress']:
            self.rejected_commands.clear()
            self.rejection_feedback = None
            self.repeat_block_count = 0

    @staticmethod
    def _finite_number(value):
        try:
            return type(value) in (int, float) and math.isfinite(value)
        except OverflowError:
            return False

    def _valid_hold(self, hold, calibration_id):
        names = RECOVERY_ARM_JOINTS | {'gripper'}
        return (isinstance(hold, dict) and hold.get('verified') is True
                and isinstance(calibration_id, str) and bool(calibration_id)
                and hold.get('calibration_id') == calibration_id
                and self._finite_number(hold.get('timestamp')) and hold['timestamp'] > 0
                and isinstance(hold.get('measured'), dict) and set(hold['measured']) == names
                and all(self._finite_number(value) for value in hold['measured'].values()))

    def _valid_recovery_constraints(self, constraints):
        return (isinstance(constraints, dict)
                and type(constraints.get('remaining')) is int
                and 0 <= constraints['remaining'] < MAX_ARM_RECONCILIATIONS
                and self._finite_number(constraints.get('max_joint_delta_deg'))
                and 0 < constraints['max_joint_delta_deg'] <= 3.
                and type(constraints.get('min_duration_ms')) is int
                and 3000 <= constraints['min_duration_ms'] <= ARM_MAX_DURATION_MS)

    def _verified_motion_warning(self, packet, action_id, command):
        """Accept only a typed, bounded endpoint error with a verified measured hold."""
        execution = packet.get('execution') or {}
        capabilities = packet.get('capabilities') or {}
        motion, warning = execution.get('arm_motion'), execution.get('warning')
        targets = command.get('joints')
        ego = self.packet['observation'].get('ego') or {}
        calibration_id = ego.get('calibration_id')
        if (any(type(capabilities.get(name)) is not int or capabilities[name] != 1
                for name in ('guarded_execution', 'arm_motion_warning'))
                or execution.get('action_id') != action_id or execution.get('outcome') != 'warning'
                or execution.get('command_sent') is not True or execution.get('arm_reached') is not False
                or execution.get('arm_hold_verified') is not True
                or command.get('action') != 'move' or any(command.get(axis) for axis in ('x', 'y', 'theta'))
                or not isinstance(targets, dict) or not targets or not set(targets) <= RECOVERY_ARM_JOINTS
                or not isinstance(motion, dict) or motion.get('targets') != targets
                or motion.get('reached') is not False or motion.get('warning_eligible') is not True
                or motion.get('communication_recoveries') != []
                or ego.get('available') is not True or ego.get('error')
                or (ego.get('calibration_status') or {}).get('valid') is not True
                or not self._valid_hold(motion.get('hold'), calibration_id)
                or not self._finite_number(execution.get('motion_finished_at'))
                or execution['motion_finished_at'] < motion['hold']['timestamp']
                or not isinstance(warning, dict) or warning.get('code') != 'arm_target_not_reached'
                or not self._finite_number(warning.get('max_error_deg'))
                or not 0 < warning['max_error_deg'] <= MAX_ARM_WARNING_ERROR_DEG):
            return False
        measured, errors = motion.get('measured'), motion.get('errors')
        limit = warning['max_error_deg']
        if (not isinstance(measured, dict) or set(measured) != set(targets)
                or not isinstance(errors, dict) or set(errors) != set(targets)
                or any(not self._finite_number(measured[name]) or not self._finite_number(errors[name])
                       or abs(errors[name]) > limit
                       or abs(measured[name] - target - errors[name]) > 1e-6
                       or abs(motion['hold']['measured'][name] - target) > limit
                       for name, target in targets.items())):
            return False
        return True

    def _accept_motion_warning(self, packet, images, before, command, requested_command, action_id):
        # A warning is a known partial result, not confirmed arrival. It authorizes
        # planning only after independent fresh camera and measured-pose checks.
        self.in_flight = None
        self.previous_result = packet
        self.completion_candidate = None
        if packet.get('status') == 'warning_without_observation':
            raise EpisodeStopped('post_motion_observation_failed',
                                 'Known arm endpoint warning; fresh observation unavailable')
        execution = packet['execution']
        motion, warning = execution['arm_motion'], execution['warning']
        hold = motion['hold']
        observation = packet.get('observation') or {}
        ego = observation.get('ego') or {}
        joints = ego.get('joints')
        names = RECOVERY_ARM_JOINTS | {'gripper'}
        if (ego.get('available') is not True or ego.get('error')
                or ego.get('calibration_id') != hold['calibration_id']
                or (ego.get('calibration_status') or {}).get('valid') is not True
                or not self._finite_number(ego.get('timestamp'))
                or ego['timestamp'] <= execution['motion_finished_at']
                or not isinstance(joints, dict) or not names <= set(joints)
                or not all(self._finite_number(joints[name]) for name in names)
                or any(abs(joints[name] - hold['measured'][name]) > (2. if name == 'gripper' else .5)
                       for name in names)
                or any(abs(joints[name] - target) > warning['max_error_deg']
                       for name, target in command['joints'].items())
                or not any(label == 'wrist' for label, _, _ in images)):
            raise EpisodeStopped('post_motion_observation_failed',
                                 'Arm warning requires fresh calibrated feedback consistent with its verified hold')
        self._check_post_motion(packet, command, execution)
        self.before_images = self.images
        self._accept_observation(packet, images, previous_id=before['observation']['observation_id'])
        errors = {name: joints[name] - target for name, target in command['joints'].items()}
        self.pending_feedback = {
            **self._feedback(before, packet, command, execution),
            'motion_warning': True, 'arm_reached': False, 'warning': warning,
            'targets': command['joints'], 'measured_joints': joints, 'joint_errors': errors,
            'guidance': 'The arm stopped at a verified measured pose with a small target error. '
                        'Plan the next action from these actual joints and current images. '
                        'Do not assume target arrival, replay a queued motion, or claim task success from this warning.',
        }
        self.record.event('motion_feedback', self.pending_feedback)
        self.record.event('motion_warning', {
            'action_id': action_id, 'runtime_id': self.runtime_id,
            'command': command, 'requested_command': requested_command,
            'execution': execution, 'arm_motion': motion, 'warning': warning,
            'targets': command['joints'], 'measured_joints': joints, 'errors': errors,
            'current_observation_id': observation['observation_id'],
            'recovery_constraints': self.recovery_constraints,
            'guidance': self.pending_feedback['guidance'],
        })
        if self.recovery_constraints is not None:
            self.motion_failure_feedback = {
                **self.motion_failure_feedback, 'resolved': False,
                'current_observation_id': observation['observation_id'], 'measured_joints': joints,
                'last_revised_action_id': action_id, 'last_warning': warning,
                'guidance': 'The revised arm motion ended with a known small target error. '
                            'Retain the existing recovery constraints and replan from measured state; '
                            'this warning does not resolve the prior incomplete motion.',
            }
            self.record.event('motion_failure_feedback', self.motion_failure_feedback)
        self.rejected_signature = (json.dumps(command, sort_keys=True)
                                   if self.pending_feedback['no_joint_progress'] else None)
        if not self.pending_feedback['no_joint_progress']:
            self.rejected_commands.clear()
            self.rejection_feedback = None
            self.repeat_block_count = 0

    def _verified_incomplete(self, packet, action_id, command):
        """Recognize one typed, held, known partial motion; never infer from error text."""
        execution = packet.get('execution') or {}
        capability = (packet.get('capabilities') or {}).get('arm_reconciliation')
        guarded = (packet.get('capabilities') or {}).get('guarded_execution')
        recovery, context = packet.get('recovery'), packet.get('action_context')
        motion = execution.get('arm_motion')
        targets = command.get('joints')
        calibration_id = (self.packet['observation'].get('ego') or {}).get('calibration_id')
        if (type(capability) is not int or capability != 1 or type(guarded) is not int or guarded != 1
                or execution.get('action_id') != action_id
                or execution.get('outcome') != 'incomplete' or execution.get('command_sent') is not True
                or execution.get('arm_reached') is not False or execution.get('arm_hold_verified') is not True
                or command.get('action') != 'move' or any(command.get(axis) for axis in ('x', 'y', 'theta'))
                or not isinstance(targets, dict) or not targets or not set(targets) <= RECOVERY_ARM_JOINTS
                or not isinstance(motion, dict) or motion.get('targets') != targets
                or motion.get('reached') is not False or motion.get('recovery_eligible') is not True
                or motion.get('communication_recoveries') != []
                or not self._valid_hold(motion.get('hold'), calibration_id)
                or not self._finite_number(execution.get('motion_finished_at'))
                or execution['motion_finished_at'] < motion['hold']['timestamp']
                or not isinstance(recovery, dict) or recovery.get('failed_action_id') != action_id
                or recovery.get('runtime_id') != self.runtime_id
                or type(recovery.get('attempts_remaining')) is not int
                or not 0 <= recovery['attempts_remaining'] <= MAX_ARM_RECONCILIATIONS
                or not isinstance(context, dict) or context.get('runtime_id') != self.runtime_id
                or context.get('movement_allowed') is not False
                or context.get('reason') != 'arm_reconciliation_required'):
            return False
        return True

    async def _reconcile_incomplete(self, packet, before, command, requested_command, action_id):
        self.in_flight = None  # The exact known partial result and measured hold were verified above.
        self.previous_result = packet
        self.before_images = self.images
        self.completion_candidate = None
        execution, recovery = packet['execution'], packet['recovery']
        self.failed_motion_targets.update((json.dumps(command, sort_keys=True),
                                           json.dumps(requested_command, sort_keys=True)))
        self.motion_failure_feedback = {
            'resolved': False, 'failed_action_id': action_id, 'runtime_id': self.runtime_id,
            'failed_command': command, 'failed_request': requested_command,
            'execution': execution, 'arm_motion': execution['arm_motion'],
            'measured_joints': execution['arm_motion']['hold']['measured'],
            'reconciliation_attempts': self.reconciliation_attempts,
            'guidance': 'Known partial motion with a verified held pose. Do not claim completion or replay '
                        'the failed command. Reinspect fresh feedback and propose one revised bounded arm action.',
        }
        self.record.event('motion_incomplete', self.motion_failure_feedback)
        if not self.allow_reconciliation:
            raise EpisodeStopped('incomplete', '목표에 도달하지 못했으며 측정 자세 유지를 확인했습니다. 새 관찰로 재계획이 필요합니다.')
        if (self.reconciliation_attempts >= MAX_ARM_RECONCILIATIONS or recovery['attempts_remaining'] <= 0
                or recovery.get('eligible') is not True or action_id in self.reconciled_action_ids):
            raise EpisodeStopped('recovery_exhausted', 'Bounded arm reconciliation is unavailable or exhausted')
        if self.actions >= self.limits.max_actions:
            raise EpisodeStopped('budget_actions', 'No action budget remains for a revised arm command')
        if self.steps >= self.limits.max_steps:
            raise EpisodeStopped('budget_steps', 'No decision budget remains for arm replanning')
        if self.remaining() <= 3.:
            raise EpisodeStopped('budget_seconds', 'Insufficient time for a constrained revised arm action')
        self.reconciliation_attempts += 1
        self.reconciled_action_ids.add(action_id)
        previous_id = before['observation']['observation_id']
        async with self.phase('reconcile', failed_action_id=action_id,
                              reconciliation_attempt=self.reconciliation_attempts) as phase:
            try:
                response, images = decode_content(await self.bounded(lambda: self.tools.reconcile(
                    self.robot_id, runtime_id=self.runtime_id, failed_action_id=action_id)))
            except (asyncio.CancelledError, EpisodeStopped, TimeoutError):
                raise
            except Exception as exc:
                raise EpisodeStopped('reconciliation_failed', f'Fresh held-state verification failed: {exc}') from exc
            self.record.event('reconcile_result', {'failed_action_id': action_id, 'packet': response})
            constraints = self._validate_reconciled(response, execution, recovery, images)
            self._check_post_motion(response, command, execution)
            self.recovery_constraints = constraints
            self._accept_observation(response, images, previous_id=previous_id)
            self.pending_feedback = {**self._feedback(before, response, command, execution),
                                     'motion_incomplete': True, 'arm_reached': False}
            self.record.event('motion_feedback', self.pending_feedback)
            self.motion_failure_feedback = {
                **self.motion_failure_feedback, 'current_observation_id': self.packet['observation']['observation_id'],
                'measured_joints': self.packet['observation']['ego']['joints'],
                'recovery_constraints': dict(constraints), 'reconciliation_attempts': self.reconciliation_attempts,
            }
            self.record.event('motion_failure_feedback', self.motion_failure_feedback)
            self.record.event('motion_replanning', {'failed_action_id': action_id,
                                                   'recovery_constraints': dict(constraints)})
            phase['observation_id'] = self.packet['observation']['observation_id']

    def _validate_reconciled(self, response, execution, prior_recovery, images):
        recovery, context = response.get('recovery'), response.get('action_context')
        capability = (response.get('capabilities') or {}).get('arm_reconciliation')
        if (response.get('status') != 'ok' or type(capability) is not int or capability != 1
                or not isinstance(recovery, dict) or recovery.get('reconciled') is not True
                or recovery.get('failed_action_id') != prior_recovery['failed_action_id']
                or recovery.get('runtime_id') != self.runtime_id or not isinstance(context, dict)
                or context.get('runtime_id') != self.runtime_id):
            raise EpisodeStopped('reconciliation_failed', 'Reconciliation identity or result was not verified')
        constraints = context.get('recovery_constraints')
        remaining = recovery.get('attempts_remaining')
        if (not isinstance(constraints, dict) or type(remaining) is not int
                or remaining != prior_recovery['attempts_remaining'] - 1
                or not 0 <= remaining < MAX_ARM_RECONCILIATIONS
                or type(constraints.get('remaining')) is not int or constraints['remaining'] != remaining
                or not self._finite_number(constraints.get('max_joint_delta_deg'))
                or not 0 < constraints['max_joint_delta_deg'] <= 3.
                or type(constraints.get('min_duration_ms')) is not int
                or not 3000 <= constraints['min_duration_ms'] <= ARM_MAX_DURATION_MS):
            raise EpisodeStopped('reconciliation_failed', 'Reconciliation did not provide bounded arm constraints')
        observation = response.get('observation') or {}
        ego = observation.get('ego') or {}
        joints = ego.get('joints')
        hold = recovery.get('hold')
        prior_hold = execution['arm_motion']['hold']
        wrist = (observation.get('cameras') or {}).get('wrist') or {}
        if (not self._valid_hold(hold, prior_hold['calibration_id'])
                or hold['timestamp'] <= execution['motion_finished_at']
                or ego.get('available') is not True or ego.get('error')
                or ego.get('calibration_id') != prior_hold['calibration_id']
                or (ego.get('calibration_status') or {}).get('valid') is not True
                or not self._finite_number(ego.get('timestamp')) or ego['timestamp'] <= hold['timestamp']
                or not isinstance(joints, dict) or not (RECOVERY_ARM_JOINTS | {'gripper'}) <= set(joints)
                or not all(self._finite_number(joints[name]) for name in RECOVERY_ARM_JOINTS | {'gripper'})
                or any(abs(joints[name] - hold['measured'][name]) > (2. if name == 'gripper' else .5)
                       for name in RECOVERY_ARM_JOINTS | {'gripper'})
                or not self._finite_number(wrist.get('timestamp')) or wrist['timestamp'] <= hold['timestamp']
                or not any(label == 'wrist' for label, _, _ in images)):
            raise EpisodeStopped('reconciliation_failed', 'Fresh calibrated joint feedback and hold certificate are required')
        return dict(constraints)

    async def _recovery_command_allowed(self, command, *, resolved):
        if self.recovery_constraints is None:
            return True
        failure = None
        if json.dumps(command, sort_keys=True) in self.failed_motion_targets:
            failure = 'The failed command was repeated unchanged; propose a revised request after inspecting feedback'
        elif command.get('action') != 'move' or any(command.get(axis) for axis in ('x', 'y', 'theta')):
            failure = 'Reconciled motion must be an arm-only move with zero base velocity'
        elif command.get('duration_ms', 0) < self.recovery_constraints['min_duration_ms']:
            failure = 'Reconciled arm motion must use at least the requested recovery duration'
        elif resolved:
            joints = command.get('joints')
            measured = (self.packet['observation'].get('ego') or {}).get('joints') or {}
            maximum = self.recovery_constraints['max_joint_delta_deg']
            if not isinstance(joints, dict) or not joints or not set(joints) <= RECOVERY_ARM_JOINTS:
                failure = 'Reconciled motion cannot command the gripper, base, or an empty arm target'
            elif any(not self._finite_number(measured.get(name)) or abs(value - measured[name]) > maximum
                     for name, value in joints.items()):
                failure = 'Reconciled joint targets must stay within 3 degrees of the fresh measured pose'
        if failure is None:
            return True
        self.rejection_feedback = {
            'source': 'host', 'command': command, 'command_sent': False,
            'error_code': 'arm_recovery_constraints', 'error': failure,
            'error_details': {'recovery_constraints': self.recovery_constraints},
            'current_observation_id': self.packet['observation']['observation_id'],
            'guidance': 'Read motion_failure_feedback and choose a changed, slow, small arm-only request. '
                        'Never repeat the failed command or treat its partial movement as completion.',
        }
        self.record.event('decision_rejected', self.rejection_feedback)
        await self._reobserve_rejection()
        return False

    async def _reject_repeated(self, command, failure, observation_id):
        # Both host IK rejection and explicit runtime rejection guarantee no motion
        # for this proposal. One extra correction opportunity never retransmits it.
        self.repeat_block_count += 1
        self.rejection_feedback = {
            **failure, 'source': 'host', 'command': command, 'rejected_command': failure['command'],
            'current_observation_id': observation_id, 'repeat_block_count': self.repeat_block_count,
            'guidance': 'This proposed command was not sent. Inspect the rejection and choose a different '
                        'feasible action, observe, or report blocked. Changing only an out-of-calibration '
                        'joint target cannot fix its invalid current position. Do not relax geometry or calibration checks.',
        }
        self.record.event('decision_rejected', {'reason': 'repeated_failed_action', **self.rejection_feedback})
        if self.repeat_block_count >= 2:
            self.record.event('rejection_feedback', self.rejection_feedback)
            raise EpisodeStopped('repeated_failed_action',
                                 'Repeated a known rejected command after a correction opportunity')
        await self._reobserve_rejection()

    async def _resolve_cartesian(self, decision, requested_command):
        request = decision.command.cartesian.model_dump()
        try:
            if self.cartesian is None:
                result = {'success': False, 'executable': False, 'reason_codes': ['cartesian_unavailable'],
                          'reasons': ['Cartesian controller is unavailable']}
            else:
                result = await self.bounded(lambda: asyncio.to_thread(
                    self.cartesian.solve, self.packet, request, decision.command.duration_ms))
            if not isinstance(result, dict):
                raise ValueError('IK controller returned an invalid result')
            resolved = None
            if result.get('success') is True and result.get('executable') is True:
                joints = result.get('joints')
                names = {'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'}
                if not isinstance(joints, dict) or set(joints) != names:
                    raise ValueError('IK result must contain exactly the five arm joints, without gripper')
                resolved = build_command('move', 0., 0., 0., decision.command.duration_ms, joints)
        except (EpisodeStopped, TimeoutError, asyncio.CancelledError):
            raise
        except Exception as exc:
            resolved = None
            result = {'success': False, 'executable': False, 'reason_codes': ['ik_error'],
                      'reasons': [f'{type(exc).__name__}: {exc}']}
        self.previous_ik_result = {**result, 'request': request, 'observation_id': decision.observation_id}
        self.record.event('ik_result', self.previous_ik_result)
        if resolved is not None:
            return resolved
        reasons = result.get('reasons') or ['IK preview cannot authorize execution']
        self.rejection_feedback = {
            'source': 'host', 'action_id': None, 'command': requested_command,
            'error': '; '.join(str(reason) for reason in reasons),
            'error_code': (result.get('reason_codes') or ['ik_unavailable'])[0],
            'error_details': {'ik_result': result}, 'command_sent': False,
            'failed_observation_id': decision.observation_id, 'current_observation_id': None,
            'repeat_block_count': self.repeat_block_count,
            'guidance': 'No motor command was sent. Inspect the IK error and fresh observation; revise the XYZ '
                        'target or choose sufficient arm-only duration within the existing speed limits, '
                        'use a feasible joint action, observe, or report blocked. Never invent mapping '
                        'verification, calibration identity, camera depth, or loosen limits to make a solution pass.',
        }
        self.previous_result = {'status': 'rejected', **self.rejection_feedback}
        self.rejected_commands[json.dumps(requested_command, sort_keys=True)] = self.rejection_feedback
        await self._reobserve_rejection()
        return None

    async def _reobserve_rejection(self):
        self.record.event('rejection_feedback', self.rejection_feedback)
        await self.observe()
        self.rejection_feedback = {
            **self.rejection_feedback,
            'current_observation_id': self.packet['observation']['observation_id'],
        }
        self.record.event('rejection_feedback', self.rejection_feedback)

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
