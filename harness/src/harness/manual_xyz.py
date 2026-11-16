"""Operator-reviewed, one-shot Cartesian moves using the existing guarded runtime.

Preparing only observes. A short-lived token binds execution to the exact displayed
joint solution. Neither a browser retry nor a restart can replay a consumed token.
An operator may confirm an unverified mapping for this one move; this never enables
Cartesian execution for the agent or changes a calibration/configuration file.
"""
import asyncio
from datetime import datetime, timezone
import math
from pathlib import Path
import time
from typing import Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .client import ServerClient
from .episode import Decision, EpisodeLimits, EpisodeRunner, EpisodeStopped
from .kinematics import ARM_JOINTS, CartesianController
from .tools import RobotTools, build_command


class ManualBusy(ValueError):
    pass


class MappingEntry(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    sign: int
    offset_deg: float = Field(ge=-180, le=180)

    @field_validator('sign')
    @classmethod
    def direction(cls, value):
        if value not in (-1, 1):
            raise ValueError('관절 방향은 +1 또는 -1이어야 합니다.')
        return value


class PrepareInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    robot_id: str = Field(min_length=1, max_length=128, pattern=r'^[A-Za-z0-9_.:-]+$')
    delta_m: list[float] = Field(min_length=3, max_length=3)
    duration_ms: int = Field(default=1000, ge=500, le=1000)
    joint_mapping: dict[str, MappingEntry] | None = None

    @model_validator(mode='after')
    def bounds(self):
        distance = math.hypot(*self.delta_m)
        if not .002 < distance <= .01:
            raise ValueError('1회 이동 거리는 2mm 초과, 10mm 이하여야 합니다. 처음에는 5mm를 입력하세요.')
        if self.joint_mapping is not None and set(self.joint_mapping) != set(ARM_JOINTS):
            raise ValueError('팔 5관절의 방향과 영점을 모두 지정해야 합니다.')
        return self


class ExecuteInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    token: str = Field(pattern=r'^[a-f0-9]{32}$')
    execute: Literal[True]
    alignment_confirmed: bool
    clearance_confirmed: Literal[True]

    @field_validator('execute', 'clearance_confirmed', mode='before')
    @classmethod
    def explicit_true(cls, value):
        if value is not True:
            raise ValueError('실제 이동과 이동 공간 확인이 필요합니다.')
        return value


class ManualXYZ:
    def __init__(self, runs_dir, server, config=None, *, tools=None, clock=time.monotonic):
        self.runs_dir, self.config = Path(runs_dir), config
        self.tools = tools or RobotTools(ServerClient(server, timeout=15))
        self.clock = clock
        self.preparing = False
        self.current_run_id = None
        self.pending = None
        self.task = None
        self.last = None

    @property
    def busy(self):
        return self.preparing or (self.task is not None and not self.task.done())

    def phase_for_run(self, run_id):
        if self.current_run_id != run_id:
            return None
        return "preparing" if self.preparing else "prepared" if self.pending else "executing" if self.busy else None

    def expire(self):
        if self.pending and self.clock() >= self.pending['deadline']:
            runner = self.pending['runner']
            self.last = {'run_id': runner.record.path.name, 'status': 'expired',
                         'detail': '계산 후 시간이 지났습니다. 현재 상태를 다시 읽고 계산하세요.'}
            runner.finish('expired', self.last['detail'])
            self.pending = None
            self.current_run_id = None

    def reserved(self):
        self.expire()
        return self.busy or self.pending is not None

    def snapshot(self):
        self.expire()
        return {'busy': self.busy, 'phase': 'preparing' if self.preparing else 'executing' if self.busy else 'idle',
                'prepared': self.pending['view'] if self.pending else None, 'last': self.last}

    def cancel(self):
        if self.busy:
            raise ManualBusy('계산 또는 전송이 진행 중입니다. 이미 보낸 명령은 취소할 수 없습니다.')
        if self.pending:
            runner = self.pending['runner']
            runner.finish('cancelled', 'Operator discarded the prepared move; no command sent')
            self.last = {'run_id': runner.record.path.name, 'status': 'cancelled', 'detail': '준비된 이동을 취소했습니다. 명령을 보내지 않았습니다.'}
            self.pending = None
            self.current_run_id = None
        return self.snapshot()

    async def prepare(self, request):
        if self.busy:
            raise ManualBusy('다른 수동 계산 또는 이동이 진행 중입니다.')
        self.cancel()
        self.preparing = True
        runner = None
        try:
            cartesian = CartesianController(self.config)
            if request.joint_mapping is not None:
                # A new GUI mapping is unverified until explicitly confirmed for a move.
                cartesian = CartesianController({
                    'version': 1, 'hardware_alignment_verified': False, 'calibration_id': None,
                    'model_hash': cartesian.geometry.model_hash,
                    'joint_mapping': {name: entry.model_dump() for name, entry in request.joint_mapping.items()},
                })
            run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-manual-' + uuid4().hex[:8]
            runner = EpisodeRunner(None, self.tools, request.robot_id, '수동 XYZ 1회 이동',
                                   self.runs_dir / run_id, EpisodeLimits(1, 1, 90),
                                   cartesian=cartesian, clock=self.clock, allow_reconciliation=False)
            self.current_run_id = run_id
            await runner.observe()
            target = {'frame': 'arm_base', 'mode': 'relative', 'xyz_m': request.delta_m, 'approach_axis': None}
            result = await runner.bounded(lambda: asyncio.to_thread(cartesian.solve, runner.packet, target, request.duration_ms))
            runner.record.event('ik_result', {**result, 'observation_id': runner.packet['observation']['observation_id']})
            state = cartesian.evaluate(runner.packet)
            reasons = list(result.get('reasons', []))
            codes = set(state['reason_codes'])
            confirm_mapping = codes == {'hardware_alignment_unverified'}
            ready = result.get('success') is True and (state['executable'] is True or confirm_mapping)
            joints = result.get('solution_runtime_joints_deg') or {}
            deltas = {name: joints[name] - state['current_runtime_joints_deg'][name] for name in joints}
            if not deltas or max(abs(value) for value in deltas.values()) > 3:
                ready = False
                reasons.append('이 시험에서는 관절마다 최대 3°까지만 변경합니다. XYZ 이동량을 줄이세요.')
            fk = state.get('fk')
            view = {
                'run_id': run_id, 'robot_id': request.robot_id, 'ready': ready, 'token': None,
                'requires_alignment_confirmation': confirm_mapping, 'state': state, 'result': result,
                'reasons': reasons, 'joint_delta_deg': deltas, 'duration_ms': request.duration_ms,
                'current_points': cartesian.geometry.joint_points(fk['joints_deg']) if fk else [],
                'solution_points': cartesian.geometry.joint_points(result['solution_urdf_joints_deg'])
                    if result.get('success') else [],
                'observation_id': runner.packet['observation']['observation_id'],
                'images': [{'label': label, 'url': f'/api/runs/{run_id}/files/observations/0001/image-{i:02d}'
                            + ('.jpg' if mime == 'image/jpeg' else '.png')} for i, (label, _, mime) in enumerate(runner.images)],
            }
            if ready:
                deadline = min(self.clock() + 30, runner.received_at + runner.packet['action_context']['max_age_s'])
                if self.clock() >= deadline:
                    raise ValueError('계산 중 관찰이 만료되었습니다. 다시 읽어주세요.')
                view.update(token=uuid4().hex, expires_at=time.time() + deadline - self.clock())
                command = build_command('move', 0., 0., 0., request.duration_ms, joints)
                self.pending = {'runner': runner, 'view': view, 'deadline': deadline, 'command': command,
                                'cartesian': cartesian}
            else:
                runner.finish('blocked', '; '.join(reasons))
                self.current_run_id = None
            self.last = {'run_id': run_id, 'status': 'prepared' if ready else 'blocked',
                         'detail': '목표를 계산했습니다. 아직 명령을 보내지 않았습니다.' if ready else '; '.join(reasons)}
            return view
        except BaseException as exc:
            if runner is not None:
                runner.finish('observation_failed' if isinstance(exc, EpisodeStopped) else 'host_error', str(exc))
            self.current_run_id = None
            self.last = {'status': 'error', 'detail': str(exc)}
            raise
        finally:
            self.preparing = False

    def execute(self, request):
        if self.busy:
            raise ManualBusy('다른 수동 계산 또는 이동이 진행 중입니다.')
        self.expire()
        pending = self.pending
        if pending is None or request.token != pending['view']['token']:
            raise ValueError('이미 사용했거나 만료된 계산입니다. 상태를 다시 읽어주세요.')
        view, runner = pending['view'], pending['runner']
        if view['requires_alignment_confirmation'] and not request.alignment_confirmed:
            raise ValueError('3D 모델의 팔 자세·축 방향이 실물과 맞는지 먼저 확인해야 합니다.')
        cartesian = pending['cartesian']
        if view['requires_alignment_confirmation']:
            cartesian = CartesianController({
                'version': 1, 'hardware_alignment_verified': True,
                'calibration_id': view['state']['calibration_id'], 'model_hash': cartesian.geometry.model_hash,
                'joint_mapping': cartesian.joint_mapping,
            })
        if not cartesian.evaluate(runner.packet)['executable'] or not runner._fresh():
            self.cancel()
            raise ValueError('현재 관찰 또는 관절 매핑이 유효하지 않습니다. 다시 계산하세요.')
        # Consume before scheduling; duplicate HTTP requests can never send twice.
        self.pending = None
        runner.cartesian = cartesian
        runner.record.event('manual_operator_confirmation', {
            'alignment_confirmed': request.alignment_confirmed,
            'clearance_confirmed': True, 'scope': 'this_one_move_only',
            'calibration_id': view['state']['calibration_id'], 'joint_mapping': cartesian.joint_mapping,
            'model_hash': cartesian.geometry.model_hash,
        })
        self.last = {'run_id': view['run_id'], 'status': 'executing', 'detail': '표시한 관절 목표를 한 번 전송하고 결과를 기다립니다.'}
        self.task = asyncio.create_task(self._execute(pending), name=f'manual-xyz-{view["run_id"]}')
        return self.last

    async def _execute(self, pending):
        runner, command, view = pending['runner'], pending['command'], pending['view']
        try:
            decision = Decision.model_validate({
                'robot_id': runner.robot_id, 'observation_id': view['observation_id'], 'tool': 'execute',
                'plan': ['사용자가 확인한 XYZ 목표로 한 번 이동'], 'reason': 'Operator confirmed the displayed endpoint and clearance.',
                'expected': 'Measured joints reach the displayed target', 'feedback': 'initial',
                'evidence': 'Fresh runtime observation and explicit operator confirmation',
                'command': {**command, 'joints': {**command['joints'], 'gripper': None}, 'cartesian': None},
            })
            runner.steps = 1
            runner.record.event('decision', decision.model_dump())
            await runner.execute(decision)
            packet = runner.previous_result or {}
            if packet.get('status') == 'completed' and (packet.get('execution') or {}).get('arm_reached') is True:
                status, detail = 'completed', '관절 도달 응답과 이동 후 관찰을 받았습니다. 실제 XYZ 방향은 화면과 실물로 확인하세요.'
            elif runner.in_flight:
                status, detail = 'uncertain_execution', '전송 결과가 확인되지 않았습니다. 자동으로 다시 보내지 않습니다.'
            elif packet.get('status') == 'completed':
                status, detail = 'uncertain_execution', '명령 응답은 받았지만 관절 도달을 확인하지 못했습니다. 실물을 확인하세요.'
            else:
                status, detail = 'rejected', packet.get('error', '명령이 실행되지 않았거나 관절 도달을 확인하지 못했습니다.')
        except BaseException as exc:
            status = 'uncertain_execution' if runner.in_flight else exc.reason if isinstance(exc, EpisodeStopped) else 'host_error'
            detail = str(exc) or type(exc).__name__
        if status == 'incomplete':
            motion = ((runner.previous_result or {}).get('execution') or {}).get('arm_motion') or {}
            targets, measured = motion.get('targets') or {}, motion.get('measured') or {}
            values = [f'{name}: 목표 {target:.2f}°, 측정 {measured[name]:.2f}°'
                      for name, target in targets.items()
                      if type(target) in (int, float) and type(measured.get(name)) in (int, float)]
            detail += (' ' + '; '.join(values)) if values else ''
        result = runner.finish(status, detail)
        self.last = {**result, 'run_id': view['run_id'], 'execution': (runner.previous_result or {}).get('execution')}
        self.current_run_id = None

    async def close(self):
        if self.task is not None and not self.task.done():
            try:
                await asyncio.wait_for(asyncio.shield(self.task), timeout=20)
            except TimeoutError:
                self.task.cancel()
                await self.task
        if self.pending:
            self.cancel()
