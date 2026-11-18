"""Local operator console for LeKiwi direct tasks and ASPIRE-style policy.py runs (SLAM-free ready)."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
from typing import Any, Literal
from urllib.parse import quote
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator
from starlette.applications import Starlette
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route

from .campaign.cli import build_default_sim_topology
from .campaign.runner import (
    CampaignPolicyRunner,
    LiveLocalizationProvider,
    PolicyValidationError,
    SimulatedCampaignWorld,
    _inspect_stage_engagement,
    validate_and_compile_policy,
)
from .campaign.semantic_memory import SemanticMemoryStore
from .campaign.topology import LocalizationProvider, RoutePlan, TopologicalMap
from .client import ServerClient
from .episode import EpisodeLimits, RunRecord, atomic_json, decode_content
from .episode_runtime import run_episode
from .gui_preview import PreviewInput, calculate_preview
from .manual_xyz import ExecuteInput, ManualBusy, ManualXYZ, PrepareInput
from .pose_reference import build_pose_reference
from .tools import RobotTools


DEFAULT_LIMITS = {'max_actions': 20, 'max_steps': 40, 'max_seconds': 300}
RUN_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
STAGE_DIR_NAME = re.compile(r'^[0-9]{4}_L[0-9]{3}_[A-Za-z0-9_]+$')
LEG_DIR_NAME = re.compile(r'^leg_[0-9]{2}_[A-Za-z0-9_]+$')
IMAGE_NAME = re.compile(r'^image-[0-9]+\.(?:jpg|jpeg|png)$')
MAX_BODY_BYTES = 16384
MAX_EVENTS_BYTES = 8 * 1024 * 1024


class LimitsInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    max_actions: int = Field(ge=1, le=100)
    max_steps: int = Field(ge=1, le=200)
    max_seconds: float = Field(ge=1, le=3600)


class StartInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    instruction: str = Field(min_length=1, max_length=8000)
    robot_id: str = Field(min_length=1, max_length=128, pattern=r'^[A-Za-z0-9_.:-]+$')
    model: str | None = Field(default=None, max_length=128)
    limits: LimitsInput
    execute: Literal[True]

    @field_validator('execute', mode='before')
    @classmethod
    def explicit_execution(cls, value):
        if value is not True:
            raise ValueError('An explicit execute: true is required')
        return value

    @field_validator('instruction', 'model')
    @classmethod
    def nonblank(cls, value):
        if value is not None and not value.strip():
            raise ValueError('Text cannot be blank')
        return value.strip() if value is not None else None


class PolicyStartInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    instruction: str = Field(min_length=1, max_length=2000)
    policy_source: str = Field(min_length=1, max_length=12000)
    robot_id: str = Field(min_length=1, max_length=128, pattern=r'^[A-Za-z0-9_.:-]+$')
    model: str | None = Field(default=None, max_length=128)
    backend: Literal['live', 'sim'] = 'live'
    execute: Literal[True]

    @field_validator('execute', mode='before')
    @classmethod
    def explicit_execution(cls, value):
        if value is not True:
            raise ValueError('An explicit execute: true is required')
        return value

    @field_validator('instruction', 'model')
    @classmethod
    def nonblank(cls, value):
        if value is not None and not value.strip():
            raise ValueError('Text cannot be blank')
        return value.strip() if value is not None else None

    @field_validator('policy_source')
    @classmethod
    def nonblank_policy(cls, value: str) -> str:
        if not value.strip():
            raise ValueError('Policy source cannot be blank')
        return value


class BusyError(RuntimeError):
    pass


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def read_text_tail(path: Path, max_bytes: int = 65536) -> str:
    try:
        with path.open('rb') as stream:
            stream.seek(0, 2)
            size = stream.tell()
            stream.seek(max(0, size - max_bytes))
            return stream.read(max_bytes).decode('utf-8', errors='replace')
    except OSError:
        return ''


def read_jsonl_records(path: Path, *, require_data_dict: bool = False) -> list[dict[str, Any]]:
    """Read completed JSONL lines, ignoring any concurrent trailing fragment."""
    try:
        with path.open('rb') as stream:
            stream.seek(0, 2)
            size = stream.tell()
            offset = max(0, size - MAX_EVENTS_BYTES)
            stream.seek(offset)
            if offset:
                stream.readline()
            raw = stream.read(MAX_EVENTS_BYTES)
    except OSError:
        return []
    records: list[dict[str, Any]] = []
    for line in raw.splitlines(keepends=True):
        if not line.endswith(b'\n'):
            continue
        try:
            item = json.loads(line)
            if isinstance(item, dict) and (not require_data_dict or isinstance(item.get('data'), dict)):
                records.append(item)
        except (ValueError, UnicodeError):
            continue
    return records[-2000:]


def read_events(path: Path) -> list[dict[str, Any]]:
    return read_jsonl_records(path, require_data_dict=True)


class _CancellableStageBridge:
    """Runs async `episode_fn` stages inside a worker thread with thread-safe cancellation."""

    def __init__(self, episode_fn, *, server: str, model: str | None, kinematics_config: Path | None):
        self._episode_fn = episode_fn
        self._server = server
        self._model = model
        self._kinematics_config = kinematics_config
        self._lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._stage_loop: asyncio.AbstractEventLoop | None = None
        self._stage_task: asyncio.Task | None = None

    def cancelled(self) -> bool:
        return self._cancel_event.is_set()

    def request_cancel(self) -> None:
        self._cancel_event.set()
        with self._lock:
            loop, task = self._stage_loop, self._stage_task
        if loop is not None and task is not None and not loop.is_closed():
            loop.call_soon_threadsafe(task.cancel)

    def run_episode_sync(self, robot_id: str, instruction: str, *, run_dir: Path, limits: EpisodeLimits) -> dict[str, Any]:
        if self._cancel_event.is_set():
            return {'status': 'interrupted', 'detail': 'Stopped before stage dispatch'}
        loop = asyncio.new_event_loop()
        options = {'kinematics_config': self._kinematics_config} if self._kinematics_config else {}
        try:
            asyncio.set_event_loop(loop)
            coro = self._episode_fn(
                self._server,
                self._model,
                robot_id,
                instruction,
                run_dir=run_dir,
                limits=limits,
                **options,
            )
            task = loop.create_task(coro)
            with self._lock:
                self._stage_loop = loop
                self._stage_task = task
                already_cancelled = self._cancel_event.is_set()
            if already_cancelled:
                task.cancel()
            try:
                res = loop.run_until_complete(task)
                return res if isinstance(res, dict) else read_json(run_dir / 'result.json')
            except asyncio.CancelledError:
                saved = read_json(run_dir / 'result.json')
                if saved.get('status') not in (None, 'running'):
                    return saved
                return {'status': 'interrupted', 'detail': 'Stage cancelled by operator'}
        finally:
            with self._lock:
                self._stage_loop = None
                self._stage_task = None
            asyncio.set_event_loop(None)
            loop.close()


class _GuiLiveNavExecutor:
    def __init__(self, bridge: _CancellableStageBridge, localization: LocalizationProvider):
        self._bridge = bridge
        self._localization = localization
        self._limits = EpisodeLimits(max_actions=10, max_steps=20, max_seconds=90.0)

    def execute_route(self, robot_id: str, route: RoutePlan, stage_dir: Path) -> dict[str, Any]:
        pose = self._localization.get_pose(robot_id)
        if pose.confidence <= 0.0:
            return {
                'ok': False,
                'status': 'localization_unavailable',
                'pose': pose.to_dict(),
                'legs_completed': 0,
                'error': 'SLAM pose is disabled (visual-only mode); use inspect_scene() and act() instead of navigate_to().',
            }
        stage_dir.mkdir(parents=True, exist_ok=True)
        completed = 0
        for idx, leg in enumerate(route.legs, start=1):
            leg_dir = stage_dir / f'leg_{idx:02d}_{leg.via}'
            instruction = (
                f'Navigate safely toward floor {leg.floor} waypoint '
                f'(x={leg.x_m:.2f}m, y={leg.y_m:.2f}m, yaw={leg.yaw_deg:.1f}deg) via {leg.via}.'
            )
            ep_res = self._bridge.run_episode_sync(robot_id, instruction, run_dir=leg_dir, limits=self._limits)
            status = str(ep_res.get('status', 'host_error'))
            if status not in ('completed', 'completion_claim'):
                return {
                    'ok': False,
                    'status': status,
                    'pose': self._localization.get_pose(robot_id).to_dict(),
                    'legs_completed': completed,
                    'error': str(ep_res.get('detail') or f'Navigation leg {idx} ended with {status}'),
                }
            completed += 1
        return {
            'ok': True,
            'status': 'completed',
            'pose': self._localization.get_pose(robot_id).to_dict(),
            'legs_completed': completed,
            'error': None,
        }


class _GuiLiveActionExecutor:
    def __init__(self, bridge: _CancellableStageBridge):
        self._bridge = bridge

    def run_subgoal(
        self,
        *,
        robot_id: str,
        kind: str,
        subgoal: str,
        highlighted_policy: str,
        limits: EpisodeLimits,
        stage_dir: Path,
    ) -> dict[str, Any]:
        instruction = (
            f'Active stage ({kind}): {subgoal}\n\n'
            f'Policy context (`>>>` marks active call):\n{highlighted_policy}'
        )
        ep_res = self._bridge.run_episode_sync(robot_id, instruction, run_dir=stage_dir, limits=limits)
        raw_status = str(ep_res.get('status', 'host_error'))
        ok = raw_status in ('completed', 'completion_claim')
        norm_status = 'completed' if ok else raw_status
        engagement = _inspect_stage_engagement(stage_dir)
        claim = ep_res.get('completion_claim') or {}
        summary = str(claim.get('evidence') or claim.get('reason') or ep_res.get('detail') or norm_status)
        return {
            'ok': ok,
            'status': norm_status,
            'summary': summary,
            'steps': int(ep_res.get('steps', 0)),
            'action_attempts': int(ep_res.get('action_attempts', 0)),
            'observations': int(ep_res.get('observations', 0)),
            'reconciliation_attempts': int(ep_res.get('reconciliation_attempts', 0)),
            'arm_moved': engagement['arm_moved'],
            'gripper_moved': engagement['gripper_moved'],
            'arm_engaged': engagement['arm_engaged'],
            'run_dir': str(stage_dir),
        }


class GuiController:
    def __init__(
        self,
        server='tcp://127.0.0.1:5556',
        robot_id='kiwi-01',
        runs_dir='runs',
        model=None,
        demo=False,
        episode_fn=None,
        kinematics_config=None,
        *,
        pose_file=None,
        topology=None,
        semantic_db=None,
    ):
        self.server, self.robot_id, self.model = server, robot_id, model
        self.kinematics_config = Path(kinematics_config).expanduser().resolve() if kinematics_config else None
        self.pose_file = Path(pose_file).expanduser().resolve() if pose_file else None
        self.topology_path = Path(topology).expanduser().resolve() if topology else None
        self.semantic_db_path = Path(semantic_db).expanduser().resolve() if semantic_db else None
        if demo and self.kinematics_config is not None:
            raise ValueError('Demo mode does not accept a hardware kinematics configuration')
        self.mode = 'demo' if demo else 'live'
        self.runs_dir = Path(runs_dir).resolve()
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        if episode_fn is None and demo:
            from .gui_demo import run_demo
            episode_fn = run_demo
        self.episode_fn = episode_fn or run_episode
        self.active_run_id = None
        self.task = None
        self.stopping = False
        self.requests: dict[str, dict[str, Any]] = {}
        self.preview_lock = asyncio.Lock()
        self.reference_busy = False
        self.manual = ManualXYZ(self.runs_dir, self.server, self.kinematics_config)
        self._policy_bridge: _CancellableStageBridge | None = None

    def preview(self, request):
        observed = None
        if request.source == 'recorded':
            path = self._run_path(request.run_id)
            events = read_events(path / 'events.jsonl')
            event = next((event for event in reversed(events)
                          if event.get('event') == 'observation' and
                          (request.observation_id is None or
                           event['data'].get('observation_id') == request.observation_id)), None)
            observed = self._observation(request.run_id, event) if event else None
            if observed is None:
                raise ValueError('No saved observation available for this run.')
        value = calculate_preview(request, observed=observed, config=self.kinematics_config)
        if observed is not None:
            reference = read_json(path / 'pose_reference.json')
            if (reference.get('provenance', {}).get('observation_id') == observed['observation_id']
                    and reference.get('mapping_id') == value['state'].get('mapping_id')
                    and reference.get('model_hash') == value['state'].get('model_hash')):
                value['pose_reference'] = reference
        return value

    def save_pose_reference(self, packet, images):
        """Persist a read-only camera & joint snapshot; no runnable action is created."""
        reference = build_pose_reference(packet, self.kinematics_config)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-pose-' + uuid4().hex[:8]
        record = RunRecord(self.runs_dir / run_id, self.robot_id,
                           'Read-only camera & joint snapshot', EpisodeLimits())
        metadata = read_json(record.path / 'run.json')
        atomic_json(record.path / 'run.json', {**metadata, 'mode': 'reference', 'execute': False})
        record.observation(packet, images)
        atomic_json(record.path / 'pose_reference.json', reference)
        record.event('pose_reference', reference)
        atomic_json(record.path / 'result.json', {
            'status': 'reference_saved', 'detail': 'Observed cameras & joint posture (no motor command sent)',
            'automatic_resume': False, 'physical_success_verified': False, 'action_attempts': 0,
            'steps': 0, 'observations': 1, 'executed': False,
        })
        return {'run_id': run_id, 'pose_reference': reference, 'executed': False}

    async def capture_pose(self):
        if self.mode != 'live':
            raise ValueError('Demo GUI does not connect to live robot hardware.')
        if (self.reference_busy or self.manual.reserved()
                or self.task is not None and not self.task.done()):
            raise BusyError('Wait for the active observation, task, or manual move to finish.')
        self.reference_busy = True
        try:
            packet, images = decode_content(await RobotTools(ServerClient(self.server)).observe(self.robot_id))
            if packet.get('status') != 'ok':
                raise ValueError(packet.get('error') or 'Failed to read robot observation.')
            return await asyncio.to_thread(self.save_pose_reference, packet, images)
        finally:
            self.reference_busy = False

    def _run_path(self, run_id, *, exists=True):
        if not RUN_ID.fullmatch(run_id) or run_id in ('.', '..'):
            raise FileNotFoundError('Invalid run ID')
        requested = self.runs_dir / run_id
        path = requested.resolve()
        if requested.is_symlink() or path.parent != self.runs_dir or (exists and not path.is_dir()):
            raise FileNotFoundError('Run not found')
        return path

    def _is_active(self, run_id):
        return self.active_run_id == run_id and self.task is not None and not self.task.done()

    def _assert_not_busy(self) -> None:
        if self.task is not None and not self.task.done():
            raise BusyError('An episode or policy run is already active; wait for cleanup to finish')
        if self.reference_busy or self.manual.reserved():
            raise BusyError('Another observation or manual action is currently reserved.')

    async def start(self, payload):
        request = StartInput.model_validate(payload)
        if request.model is None and self.model is not None:
            request = request.model_copy(update={'model': self.model})
        self._assert_not_busy()
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:8]
        pending = request.model_dump()
        pending['mode'] = self.mode
        pending['started_at'] = datetime.now(timezone.utc).isoformat()
        self.requests[run_id] = pending
        self.active_run_id, self.stopping = run_id, False
        self.task = asyncio.create_task(self._run(run_id, request), name=f'lekiwi-gui-{run_id}')
        self.task.add_done_callback(lambda task: self._on_done(run_id, task))
        return self.run_view(run_id)

    async def start_policy(self, payload):
        request = PolicyStartInput.model_validate(payload)
        try:
            validate_and_compile_policy(request.policy_source)
        except PolicyValidationError as exc:
            raise ValueError(str(exc)) from exc
        if self.mode == 'demo' and request.backend == 'live':
            raise ValueError('Demo GUI cannot run a live policy; select Sim Sandbox or launch without --demo.')
        if request.model is None and self.model is not None:
            request = request.model_copy(update={'model': self.model})
        self._assert_not_busy()

        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-policy-' + uuid4().hex[:8]
        started_at = datetime.now(timezone.utc).isoformat()
        pending = {
            **request.model_dump(),
            'mode': 'policy',
            'slam_enabled': bool(self.pose_file is not None and request.backend == 'live'),
            'started_at': started_at,
        }
        path = self._run_path(run_id, exists=False)
        path.mkdir(parents=True, exist_ok=True)
        atomic_json(path / 'run.json', {
            'version': 1,
            'mode': 'policy',
            'backend': request.backend,
            'slam_enabled': pending['slam_enabled'],
            'robot_id': request.robot_id,
            'instruction': request.instruction,
            'model': request.model,
            'started_at': started_at,
            'automatic_resume': False,
        })
        atomic_json(path / 'result.json', {'status': 'running', 'automatic_resume': False})

        self.requests[run_id] = pending
        self.active_run_id, self.stopping = run_id, False
        self.task = asyncio.create_task(self._run_policy(run_id, request), name=f'lekiwi-policy-{run_id}')
        self.task.add_done_callback(lambda task: self._on_done(run_id, task))
        return self.run_view(run_id)

    async def _run(self, run_id, request):
        try:
            options = {'kinematics_config': self.kinematics_config} if self.kinematics_config else {}
            await self.episode_fn(
                self.server, request.model, request.robot_id, request.instruction,
                run_dir=self._run_path(run_id, exists=False),
                limits=EpisodeLimits(**request.limits.model_dump()),
                **options,
            )
        except asyncio.CancelledError:
            self._fallback_result(run_id, 'interrupted', 'Loop stopped; an already sent command may still finish')
        except Exception as exc:
            self._fallback_result(run_id, 'gui_error', f'{type(exc).__name__}: {exc}')

    async def _run_policy(self, run_id: str, request: PolicyStartInput) -> None:
        path = self._run_path(run_id, exists=False)
        bridge = _CancellableStageBridge(
            self.episode_fn,
            server=self.server,
            model=request.model or 'gpt-5.4',
            kinematics_config=self.kinematics_config,
        )
        self._policy_bridge = bridge

        def run_sync():
            store = SemanticMemoryStore(persist_dir=self.semantic_db_path if request.backend == 'live' else None)
            if self.topology_path and self.topology_path.is_file():
                topo = TopologicalMap.load(self.topology_path)
            elif request.backend == 'sim':
                topo = build_default_sim_topology()
            else:
                topo = TopologicalMap()

            if request.backend == 'sim':
                world = SimulatedCampaignWorld.default_two_floor_world()
                store.upsert(
                    'red ceramic mug on pantry counter',
                    floor='2F',
                    x_m=4.0,
                    y_m=1.5,
                    room='pantry',
                    tags=('mug', 'red'),
                    robot_id=request.robot_id,
                )
                runner = CampaignPolicyRunner(
                    task_language=request.instruction,
                    robot_id=request.robot_id,
                    mode='sim',
                    semantic_memory=store,
                    topology=topo,
                    localization=world,
                    navigation_executor=world,
                    action_executor=world,
                    cancel_check=bridge.cancelled,
                )
            else:
                localization = LiveLocalizationProvider(pose_file=self.pose_file)
                nav_exec = _GuiLiveNavExecutor(bridge, localization)
                act_exec = _GuiLiveActionExecutor(bridge)
                runner = CampaignPolicyRunner(
                    task_language=request.instruction,
                    robot_id=request.robot_id,
                    mode='live',
                    semantic_memory=store,
                    topology=topo,
                    localization=localization,
                    navigation_executor=nav_exec,
                    action_executor=act_exec,
                    cancel_check=bridge.cancelled,
                )
            outcome = runner.run_policy_source(request.policy_source, path)
            term = outcome.termination_reason
            status = 'completed' if outcome.task_success else term
            atomic_json(path / 'result.json', {
                'status': status,
                'detail': outcome.summary,
                'robot_id': request.robot_id,
                'task': request.instruction,
                'task_success': outcome.task_success,
                'navigation_reached': outcome.navigation_reached,
                'stages_executed': outcome.stages_executed,
                'calls_executed': outcome.calls_executed,
                'elapsed_s': outcome.elapsed_s,
                'automatic_resume': False,
                'physical_success_verified': False,
                'run_dir': str(path),
            })

        try:
            await asyncio.shield(asyncio.to_thread(run_sync))
        except asyncio.CancelledError:
            bridge.request_cancel()
            self._fallback_result(run_id, 'interrupted', 'Policy stopped by operator')
        except Exception as exc:
            self._fallback_result(run_id, 'gui_error', f'{type(exc).__name__}: {exc}')
        finally:
            self._policy_bridge = None

    def _fallback_result(self, run_id, status, detail):
        """Preserve the runner's terminal result, especially uncertain execution."""
        path = self._run_path(run_id, exists=False)
        path.mkdir(exist_ok=True)
        request = self.requests[run_id]
        if not (path / 'run.json').exists():
            atomic_json(path / 'run.json', {**request, 'automatic_resume': False})
        result = read_json(path / 'result.json')
        if result.get('status') not in (None, 'running'):
            return
        pending = None
        for event in read_events(path / 'events.jsonl'):
            if event.get('event') == 'action_prepared':
                pending = event['data'].get('action_id')
            elif event.get('event') == 'motion_incomplete' and event['data'].get('failed_action_id') == pending:
                pending = None
            elif event.get('event') == 'motion_warning' and event['data'].get('action_id') == pending:
                pending = None
            elif event.get('event') == 'action_result' and event['data'].get('action_id') == pending:
                packet = event['data'].get('packet') or {}
                execution = packet.get('execution') or {}
                confirmed = (
                    packet.get('status') in ('completed', 'completed_without_observation')
                    and execution.get('outcome') == 'completed'
                    and type(execution.get('command_sent')) is bool
                ) or (packet.get('status') == 'rejected' and execution.get('outcome') == 'rejected'
                      and execution.get('command_sent') is False)
                if execution.get('action_id') == pending and confirmed:
                    pending = None
        if pending:
            status = 'uncertain_execution'
        atomic_json(path / 'result.json', {
            'status': status, 'detail': detail, 'robot_id': request['robot_id'],
            'task': request['instruction'], 'pending_action_id': pending,
            'automatic_resume': False, 'physical_success_verified': False,
            'run_dir': str(path),
        })

    def _on_done(self, run_id, task):
        try:
            if task.cancelled():
                self._fallback_result(run_id, 'interrupted', 'Stopped before episode startup')
            elif task.exception() is not None:
                self._fallback_result(run_id, 'gui_error', str(task.exception()))
            else:
                self._fallback_result(run_id, 'interrupted_unknown', 'Episode ended without a terminal record')
            path = self._run_path(run_id)
            atomic_json(path / 'gui_request.json', self.requests[run_id])
        finally:
            if self.active_run_id == run_id:
                self.active_run_id, self.stopping = None, False

    async def stop(self, run_id):
        if not self._is_active(run_id):
            raise BusyError('This GUI has no active episode with that ID')
        if not self.stopping:
            self.stopping = True
            if self._policy_bridge is not None:
                self._policy_bridge.request_cancel()
            else:
                self.task.cancel()
        return {'id': run_id, 'status': 'stopping'}

    async def close(self):
        await self.manual.close()
        if self.task is not None and not self.task.done():
            await self.stop(self.active_run_id)
            try:
                await asyncio.wait_for(asyncio.shield(self.task), timeout=10)
            except (TimeoutError, asyncio.CancelledError):
                pass

    @staticmethod
    def _allowed_observation_parts(parts: tuple[str, ...]) -> bool:
        if len(parts) == 3:
            obs_lit, seq, fname = parts
        elif len(parts) == 5 and parts[0] == 'stages' and STAGE_DIR_NAME.fullmatch(parts[1]):
            obs_lit, seq, fname = parts[2], parts[3], parts[4]
        elif (
            len(parts) == 6
            and parts[0] == 'stages'
            and STAGE_DIR_NAME.fullmatch(parts[1])
            and LEG_DIR_NAME.fullmatch(parts[2])
        ):
            obs_lit, seq, fname = parts[3], parts[4], parts[5]
        else:
            return False
        return (
            obs_lit == 'observations'
            and seq.isdigit()
            and bool(IMAGE_NAME.fullmatch(fname) or fname in ('observation.json', 'images.json'))
        )

    def file_path(self, run_id, relative):
        run = self._run_path(run_id)
        parts = Path(relative).parts
        if not self._allowed_observation_parts(parts):
            raise FileNotFoundError('Only observation images and metadata are accessible')
        candidate = (run / relative).resolve()
        if candidate != run / relative or not candidate.is_relative_to(run) or not candidate.is_file():
            raise FileNotFoundError('Observation file not found')
        return candidate

    def _observation(self, run_id, event, *, prefix: str = ''):
        relative = event['data'].get('path')
        if not isinstance(relative, str):
            return None
        scoped_rel = f'{prefix}/{relative}' if prefix else relative
        try:
            path = self.file_path(run_id, scoped_rel + '/observation.json')
        except FileNotFoundError:
            return None
        packet = read_json(path)
        if not packet:
            return None
        manifest = event['data'].get('images')
        if not isinstance(manifest, list):
            metadata = read_json(path.with_name('images.json'))
            manifest = metadata.get('images')
        run_root = self._run_path(run_id)
        if not isinstance(manifest, list):
            manifest = [
                {
                    'label': image.stem,
                    'path': str(image.relative_to(run_root)),
                    'mime_type': 'image/png' if image.suffix == '.png' else 'image/jpeg',
                }
                for image in sorted(path.parent.glob('image-*'))
                if IMAGE_NAME.fullmatch(image.name)
            ]
        images = []
        for item in manifest:
            if not isinstance(item, dict) or not isinstance(item.get('path'), str):
                continue
            raw_item_path = item['path']
            rel_img_path = f'{prefix}/{raw_item_path}' if (prefix and not raw_item_path.startswith('stages/')) else raw_item_path
            try:
                self.file_path(run_id, rel_img_path)
            except FileNotFoundError:
                continue
            images.append({
                'label': str(item.get('label', 'camera')),
                'url': f'/api/runs/{quote(run_id, safe="")}/files/{quote(rel_img_path, safe="/")}',
                'mime_type': item.get('mime_type', 'image/jpeg'),
            })
        return {
            'sequence': int(path.parent.name),
            'observation_id': event['data'].get('observation_id'),
            'at': event.get('at'),
            'packet': packet,
            'images': images,
        }

    def _collect_policy_details(self, run_id: str, path: Path, request: dict[str, Any], *, stage_name: str | None = None):
        if stage_name is not None and not STAGE_DIR_NAME.fullmatch(stage_name):
            raise FileNotFoundError('Invalid stage name')
        source_path = path / 'source' / 'policy.py'
        policy_source = source_path.read_text(encoding='utf-8') if source_path.is_file() else str(request.get('policy_source', ''))
        stdout_text = read_text_tail(path / 'logs' / 'policy.stdout')
        report_payload = read_json(path / 'source' / 'policy-result.json').get('result')
        outcome = read_json(path / 'outcome.json') or None
        trace = read_jsonl_records(path / 'trace' / 'events.jsonl', require_data_dict=False)
        active_highlighted = trace[-1].get('highlighted_policy') if trace else ''

        stages_summary: list[dict[str, Any]] = []
        chosen_stage: tuple[str, str, list[dict[str, Any]]] | None = None
        stages_dir = path / 'stages'
        if stages_dir.is_dir():
            for sdir in sorted(stages_dir.iterdir(), key=lambda p: p.name):
                if not sdir.is_dir() or not STAGE_DIR_NAME.fullmatch(sdir.name):
                    continue
                sres = read_json(sdir / 'result.json')
                sevents = read_events(sdir / 'events.jsonl')
                stages_summary.append({
                    'name': sdir.name,
                    'status': sres.get('status', 'running' if not sres else 'completed'),
                    'steps': sres.get('steps', sum(1 for ev in sevents if ev.get('event') == 'decision')),
                    'action_attempts': sres.get('action_attempts', sum(1 for ev in sevents if ev.get('event') == 'action_prepared')),
                    'summary': sres.get('summary') or sres.get('detail') or '',
                })
                if stage_name is not None:
                    if sdir.name == stage_name:
                        chosen_stage = (sdir.name, f'stages/{sdir.name}', sevents)
                elif sevents or chosen_stage is None:
                    chosen_stage = (sdir.name, f'stages/{sdir.name}', sevents)

        current_obs = None
        before_obs = None
        kinematics = None
        stage_events: list[dict[str, Any]] = []
        selected_stage_name: str | None = None
        if chosen_stage is not None:
            selected_stage_name, prefix, stage_events = chosen_stage
            observations = [ev for ev in stage_events if ev.get('event') == 'observation']
            current_obs = self._observation(run_id, observations[-1], prefix=prefix) if observations else None
            before_id = next(
                (ev['data'].get('before_observation_id') for ev in reversed(stage_events) if ev.get('event') == 'motion_feedback'),
                None,
            )
            before_ev = next((ev for ev in reversed(observations) if ev['data'].get('observation_id') == before_id), None)
            if before_ev is None and len(observations) > 1:
                before_ev = observations[-2]
            before_obs = self._observation(run_id, before_ev, prefix=prefix) if before_ev is not None else None
            kinematics = next((ev['data'] for ev in reversed(stage_events) if ev.get('event') == 'kinematics_state'), None)

        policy_info = {
            'backend': request.get('backend', read_json(path / 'run.json').get('backend', 'live')),
            'slam_enabled': bool(request.get('slam_enabled', read_json(path / 'run.json').get('slam_enabled', False))),
            'source': policy_source,
            'stdout': stdout_text,
            'report': report_payload,
            'outcome': outcome,
            'trace': trace[-200:],
            'active_highlighted': active_highlighted,
            'stages': stages_summary,
            'selected_stage': selected_stage_name,
        }
        return policy_info, stage_events, current_obs, before_obs, kinematics

    def run_view(self, run_id, *, stage_name: str | None = None):
        path = self._run_path(run_id, exists=run_id not in self.requests)
        record = read_json(path / 'run.json')
        request = self.requests.get(run_id) or read_json(path / 'gui_request.json') or record
        result = read_json(path / 'result.json') or None
        raw_mode = request.get('mode', record.get('mode', 'execute'))
        is_policy = raw_mode == 'policy' or (path / 'source' / 'policy.py').is_file()

        manual_phase = self.manual.phase_for_run(run_id)
        active = self._is_active(run_id) or manual_phase is not None
        status = manual_phase or (result or {}).get('status', 'starting' if active else 'interrupted_unknown')
        if active and self.stopping:
            status = 'stopping'
        elif not active and status in ('running', 'starting'):
            status = 'interrupted_unknown'

        phases = {
            'startup_started': 'starting', 'startup_finished': 'ready',
            'observe_started': 'observing', 'observe_finished': 'observed',
            'planning_started': 'planning', 'planning_finished': 'planned',
            'decision': 'decided', 'action_prepared': 'executing', 'action_started': 'executing',
            'action_finished': 'reviewing', 'action_result': 'reviewing',
            'motion_incomplete': 'reviewing', 'motion_warning': 'replanning',
            'reconcile_started': 'reconciling', 'reconcile_finished': 'replanning',
            'motion_replanning': 'replanning', 'motion_failure_feedback': 'replanning',
            'motion_recovery_completed': 'reviewing', 'motion_feedback': 'reviewing',
            'completion_candidate': 'confirming',
        }

        if is_policy:
            policy_info, events, current, before, kinematics = self._collect_policy_details(
                run_id, path, request, stage_name=stage_name
            )
            phase = 'executing' if active else status
            for ev in events:
                phase = phases.get(ev.get('event'), phase)
            if status not in ('running', 'starting'):
                phase = status
            return {
                'id': run_id,
                'active': active,
                'mode': 'policy',
                'request': {
                    'robot_id': request.get('robot_id'),
                    'instruction': request.get('instruction'),
                    'model': request.get('model'),
                    'limits': request.get('limits', DEFAULT_LIMITS),
                },
                'result': result,
                'status': status,
                'phase': phase,
                'events': events,
                'current_observation': current,
                'before_observation': before,
                'kinematics': kinematics,
                'policy': policy_info,
            }

        events = read_events(path / 'events.jsonl')
        phase = 'starting'
        for event in events:
            phase = phases.get(event.get('event'), phase)
        if status not in ('running', 'starting'):
            phase = status
        observations = [event for event in events if event.get('event') == 'observation']
        current = self._observation(run_id, observations[-1]) if observations else None
        before_id = next((event['data'].get('before_observation_id') for event in reversed(events)
                          if event.get('event') == 'motion_feedback'), None)
        before_event = next((event for event in reversed(observations)
                             if event['data'].get('observation_id') == before_id), None)
        if before_event is None and len(observations) > 1:
            before_event = observations[-2]
        before = self._observation(run_id, before_event) if before_event is not None else None
        model = request.get('model')
        if model is None:
            model = next((event['data'].get('model') for event in events
                          if event.get('event') == 'planner_started'), None)
        kinematics = next((event['data'] for event in reversed(events)
                           if event.get('event') == 'kinematics_state'), None)
        return {
            'id': run_id,
            'active': active,
            'mode': raw_mode if raw_mode in ('demo', 'reference') else 'live',
            'request': {
                'robot_id': request.get('robot_id'),
                'instruction': request.get('instruction'),
                'model': model,
                'limits': request.get('limits', DEFAULT_LIMITS),
            },
            'result': result,
            'status': status,
            'phase': phase,
            'events': events,
            'current_observation': current,
            'before_observation': before,
            'kinematics': kinematics,
            'policy': None,
        }

    def snapshot(self):
        active_id = self.active_run_id
        candidates = []
        for path in self.runs_dir.iterdir():
            if path.is_dir() and not path.is_symlink() and RUN_ID.fullmatch(path.name):
                candidates.append(path.name)
        if active_id is not None and active_id not in candidates:
            candidates.append(active_id)
        runs = []
        for run_id in sorted(candidates, reverse=True)[:200]:
            try:
                path = self._run_path(run_id, exists=run_id not in self.requests)
                record = self.requests.get(run_id) or read_json(path / 'run.json')
                result = read_json(path / 'result.json')
                manual_phase = self.manual.phase_for_run(run_id)
                active = self._is_active(run_id) or manual_phase is not None
                status = manual_phase or result.get('status', 'starting' if active else 'interrupted_unknown')
                if active and self.stopping:
                    status = 'stopping'
                elif not active and status in ('running', 'starting'):
                    status = 'interrupted_unknown'
                rec_mode = record.get('mode')
                if (path / 'source' / 'policy.py').is_file() or rec_mode == 'policy':
                    display_mode = 'policy'
                elif rec_mode in ('demo', 'reference'):
                    display_mode = rec_mode
                else:
                    display_mode = 'live'
                runs.append({
                    'id': run_id,
                    'instruction': record.get('instruction', ''),
                    'started_at': record.get('started_at'),
                    'status': status,
                    'active': active,
                    'mode': display_mode,
                    'backend': record.get('backend'),
                })
            except (FileNotFoundError, OSError):
                continue
        return {
            'mode': self.mode,
            'config': {
                'server': self.server,
                'robot_id': self.robot_id,
                'model': self.model,
                'limits': dict(DEFAULT_LIMITS),
                'kinematics_configured': self.kinematics_config is not None,
                'slam_configured': self.pose_file is not None,
                'topology_configured': self.topology_path is not None,
                'navigation_mode': 'slam_enabled' if self.pose_file is not None else 'visual_only',
            },
            'capabilities': {
                'xyz_preview': 1,
                'manual_xyz': 1 if self.mode == 'live' else 0,
                'pose_capture': 1 if self.mode == 'live' else 0,
                'policy_run': 1,
            },
            'pose_capture_busy': self.reference_busy,
            'active_run_id': active_id,
            'runs': runs,
            'active': self.run_view(active_id) if active_id is not None else None,
        }


class LocalOnly:
    def __init__(self, app, port):
        self.app = app
        self.hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}
        if port == 80:
            self.hosts.update(('127.0.0.1', 'localhost'))

    async def __call__(self, scope, receive, send):
        if scope['type'] != 'http':
            return await self.app(scope, receive, send)
        headers = [(key.lower(), value.decode('latin1')) for key, value in scope['headers']]
        hosts = [value for key, value in headers if key == b'host']
        origins = [value for key, value in headers if key == b'origin']
        if (len(hosts) != 1 or hosts[0] not in self.hosts or len(origins) > 1 or
                (origins and origins[0] != f'http://{hosts[0]}')):
            return await JSONResponse({'error': 'Local same-origin requests only'}, 403)(scope, receive, send)

        async def headers_added(message):
            if message['type'] == 'http.response.start':
                message.setdefault('headers', []).extend([
                    (b'cache-control', b'no-store'), (b'x-content-type-options', b'nosniff'),
                    (b'x-frame-options', b'DENY'), (b'referrer-policy', b'no-referrer'),
                ])
            await send(message)
        await self.app(scope, receive, headers_added)


async def json_body(request):
    if request.headers.get('content-type', '').split(';', 1)[0].strip().lower() != 'application/json':
        raise ValueError('Expected application/json')
    raw = bytearray()
    async for chunk in request.stream():
        raw.extend(chunk)
        if len(raw) > MAX_BODY_BYTES:
            raise ValueError('Request body too large')
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError('Expected a JSON object')
    return value


def create_app(controller=None, *, port=8765, static_dir=None):
    controller = controller or GuiController()
    static = (Path(static_dir) if static_dir is not None else Path(__file__).parent / 'static').resolve()

    @asynccontextmanager
    async def lifespan(app):
        yield
        await controller.close()

    async def state(request):
        value = await asyncio.to_thread(controller.snapshot)
        value['manual_xyz'] = controller.manual.snapshot()
        return JSONResponse(value)

    async def manual(request):
        try:
            if controller.mode != 'live':
                raise ValueError('Demo GUI does not connect to live robot hardware.')
            operation = request.path_params['operation']
            payload = await json_body(request)
            if operation == 'cancel':
                if payload:
                    raise ValueError('Cancel takes an empty JSON object.')
                return JSONResponse(controller.manual.cancel())
            if controller.task is not None and not controller.task.done():
                raise ManualBusy('An agent task is active; wait for it to finish.')
            if controller.reference_busy:
                raise ManualBusy('A camera/pose observation is currently in progress.')
            if operation == 'prepare':
                return JSONResponse(await controller.manual.prepare(PrepareInput.model_validate(payload)))
            if operation == 'execute':
                return JSONResponse(controller.manual.execute(ExecuteInput.model_validate(payload)), 202)
            return JSONResponse({'error': 'Unknown manual operation'}, 404)
        except ManualBusy as exc:
            return JSONResponse({'error': str(exc)}, 409)
        except (ValueError, TypeError, OSError) as exc:
            return JSONResponse({'error': str(exc)}, 400)
        except Exception as exc:
            return JSONResponse({'error': str(exc) or type(exc).__name__}, 503)

    async def detail(request):
        try:
            stage_param = request.query_params.get('stage') or None
            value = await asyncio.to_thread(controller.run_view, request.path_params['run_id'], stage_name=stage_param)
            return JSONResponse(value)
        except (FileNotFoundError, OSError):
            return JSONResponse({'error': 'Run not found'}, 404)

    async def preview(request):
        try:
            payload = PreviewInput.model_validate(await json_body(request))
            if controller.preview_lock.locked():
                return JSONResponse({'error': 'Another preview calculation is in progress.'}, 409)
            async with controller.preview_lock:
                return JSONResponse(await asyncio.to_thread(controller.preview, payload))
        except FileNotFoundError:
            return JSONResponse({'error': 'Run record or configuration file not found.'}, 404)
        except (ValueError, TypeError, OSError) as exc:
            return JSONResponse({'error': str(exc)}, 400)

    async def capture(request):
        try:
            if await json_body(request):
                raise ValueError('Observe snapshot takes an empty JSON object.')
            return JSONResponse(await controller.capture_pose())
        except BusyError as exc:
            return JSONResponse({'error': str(exc)}, 409)
        except (ValueError, TypeError, OSError) as exc:
            return JSONResponse({'error': str(exc)}, 400)
        except Exception as exc:
            return JSONResponse({'error': str(exc) or type(exc).__name__}, 503)

    async def start(request):
        try:
            return JSONResponse(await controller.start(await json_body(request)), 202)
        except BusyError as exc:
            return JSONResponse({'error': str(exc)}, 409)
        except (ValueError, TypeError) as exc:
            return JSONResponse({'error': str(exc)}, 400)

    async def start_policy(request):
        try:
            return JSONResponse(await controller.start_policy(await json_body(request)), 202)
        except BusyError as exc:
            return JSONResponse({'error': str(exc)}, 409)
        except (ValueError, TypeError) as exc:
            return JSONResponse({'error': str(exc)}, 400)

    async def stop(request):
        try:
            if await json_body(request):
                raise ValueError('Stop takes an empty JSON object')
            return JSONResponse(await controller.stop(request.path_params['run_id']), 202)
        except BusyError as exc:
            return JSONResponse({'error': str(exc)}, 409)
        except (ValueError, TypeError) as exc:
            return JSONResponse({'error': str(exc)}, 400)

    async def files(request):
        try:
            path = controller.file_path(request.path_params['run_id'], request.path_params['path'])
            return FileResponse(path)
        except (FileNotFoundError, OSError):
            return JSONResponse({'error': 'Observation file not found'}, 404)

    async def assets(request):
        name = {'/': 'console.html', '/console.css': 'console.css', '/console.js': 'console.js',
                '/static/console.css': 'console.css', '/static/console.js': 'console.js'}[request.url.path]
        path = static / name
        if not path.is_file():
            return JSONResponse({'error': 'GUI asset not found'}, 404)
        return FileResponse(path)

    async def viewer_assets(request):
        relative = request.path_params['path']
        root = static if request.url.path.startswith('/static/') else Path(__file__).parent / 'data'
        path = root / relative
        allowed = ((relative == 'urdf-viewer.js' or relative.startswith('vendor/') and relative.endswith('.js'))
                   if root == static else (relative == 'so101_new_calib.urdf' or
                       re.fullmatch(r'assets/[A-Za-z0-9_]+\.stl', relative)))
        if not allowed or path.resolve() != path or not path.is_file():
            return JSONResponse({'error': 'Viewer asset not found'}, 404)
        media_type = 'application/xml' if path.suffix == '.urdf' else 'text/javascript' if path.suffix == '.js' else 'application/octet-stream'
        return FileResponse(path, media_type=media_type)

    app = Starlette(lifespan=lifespan, routes=[
        Route('/api/state', state),
        Route('/api/runs', start, methods=['POST']),
        Route('/api/policy/run', start_policy, methods=['POST']),
        Route('/api/kinematics/preview', preview, methods=['POST']),
        Route('/api/kinematics/capture', capture, methods=['POST']),
        Route('/api/manual-xyz/{operation}', manual, methods=['POST']),
        Route('/api/runs/{run_id}/stop', stop, methods=['POST']),
        Route('/api/runs/{run_id}/files/{path:path}', files),
        Route('/api/runs/{run_id}', detail),
        Route('/', assets), Route('/console.css', assets), Route('/console.js', assets),
        Route('/static/console.css', assets), Route('/static/console.js', assets),
        Route('/static/{path:path}', viewer_assets), Route('/models/{path:path}', viewer_assets),
    ])
    app.state.controller = controller
    app.add_middleware(LocalOnly, port=port)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--server', default='tcp://127.0.0.1:5556')
    parser.add_argument('--robot-id', default='kiwi-01')
    parser.add_argument('--model', default=None)
    parser.add_argument('--runs-dir', type=Path, default=Path('runs'))
    parser.add_argument('--demo', action='store_true', help='Synthetic episode, no model or robot calls')
    parser.add_argument('--kinematics-config', type=Path, help='Verified local joint mapping for guarded XYZ execution')
    parser.add_argument('--pose-file', type=Path, default=None, help='Optional live SLAM pose JSON (omit for SLAM-free visual mode)')
    parser.add_argument('--topology', type=Path, default=None, help='Optional multi-floor TopologicalMap JSON')
    parser.add_argument('--semantic-db', type=Path, default=None, help='Optional ChromaDB spatial memory directory')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be 1..65535')
    if args.demo and args.kinematics_config:
        parser.error('--demo cannot use --kinematics-config')
    controller = GuiController(
        args.server,
        args.robot_id,
        args.runs_dir,
        args.model,
        args.demo,
        kinematics_config=args.kinematics_config,
        pose_file=args.pose_file,
        topology=args.topology,
        semantic_db=args.semantic_db,
    )
    import uvicorn
    nav_label = 'SLAM pose file' if controller.pose_file else 'SLAM-free visual mode'
    print(f'LeKiwi console: http://127.0.0.1:{args.port} ({controller.mode}, {nav_label})', flush=True)
    uvicorn.run(create_app(controller, port=args.port), host='127.0.0.1', port=args.port)


if __name__ == '__main__':
    main()
