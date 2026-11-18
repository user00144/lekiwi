"""Local GUI for the existing durable episode loop; never polls robot sensors."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Literal
from urllib.parse import quote
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route

from .episode import EpisodeLimits, RunRecord, atomic_json, decode_content
from .client import ServerClient
from .episode_runtime import run_episode
from .gui_preview import PreviewInput, calculate_preview
from .manual_xyz import ExecuteInput, ManualBusy, ManualXYZ, PrepareInput
from .pose_reference import build_pose_reference
from .tools import RobotTools


DEFAULT_LIMITS = {'max_actions': 20, 'max_steps': 40, 'max_seconds': 300}
PLAN_ACT_LIMITS = {'max_actions': 120, 'max_steps': 60, 'max_seconds': 1200}
RUN_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
IMAGE_NAME = re.compile(r'^image-[0-9]+\.(?:jpg|jpeg|png)$')
MAX_BODY_BYTES = 16384
MAX_EVENTS_BYTES = 8 * 1024 * 1024


class LimitsInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    max_actions: int = Field(ge=1, le=300)
    max_steps: int = Field(ge=1, le=200)
    max_seconds: float = Field(ge=1, le=3600)


class StartInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    instruction: str = Field(min_length=1, max_length=8000)
    robot_id: str = Field(min_length=1, max_length=128, pattern=r'^[A-Za-z0-9_.:-]+$')
    model: str | None = Field(max_length=128)
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


class BusyError(RuntimeError):
    pass


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def read_events(path):
    """A concurrent final JSONL fragment is ignored until the writer completes it."""
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
    events = []
    for line in raw.splitlines(keepends=True):
        if not line.endswith(b'\n'):
            continue
        try:
            event = json.loads(line)
            if isinstance(event, dict) and isinstance(event.get('data'), dict):
                events.append(event)
        except (ValueError, UnicodeError):
            continue
    return events[-2000:]


class GuiController:
    def __init__(self, server='tcp://127.0.0.1:5556', robot_id='kiwi-01', runs_dir='runs',
                 model=None, demo=False, episode_fn=None, kinematics_config=None, default_limits=None):
        self.server, self.robot_id, self.model = server, robot_id, model
        self.default_limits = dict(default_limits or DEFAULT_LIMITS)
        self.kinematics_config = Path(kinematics_config).expanduser().resolve() if kinematics_config else None
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
        self.requests = {}
        self.preview_lock = asyncio.Lock()
        self.reference_busy = False
        self.manual = ManualXYZ(self.runs_dir, self.server, self.kinematics_config)

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
                raise ValueError('이 실행에 사용할 저장 관찰이 없습니다. URDF 예시 자세로 먼저 확인할 수 있습니다.')
        value = calculate_preview(request, observed=observed, config=self.kinematics_config)
        if observed is not None:
            reference = read_json(path / 'pose_reference.json')
            # A reference must belong to the exact observation being displayed.
            if (reference.get('provenance', {}).get('observation_id') == observed['observation_id']
                    and reference.get('mapping_id') == value['state'].get('mapping_id')
                    and reference.get('model_hash') == value['state'].get('model_hash')):
                value['pose_reference'] = reference
        return value

    def save_pose_reference(self, packet, images):
        """Persist a read-only capture; no runnable action is created."""
        reference = build_pose_reference(packet, self.kinematics_config)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-pose-' + uuid4().hex[:8]
        record = RunRecord(self.runs_dir / run_id, self.robot_id,
                           '측정 자세 ↔ 기본 자세 · 관절/XYZ 비교', EpisodeLimits())
        metadata = read_json(record.path / 'run.json')
        atomic_json(record.path / 'run.json', {**metadata, 'mode': 'reference', 'execute': False})
        record.observation(packet, images)
        atomic_json(record.path / 'pose_reference.json', reference)
        record.event('pose_reference', reference)
        atomic_json(record.path / 'result.json', {
            'status': 'reference_saved', 'detail': '관절·XYZ 기준 자세 저장 · 이동 명령 없음',
            'automatic_resume': False, 'physical_success_verified': False, 'action_attempts': 0,
            'steps': 0, 'observations': 1, 'executed': False,
        })
        return {'run_id': run_id, 'pose_reference': reference, 'executed': False}

    async def capture_pose(self):
        if self.mode != 'live':
            raise ValueError('데모 GUI는 실제 로봇에 연결하지 않습니다.')
        if (self.reference_busy or self.manual.reserved()
                or self.task is not None and not self.task.done()):
            raise BusyError('진행 중인 관찰·에이전트·수동 이동을 마친 후 자세를 읽어주세요.')
        # Reserve before the first await so a start/prepare cannot invalidate a capture.
        self.reference_busy = True
        try:
            packet, images = decode_content(await RobotTools(ServerClient(self.server)).observe(self.robot_id))
            if packet.get('status') != 'ok':
                raise ValueError(packet.get('error') or '로봇 관찰을 읽지 못했습니다.')
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

    async def start(self, payload):
        request = StartInput.model_validate(payload)
        if request.model is None and self.model is not None:
            request = request.model_copy(update={'model': self.model})
        if self.task is not None and not self.task.done():
            raise BusyError('An episode is already active; wait for its cleanup to finish')
        if self.reference_busy or self.manual.reserved():
            raise BusyError('수동 XYZ 이동을 계산/실행 중입니다. 먼저 실행 또는 취소해주세요.')
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:8]
        pending = request.model_dump()
        pending['mode'] = self.mode
        pending['started_at'] = datetime.now(timezone.utc).isoformat()
        self.requests[run_id] = pending
        self.active_run_id, self.stopping = run_id, False
        self.task = asyncio.create_task(self._run(run_id, request), name=f'lekiwi-gui-{run_id}')
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
                # This host event is written only after the exact held partial
                # result has passed validation. It never certifies target arrival.
                pending = None
            elif event.get('event') == 'motion_warning' and event['data'].get('action_id') == pending:
                # The runner records this only after validating the warning and
                # fresh post-move observation; target arrival remains unconfirmed.
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
            self.task.cancel()
        return {'id': run_id, 'status': 'stopping'}

    async def close(self):
        await self.manual.close()
        if self.task is not None and not self.task.done():
            await self.stop(self.active_run_id)
            try:
                await asyncio.wait_for(asyncio.shield(self.task), timeout=10)
            except (TimeoutError, asyncio.CancelledError):
                # A loop stop is not a remote hardware emergency-stop acknowledgement.
                pass

    def file_path(self, run_id, relative):
        run = self._run_path(run_id)
        parts = Path(relative).parts
        if (len(parts) != 3 or parts[0] != 'observations' or not parts[1].isdigit() or
                not (IMAGE_NAME.fullmatch(parts[2]) or parts[2] in ('observation.json', 'images.json'))):
            raise FileNotFoundError('Only observation images and metadata are accessible')
        candidate = (run / relative).resolve()
        if candidate != run / relative or not candidate.is_relative_to(run) or not candidate.is_file():
            raise FileNotFoundError('Observation file not found')
        return candidate

    def _observation(self, run_id, event):
        relative = event['data'].get('path')
        if not isinstance(relative, str):
            return None
        try:
            path = self.file_path(run_id, relative + '/observation.json')
        except FileNotFoundError:
            return None
        packet = read_json(path)
        if not packet:
            return None
        manifest = event['data'].get('images')
        if not isinstance(manifest, list):
            metadata = read_json(path.with_name('images.json'))
            manifest = metadata.get('images')
        if not isinstance(manifest, list):
            manifest = [{'label': image.stem, 'path': str(image.relative_to(self._run_path(run_id))),
                         'mime_type': 'image/png' if image.suffix == '.png' else 'image/jpeg'}
                        for image in sorted(path.parent.glob('image-*')) if IMAGE_NAME.fullmatch(image.name)]
        images = []
        for item in manifest:
            if not isinstance(item, dict) or not isinstance(item.get('path'), str):
                continue
            try:
                self.file_path(run_id, item['path'])
            except FileNotFoundError:
                continue
            images.append({'label': str(item.get('label', 'camera')),
                           'url': f'/api/runs/{quote(run_id, safe="")}/files/{quote(item["path"], safe="/")}',
                           'mime_type': item.get('mime_type', 'image/jpeg')})
        return {'sequence': int(path.parent.name), 'observation_id': event['data'].get('observation_id'),
                'at': event.get('at'), 'packet': packet, 'images': images}

    def run_view(self, run_id):
        path = self._run_path(run_id, exists=run_id not in self.requests)
        record = read_json(path / 'run.json')
        request = self.requests.get(run_id) or read_json(path / 'gui_request.json') or record
        result = read_json(path / 'result.json') or None
        events = read_events(path / 'events.jsonl')
        manual_phase = self.manual.phase_for_run(run_id)
        active = self._is_active(run_id) or manual_phase is not None
        status = manual_phase or (result or {}).get('status', 'starting' if active else 'interrupted_unknown')
        if active and self.stopping:
            status = 'stopping'
        elif not active and status in ('running', 'starting'):
            status = 'interrupted_unknown'
        phase = 'starting'
        phases = {'startup_started': 'starting', 'startup_finished': 'ready',
                  'observe_started': 'observing', 'observe_finished': 'observed',
                  'planning_started': 'planning', 'planning_finished': 'planned',
                  'decision': 'decided', 'action_prepared': 'executing', 'action_started': 'executing',
                  'action_finished': 'reviewing', 'action_result': 'reviewing',
                  'motion_incomplete': 'reviewing', 'motion_warning': 'replanning',
                  'reconcile_started': 'reconciling',
                  'reconcile_finished': 'replanning', 'motion_replanning': 'replanning',
                  'motion_failure_feedback': 'replanning', 'motion_recovery_completed': 'reviewing',
                  'motion_feedback': 'reviewing', 'completion_candidate': 'confirming'}
        for event in events:
            phase = phases.get(event.get('event'), phase)
        if status not in ('running', 'starting'):
            phase = status
        # Large image metadata must not be loaded for every historical observation on each poll.
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
        run_mode = request.get('mode', record.get('mode', 'execute'))
        kinematics = next((event['data'] for event in reversed(events)
                           if event.get('event') == 'kinematics_state'), None)
        return {'id': run_id, 'active': active, 'mode': run_mode if run_mode in ('demo', 'reference') else 'live',
                'request': {'robot_id': request.get('robot_id'), 'instruction': request.get('instruction'),
                            'model': model, 'limits': request.get('limits', self.default_limits)},
                'result': result, 'status': status, 'phase': phase, 'events': events,
                'current_observation': current, 'before_observation': before, 'kinematics': kinematics}

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
                runs.append({'id': run_id, 'instruction': record.get('instruction', ''),
                             'started_at': record.get('started_at'), 'status': status, 'active': active,
                             'mode': record.get('mode') if record.get('mode') in ('demo', 'reference') else 'live'})
            except (FileNotFoundError, OSError):
                continue
        return {'mode': self.mode, 'config': {'server': self.server, 'robot_id': self.robot_id,
                                            'model': self.model, 'limits': dict(self.default_limits),
                                            'kinematics_configured': self.kinematics_config is not None},
                'capabilities': {'xyz_preview': 1, 'manual_xyz': 1 if self.mode == 'live' else 0,
                                 'pose_capture': 1 if self.mode == 'live' else 0},
                'pose_capture_busy': self.reference_busy,
                'active_run_id': active_id, 'runs': runs,
                'active': self.run_view(active_id) if active_id is not None else None}


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
        # Reservation expiry mutates a run journal; keep it on the event-loop thread.
        value['manual_xyz'] = controller.manual.snapshot()
        return JSONResponse(value)

    async def manual(request):
        try:
            if controller.mode != 'live':
                raise ValueError('데모 GUI는 실제 로봇에 연결하지 않습니다.')
            operation = request.path_params['operation']
            payload = await json_body(request)
            if operation == 'cancel':
                if payload:
                    raise ValueError('취소 요청에는 추가 필드가 필요하지 않습니다.')
                return JSONResponse(controller.manual.cancel())
            if controller.task is not None and not controller.task.done():
                raise ManualBusy('에이전트 작업이 실행 중입니다. 종료 후 수동 실험을 시작해주세요.')
            if controller.reference_busy:
                raise ManualBusy('현재 기준 자세를 읽는 중입니다. 완료 후 수동 실험을 시작해주세요.')
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
            value = await asyncio.to_thread(controller.run_view, request.path_params['run_id'])
            return JSONResponse(value)
        except (FileNotFoundError, OSError):
            return JSONResponse({'error': 'Run not found'}, 404)

    async def preview(request):
        try:
            payload = PreviewInput.model_validate(await json_body(request))
            if controller.preview_lock.locked():
                return JSONResponse({'error': '다른 미리보기 계산이 진행 중입니다. 잠시 후 다시 시도해주세요.'}, 409)
            async with controller.preview_lock:
                return JSONResponse(await asyncio.to_thread(controller.preview, payload))
        except FileNotFoundError:
            return JSONResponse({'error': '기록 또는 설정 파일을 찾을 수 없습니다.'}, 404)
        except (ValueError, TypeError, OSError) as exc:
            return JSONResponse({'error': str(exc)}, 400)

    async def capture(request):
        try:
            if await json_body(request):
                raise ValueError('자세 읽기는 추가 필드를 받지 않습니다.')
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
        Route('/api/state', state), Route('/api/runs', start, methods=['POST']),
        Route('/api/kinematics/preview', preview, methods=['POST']),
        Route('/api/kinematics/capture', capture, methods=['POST']),
        Route('/api/manual-xyz/{operation}', manual, methods=['POST']),
        Route('/api/runs/{run_id}/stop', stop, methods=['POST']),
        Route('/api/runs/{run_id}/files/{path:path}', files), Route('/api/runs/{run_id}', detail),
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
    parser.add_argument('--model')
    parser.add_argument('--runs-dir', type=Path, default=Path('runs'))
    parser.add_argument('--demo', action='store_true', help='Synthetic episode, no model or robot calls')
    parser.add_argument('--kinematics-config', type=Path, help='Verified local joint mapping for guarded XYZ execution')
    parser.add_argument('--loop', choices=('step', 'plan-act'), default='step',
                        help='step: one small action per decision; plan-act: Planning → big-skill Action Loop → Verify/replan')
    parser.add_argument('--skills-config', type=Path, help='plan-act: home/grasp/gripper skill settings')
    parser.add_argument('--camera-config', type=Path, help='plan-act: calibrated front camera for locate (optional)')
    parser.add_argument('--experience', type=Path, help='plan-act: experience JSONL (default: RUNS_DIR/experience.jsonl)')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be 1..65535')
    if args.demo and args.kinematics_config:
        parser.error('--demo cannot use --kinematics-config')
    episode_fn, limits = None, None
    if args.loop == 'plan-act':
        if args.demo or not args.kinematics_config or not args.skills_config:
            parser.error('--loop plan-act needs --kinematics-config and --skills-config (not --demo)')
        from functools import partial
        from .plan_loop import run_plan_episode
        episode_fn = partial(run_plan_episode, skills_config=args.skills_config, camera_config=args.camera_config,
                             experience_path=args.experience or args.runs_dir / 'experience.jsonl')
        limits = PLAN_ACT_LIMITS
    controller = GuiController(args.server, args.robot_id, args.runs_dir, args.model, args.demo,
                               episode_fn=episode_fn, kinematics_config=args.kinematics_config, default_limits=limits)
    import uvicorn
    print(f'LeKiwi console: http://127.0.0.1:{args.port} ({controller.mode})', flush=True)
    uvicorn.run(create_app(controller, port=args.port), host='127.0.0.1', port=args.port)


if __name__ == '__main__':
    main()
