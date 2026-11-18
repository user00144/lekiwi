"""ASPIRE-style block-by-block policy runner for LeKiwi mobile manipulation campaigns.

This module contains zero LLM prompt strings. Policies are plain Python files (`policy.py`)
authored block by block by the coding agent, validated against a restricted AST, executed
with line-cursor tracing (`>>> Line N`), and logged into structured attempt directories:

  <output_dir>/
    source/policy.py
    source/policy-result.json
    logs/policy.stdout
    trace/events.jsonl
    stages/<NNNN>_L<YY>_<tool>/
    outcome.json

Robot stages are either inner EpisodeRunner episodes (`inspect_scene`, `act`, `navigate_to` legs)
or deterministic host skills (`locate`, `pick`, `place`, `home_arm`, `drive`) run by a
`SkillExecutor`; both write the same per-stage `events.jsonl` / `result.json` artifacts.
"""
from __future__ import annotations

import ast
import asyncio
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import inspect
import json
import math
from pathlib import Path
import time
from typing import Any, Protocol

from ..episode import EpisodeLimits, atomic_json
from ..skill_session import HOST_SKILLS, MAX_DRIVE_M, MAX_TURN_DEG
from .semantic_memory import SemanticMemoryStore
from .topology import LocalizationProvider, PoseEstimate, RouteLeg, RoutePlan, TopologicalMap


POLICY_FILENAME = '<policy>'
DEFAULT_MAX_LOOP_TICKS = 500
# EpisodeRunner (step and plan-act loops) accepts completion only as `completion_claim`: two
# completion claims on independent fresh observations. It never returns the status `completed`.
STAGE_SUCCESS_STATUSES = frozenset({'completed', 'completion_claim'})
STAGE_HISTORY_LIMIT = 6
STAGE_SUMMARY_CHARS = 300
ARM_JOINT_NAMES = frozenset({'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'})

# Keys whose presence in `result` could leak privileged or ambiguous success labels;
# mirrors ASPIRE's `source/program-result.json` sanitization rule.
FORBIDDEN_RESULT_SUBSTRINGS = (
    'success',
    'reward',
    'object_pose',
    'raw_state',
    'raw_observation',
    'qpos',
    'qvel',
    'sim_state',
    'privileged',
    'ground_truth',
    'groundtruth',
    'native_env',
    'mujoco',
)

SAFE_BUILTINS: dict[str, Any] = {
    'abs': abs,
    'all': all,
    'any': any,
    'bool': bool,
    'dict': dict,
    'enumerate': enumerate,
    'float': float,
    'int': int,
    'isinstance': isinstance,
    'len': len,
    'list': list,
    'max': max,
    'min': min,
    'range': range,
    'reversed': reversed,
    'round': round,
    'set': set,
    'sorted': sorted,
    'str': str,
    'sum': sum,
    'tuple': tuple,
    'zip': zip,
    'True': True,
    'False': False,
    'None': None,
}


class PolicyValidationError(ValueError):
    """Raised when `policy.py` violates the restricted AST contract."""


class PolicyLoopLimitExceeded(RuntimeError):
    """Raised when `for`/`while` iterations exceed `max_loop_ticks`."""


class _PolicyTerminalHalt(Exception):
    """Raised internally by `finish()` or `block()` to stop policy execution cleanly."""

    def __init__(self, reason: str, summary: str, *, task_success: bool, details: dict[str, Any]):
        super().__init__(summary)
        self.reason = reason
        self.summary = summary
        self.task_success = task_success
        self.details = details


class _UncertainExecutionHalt(Exception):
    """Raised immediately when any robot stage reports `uncertain_execution`."""

    def __init__(self, tool: str, detail: str):
        super().__init__(detail)
        self.tool = tool
        self.detail = detail


class _PolicyCancelledHalt(Exception):
    """Raised when an operator stops an active policy run."""

    def __init__(self, detail: str = 'Policy stopped by operator'):
        super().__init__(detail)
        self.detail = detail


def _safe_import(name: str, globals_dict=None, locals_dict=None, fromlist=(), level: int = 0):
    if name != 'math' or fromlist or level != 0:
        raise PolicyValidationError("Only 'import math' without aliases or from-imports is permitted")
    return math


class _PolicyASTValidator(ast.NodeVisitor):
    """Enforces ASPIRE's restricted Python subset (`import math` only, no `_` names)."""

    _FORBIDDEN_NODES = (
        ast.ImportFrom,
        ast.ClassDef,
        ast.Try,
        ast.With,
        ast.AsyncWith,
        ast.AsyncFor,
        ast.AsyncFunctionDef,
        ast.Await,
        ast.Lambda,
        ast.Global,
        ast.Nonlocal,
        ast.Delete,
        ast.Yield,
        ast.YieldFrom,
        ast.Raise,
    )

    def generic_visit(self, node: ast.AST) -> None:
        if isinstance(node, self._FORBIDDEN_NODES):
            raise PolicyValidationError(
                f'Unsupported syntax {type(node).__name__} at line {getattr(node, "lineno", "?")}'
            )
        super().generic_visit(node)

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            if alias.name != 'math' or alias.asname is not None:
                raise PolicyValidationError(
                    f"Only 'import math' is permitted (got import {alias.name!r} at line {node.lineno})"
                )

    def visit_Name(self, node: ast.Name) -> None:
        if node.id.startswith('_'):
            raise PolicyValidationError(
                f"Underscore-prefixed identifier {node.id!r} is forbidden at line {node.lineno}; name throwaways explicitly"
            )
        self.generic_visit(node)

    def visit_arg(self, node: ast.arg) -> None:
        if node.arg.startswith('_'):
            raise PolicyValidationError(
                f"Underscore-prefixed parameter {node.arg!r} is forbidden at line {node.lineno}"
            )
        self.generic_visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        if node.name.startswith('_'):
            raise PolicyValidationError(
                f"Underscore-prefixed function name {node.name!r} is forbidden at line {node.lineno}"
            )
        if node.decorator_list:
            raise PolicyValidationError(f'Decorators are forbidden at line {node.lineno}')
        self.generic_visit(node)

    def visit_Attribute(self, node: ast.Attribute) -> None:
        if node.attr.startswith('_'):
            raise PolicyValidationError(
                f'Private/dunder attribute {node.attr!r} is forbidden at line {node.lineno}'
            )
        self.generic_visit(node)


class _LoopTickInjector(ast.NodeTransformer):
    """Prepends `__tick__()` to every `for` and `while` body after validation."""

    @staticmethod
    def _tick_stmt(lineno: int) -> ast.stmt:
        call = ast.Expr(
            value=ast.Call(
                func=ast.Name(id='__tick__', ctx=ast.Load()),
                args=[],
                keywords=[],
            )
        )
        return ast.copy_location(call, ast.Pass(lineno=lineno, col_offset=0))

    def visit_For(self, node: ast.For) -> ast.AST:
        self.generic_visit(node)
        node.body.insert(0, self._tick_stmt(node.lineno))
        return node

    def visit_While(self, node: ast.While) -> ast.AST:
        self.generic_visit(node)
        node.body.insert(0, self._tick_stmt(node.lineno))
        return node


def validate_and_compile_policy(source: str, filename: str = POLICY_FILENAME):
    """Validate `policy.py` source and return compiled bytecode with loop-tick guards."""
    if not isinstance(source, str) or not source.strip():
        raise PolicyValidationError('Policy source cannot be empty')
    try:
        tree = ast.parse(source, filename=filename, mode='exec')
    except SyntaxError as exc:
        raise PolicyValidationError(f'Syntax error at line {exc.lineno}: {exc.msg}') from exc

    _PolicyASTValidator().visit(tree)
    instrumented = _LoopTickInjector().visit(tree)
    ast.fix_missing_locations(instrumented)
    return compile(instrumented, filename=filename, mode='exec')


def highlight_policy_source(source: str, active_lineno: int, *, radius: int = 4) -> str:
    """Render a window around `active_lineno` with `>>>` marking the executing call."""
    lines = source.splitlines()
    if not lines:
        return ''
    lo = max(1, active_lineno - radius)
    hi = min(len(lines), active_lineno + radius)
    rendered: list[str] = []
    for idx in range(lo, hi + 1):
        prefix = '>>>' if idx == active_lineno else '   '
        rendered.append(f'{prefix} {idx:3d} | {lines[idx - 1]}')
    return '\n'.join(rendered)


def sanitize_policy_result(value: Any) -> Any:
    """Strip forbidden report keys and ensure JSON-serializable output."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (list, tuple)):
        return [sanitize_policy_result(item) for item in value]
    if isinstance(value, dict):
        cleaned: dict[str, Any] = {}
        for raw_key, raw_val in value.items():
            key_str = str(raw_key)
            lowered = key_str.lower()
            if any(bad in lowered for bad in FORBIDDEN_RESULT_SUBSTRINGS):
                continue
            cleaned[key_str] = sanitize_policy_result(raw_val)
        return cleaned
    return str(value)


class NavigationExecutor(Protocol):
    def execute_route(self, robot_id: str, route: RoutePlan, stage_dir: Path) -> dict[str, Any]: ...


class InnerActionExecutor(Protocol):
    def run_subgoal(
        self,
        *,
        robot_id: str,
        kind: str,
        subgoal: str,
        highlighted_policy: str,
        limits: EpisodeLimits,
        stage_dir: Path,
        history: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]: ...


class SkillExecutor(Protocol):
    """Runs one deterministic host skill (see `HOST_SKILLS`) as a guarded robot stage."""

    def run_skill(self, *, robot_id: str, skill: str, args: dict[str, Any], stage_dir: Path) -> dict[str, Any]: ...


class LiveLocalizationProvider:
    """Localization provider for live hardware runs (file-backed SLAM pose or SLAM-free fallback)."""

    def __init__(self, pose_file: str | Path | None = None, *, default_floor: str = 'local'):
        self._pose_file = Path(pose_file) if pose_file else None
        self._default_floor = default_floor

    def _read(self, *, relocalize_requested: bool) -> PoseEstimate:
        if self._pose_file is not None and self._pose_file.is_file():
            data = json.loads(self._pose_file.read_text(encoding='utf-8'))
            return PoseEstimate(
                floor=str(data.get('floor', self._default_floor)),
                x_m=float(data.get('x_m', 0.0)),
                y_m=float(data.get('y_m', 0.0)),
                yaw_deg=float(data.get('yaw_deg', 0.0)),
                confidence=float(data.get('confidence', 1.0)),
                relocalized=bool(data.get('relocalized', True)),
            )
        # SLAM-free visual-only mode: confidence=0 signals that global coordinates are unavailable.
        return PoseEstimate(
            floor=self._default_floor,
            x_m=0.0,
            y_m=0.0,
            yaw_deg=0.0,
            confidence=0.0,
            relocalized=False if not relocalize_requested else False,
        )

    def get_pose(self, robot_id: str) -> PoseEstimate:
        del robot_id
        return self._read(relocalize_requested=False)

    def relocalize(self, robot_id: str) -> PoseEstimate:
        del robot_id
        return self._read(relocalize_requested=True)


def _inspect_stage_engagement(stage_dir: Path) -> dict[str, bool]:
    """Check `EpisodeRunner` events.jsonl to see whether confirmed arm and gripper actions ran."""
    events_path = stage_dir / 'events.jsonl'
    if not events_path.is_file():
        return {'arm_moved': False, 'gripper_moved': False, 'arm_engaged': False}

    prepared: dict[str, dict[str, Any]] = {}
    confirmed_ids: set[str] = set()
    for line in events_path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = row.get('event')
        data = row.get('data') or {}
        if kind == 'action_prepared' and isinstance(data.get('action_id'), str):
            prepared[data['action_id']] = data
        elif kind == 'action_result' and isinstance(data.get('action_id'), str):
            packet = data.get('packet') or {}
            execution = packet.get('execution') or {}
            if execution.get('command_sent') is True:
                confirmed_ids.add(data['action_id'])

    arm_moved = False
    gripper_moved = False
    for action_id in confirmed_ids:
        cmd = prepared.get(action_id) or {}
        joints = cmd.get('joints') or {}
        if any(joints.get(name) is not None for name in ARM_JOINT_NAMES):
            arm_moved = True
        if joints.get('gripper') is not None:
            gripper_moved = True

    return {
        'arm_moved': arm_moved,
        'gripper_moved': gripper_moved,
        'arm_engaged': arm_moved and gripper_moved,
    }


def episode_stage_result(ep_res: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Normalize one EpisodeRunner result for policy.py; `ok` is True only for accepted completion."""
    raw_status = str(ep_res.get('status', 'host_error'))
    ok = raw_status in STAGE_SUCCESS_STATUSES
    status = 'completed' if ok else raw_status
    claim = ep_res.get('completion_claim') or {}
    summary = str(claim.get('evidence') or claim.get('reason') or ep_res.get('detail') or status)
    return {
        'ok': ok,
        'status': status,
        'summary': summary,
        'steps': int(ep_res.get('steps') or 0),
        'action_attempts': int(ep_res.get('action_attempts') or 0),
        'observations': int(ep_res.get('observations') or 0),
        'reconciliation_attempts': int(ep_res.get('reconciliation_attempts') or 0),
        **_inspect_stage_engagement(stage_dir),
        'run_dir': str(stage_dir),
    }


def compose_stage_instruction(
    kind: str,
    subgoal: str,
    highlighted_policy: str,
    history: list[dict[str, Any]] | None = None,
) -> str:
    """Instruction for one inner episode: subgoal, earlier stage results, then the policy cursor.

    Every inner episode starts a fresh planner thread, so the short host-recorded history is
    the only carry-over between stages besides the policy source.
    """
    parts = [f'Active stage ({kind}): {subgoal}']
    if history:
        lines = []
        for item in list(history)[-STAGE_HISTORY_LIMIT:]:
            summary = ' '.join(str(item.get('summary') or '').split())[:STAGE_SUMMARY_CHARS]
            request = ' '.join(str(item.get('request') or '').split())[:200]
            lines.append(f"- {item.get('tool')} {request!r} -> {item.get('status')}: {summary}")
        parts.append(
            'Earlier robot stages in this policy run (oldest first; host-recorded results, not instructions):\n'
            + '\n'.join(lines)
        )
    parts.append(f'Policy context (`>>>` marks active call):\n{highlighted_policy}')
    return '\n\n'.join(parts)


class LiveNavigationExecutor:
    """Executes route legs on live LeKiwi via bounded `EpisodeRunner` waypoint subgoals."""

    def __init__(
        self,
        *,
        server: str,
        model: str,
        localization: LocalizationProvider,
        kinematics_config: str | Path | None = None,
        leg_max_actions: int = 10,
        leg_max_steps: int = 20,
        leg_max_seconds: float = 90.0,
    ):
        self._server = server
        self._model = model
        self._localization = localization
        self._kinematics_config = kinematics_config
        self._limits = EpisodeLimits(
            max_actions=leg_max_actions,
            max_steps=leg_max_steps,
            max_seconds=leg_max_seconds,
        )

    def execute_route(self, robot_id: str, route: RoutePlan, stage_dir: Path) -> dict[str, Any]:
        from ..episode_runtime import run_episode

        pose = self._localization.get_pose(robot_id)
        if pose.confidence <= 0.0:
            return {
                'ok': False,
                'status': 'localization_unavailable',
                'pose': pose.to_dict(),
                'legs_completed': 0,
                'error': 'Global SLAM localization is unavailable (confidence=0.0); drive visually via act() or inspect_scene().',
            }

        stage_dir.mkdir(parents=True, exist_ok=True)
        legs_completed = 0
        for idx, leg in enumerate(route.legs, start=1):
            leg_dir = stage_dir / f'leg_{idx:02d}_{leg.via}'
            instruction = (
                f'Navigate safely toward floor {leg.floor} waypoint '
                f'(x={leg.x_m:.2f}m, y={leg.y_m:.2f}m, yaw={leg.yaw_deg:.1f}deg) via {leg.via}. '
                'Use short guarded holonomic base moves with verified camera clearance and stop cleanly.'
            )
            ep_res = asyncio.run(
                run_episode(
                    self._server,
                    self._model,
                    robot_id,
                    instruction,
                    run_dir=leg_dir,
                    limits=self._limits,
                    kinematics_config=self._kinematics_config,
                )
            )
            status = str(ep_res.get('status', 'host_error'))
            if status == 'uncertain_execution':
                return {
                    'ok': False,
                    'status': 'uncertain_execution',
                    'pose': self._localization.get_pose(robot_id).to_dict(),
                    'legs_completed': legs_completed,
                    'error': str(ep_res.get('detail') or 'Uncertain execution during navigation leg'),
                }
            if status not in STAGE_SUCCESS_STATUSES:
                return {
                    'ok': False,
                    'status': status,
                    'pose': self._localization.get_pose(robot_id).to_dict(),
                    'legs_completed': legs_completed,
                    'error': str(ep_res.get('detail') or f'Navigation leg {idx} ended with {status}'),
                }
            legs_completed += 1

        updated_pose = self._localization.get_pose(robot_id)
        return {
            'ok': True,
            'status': 'completed',
            'pose': updated_pose.to_dict(),
            'legs_completed': legs_completed,
            'error': None,
        }


class LiveInnerActionExecutor:
    """Delegates `inspect_scene()` and `act()` to the host-owned `EpisodeRunner`.

    `act_episode_fn` (same calling convention as `episode_runtime.run_episode`, e.g. the
    plan-act loop with calibrated locate/pick skills) replaces the step loop for `act()` only.
    """

    def __init__(
        self,
        *,
        server: str,
        model: str | None,
        kinematics_config: str | Path | None = None,
        act_episode_fn: Any | None = None,
    ):
        self._server = server
        self._model = model
        self._kinematics_config = kinematics_config
        self._act_episode_fn = act_episode_fn

    def run_subgoal(
        self,
        *,
        robot_id: str,
        kind: str,
        subgoal: str,
        highlighted_policy: str,
        limits: EpisodeLimits,
        stage_dir: Path,
        history: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        from ..episode_runtime import run_episode

        episode_fn = self._act_episode_fn if kind == 'act' and self._act_episode_fn is not None else run_episode
        ep_res = asyncio.run(
            episode_fn(
                self._server,
                self._model,
                robot_id,
                compose_stage_instruction(kind, subgoal, highlighted_policy, history),
                run_dir=stage_dir,
                limits=limits,
                kinematics_config=self._kinematics_config,
            )
        )
        return episode_stage_result(ep_res, stage_dir)


def skill_stage_result(raw: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    """Flatten one `skill_session` result for policy.py; `ok` means the skill finished as intended."""
    status = str(raw.get('status', 'host_error'))
    payload = dict(raw.get('skill_result') or {})
    skill = str(raw.get('skill', ''))
    error = payload.pop('error', None) or (None if status == 'completed' else raw.get('detail'))
    if skill == 'locate' and payload.get('found'):
        summary = (f"{payload.get('label')} at xyz_m={payload.get('xyz_m')} "
                   f"(distance {payload.get('horizontal_distance_m')} m, reachable={payload.get('reachable')})")
    elif skill == 'pick' and status == 'completed':
        summary = (f"picked: gripper {payload.get('gripper_after_close')} after close, "
                   f"object_between_jaws_likely={payload.get('object_between_jaws_likely')}")
    else:
        summary = str(error or payload.get('note') or status)
    return {
        **payload,
        'ok': status == 'completed',
        'status': status,
        'skill': skill,
        'summary': summary,
        'error': error,
        'arm_xyz_m': raw.get('arm_xyz_m'),
        'gripper_percent': raw.get('gripper_percent'),
        'primitives': list(raw.get('primitives') or [])[-8:],
        'action_attempts': int(raw.get('action_attempts') or 0),
        **_inspect_stage_engagement(stage_dir),
        'run_dir': str(stage_dir),
    }


class LiveSkillExecutor:
    """Runs host skills (`HOST_SKILLS`) on live LeKiwi through `skill_session.run_skill_session`.

    `run_sync(make_coroutine, stage_dir)` lets the GUI run the session on its cancellable bridge;
    by default each skill runs in its own `asyncio.run`.
    """

    def __init__(
        self,
        *,
        server: str,
        model: str | None,
        kinematics_config: str | Path,
        skills_config: str | Path,
        camera_config: str | Path | None = None,
        run_sync: Any | None = None,
    ):
        self._server = server
        self._model = model
        self._kinematics_config = kinematics_config
        self._skills_config = skills_config
        self._camera_config = camera_config
        self._run_sync = run_sync

    def run_skill(self, *, robot_id: str, skill: str, args: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
        from ..skill_session import run_skill_session

        def make_coroutine():
            return run_skill_session(
                self._server,
                self._model,
                robot_id,
                skill,
                args,
                run_dir=stage_dir,
                kinematics_config=self._kinematics_config,
                skills_config=self._skills_config,
                camera_config=self._camera_config,
            )

        raw = self._run_sync(make_coroutine, stage_dir) if self._run_sync is not None else asyncio.run(make_coroutine())
        return skill_stage_result(raw if isinstance(raw, dict) else {}, stage_dir)


@dataclass
class SimObject:
    object_id: str
    label: str
    floor: str
    x_m: float
    y_m: float
    graspable: bool = True
    held: bool = False


@dataclass
class SimulatedCampaignWorld:
    """Deterministic multi-floor world for offline unit testing and dry-run block development."""

    pose: PoseEstimate = field(
        default_factory=lambda: PoseEstimate(floor='1F', x_m=0.0, y_m=0.0, yaw_deg=0.0, confidence=1.0, relocalized=True)
    )
    relocalized_pose: PoseEstimate | None = None
    objects: dict[str, SimObject] = field(default_factory=dict)
    reach_radius_m: float = 0.65
    view_radius_m: float = 1.8
    force_uncertain_on_tool: str | None = None
    # Host-skill dry run: calibrated floor reach of the real SO-101 pick skill (arm_base metres).
    skill_reach_x_m: tuple[float, float] = (0.17, 0.30)
    skill_reach_y_m: float = 0.08
    camera_half_fov_deg: float = 35.0
    floor_grasp_z_m: float = -0.09
    grasp_tolerance_m: float = 0.04

    @classmethod
    def default_two_floor_world(cls) -> 'SimulatedCampaignWorld':
        world = cls(
            pose=PoseEstimate(floor='1F', x_m=0.5, y_m=0.5, yaw_deg=0.0, confidence=0.35, relocalized=False),
            relocalized_pose=PoseEstimate(floor='1F', x_m=0.5, y_m=0.5, yaw_deg=0.0, confidence=0.98, relocalized=True),
        )
        world.objects = {
            'red_mug': SimObject('red_mug', 'red ceramic mug', floor='2F', x_m=4.2, y_m=1.5),
            'blue_bottle': SimObject('blue_bottle', 'blue vitamin bottle', floor='1F', x_m=2.4, y_m=0.8),
        }
        return world

    def get_pose(self, robot_id: str) -> PoseEstimate:
        del robot_id
        return self.pose

    def relocalize(self, robot_id: str) -> PoseEstimate:
        del robot_id
        if self.relocalized_pose is not None:
            self.pose = self.relocalized_pose
        else:
            self.pose = PoseEstimate(
                floor=self.pose.floor,
                x_m=self.pose.x_m,
                y_m=self.pose.y_m,
                yaw_deg=self.pose.yaw_deg,
                confidence=0.99,
                relocalized=True,
            )
        return self.pose

    def execute_route(self, robot_id: str, route: RoutePlan, stage_dir: Path) -> dict[str, Any]:
        del robot_id
        stage_dir.mkdir(parents=True, exist_ok=True)
        if self.force_uncertain_on_tool == 'navigate_to':
            atomic_json(stage_dir / 'result.json', {'status': 'uncertain_execution'})
            return {
                'ok': False,
                'status': 'uncertain_execution',
                'pose': self.pose.to_dict(),
                'legs_completed': 0,
                'error': 'Simulated uncertain_execution during base navigation',
            }
        if not self.pose.relocalized or self.pose.confidence < 0.5:
            return {
                'ok': False,
                'status': 'relocalization_required',
                'pose': self.pose.to_dict(),
                'legs_completed': 0,
                'error': 'Pose confidence < 0.5; call relocalize() before multi-waypoint navigation',
            }
        if not route.legs:
            return {
                'ok': True,
                'status': 'completed',
                'pose': self.pose.to_dict(),
                'legs_completed': 0,
                'error': None,
            }
        final_leg: RouteLeg = route.legs[-1]
        self.pose = PoseEstimate(
            floor=final_leg.floor,
            x_m=final_leg.x_m,
            y_m=final_leg.y_m,
            yaw_deg=final_leg.yaw_deg,
            confidence=self.pose.confidence,
            relocalized=True,
        )
        for obj in self.objects.values():
            if obj.held:
                obj.floor = self.pose.floor
                obj.x_m = self.pose.x_m
                obj.y_m = self.pose.y_m
        atomic_json(stage_dir / 'result.json', {'status': 'completed', 'pose': self.pose.to_dict()})
        return {
            'ok': True,
            'status': 'completed',
            'pose': self.pose.to_dict(),
            'legs_completed': len(route.legs),
            'error': None,
        }

    def _visible_objects(self) -> list[tuple[SimObject, float]]:
        visible: list[tuple[SimObject, float]] = []
        for obj in self.objects.values():
            if obj.held:
                visible.append((obj, 0.0))
                continue
            if obj.floor != self.pose.floor:
                continue
            dist = math.hypot(obj.x_m - self.pose.x_m, obj.y_m - self.pose.y_m)
            if dist <= self.view_radius_m:
                visible.append((obj, round(dist, 3)))
        visible.sort(key=lambda pair: pair[1])
        return visible

    def _match_target(self, text: str) -> SimObject | None:
        lowered = text.lower()
        for obj in self.objects.values():
            tokens = [tok for tok in obj.label.lower().split() if len(tok) >= 3]
            if obj.object_id.lower() in lowered or any(tok in lowered for tok in tokens):
                return obj
        return None

    def run_subgoal(
        self,
        *,
        robot_id: str,
        kind: str,
        subgoal: str,
        highlighted_policy: str,
        limits: EpisodeLimits,
        stage_dir: Path,
        history: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        del robot_id, limits, history
        stage_dir.mkdir(parents=True, exist_ok=True)
        (stage_dir / 'observations').mkdir(exist_ok=True)
        if self.force_uncertain_on_tool == kind:
            payload = {
                'ok': False,
                'status': 'uncertain_execution',
                'summary': f'Simulated uncertain_execution in {kind}',
                'steps': 1,
                'action_attempts': 1,
                'observations': 1,
                'reconciliation_attempts': 0,
                'arm_moved': False,
                'gripper_moved': False,
                'arm_engaged': False,
                'visible_objects': [],
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        visible = self._visible_objects()
        visible_list = [
            {
                'object_id': obj.object_id,
                'label': obj.label,
                'floor': obj.floor,
                'x_m': obj.x_m,
                'y_m': obj.y_m,
                'distance_m': dist,
                'held': obj.held,
            }
            for obj, dist in visible
        ]

        if kind == 'inspect_scene':
            summary = (
                f"Visible on {self.pose.floor}: " + ', '.join(f"{o['label']} ({o['distance_m']}m)" for o in visible_list)
                if visible_list
                else f'No target objects visible near ({self.pose.x_m:.2f}, {self.pose.y_m:.2f}) on {self.pose.floor}'
            )
            payload = {
                'ok': True,
                'status': 'completed',
                'summary': summary,
                'steps': 2,
                'action_attempts': 1,
                'observations': 2,
                'reconciliation_attempts': 0,
                'arm_moved': False,
                'gripper_moved': False,
                'arm_engaged': False,
                'visible_objects': visible_list,
                'highlighted_policy': highlighted_policy,
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        lowered = subgoal.lower()
        target = self._match_target(subgoal)

        # Place / release handling
        if any(word in lowered for word in ('place', 'release', 'put down', 'drop')):
            held_objs = [o for o in self.objects.values() if o.held]
            if not held_objs:
                payload = {
                    'ok': False,
                    'status': 'blocked',
                    'summary': 'Cannot place: gripper is not holding an object',
                    'steps': 2,
                    'action_attempts': 0,
                    'observations': 2,
                    'reconciliation_attempts': 0,
                    'arm_moved': False,
                    'gripper_moved': False,
                    'arm_engaged': False,
                    'visible_objects': visible_list,
                    'run_dir': str(stage_dir),
                }
                atomic_json(stage_dir / 'result.json', payload)
                return payload
            placed = held_objs[0]
            placed.held = False
            placed.floor = self.pose.floor
            placed.x_m = round(self.pose.x_m + 0.25, 3)
            placed.y_m = round(self.pose.y_m, 3)
            payload = {
                'ok': True,
                'status': 'completed',
                'summary': f'Placed and released {placed.label} at ({placed.x_m:.2f}, {placed.y_m:.2f}) on {placed.floor}',
                'steps': 4,
                'action_attempts': 3,
                'observations': 4,
                'reconciliation_attempts': 0,
                'arm_moved': True,
                'gripper_moved': True,
                'arm_engaged': True,
                'visible_objects': visible_list,
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        if target is None or target.floor != self.pose.floor:
            payload = {
                'ok': False,
                'status': 'blocked',
                'summary': f'Target for {subgoal!r} is not visible on floor {self.pose.floor}',
                'steps': 2,
                'action_attempts': 0,
                'observations': 2,
                'reconciliation_attempts': 0,
                'arm_moved': False,
                'gripper_moved': False,
                'arm_engaged': False,
                'visible_objects': visible_list,
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        dist = math.hypot(target.x_m - self.pose.x_m, target.y_m - self.pose.y_m)
        if dist > self.view_radius_m:
            payload = {
                'ok': False,
                'status': 'blocked',
                'summary': f'{target.label} is outside camera range ({dist:.2f}m > {self.view_radius_m:.2f}m)',
                'steps': 2,
                'action_attempts': 0,
                'observations': 2,
                'reconciliation_attempts': 0,
                'arm_moved': False,
                'gripper_moved': False,
                'arm_engaged': False,
                'visible_objects': visible_list,
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        # Local base+arm visual adjustment inside `act()` can close a moderate visible gap up to 1.2m;
        # beyond that, coarse navigation must bring the base closer first.
        if dist > 1.20:
            payload = {
                'ok': False,
                'status': 'blocked',
                'summary': f'{target.label} visible at {dist:.2f}m but too far for local manipulation; navigate closer first',
                'steps': 3,
                'action_attempts': 1,
                'observations': 3,
                'reconciliation_attempts': 0,
                'arm_moved': False,
                'gripper_moved': False,
                'arm_engaged': False,
                'visible_objects': visible_list,
                'run_dir': str(stage_dir),
            }
            atomic_json(stage_dir / 'result.json', payload)
            return payload

        if dist > self.reach_radius_m:
            # Step base locally within `act()` into arm reach, just like EpisodeRunner holonomic base+arm loop.
            ratio = (dist - 0.30) / dist
            self.pose = PoseEstimate(
                floor=self.pose.floor,
                x_m=round(self.pose.x_m + (target.x_m - self.pose.x_m) * ratio, 3),
                y_m=round(self.pose.y_m + (target.y_m - self.pose.y_m) * ratio, 3),
                yaw_deg=self.pose.yaw_deg,
                confidence=self.pose.confidence,
                relocalized=self.pose.relocalized,
            )

        target.held = True
        payload = {
            'ok': True,
            'status': 'completed',
            'summary': f'Grasped and lifted {target.label} on {self.pose.floor} with two-observation verification',
            'steps': 6,
            'action_attempts': 4,
            'observations': 6,
            'reconciliation_attempts': 0,
            'arm_moved': True,
            'gripper_moved': True,
            'arm_engaged': True,
            'held_object': target.object_id,
            'visible_objects': visible_list,
            'run_dir': str(stage_dir),
        }
        atomic_json(stage_dir / 'result.json', payload)
        return payload

    # -- host skills (dry run of locate / pick / place / home_arm / drive) -------------------
    def _to_robot(self, x_m: float, y_m: float) -> tuple[float, float]:
        yaw = math.radians(self.pose.yaw_deg)
        dx, dy = x_m - self.pose.x_m, y_m - self.pose.y_m
        return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy

    def _to_world(self, forward_m: float, left_m: float) -> tuple[float, float]:
        yaw = math.radians(self.pose.yaw_deg)
        return (self.pose.x_m + math.cos(yaw) * forward_m - math.sin(yaw) * left_m,
                self.pose.y_m + math.sin(yaw) * forward_m + math.cos(yaw) * left_m)

    def _in_skill_reach(self, xyz: list[float]) -> bool:
        low, high = self.skill_reach_x_m
        return low <= xyz[0] <= high and abs(xyz[1]) <= self.skill_reach_y_m

    def run_skill(self, *, robot_id: str, skill: str, args: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
        del robot_id
        stage_dir.mkdir(parents=True, exist_ok=True)
        payload: dict[str, Any] = {'skill': skill, 'arm_moved': False, 'gripper_moved': False, 'arm_engaged': False}
        held = [obj for obj in self.objects.values() if obj.held]
        if self.force_uncertain_on_tool == skill:
            payload.update(ok=False, status='uncertain_execution', error=f'Simulated uncertain_execution in {skill}')
        elif skill == 'locate':
            target = self._match_target(str(args.get('label', '')))
            found = None
            if target is not None and not target.held and target.floor == self.pose.floor:
                forward, left = self._to_robot(target.x_m, target.y_m)
                distance = math.hypot(forward, left)
                bearing = math.degrees(math.atan2(left, forward))
                if forward > 0 and distance <= self.view_radius_m and abs(bearing) <= self.camera_half_fov_deg:
                    xyz = [round(forward, 4), round(left, 4), self.floor_grasp_z_m]
                    found = {'label': target.label, 'xyz_m': xyz, 'horizontal_distance_m': round(distance, 4),
                             'reachable': self._in_skill_reach(xyz), 'in_calibrated_region': distance <= 0.6}
            if found is None:
                payload.update(ok=False, status='not_found', found=False,
                               summary=f"{args.get('label')!r} is not visible in the front camera")
            else:
                payload.update(ok=True, status='completed', found=True, **found,
                               summary=f"{found['label']} at {found['xyz_m']} (reachable={found['reachable']})")
        elif skill == 'drive':
            forward, left = float(args.get('forward_m', 0.0)), float(args.get('left_m', 0.0))
            x_m, y_m = self._to_world(forward, left)
            self.pose = PoseEstimate(floor=self.pose.floor, x_m=round(x_m, 4), y_m=round(y_m, 4),
                                     yaw_deg=round(self.pose.yaw_deg + float(args.get('turn_deg', 0.0)), 3),
                                     confidence=self.pose.confidence, relocalized=self.pose.relocalized)
            for obj in held:
                obj.floor, obj.x_m, obj.y_m = self.pose.floor, self.pose.x_m, self.pose.y_m
            payload.update(ok=True, status='completed', summary=f'Drove {forward:+.2f} m forward, {left:+.2f} m left')
        elif skill == 'home_arm':
            payload.update(ok=True, status='completed', arm_moved=True, summary='Arm at home pose')
        elif skill == 'pick':
            xyz = [float(value) for value in args['xyz_m']]
            if held:
                payload.update(ok=False, status='skill_failed', error='Already holding an object; place it first')
            elif not self._in_skill_reach(xyz):
                payload.update(ok=False, status='skill_failed', error=f'pick hover: IK not executable at {xyz}')
            else:
                target = None
                for obj in self.objects.values():
                    if obj.floor == self.pose.floor and obj.graspable:
                        forward, left = self._to_robot(obj.x_m, obj.y_m)
                        if math.hypot(forward - xyz[0], left - xyz[1]) <= self.grasp_tolerance_m:
                            target = obj
                            break
                payload.update(arm_moved=True, gripper_moved=True, arm_engaged=True)
                if target is None:
                    payload.update(ok=False, status='skill_failed', object_between_jaws_likely=False,
                                   error='after lift: the object is no longer between the jaws')
                else:
                    target.held = True
                    payload.update(ok=True, status='completed', object_between_jaws_likely=True,
                                   target_xyz_m=xyz, summary=f'Picked and lifted {target.label}')
        elif skill == 'place':
            xyz = [float(value) for value in args['xyz_m']]
            if not held:
                payload.update(ok=False, status='skill_failed', error='before place: nothing between the jaws')
            elif not self._in_skill_reach(xyz):
                payload.update(ok=False, status='skill_failed', error=f'place hover: IK not executable at {xyz}')
            else:
                obj = held[0]
                obj.held = False
                x_m, y_m = self._to_world(xyz[0], xyz[1])
                obj.floor, obj.x_m, obj.y_m = self.pose.floor, round(x_m, 4), round(y_m, 4)
                payload.update(ok=True, status='completed', arm_moved=True, gripper_moved=True, arm_engaged=True,
                               target_xyz_m=xyz, summary=f'Placed {obj.label}')
        else:
            payload.update(ok=False, status='invalid_arguments', error=f'Unknown host skill {skill!r}')
        payload.update(pose=self.pose.to_dict(), run_dir=str(stage_dir))
        atomic_json(stage_dir / 'result.json', payload)
        return payload


@dataclass(frozen=True)
class PolicyRunOutcome:
    program_ok: bool
    task_success: bool
    navigation_reached: bool
    termination_reason: str
    summary: str
    stages_executed: int
    calls_executed: int
    loop_ticks: int
    elapsed_s: float
    output_dir: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return None
    return float(value)


def _xyz_argument(value: Any) -> list[float] | None:
    """An arm_base target: three finite metres within 1 m of the arm base."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    xyz = [_finite_number(item) for item in value]
    if any(item is None for item in xyz) or math.hypot(*xyz) >= 1.0:
        return None
    return xyz


class CampaignPolicyRunner:
    """Executes one `policy.py` attempt and writes ASPIRE-compatible run artifacts."""

    def __init__(
        self,
        *,
        task_language: str,
        robot_id: str,
        mode: str,
        semantic_memory: SemanticMemoryStore,
        topology: TopologicalMap,
        localization: LocalizationProvider,
        navigation_executor: NavigationExecutor,
        action_executor: InnerActionExecutor,
        max_loop_ticks: int = DEFAULT_MAX_LOOP_TICKS,
        goal_checker: Any | None = None,
        cancel_check: Any | None = None,
        skill_executor: SkillExecutor | None = None,
    ):
        self.task_language = task_language
        self.robot_id = robot_id
        self.mode = mode
        self.semantic_memory = semantic_memory
        self.topology = topology
        self.localization = localization
        self.navigation_executor = navigation_executor
        self.action_executor = action_executor
        self.max_loop_ticks = max_loop_ticks
        self.goal_checker = goal_checker
        self.cancel_check = cancel_check
        self.skill_executor = skill_executor

    @staticmethod
    def _active_policy_lineno(source_lines: list[str]) -> tuple[int, str]:
        frame = inspect.currentframe()
        try:
            cursor = frame.f_back if frame is not None else None
            while cursor is not None:
                if cursor.f_code.co_filename == POLICY_FILENAME:
                    lineno = int(cursor.f_lineno)
                    text = source_lines[lineno - 1].strip() if 1 <= lineno <= len(source_lines) else ''
                    return lineno, text
                cursor = cursor.f_back
        finally:
            del frame
        return 1, source_lines[0].strip() if source_lines else ''

    def run_policy_file(self, policy_path: str | Path, output_dir: str | Path) -> PolicyRunOutcome:
        source = Path(policy_path).read_text(encoding='utf-8')
        return self.run_policy_source(source, output_dir)

    def run_policy_source(self, source: str, output_dir: str | Path) -> PolicyRunOutcome:
        out = Path(output_dir).resolve()
        source_dir = out / 'source'
        logs_dir = out / 'logs'
        trace_dir = out / 'trace'
        stages_dir = out / 'stages'
        for directory in (source_dir, logs_dir, trace_dir, stages_dir):
            directory.mkdir(parents=True, exist_ok=True)

        (source_dir / 'policy.py').write_text(source, encoding='utf-8')
        stdout_path = logs_dir / 'policy.stdout'
        stdout_path.write_text('', encoding='utf-8')
        events_path = trace_dir / 'events.jsonl'
        events_path.write_text('', encoding='utf-8')

        started = time.monotonic()
        source_lines = source.splitlines()
        call_seq = 0
        stage_seq = 0
        completed_stages = 0
        loop_ticks = 0
        navigation_reached = False
        stage_history: list[dict[str, Any]] = []

        def remember_stage(tool: str, request: str, res: dict[str, Any]) -> None:
            stage_history.append({
                'tool': tool,
                'request': request,
                'status': res.get('status'),
                'summary': res.get('summary') or res.get('error') or '',
            })

        def append_event(tool: str, args: dict[str, Any], output: dict[str, Any], lineno: int, line_text: str) -> None:
            nonlocal call_seq
            call_seq += 1
            record = {
                'seq': call_seq,
                'at': datetime.now(timezone.utc).isoformat(),
                'elapsed_s': round(time.monotonic() - started, 3),
                'tool': tool,
                'lineno': lineno,
                'line_text': line_text,
                'highlighted_policy': highlight_policy_source(source, lineno),
                'args': sanitize_policy_result(args),
                'output': sanitize_policy_result(output),
            }
            with events_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')

        def policy_print(*values: Any, sep: str = ' ', end: str = '\n') -> None:
            text = sep.join(str(v) for v in values) + end
            with stdout_path.open('a', encoding='utf-8') as stream:
                stream.write(text)

        def check_cancelled() -> None:
            if self.cancel_check is not None and bool(self.cancel_check()):
                raise _PolicyCancelledHalt('Policy stopped; no further blocks dispatched')

        def tick_guard() -> None:
            nonlocal loop_ticks
            check_cancelled()
            loop_ticks += 1
            if loop_ticks > self.max_loop_ticks:
                raise PolicyLoopLimitExceeded(
                    f'Loop iteration budget ({self.max_loop_ticks}) exceeded'
                )

        def api_get_task_context() -> dict[str, Any]:
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            res = {'language': self.task_language, 'robot_id': self.robot_id, 'mode': self.mode}
            append_event('get_task_context', {}, res, lineno, line_text)
            return res

        def api_get_pose() -> dict[str, Any]:
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            pose = self.localization.get_pose(self.robot_id)
            res = {'ok': pose.confidence > 0.0, **pose.to_dict()}
            append_event('get_pose', {}, res, lineno, line_text)
            return res

        def api_relocalize() -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            pose = self.localization.relocalize(self.robot_id)
            res = {'ok': pose.relocalized and pose.confidence > 0.0, **pose.to_dict()}
            append_event('relocalize', {}, res, lineno, line_text)
            return res

        def api_search_memory(query: str, floor: str | None = None, top_k: int = 3) -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            hits = self.semantic_memory.search(query, floor=floor, top_k=top_k)
            res = {
                'ok': True,
                'count': len(hits),
                'hits': [hit.to_dict() for hit in hits],
            }
            append_event('search_memory', {'query': query, 'floor': floor, 'top_k': top_k}, res, lineno, line_text)
            return res

        def api_save_memory(
            text: str,
            floor: str | None = None,
            x_m: float | None = None,
            y_m: float | None = None,
            yaw_deg: float | None = None,
            room: str = '',
            tags: tuple[str, ...] | list[str] = (),
        ) -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            pose = self.localization.get_pose(self.robot_id)
            use_floor = floor if floor is not None else pose.floor
            use_x = float(x_m) if x_m is not None else pose.x_m
            use_y = float(y_m) if y_m is not None else pose.y_m
            use_yaw = float(yaw_deg) if yaw_deg is not None else pose.yaw_deg
            # Without SLAM (confidence 0) the default pose is a placeholder, not a map position.
            localized = (x_m is not None and y_m is not None) or pose.confidence > 0.0
            hit = self.semantic_memory.upsert(
                text,
                floor=use_floor,
                x_m=use_x,
                y_m=use_y,
                yaw_deg=use_yaw,
                room=room,
                tags=tags,
                robot_id=self.robot_id,
                localized=localized,
            )
            res = {'ok': True, 'record': hit.to_dict()}
            append_event(
                'save_memory',
                {'text': text, 'floor': use_floor, 'x_m': use_x, 'y_m': use_y, 'yaw_deg': use_yaw, 'room': room,
                 'localized': localized},
                res,
                lineno,
                line_text,
            )
            return res

        def api_mark_absent(record_id_or_query: str, floor: str | None = None, note: str = 'not_found') -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            updated = self.semantic_memory.mark_absent(record_id_or_query, floor=floor, note=note)
            res = {'ok': updated > 0, 'updated': updated}
            append_event(
                'mark_absent',
                {'record_id_or_query': record_id_or_query, 'floor': floor, 'note': note},
                res,
                lineno,
                line_text,
            )
            return res

        def api_navigate_to(floor: str, x_m: float, y_m: float, yaw_deg: float = 0.0) -> dict[str, Any]:
            nonlocal stage_seq, completed_stages
            lineno, line_text = self._active_policy_lineno(source_lines)
            pose = self.localization.get_pose(self.robot_id)
            route = self.topology.plan_route(
                pose.floor,
                pose.x_m,
                pose.y_m,
                str(floor),
                float(x_m),
                float(y_m),
                target_yaw_deg=float(yaw_deg),
            )
            if not route.ok:
                res = {
                    'ok': False,
                    'status': 'route_planning_failed',
                    'pose': pose.to_dict(),
                    'route': route.to_dict(),
                    'legs_completed': 0,
                    'error': route.error,
                }
                append_event(
                    'navigate_to',
                    {'floor': floor, 'x_m': x_m, 'y_m': y_m, 'yaw_deg': yaw_deg},
                    res,
                    lineno,
                    line_text,
                )
                return res

            stage_seq += 1
            stage_dir = stages_dir / f'{stage_seq:04d}_L{lineno:03d}_navigate_to'
            exec_res = self.navigation_executor.execute_route(self.robot_id, route, stage_dir)
            res = {**exec_res, 'route': route.to_dict(), 'stage_dir': str(stage_dir)}
            if res.get('ok') is True:
                completed_stages += 1
            remember_stage('navigate_to', f'{floor} ({x_m}, {y_m})', {**res, 'summary': res.get('error') or ''})
            append_event(
                'navigate_to',
                {'floor': floor, 'x_m': x_m, 'y_m': y_m, 'yaw_deg': yaw_deg},
                res,
                lineno,
                line_text,
            )
            if res.get('status') == 'uncertain_execution':
                raise _UncertainExecutionHalt('navigate_to', str(res.get('error') or 'Uncertain navigation execution'))
            if res.get('status') == 'interrupted':
                raise _PolicyCancelledHalt(str(res.get('error') or 'Navigation interrupted'))
            return res

        def api_navigate_to_place(place: str, floor: str | None = None) -> dict[str, Any]:
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            node = self.topology.find_place(place, floor=floor)
            if node is None:
                res = {
                    'ok': False,
                    'status': 'place_not_found',
                    'error': f'Named place {place!r} not found in topological map',
                }
                append_event('navigate_to_place', {'place': place, 'floor': floor}, res, lineno, line_text)
                return res
            return api_navigate_to(node.floor, node.x_m, node.y_m, node.yaw_deg)

        def api_inspect_scene(
            question: str,
            max_actions: int = 2,
            max_steps: int = 6,
            max_seconds: float = 45.0,
        ) -> dict[str, Any]:
            nonlocal stage_seq, completed_stages
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            stage_seq += 1
            stage_dir = stages_dir / f'{stage_seq:04d}_L{lineno:03d}_inspect_scene'
            limits = EpisodeLimits(max_actions=max_actions, max_steps=max_steps, max_seconds=max_seconds)
            highlighted = highlight_policy_source(source, lineno)
            res = self.action_executor.run_subgoal(
                robot_id=self.robot_id,
                kind='inspect_scene',
                subgoal=question,
                highlighted_policy=highlighted,
                limits=limits,
                stage_dir=stage_dir,
                history=list(stage_history),
            )
            if res.get('ok') is True:
                completed_stages += 1
            remember_stage('inspect_scene', question, res)
            append_event(
                'inspect_scene',
                {'question': question, 'max_actions': max_actions, 'max_steps': max_steps, 'max_seconds': max_seconds},
                res,
                lineno,
                line_text,
            )
            if res.get('status') == 'uncertain_execution':
                raise _UncertainExecutionHalt('inspect_scene', str(res.get('summary') or 'Uncertain inspection execution'))
            if res.get('status') == 'interrupted':
                raise _PolicyCancelledHalt(str(res.get('summary') or 'Inspection interrupted'))
            return res

        def api_act(
            subgoal: str,
            max_actions: int = 12,
            max_steps: int = 24,
            max_seconds: float = 120.0,
        ) -> dict[str, Any]:
            nonlocal stage_seq, completed_stages, navigation_reached
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            stage_seq += 1
            stage_dir = stages_dir / f'{stage_seq:04d}_L{lineno:03d}_act'
            limits = EpisodeLimits(max_actions=max_actions, max_steps=max_steps, max_seconds=max_seconds)
            highlighted = highlight_policy_source(source, lineno)
            res = self.action_executor.run_subgoal(
                robot_id=self.robot_id,
                kind='act',
                subgoal=subgoal,
                highlighted_policy=highlighted,
                limits=limits,
                stage_dir=stage_dir,
                history=list(stage_history),
            )
            if res.get('ok') is True:
                completed_stages += 1
            remember_stage('act', subgoal, res)
            if bool(res.get('arm_engaged')):
                navigation_reached = True
            append_event(
                'act',
                {'subgoal': subgoal, 'max_actions': max_actions, 'max_steps': max_steps, 'max_seconds': max_seconds},
                res,
                lineno,
                line_text,
            )
            if res.get('status') == 'uncertain_execution':
                raise _UncertainExecutionHalt('act', str(res.get('summary') or 'Uncertain manipulation execution'))
            if res.get('status') == 'interrupted':
                raise _PolicyCancelledHalt(str(res.get('summary') or 'Manipulation interrupted'))
            return res

        def reject_arguments(tool: str, args: dict[str, Any], error: str) -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            res = {'ok': False, 'status': 'invalid_arguments', 'error': error}
            append_event(tool, args, res, lineno, line_text)
            return res

        def run_skill_stage(tool: str, args: dict[str, Any], request: str) -> dict[str, Any]:
            nonlocal stage_seq, completed_stages, navigation_reached
            check_cancelled()
            lineno, line_text = self._active_policy_lineno(source_lines)
            if self.skill_executor is None:
                res = {
                    'ok': False,
                    'status': 'skills_unavailable',
                    'error': 'Host skills are not configured (live: --kinematics-config and --skills-config; '
                             'locate also needs --camera-config). Use act() instead.',
                }
                append_event(tool, args, res, lineno, line_text)
                return res
            stage_seq += 1
            stage_dir = stages_dir / f'{stage_seq:04d}_L{lineno:03d}_{tool}'
            res = self.skill_executor.run_skill(robot_id=self.robot_id, skill=tool, args=dict(args), stage_dir=stage_dir)
            if res.get('ok') is True:
                completed_stages += 1
            if bool(res.get('arm_engaged')):
                navigation_reached = True
            remember_stage(tool, request, res)
            append_event(tool, args, res, lineno, line_text)
            if res.get('status') == 'uncertain_execution':
                raise _UncertainExecutionHalt(tool, str(res.get('error') or f'Uncertain {tool} execution'))
            if res.get('status') == 'interrupted':
                raise _PolicyCancelledHalt(str(res.get('error') or f'{tool} interrupted'))
            return res

        def api_locate(label: str) -> dict[str, Any]:
            if not isinstance(label, str) or not label.strip() or len(label.strip()) > 80:
                return reject_arguments('locate', {'label': label}, 'locate(label) needs a non-empty label (<= 80 chars)')
            return run_skill_stage('locate', {'label': label.strip()}, label.strip())

        def api_pick(xyz_m: Any) -> dict[str, Any]:
            xyz = _xyz_argument(xyz_m)
            if xyz is None:
                return reject_arguments('pick', {'xyz_m': xyz_m}, 'pick(xyz_m) needs [x, y, z] metres in arm_base within 1 m')
            return run_skill_stage('pick', {'xyz_m': xyz}, str(xyz))

        def api_place(xyz_m: Any) -> dict[str, Any]:
            xyz = _xyz_argument(xyz_m)
            if xyz is None:
                return reject_arguments('place', {'xyz_m': xyz_m}, 'place(xyz_m) needs [x, y, z] metres in arm_base within 1 m')
            return run_skill_stage('place', {'xyz_m': xyz}, str(xyz))

        def api_home_arm() -> dict[str, Any]:
            return run_skill_stage('home_arm', {}, 'home')

        def api_drive(forward_m: float = 0.0, left_m: float = 0.0, turn_deg: float = 0.0) -> dict[str, Any]:
            args = {'forward_m': forward_m, 'left_m': left_m, 'turn_deg': turn_deg}
            values = [_finite_number(value) for value in (forward_m, left_m, turn_deg)]
            if any(value is None for value in values):
                return reject_arguments('drive', args, 'drive() arguments must be finite numbers')
            forward, left, turn = values
            if abs(forward) > MAX_DRIVE_M or abs(left) > MAX_DRIVE_M or abs(turn) > MAX_TURN_DEG:
                return reject_arguments(
                    'drive', args, f'drive() is limited to |forward_m|,|left_m| <= {MAX_DRIVE_M} and |turn_deg| <= {MAX_TURN_DEG:g}'
                )
            if max(abs(forward), abs(left)) < 0.005 and abs(turn) < 0.5:
                return reject_arguments('drive', args, 'drive() needs a displacement of at least 5 mm or 0.5 deg')
            args = {'forward_m': forward, 'left_m': left, 'turn_deg': turn}
            return run_skill_stage('drive', args, f'forward {forward:+.2f} m, left {left:+.2f} m, turn {turn:+.0f} deg')

        def api_finish(summary: str, **details: Any) -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            res = {'ok': True, 'status': 'completed', 'summary': str(summary), 'details': details}
            append_event('finish', {'summary': summary, **details}, res, lineno, line_text)
            raise _PolicyTerminalHalt('completed', str(summary), task_success=True, details=details)

        def api_block(reason: str, **details: Any) -> dict[str, Any]:
            lineno, line_text = self._active_policy_lineno(source_lines)
            res = {'ok': False, 'status': 'blocked', 'reason': str(reason), 'details': details}
            append_event('block', {'reason': reason, **details}, res, lineno, line_text)
            raise _PolicyTerminalHalt('blocked', str(reason), task_success=False, details=details)

        try:
            compiled = validate_and_compile_policy(source, filename=POLICY_FILENAME)
        except PolicyValidationError as exc:
            outcome = PolicyRunOutcome(
                program_ok=False,
                task_success=False,
                navigation_reached=False,
                termination_reason='program_validation_error',
                summary=str(exc),
                stages_executed=0,
                calls_executed=0,
                loop_ticks=0,
                elapsed_s=round(time.monotonic() - started, 3),
                output_dir=str(out),
            )
            atomic_json(source_dir / 'policy-result.json', {
                'task_language': self.task_language,
                'robot_id': self.robot_id,
                'result': None,
            })
            atomic_json(out / 'outcome.json', outcome.to_dict())
            return outcome

        sandbox_builtins = {**SAFE_BUILTINS, 'print': policy_print, '__import__': _safe_import}
        env: dict[str, Any] = {
            '__builtins__': sandbox_builtins,
            '__tick__': tick_guard,
            'get_task_context': api_get_task_context,
            'get_pose': api_get_pose,
            'relocalize': api_relocalize,
            'search_memory': api_search_memory,
            'save_memory': api_save_memory,
            'mark_absent': api_mark_absent,
            'navigate_to': api_navigate_to,
            'navigate_to_place': api_navigate_to_place,
            'inspect_scene': api_inspect_scene,
            'act': api_act,
            'locate': api_locate,
            'pick': api_pick,
            'place': api_place,
            'home_arm': api_home_arm,
            'drive': api_drive,
            'finish': api_finish,
            'block': api_block,
        }

        program_ok = True
        task_success = False
        termination_reason = 'end_of_policy'
        summary = 'Policy reached end of file without calling finish()'

        try:
            exec(compiled, env, env)
        except _PolicyTerminalHalt as halt:
            program_ok = True
            if halt.task_success:
                if completed_stages == 0:
                    task_success = False
                    termination_reason = 'unverified_finish'
                    summary = f'finish() called with zero completed robot stages: {halt.summary}'
                elif self.goal_checker is not None and not bool(self.goal_checker(self.task_language)):
                    task_success = False
                    termination_reason = 'goal_not_satisfied'
                    summary = f'finish() called but physical goal predicate failed: {halt.summary}'
                else:
                    task_success = True
                    termination_reason = halt.reason
                    summary = halt.summary
            else:
                task_success = False
                termination_reason = halt.reason
                summary = halt.summary
        except _UncertainExecutionHalt as halt:
            program_ok = False
            task_success = False
            termination_reason = 'uncertain_execution'
            summary = f'{halt.tool}: {halt.detail}'
        except _PolicyCancelledHalt as halt:
            program_ok = False
            task_success = False
            termination_reason = 'interrupted'
            summary = halt.detail
        except PolicyLoopLimitExceeded as exc:
            program_ok = False
            task_success = False
            termination_reason = 'loop_limit_exceeded'
            summary = str(exc)
        except Exception as exc:
            program_ok = False
            task_success = False
            termination_reason = 'program_error'
            summary = f'{type(exc).__name__}: {exc}'

        raw_result = env.get('result', env.get('report'))
        sanitized_result = sanitize_policy_result(raw_result)
        atomic_json(
            source_dir / 'policy-result.json',
            {
                'task_language': self.task_language,
                'robot_id': self.robot_id,
                'result': sanitized_result,
            },
        )

        outcome = PolicyRunOutcome(
            program_ok=program_ok,
            task_success=task_success,
            navigation_reached=navigation_reached,
            termination_reason=termination_reason,
            summary=summary,
            stages_executed=stage_seq,
            calls_executed=call_seq,
            loop_ticks=loop_ticks,
            elapsed_s=round(time.monotonic() - started, 3),
            output_dir=str(out),
        )
        atomic_json(out / 'outcome.json', outcome.to_dict())
        return outcome
