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
from .semantic_memory import SemanticMemoryStore
from .topology import LocalizationProvider, PoseEstimate, RouteLeg, RoutePlan, TopologicalMap


POLICY_FILENAME = '<policy>'
DEFAULT_MAX_LOOP_TICKS = 500
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
    ) -> dict[str, Any]: ...


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
            if status != 'completed':
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
    """Delegates `inspect_scene()` and `act()` to the host-owned `EpisodeRunner`."""

    def __init__(
        self,
        *,
        server: str,
        model: str,
        kinematics_config: str | Path | None = None,
    ):
        self._server = server
        self._model = model
        self._kinematics_config = kinematics_config

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
        from ..episode_runtime import run_episode

        composed_instruction = (
            f'Active stage ({kind}): {subgoal}\n\n'
            f'Policy context (`>>>` marks active call):\n{highlighted_policy}'
        )
        ep_res = asyncio.run(
            run_episode(
                self._server,
                self._model,
                robot_id,
                composed_instruction,
                run_dir=stage_dir,
                limits=limits,
                kinematics_config=self._kinematics_config,
            )
        )
        status = str(ep_res.get('status', 'host_error'))
        engagement = _inspect_stage_engagement(stage_dir)
        claim = ep_res.get('completion_claim') or {}
        summary = str(claim.get('evidence') or claim.get('reason') or ep_res.get('detail') or status)
        return {
            'ok': status == 'completed',
            'status': status,
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
    ) -> dict[str, Any]:
        del robot_id, limits
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
            hit = self.semantic_memory.upsert(
                text,
                floor=use_floor,
                x_m=use_x,
                y_m=use_y,
                yaw_deg=use_yaw,
                room=room,
                tags=tags,
                robot_id=self.robot_id,
            )
            res = {'ok': True, 'record': hit.to_dict()}
            append_event(
                'save_memory',
                {'text': text, 'floor': use_floor, 'x_m': use_x, 'y_m': use_y, 'yaw_deg': use_yaw, 'room': room},
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
            )
            if res.get('ok') is True:
                completed_stages += 1
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
            )
            if res.get('ok') is True:
                completed_stages += 1
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
