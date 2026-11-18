"""One host skill (locate / pick / place / home_arm / drive) as a guarded robot stage.

policy.py calls these as deterministic primitives. Each call is one short session:
startup (kinematics, skills, camera) -> MCP transport -> fresh observation -> the skill ->
``result.json`` in the stage directory. Every motor command goes through
``EpisodeRunner.execute`` with the latest measured observation, so runtime freshness, IK,
speed, calibration, warning and reconciliation checks are the same as in the model loops.
No planner is involved; only ``locate`` asks the isolated vision locator for a pixel box,
and the calibrated front camera turns that box into an arm_base grasp point.
"""
from __future__ import annotations

import asyncio
from contextlib import AsyncExitStack
from functools import partial
import json
import math
from pathlib import Path
import sys

from .episode import EpisodeLimits, EpisodeRunner, EpisodeStopped, MCPRobotTools, atomic_json
from .skills import SkillFailed, Skills

HOST_SKILLS = ('locate', 'pick', 'place', 'home_arm', 'drive')
DRIVE_SPEED_M_S = 0.10
TURN_SPEED_DEG_S = 30.0
DRIVE_CHUNK_MS = 1000  # runtime limit for one base command
MAX_DRIVE_M = 0.5  # per drive() call; shorter moves keep the commanded-vs-actual error small
MAX_TURN_DEG = 90.0
# pick = open + hover + descend + close + lift (plus constrained steps while recovering);
# drive 0.5 m at 0.1 m/s = 5 commands, a 90 deg turn at 30 deg/s = 3 commands.
SKILL_LIMITS = EpisodeLimits(max_actions=24, max_steps=1, max_seconds=240)


def drive_chunks(forward_m, left_m, turn_deg):
    """Base velocity commands of at most 1000 ms: translation first, then rotation in place."""
    chunks = []
    distance = math.hypot(forward_m, left_m)
    if distance >= 0.005:
        total_ms = distance / DRIVE_SPEED_M_S * 1000
        count = max(1, math.ceil(total_ms / DRIVE_CHUNK_MS - 1e-9))
        duration = max(1, round(total_ms / count))
        speed = distance / (count * duration / 1000)  # exact distance despite ms rounding
        chunk = {'x': round(forward_m / distance * speed, 4), 'y': round(left_m / distance * speed, 4),
                 'theta': 0.0, 'duration_ms': duration}
        chunks += [dict(chunk) for _ in range(count)]
    if abs(turn_deg) >= 0.5:
        total_ms = abs(turn_deg) / TURN_SPEED_DEG_S * 1000
        count = max(1, math.ceil(total_ms / DRIVE_CHUNK_MS - 1e-9))
        duration = max(1, round(total_ms / count))
        chunk = {'x': 0.0, 'y': 0.0, 'theta': round(turn_deg / (count * duration / 1000), 3), 'duration_ms': duration}
        chunks += [dict(chunk) for _ in range(count)]
    return chunks


async def locate_object(runner, skills, camera, locate_fn, label):
    if camera is None:
        return {'status': 'unavailable', 'found': False,
                'error': 'Front camera is not calibrated; start with --camera-config to use locate()'}
    front = next(((data, mime) for name, data, mime in runner.images if name == 'front'), None)
    if front is None:
        return {'status': 'not_found', 'found': False, 'label': label, 'error': 'No fresh front camera image'}
    answer = await locate_fn(label, front[0], front[1], (camera.width, camera.height))
    runner.record.event('locate_answer', {'label': label, **answer.model_dump()})
    base = {'label': label, 'confidence': answer.confidence, 'note': answer.note,
            'observation_id': runner.packet['observation']['observation_id']}
    if not answer.found:
        return {'status': 'not_found', 'found': False, **base}
    try:
        found = camera.locate_box(answer.bbox_px)
    except ValueError as exc:
        return {'status': 'not_found', 'found': False, 'bbox_px': answer.bbox_px, 'error': str(exc), **base}
    if not found['ok']:
        return {'status': 'not_found', 'found': False, **found, **base}
    found.pop('ok')
    reachable, reach = skills.reach_report(found['xyz_m'])
    return {'status': 'completed', 'found': True, **found, 'reachable': reachable, 'reach': reach, **base}


async def perform_skill(runner, skills, camera, locate_fn, skill, args):
    """Run one skill from the current observation; returns its payload with a ``status``."""
    skills.log = []
    if skill == 'locate':
        return await locate_object(runner, skills, camera, locate_fn, args['label'])
    if skill == 'pick':
        payload = await skills.pick(args['xyz_m'])
    elif skill == 'place':
        payload = await skills.place(args['xyz_m'])
    elif skill == 'home_arm':
        payload = await skills.home()
    elif skill == 'drive':
        forward, left, turn = (float(args.get(name, 0.0)) for name in ('forward_m', 'left_m', 'turn_deg'))
        if not all(math.isfinite(value) for value in (forward, left, turn)) or max(abs(forward), abs(left)) > MAX_DRIVE_M \
                or abs(turn) > MAX_TURN_DEG:
            raise ValueError(f'drive is limited to {MAX_DRIVE_M} m and {MAX_TURN_DEG:g} deg per call')
        chunks = drive_chunks(forward, left, turn)
        for index, chunk in enumerate(chunks, start=1):
            await skills.send({'action': 'move', **chunk, 'joints': None, 'cartesian': None},
                              f'drive {index}/{len(chunks)}')
        payload = {'commanded': args, 'base_commands': len(chunks),
                   'note': 'Commanded speed*time, not measured travel; locate again before picking.'}
    else:
        raise ValueError(f'Unknown host skill {skill!r}')
    return {'status': 'completed', **payload}


async def run_skill_session(server, model, robot_id, skill, args, *, run_dir, kinematics_config, skills_config,
                            camera_config=None, locate_fn=None, limits=None, tools=None):
    """Same artifact layout as an episode stage; ``tools`` replaces the MCP transport in tests."""
    from .kinematics import CartesianController
    from .perception import CameraModel

    if skill not in HOST_SKILLS:
        raise ValueError(f'Unknown host skill {skill!r}')
    path = Path(run_dir)
    runner = EpisodeRunner(None, None, robot_id, f'Host skill {skill} {json.dumps(args, ensure_ascii=False)}',
                           path, limits or SKILL_LIMITS)
    skills, payload = None, {}
    reason, detail = 'host_error', ''
    try:
        async with runner.phase('startup', component='kinematics'):
            if kinematics_config is None or skills_config is None:
                raise ValueError('Host skills need --kinematics-config and --skills-config')
            runner.cartesian = CartesianController(kinematics_config)
            skills = Skills(runner, skills_config, reason=f'Host skill {skill} called from policy.py')
            camera = CameraModel(camera_config) if camera_config else None
        if locate_fn is None:
            from .locator import codex_locate
            locate_fn = partial(codex_locate, model)
        async with AsyncExitStack() as transport:
            if tools is None:
                from mcp import ClientSession, StdioServerParameters
                from mcp.client.stdio import stdio_client
                params = StdioServerParameters(command=sys.executable, args=['-m', 'harness', '--server', server])
                async with runner.phase('startup', component='transport'):
                    read, write = await transport.enter_async_context(stdio_client(params))
                    session = await transport.enter_async_context(ClientSession(read, write))
                    await runner.bounded(session.initialize)
                tools = MCPRobotTools(session)
            runner.tools = tools
            runner.record.event('skill_started', {'skill': skill, 'args': args})
            await runner.observe()
            payload = await perform_skill(runner, skills, camera, locate_fn, skill, args)
            reason = payload.pop('status')
            detail = payload.get('error') or payload.get('note') or ''
    except SkillFailed as exc:
        reason, detail = 'skill_failed', str(exc)
        payload = {'error': str(exc), 'failure': exc.detail}
    except EpisodeStopped as exc:
        reason, detail = exc.reason, exc.detail
    except TimeoutError:
        reason = 'uncertain_execution' if runner.in_flight else 'budget_seconds'
        detail = 'Deadline expired; no action is retried'
    except asyncio.CancelledError:
        reason = 'uncertain_execution' if runner.in_flight else 'interrupted'
        detail = 'Skill interrupted; no automatic motor resume'
        raise
    except Exception as exc:
        reason = 'uncertain_execution' if runner.in_flight else 'host_error'
        detail = f'{type(exc).__name__}: {exc}'
    finally:
        result = runner.finish(reason, detail)
        joints = ((runner.packet or {}).get('observation') or {}).get('ego', {}).get('joints') or {}
        result.update(skill=skill, args=args, skill_result=payload,
                      arm_xyz_m=runner.geometry.get('current_xyz_m'), gripper_percent=joints.get('gripper'),
                      primitives=list(skills.log) if skills is not None else [])
        atomic_json(path / 'result.json', result)
        runner.record.event('skill_finished', {'skill': skill, 'status': reason, 'detail': detail})
    return result
