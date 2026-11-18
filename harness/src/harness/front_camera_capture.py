"""Collect front-camera extrinsic calibration data with the real arm (moves the arm).

Preconditions: runtime connected, arm calibration valid, the cube held in the
gripper, the checkerboard flat on the floor in front-camera view. The base never
moves. For each floor target the arm places the cube, returns home (out of the
front view), photographs it, picks it up again at the same commanded target and
returns home. ``capture.json`` is rewritten after every placement.

  python -m harness.front_camera_capture OUT_DIR [--targets "0.20,-0.05;0.24,0.0"]

Solve afterwards with the OpenCV environment:
  lekiwi_runtime/.venv/bin/python harness/util/solve_front_camera.py OUT_DIR ...
"""
from __future__ import annotations

import argparse
import asyncio
import base64
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from .client import ServerClient
from .episode import EpisodeLimits, EpisodeRunner, EpisodeStopped, atomic_json
from .kinematics import CartesianController
from .skills import SkillFailed, Skills
from .tools import RobotTools

DEFAULT_TARGETS = [(.20, -.05), (.20, 0.), (.20, .05), (.24, .05), (.24, 0.), (.24, -.05)]
HOME_TOLERANCE_M = .02


def log(**data):
    print(json.dumps(data, ensure_ascii=False, default=str), flush=True)


def save_images(runner, out, stem):
    saved = {}
    for label, data, _ in runner.images:
        if label in ('front', 'wrist'):
            path = out / f'{stem}-{label}.jpg'
            path.write_bytes(base64.b64decode(data, validate=True))
            saved[label] = path.name
    atomic_json(out / f'{stem}.json', runner.packet)
    return {'images': saved, 'observation_id': runner.packet['observation']['observation_id'],
            'joints': runner.packet['observation']['ego']['joints'],
            'fk_xyz_m': runner.cartesian.evaluate(runner.packet).get('current_xyz_m')}


async def go_home(skills):
    if math.dist(skills.current_xyz(), skills.home_xyz) > HOME_TOLERANCE_M:
        await skills.home()


async def main_async(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    run = Path('runs') / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-front-camera-capture')
    runner = EpisodeRunner(None, RobotTools(ServerClient(args.server)), args.robot_id,
                           'Front camera extrinsic calibration: place the held cube on floor targets and photograph it. '
                           'Base stays still.', run, EpisodeLimits(300, 600, 3600))
    runner.cartesian = CartesianController(args.kinematics)
    skills = Skills(runner, args.skills, reason='Operator-requested front camera calibration capture; base stays still.')
    capture = {'version': 1, 'created_at': datetime.now(timezone.utc).isoformat(), 'run_dir': str(run.resolve()),
               'robot_id': args.robot_id, 'board': None, 'placements': []}
    status = 'error'
    try:
        await runner.observe()
        ego = runner.packet['observation']['ego']
        geometry = runner.geometry
        capture.update(calibration_id=ego.get('calibration_id'), arm_revision=(ego.get('control') or {}).get('revision'),
                       mapping_id=geometry.get('mapping_id'), model_hash=geometry.get('model_hash'),
                       joint_mapping=geometry.get('joint_mapping'))
        if geometry.get('executable') is not True:
            raise RuntimeError(f"Cartesian control not executable: {geometry.get('reason_codes')}")
        skills.check_calibration()
        await go_home(skills)
        previous = out / 'capture.json'
        earlier = json.loads(previous.read_text()) if previous.exists() else {}
        if earlier.get('board') and not args.board_only:
            # The board was photographed earlier from this same base pose and then removed.
            capture['board'] = earlier['board']
            log(event='board_reused', **capture['board'])
        else:
            await runner.observe()
            capture['board'] = save_images(runner, out, 'board')
            log(event='board_captured', **capture['board'])
        atomic_json(out / 'capture.json', capture)
        if args.board_only:
            status = 'board_only'
            return
        if (skills.gripper_percent() or 0) <= skills.close_percent + skills.held_margin:
            raise RuntimeError(f'The cube does not appear held (gripper {skills.gripper_percent()}%).')
        for index, (x, y) in enumerate(args.targets):
            target = [x, y, skills.floor_grasp_z]
            entry = {'index': index, 'target_xyz_m': target}
            await runner.observe()
            entry['reference'] = save_images(runner, out, f'p{index:02d}-ref')
            entry['place'] = await skills.place(target)
            await skills.home()
            await runner.observe()
            entry['cube'] = save_images(runner, out, f'p{index:02d}-cube')
            entry['pick'] = await skills.pick(target)
            await skills.home()
            capture['placements'].append(entry)
            atomic_json(out / 'capture.json', capture)
            log(event='placement', index=index, target=target, release=entry['place']['release_fk_xyz_m'],
                held=entry['pick']['object_between_jaws_likely'], gripper=entry['pick']['gripper_after_close'])
            if not entry['pick']['object_between_jaws_likely']:
                raise RuntimeError('Cube was not regrasped; stopping (it is on the floor at the last target).')
        status = 'complete'
    except SkillFailed as exc:
        status = 'skill_failed'
        log(event='stopped', status=status, error=str(exc), detail=exc.detail, log=exc.log[-3:])
    except EpisodeStopped as exc:
        status = exc.reason
        log(event='stopped', status=status, error=exc.detail)
    except Exception as exc:
        log(event='stopped', status=status, error=f'{type(exc).__name__}: {exc}')
    finally:
        capture['status'] = status
        atomic_json(out / 'capture.json', capture)
        atomic_json(run / 'result.json', {'status': status, 'action_attempts': runner.actions,
                                          'observations': runner.record.sequence, 'skill_log': skills.log,
                                          'physical_success_verified': False})
        log(event='end', status=status, placements=len(capture['placements']), out=str(out))


def parse_targets(text):
    targets = []
    for item in text.split(';'):
        x, y = (float(value) for value in item.split(','))
        targets.append((x, y))
    return targets


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out', type=Path)
    parser.add_argument('--server', default='tcp://127.0.0.1:5556')
    parser.add_argument('--robot-id', default='kiwi-01')
    parser.add_argument('--kinematics', type=Path, default=Path('outputs/kinematics/cartesian-kiwi-01.json'))
    parser.add_argument('--skills', type=Path, default=Path('outputs/kinematics/skills-kiwi-01.json'))
    parser.add_argument('--targets', type=parse_targets, default=DEFAULT_TARGETS, help='"x,y;x,y" in arm_base metres')
    parser.add_argument('--board-only', action='store_true', help='Go home if needed and photograph the board only')
    asyncio.run(main_async(parser.parse_args(argv)))


if __name__ == '__main__':
    main()
