"""Carry a verified XYZ joint mapping across an arm range recalibration (preview by default).

Runtime angles are ``(raw - (range_min + range_max) / 2) * 360 / 4095``. A new
range moves each joint's zero by ``(mid_new - mid_old) * 360 / 4095`` while the
same encoder raw stays at the same physical angle (homing offsets unchanged).
For the URDF to see the same angle, ``q_urdf = sign * q_new + offset_new`` with
``offset_new = offset_old + sign * shift``. Recorded joint poses shift by ``-shift``.

The new range midpoints are derived from a fresh observation (``raw`` and
``joints``), so the new calibration file itself is not needed here.

  python -m harness.calibration_shift OLD_CALIBRATION.json OBSERVATION.json \
      --cartesian outputs/kinematics/cartesian-kiwi-01.json \
      --skills outputs/kinematics/skills-kiwi-01.json [--write]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
TICKS_PER_DEG = 4095 / 360


def shifts(old_calibration, observation):
    ego = observation['observation']['ego'] if 'observation' in observation else observation['ego']
    result = {}
    for name in ARM_JOINTS:
        old = old_calibration[name]
        raw, joint = ego['raw'][name], ego['joints'][name]
        mid_old = (old['range_min'] + old['range_max']) / 2
        mid_new = raw - joint * TICKS_PER_DEG  # Inverse of the runtime conversion (joints rounded to 1e-3 deg).
        result[name] = {'mid_old': mid_old, 'mid_new': round(mid_new * 2) / 2,
                        'shift_deg': (round(mid_new * 2) / 2 - mid_old) / TICKS_PER_DEG}
    return result, ego['calibration_id']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('old_calibration', type=Path)
    parser.add_argument('observation', type=Path, help='Fresh runtime observation after recalibration')
    parser.add_argument('--cartesian', type=Path, required=True)
    parser.add_argument('--skills', type=Path)
    parser.add_argument('--write', action='store_true', help='Write the shifted files (default: preview only)')
    args = parser.parse_args(argv)
    old = json.loads(args.old_calibration.read_text())
    observation = json.loads(args.observation.read_text())
    table, new_id = shifts(old, observation)
    cartesian = json.loads(args.cartesian.read_text())
    mapping = cartesian['joint_mapping']
    new_mapping = {name: {'sign': mapping[name]['sign'],
                          'offset_deg': round(mapping[name]['offset_deg'] + mapping[name]['sign'] * table[name]['shift_deg'], 4)}
                   for name in ARM_JOINTS}
    print(json.dumps({'new_calibration_id': new_id, 'old_calibration_id': cartesian.get('calibration_id'),
                      'joints': {name: {**table[name], 'offset_deg': new_mapping[name]['offset_deg']}
                                 for name in ARM_JOINTS}}, indent=2))
    if not args.write:
        print('Preview only; pass --write to update the files.')
        return 0
    cartesian.update(calibration_id=new_id, joint_mapping=new_mapping)
    args.cartesian.write_text(json.dumps(cartesian, indent=2) + '\n')
    if args.skills:
        skills = json.loads(args.skills.read_text())
        skills['home']['joints_deg'] = {name: round(value - table[name]['shift_deg'], 3)
                                        for name, value in skills['home']['joints_deg'].items()}
        skills['calibration_id'] = new_id
        args.skills.write_text(json.dumps(skills, indent=2) + '\n')
    print('Updated:', args.cartesian, args.skills or '')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
