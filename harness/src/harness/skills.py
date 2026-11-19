"""Big arm skills built from guarded single commands: move_to, pick, place, home, gripper.

Every primitive goes through ``EpisodeRunner.execute`` with the latest measured
observation, so runtime freshness, IK, speed, calibration, warning and
reconciliation checks are unchanged. A skill stops at the first primitive that is
not ``completed``/``warning`` and raises :class:`SkillFailed` with its log.

One XYZ target is one IK command (up to 20 s, duration from the joint distance).
Only while runtime recovery constraints are active is a move split into the
constrained joint-delta/duration steps.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .episode import Decision

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
ARM_SPEED_DEG_S = 30.
MAX_CONSTRAINED_STEPS = 12
MAX_ROLL_CHANGE_DEG = 25.  # a bigger wrist spin can wrap the wrist camera cable
MAX_JOINT_RECOVERY_DEG = 20.  # joint-space home recovery only for small moves


class SkillFailed(Exception):
    def __init__(self, message, log=None, detail=None):
        super().__init__(message)
        self.log, self.detail = list(log or []), detail or {}


def _round(values, digits=4):
    return [round(float(value), digits) for value in values] if values is not None else None


class Skills:
    def __init__(self, runner, config, *, reason):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text(encoding='utf-8'))
        if config.get('version') != 1:
            raise ValueError('Skills config version 1 is required')
        self.runner, self.config, self.reason = runner, config, reason
        home, grasp, gripper, motion = config['home'], config['grasp'], config['gripper'], config['motion']
        self.home_xyz, self.home_axis = list(home['xyz_m']), list(home['approach_axis'])
        self.grasp_axis0 = np.array(grasp['approach_axis'], dtype=float)
        reference = grasp['reference_xyz_m']
        self.grasp_azimuth0 = math.atan2(reference[1], reference[0])
        self.hover_m, self.floor_grasp_z = float(grasp['hover_m']), float(grasp['floor_grasp_z_m'])
        self.open_percent, self.close_percent = float(gripper['open_percent']), float(gripper['close_percent'])
        self.gripper_ms, self.held_margin = int(gripper['duration_ms']), float(gripper['held_margin_percent'])
        self.speed_margin = float(motion['speed_margin'])
        self.min_ms, self.max_ms = int(motion['min_duration_ms']), int(motion['max_duration_ms'])
        self.calibration_id = config.get('calibration_id')
        # In-motion monitor (runtime capability arm_motion_monitor): while the gripper reads as holding,
        # every arm move asks the runtime to stop and hold if it closes past the held threshold.
        self.monitor_enabled = bool((config.get('monitor') or {}).get('enabled', True))
        self.log = []

    # -- state ---------------------------------------------------------------
    @property
    def packet(self):
        return self.runner.packet

    def joints(self):
        return self.packet['observation']['ego']['joints']

    def current_xyz(self):
        return self.runner.cartesian.evaluate(self.packet).get('current_xyz_m')

    def gripper_percent(self):
        return self.joints().get('gripper')

    def check_calibration(self):
        observed = self.packet['observation']['ego'].get('calibration_id')
        if self.calibration_id and observed != self.calibration_id:
            raise SkillFailed('Skills config (home pose) belongs to another arm calibration; run calibration_shift')

    def grasp_axis(self, xyz):
        """Recorded floor-grasp approach axis, rotated about z to the target's azimuth."""
        angle = math.atan2(xyz[1], xyz[0]) - self.grasp_azimuth0
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
        return (rotation @ self.grasp_axis0).tolist()

    def _fk_runtime(self, joints):
        cartesian = self.runner.cartesian
        urdf = np.array([joints[name] for name in ARM]) * cartesian.signs + cartesian.offsets
        return cartesian.geometry.forward(dict(zip(ARM, urdf.tolist())))

    def duration_for(self, span_deg):
        milliseconds = math.ceil(span_deg / ARM_SPEED_DEG_S * self.speed_margin * 10) * 100
        return int(min(self.max_ms, max(self.min_ms, milliseconds)))

    async def ensure_fresh(self):
        if not self.runner._fresh():
            await self.runner.observe()

    @property
    def held_threshold(self):
        """Gripper percent above which the jaws are stopped by an object (same rule as holding())."""
        return self.close_percent + self.held_margin

    def monitor_for_move(self):
        """Monitor for the next arm-only move, or None when the runtime cannot run one.

        Not used during constrained recovery steps, so that runtime recovery stays unchanged.
        """
        capabilities = self.runner.packet.get('capabilities') or {}
        if (not self.monitor_enabled or capabilities.get('arm_motion_monitor') != 1
                or self.runner.recovery_constraints):
            return None
        held = self.gripper_state() == 'closed_on_object'  # an open gripper also reads above the threshold
        return {'gripper_min_percent': self.held_threshold if held else None}

    # -- primitives ----------------------------------------------------------
    async def send(self, command, label):
        """One guarded command from the latest measured observation."""
        await self.ensure_fresh()
        runner = self.runner
        if command.get('joints') or command.get('cartesian'):
            # After base motion the runtime keeps only a fresh front frame; arm commands
            # need a fresh wrist frame in the runtime baseline, so observe again first.
            wrist = (runner.packet['observation'].get('cameras') or {}).get('wrist') or {}
            if wrist.get('available') is not True:
                await runner.observe()
        before = runner.actions
        decision = Decision.model_validate({
            'robot_id': runner.robot_id, 'observation_id': runner.packet['observation']['observation_id'],
            'tool': 'execute', 'plan': [label], 'reason': self.reason[:2000], 'expected': label,
            'feedback': 'progress', 'evidence': 'Host skill command computed from the latest measured joints.',
            'command': command})
        await runner.execute(decision)
        result = runner.previous_result or {}
        execution = result.get('execution') or {}
        entry = {'label': label, 'sent': runner.actions > before and execution.get('command_sent') is True,
                 'status': result.get('status'), 'outcome': execution.get('outcome'),
                 'arm_reached': execution.get('arm_reached'),
                 'error_code': result.get('error_code'), 'error': result.get('error') or result.get('reason'),
                 'duration_ms': command.get('duration_ms'),
                 'xyz_after_m': _round(self.current_xyz()), 'gripper_after': self.gripper_percent()}
        telemetry = (execution.get('arm_motion') or {}).get('monitor')
        if command.get('monitor') is not None:
            entry['monitor'] = {'gripper_min_percent': command['monitor'].get('gripper_min_percent'),
                                **{key: telemetry.get(key) for key in ('gripper_min_seen', 'max_following_error_deg')
                                   if isinstance(telemetry, dict)}}
        self.log.append(entry)
        if entry['status'] == 'stopped':
            stop = execution.get('monitor_stop') or {}
            entry['monitor_stop'] = stop
            raise SkillFailed(f"{label}: stopped mid-motion by the in-motion monitor at "
                              f"{stop.get('progress_fraction', 0):.0%} (gripper {stop.get('gripper_percent')}% < "
                              f"{stop.get('gripper_min_percent')}%): the object is probably no longer between the jaws",
                              self.log, {'monitor_stop': stop, 'failed_step': label, 'failed_primitive': entry})
        if not entry['sent'] or entry['status'] not in ('completed', 'warning'):
            raise SkillFailed(f"{label}: {entry['status']} {entry['error_code'] or ''} {entry['error'] or ''}".strip(),
                              self.log, {'failed_primitive': entry, 'failed_step': label,
                                         'recovery_constraints': runner.recovery_constraints})
        return entry

    async def gripper(self, percent, label=None):
        joints = {name: None for name in ARM}
        joints['gripper'] = float(percent)
        return await self.send({'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': self.gripper_ms,
                                'joints': joints, 'cartesian': None}, label or f'gripper {percent:g}%')

    def preview(self, xyz, axis):
        """IK at the longest duration: feasibility and joint distance, no motion."""
        request = {'frame': 'arm_base', 'mode': 'absolute', 'xyz_m': [float(value) for value in xyz],
                   'approach_axis': None if axis is None else [float(value) for value in axis]}
        result = self.runner.cartesian.solve(self.packet, request, self.max_ms)
        joints = self.joints()
        span = (max(abs(result['joints'][name] - joints[name]) for name in ARM)
                if result.get('executable') and result.get('joints') else None)
        return request, result, span

    async def move_to(self, xyz, axis, label):
        for _ in range(MAX_CONSTRAINED_STEPS):
            await self.ensure_fresh()
            request, result, span = self.preview(xyz, axis)
            if span is None:
                raise SkillFailed(f"{label}: IK not executable {result.get('reason_codes')}", self.log,
                                  {'ik_reason_codes': result.get('reason_codes'), 'ik_reasons': result.get('reasons'),
                                   'target_xyz_m': request['xyz_m'], 'swept_path': result.get('swept_path'),
                                   'failed_step': label})
            if span < .5:
                return {'label': label, 'already_there': True}
            roll_change = abs(result['joints']['wrist_roll'] - self.joints()['wrist_roll'])
            if roll_change > MAX_ROLL_CHANGE_DEG:
                raise SkillFailed(f'{label}: IK needs a {roll_change:.0f} deg wrist_roll change (limit '
                                  f'{MAX_ROLL_CHANGE_DEG:g}); not sent', self.log,
                                  {'target_xyz_m': request['xyz_m'], 'roll_change_deg': roll_change,
                                   'failed_step': label})
            constraints = self.runner.recovery_constraints
            if not constraints:
                command = {'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': self.duration_for(span),
                           'joints': None, 'cartesian': request}
                monitor = self.monitor_for_move()
                if monitor is not None:
                    command['monitor'] = monitor
                return await self.send(command, label)
            # Constrained recovery: joint-interpolated waypoint within the runtime limit.
            limit = float(constraints['max_joint_delta_deg']) * .95
            joints = self.joints()
            fraction = min(1., limit / span)
            via = {name: joints[name] + fraction * (result['joints'][name] - joints[name]) for name in ARM}
            fk = self._fk_runtime(via)
            step = {'frame': 'arm_base', 'mode': 'absolute', 'xyz_m': fk['xyz_m'],
                    'approach_axis': np.array(fk['transform'])[:3, 2].tolist()}
            duration = max(int(constraints['min_duration_ms']), self.duration_for(span * fraction))
            await self.send({'action': 'move', 'x': 0., 'y': 0., 'theta': 0., 'duration_ms': duration,
                             'joints': None, 'cartesian': step}, f'{label} (constrained step)')
            if fraction >= 1.:
                return self.log[-1]
        raise SkillFailed(f'{label}: constrained steps exhausted', self.log, {'failed_step': label})

    def reach_report(self, xyz):
        """IK feasibility (no motion) of the pick hover and grasp points for a floor target."""
        reach = {}
        for name, point in (('hover', self._hover(xyz)), ('grasp', list(xyz))):
            _, ik, span = self.preview(point, self.grasp_axis(xyz))
            reach[name] = {'executable': bool(ik.get('executable')), 'reason_codes': ik.get('reason_codes'),
                           'joint_span_deg': None if span is None else round(span, 1)}
        return all(item['executable'] for item in reach.values()), reach

    # -- skills --------------------------------------------------------------
    def holding(self):
        grip = self.gripper_percent()
        return grip is not None and grip > self.close_percent + self.held_margin

    def gripper_state(self):
        """open / closed_on_object (jaws stopped above the close target) / closed_empty / unknown."""
        grip = self.gripper_percent()
        if grip is None:
            return 'unknown'
        if grip >= self.open_percent - 5:
            return 'open'
        return 'closed_on_object' if grip > self.held_threshold else 'closed_empty'

    def require_holding(self, label):
        if not self.holding():
            raise SkillFailed(f'{label}: the object is no longer between the jaws (gripper {self.gripper_percent()}%)',
                              self.log, {'gripper_percent': self.gripper_percent(), 'failed_step': label})

    def _hover(self, xyz):
        return [xyz[0], xyz[1], xyz[2] + self.hover_m]

    async def home(self):
        self.check_calibration()
        await self.ensure_fresh()
        state = self.runner.cartesian.evaluate(self.packet)
        if 'urdf_seed_out_of_range' in state.get('reason_codes', []):
            # The arm sags slightly past the URDF model limit when torque is released.
            # IK cannot start there, so return to the recorded home joints directly; the
            # runtime still checks calibration and speed. Only small moves are allowed.
            await self.joint_home('recover home (pose outside URDF range)')
        else:
            await self.move_to(self.home_xyz, self.home_axis, 'home')
        return {'xyz_m': _round(self.current_xyz()), 'gripper': self.gripper_percent()}

    async def joint_home(self, label):
        home = self.config['home']['joints_deg']
        joints = self.joints()
        changes = {name: abs(home[name] - joints[name]) for name in ARM}
        if max(changes.values()) > MAX_JOINT_RECOVERY_DEG or changes['wrist_roll'] > MAX_ROLL_CHANGE_DEG:
            raise SkillFailed(f'{label}: joint change too large for a blind recovery '
                              f'({ {k: round(v, 1) for k, v in changes.items()} }); move the arm near home by hand',
                              self.log, {'joint_changes_deg': changes})
        return await self.send({'action': 'move', 'x': 0., 'y': 0., 'theta': 0.,
                                'duration_ms': self.duration_for(max(changes.values())),
                                'joints': {**{name: float(home[name]) for name in ARM}, 'gripper': None},
                                'cartesian': None}, label)

    async def pick(self, xyz):
        self.check_calibration()
        axis = self.grasp_axis(xyz)
        if (self.gripper_percent() or 0) < self.open_percent - 5:
            await self.gripper(self.open_percent, 'open gripper before pick')
        await self.move_to(self._hover(xyz), axis, 'pick hover')
        await self.move_to(xyz, axis, 'pick descend')
        grasp_xyz = _round(self.current_xyz())
        await self.gripper(self.close_percent, 'close gripper')
        closed = self.gripper_percent()
        await self.move_to(self._hover(xyz), axis, 'lift')
        self.require_holding('after lift')
        return {'target_xyz_m': _round(xyz), 'grasp_fk_xyz_m': grasp_xyz, 'gripper_after_close': closed,
                'gripper_after_lift': self.gripper_percent(),
                'object_between_jaws_likely': closed is not None and closed > self.close_percent + self.held_margin,
                'note': 'Gripper stopping above its close target suggests an object between the jaws; '
                        'confirm with the wrist image.'}

    async def place(self, xyz):
        self.check_calibration()
        axis = self.grasp_axis(xyz)
        self.require_holding('before place')
        await self.move_to(self._hover(xyz), axis, 'place hover')
        self.require_holding('after moving to place hover')
        await self.move_to(xyz, axis, 'place descend')
        release_xyz = _round(self.current_xyz())
        release_joints = dict(self.joints())
        await self.gripper(self.open_percent, 'open gripper (release)')
        await self.move_to(self._hover(xyz), axis, 'retreat')
        return {'target_xyz_m': _round(xyz), 'release_fk_xyz_m': release_xyz, 'release_joints': release_joints,
                'gripper_after_open': self.gripper_percent()}
