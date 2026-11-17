"""Disk/model-only GUI previews. This module has no robot or planner client."""
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .episode import CartesianTarget
from .kinematics import ARM_JOINTS, CartesianController, DEFAULT_URDF, RobotGeometry


MODEL_SEED = dict(zip(ARM_JOINTS, (0., -15., 35., -20., 0.)))


class PreviewInput(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    source: Literal['model', 'recorded']
    run_id: str | None = Field(default=None, min_length=1, max_length=128)
    observation_id: str | None = Field(default=None, min_length=1, max_length=256)
    joints_deg: dict[str, float] | None = None
    target: CartesianTarget | None = None
    duration_ms: int = Field(default=3000, ge=100, le=20000)

    @model_validator(mode='after')
    def source_fields(self):
        if self.source == 'model':
            if self.run_id is not None or self.observation_id is not None:
                raise ValueError('Model previews do not accept observation IDs')
            if self.joints_deg is not None and set(self.joints_deg) != set(ARM_JOINTS):
                raise ValueError('Specify all five URDF joint angles')
        elif self.run_id is None or self.joints_deg is not None:
            raise ValueError('Recorded previews require run_id and cannot override measured joints')
        return self


def calculate_preview(request, *, observed=None, config=None):
    """Return FK, optional bounded IK, and schematic points, always non-executable."""
    source = {'kind': request.source, 'run_id': request.run_id}
    if request.source == 'model':
        geometry = RobotGeometry(DEFAULT_URDF)
        joints = request.joints_deg if request.joints_deg is not None else MODEL_SEED
        fk = geometry.forward(joints)
        state = {'available': True, 'executable': False, 'fk': fk, 'current_xyz_m': fk['xyz_m'],
                 'reasons': ['URDF 예시 자세입니다. 실물 관절 상태·보정·속도 제한을 나타내지 않습니다.']}
        result = None
        if request.target is not None:
            target = request.target.xyz_m
            if request.target.mode == 'relative':
                target = [value + delta for value, delta in zip(fk['xyz_m'], target)]
            try:
                result = geometry.solve_position(target, joints, approach_axis=request.target.approach_axis)
                result['solution_urdf_joints_deg'] = result['solution_joints_deg']
                result['reasons'] = ([] if result['success'] else
                                     ['URDF 범위 안에서 목표 위치에 도달하는 해를 찾지 못했습니다.'])
            except ValueError as exc:
                result = {'success': False, 'reasons': [str(exc)]}
            result.update(target_xyz_m=target, request=request.target.model_dump())
    else:
        if observed is None:
            raise ValueError('Saved arm observation is required')
        packet = observed['packet']
        original_source = packet.get('source')
        packet = {**packet, 'source': {**(original_source if isinstance(original_source, dict) else {}),
                                      'recorded': True}}
        controller = CartesianController(config)
        geometry = controller.geometry
        state = controller.evaluate(packet)
        source.update(observation_id=observed['observation_id'], at=observed['at'],
                      observed_at=packet.get('observation', {}).get('timestamp'))
        result = (controller.solve(packet, request.target.model_dump(), request.duration_ms)
                  if request.target is not None else None)
        if (result is not None and not result['success'] and request.duration_ms < 20000
                and 'ik_no_solution' in result.get('reason_codes', [])):
            # Diagnose a tighter time/speed bound without replacing the requested
            # result, changing motor limits, or creating an executable command.
            for checked_duration in (1000, 2000, 3000, 5000, 10000, 20000):
                if checked_duration <= request.duration_ms:
                    continue
                slower = controller.solve(packet, request.target.model_dump(), checked_duration)
                if slower['success']:
                    break
            result['duration_check'] = {
                'requested_duration_ms': request.duration_ms, 'checked_duration_ms': checked_duration,
                'success': slower['success'], 'residual_m': slower.get('residual_m'),
                'preview_only': True, 'executable': False,
            }
            if slower['success']:
                result['reasons'].append(
                    f'{request.duration_ms}ms의 시간·속도 제한에서는 해를 찾지 못했지만 '
                    f'{checked_duration:,}ms 미리보기에서는 해를 찾았습니다. '
                    f'이동 시간을 {checked_duration:,}ms로 바꿔 다시 계산하세요.')
        fk = state.get('fk')
    points = geometry.joint_points(fk['joints_deg']) if fk else []
    solution_points = []
    if result is not None:
        # Even a verified config and a fresh-looking disk packet cannot authorize a command.
        result.update(executable=False, preview_only=True, joints=None, mode='preview')
        if result.get('success') and result.get('solution_urdf_joints_deg'):
            solution_points = geometry.joint_points(result['solution_urdf_joints_deg'])
    return {'preview_only': True, 'executable': False, 'source': source,
            'state': state, 'result': result, 'current_points': points, 'solution_points': solution_points}
