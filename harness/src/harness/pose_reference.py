"""Compare a measured arm snapshot with model poses without any robot I/O.

A reference records a model-space tool point and its measured joint seed. It is
not a measurement of the cube centre, a joint-zero calibration, or a trajectory.
"""
from copy import deepcopy
import math

from .gui_preview import MODEL_SEED
from .kinematics import ARM_JOINTS, CartesianController


_INVALID_REFERENCE_REASONS = {
    'observation_unavailable', 'calibration_invalid', 'calibration_id_missing',
    'calibration_id_mismatch', 'model_hash_mismatch',
    'runtime_seed_out_of_range', 'urdf_seed_out_of_range',
}


def _json_safe(value):
    """Preserve small provenance fields without leaking non-JSON numeric values."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    return None


def _pose(fk, label, *, runtime_joints=None):
    result = {
        'label': label, 'urdf_joints_deg': dict(fk['joints_deg']),
        'xyz_m': list(fk['xyz_m']),
        'approach_axis': [row[2] for row in fk['transform'][:3]],
        'urdf_limit_violations': deepcopy(fk['urdf_limit_violations']),
    }
    if runtime_joints is not None:
        result['runtime_joints_deg'] = dict(runtime_joints)
    return result


def build_pose_reference(packet, config=None):
    """Return a JSON-safe, non-executable correspondence for one observation.

    The default pose is the GUI's URDF example, not a robot home command. A third
    all-zero URDF pose makes the other common meaning of "default" explicit.
    Valid snapshots are checked with the existing endpoint IK solver using the
    full calibrated joint ranges, without a duration/speed constraint. Each
    calculation remains bounded by that solver's limits. Invalid measured seeds
    are displayed, when possible, but never clipped or passed into IK.
    """
    controller = CartesianController(config)
    geometry = controller.geometry
    state = controller.evaluate(packet)
    packet_dict = packet if isinstance(packet, dict) else {}
    observation = packet_dict.get('observation')
    observation = observation if isinstance(observation, dict) else {}
    ego = observation.get('ego')
    ego = ego if isinstance(ego, dict) else {}
    action_context = packet_dict.get('action_context')
    action_context = action_context if isinstance(action_context, dict) else {}
    observed_joints = ego.get('joints')
    observed_joints = observed_joints if isinstance(observed_joints, dict) else {}
    reason_codes = list(state['reason_codes'])
    reasons = list(state['reasons'])
    if packet_dict.get('status', 'ok') != 'ok':
        reason_codes.append('observation_packet_error')
        reasons.append('오류 응답에 포함된 관절값은 유효한 파지 기준으로 등록하지 않습니다.')
    if ego.get('error'):
        reason_codes.append('feedback_error')
        reasons.append('현재 관절 피드백에 오류가 있어 IK 기준으로 사용하지 않습니다.')
    reference_valid = (state['available'] and not (
        _INVALID_REFERENCE_REASONS | {'observation_packet_error', 'feedback_error'}
    ).intersection(reason_codes))
    default_fk = geometry.forward(MODEL_SEED)
    zero_fk = geometry.forward(dict.fromkeys(ARM_JOINTS, 0.))
    current_fk = state.get('fk')
    poses = {
        'current': (_pose(current_fk, '측정한 파지 관절값의 모델 TCP',
                          runtime_joints=state['current_runtime_joints_deg'])
                    if current_fk else None),
        'default': _pose(default_fk, 'GUI 기본 URDF 예시 자세 (실물 홈 아님)'),
        'zero': _pose(zero_fk, 'URDF 모든 팔 관절 0° (실물 홈 아님)'),
    }
    result = {
        'version': 1, 'kind': 'grasp_pose_reference',
        'preview_only': True, 'executable': False, 'executed': False,
        'reference_valid': bool(reference_valid),
        'reference_validity': 'model_space_only' if reference_valid else 'invalid_measured_state',
        'frame': state['frame'], 'tip_frame': state['tip_frame'],
        'model_hash': state['model_hash'], 'mapping_id': state['mapping_id'],
        'joint_mapping': deepcopy(state['joint_mapping']),
        'hardware_alignment_verified': False,
        'mapping_was_verified': state['hardware_alignment_verified'],
        'kinematics_library': default_fk.get('kinematics_library'),
        'kinematics_version': default_fk.get('kinematics_version'),
        'provenance': {
            'observation_id': observation.get('observation_id', action_context.get('observation_id')),
            'runtime_id': observation.get('runtime_id', action_context.get('runtime_id')),
            'timestamp': observation.get('timestamp'), 'ego_timestamp': ego.get('timestamp'),
            'simulated': observation.get('simulated'), 'calibration_id': ego.get('calibration_id'),
            'calibration_status': deepcopy(ego.get('calibration_status')),
            'raw': deepcopy(ego.get('raw')), 'joints': deepcopy(observed_joints),
            'gripper': {'value': observed_joints.get('gripper'), 'unit': '%',
                        'used_in_arm_ik': False},
            'source': deepcopy(packet_dict.get('source')),
        },
        'poses': poses, 'delta': None, 'effective_urdf_limits_deg': None,
        'checks': {}, 'reason_codes': reason_codes, 'reasons': reasons,
        'scope': {
            'geometric_only': True, 'speed_time_constrained': False,
            'collision_checked': False, 'position_only': True,
            'calibration_estimated': False, 'cube_centre_measured': False,
        },
        'notes': [
            'XYZ는 arm_base 기준 gripper_frame_link의 모델 좌표입니다. 실제 큐브 중심을 측정한 값이 아닙니다.',
            '현재 관절값과 FK 결과 한 쌍으로 실물 영점·회전 부호·좌표 정합이 검증되지는 않습니다.',
            '기본값은 GUI URDF 예시 (0, -15, 35, -20, 0)°이며 실물 홈 자세를 지정하지 않습니다.',
            'IK 비교는 전체 보정 범위의 끝점 계산입니다. 속도·시간 제한, 경로·충돌·실물 이동을 검증하지 않습니다.',
            'XYZ만 같아도 관절 자세는 여러 개일 수 있습니다. 원래 파지 자세를 재현할 때는 저장한 관절값과 접근 방향도 확인해야 합니다.',
        ],
    }
    if current_fk:
        result['delta'] = {
            'current_minus_default_xyz_m': [
                current - default for current, default in zip(current_fk['xyz_m'], default_fk['xyz_m'])],
            'current_minus_default_urdf_joints_deg': {
                name: current_fk['joints_deg'][name] - MODEL_SEED[name] for name in ARM_JOINTS},
        }

    specifications = (
        ('current_round_trip', 'current', 'current'),
        ('current_to_default', 'current', 'default'),
        ('default_to_current', 'default', 'current'),
    )
    limits = None
    if reference_valid:
        limits = {}
        for name in ARM_JOINTS:
            entry = ego['limits'][name]
            mapping = controller.joint_mapping[name]
            mapped = sorted(mapping['sign'] * entry[edge] + mapping['offset_deg'] for edge in ('min', 'max'))
            urdf_low, urdf_high = default_fk['urdf_limits_deg'][name]
            limits[name] = [max(mapped[0], urdf_low), min(mapped[1], urdf_high)]
        result['effective_urdf_limits_deg'] = limits
    for name, seed_name, target_name in specifications:
        seed_pose, target_pose = poses[seed_name], poses[target_name]
        check = {
            'seed_pose': seed_name, 'target_pose': target_name,
            'seed_joints_deg': deepcopy(seed_pose['urdf_joints_deg']) if seed_pose else None,
            'target_xyz_m': list(target_pose['xyz_m']) if target_pose else None,
            'success': False, 'skipped': True,
            'reason_codes': [], 'reasons': [],
        }
        if not reference_valid:
            check['reason_codes'] = [code for code in reason_codes if code != 'hardware_alignment_unverified']
            check['reasons'] = ['유효한 측정 관절 기준이 없어 IK 비교를 실행하지 않았습니다.']
        else:
            try:
                solved = geometry.solve_position(check['target_xyz_m'], check['seed_joints_deg'], limits)
                check.update(solved, skipped=False)
                if solved['success']:
                    check['solution_runtime_joints_deg'] = {
                        joint: (solved['solution_joints_deg'][joint] - controller.joint_mapping[joint]['offset_deg'])
                        / controller.joint_mapping[joint]['sign'] for joint in ARM_JOINTS}
                else:
                    check['reason_codes'] = ['ik_no_solution']
                    check['reasons'] = ['전체 보정 범위의 제한된 IK 계산에서 해를 찾지 못했습니다.']
            except ValueError as exc:
                check['reason_codes'] = ['ik_invalid_seed_or_bounds']
                check['reasons'] = [str(exc)]
        check.update(preview_only=True, executable=False, executed=False, joints=None,
                     hardware_alignment_verified=False, mode='preview', geometric_only=True,
                     speed_time_constrained=False)
        result['checks'][name] = check
    return _json_safe(result)
