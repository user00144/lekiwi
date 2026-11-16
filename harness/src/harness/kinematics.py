"""SO-101 endpoint IK using Pinocchio kinematics and bounded OSQP steps.

This module never sends robot commands. Runtime degrees are explicitly mapped to
the URDF; execution readiness is tied to an independently verified mapping and the
current calibration fingerprint. XYZ means the fixed arm mount frame, not camera,
world or mobile-base odometry. Endpoint IK does not implement Cartesian MoveL or
collision avoidance. The optional approach axis constrains two orientation DOFs.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import osqp
import pinocchio as pin
from scipy import sparse


ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
POSITION_TOLERANCE_M = .002
APPROACH_TOLERANCE_DEG = 5.0
AXIS_RESIDUAL_SCALE_M = .05
MAX_IK_STARTS = 4
MAX_IK_NFEV_PER_START = 100
MAX_QP_ITERATIONS = 2000
MAX_IK_STEP_RAD = math.radians(10.)
MAX_CARTESIAN_DURATION_MS = 10000
SEED_REGULARIZATION_M_PER_RAD = .0001
DATA_DIR = Path(__file__).with_name('data')
DEFAULT_URDF = DATA_DIR / 'so101_new_calib.urdf'


def _number(value, label):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(f'{label}: 유한한 숫자가 필요합니다. bool·문자열은 허용하지 않습니다.')
    return float(value)


def _joints(joints):
    if not isinstance(joints, dict) or set(joints) != set(ARM_JOINTS):
        raise ValueError('그리퍼를 제외한 팔 5관절을 이름별로 모두 지정해야 합니다.')
    return np.array([_number(joints[name], name) for name in ARM_JOINTS], dtype=float)


def _input_vector(value, label):
    if isinstance(value, np.ndarray):
        if value.shape != (3,):
            raise ValueError(f'{label}: 숫자 3개가 필요합니다.')
        value = value.tolist()
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{label}: 숫자 3개가 필요합니다.')
    return np.array([_number(part, label) for part in value], dtype=float)


def _xml_vector(value, label, default):
    try:
        values = [float(part) for part in (value if value is not None else default).split()]
    except (TypeError, ValueError) as exc:
        raise ValueError(f'URDF {label}: 잘못된 벡터') from exc
    if len(values) != 3 or not all(math.isfinite(part) for part in values):
        raise ValueError(f'URDF {label}: 유한한 3차원 벡터가 필요합니다.')
    return np.array(values, dtype=float)


class RobotGeometry:
    """Pinocchio base_link → gripper_frame_link FK and bounded differential IK.

    Pinocchio Data objects are local to each evaluation so one geometry can safely
    serve concurrent preview workers. Only the five arm coordinates participate;
    the separate jaw joint stays at Pinocchio's neutral configuration.
    """

    def __init__(self, urdf_path):
        raw = Path(urdf_path).read_bytes()
        self.model_hash = hashlib.sha256(raw).hexdigest()
        try:
            robot = ET.fromstring(raw)
        except ET.ParseError as exc:
            raise ValueError('유효한 URDF XML이 필요합니다.') from exc
        if robot.tag != 'robot':
            raise ValueError('URDF robot 루트가 필요합니다.')
        self.model_name = robot.get('name', 'unnamed')
        links = {element.get('name') for element in robot.findall('link')}
        if not {'base_link', 'gripper_frame_link'} <= links:
            raise ValueError('base_link와 gripper_frame_link가 필요합니다.')
        by_child = {}
        for joint in robot.findall('joint'):
            parent, child = joint.find('parent'), joint.find('child')
            if parent is None or child is None or not parent.get('link') or not child.get('link'):
                raise ValueError('URDF joint에 parent/child link가 필요합니다.')
            if child.get('link') in by_child:
                raise ValueError('URDF 링크의 부모 관절이 중복되었습니다.')
            by_child[child.get('link')] = joint
        chain, visited = [], set()
        link = 'gripper_frame_link'
        while link != 'base_link':
            if link in visited or link not in by_child or link not in links:
                raise ValueError('base_link까지 이어지는 유효한 관절 체인이 필요합니다.')
            visited.add(link)
            joint = by_child[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        chain.reverse()
        names, limits = [], {}
        for joint in chain:
            name, kind = joint.get('name'), joint.get('type')
            origin = joint.find('origin')
            for component in ('xyz', 'rpy'):
                _xml_vector(origin.get(component) if origin is not None else None,
                            'origin ' + component, '0 0 0')
            if kind == 'fixed':
                continue
            if kind != 'revolute' or name not in ARM_JOINTS or name in names:
                raise ValueError('체인에는 팔 5개의 유한 범위 revolute 관절만 허용합니다.')
            names.append(name)
            axis_element, limit = joint.find('axis'), joint.find('limit')
            axis = _xml_vector(axis_element.get('xyz') if axis_element is not None else None, 'axis', '1 0 0')
            norm = float(np.linalg.norm(axis))
            if not math.isfinite(norm) or norm <= 0:
                raise ValueError('URDF 회전축은 유한한 0이 아닌 벡터여야 합니다.')
            try:
                low, high = float(limit.get('lower')), float(limit.get('upper'))
            except (AttributeError, TypeError, ValueError) as exc:
                raise ValueError('URDF 팔 관절에는 lower/upper 범위가 필요합니다.') from exc
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError('URDF 팔 관절 범위가 잘못되었습니다.')
            limits[name] = (low, high)
        if tuple(names) != ARM_JOINTS:
            raise ValueError('SO-101 팔 5관절 순서가 일치하지 않습니다.')
        self._limits_rad = np.array([limits[name] for name in ARM_JOINTS], dtype=float)
        try:
            self._model = pin.buildModelFromXML(raw.decode('utf-8'))
        except Exception as exc:
            raise ValueError('Pinocchio가 URDF 모델을 읽지 못했습니다.') from exc
        self._tip_id = self._model.getFrameId('gripper_frame_link')
        self._base_id = self._model.getFrameId('base_link')
        if (self._tip_id >= self._model.nframes or self._base_id >= self._model.nframes
                or self._model.frames[self._base_id].parentJoint != 0):
            raise ValueError('고정 base_link와 gripper_frame_link가 필요합니다.')
        self._joint_ids = [self._model.getJointId(name) for name in ARM_JOINTS]
        if any(index == 0 or index >= self._model.njoints for index in self._joint_ids):
            raise ValueError('Pinocchio 모델에서 팔 관절을 찾지 못했습니다.')
        joints = [self._model.joints[index] for index in self._joint_ids]
        if any(joint.nq != 1 or joint.nv != 1 for joint in joints):
            raise ValueError('팔 관절은 각각 하나의 각도와 속도 좌표가 필요합니다.')
        self._q_indices = np.array([joint.idx_q for joint in joints])
        self._v_indices = np.array([joint.idx_v for joint in joints])
        self._neutral = pin.neutral(self._model)

    def _metadata(self):
        return {'mode': 'preview', 'frame': 'urdf_base_link', 'tip_frame': 'gripper_frame_link',
                'model_hash': self.model_hash, 'model_name': self.model_name,
                'joint_units': 'degrees', 'position_units': 'metres',
                'kinematics_library': 'pinocchio', 'kinematics_version': pin.__version__,
                'hardware_alignment_verified': False, 'executable': False, 'collision_checked': False,
                'mapping_note': '입력 각도를 URDF 관절각으로 사용한 계산입니다. 실물 축 방향·영점·카메라 좌표 정합은 검증되지 않았습니다.'}

    def _configuration(self, radians):
        q = self._neutral.copy()
        q[self._q_indices] = radians
        return q

    def _kinematics(self, radians, *, jacobian=False):
        q = self._configuration(radians)
        data = self._model.createData()
        pin.framesForwardKinematics(self._model, data, q)
        base_inverse = data.oMf[self._base_id].inverse()
        transform = np.array((base_inverse * data.oMf[self._tip_id]).homogeneous)
        if not np.isfinite(transform).all():
            raise ValueError('URDF 변환 계산 결과가 유한하지 않습니다.')
        if not jacobian:
            return transform, None
        world_jacobian = pin.computeFrameJacobian(
            self._model, data, q, self._tip_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # The origin is the tip, with axes expressed in the fixed arm base frame.
        rotation = np.array(base_inverse.rotation)
        arm_jacobian = np.vstack((rotation @ world_jacobian[:3, self._v_indices],
                                  rotation @ world_jacobian[3:, self._v_indices]))
        if not np.isfinite(arm_jacobian).all():
            raise ValueError('Pinocchio Jacobian 계산 결과가 유한하지 않습니다.')
        return transform, arm_jacobian

    def _transform(self, radians):
        return self._kinematics(radians)[0]

    def forward(self, joints):
        """Compute preview FK even outside URDF limits, reporting every violation explicitly."""
        degrees = _joints(joints)
        transform = self._transform(np.deg2rad(degrees))
        limits = np.rad2deg(self._limits_rad)
        violations = {name: {'value_deg': float(degrees[i]), 'lower_deg': float(limits[i, 0]),
                             'upper_deg': float(limits[i, 1])}
                      for i, name in enumerate(ARM_JOINTS) if not limits[i, 0] <= degrees[i] <= limits[i, 1]}
        return {**self._metadata(), 'joints_deg': dict(zip(ARM_JOINTS, degrees.tolist())),
                'transform': transform.tolist(), 'xyz_m': transform[:3, 3].tolist(),
                'urdf_limits_deg': {name: limits[i].tolist() for i, name in enumerate(ARM_JOINTS)},
                'urdf_limit_violations': violations}

    def joint_points(self, joints):
        """Joint origins and tool tip from the same Pinocchio FK as the solver."""
        data = self._model.createData()
        pin.framesForwardKinematics(self._model, data, self._configuration(np.deg2rad(_joints(joints))))
        base_inverse = data.oMf[self._base_id].inverse()
        points = [{'name': 'base_link', 'xyz_m': [0., 0., 0.]}]
        for name, joint_id in zip(ARM_JOINTS, self._joint_ids):
            points.append({'name': name, 'xyz_m': (base_inverse * data.oMi[joint_id]).translation.tolist()})
        points.append({'name': 'gripper_frame_link',
                       'xyz_m': (base_inverse * data.oMf[self._tip_id]).translation.tolist()})
        return points

    def solve_position(self, target_xyz_m, seed_joints, hardware_limits=None, approach_axis=None):
        """Bounded differential IK, optionally constraining the tip's local +Z axis.

        Pinocchio supplies FK and analytic LOCAL_WORLD_ALIGNED frame Jacobians.
        OSQP minimizes a damped linearized task error inside hard joint and step
        bounds. A line search accepts only decreasing nonlinear task costs. The
        optional axis constrains two orientation DOFs, not a full 6DOF pose.

        Invalid seeds are rejected, never clipped. Supplied hardware limits must
        include all five named arm joints in degrees. Equal limits lock a joint.
        At most four deterministic starts use 100 FK evaluations each. This is an
        endpoint solver, not Cyclo's live controller or collision-avoidance system.
        """
        target = _input_vector(target_xyz_m, '목표 위치(m)')
        if math.hypot(*target) >= 2:
            raise ValueError('목표 위치는 URDF 원점에서 2m 미만이어야 합니다.')
        axis = None
        if approach_axis is not None:
            axis = _input_vector(approach_axis, '접근 방향')
            magnitude = math.hypot(*axis)
            if not math.isfinite(magnitude) or magnitude == 0:
                raise ValueError('접근 방향은 유한한 0이 아닌 벡터여야 합니다.')
            axis = axis / magnitude
        seed_deg = _joints(seed_joints)
        bounds_deg = np.rad2deg(self._limits_rad)
        if hardware_limits is not None:
            if not isinstance(hardware_limits, dict) or set(hardware_limits) != set(ARM_JOINTS):
                raise ValueError('하드웨어 범위는 팔 5관절 전체를 지정해야 합니다.')
            for index, name in enumerate(ARM_JOINTS):
                bounds = hardware_limits[name]
                if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                    raise ValueError('하드웨어 관절 범위는 [최소, 최대]여야 합니다.')
                low, high = (_number(value, name) for value in bounds)
                if low > high:
                    raise ValueError('하드웨어 최소값이 최대값보다 큽니다.')
                bounds_deg[index] = (max(bounds_deg[index, 0], low), min(bounds_deg[index, 1], high))
        if np.any(bounds_deg[:, 0] > bounds_deg[:, 1]):
            raise ValueError('URDF와 하드웨어 관절 범위의 교집합이 없습니다.')
        if np.any(seed_deg < bounds_deg[:, 0]) or np.any(seed_deg > bounds_deg[:, 1]):
            raise ValueError('초기 관절값이 유효 범위 밖입니다. 자동으로 잘라 맞추지 않습니다.')
        seed = np.deg2rad(seed_deg)
        bounds = np.deg2rad(bounds_deg)
        free = bounds[:, 0] < bounds[:, 1]
        residual_calls = 0

        def evaluate(values, *, jacobian=False):
            nonlocal residual_calls
            residual_calls += 1
            transform, frame_jacobian = self._kinematics(values, jacobian=jacobian)
            position_error = transform[:3, 3] - target
            error = float(np.linalg.norm(position_error))
            angle = (math.degrees(math.acos(float(np.clip(np.dot(transform[:3, 2], axis), -1, 1))))
                     if axis is not None else None)
            feasible = error <= POSITION_TOLERANCE_M and (angle is None or angle <= APPROACH_TOLERANCE_DEG)
            residual = position_error
            task_jacobian = frame_jacobian[:3] if jacobian else None
            if axis is not None:
                direction = transform[:3, 2]
                residual = np.concatenate((residual, AXIS_RESIDUAL_SCALE_M * (direction - axis)))
                if jacobian:
                    # d(R*z)/dq = omega x (R*z); no finite-difference probes.
                    axis_jacobian = -pin.skew(direction) @ frame_jacobian[3:]
                    task_jacobian = np.vstack((task_jacobian, AXIS_RESIDUAL_SCALE_M * axis_jacobian))
            cost = float(residual @ residual) + SEED_REGULARIZATION_M_PER_RAD ** 2 * float(np.sum((values - seed) ** 2))
            return transform, residual, task_jacobian, error, angle, feasible, cost

        best = seed.copy()
        initial = evaluate(best)
        best_error, best_angle, success, best_cost = initial[3:]
        attempts = []
        if not success and np.any(free):
            lower, upper = bounds[:, 0], bounds[:, 1]
            span = upper - lower
            starts = [seed, (lower + upper) / 2,
                      lower + span * np.array([.65, .25, .75, .35, .55]),
                      lower + span * np.array([.35, .75, .25, .65, .45])]
            for start in starts[:MAX_IK_STARTS]:
                candidate = start.copy()
                before = residual_calls
                count, qp_iterations, status, stop_reason = 0, 0, None, 'evaluation_budget'
                damping = 1e-6
                current = evaluate(candidate, jacobian=True)
                while residual_calls - before < MAX_IK_NFEV_PER_START:
                    _, residual, jacobian, error, angle, feasible, cost = current
                    if (feasible and not success) or (feasible == success and cost < best_cost):
                        best, best_error, best_angle, success, best_cost = candidate.copy(), error, angle, feasible, cost
                    # Refine successful solutions well inside user-facing tolerances.
                    if error <= 1e-6 and (angle is None or angle <= 1e-3):
                        stop_reason = 'target_converged'
                        break
                    if jacobian is None:
                        if residual_calls - before >= MAX_IK_NFEV_PER_START - 1:
                            break
                        current = evaluate(candidate, jacobian=True)
                        _, residual, jacobian, error, angle, feasible, cost = current
                    step_lower = np.maximum(lower - candidate, -MAX_IK_STEP_RAD)
                    step_upper = np.minimum(upper - candidate, MAX_IK_STEP_RAD)
                    regularizer = SEED_REGULARIZATION_M_PER_RAD ** 2
                    # Common scaling improves OSQP accuracy for metre-sized tasks;
                    # it does not change the minimizer or loosen hard bounds.
                    hessian = jacobian.T @ jacobian + (damping + regularizer) * np.eye(5)
                    gradient = jacobian.T @ residual + regularizer * (candidate - seed)
                    qp = osqp.OSQP()
                    qp.setup(P=sparse.csc_matrix(np.triu(2. * hessian * 100.)),
                             q=2. * gradient * 100., A=sparse.eye(5, format='csc'),
                             l=step_lower, u=step_upper, verbose=False,
                             eps_abs=1e-9, eps_rel=1e-9, max_iter=MAX_QP_ITERATIONS,
                             polishing=False, warm_starting=False)
                    solution = qp.solve(raise_error=False)
                    count += 1
                    status = str(solution.info.status)
                    qp_iterations += int(solution.info.iter)
                    step = solution.x
                    # A non-converged QP is not used, including 'solved inaccurate'.
                    if solution.info.status_val != 1:
                        stop_reason = 'qp_' + status.replace(' ', '_')
                        break
                    if step is None or not np.isfinite(step).all():
                        stop_reason = 'qp_nonfinite_solution'
                        break
                    # OSQP is a numerical solver: tolerate only tiny roundoff and
                    # project that roundoff into the exact *step* box. The measured
                    # initial joint state is never clipped or otherwise repaired.
                    if np.any(step < step_lower - 1e-8) or np.any(step > step_upper + 1e-8):
                        stop_reason = 'qp_constraint_violation'
                        break
                    step = np.minimum(np.maximum(step, step_lower), step_upper)
                    if float(np.linalg.norm(step)) < 1e-10:
                        stop_reason = 'stationary'
                        break
                    accepted = False
                    for scale in (1., .5, .25, .125, .0625):
                        if residual_calls - before >= MAX_IK_NFEV_PER_START:
                            break
                        trial = candidate + scale * step
                        # Rounding an increment at a bound must not violate the
                        # original degree-domain execution check by a single ulp.
                        trial = np.minimum(np.maximum(trial, lower), upper)
                        trial_result = evaluate(trial)
                        if trial_result[-1] < cost - 1e-15:
                            candidate, current = trial, trial_result
                            accepted = True
                            damping = max(1e-9, damping / 3.)
                            break
                    if not accepted:
                        if damping >= 1e-1:
                            stop_reason = 'stationary'
                            break
                        damping *= 10.
                # Retain a final accepted point even if it consumed the FK budget.
                _, _, _, error, angle, feasible, cost = current
                if (feasible and not success) or (feasible == success and cost < best_cost):
                    best, best_error, best_angle, success, best_cost = candidate.copy(), error, angle, feasible, cost
                attempts.append({'nfev': residual_calls - before, 'qp_solves': count,
                                 'qp_iterations': qp_iterations, 'qp_status': status,
                                 'stop_reason': stop_reason, 'solver_converged': stop_reason == 'target_converged',
                                 'residual_m': error, 'orientation_error_deg': angle})
                if success:
                    break
        achieved_transform = self._transform(best)
        solved = dict(zip(ARM_JOINTS, np.rad2deg(best).tolist()))
        return {**self._metadata(), 'success': bool(success), 'target_xyz_m': target.tolist(),
                'solution_joints_deg': solved if success else None, 'best_candidate_joints_deg': solved,
                'seed_joints_deg': dict(zip(ARM_JOINTS, seed_deg.tolist())),
                'achieved_xyz_m': achieved_transform[:3, 3].tolist(),
                'residual_m': best_error, 'tolerance_m': POSITION_TOLERANCE_M,
                'effective_limits_deg': {name: bounds_deg[i].tolist() for i, name in enumerate(ARM_JOINTS)},
                'target_approach_axis': axis.tolist() if axis is not None else None,
                'achieved_approach_axis': achieved_transform[:3, 2].tolist(),
                'orientation_error_deg': best_angle, 'orientation_checked': axis is not None,
                'orientation_tolerance_deg': APPROACH_TOLERANCE_DEG if axis is not None else None,
                'orientation_constrained': axis is not None, 'solver': 'pinocchio+osqp',
                'qp_library': 'osqp', 'qp_version': osqp.__version__,
                'jacobian': 'pinocchio.LOCAL_WORLD_ALIGNED.analytic',
                'algorithm': 'bounded_damped_differential_ik',
                'attempts': attempts, 'residual_calls': residual_calls,
                'max_starts': MAX_IK_STARTS, 'max_nfev_per_start': MAX_IK_NFEV_PER_START,
                'max_qp_iterations': MAX_QP_ITERATIONS, 'max_step_deg': math.degrees(MAX_IK_STEP_RAD),
                'note': ('URDF 내 수치 해를 찾은 미리보기입니다. 선택한 +Z 접근 방향 외 전체 자세·충돌·실물 도달은 검증되지 않았습니다.'
                         if success else '제한된 최적화에서 요청한 허용 오차 내 해를 찾지 못했습니다. 수학적 도달 불가능의 증명은 아닙니다.')}


class CartesianController:
    """Resolve one XYZ endpoint against one observation; no I/O to robot hardware.

    ``q_urdf = sign * q_runtime + offset_deg`` is required for all five joints.
    A missing config uses identity mapping for preview only. Changing calibration
    bounds changes runtime angle zero; its new fingerprint invalidates the old
    mapping confirmation. The verified URDF hash must also match the loaded model.
    ``executable`` is necessary but not sufficient to send:
    the episode and runtime must still enforce freshness, IDs and motion feedback.
    """

    def __init__(self, config=None, *, urdf_path=None):
        if config is None:
            config = {
                'version': 1, 'hardware_alignment_verified': False, 'calibration_id': None,
                'model_hash': None,
                'joint_mapping': {name: {'sign': 1, 'offset_deg': 0} for name in ARM_JOINTS},
            }
        elif isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text(encoding='utf-8'))
        if not isinstance(config, dict):
            raise ValueError('Cartesian configuration must be a JSON object')
        allowed = {'version', 'hardware_alignment_verified', 'calibration_id', 'model_hash', 'joint_mapping',
                   'frame', 'tip_frame'}
        if set(config) - allowed:
            raise ValueError('Unknown Cartesian configuration fields: ' + ', '.join(sorted(set(config) - allowed)))
        if type(config.get('version')) is not int or config['version'] != 1:
            raise ValueError('Cartesian configuration version must be 1')
        if config.get('frame', 'arm_base') != 'arm_base' or config.get('tip_frame', 'gripper_frame_link') != 'gripper_frame_link':
            raise ValueError('Only arm_base → gripper_frame_link is supported')
        verified = config.get('hardware_alignment_verified')
        if type(verified) is not bool:
            raise ValueError('hardware_alignment_verified must be a boolean')
        calibration_id = config.get('calibration_id')
        if calibration_id is not None and (not isinstance(calibration_id, str) or not calibration_id.strip()
                                           or len(calibration_id) > 256):
            raise ValueError('calibration_id must be a nonempty string or null')
        if verified and calibration_id is None:
            raise ValueError('A verified joint mapping requires its calibration_id')
        model_hash = config.get('model_hash')
        if model_hash is not None:
            if (not isinstance(model_hash, str) or len(model_hash) != 64
                    or any(character not in '0123456789abcdefABCDEF' for character in model_hash)):
                raise ValueError('model_hash must be a 64-character SHA-256 hex string or null')
            model_hash = model_hash.lower()
        if verified and model_hash is None:
            raise ValueError('A verified joint mapping requires its model_hash (64-character URDF SHA-256)')
        mapping = config.get('joint_mapping')
        if not isinstance(mapping, dict) or set(mapping) != set(ARM_JOINTS):
            raise ValueError('joint_mapping must explicitly name all five arm joints')
        signs, offsets = [], []
        for name in ARM_JOINTS:
            entry = mapping[name]
            if not isinstance(entry, dict) or set(entry) != {'sign', 'offset_deg'}:
                raise ValueError(f'{name}: mapping requires only sign and offset_deg')
            sign = _number(entry['sign'], f'{name}.sign')
            if sign not in (-1, 1):
                raise ValueError(f'{name}: sign must be -1 or 1')
            signs.append(sign)
            offsets.append(_number(entry['offset_deg'], f'{name}.offset_deg'))
        self.signs = np.array(signs)
        self.offsets = np.array(offsets)
        self.hardware_alignment_verified = verified
        self.calibration_id = calibration_id
        self.verified_model_hash = model_hash
        self.geometry = RobotGeometry(urdf_path or DEFAULT_URDF)
        self.joint_mapping = {name: {'sign': int(self.signs[i]), 'offset_deg': float(self.offsets[i])}
                              for i, name in enumerate(ARM_JOINTS)}
        self.mapping_id = hashlib.sha256(json.dumps({
            'version': 1, 'joint_mapping': self.joint_mapping,
            'model_hash': self.geometry.model_hash, 'verified_model_hash': model_hash,
            'calibration_id': calibration_id,
            'hardware_alignment_verified': verified,
        }, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    def _metadata(self, observed_id=None):
        return {
            'frame': 'arm_base', 'tip_frame': 'gripper_frame_link',
            'model_hash': self.geometry.model_hash, 'model_name': self.geometry.model_name,
            'verified_model_hash': self.verified_model_hash,
            'config_version': 1, 'mapping_id': self.mapping_id,
            'joint_mapping': {name: dict(entry) for name, entry in self.joint_mapping.items()},
            'hardware_alignment_verified': self.hardware_alignment_verified,
            'calibration_id': observed_id, 'verified_calibration_id': self.calibration_id,
            'joint_units': 'degrees', 'position_units': 'metres',
            'collision_checked': False, 'cartesian_straight_line': False,
            'orientation_support': 'optional gripper local +Z approach axis; no full 6D pose',
        }

    @staticmethod
    def _reason(result, code, message):
        if code not in result['reason_codes']:
            result['reason_codes'].append(code)
            result['reasons'].append(message)

    def _state(self, packet):
        if not isinstance(packet, dict) or not isinstance(packet.get('observation'), dict):
            raise ValueError('A runtime observation packet is required')
        ego = packet['observation'].get('ego')
        if not isinstance(ego, dict) or ego.get('available') is not True:
            raise ValueError('Current arm feedback is unavailable')
        joints = ego.get('joints')
        if not isinstance(joints, dict) or not all(name in joints for name in ARM_JOINTS):
            raise ValueError('Current feedback must contain all five arm joints')
        runtime = _joints({name: joints[name] for name in ARM_JOINTS})
        limits = ego.get('limits')
        if not isinstance(limits, dict):
            raise ValueError('Runtime arm limits are missing')
        bounds, speeds = [], []
        for name in ARM_JOINTS:
            limit = limits.get(name)
            if not isinstance(limit, dict) or limit.get('unit') != 'deg':
                raise ValueError(f'{name}: current degree limits are required')
            low, high, speed = (_number(limit.get(field), f'{name}.{field}')
                                for field in ('min', 'max', 'max_speed'))
            if low >= high or speed <= 0:
                raise ValueError(f'{name}: invalid range or speed limit')
            bounds.append((low, high))
            speeds.append(speed)
        return ego, runtime, np.array(bounds), np.array(speeds)

    def evaluate(self, packet):
        """Return FK/readiness without assuming that numeric FK verifies hardware."""
        result = {**self._metadata(), 'available': False, 'executable': False,
                  'mode': 'preview', 'current_xyz_m': None, 'fk': None,
                  'reason_codes': [], 'reasons': []}
        try:
            ego, runtime, bounds, _ = self._state(packet)
        except (ValueError, TypeError, OverflowError) as exc:
            self._reason(result, 'observation_unavailable', str(exc))
            return result
        observed_id = ego.get('calibration_id')
        result.update(self._metadata(observed_id))
        urdf = runtime * self.signs + self.offsets
        fk = self.geometry.forward(dict(zip(ARM_JOINTS, urdf.tolist())))
        result.update(available=True, current_xyz_m=fk['xyz_m'],
                      current_runtime_joints_deg=dict(zip(ARM_JOINTS, runtime.tolist())))
        if not self.hardware_alignment_verified:
            self._reason(result, 'hardware_alignment_unverified', '실물 관절 부호·영점 매핑이 검증되지 않아 미리보기만 가능합니다.')
        if self.verified_model_hash is not None and self.verified_model_hash != self.geometry.model_hash:
            self._reason(result, 'model_hash_mismatch', 'URDF 모델이 검증된 관절 매핑과 다릅니다. 모델과 매핑을 다시 확인해야 합니다.')
        if not isinstance(observed_id, str) or not observed_id:
            self._reason(result, 'calibration_id_missing', '관측에 보정 파일 식별자가 없습니다.')
        elif self.hardware_alignment_verified and observed_id != self.calibration_id:
            self._reason(result, 'calibration_id_mismatch', '보정 파일이 검증된 관절 매핑과 다릅니다. 영점 매핑을 다시 확인해야 합니다.')
        calibration_status = ego.get('calibration_status')
        if not isinstance(calibration_status, dict) or calibration_status.get('valid') is not True:
            self._reason(result, 'calibration_invalid', '현재 인코더가 보정 범위 안에 있는지 확인되지 않았습니다.')
        if np.any(runtime < bounds[:, 0]) or np.any(runtime > bounds[:, 1]):
            self._reason(result, 'runtime_seed_out_of_range', '현재 관절값이 runtime 범위 밖입니다. 초기값을 잘라 맞추지 않습니다.')
        if fk['urdf_limit_violations']:
            self._reason(result, 'urdf_seed_out_of_range', '매핑된 현재 관절값이 URDF 범위 밖입니다. 초기값을 잘라 맞추지 않습니다.')
        source = packet.get('source')
        if isinstance(source, dict) and source.get('recorded'):
            self._reason(result, 'recorded_observation', '저장된 관측은 동작을 승인할 수 없습니다.')
        result['executable'] = not result['reason_codes']
        result['mode'] = 'ready' if result['executable'] else 'preview'
        # Override the copied preview helper's fixed false metadata explicitly.
        result['fk'] = {**fk, **self._metadata(observed_id), 'executable': result['executable'],
                        'mode': result['mode'], 'mapping_note': 'q_urdf = sign × q_runtime + offset_deg'}
        return result

    def solve(self, packet, request, duration_ms):
        """Solve an endpoint inside URDF, runtime and per-duration speed bounds.

        Invalid request shape raises ValueError. Infeasible/unavailable geometry
        returns a result with executable=false and no dispatchable ``joints``.
        A successful unverified preview has solution fields but still no ``joints``.
        """
        if not isinstance(request, dict) or set(request) - {'frame', 'mode', 'xyz_m', 'approach_axis'}:
            raise ValueError('Cartesian request accepts only frame, mode, xyz_m and approach_axis')
        if request.get('frame') != 'arm_base' or request.get('mode') not in ('absolute', 'relative'):
            raise ValueError('Cartesian request requires arm_base and absolute/relative mode')
        target = _input_vector(request.get('xyz_m'), 'xyz_m')
        if type(duration_ms) is not int or not 100 <= duration_ms <= MAX_CARTESIAN_DURATION_MS:
            raise ValueError(f'Cartesian duration_ms must be an integer from 100 to {MAX_CARTESIAN_DURATION_MS}')
        axis = request.get('approach_axis')
        if axis is not None:
            axis = _input_vector(axis, 'approach_axis')
            if not math.isfinite(math.hypot(*axis)) or math.hypot(*axis) == 0:
                raise ValueError('approach_axis must be a finite nonzero vector')
            axis = axis.tolist()
        state = self.evaluate(packet)
        result = {**state, 'success': False, 'executable': False, 'joints': None,
                  'solution_runtime_joints_deg': None, 'solution_urdf_joints_deg': None,
                  'duration_ms': duration_ms, 'request': dict(request), 'residual_m': None}
        if not state['available']:
            return result
        if request['mode'] == 'relative':
            target += np.array(state['current_xyz_m'])
        result['target_xyz_m'] = target.tolist()
        if math.hypot(*target) >= 2:
            raise ValueError('Target must be less than 2 metres from arm_base')
        blocked = {'runtime_seed_out_of_range', 'urdf_seed_out_of_range', 'calibration_invalid'}
        if blocked.intersection(state['reason_codes']):
            return result
        _, runtime, bounds, speeds = self._state(packet)
        step = speeds * duration_ms / 1000
        bounded_runtime = np.column_stack((np.maximum(bounds[:, 0], runtime - step),
                                          np.minimum(bounds[:, 1], runtime + step)))
        mapped = bounded_runtime * self.signs[:, None] + self.offsets[:, None]
        mapped.sort(axis=1)  # Negative signs reverse both endpoints.
        urdf = runtime * self.signs + self.offsets
        try:
            solved = self.geometry.solve_position(
                target.tolist(), dict(zip(ARM_JOINTS, urdf.tolist())),
                {name: mapped[i].tolist() for i, name in enumerate(ARM_JOINTS)}, axis)
        except ValueError as exc:
            self._reason(result, 'ik_invalid_seed_or_bounds', str(exc))
            return result
        # Use controller readiness, not RobotGeometry's deliberately preview-only metadata.
        result.update({key: value for key, value in solved.items() if key not in {
            'mode', 'frame', 'hardware_alignment_verified', 'executable', 'mapping_note', 'note'}})
        result.update(self._metadata(state['calibration_id']))
        result['success'] = solved['success']
        if not solved['success']:
            self._reason(result, 'ik_no_solution', '시간·속도·관절 범위 안에서 허용 오차를 만족하는 IK 해를 찾지 못했습니다.')
            return result
        solution_urdf = _joints(solved['solution_joints_deg'])
        solution_runtime = (solution_urdf - self.offsets) / self.signs
        # A solver or mapping regression must never bypass the original runtime bounds.
        if np.any(solution_runtime < bounded_runtime[:, 0] - 1e-9) or np.any(solution_runtime > bounded_runtime[:, 1] + 1e-9):
            result['success'] = False
            self._reason(result, 'ik_solution_out_of_bounds', 'IK 변환 결과가 runtime 속도·관절 범위를 벗어났습니다.')
            return result
        result['solution_runtime_joints_deg'] = dict(zip(ARM_JOINTS, solution_runtime.tolist()))
        result['solution_urdf_joints_deg'] = solved['solution_joints_deg']
        result['effective_runtime_limits_deg'] = {name: bounded_runtime[i].tolist() for i, name in enumerate(ARM_JOINTS)}
        result['executable'] = state['executable'] and solved['success']
        result['mode'] = 'ready' if result['executable'] else 'preview'
        if result['executable']:
            result['joints'] = dict(result['solution_runtime_joints_deg'])
        result['note'] = 'XYZ 끝점 해입니다. 직선 경로·충돌 회피·전체 6D 자세는 보장하지 않습니다.'
        return result


def main(argv=None):
    """Read a saved observation and print a numerical preview; never dispatch."""
    import argparse

    parser = argparse.ArgumentParser(description='Read-only SO-101 XYZ IK preview from saved observations')
    parser.add_argument('--observation', type=Path, required=True, help='Saved observation.json packet')
    parser.add_argument('--xyz', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'), help='Metres in arm_base')
    parser.add_argument('--mode', choices=('absolute', 'relative'), default='absolute')
    parser.add_argument('--duration-ms', type=int, default=500)
    parser.add_argument('--approach-axis', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--config', type=Path, help='Explicit joint mapping configuration JSON')
    args = parser.parse_args(argv)
    try:
        observed = json.loads(args.observation.read_text(encoding='utf-8'))
        if not isinstance(observed, dict):
            raise ValueError('Saved observation must be a JSON object')
        # Disk observations can support numerical analysis, never a motion authorization.
        source = observed.get('source')
        observed['source'] = {**(source if isinstance(source, dict) else {}), 'recorded': True}
        result = CartesianController(args.config).solve(observed, {
            'frame': 'arm_base', 'mode': args.mode, 'xyz_m': args.xyz,
            'approach_axis': args.approach_axis,
        }, args.duration_ms)
        result['preview_only'] = True
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if result['success'] else 1
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f'{parser.prog}: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
