"""Front-camera pixel → arm_base XYZ on a calibrated horizontal grasp plane (numpy only).

The camera config is produced by ``util/solve_front_camera.py`` from a floor
checkerboard (camera pitch/roll/height) and cube placements whose grasp points
come from arm FK. A bounding-box centre is mapped by intersecting its viewing
ray with the plane z = ``grasp_plane_z_m``; the result is a grasp-point estimate
for a cube resting on the floor, not a verified object pose.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


SENSITIVITY_PX = 3.
REGION_MARGIN_M = .02


def _matrix(value, shape, label):
    array = np.array(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f'{label} must be a finite {shape} array')
    return array


class CameraModel:
    def __init__(self, config):
        if isinstance(config, (str, Path)):
            config = json.loads(Path(config).read_text(encoding='utf-8'))
        if not isinstance(config, dict) or config.get('version') != 1:
            raise ValueError('Camera config version 1 is required')
        self.config = config
        self.camera = config.get('camera')
        if self.camera != 'front':
            raise ValueError('Only the front camera is supported')
        size = config.get('image_size')
        if (not isinstance(size, list) or len(size) != 2
                or any(type(value) is not int or value <= 0 for value in size)):
            raise ValueError('image_size must be [width, height]')
        self.width, self.height = size
        self.K = _matrix(config.get('camera_matrix'), (3, 3), 'camera_matrix')
        self.D = _matrix(config.get('distortion'), (5,), 'distortion')
        transform = _matrix(config.get('T_base_camera'), (4, 4), 'T_base_camera')
        self.R, self.t = transform[:3, :3], transform[:3, 3]
        if abs(np.linalg.det(self.R) - 1) > 1e-6 or np.abs(self.R.T @ self.R - np.eye(3)).max() > 1e-6:
            raise ValueError('T_base_camera rotation must be orthonormal')
        self.grasp_plane_z = float(config.get('grasp_plane_z_m'))
        if not math.isfinite(self.grasp_plane_z):
            raise ValueError('grasp_plane_z_m must be finite')
        region = config.get('calibrated_region_m') or {}
        self.region = {axis: tuple(float(v) for v in region[axis]) for axis in ('x', 'y') if axis in region}
        self.calibration_id = config.get('calibration_id')
        self.feature = config.get('feature', 'bbox_center')
        if self.feature not in ('bbox_center', 'bbox_top_center'):
            raise ValueError('feature must be bbox_center or bbox_top_center')

    def _distort(self, x, y):
        k1, k2, p1, p2, k3 = self.D
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        return (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)

    def undistort(self, u, v):
        """Normalized undistorted image coordinates (OpenCV model, fixed-point inverse)."""
        xd = (u - self.K[0, 2]) / self.K[0, 0]
        yd = (v - self.K[1, 2]) / self.K[1, 1]
        x, y = xd, yd
        for _ in range(30):
            dx, dy = self._distort(x, y)
            x, y = x + (xd - dx), y + (yd - dy)
        return x, y

    def project(self, xyz):
        """arm_base point → distorted pixel, or None if behind the camera."""
        p = self.R.T @ (np.asarray(xyz, dtype=float) - self.t)
        if p[2] <= 1e-6:
            return None
        x, y = self._distort(p[0] / p[2], p[1] / p[2])
        return [float(self.K[0, 0] * x + self.K[0, 2]), float(self.K[1, 1] * y + self.K[1, 2])]

    def pixel_to_plane(self, u, v, z):
        x, y = self.undistort(u, v)
        direction = self.R @ np.array([x, y, 1.])
        if direction[2] >= -1e-9:
            return None  # Ray at or above the horizon never meets the plane below the camera.
        scale = (z - self.t[2]) / direction[2]
        if scale <= 0:
            return None
        return (self.t + scale * direction).tolist()

    def in_region(self, xyz):
        if not self.region:
            return None
        return all(self.region[axis][0] - REGION_MARGIN_M <= xyz[index] <= self.region[axis][1] + REGION_MARGIN_M
                   for index, axis in ((0, 'x'), (1, 'y')) if axis in self.region)

    def locate_box(self, bbox):
        """Grasp-point estimate for an object box [u_min, v_min, u_max, v_max] in pixels."""
        if (not isinstance(bbox, (list, tuple)) or len(bbox) != 4
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in bbox)):
            raise ValueError('bbox_px must be four finite pixel values')
        u0, v0, u1, v1 = (float(value) for value in bbox)
        if not (0 <= u0 < u1 <= self.width and 0 <= v0 < v1 <= self.height) or u1 - u0 < 3 or v1 - v0 < 3:
            raise ValueError(f'bbox_px must lie inside the {self.width}x{self.height} image with u_min<u_max, v_min<v_max')
        u, v = (u0 + u1) / 2, (v0 if self.feature == 'bbox_top_center' else (v0 + v1) / 2)
        xyz = self.pixel_to_plane(u, v, self.grasp_plane_z)
        if xyz is None:
            return {'ok': False, 'pixel': [u, v], 'bbox_px': [u0, v0, u1, v1],
                    'error': 'Box centre is at or above the horizon; it cannot be on the floor plane.'}
        spread = 0.
        for du, dv in ((SENSITIVITY_PX, 0), (-SENSITIVITY_PX, 0), (0, SENSITIVITY_PX), (0, -SENSITIVITY_PX)):
            moved = self.pixel_to_plane(u + du, v + dv, self.grasp_plane_z)
            spread = max(spread, math.inf if moved is None else math.dist(moved[:2], xyz[:2]))
        return {'ok': True, 'xyz_m': [round(value, 4) for value in xyz], 'pixel': [u, v],
                'bbox_px': [u0, v0, u1, v1], 'frame': 'arm_base',
                'meaning': 'gripper grasp point for an object resting on the floor (grasp_plane_z_m)',
                'horizontal_distance_m': round(math.dist(xyz[:2], self.t[:2]), 4),
                'xy_change_per_3px_m': round(spread, 4) if math.isfinite(spread) else None,
                'in_calibrated_region': self.in_region(xyz)}
