"""Solve the front camera pose in arm_base from a capture directory (OpenCV; no robot I/O).

Run with an environment that has OpenCV, e.g. the runtime venv:

  lekiwi_runtime/.venv/bin/python harness/util/solve_front_camera.py CAPTURE_DIR \
      --intrinsics lekiwi_runtime/tests/SLAM/calibration_result_20261113_48views/calibration.json \
      --output outputs/kinematics/camera-front-kiwi-01.json

1. Floor checkerboard (8x6 inner corners, 24 mm) → camera pitch/roll and height.
2. Cube placements: image difference (cube on floor vs. cube held) gives the cube
   box; its centre is matched to the arm FK grasp point at release.
3. With pitch/roll fixed by the board, yaw and camera position (4 DOF) are fitted
   to the cube points; leave-one-out errors report the expected XY accuracy.
Without a detected board, a 6-DOF PnP on the cube points is used and reported.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import cv2
import numpy as np

BOARD = (8, 6)
SQUARE_M = .024
DIFF_THRESHOLD = 35
MIN_BLOB_PX = 150
FLOOR_ROI_TOP_PX = 300  # objects on the floor appear below the horizon (~282 px)


def load_intrinsics(path):
    data = json.loads(Path(path).read_text())
    if data.get('image_size') != [640, 480]:
        raise ValueError('Intrinsics must be for 640x480')
    return np.array(data['camera_matrix'], float), np.array(data['distortion'], float), str(Path(path))


def find_corners(gray):
    """Board corners in original pixels. The floor board is seen at a grazing angle, so
    fall back to an anisotropically enlarged image (2x wide, 4x tall) for detection."""
    found, corners = cv2.findChessboardCornersSB(gray, BOARD, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
                                                 | cv2.CALIB_CB_ACCURACY)
    if found:
        return corners.reshape(-1, 2), 'sb'
    sx, sy = 2., 4.
    big = cv2.resize(gray, None, fx=sx, fy=sy, interpolation=cv2.INTER_CUBIC)
    found, corners = cv2.findChessboardCorners(big, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None, None
    corners = cv2.cornerSubPix(big, corners, (5, 5), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)).reshape(-1, 2)
    # Pixel-centre convention: original = (scaled + 0.5) / scale - 0.5.
    return np.column_stack(((corners[:, 0] + .5) / sx - .5, (corners[:, 1] + .5) / sy - .5)).astype(np.float32), \
        'classic_2x4'


def detect_board(image, K, D):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, method = find_corners(gray)
    if corners is None:
        return None
    objects = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    objects[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_M
    ok, rvec, tvec = cv2.solvePnP(objects, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(objects, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()
    normal = R[:, 2]
    if normal @ (-t) < 0:  # Point the floor normal toward the camera (up).
        normal = -normal
    return {'normal_camera': normal, 'height_m': float(abs(normal @ t)), 'rms_px': rms, 'method': method,
            'corners': corners.reshape(-1, 2), 'board_origin_camera_m': t.tolist(),
            'board_distance_m': float(np.linalg.norm(t))}


def detect_cube(reference, cube, debug_path):
    a = cv2.GaussianBlur(reference, (5, 5), 0).astype(np.int16)
    b = cv2.GaussianBlur(cube, (5, 5), 0).astype(np.int16)
    diff = np.abs(a - b).max(axis=2).astype(np.uint8)
    mask = (diff > DIFF_THRESHOLD).astype(np.uint8) * 255
    mask[:FLOOR_ROI_TOP_PX] = 0  # ignore people/lighting changes above the floor
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = (int(value) for value in stats[index])
    overlay = cube.copy()
    overlay[labels == index] = (0.5 * overlay[labels == index] + (0, 0, 127)).astype(np.uint8)
    cv2.rectangle(overlay, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 1)
    cv2.imwrite(str(debug_path), overlay)
    if area < MIN_BLOB_PX:
        return None
    # The cube bottom is often cut off by the image edge near the robot, so the
    # feature is the top-centre of the box (also used by perception.locate_box).
    return {'bbox_px': [x, y, x + w, y + h], 'feature_px': [x + w / 2, float(y)], 'area_px': area,
            'touches_edge': x == 0 or y <= FLOOR_ROI_TOP_PX or x + w >= cube.shape[1],
            'bottom_cut': y + h >= cube.shape[0]}


def level_rotation(normal):
    """Rows map camera vectors into a level frame: x forward (optical axis on floor), z up."""
    z = normal / np.linalg.norm(normal)
    forward = np.array([0., 0., 1.]) - z * z[2]
    x = forward / np.linalg.norm(forward)
    return np.vstack((x, np.cross(z, x), z))


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def project(points, R_bc, c, K, D):
    R_cb = R_bc.T
    rvec, _ = cv2.Rodrigues(R_cb)
    projected, _ = cv2.projectPoints(np.asarray(points, float), rvec, -R_cb @ c, K, D)
    return projected.reshape(-1, 2)


def fit(points, pixels, K, D, R_level, initial):
    params = np.array(initial, float)  # yaw, cx, cy, cz

    def residual(p):
        return (project(points, rot_z(p[0]) @ R_level, p[1:], K, D) - pixels).ravel()

    damping = 1e-3
    for _ in range(200):
        r = residual(params)
        J = np.empty((r.size, 4))
        for i, step in enumerate((1e-5, 1e-6, 1e-6, 1e-6)):
            delta = np.zeros(4)
            delta[i] = step
            J[:, i] = (residual(params + delta) - r) / step
        A = J.T @ J
        update = np.linalg.solve(A + damping * np.diag(np.diag(A) + 1e-12), -J.T @ r)
        if np.sum(residual(params + update) ** 2) < np.sum(r ** 2):
            params, damping = params + update, damping * .3
            if np.linalg.norm(update) < 1e-10:
                break
        else:
            damping *= 10
    return params, float(np.sqrt(np.mean(residual(params).reshape(-1, 2) ** 2 * 2)))


def ray_plane(pixel, R_bc, c, K, D, z):
    x, y = cv2.undistortPoints(np.array([[pixel]], float), K, D).reshape(2)
    direction = R_bc @ np.array([x, y, 1.])
    scale = (z - c[2]) / direction[2]
    return c + scale * direction


def pnp_initial(points, pixels, K, D):
    ok, rvec, tvec = cv2.solvePnP(np.asarray(points, float), np.asarray(pixels, float), K, D,
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise RuntimeError('PnP on cube points failed')
    R_cb, _ = cv2.Rodrigues(rvec)
    return R_cb.T, -R_cb.T @ tvec.ravel()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--intrinsics', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--board-thickness-m', type=float, default=0.)
    args = parser.parse_args(argv)
    capture = json.loads((args.capture / 'capture.json').read_text())
    K, D, intrinsics_source = load_intrinsics(args.intrinsics)
    board = None
    if capture.get('board'):
        image = cv2.imread(str(args.capture / capture['board']['images']['front']))
        board = detect_board(image, K, D)
        if board:
            debug = image.copy()
            cv2.drawChessboardCorners(debug, BOARD, board['corners'].reshape(-1, 1, 2).astype(np.float32), True)
            cv2.imwrite(str(args.capture / 'board-detected.jpg'), debug)
    points, pixels, used = [], [], []
    for entry in capture['placements']:
        reference = cv2.imread(str(args.capture / entry['reference']['images']['front']))
        cube = cv2.imread(str(args.capture / entry['cube']['images']['front']))
        found = detect_cube(reference, cube, args.capture / f"p{entry['index']:02d}-detected.jpg")
        entry_report = {'index': entry['index'], 'release_fk_xyz_m': entry['place']['release_fk_xyz_m'],
                        'detection': found}
        used.append(entry_report)
        if found and not found['touches_edge']:
            points.append(entry['place']['release_fk_xyz_m'])
            pixels.append(found['feature_px'])
    if len(points) < 4:
        raise SystemExit(f'Only {len(points)} usable cube detections; need at least 4. See p*-detected.jpg.')
    points, pixels = np.array(points, float), np.array(pixels, float)
    R0, c0 = pnp_initial(points, pixels, K, D)
    report = {'cube_points': len(points), 'pnp6_camera_position_m': c0.tolist()}
    if board:
        R_level = level_rotation(board['normal_camera'])
        M = R0 @ R_level.T
        params, rms = fit(points, pixels, K, D, R_level, [math.atan2(M[1, 0], M[0, 0]), *c0])
        R_bc, c = rot_z(params[0]) @ R_level, params[1:]
        loo = []
        for i in range(len(points)):
            keep = np.arange(len(points)) != i
            p_i, _ = fit(points[keep], pixels[keep], K, D, R_level, params)
            predicted = ray_plane(pixels[i], rot_z(p_i[0]) @ R_level, p_i[1:], K, D, points[i, 2])
            loo.append(float(np.linalg.norm(predicted[:2] - points[i, :2])))
        floor_z = float(c[2] - board['height_m'] - args.board_thickness_m)
        tilt = math.degrees(math.acos(float(np.clip((R0 @ np.array([0, 0, 1.])) @ (R_bc @ np.array([0, 0, 1.])), -1, 1))))
        report.update(method='board pitch/roll + 4-DOF cube fit', reprojection_rms_px=rms,
                      leave_one_out_xy_error_m=loo, leave_one_out_max_m=max(loo),
                      board_rms_px=board['rms_px'], camera_height_above_board_m=board['height_m'],
                      floor_z_m=floor_z, grasp_height_above_floor_m=float(points[:, 2].mean() - floor_z),
                      pnp6_vs_fit_optical_axis_deg=tilt)
    else:
        R_bc, c = R0, c0
        residual = project(points, R_bc, c, K, D) - pixels
        report.update(method='6-DOF PnP on cube points only (no board detected)',
                      reprojection_rms_px=float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))), floor_z_m=None)
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = R_bc, c
    config = {
        'version': 1, 'camera': 'front', 'image_size': [640, 480], 'feature': 'bbox_top_center',
        'camera_matrix': K.tolist(), 'distortion': D.tolist(), 'intrinsics_source': intrinsics_source,
        'T_base_camera': transform.tolist(), 'grasp_plane_z_m': float(points[:, 2].mean()),
        'floor_z_m': report.get('floor_z_m'),
        'calibrated_region_m': {'x': [float(points[:, 0].min()), float(points[:, 0].max())],
                                'y': [float(points[:, 1].min()), float(points[:, 1].max())]},
        'calibration_id': capture.get('calibration_id'), 'mapping_id': capture.get('mapping_id'),
        'created_at': datetime.now(timezone.utc).isoformat(), 'capture': str(args.capture.resolve()),
        'report': {**report, 'placements': used},
    }
    args.output.write_text(json.dumps(config, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()}, indent=2, default=float))
    print('camera position in arm_base (m):', np.round(c, 4).tolist(), '→', args.output)


if __name__ == '__main__':
    main()
