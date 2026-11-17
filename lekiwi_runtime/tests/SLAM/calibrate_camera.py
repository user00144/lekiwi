"""Calibrate saved OV5648 checkerboard images; never open hardware."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("결과 폴더가 이미 존재합니다. 새 경로를 지정하세요.")

    obj = np.zeros((48, 3), np.float32)
    obj[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * 0.024
    objects, points, sources, skipped = [], [], [], []
    seen = set()
    coverage = np.zeros((3, 4), dtype=int)
    for folder in args.sessions:
        meta = json.loads((folder / "capture.json").read_text())
        if (meta["status"] != "complete" or meta["inner_corners"] != [8, 6]
                or meta["image_size"] != [640, 480]
                or not np.isclose(meta["square_size_m"], 0.024)):
            raise RuntimeError(f"촬영 상태 또는 보드 설정 불일치: {folder}")
        for record in meta["frames"]:
            path = folder / "images" / record["file"]
            frame = cv2.imread(str(path))
            if frame is None or frame.shape != (480, 640, 3):
                raise RuntimeError(f"이미지 오류: {path}")
            digest = hashlib.sha256(frame.tobytes()).hexdigest()
            if digest != record["pixel_sha256"]:
                raise RuntimeError(f"촬영 기록과 이미지 해시 불일치: {path}")
            if digest in seen:
                skipped.append({"file": str(path.resolve()), "reason": "duplicate"})
                continue
            seen.add(digest)
            found, corners = cv2.findChessboardCornersSB(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (8, 6), cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if not found:
                skipped.append({"file": str(path.resolve()), "reason": "not_detected"})
                continue
            corners = corners.reshape(-1, 1, 2)
            objects.append(obj.copy())
            points.append(corners)
            sources.append({"file": str(path.resolve()), "pixel_sha256": digest})
            xy = corners.reshape(-1, 2)
            cols = np.clip((xy[:, 0] * 4 / 640).astype(int), 0, 3)
            rows = np.clip((xy[:, 1] * 3 / 480).astype(int), 0, 2)
            for row, col in set(zip(rows.tolist(), cols.tolist())):
                coverage[row, col] += 1
    if len(points) < 15:
        raise RuntimeError(f"유효 이미지 부족: {len(points)}장")

    rms, K, D, rvecs, tvecs, std, _, per_view = cv2.calibrateCameraExtended(
        objects, points, (640, 480), None, None
    )
    if not (np.isfinite(K).all() and np.isfinite(D).all() and np.isfinite(rms)
            and K[0, 0] > 0 and K[1, 1] > 0):
        raise RuntimeError("Calibration 결과가 유효하지 않습니다.")
    for source, error, rv, tv in zip(sources, per_view.ravel(), rvecs, tvecs):
        source.update(rms_px=float(error), board_rvec=rv.ravel().tolist(),
                      board_tvec_m=tv.ravel().tolist())
    xy = np.concatenate(points).reshape(-1, 2)
    report = {
        "created_at": datetime.now().astimezone().isoformat(),
        "opencv_version": cv2.__version__, "model": "OpenCV pinhole, k1 k2 p1 p2 k3",
        "image_size": [640, 480], "inner_corners": [8, 6], "square_size_m": 0.024,
        "camera_matrix": K.tolist(), "distortion": D.ravel().tolist(),
        "rms_px": float(rms), "std_intrinsics": std.ravel().tolist(),
        "coverage_grid_4x3": coverage.tolist(),
        "corner_bounds_xy": [xy.min(axis=0).tolist(), xy.max(axis=0).tolist()],
        "sources": sources, "skipped": skipped,
        "conditions": "640x480 MJPG, original orientation, unchanged focus; BGR input. "
                      "Training reprojection error is not independent accuracy validation.",
    }
    k1, k2, p1, p2, k3 = D.ravel()
    params = {
        "Camera1.fx": K[0, 0], "Camera1.fy": K[1, 1],
        "Camera1.cx": K[0, 2], "Camera1.cy": K[1, 2],
        "Camera1.k1": k1, "Camera1.k2": k2, "Camera1.p1": p1,
        "Camera1.p2": p2, "Camera1.k3": k3,
        "Camera.fps": 30, "Camera.RGB": 0,
        "Camera.width": 640, "Camera.height": 480,
        "ORBextractor.nFeatures": 1000, "ORBextractor.scaleFactor": 1.2,
        "ORBextractor.nLevels": 8, "ORBextractor.iniThFAST": 20, "ORBextractor.minThFAST": 7,
        "Viewer.KeyFrameSize": 0.05, "Viewer.KeyFrameLineWidth": 1.0,
        "Viewer.GraphLineWidth": 0.9, "Viewer.PointSize": 2.0,
        "Viewer.CameraSize": 0.08, "Viewer.CameraLineWidth": 3.0,
        "Viewer.ViewpointX": 0.0, "Viewer.ViewpointY": -0.7,
        "Viewer.ViewpointZ": -1.8, "Viewer.ViewpointF": 500.0,
    }
    yaml = '%YAML:1.0\n---\n# OV5648: raw BGR 640x480, unchanged focus and orientation.\n'
    yaml += 'File.version: "1.0"\nCamera.type: "PinHole"\n'
    yaml += "\n".join(f"{key}: {float(value):.12f}" if not isinstance(value, int)
                      else f"{key}: {value}" for key, value in params.items()) + "\n"
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "calibration.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    (args.output / "OV5648.yaml").write_text(yaml, encoding="utf-8")
    print(f"유효 {len(points)}장 / 제외 {len(skipped)}장 / RMS {rms:.6f} px")
    print("K:", K, "\nD:", D.ravel(), "\n관측 분포:", coverage)
    print("교차점 범위:", report["corner_bounds_xy"])
    print("영상별 최대 RMS:", float(per_view.max()))
    print("결과:", args.output.resolve())


if __name__ == "__main__":
    main()
