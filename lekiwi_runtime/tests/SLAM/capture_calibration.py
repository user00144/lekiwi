"""Capture varied checkerboard images, or audit saved images without a camera."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


DEVICE = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_USB_Camera_YL20240927V0-video-index0"
ROOT = Path(__file__).resolve().parent


def draw_preview(frame, corners, coverage):
    """Draw only on a copy: calibration PNGs must remain unannotated."""
    canvas = frame.copy()
    height, width = canvas.shape[:2]
    for row in range(3):
        for col in range(4):
            left, top = col * width // 4, row * height // 3
            right, bottom = (col + 1) * width // 4, (row + 1) * height // 3
            hits = int(coverage[row, col])
            color = (70, 180, 70) if hits else (80, 170, 240)
            cv2.rectangle(canvas, (left, top), (right - 1, bottom - 1), color, 1)
            cv2.putText(canvas, f"{hits} views", (left + 6, top + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (40, 40), (width - 41, height - 41), (0, 200, 255), 1)
    if corners is not None:
        cv2.drawChessboardCorners(canvas, (8, 6), corners.reshape(-1, 1, 2), True)
    return canvas


def update_coverage(coverage, corners):
    cols = np.clip((corners[:, 0] * 4 / 640).astype(int), 0, 3)
    rows = np.clip((corners[:, 1] * 3 / 480).astype(int), 0, 2)
    for row, col in set(zip(rows.tolist(), cols.tolist())):
        coverage[row, col] += 1


class Preview:
    def __init__(self, count, automatic):
        # cv2 is headless; use the already installed Tk/Pillow for display.
        import tkinter as tk
        from PIL import Image, ImageTk

        self.Image, self.ImageTk = Image, ImageTk
        self.root = tk.Tk()
        self.root.title("OV5648 Calibration — 8x6 / 24 mm")
        self.save_requested = False
        self.closed = False
        self.count = count
        self.image = tk.Label(self.root)
        self.image.pack()
        self.summary = tk.StringVar(value="카메라 준비 중")
        tk.Label(self.root, textvariable=self.summary, wraplength=640).pack(pady=5)
        self.hint = tk.StringVar(value="")
        tk.Label(self.root, textvariable=self.hint, wraplength=640).pack(pady=3)
        controls = tk.Frame(self.root)
        controls.pack(pady=5)
        tk.Button(controls, text="저장 [Space]", command=self.request_save).pack(side=tk.LEFT, padx=8)
        tk.Button(controls, text="종료 [Q / Esc]", command=self.request_close).pack(side=tk.LEFT, padx=8)
        mode = "자동 촬영" if automatic else "수동 촬영: 보드를 멈춘 뒤 Space 또는 저장 버튼"
        tk.Label(self.root, text=mode).pack()
        tk.Label(self.root, text="주황 격자: 촬영 없음 / 초록 격자: 저장된 교차점 관측 있음\n"
                 "전체 보드가 보이도록, 아래쪽·좌우·모서리와 거리·기울기를 다양하게 촬영").pack(pady=5)
        self.root.bind("<space>", self.request_save)
        self.root.bind("q", self.request_close)
        self.root.bind("Q", self.request_close)
        self.root.bind("<Escape>", self.request_close)
        self.root.protocol("WM_DELETE_WINDOW", self.request_close)

    def request_save(self, event=None):
        self.save_requested = True

    def request_close(self, event=None):
        self.closed = True

    def poll(self):
        self.root.update_idletasks()
        self.root.update()
        return not self.closed

    def show(self, frame, corners, coverage, saved, message, fps):
        annotated = draw_preview(frame, corners, coverage)
        rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        self.photo = self.ImageTk.PhotoImage(self.Image.fromarray(rgb), master=self.root)
        self.image.configure(image=self.photo)
        detected = "8×6 검출됨" if corners is not None else "8×6 검출 안 됨"
        self.summary.set(f"저장 {saved}/{self.count}장 | {detected} | 미리보기 {fps:.1f} FPS")
        self.hint.set(message)

    def close(self):
        self.root.destroy()


class FrameGuard:
    def __init__(self, timeout=3.0):
        self.timeout = timeout
        self.last_hash = None
        self.last_change = None

    def observe(self, frame, now):
        digest = hashlib.sha256(frame.tobytes()).hexdigest()
        if digest != self.last_hash:
            self.last_hash = digest
            self.last_change = now
        elif now - self.last_change >= self.timeout:
            raise RuntimeError(
                f"동일 픽셀이 {self.timeout:.1f}초 이상 반복됩니다. "
                "카메라 스트림 정지 의심: 촬영을 중단합니다."
            )
        return digest


def detect(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray, (8, 6), cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    return None if not found else corners.reshape(-1, 2)


def pose_distance(corners, previous):
    # An unmarked board can be detected in the opposite corner order.
    return min(
        float(np.sqrt(np.mean(np.sum((corners - candidate) ** 2, axis=1))))
        for saved in previous
        for candidate in (saved, saved[::-1])
    ) if previous else float("inf")


def audit(folder, min_shift):
    hashes = {}
    poses = []
    total = 0
    for path in sorted(folder.glob("*.png")):
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"이미지 읽기 실패: {path}")
        total += 1
        digest = hashlib.sha256(frame.tobytes()).hexdigest()
        if digest in hashes:
            print(f"{path.name}: 중복 → {hashes[digest]}")
            continue
        hashes[digest] = path.name
        corners = detect(frame)
        if corners is None:
            print(f"{path.name}: 체커보드 검출 실패")
        elif pose_distance(corners, poses) < min_shift:
            print(f"{path.name}: 기존 보드 자세와 유사")
        else:
            poses.append(corners.copy())
            print(f"{path.name}: 후보")
    print(f"전체 {total}장 / 서로 다른 픽셀 {len(hashes)}장 / 자세 변화 후보 {len(poses)}장")
    print("원본 파일은 변경하지 않았습니다. 후보 수는 Calibration 품질 보장이 아닙니다.")


def capture(args):
    preview = None
    if not args.no_preview:
        try:
            preview = Preview(args.count, args.auto)
        except Exception as exc:
            raise RuntimeError("미리보기 생성 실패. Orin Desktop 터미널의 DISPLAY를 확인하세요. "
                               "화면 없이 자동 촬영하려면 --no-preview를 사용하세요.") from exc
    cap = None
    folder = None
    records, poses, seen = [], [], set()
    coverage = np.zeros((3, 4), dtype=int)
    status = "incomplete"
    try:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError("Camera open failed")
        for prop, value in (
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
            (cv2.CAP_PROP_FRAME_WIDTH, 640),
            (cv2.CAP_PROP_FRAME_HEIGHT, 480),
            (cv2.CAP_PROP_FPS, 30),
            (cv2.CAP_PROP_BUFFERSIZE, 1),
        ):
            if not cap.set(prop, value):
                print(f"주의: Capture 속성 설정 미수락: {prop}", flush=True)
        folder = ROOT / ("calibration_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        (folder / "images").mkdir(parents=True, exist_ok=False)
        guard = FrameGuard()
        started = time.monotonic()
        next_check = started + 3
        automatic = args.auto or args.no_preview
        message = "준비 중: 보드를 화면에 배치하세요."
        last_display = started
        print(f"저장 위치: {folder}\n3초 준비 후 {'자동' if automatic else 'Space/버튼으로'} 촬영합니다.", flush=True)
        while len(records) < args.count:
            if preview is not None and not preview.poll():
                status = "interrupted"
                break
            if time.monotonic() - started > args.duration:
                raise RuntimeError("촬영 제한 시간 종료: 이미지가 부족하면 재촬영하세요.")
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Frame read failed; 촬영을 중단합니다.")
            if frame.shape != (480, 640, 3):
                raise RuntimeError(f"Unexpected frame shape: {frame.shape}")
            now = time.monotonic()
            # Inspect every received frame, including between saving intervals.
            digest = guard.observe(frame, now)
            corners = detect(frame)
            requested = preview is not None and preview.save_requested
            if preview is not None:
                preview.save_requested = False
            if now >= next_check and (automatic or requested):
                next_check = now + 0.5
                shift = pose_distance(corners, poses) if corners is not None else 0
                if digest in seen:
                    message = "저장 안 함: 이미 저장한 프레임"
                elif corners is None:
                    message = "저장 안 함: 8×6 검출 실패. 보드 전체를 선명하게 보여주세요."
                elif shift < args.min_shift:
                    message = f"저장 안 함: 기존 자세와 유사 ({shift:.1f}px). 위치·거리·각도를 바꾸세요."
                else:
                    name = f"frame_{len(records) + 1:03d}.png"
                    # Save the raw frame, never the preview overlay.
                    if not cv2.imwrite(str(folder / "images" / name), frame):
                        raise RuntimeError(f"Save failed: {name}")
                    seen.add(digest)
                    poses.append(corners.copy())
                    update_coverage(coverage, corners)
                    records.append({"file": name, "pixel_sha256": digest,
                                    "host_monotonic_s": now, "corners": corners.tolist()})
                    message = f"[{len(records):02d}/{args.count}] 저장: {name} — 다음 위치로 옮겨주세요."
                    next_check = time.monotonic() + (2 if automatic else 0.5)
                print(message, flush=True)
            elif requested:
                message = "잠시 기다린 뒤 Space를 다시 누르세요."
            if preview is not None:
                display_now = time.monotonic()
                fps = 1 / max(display_now - last_display, 1e-6)
                last_display = display_now
                preview.show(frame, corners, coverage, len(records), message, fps)
        if len(records) == args.count:
            status = "complete"
    except KeyboardInterrupt:
        status = "interrupted"
        print("촬영 중단", flush=True)
    finally:
        if cap is not None:
            cap.release()
        if preview is not None:
            preview.close()
        if folder is not None and folder.is_dir():
            metadata = {"status": status, "device": args.device,
                        "image_size": [640, 480], "inner_corners": [8, 6],
                        "square_size_m": 0.024, "min_corner_shift_px": args.min_shift,
                        "coverage_grid_4x3": coverage.tolist(),
                        "capture_mode": "automatic" if args.auto or args.no_preview else "manual",
                        "frames": records}
            (folder / "capture.json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            print(f"저장 {len(records)}장 / 상태 {status}\n{folder}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--audit", type=Path, help="기존 PNG 분석만 수행; 파일 변경·카메라 사용 없음")
    mode.add_argument("--capture", action="store_true", help="실제 카메라 촬영 및 새 폴더 저장")
    parser.add_argument("--device", default=DEVICE)
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--min-shift", type=float, default=12)
    parser.add_argument("--auto", action="store_true", help="미리보기를 보면서 자동 저장")
    parser.add_argument("--no-preview", action="store_true", help="미리보기 없이 자동 촬영")
    args = parser.parse_args()
    if args.count <= 0 or args.duration <= 0 or args.min_shift <= 0:
        parser.error("count, duration, min-shift는 양수여야 합니다.")
    if args.audit is not None:
        if not args.audit.is_dir() or not any(args.audit.glob("*.png")):
            parser.error("분석 폴더에 PNG 이미지가 없습니다.")
        audit(args.audit, args.min_shift)
    else:
        capture(args)


if __name__ == "__main__":
    main()
