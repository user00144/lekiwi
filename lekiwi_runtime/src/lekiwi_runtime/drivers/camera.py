"""Latest-frame camera capture and JPEG encoding."""

import base64
import os
import sys
import threading
import time
from typing import Any

class CameraStream:
    """단일 카메라의 최신 프레임을 백그라운드 스레드에서 유지합니다."""

    def __init__(self, name: str, cfg: dict[str, Any], mock: bool = False):
        self.name = name
        self.cfg = cfg
        self.mock = mock
        self.enabled = bool(cfg.get("enabled", True))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._frame = None
        self._frame_mono: float | None = None
        self._frame_time: float | None = None
        self._error: str | None = None if self.enabled else "disabled"

    def start(self) -> None:
        if not self.enabled:
            return
        target = self._run_mock if self.mock else self._run_live
        self._thread = threading.Thread(target=self._run, args=(target,), daemon=True, name=f"cam-{self.name}")
        self._thread.start()

    def _run(self, target) -> None:
        try:
            target()
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"

    @property
    def ready(self) -> bool:
        with self._lock:
            return not self.enabled or self._frame is not None or self._error is not None

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _apply_rotation(self, frame):
        import cv2

        rot = int(self.cfg.get("rotation", 0)) % 360
        if rot == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if rot == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if rot == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def _open_capture(self):
        import cv2

        dev = self.cfg.get("device", 0)
        if isinstance(dev, str) and dev.isdigit():
            target: int | str = int(dev)
        elif isinstance(dev, str) and dev.startswith("/dev/"):
            if not os.path.exists(dev):
                return None, f"Device not found: {dev}"
            target = os.path.realpath(dev)
        else:
            target = dev

        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
        cap = cv2.VideoCapture(target, backend)
        if not cap.isOpened():
            cap.release()
            return None, f"Cannot open camera: {dev}"

        fourcc = self.cfg.get("fourcc", "MJPG")
        if fourcc and len(str(fourcc)) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(fourcc)))
        if self.cfg.get("width"):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cfg["width"]))
        if self.cfg.get("height"):
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cfg["height"]))
        if self.cfg.get("fps"):
            cap.set(cv2.CAP_PROP_FPS, int(self.cfg["fps"]))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap, None

    def _run_live(self) -> None:
        while not self._stop.is_set():
            cap, err = self._open_capture()
            if cap is None:
                with self._lock:
                    self._error = err
                self._stop.wait(2.0)
                continue

            try:
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        with self._lock:
                            self._error = f"Frame read failed ({self.cfg.get('device')})"
                        break
                    frame = self._apply_rotation(frame)
                    now_mono = time.monotonic()
                    now_wall = time.time()
                    with self._lock:
                        self._frame = frame
                        self._frame_mono = now_mono
                        self._frame_time = now_wall
                        self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
            finally:
                cap.release()

            if not self._stop.is_set():
                self._stop.wait(1.0)

    def _run_mock(self) -> None:
        import cv2
        import numpy as np

        w = int(self.cfg.get("width", 640))
        h = int(self.cfg.get("height", 480))
        idx = 0
        while not self._stop.is_set():
            img = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(
                img,
                f"MOCK {self.name.upper()} CAM #{idx}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            idx += 1
            with self._lock:
                self._frame = img
                self._frame_mono = time.monotonic()
                self._frame_time = time.time()
                self._error = None
            self._stop.wait(0.05)

    def snapshot_jpeg_base64(self, jpeg_quality: int | None = None) -> dict[str, Any]:
        """최신 프레임을 JPEG 압축 후 Base64 문자열을 포함한 dict로 반환합니다."""
        import cv2

        with self._lock:
            frame = None if self._frame is None else self._frame.copy()
            mono = self._frame_mono
            wall = self._frame_time
            err = self._error

        if frame is None:
            return {
                "available": False,
                "encoding": "jpeg_base64",
                "width": None,
                "height": None,
                "data": None,
                "timestamp": None,
                "age_s": None,
                "error": err or "No frame captured yet",
            }

        quality = int(jpeg_quality if jpeg_quality is not None else self.cfg.get("jpeg_quality", 85))
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return {
                "available": False,
                "encoding": "jpeg_base64",
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "data": None,
                "timestamp": wall,
                "age_s": round(time.monotonic() - mono, 4) if mono is not None else None,
                "error": "JPEG encode failed",
            }

        b64_str = base64.b64encode(buf.tobytes()).decode("ascii")
        return {
            "available": True,
            "encoding": "jpeg_base64",
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "data": b64_str,
            "timestamp": wall,
            "age_s": round(time.monotonic() - mono, 4) if mono is not None else None,
            "error": err,
        }
