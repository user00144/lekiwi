"""AWR2944 serial capture and optional sensor configuration."""

import copy
import glob
import math
import os
from pathlib import Path
import struct
import threading
import time
from typing import Any

from . import radar_protocol as reader


def detect_radar_port(interface: str) -> str | None:
    """Use the XDS110 interface identity; never guess from serial port order."""
    matches = sorted(glob.glob(f"/dev/serial/by-id/*Texas_Instruments_XDS110*-if{interface}*"))
    return matches[0] if len(matches) == 1 else None


class RadarStream:
    """AWR2944 밀리미터파 레이더 포인트 클라우드를 백그라운드에서 수신합니다."""

    def __init__(self, cfg: dict[str, Any], mock: bool = False):
        self.cfg = cfg
        self.mock = mock
        self.enabled = bool(cfg.get("enabled", True))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._sample: dict[str, Any] | None = None
        self._sample_mono: float | None = None
        self._initializing: bool = False
        self._error: str | None = None if self.enabled else "disabled"

    def start(self) -> None:
        if not self.enabled:
            return
        target = self._run_mock if self.mock else self._run_live
        self._thread = threading.Thread(target=self._run, args=(target,), daemon=True, name="sensor-radar")
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
            return not self.enabled or self._sample is not None or (
                not self._initializing and self._error is not None
                and self._error != "Waiting for radar frames (check sensorStart)"
            )

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _resolve_port(self) -> str | None:
        port_cfg = self.cfg.get("port", "auto")
        if port_cfg and port_cfg != "auto":
            return str(port_cfg)
        return detect_radar_port("03")

    def _resolve_cli_port(self) -> str | None:
        cli_cfg = self.cfg.get("cli_port", "auto")
        if cli_cfg and cli_cfg != "auto":
            return str(cli_cfg)
        return detect_radar_port("00")

    def _send_radar_cfg(self) -> tuple[bool, str]:
        """레이더가 정지 상태일 때 CLI 포트(-if00)로 cfg 및 sensorStart를 전송합니다."""
        import serial

        cli_port = self._resolve_cli_port()
        if not cli_port or (cli_port.startswith("/dev/") and not os.path.exists(cli_port)):
            return False, f"Radar CLI port not found ({cli_port or 'auto'})"

        cfg_path = Path(self.cfg["cfg_path"])

        if not cfg_path.exists():
            return False, f"Radar cfg file not found: {cfg_path}"

        cli_baud = int(self.cfg.get("cli_baud", 115200))
        data_baud = int(self.cfg.get("baud", 3125000))

        def _send_cmd(ser_cli: serial.Serial, cmd: str, timeout: float = 3.0) -> bool:
            ser_cli.reset_input_buffer()
            ser_cli.write((cmd + "\n").encode("ascii"))
            ser_cli.flush()
            resp = ""
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not self._stop.is_set():
                chunk = ser_cli.read(ser_cli.in_waiting or 1)
                if chunk:
                    resp += chunk.decode(errors="ignore")
                    if "Done" in resp and "mmwDemo:/>" in resp:
                        return True
                    if "Error" in resp and "mmwDemo:/>" in resp:
                        return False
            return "Done" in resp

        try:
            with serial.Serial(cli_port, cli_baud, timeout=0.05) as ser_cli:
                time.sleep(0.2)
                ser_cli.reset_input_buffer()
                ser_cli.reset_output_buffer()

                _send_cmd(ser_cli, "sensorStop", timeout=2.0)
                if not _send_cmd(ser_cli, f"configDataPort {data_baud} 1", timeout=2.0):
                    return False, "Radar configDataPort failed"

                for raw_line in cfg_path.read_text(encoding="utf-8").splitlines():
                    line = raw_line.strip()
                    if not line or line.startswith("%"):
                        continue
                    if line.startswith("sensorStart") or line.startswith("configDataPort") or line.startswith("sensorStop"):
                        continue
                    if not _send_cmd(ser_cli, line, timeout=3.0):
                        return False, f"Radar cfg command failed: {line}"

                if not _send_cmd(ser_cli, "sensorStart", timeout=5.0):
                    return False, "Radar sensorStart failed"
                return True, f"Started with {cfg_path.name}"
        except Exception as exc:
            return False, f"Radar init error: {type(exc).__name__}: {exc}"

    def _run_live(self) -> None:
        import serial

        baud = int(self.cfg.get("baud", 3125000))
        max_points = self.cfg.get("max_points", 256)

        while not self._stop.is_set():
            port_name = self._resolve_port()
            if not port_name or (port_name.startswith("/dev/") and not os.path.exists(port_name)):
                with self._lock:
                    self._error = f"Radar data port not found ({port_name or 'auto'})"
                self._stop.wait(2.0)
                continue

            try:
                with serial.Serial(port_name, baud, timeout=0.1) as ser:
                    buf = bytearray()
                    dropped = 0
                    init_attempted = False
                    opened_mono = time.monotonic()
                    with self._lock:
                        if self._sample is None:
                            self._error = "Waiting for radar frames (check sensorStart)"

                    while not self._stop.is_set():
                        chunk = ser.read(min(max(ser.in_waiting, 1), 65536))
                        if not chunk:
                            if (
                                not init_attempted
                                and self._sample is None
                                and bool(self.cfg.get("auto_init", True))
                                and (time.monotonic() - opened_mono) >= 0.4
                            ):
                                init_attempted = True
                                self._initializing = True
                                try:
                                    with self._lock:
                                        self._error = "Initializing radar cfg..."
                                    ok, msg = self._send_radar_cfg()
                                    with self._lock:
                                        if not ok and self._sample is None:
                                            self._error = msg
                                    ser.reset_input_buffer()
                                finally:
                                    self._initializing = False
                            continue
                        buf.extend(chunk)

                        while True:
                            idx = buf.find(reader.MAGIC)
                            if idx < 0:
                                if len(buf) > 7:
                                    del buf[:-7]
                                break
                            if idx > 0:
                                del buf[:idx]
                            if len(buf) < reader.HEADER_SIZE:
                                break

                            total_len = struct.unpack_from("<I", buf, 12)[0]
                            if not reader.HEADER_SIZE <= total_len <= 1_000_000:
                                del buf[0]
                                dropped += 1
                                continue
                            if len(buf) < total_len:
                                break

                            packet = bytes(buf[:total_len])
                            if not reader.packet_is_valid(packet):
                                del buf[0]
                                dropped += 1
                                continue

                            del buf[:total_len]
                            frame_num, num_obj, num_tlvs = reader.parse_header(packet)
                            xyzd, snr = reader.parse_packet(packet, num_obj, num_tlvs)

                            pts_list: list[list[float]] = []
                            snr_list: list[float | None] = []
                            for i in range(len(xyzd)):
                                row = xyzd[i]
                                vals = [float(row[0]), float(row[1]), float(row[2]), float(row[3])]
                                if not all(math.isfinite(v) for v in vals):
                                    continue
                                pts_list.append([round(v, 4) for v in vals])
                                s_val = float(snr[i]) if i < len(snr) and snr[i] is not None else math.nan
                                snr_list.append(round(s_val, 2) if math.isfinite(s_val) else None)

                            total_valid = len(pts_list)
                            if isinstance(max_points, int) and max_points > 0:
                                pts_list = pts_list[:max_points]
                                snr_list = snr_list[:max_points]

                            sample = {
                                "port": port_name,
                                "frame": int(frame_num),
                                "count": total_valid,
                                "points_xyzd": pts_list,
                                "snr_db": snr_list,
                                "dropped_packets": dropped,
                                "timestamp": time.time(),
                            }
                            with self._lock:
                                self._sample = sample
                                self._sample_mono = time.monotonic()
                                self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
                if not self._stop.is_set():
                    self._stop.wait(2.0)

    def _run_mock(self) -> None:
        frame = 0
        while not self._stop.is_set():
            frame += 1
            sample = {
                "port": "mock",
                "frame": frame,
                "count": 2,
                "points_xyzd": [
                    [0.0, 0.85, 0.05, 0.0],
                    [0.18, 1.20, 0.02, 0.0],
                ],
                "snr_db": [18.5, 14.2],
                "dropped_packets": 0,
                "timestamp": time.time(),
            }
            with self._lock:
                self._sample = sample
                self._sample_mono = time.monotonic()
                self._error = None
            self._stop.wait(0.1)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            sample = copy.deepcopy(self._sample)
            mono = self._sample_mono
            err = self._error

        if sample is None:
            return {
                "available": False,
                "frame": None,
                "count": 0,
                "points_xyzd": [],
                "snr_db": [],
                "timestamp": None,
                "age_s": None,
                "error": err or "No radar frame received yet",
            }

        return {
            "available": True,
            "frame": sample["frame"],
            "count": sample["count"],
            "points_xyzd": sample["points_xyzd"],
            "snr_db": sample["snr_db"],
            "dropped_packets": sample["dropped_packets"],
            "timestamp": sample["timestamp"],
            "age_s": round(time.monotonic() - mono, 4) if mono is not None else None,
            "error": err,
        }
