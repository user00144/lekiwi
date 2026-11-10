"""Click-to-go navigation on the radar BEV, with IMU verification.

Runs standalone on the Orin (no ZMQ server). From lekiwi_runtime/:

    uv run python tests/test_moving.py --check-axes   # 1) measure mounting constants below
    uv run python tests/test_moving.py --dry-run      # 2) simulate: wheels never energize
    uv run python tests/test_moving.py                # 3) drive

Radar: the AWR2944 data port is read and parsed directly with lekiwi_runtime.drivers.radar_protocol,
the same packet format exercised by tests/test_hardware.py (RadarStream is not used).

Plot (separate process; needs a display):
    left  = robot-relative BEV of the current radar frame, points colored by SNR
    right = global BEV, fixed x 0..4 m / y +/-4 m (origin = robot pose at start): radar map captured
            at start, current frame, odometry trajectory, IMU-only trajectory, goal
    Left-click               -> select the radar point nearest the click and go to it, stopping
                                --standoff (0.3 m) before it, facing it
    Left-drag (press-release) -> same point; final heading = drag direction
    Right-click              -> cancel the goal (stop)
    Clicking again while moving replaces the goal immediately. The title shows the selected
    point's current range and angle from the robot.
Terminal (also works with --no-plot): "x y [theta_deg]" global goal (no snapping), "c" cancel, "q" quit.

Navigation pose (default --pose-source odom): heading from the IMU, position from the commanded
wheel velocity integrated along that heading (no wheel encoders). The IMU-only position (double
integration, drifts within seconds) is drawn for comparison; --pose-source imu navigates on it.

Unit tests for the pure helpers (no hardware):
    uv run python -m unittest tests.test_moving -v

Robot frame: +x forward, +y left, +theta counter-clockwise seen from above.
Radar frame (TI): +y boresight (azimuth 0), +x right, azimuth = atan2(x, y).
"""

import argparse
from dataclasses import dataclass
import math
import multiprocessing as mp
from pathlib import Path
import queue as queue_module
import select
import struct
import sys
import threading
import time
import unittest

import numpy as np

from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers import radar_protocol as protocol
from lekiwi_runtime.drivers.imu import ImuStream
from lekiwi_runtime.drivers.radar import detect_radar_port
from lekiwi_runtime.drivers.servo import ServoBus
from lekiwi_runtime.drivers.wheels import GOAL_VELOCITY, WheelDrive, body_to_wheel_raw, valid_motion

# --- Mounting: confirm with --check-axes, then edit (or pass the CLI overrides). ---
RADAR_YAW_DEG = 0.0       # Radar boresight direction in the robot frame, CCW from +x (0 = faces forward).
RADAR_AZ_SIGN = 1         # +1: positive radar azimuth is to the robot's right (TI convention); -1 if mirrored.
RADAR_POS_M = (0.0, 0.0)  # Radar position in the robot frame (x forward, y left).
IMU_YAW_DEG = 0.0         # Direction of the IMU's levelled +x axis in the robot frame, CCW from +x.

# --- Timing and safety ---
LOOP_HZ = 50
PLOT_HZ = 10
IMU_ABORT_S = 0.5      # IMU snapshot unavailable/stale for this long -> abort.

# --- IMU verification tolerances (EMA-filtered command vs measurement) ---
EMA_TAU_S = 0.3
YAW_TOL_ABS = 8.0      # deg/s
YAW_TOL_REL = 0.3
VEL_TOL_ABS = 0.05     # m/s
VEL_TOL_REL = 0.4
YAW_MISMATCH_ABORT_S = 1.5

HELP = ('Left-click: nearest radar point | left-drag: + final heading | right-click: stop | '
        'terminal: "x y [theta]", "c", "q"')


# ----------------------------------------------------------------------------------------------
# Radar: AWR2944 data port parsed directly with radar_protocol (same packet format as
# tests/test_hardware.py: header + point TLV (x, y, z, doppler) + side-info TLV (SNR x10, noise)).
# ----------------------------------------------------------------------------------------------

def extract_packets(buf: bytearray) -> list[bytes]:
    """Remove and return every complete, valid packet in buf; keeps a trailing partial packet."""
    packets = []
    while True:
        index = buf.find(protocol.MAGIC)
        if index < 0:
            del buf[:-(len(protocol.MAGIC) - 1)]  # A magic word may straddle two reads.
            return packets
        del buf[:index]
        if len(buf) < protocol.HEADER_SIZE:
            return packets
        total = struct.unpack_from("<I", buf, 12)[0]
        if not protocol.HEADER_SIZE <= total <= 1_000_000:
            del buf[0]
            continue
        if len(buf) < total:
            return packets
        packet = bytes(buf[:total])
        if not protocol.packet_is_valid(packet):
            del buf[0]
            continue
        del buf[:total]
        packets.append(packet)


def decode_packet(packet: bytes) -> dict:
    frame, count, num_tlvs = protocol.parse_header(packet)
    points, snr = protocol.parse_packet(packet, count, num_tlvs)
    kept_points, kept_snr = [], []
    for point, value in zip(points, snr):
        if all(math.isfinite(v) for v in point):
            kept_points.append([float(v) for v in point])
            kept_snr.append(None if value is None else float(value))
    return {"frame": int(frame), "points_xyzd": kept_points, "snr_db": kept_snr}


def send_radar_cfg(cfg: dict) -> None:
    """sensorStop, configDataPort, the .cfg file, sensorStart over the CLI port (-if00)."""
    import serial

    port = cfg.get("cli_port") or "auto"
    port = detect_radar_port("00") if port == "auto" else port
    if not port:
        raise RuntimeError("Radar CLI port not found")
    lines = [line.strip() for line in Path(cfg["cfg_path"]).read_text(encoding="utf-8").splitlines()]
    body = [line for line in lines if line and not line.startswith("%")
            and line.split()[0] not in ("sensorStop", "sensorStart", "configDataPort")]
    commands = ["sensorStop", f"configDataPort {int(cfg.get('baud', 3125000))} 1", *body, "sensorStart"]
    with serial.Serial(port, int(cfg.get("cli_baud", 115200)), timeout=0.05) as cli:
        time.sleep(0.2)
        cli.reset_input_buffer()
        for command in commands:
            cli.reset_input_buffer()
            cli.write((command + "\n").encode("ascii"))
            reply, deadline = "", time.monotonic() + (5.0 if command == "sensorStart" else 3.0)
            while time.monotonic() < deadline and "mmwDemo:/>" not in reply:
                reply += cli.read(cli.in_waiting or 1).decode(errors="ignore")
            if "Done" not in reply and command != "sensorStop":
                raise RuntimeError(f"Radar cfg command failed: {command}")


class RadarReader:
    """Background reader: serial data port (-if03) -> extract_packets -> decode_packet."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sample = None
        self._sample_time = None
        self._error = "Not started" if cfg.get("enabled", True) else "disabled"

    def start(self) -> None:
        if self.cfg.get("enabled", True):
            self._thread = threading.Thread(target=self._run, name="radar", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> dict:
        with self._lock:
            sample, sample_time, error = self._sample, self._sample_time, self._error
        if sample is None:
            return {"available": False, "frame": None, "points_xyzd": [], "snr_db": [], "error": error}
        return {**sample, "available": True, "age_s": time.monotonic() - sample_time, "error": error}

    def _set_error(self, error: str | None) -> None:
        with self._lock:
            self._error = error

    def _run(self) -> None:
        import serial

        while not self._stop.is_set():
            port = self.cfg.get("port") or "auto"
            port = detect_radar_port("03") if port == "auto" else port
            if not port:
                self._set_error("Radar data port not found (XDS110 -if03)")
                self._stop.wait(2.0)
                continue
            try:
                with serial.Serial(port, int(self.cfg.get("baud", 3125000)), timeout=0.1) as data:
                    buf, opened, init_tried = bytearray(), time.monotonic(), False
                    self._set_error("Waiting for radar frames")
                    while not self._stop.is_set():
                        chunk = data.read(min(max(data.in_waiting, 1), 65536))
                        if not chunk:
                            if (not init_tried and self._sample is None and self.cfg.get("auto_init", True)
                                    and time.monotonic() - opened > 1.0):
                                init_tried = True  # The sensor is silent: configure and start it once.
                                self._set_error("Sending radar cfg...")
                                send_radar_cfg(self.cfg)
                                data.reset_input_buffer()
                                self._set_error("Waiting for radar frames")
                            continue
                        buf.extend(chunk)
                        for packet in extract_packets(buf):
                            sample = decode_packet(packet)
                            with self._lock:
                                self._sample, self._sample_time, self._error = sample, time.monotonic(), None
            except Exception as exc:
                self._set_error(f"{type(exc).__name__}: {exc}")
                self._stop.wait(2.0)


@dataclass
class NavParams:
    max_speed: float = 0.15          # m/s
    min_speed: float = 0.03          # m/s, so the last centimetres are still covered
    k_speed: float = 0.8             # (m/s) per m of distance
    max_theta: float = 45.0          # deg/s
    k_theta: float = 1.5             # (deg/s) per deg of heading error
    pos_tol: float = 0.05            # m
    yaw_tol: float = 5.0             # deg
    max_goal_distance: float = 3.0   # m, farther clicks are rejected
    standoff: float = 0.3            # m, stop this far before a selected radar point (it is an object surface)


@dataclass
class Goal:
    x: float      # global m
    y: float      # global m
    theta: float  # global deg


@dataclass
class Target:
    x: float
    y: float
    range_m: float
    azimuth_deg: float
    snr_db: float


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def rotate(v, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def transform(points, pose) -> np.ndarray:
    """Robot-frame points (N,2) -> global, for pose (x, y, yaw_deg)."""
    x, y, yaw = pose
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return np.asarray(points, dtype=float).reshape(-1, 2) @ np.array([[c, s], [-s, c]]) + [x, y]


def to_robot(points, pose) -> np.ndarray:
    """Global points (N,2) -> robot frame at pose (inverse of transform)."""
    x, y, yaw = pose
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (np.asarray(points, dtype=float).reshape(-1, 2) - [x, y]) @ np.array([[c, -s], [s, c]])


def make_goal(x: float, y: float, theta: float | None, pose) -> Goal:
    """Without a heading, face the direction of travel from the current pose."""
    if theta is None:
        dx, dy = x - pose[0], y - pose[1]
        theta = math.degrees(math.atan2(dy, dx)) if math.hypot(dx, dy) > 1e-6 else pose[2]
    return Goal(float(x), float(y), wrap_deg(float(theta)))


def make_point_goal(point, theta: float | None, pose, standoff: float) -> Goal:
    """Goal that faces a selected radar point and stops `standoff` before it.

    Without a heading, approach along the line from the robot; with one, end at that heading
    with the point straight ahead.
    """
    px, py = point
    if theta is None:
        dx, dy = px - pose[0], py - pose[1]
        dist = math.hypot(dx, dy)
        if dist <= max(standoff, 1e-6):  # Already within the standoff: just turn to face it.
            return make_goal(pose[0], pose[1], math.degrees(math.atan2(dy, dx)) if dist > 1e-6 else None, pose)
        theta = math.degrees(math.atan2(dy, dx))
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    return make_goal(px - standoff * c, py - standoff * s, theta, pose)


def point_bearing(point, pose) -> tuple[float, float]:
    """(range m, bearing deg, CCW/left +) of a global point seen from the robot pose."""
    x, y = to_robot([point], pose)[0]
    return float(math.hypot(x, y)), float(math.degrees(math.atan2(y, x)))


def goto_command(pose, goal: Goal, p: NavParams) -> tuple[float, float, float, bool]:
    """Holonomic go-to-pose: (x m/s, y m/s, theta deg/s in the robot frame, arrived)."""
    dx, dy = goal.x - pose[0], goal.y - pose[1]
    dist = math.hypot(dx, dy)
    heading_error = wrap_deg(goal.theta - pose[2])
    if dist <= p.pos_tol and abs(heading_error) <= p.yaw_tol:
        return 0.0, 0.0, 0.0, True
    theta = clamp(p.k_theta * heading_error, p.max_theta)
    if dist <= p.pos_tol:
        return 0.0, 0.0, theta, False
    speed = max(p.min_speed, min(p.max_speed, p.k_speed * dist))
    bx, by = rotate((dx / dist * speed, dy / dist * speed), -pose[2])
    return float(bx), float(by), theta, False


def parse_command(line: str):
    """Terminal command -> ('goal', x, y, theta|None) | ('cancel',) | ('quit',) | None."""
    parts = line.split()
    if not parts:
        return None
    if parts[0].lower() in ("q", "quit", "exit"):
        return ("quit",)
    if parts[0].lower() in ("c", "cancel", "s", "stop"):
        return ("cancel",)
    try:
        values = [float(v) for v in parts]
    except ValueError:
        values = []
    if len(values) not in (2, 3) or not all(math.isfinite(v) for v in values):
        print(f"Ignored {line.strip()!r}. Usage: x y [theta_deg] | c | q", flush=True)
        return None
    return ("goal", values[0], values[1], values[2] if len(values) == 3 else None)


def poll_terminal() -> list:
    events = []
    try:
        while select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:  # EOF (stdin not a terminal)
                break
            command = parse_command(line)
            if command:
                events.append(command)
    except (OSError, ValueError):
        pass
    return events


def radar_xy_to_robot(xy, radar_yaw_deg: float = RADAR_YAW_DEG, az_sign: int = RADAR_AZ_SIGN,
                      offset=RADAR_POS_M) -> np.ndarray:
    """TI radar x/y (N,2) -> robot-frame x forward / y left (N,2)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    rng = np.hypot(xy[:, 0], xy[:, 1])
    bearing = math.radians(radar_yaw_deg) - az_sign * np.arctan2(xy[:, 0], xy[:, 1])
    return np.column_stack([offset[0] + rng * np.cos(bearing), offset[1] + rng * np.sin(bearing)])


def radar_cloud(radar: dict, radar_yaw_deg: float, az_sign: int) -> tuple[np.ndarray, np.ndarray]:
    """Robot-frame points (N,2) and SNR dB (N,), NaN where the radar sent no SNR."""
    points = radar.get("points_xyzd") or []
    snr = list(radar.get("snr_db") or [])
    snr = [np.nan if s is None else float(s) for s in snr[:len(points)]] + [np.nan] * (len(points) - len(snr))
    return radar_xy_to_robot([p[:2] for p in points], radar_yaw_deg, az_sign), np.asarray(snr, dtype=float)


def strongest_point(radar: dict, min_range=0.2, max_range=4.0, max_az=60.0, min_snr=10.0) -> Target | None:
    """Highest-SNR point within limits (used by --check-axes)."""
    best = None
    for point, snr in zip(radar.get("points_xyzd") or [], radar.get("snr_db") or []):
        if snr is None:
            continue
        x, y = float(point[0]), float(point[1])
        rng, az = math.hypot(x, y), math.degrees(math.atan2(x, y))
        if min_range <= rng <= max_range and abs(az) <= max_az and snr >= min_snr:
            if best is None or snr > best.snr_db:
                best = Target(x, y, rng, az, float(snr))
    return best


def yaw_rate(imu: dict, up: np.ndarray) -> float:
    """Rotation rate about vertical (CCW +), independent of how the IMU is mounted."""
    return float(np.dot(imu["gyro_corrected_deg_s"], up))


def levelled_velocity(imu: dict, v0: np.ndarray) -> np.ndarray:
    """IMU velocity since baseline, in the IMU's current levelled heading frame."""
    return rotate(np.asarray(imu["velocity_m_s"][:2]) - v0, -imu["orientation_deg"][2])


def body_velocity(imu: dict, v0: np.ndarray, imu_yaw_deg: float) -> np.ndarray:
    return rotate(levelled_velocity(imu, v0), imu_yaw_deg)


def velocity_valid(imu: dict) -> bool:
    return bool(imu.get("estimate_valid")) and imu.get("velocity_m_s") is not None


class OdomPose:
    """Navigation pose in the global frame (robot pose at start = origin).

    Heading: IMU orientation while its estimate is valid, then the integrated gyro. With
    simulate=True (dry run) the commanded rate is integrated, so the plot simulates the controller.
    Position: commanded body velocity integrated along that heading (no wheel encoders).
    """

    def __init__(self, imu: dict, up: np.ndarray, simulate: bool):
        self.up, self.simulate = up, simulate
        self.x = self.y = self.yaw = 0.0
        self.yaw0 = imu["orientation_deg"][2] if velocity_valid(imu) else None

    def update(self, imu: dict, command, dt: float) -> tuple[float, float, float]:
        previous = self.yaw
        if self.simulate:
            self.yaw = wrap_deg(self.yaw + command[2] * dt)
        elif self.yaw0 is not None and velocity_valid(imu):
            self.yaw = wrap_deg(imu["orientation_deg"][2] - self.yaw0)
        else:
            self.yaw0 = None  # The IMU estimate never recovers once invalid; keep integrating the gyro.
            self.yaw = wrap_deg(self.yaw + yaw_rate(imu, self.up) * dt)
        vx, vy = rotate(command[:2], previous + wrap_deg(self.yaw - previous) / 2)
        self.x += vx * dt
        self.y += vy * dt
        return self.x, self.y, self.yaw


class ImuPose:
    """Global pose from IMU displacement only (double integration, for comparison).

    The IMU velocity at start should be zero; its residual is treated as constant drift and removed.
    """

    def __init__(self, imu: dict, imu_yaw_deg: float, t0: float):
        self.imu_yaw_deg, self.t0 = imu_yaw_deg, t0
        self.d0 = self.v0 = self.yaw0 = None
        if velocity_valid(imu) and imu.get("displacement_m") is not None:
            self.d0 = np.asarray(imu["displacement_m"][:2])
            self.v0 = np.asarray(imu["velocity_m_s"][:2])
            self.yaw0 = imu["orientation_deg"][2]

    def pose(self, imu: dict, now: float) -> tuple[float, float, float] | None:
        if self.d0 is None or not velocity_valid(imu) or imu.get("displacement_m") is None:
            return None
        rel = np.asarray(imu["displacement_m"][:2]) - self.d0 - self.v0 * (now - self.t0)
        x, y = rotate(rotate(rel, -self.yaw0), self.imu_yaw_deg)
        return float(x), float(y), wrap_deg(imu["orientation_deg"][2] - self.yaw0)


class ImuVerifier:
    """Compares commanded body velocity with IMU yaw rate and integrated velocity."""

    def __init__(self, imu: dict, up: np.ndarray, imu_yaw_deg: float):
        self.up, self.imu_yaw_deg = up, imu_yaw_deg
        self.v0 = np.asarray(imu["velocity_m_s"][:2]) if velocity_valid(imu) else None
        self.cmd = np.zeros(3)
        self.meas = np.zeros(3)
        self.last = None
        self.yaw_bad_since = None
        self.n = self.n_yaw_ok = self.n_vel = self.n_vel_ok = 0
        self.yaw_err_sq = self.vel_err_sq = 0.0

    def update(self, imu: dict, command, now: float) -> dict:
        dt = 0.0 if self.last is None else now - self.last
        self.last = now
        alpha = 1 - math.exp(-dt / EMA_TAU_S) if dt > 0 else 1.0
        meas = self.meas.copy()
        meas[2] = yaw_rate(imu, self.up)
        vel_ok_input = self.v0 is not None and velocity_valid(imu)
        if vel_ok_input:
            meas[:2] = body_velocity(imu, self.v0, self.imu_yaw_deg)
        self.cmd += alpha * (np.asarray(command[:3], dtype=float) - self.cmd)
        self.meas += alpha * (meas - self.meas)

        yaw_err = float(self.meas[2] - self.cmd[2])
        yaw_ok = abs(yaw_err) <= YAW_TOL_ABS + YAW_TOL_REL * abs(self.cmd[2])
        vel_err = float(np.linalg.norm(self.meas[:2] - self.cmd[:2]))
        vel_ok = (vel_err <= VEL_TOL_ABS + VEL_TOL_REL * float(np.linalg.norm(self.cmd[:2]))
                  if vel_ok_input else None)
        self.yaw_bad_since = None if yaw_ok else (self.yaw_bad_since or now)

        self.n += 1
        self.n_yaw_ok += yaw_ok
        self.yaw_err_sq += yaw_err ** 2
        if vel_ok is not None:
            self.n_vel += 1
            self.n_vel_ok += vel_ok
            self.vel_err_sq += vel_err ** 2
        return dict(yaw_rate=float(self.meas[2]), yaw_err=yaw_err, yaw_ok=yaw_ok,
                    velocity=self.meas[:2].copy(), vel_err=vel_err, vel_ok=vel_ok,
                    yaw_bad_for=0.0 if yaw_ok else now - self.yaw_bad_since)

    def summary(self) -> str:
        if not self.n:
            return "IMU verification: no samples"
        text = (f"IMU verification: yaw rate within tolerance {100 * self.n_yaw_ok / self.n:.0f}% "
                f"(RMS error {math.sqrt(self.yaw_err_sq / self.n):.1f} deg/s)")
        if self.n_vel:
            text += (f"; velocity within tolerance {100 * self.n_vel_ok / self.n_vel:.0f}% "
                     f"(RMS error {math.sqrt(self.vel_err_sq / self.n_vel):.3f} m/s, drift-prone)")
        else:
            text += "; velocity not checked (IMU estimate invalid)"
        return text


class Base:
    """Continuous wheel velocity in one torque session; stop() zeroes and releases the wheels."""

    def __init__(self, wheels: WheelDrive, dry_run: bool):
        self.wheels, self.dry_run = wheels, dry_run
        self.active = False

    def send(self, x: float, y: float, theta: float) -> None:
        x, y, theta = float(x), float(y), float(theta)
        if not valid_motion(dict(x=x, y=y, theta=theta, duration_ms=1)):
            raise ValueError(f"Invalid body velocity x={x} y={y} theta={theta}")
        if self.dry_run:
            return
        import scservo_sdk as sdk

        if not self.active:
            self.wheels._prepare()  # Velocity mode, goal 0, torque on.
            self.active = True
        bus = self.wheels.bus
        writer = sdk.GroupSyncWrite(bus.port_handler, bus.packet_handler, GOAL_VELOCITY, 2)
        for motor_id, value in body_to_wheel_raw(x, y, theta).items():
            if not writer.addParam(motor_id, [value & 0xFF, value >> 8]):
                raise OSError(f"Cannot build wheel command for {motor_id}")
        if writer.txPacket() != sdk.COMM_SUCCESS:
            raise OSError("Wheel velocity transmission failed")

    def stop(self) -> None:
        if self.active:
            self.active = False
            self.wheels.close(release_all=True)


# ----------------------------------------------------------------------------------------------
# Live plot (runs in its own process so rendering never stalls the IMU/radar threads)
# ----------------------------------------------------------------------------------------------

ROBOT_SHAPE = np.array([[0.18, 0.0], [-0.10, 0.10], [-0.10, -0.10]])  # Triangle pointing forward.
NON_INTERACTIVE_BACKENDS = {"agg", "pdf", "pgf", "ps", "svg", "cairo", "template"}
EMPTY = np.empty((0, 2))
MIN_DRAG_M = 0.1  # Shorter drags count as a plain click.


def bev(points) -> np.ndarray:
    """x forward / y left -> screen (horizontal = y, vertical = x); axes invert horizontal so left is left."""
    return np.asarray(points, dtype=float).reshape(-1, 2)[:, ::-1]


class PlotView:
    def __init__(self, fig, settings: dict, emit=None):
        import matplotlib
        from matplotlib.lines import Line2D
        from matplotlib.patches import Polygon

        self.fig, self.emit = fig, emit or (lambda _msg: None)
        self.ax_rel, self.ax_glob = fig.subplots(1, 2, gridspec_kw=dict(
            left=0.05, right=0.96, bottom=0.2, top=0.84, wspace=0.12))
        below = dict(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)  # Keep points visible.
        r = settings["plot_range"]
        for ax, title in ((self.ax_rel, "Robot-relative BEV (current radar frame)"),
                          (self.ax_glob, "Global BEV: radar map at start + trajectory")):
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("y [m]  (left +)")
            ax.set_ylabel("x [m]  (forward +)")
            ax.set_title(title)
        self.ax_rel.set_xlim(r, -r)
        self.ax_rel.set_ylim(-0.5, r + 0.5)
        # Fixed global extent (origin = robot pose at start); the trajectory is drawn over the map.
        self.ax_glob.set_xlim(settings["map_y"], -settings["map_y"])
        self.ax_glob.set_ylim(0.0, settings["map_x"])

        cmap = matplotlib.colormaps["viridis"].with_extremes(bad="0.6")
        color = dict(cmap=cmap, vmin=settings["snr_min"], vmax=settings["snr_max"])

        def goal_artists(ax):
            marker = ax.scatter([], [], s=180, c="magenta", marker="X", zorder=7, label="goal")
            (heading,) = ax.plot([], [], "-", c="magenta", lw=2.5, zorder=7)
            (path,) = ax.plot([], [], ":", c="magenta", lw=1.2, zorder=3)
            (drag,) = ax.plot([], [], "-", c="magenta", lw=1.5, alpha=0.6, zorder=8)
            selected = ax.scatter([], [], s=320, facecolors="none", edgecolors="red", linewidths=2, zorder=8,
                                  label="selected radar point")
            return marker, heading, path, drag, selected

        def snr_handle(label, marker="o"):  # Legend proxy: SNR scatters start empty and have no colour yet.
            return Line2D([], [], ls="", marker=marker, color=cmap(0.6), label=label)

        ax = self.ax_rel
        ax.add_patch(Polygon(bev(ROBOT_SHAPE), closed=True, color="black", zorder=4))
        self.rel_points = ax.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=12, **color)
        self.rel_goal, self.rel_heading, self.rel_path, self.rel_drag, self.rel_selected = goal_artists(ax)
        ax.legend(handles=[snr_handle("radar points (color = SNR)"), self.rel_selected, self.rel_goal],
                  **below)

        ax = self.ax_glob
        self.map_points = ax.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=10, marker="s", alpha=0.45,
                                     **color)
        self.glob_points = ax.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=12, alpha=0.9, **color)
        (self.odom_traj,) = ax.plot([], [], "-", c="tab:green", lw=2, label="odometry (IMU yaw + wheel cmd)")
        (self.imu_traj,) = ax.plot([], [], "--", c="tab:purple", lw=1.2, label="IMU only (double integration)")
        self.robot = Polygon(bev(ROBOT_SHAPE), closed=True, color="tab:green", zorder=6)
        ax.add_patch(self.robot)
        self.glob_goal, self.glob_heading, self.glob_path, self.glob_drag, self.glob_selected = goal_artists(ax)
        start = ax.scatter([0], [0], marker="x", c="black", zorder=6, clip_on=False, label="start")
        ax.legend(handles=[snr_handle("initial radar map", "s"), snr_handle("current frame"), self.odom_traj,
                           self.imu_traj, self.glob_selected, self.glob_goal, start], **below)
        fig.colorbar(self.glob_points, ax=self.ax_glob, fraction=0.04, pad=0.02, shrink=0.8, label="SNR [dB]")

        self.title = fig.suptitle("Waiting for radar map...")
        self.info = fig.text(0.005, 0.005, "", fontsize=8, color="0.4")
        # Blitting: the static background (axes, grid, legends, map) is rendered once and cached;
        # each frame only restores it and redraws these artists.
        self.dynamic = [self.rel_points, self.glob_points, self.odom_traj, self.imu_traj, self.robot,
                        self.rel_path, self.rel_goal, self.rel_heading, self.rel_selected, self.rel_drag,
                        self.glob_path, self.glob_goal, self.glob_heading, self.glob_selected, self.glob_drag,
                        self.title, self.info]
        for artist in self.dynamic:
            artist.set_animated(True)
        self.background = None
        self.needs_full_draw = True
        canvas = fig.canvas
        canvas.mpl_connect("draw_event", self._on_draw)
        canvas.mpl_connect("button_press_event", self.on_press)
        canvas.mpl_connect("motion_notify_event", self.on_motion)
        canvas.mpl_connect("button_release_event", self.on_release)

        self.pose = (0.0, 0.0, 0.0)
        self.goal = None
        self.selected = None
        self.map_xy, self.map_snr = EMPTY, np.empty(0)        # Global (= robot frame at start).
        self.points, self.snr = EMPTY, np.empty(0)            # Current frame, robot frame.
        self.odom_xy, self.imu_xy = [], []
        self.press = None
        self.finished = False
        self.latest_sent = None
        self.last_draw = None
        self.fps = 0.0
        self.latency = None

    # --- rendering ---

    def _on_draw(self, _event) -> None:
        """Full redraw (first show, resize, zoom, limit change): re-cache background, overlay dynamic artists."""
        canvas = self.fig.canvas
        if canvas.supports_blit:
            self.background = canvas.copy_from_bbox(self.fig.bbox)
        self._draw_dynamic()
        self.note_draw(time.time())

    def _draw_dynamic(self) -> None:
        for artist in self.dynamic:
            self.fig.draw_artist(artist)

    def render(self) -> None:
        canvas = self.fig.canvas
        if self.needs_full_draw or self.background is None or not canvas.supports_blit:
            self.needs_full_draw = False
            canvas.draw()  # Triggers _on_draw.
        else:
            canvas.restore_region(self.background)
            self._draw_dynamic()
            canvas.blit(self.fig.bbox)
            self.note_draw(time.time())
        canvas.flush_events()

    def note_draw(self, now: float) -> None:
        """Called after each canvas draw; the numbers appear on the next frame."""
        if self.last_draw is not None and now > self.last_draw:
            self.fps = 0.8 * self.fps + 0.2 / (now - self.last_draw) if self.fps else 1 / (now - self.last_draw)
        self.last_draw = now
        if self.latest_sent is not None:
            self.latency = now - self.latest_sent

    # --- data ---

    def handle(self, msg) -> None:
        if msg is None:
            if not self.finished:
                self.finished = True
                self.title.set_text(self.title.get_text() + "   [finished]")
            return
        if msg["type"] == "map":
            self.map_xy = np.asarray(msg["points"], dtype=float).reshape(-1, 2)
            self.map_snr = np.asarray(msg["snr"], dtype=float)
            self.map_points.set_offsets(bev(self.map_xy))
            self.map_points.set_array(self.map_snr)
            self.needs_full_draw = True  # The map is part of the cached background.
            return
        self.points = np.asarray(msg["points"], dtype=float).reshape(-1, 2)
        self.snr = np.asarray(msg["snr"], dtype=float)
        if msg.get("pose") is not None:
            self.pose = tuple(msg["pose"])
        if msg.get("odom") is not None:
            self.odom_xy.append(msg["odom"][:2])
        if msg.get("imu") is not None:
            self.imu_xy.append(msg["imu"][:2])
        self.goal = msg.get("goal")
        self.selected = msg.get("selected")
        self.rel_points.set_offsets(bev(self.points))
        self.rel_points.set_array(self.snr)
        self.glob_points.set_offsets(bev(transform(self.points, self.pose)))
        self.glob_points.set_array(self.snr)
        self.odom_traj.set_data(*bev(self.odom_xy).T)
        self.imu_traj.set_data(*bev(self.imu_xy).T)
        self.robot.set_xy(bev(transform(ROBOT_SHAPE, self.pose)))
        self._show_goal()
        self.title.set_text(msg.get("text", ""))
        self.latest_sent = msg.get("sent", self.latest_sent)
        latency = "n/a" if self.latency is None else f"{self.latency * 1000:.0f} ms"
        self.info.set_text(f"plot {self.fps:.1f} fps, latency {latency}")

    def _show_goal(self) -> None:
        sets = ((self.glob_goal, self.glob_heading, self.glob_path, self.glob_selected, lambda pts: pts,
                 self.pose[:2]),
                (self.rel_goal, self.rel_heading, self.rel_path, self.rel_selected,
                 lambda pts: to_robot(pts, self.pose), (0, 0)))
        for marker, heading, path, selected, frame, robot_xy in sets:
            selected.set_offsets(EMPTY if self.selected is None else bev(frame([self.selected[:2]])))
            if self.goal is None:
                marker.set_offsets(EMPTY)
                heading.set_data([], [])
                path.set_data([], [])
                continue
            gx, gy, gtheta = self.goal
            tip = (gx + 0.3 * math.cos(math.radians(gtheta)), gy + 0.3 * math.sin(math.radians(gtheta)))
            goal_xy, tip_xy = frame([(gx, gy), tip])
            marker.set_offsets(bev([goal_xy]))
            heading.set_data(*bev([goal_xy, tip_xy]).T)
            path.set_data(*bev([robot_xy, goal_xy]).T)

    def nearest_point(self, xy) -> tuple[float, float, float] | None:
        """Nearest radar point (global x, y, SNR) to a global position: initial map + current frame."""
        candidates = np.vstack([self.map_xy, transform(self.points, self.pose)])
        snr = np.concatenate([self.map_snr, self.snr])
        if not len(candidates):
            return None
        index = int(np.argmin(np.hypot(*(candidates - xy).T)))
        return float(candidates[index, 0]), float(candidates[index, 1]), float(snr[index])

    # --- mouse goals ---

    def _clickable(self, event) -> bool:
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar is not None and toolbar.mode:  # Zoom/pan active: leave the mouse to the toolbar.
            return False
        return event.inaxes in (self.ax_rel, self.ax_glob) and event.xdata is not None

    def _to_global(self, ax, sx: float, sy: float) -> np.ndarray:
        point = np.array([sy, sx])  # Screen (y, x) -> (x, y).
        return transform([point], self.pose)[0] if ax is self.ax_rel else point

    def _drag_line(self, ax):
        return self.rel_drag if ax is self.ax_rel else self.glob_drag

    def on_press(self, event) -> None:
        if not self._clickable(event):
            return
        if event.button == 3:
            self.emit(("cancel",))
        elif event.button == 1:
            self.press = (event.inaxes, event.xdata, event.ydata)

    def on_motion(self, event) -> None:
        if self.press is None or event.inaxes is not self.press[0] or event.xdata is None:
            return
        ax, sx, sy = self.press
        self._drag_line(ax).set_data([sx, event.xdata], [sy, event.ydata])
        self.render()

    def on_release(self, event) -> None:
        if self.press is None or event.button != 1:
            return
        ax, sx, sy = self.press
        self.press = None
        self._drag_line(ax).set_data([], [])
        start = self._to_global(ax, sx, sy)
        theta = None
        if event.inaxes is ax and event.xdata is not None:
            drag = self._to_global(ax, event.xdata, event.ydata) - start
            if np.hypot(*drag) >= MIN_DRAG_M:
                theta = math.degrees(math.atan2(drag[1], drag[0]))
        point = self.nearest_point(start)
        if point is None:
            self.info.set_text("No radar points to select")
        else:
            self.emit(("point", *point, theta))
        self.render()


def plot_worker(messages, goals, settings: dict, shown) -> None:
    import signal

    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl+C stops the robot in the main process only.
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        print("Plot disabled: matplotlib not installed (run `uv sync`)", flush=True)
        return
    backend = plt.get_backend()
    if backend.lower() in NON_INTERACTIVE_BACKENDS:
        print("Plot disabled: no display (run on the Orin desktop or set DISPLAY)", flush=True)
        return
    matplotlib.rcParams["figure.raise_window"] = False  # Do not steal focus from the terminal.
    fig = plt.figure(figsize=(15, 7.5))
    view = PlotView(fig, settings, emit=goals.put)
    try:
        fig.canvas.manager.set_window_title("LeKiwi click-to-go")
    except AttributeError:
        pass
    plt.show(block=False)
    print(f"Plot window open (backend {backend})", flush=True)
    shown.set()
    while plt.fignum_exists(fig.number):
        changed = False
        while True:  # Drain everything queued; only the newest frame is drawn.
            try:
                msg = messages.get_nowait()
            except queue_module.Empty:
                break
            except (EOFError, OSError):
                msg = None
            view.handle(msg)
            changed = True
        if changed:
            view.render()
        fig.canvas.start_event_loop(0.03)  # Unlike plt.pause, does not re-show/raise the window.


class Plotter:
    def __init__(self, settings: dict, enabled: bool):
        self.queue = self.goals = self.process = self.shown = None
        if not enabled:
            return
        ctx = mp.get_context("spawn")  # Never fork a process that owns serial ports and threads.
        self.queue = ctx.Queue(maxsize=20)
        self.goals = ctx.Queue()
        self.shown = ctx.Event()
        self.process = ctx.Process(target=plot_worker, args=(self.queue, self.goals, settings, self.shown),
                                   daemon=True, name="plot")
        self.process.start()

    @property
    def closed(self) -> bool:
        """The window was shown and the user has closed it."""
        return self.shown is not None and self.shown.is_set() and not self.process.is_alive()

    def poll(self) -> list:
        events = []
        while self.goals is not None:
            try:
                events.append(self.goals.get_nowait())
            except (queue_module.Empty, EOFError, OSError):
                break
        return events

    def send(self, msg, block: bool = False) -> None:
        if self.process is None or not self.process.is_alive():
            return
        try:
            self.queue.put(msg, block=block, timeout=1.0 if block else None)
        except queue_module.Full:
            pass  # Drop a frame rather than slow the control loop.

    def finish(self) -> None:
        if self.process is None:
            return
        self.send(None, block=True)
        if self.shown.is_set() and self.process.is_alive():
            print("Run finished. Close the plot window to exit.", flush=True)
        try:
            self.process.join()
        except KeyboardInterrupt:
            self.process.terminate()


# ----------------------------------------------------------------------------------------------
# Sensors and runs
# ----------------------------------------------------------------------------------------------

def wait_sensors(imu: ImuStream, radar: RadarReader, imu_timeout=90.0, radar_timeout=30.0) -> None:
    print("Keep the robot still: IMU calibrating (retries until stationary)...", flush=True)
    deadline = time.monotonic() + imu_timeout
    while True:
        state = imu.snapshot()
        if state.get("calibrated") and state.get("available") and state.get("gyro_corrected_deg_s") is not None:
            break
        if imu.ready and not state.get("calibrated"):
            raise RuntimeError(f"IMU unavailable: {state.get('error')}")
        if time.monotonic() > deadline:
            raise TimeoutError(f"IMU not calibrated within {imu_timeout:.0f} s: {state.get('error')}")
        time.sleep(0.05)
    print("IMU ready. Waiting for radar frames...", flush=True)
    deadline = time.monotonic() + radar_timeout
    last_error = None
    while True:
        state = radar.snapshot()
        if state["available"]:
            break
        if state.get("error") == "disabled":
            raise RuntimeError("Radar is disabled in the configuration")
        if state.get("error") != last_error:
            last_error = state.get("error")
            print(f"  radar: {last_error}", flush=True)
        if time.monotonic() > deadline:
            raise TimeoutError(f"No radar frame within {radar_timeout:.0f} s: {state.get('error')}")
        time.sleep(0.05)
    if state["points_xyzd"] and not any(s is not None for s in state["snr_db"]):
        print("Warning: radar frames carry no SNR (side-info TLV disabled?); points will be grey.")
    print("Radar ready.", flush=True)


def gravity_up(imu: ImuStream, samples: int = 20) -> np.ndarray:
    """Unit 'up' vector in the IMU frame from the stationary accelerometer reading."""
    accel = []
    for _ in range(samples):
        accel.append(imu.snapshot()["accel_m_s2"])
        time.sleep(0.01)
    mean = np.mean(accel, axis=0)
    return mean / np.linalg.norm(mean)


def capture_map(radar: RadarReader, seconds: float, radar_yaw_deg: float, az_sign: int):
    """Robot-frame points and SNR of the frames received during `seconds` (at least one frame)."""
    clouds, last_frame = [], None
    end = time.monotonic() + seconds
    while not clouds or time.monotonic() < end:
        state = radar.snapshot()
        if state["available"] and state["frame"] != last_frame:
            last_frame = state["frame"]
            clouds.append(radar_cloud(state, radar_yaw_deg, az_sign))
        time.sleep(0.02)
    return np.vstack([c[0] for c in clouds]), np.concatenate([c[1] for c in clouds])


def average_target(radar: RadarReader, seconds: float = 1.0) -> Target | None:
    """Median of per-frame strongest points over a short window."""
    found, last_frame = [], None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state = radar.snapshot()
        if state["available"] and state["frame"] != last_frame:
            last_frame = state["frame"]
            target = strongest_point(state)
            if target:
                found.append(target)
        time.sleep(0.02)
    if not found:
        return None
    med = lambda attr: float(np.median([getattr(t, attr) for t in found]))
    return Target(med("x"), med("y"), med("range_m"), med("azimuth_deg"), med("snr_db"))


def describe(target: Target | None) -> str:
    if target is None:
        return "no target"
    return f"range={target.range_m:.2f} m azimuth={target.azimuth_deg:+.1f} deg snr={target.snr_db:.1f} dB"


def fmt_pose(pose) -> str:
    return "n/a" if pose is None else f"x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={pose[2]:+.0f}"


def navigate(imu: ImuStream, radar: RadarReader, base: Base, p: NavParams, args, plotter: Plotter) -> None:
    up = gravity_up(imu)
    map_points, map_snr = capture_map(radar, args.map_seconds, args.radar_yaw, args.radar_az_sign)
    plotter.send({"type": "map", "points": map_points, "snr": map_snr}, block=True)
    print(f"Global map: {len(map_points)} points from the first {args.map_seconds:g} s of radar frames")
    print(f"Mounting: radar_yaw={args.radar_yaw} deg, radar_az_sign={args.radar_az_sign}, imu_yaw={args.imu_yaw} deg"
          f" | pose source: {args.pose_source}{'  [DRY RUN: wheels stay off, motion simulated]' if args.dry_run else ''}")
    print(HELP, flush=True)

    start = prev = time.monotonic()
    first = imu.snapshot()
    odom = OdomPose(first, up, simulate=args.dry_run)
    imu_only = ImuPose(first, args.imu_yaw, start)
    verifier = ImuVerifier(first, up, args.imu_yaw)
    if imu_only.d0 is None:
        print("Warning: IMU position estimate invalid; IMU-only trajectory will not be drawn.")
    goal = selected = None
    command = (0.0, 0.0, 0.0)
    odom_pose, imu_pose, nav_pose = (0.0, 0.0, 0.0), None, (0.0, 0.0, 0.0)
    cloud = (EMPTY, np.empty(0))
    last_frame = last_plot = last_print = imu_bad_since = None
    travelled = 0.0
    status = None
    try:
        while True:
            now = time.monotonic()
            dt, prev = now - prev, now
            if args.max_time and now - start > args.max_time:
                print(f"Stopped: time limit {args.max_time:.0f} s")
                break
            if plotter.closed:
                print("Plot window closed")
                break

            imu_state = imu.snapshot()
            if not imu_state.get("available") or imu_state.get("gyro_corrected_deg_s") is None:
                command = (0.0, 0.0, 0.0)
                base.send(*command)
                imu_bad_since = imu_bad_since or now
                if now - imu_bad_since > IMU_ABORT_S:
                    raise RuntimeError(f"IMU unavailable: {imu_state.get('error')}")
                time.sleep(1 / LOOP_HZ)
                continue
            imu_bad_since = None
            odom_pose = odom.update(imu_state, command, dt)  # The command in effect over the last dt.
            imu_pose = imu_only.pose(imu_state, now)
            nav_pose = odom_pose if args.pose_source == "odom" else imu_pose
            travelled += math.hypot(command[0], command[1]) * dt
            if travelled > args.max_travel:
                print(f"Stopped: commanded travel limit {args.max_travel:.1f} m")
                break

            quit_requested = False
            for event in plotter.poll() + poll_terminal():
                if event[0] == "quit":
                    quit_requested = True
                elif event[0] == "cancel":
                    if goal is not None:
                        print("Goal cancelled; stopping", flush=True)
                    goal = selected = None
                elif nav_pose is None:
                    print("No valid pose; goal ignored", flush=True)
                else:
                    if event[0] == "point":  # Nearest radar point to the click: (x, y, snr, theta).
                        _, px, py, psnr, gtheta = event
                        new_goal = make_point_goal((px, py), gtheta, nav_pose, p.standoff)
                        rng, bearing = point_bearing((px, py), nav_pose)
                        print(f"Selected radar point x={px:+.2f} y={py:+.2f} (range {rng:.2f} m, angle "
                              f"{bearing:+.1f} deg, SNR {psnr:.1f} dB); stopping {p.standoff:.2f} m before it",
                              flush=True)
                    else:  # Terminal goal: (x, y, theta), used as is.
                        _, gx, gy, gtheta = event
                        new_goal, px, py, psnr = make_goal(gx, gy, gtheta, nav_pose), None, None, None
                    distance = math.hypot(new_goal.x - nav_pose[0], new_goal.y - nav_pose[1])
                    if distance > p.max_goal_distance:
                        print(f"Goal {distance:.2f} m away rejected (max {p.max_goal_distance} m)", flush=True)
                        continue
                    goal = new_goal
                    selected = None if px is None else (px, py, psnr)
                    print(f"New goal x={goal.x:+.2f} y={goal.y:+.2f} theta={goal.theta:+.0f} "
                          f"({distance:.2f} m from {fmt_pose(nav_pose)})", flush=True)
            if quit_requested:
                print("Quit requested")
                break

            command = (0.0, 0.0, 0.0)
            if goal is not None and nav_pose is not None:
                x, y, theta, arrived = goto_command(nav_pose, goal, p)
                if arrived:
                    print(f"Reached goal: {fmt_pose(nav_pose)}  (IMU only: {fmt_pose(imu_pose)})", flush=True)
                    goal = None
                else:
                    command = (x, y, theta)
            base.send(*command)
            status = verifier.update(imu_state, command, now)
            if not args.dry_run and status["yaw_bad_for"] > YAW_MISMATCH_ABORT_S:
                raise RuntimeError(f"IMU yaw rate {status['yaw_rate']:+.1f} deg/s does not follow "
                                   f"theta command {command[2]:+.1f} deg/s for {YAW_MISMATCH_ABORT_S} s")

            radar_state = radar.snapshot()
            if radar_state["available"] and radar_state["frame"] != last_frame:
                last_frame = radar_state["frame"]
                cloud = radar_cloud(radar_state, args.radar_yaw, args.radar_az_sign)
            if last_plot is None or now - last_plot >= 1 / PLOT_HZ:
                last_plot = now
                shown_pose = nav_pose or odom_pose
                if selected is None:
                    selected_text = "Selected point: none (left-click near a radar point)"
                else:
                    rng, bearing = point_bearing(selected[:2], shown_pose)
                    selected_text = (f"Selected point: range {rng:.2f} m, angle {bearing:+.1f} deg (left +), "
                                     f"SNR {selected[2]:.1f} dB  @ global ({selected[0]:+.2f}, {selected[1]:+.2f})")
                goal_text = ("no goal" if goal is None else
                             f"goal x={goal.x:+.2f} y={goal.y:+.2f} theta={goal.theta:+.0f} "
                             f"({math.hypot(goal.x - shown_pose[0], goal.y - shown_pose[1]):.2f} m)")
                text = (f"{selected_text}\npose {fmt_pose(nav_pose)}  |  {goal_text}  |  cmd x={command[0]:+.2f} "
                        f"y={command[1]:+.2f} th={command[2]:+.0f}  |  IMU yaw rate {status['yaw_rate']:+.0f} "
                        f"{'OK' if status['yaw_ok'] else 'MISMATCH'}\n{HELP}")
                plotter.send({"type": "frame", "points": cloud[0], "snr": cloud[1],
                              "pose": shown_pose, "odom": odom_pose, "imu": imu_pose,
                              "goal": None if goal is None else (goal.x, goal.y, goal.theta),
                              "selected": selected, "text": text, "sent": time.time()})
            if goal is not None and (last_print is None or now - last_print >= 0.5):
                last_print = now
                vel = status["velocity"]
                vel_tag = "n/a" if status["vel_ok"] is None else ("OK" if status["vel_ok"] else "MISMATCH")
                print(f"pose {fmt_pose(nav_pose)} | cmd x={command[0]:+.2f} y={command[1]:+.2f} "
                      f"th={command[2]:+5.1f} | imu th={status['yaw_rate']:+5.1f} "
                      f"{'OK' if status['yaw_ok'] else 'MISMATCH'} v=({vel[0]:+.2f},{vel[1]:+.2f}) {vel_tag}",
                      flush=True)
            time.sleep(max(0.0, 1 / LOOP_HZ - (time.monotonic() - now)))
        base.send(0, 0, 0)
    finally:
        print(verifier.summary())
        print(f"End pose  navigation: {fmt_pose(nav_pose)} | IMU only: {fmt_pose(imu_pose)}")


def drive_and_measure(imu: ImuStream, base: Base, up: np.ndarray, command, duration: float, settle: float = 0.5):
    """Drive for duration, then hold zero; return (integrated yaw deg, mean levelled IMU velocity or None)."""
    first = imu.snapshot()
    v0 = np.asarray(first["velocity_m_s"][:2]) if velocity_valid(first) else None
    yaw, velocities = 0.0, []
    start = prev = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= start + duration + settle:
            break
        moving = now < start + duration
        base.send(*(command if moving else (0, 0, 0)))
        state = imu.snapshot()
        if state.get("gyro_corrected_deg_s") is not None:
            yaw += yaw_rate(state, up) * (now - prev)
        prev = now
        if moving and v0 is not None and velocity_valid(state):
            velocities.append(levelled_velocity(state, v0))
        time.sleep(1 / LOOP_HZ)
    return yaw, (np.mean(velocities, axis=0) if velocities else None)


def check_axes(imu: ImuStream, radar: RadarReader, base: Base) -> None:
    up = gravity_up(imu)
    print(f"Gravity 'up' in IMU frame: {np.round(up, 3)}")
    print("Axis check. Place one strong reflector ~1 m directly in front of the robot; keep 0.5 m clear around.")
    input("Press Enter to measure the target... ")
    before = average_target(radar)
    print(f"Target (front): {describe(before)}  -> expect azimuth ~0 if the radar faces forward")

    input("Press Enter to rotate CCW at +30 deg/s for 1 s... ")
    yaw, _ = drive_and_measure(imu, base, up, (0, 0, 30), 1.0)
    after = average_target(radar)
    print(f"IMU yaw change: {yaw:+.1f} deg (expect about +30)")
    if yaw < -5:
        print("  -> Rotation sign reversed: wheel kinematics or wheel IDs differ from the code.")
    elif abs(yaw) > 5:
        print(f"  -> Measured/commanded ratio {yaw / 30:.2f} (1.0 = kinematics radius matches).")
    print(f"Target after rotation: {describe(after)}")
    if before and after and abs(after.azimuth_deg - before.azimuth_deg) > 10 and yaw > 5:
        sign = 1 if after.azimuth_deg > before.azimuth_deg else -1
        print(f"  -> Suggested RADAR_AZ_SIGN = {sign}")
    drive_and_measure(imu, base, up, (0, 0, -30), 1.0)

    input("Press Enter to drive forward at 0.10 m/s for 1 s... ")
    before = average_target(radar)
    _, velocity = drive_and_measure(imu, base, up, (0.10, 0, 0), 1.0)
    after = average_target(radar)
    if velocity is not None and np.linalg.norm(velocity) > 0.02:
        phi = math.degrees(math.atan2(velocity[1], velocity[0]))
        print(f"IMU velocity direction during forward drive: {phi:+.0f} deg in the IMU frame "
              f"(|v|={np.linalg.norm(velocity):.2f} m/s, expect ~0.10)")
        print(f"  -> Suggested IMU_YAW_DEG = {-phi:+.0f} (nearest 90: {round(-phi / 90) * 90 % 360:.0f})")
    else:
        print("IMU velocity too small or estimate invalid; cannot infer IMU_YAW_DEG.")
    if before and after:
        print(f"Radar range change: {after.range_m - before.range_m:+.3f} m "
              "(about -0.10 if the radar faces forward and the target is ahead)")
    drive_and_measure(imu, base, up, (-0.10, 0, 0), 1.0)
    base.send(0, 0, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Click-to-go on the radar BEV; motion verified with the IMU.")
    parser.add_argument("--check-axes", action="store_true", help="measure radar/IMU mounting with short moves")
    parser.add_argument("--dry-run", action="store_true",
                        help="never energize wheels; the plot simulates the commanded motion")
    parser.add_argument("--no-plot", action="store_true", help="no plot window; goals from the terminal only")
    parser.add_argument("--pose-source", choices=("odom", "imu"), default="odom",
                        help="odom = IMU heading + commanded velocity (default); imu = IMU double integration")
    parser.add_argument("--map-seconds", type=float, default=1.0,
                        help="radar frames accumulated at start as the global map (0 = first frame only)")
    parser.add_argument("--config", type=Path, help="per-robot JSON configuration overrides")
    parser.add_argument("--max-speed", type=float, default=NavParams.max_speed)
    parser.add_argument("--max-theta", type=float, default=NavParams.max_theta)
    parser.add_argument("--max-goal-distance", type=float, default=NavParams.max_goal_distance)
    parser.add_argument("--standoff", type=float, default=NavParams.standoff,
                        help="stop this far before the selected radar point in m (0 = drive onto it)")
    parser.add_argument("--plot-range", type=float, default=4.0, help="robot-relative plot extent in m")
    parser.add_argument("--map-x", type=float, default=4.0, help="global map extent forward: 0..map-x m")
    parser.add_argument("--map-y", type=float, default=4.0, help="global map extent lateral: +/- map-y m")
    parser.add_argument("--snr-min", type=float, default=10.0, help="colormap lower bound in dB")
    parser.add_argument("--snr-max", type=float, default=40.0, help="colormap upper bound in dB")
    parser.add_argument("--max-time", type=float, default=0.0, help="session time limit in s (0 = none)")
    parser.add_argument("--max-travel", type=float, default=10.0, help="total commanded travel limit in m")
    parser.add_argument("--radar-yaw", type=float, default=RADAR_YAW_DEG)
    parser.add_argument("--radar-az-sign", type=int, choices=(-1, 1), default=RADAR_AZ_SIGN)
    parser.add_argument("--imu-yaw", type=float, default=IMU_YAW_DEG)
    args = parser.parse_args()
    if args.check_axes and args.dry_run:
        parser.error("--check-axes moves the robot; it cannot be combined with --dry-run")
    if not 0 < args.max_speed <= 0.4 or not 0 < args.max_theta <= 150:
        parser.error("max speed must be in (0, 0.4] m/s and max theta in (0, 150] deg/s")

    if args.standoff < 0:
        parser.error("--standoff must be >= 0")
    p = NavParams(max_speed=args.max_speed, max_theta=args.max_theta, max_goal_distance=args.max_goal_distance,
                  standoff=args.standoff)
    plotter = Plotter(dict(plot_range=args.plot_range, map_x=args.map_x, map_y=args.map_y,
                           snr_min=args.snr_min, snr_max=args.snr_max),
                      enabled=not (args.no_plot or args.check_axes))
    config = load_config(args.config)
    imu = ImuStream(config["imu"])
    radar = RadarReader(config["radar"])
    bus = ServoBus(config["arm"])
    base = Base(WheelDrive(bus), args.dry_run)
    try:
        imu.start()
        radar.start()
        wait_sensors(imu, radar)
        if args.check_axes:
            check_axes(imu, radar, base)
        else:
            navigate(imu, radar, base, p, args, plotter)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            base.stop()
        finally:
            radar.close()
            imu.close()
            bus.close()
            plotter.finish()


class NavigationLogicTest(unittest.TestCase):
    P = NavParams()

    def test_goto_straight_and_in_body_frame(self):
        x, y, theta, arrived = goto_command((0, 0, 0), Goal(1, 0, 0), self.P)
        self.assertAlmostEqual(x, self.P.max_speed)
        self.assertAlmostEqual(y, 0)
        self.assertEqual(theta, 0)
        self.assertFalse(arrived)
        # Robot facing +y (yaw 90): a goal at global +y is straight ahead in the body frame.
        x, y, theta, _ = goto_command((0, 0, 90), Goal(0, 1, 90), self.P)
        self.assertAlmostEqual(x, self.P.max_speed)
        self.assertAlmostEqual(y, 0, places=9)
        self.assertEqual(theta, 0)
        x, y, _, _ = goto_command((0, 0, 0), Goal(0, -1, 0), self.P)  # Strafe right.
        self.assertAlmostEqual(x, 0, places=9)
        self.assertLess(y, 0)

    def test_goto_heading_rotate_in_place_and_arrival(self):
        _, _, theta, _ = goto_command((0, 0, 0), Goal(1, 0, 90), self.P)
        self.assertEqual(theta, self.P.max_theta)
        self.assertLess(goto_command((0, 0, 0), Goal(1, 0, -10), self.P)[2], 0)
        x, y, theta, arrived = goto_command((1, 0, 0), Goal(1.02, 0, 30), self.P)
        self.assertEqual((x, y), (0, 0))
        self.assertGreater(theta, 0)
        self.assertFalse(arrived)
        self.assertEqual(goto_command((1, 0, 28), Goal(1.02, 0, 30), self.P), (0, 0, 0, True))
        # Wrap-around: 170 -> -170 is +20 deg, not -340.
        self.assertGreater(goto_command((0, 0, 170), Goal(0, 0, -170), self.P)[2], 0)
        command = goto_command((0, 0, 0), Goal(3, 2, 100), self.P)[:3]
        self.assertTrue(valid_motion(dict(x=command[0], y=command[1], theta=command[2], duration_ms=1)))

    def test_make_goal_faces_travel_direction(self):
        self.assertAlmostEqual(make_goal(1, 1, None, (0, 0, 0)).theta, 45)
        self.assertEqual(make_goal(0, 0, None, (0, 0, 30)).theta, 30)
        self.assertEqual(make_goal(0, 0, 270, (0, 0, 0)).theta, -90)

    def test_point_goal_stops_before_point_and_faces_it(self):
        goal = make_point_goal((2, 0), None, (0, 0, 0), 0.3)
        self.assertAlmostEqual(goal.x, 1.7)
        self.assertAlmostEqual(goal.y, 0)
        self.assertAlmostEqual(goal.theta, 0)
        goal = make_point_goal((1, 1), 90, (0, 0, 0), 0.3)  # Dragged heading: approach from below.
        self.assertAlmostEqual(goal.x, 1)
        self.assertAlmostEqual(goal.y, 0.7)
        self.assertAlmostEqual(goal.theta, 90)
        goal = make_point_goal((0, 0.2), None, (0, 0, 0), 0.3)  # Inside the standoff: turn only.
        self.assertEqual((goal.x, goal.y), (0, 0))
        self.assertAlmostEqual(goal.theta, 90)
        self.assertEqual(make_point_goal((2, 0), None, (0, 0, 0), 0).x, 2)
        rng, bearing = point_bearing((1, 1), (1, 0, 0))
        self.assertAlmostEqual(rng, 1)
        self.assertAlmostEqual(bearing, 90)
        self.assertAlmostEqual(point_bearing((2, 0), (1, 0, 90))[1], -90)

    def test_radar_packets_match_test_hardware_format(self):
        try:
            from test_hardware import radar_packet
        except ImportError:
            from tests.test_hardware import radar_packet
        first = radar_packet(((0.0, 1.0, 0.0, 0.0), (0.5, 2.0, 0.25, -0.5)))  # float32-exact values
        second = radar_packet(((1.0, 1.0, 0.0, 0.0),), snr=False)
        buf = bytearray(b"\x00garbage" + first + second[:30])
        packets = extract_packets(buf)
        self.assertEqual(packets, [first])
        self.assertEqual(bytes(buf), second[:30])  # Partial packet kept for the next read.
        buf.extend(second[30:])
        self.assertEqual(extract_packets(buf), [second])
        self.assertEqual(len(buf), 0)
        sample = decode_packet(first)
        self.assertEqual(sample["frame"], 42)
        self.assertEqual(sample["points_xyzd"], [[0.0, 1.0, 0.0, 0.0], [0.5, 2.0, 0.25, -0.5]])
        self.assertEqual(sample["snr_db"], [18.5, 18.5])
        self.assertEqual(decode_packet(second)["snr_db"], [None])
        corrupt = bytearray(first)
        corrupt[50] ^= 0xFF  # Valid framing, different payload: still parsed.
        self.assertEqual(len(extract_packets(bytearray(corrupt))), 1)
        broken = bytearray(first)
        struct.pack_into("<I", broken, 12, len(first) + 32)  # Wrong length: rejected, no crash.
        self.assertEqual(extract_packets(bytearray(broken) + bytearray(64)), [])

    def test_parse_command(self):
        self.assertEqual(parse_command("1 -0.5"), ("goal", 1.0, -0.5, None))
        self.assertEqual(parse_command("1 2 90"), ("goal", 1.0, 2.0, 90.0))
        self.assertEqual(parse_command("c"), ("cancel",))
        self.assertEqual(parse_command("q"), ("quit",))
        self.assertIsNone(parse_command(""))
        self.assertIsNone(parse_command("1 nan"))
        self.assertIsNone(parse_command("go there"))

    def test_frames(self):
        # TI: +y ahead, +x right -> robot: +x ahead, -y right
        np.testing.assert_allclose(radar_xy_to_robot([[0, 2], [1, 0]], 0, 1, (0, 0)), [[2, 0], [0, -1]], atol=1e-12)
        np.testing.assert_allclose(radar_xy_to_robot([[1, 0]], 0, -1, (0.1, 0)), [[0.1, 1]], atol=1e-12)
        np.testing.assert_allclose(transform([[1, 0]], (1, 2, 90)), [[1, 3]], atol=1e-12)
        np.testing.assert_allclose(to_robot(transform([[0.3, -0.7]], (1, 2, 33)), (1, 2, 33)), [[0.3, -0.7]])
        self.assertEqual(transform([], (1, 2, 90)).shape, (0, 2))
        points, snr = radar_cloud({"points_xyzd": [[0, 1, 0, 0], [1, 1, 0, 0]], "snr_db": [20.0, None]}, 0, 1)
        self.assertEqual(points.shape, (2, 2))
        self.assertEqual(snr[0], 20)
        self.assertTrue(np.isnan(snr[1]))

    def test_odom_pose(self):
        imu = {"estimate_valid": True, "velocity_m_s": [0, 0, 0], "orientation_deg": [0, 0, 10],
               "gyro_corrected_deg_s": [0, 0, 0]}
        sim = OdomPose(imu, np.array([0, 0, 1]), simulate=True)
        for _ in range(100):
            pose = sim.update(imu, (0.1, 0, 90), 0.01)
        self.assertAlmostEqual(pose[2], 90, places=6)
        self.assertAlmostEqual(pose[0], 0.1 * 2 / math.pi, places=3)  # Quarter circle of radius v/w.
        self.assertAlmostEqual(pose[1], 0.1 * 2 / math.pi, places=3)
        real = OdomPose(imu, np.array([0, 0, 1]), simulate=False)
        real.update({**imu, "orientation_deg": [0, 0, 100]}, (0, 0, 0), 0.01)
        self.assertAlmostEqual(real.yaw, 90)
        pose = real.update({**imu, "estimate_valid": False, "gyro_corrected_deg_s": [0, 0, 10]}, (0.1, 0, 0), 1.0)
        self.assertAlmostEqual(pose[2], 100)  # Falls back to the gyro.
        self.assertAlmostEqual(pose[0], 0.1 * math.cos(math.radians(95)))

    def test_imu_pose_and_frames(self):
        imu = {"velocity_m_s": [0, .1, 0], "orientation_deg": [0, 0, 90], "gyro_corrected_deg_s": [0, 0, -10]}
        np.testing.assert_allclose(body_velocity(imu, np.zeros(2), 0), [.1, 0], atol=1e-12)
        self.assertAlmostEqual(yaw_rate(imu, np.array([0, 0, -1])), 10)
        start = {"estimate_valid": True, "displacement_m": [1, 1, 0], "velocity_m_s": [.1, 0, 0],
                 "orientation_deg": [0, 0, 0]}
        later = {**start, "displacement_m": [1.3, 1, 0], "orientation_deg": [0, 0, 190]}
        x, y, yaw = ImuPose(start, 0, 0).pose(later, 1.0)
        self.assertAlmostEqual(x, .2)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(yaw, -170)
        self.assertIsNone(ImuPose(start, 0, 0).pose({**later, "estimate_valid": False}, 1.0))

    def test_verifier_flags_missing_rotation(self):
        imu = {"velocity_m_s": [0, 0, 0], "orientation_deg": [0, 0, 0], "gyro_corrected_deg_s": [0, 0, 0],
               "estimate_valid": True}
        verifier = ImuVerifier(imu, np.array([0, 0, 1]), 0)
        status = None
        for i in range(100):
            status = verifier.update(imu, (0, 0, 40), i * .02)
        self.assertFalse(status["yaw_ok"])
        self.assertGreater(status["yaw_bad_for"], YAW_MISMATCH_ABORT_S)
        imu["gyro_corrected_deg_s"] = [0, 0, 40]
        for i in range(100, 200):
            status = verifier.update(imu, (0, 0, 40), i * .02)
        self.assertTrue(status["yaw_ok"])

    def test_plot_view_render_and_clicks(self):
        try:
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            from matplotlib.figure import Figure
        except ImportError:
            self.skipTest("matplotlib not installed")
        from types import SimpleNamespace

        fig = Figure(figsize=(12, 6))
        FigureCanvasAgg(fig)
        emitted = []
        settings = dict(plot_range=4, map_x=4, map_y=4, snr_min=10, snr_max=40)
        view = PlotView(fig, settings, emit=emitted.append)
        self.assertEqual((view.ax_glob.get_xlim(), view.ax_glob.get_ylim()), ((4, -4), (0, 4)))
        view.handle({"type": "map", "points": np.array([[1, 0], [2, 1]]), "snr": [15, np.nan]})
        # Current frame point 1 m ahead of a robot at (1, 0) facing +y -> global (1, 1).
        frame = {"type": "frame", "points": [[1, 0]], "snr": [30], "pose": (1, 0, 90), "odom": (1, 0, 90),
                 "imu": (1.1, 0, 90), "goal": (2, 1, 0), "selected": (2, 1, 20), "text": "moving",
                 "sent": time.time()}
        view.handle(frame)
        view.render()  # Full draw caches the background.
        self.assertIsNotNone(view.background)
        np.testing.assert_allclose(view.rel_selected.get_offsets(), bev(to_robot([(2, 1)], (1, 0, 90))))
        view.handle({**frame, "goal": None, "selected": None, "imu": None, "odom": (9, 0, 0)})
        view.render()  # Blit path; the global extent stays fixed.
        self.assertEqual(view.ax_glob.get_ylim(), (0, 4))
        self.assertEqual(len(view.rel_selected.get_offsets()), 0)
        self.assertEqual((len(view.odom_xy), len(view.imu_xy)), (2, 1))

        def click(ax, x, y, button=1, release=None):
            # Screen coordinates: xdata = y (left +), ydata = x (forward +).
            view.on_press(SimpleNamespace(inaxes=ax, xdata=y, ydata=x, button=button))
            rx, ry = release or (x, y)
            view.on_release(SimpleNamespace(inaxes=ax, xdata=ry, ydata=rx, button=button))

        click(view.ax_glob, 2.1, 0.8)  # Snaps to the map point (2, 1), which has no SNR.
        self.assertEqual(emitted[-1][:3], ("point", 2.0, 1.0))
        self.assertTrue(math.isnan(emitted[-1][3]))
        self.assertIsNone(emitted[-1][4])
        click(view.ax_glob, 1.05, -0.1, release=(1.05, 0.9))  # Snaps to (1, 0); drag toward +y: heading 90.
        self.assertEqual(emitted[-1][:4], ("point", 1.0, 0.0, 15.0))
        self.assertAlmostEqual(emitted[-1][4], 90)
        click(view.ax_rel, 0.9, 0.1)  # Robot frame -> global (0.9, 1.1) region: current-frame point (1, 1).
        np.testing.assert_allclose(emitted[-1][1:4], (1, 1, 30), atol=1e-12)
        view.on_press(SimpleNamespace(inaxes=view.ax_glob, xdata=0, ydata=0, button=3))
        self.assertEqual(emitted[-1], ("cancel",))
        view.on_press(SimpleNamespace(inaxes=None, xdata=None, ydata=None, button=1))
        self.assertIsNone(view.press)
        view.handle(None)
        self.assertTrue(view.title.get_text().endswith("[finished]"))

        empty_fig = Figure()
        FigureCanvasAgg(empty_fig)
        empty = PlotView(empty_fig, settings, emit=emitted.append)
        count = len(emitted)
        empty.on_press(SimpleNamespace(inaxes=empty.ax_glob, xdata=0, ydata=1, button=1))
        empty.on_release(SimpleNamespace(inaxes=empty.ax_glob, xdata=0, ydata=1, button=1))
        self.assertEqual(len(emitted), count)  # No radar points: nothing to select.
        self.assertIn("No radar points", empty.info.get_text())


if __name__ == "__main__":
    main()
