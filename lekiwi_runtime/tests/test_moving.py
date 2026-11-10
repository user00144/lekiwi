"""Radar-guided approach: drive toward the strongest radar return, verified by the IMU.

Runs standalone on the Orin (no ZMQ server). From lekiwi_runtime/:

    uv run python tests/test_moving.py --check-axes   # 1) measure mounting constants below
    uv run python tests/test_moving.py --dry-run      # 2) sensors + live plot; wheels never energize
    uv run python tests/test_moving.py                # 3) drive with live plot

Live plot (separate process, needs a display; --no-plot to disable):
    left  = robot-relative BEV of the current radar frame, highest-SNR target in red
    right = global BEV: radar map captured at start (global frame = robot pose at start),
            current frame placed by the IMU pose, IMU trajectory and commanded dead-reckoning

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
import time
import unittest

import numpy as np

from lekiwi_runtime.config import load_config
from lekiwi_runtime.drivers.imu import ImuStream
from lekiwi_runtime.drivers.radar import RadarStream
from lekiwi_runtime.drivers.servo import ServoBus
from lekiwi_runtime.drivers.wheels import GOAL_VELOCITY, WheelDrive, body_to_wheel_raw, valid_motion

# --- Mounting: confirm with --check-axes, then edit (or pass the CLI overrides). ---
RADAR_YAW_DEG = 0.0       # Radar boresight direction in the robot frame, CCW from +x (0 = faces forward).
RADAR_AZ_SIGN = 1         # +1: positive radar azimuth is to the robot's right (TI convention); -1 if mirrored.
RADAR_POS_M = (0.0, 0.0)  # Radar position in the robot frame (x forward, y left), used for plotting.
IMU_YAW_DEG = 0.0         # Direction of the IMU's levelled +x axis in the robot frame, CCW from +x.

# --- Timing and safety ---
LOOP_HZ = 50
PLOT_HZ = 10
RADAR_HOLD_S = 0.3     # No new radar frame for this long -> zero velocity.
RADAR_ABORT_S = 2.0
TARGET_HOLD_S = 0.5    # Target missing for this long -> zero velocity.
TARGET_ABORT_S = 3.0
IMU_ABORT_S = 0.5      # IMU snapshot unavailable/stale for this long -> abort.

# --- IMU verification tolerances (EMA-filtered command vs measurement) ---
EMA_TAU_S = 0.3
YAW_TOL_ABS = 8.0      # deg/s
YAW_TOL_REL = 0.3
VEL_TOL_ABS = 0.05     # m/s
VEL_TOL_REL = 0.4
YAW_MISMATCH_ABORT_S = 1.5


@dataclass
class ApproachParams:
    stop_distance: float = 0.5   # m, stop when target range reaches this
    arrive_tol: float = 0.05     # m
    max_speed: float = 0.15      # m/s
    k_speed: float = 0.5         # (m/s) per m of range error
    max_theta: float = 45.0      # deg/s
    k_theta: float = 1.5         # (deg/s) per deg of azimuth error
    az_deadband: float = 2.0     # deg
    turn_only_deg: float = 30.0  # |azimuth| at which translation reaches zero (rotate in place)
    min_range: float = 0.2       # m
    max_range: float = 4.0       # m
    max_az: float = 60.0         # deg
    min_snr: float = 10.0        # dB
    gate: float = 0.5            # m, max jump from previous target in radar x/y (0 disables)


@dataclass
class Target:
    x: float
    y: float
    range_m: float
    azimuth_deg: float
    snr_db: float


def select_target(radar: dict, p: ApproachParams, last: Target | None = None) -> Target | None:
    """Highest-SNR point within range/azimuth limits (and near the previous target if gated)."""
    best = None
    for point, snr in zip(radar.get("points_xyzd") or [], radar.get("snr_db") or []):
        if snr is None:
            continue
        x, y = float(point[0]), float(point[1])
        rng = math.hypot(x, y)
        az = math.degrees(math.atan2(x, y))
        if not p.min_range <= rng <= p.max_range or abs(az) > p.max_az or snr < p.min_snr:
            continue
        if last is not None and p.gate > 0 and math.hypot(x - last.x, y - last.y) > p.gate:
            continue
        if best is None or snr > best.snr_db:
            best = Target(x, y, rng, az, float(snr))
    return best


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def approach_command(target: Target, p: ApproachParams, radar_yaw_deg: float = RADAR_YAW_DEG,
                     az_sign: int = RADAR_AZ_SIGN) -> tuple[float, float, float, bool]:
    """(x m/s, y m/s, theta deg/s, arrived): turn to put the target at azimuth 0 while moving toward it."""
    az_ccw = -az_sign * target.azimuth_deg  # Target bearing from boresight, CCW positive.
    error = target.range_m - p.stop_distance
    if error <= p.arrive_tol:
        return 0.0, 0.0, 0.0, True
    theta = 0.0 if abs(az_ccw) <= p.az_deadband else clamp(p.k_theta * az_ccw, p.max_theta)
    speed = min(p.max_speed, p.k_speed * error) * max(0.0, 1 - abs(az_ccw) / p.turn_only_deg)
    heading = math.radians(radar_yaw_deg + az_ccw)
    return speed * math.cos(heading), speed * math.sin(heading), theta, False


def radar_xy_to_robot(xy, radar_yaw_deg: float = RADAR_YAW_DEG, az_sign: int = RADAR_AZ_SIGN,
                      offset=RADAR_POS_M) -> np.ndarray:
    """TI radar x/y (N,2) -> robot-frame x forward / y left (N,2)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    rng = np.hypot(xy[:, 0], xy[:, 1])
    bearing = math.radians(radar_yaw_deg) - az_sign * np.arctan2(xy[:, 0], xy[:, 1])
    return np.column_stack([offset[0] + rng * np.cos(bearing), offset[1] + rng * np.sin(bearing)])


def radar_points_robot(radar: dict, radar_yaw_deg: float, az_sign: int) -> np.ndarray:
    return radar_xy_to_robot([p[:2] for p in radar.get("points_xyzd") or []], radar_yaw_deg, az_sign)


def transform(points, pose) -> np.ndarray:
    """Robot-frame points (N,2) -> global, for pose (x, y, yaw_deg)."""
    x, y, yaw = pose
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return np.asarray(points, dtype=float).reshape(-1, 2) @ np.array([[c, s], [-s, c]]) + [x, y]


def rotate(v, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


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


class ImuPose:
    """Global pose (robot frame at start) from IMU displacement.

    The IMU velocity at start should be zero; its residual is treated as constant drift and removed.
    Double-integrated position still drifts with time, so read the trajectory as an estimate.
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
        yaw = (imu["orientation_deg"][2] - self.yaw0 + 180) % 360 - 180
        return float(x), float(y), float(yaw)


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


def bev(points) -> np.ndarray:
    """x forward / y left -> screen (horizontal = y, vertical = x); axes invert horizontal so left is left."""
    return np.asarray(points, dtype=float).reshape(-1, 2)[:, ::-1]


class PlotView:
    def __init__(self, fig, settings: dict):
        from matplotlib.patches import Circle, Polygon

        self.fig = fig
        self.ax_rel, self.ax_glob = fig.subplots(1, 2)
        r, az = settings["max_range"], math.radians(settings["max_az"])
        for ax, title in ((self.ax_rel, "Robot-relative BEV (current radar frame)"),
                          (self.ax_glob, "Global BEV (start pose = origin)")):
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("y [m]  (left +)")
            ax.set_ylabel("x [m]  (forward +)")
            ax.set_title(title)
            ax.set_xlim(r, -r)
            ax.set_ylim(-1.0, r + 0.5)

        ax = self.ax_rel
        for side in (-1, 1):
            edge = bev([[0, 0], [r * math.cos(az), side * r * math.sin(az)]])
            ax.plot(edge[:, 0], edge[:, 1], "k:", lw=0.8)
        ax.add_patch(Circle((0, 0), settings["stop_distance"], fill=False, ls="--", color="gray",
                            label="stop distance"))
        ax.add_patch(Polygon(bev(ROBOT_SHAPE), closed=True, color="black", zorder=4))
        self.rel_points = ax.scatter([], [], s=10, c="tab:blue", alpha=0.7, label="radar points")
        self.rel_target = ax.scatter([], [], s=200, c="red", marker="*", zorder=5, label="highest SNR (target)")
        ax.legend(loc="upper right", fontsize=8)

        ax = self.ax_glob
        self.map_points = ax.scatter([], [], s=8, c="0.3", label="initial radar map")
        self.glob_points = ax.scatter([], [], s=8, c="tab:blue", alpha=0.35, label="current frame @ IMU pose")
        self.glob_target = ax.scatter([], [], s=200, c="red", marker="*", zorder=5, label="target")
        (self.traj,) = ax.plot([], [], "-", c="tab:green", lw=2, label="IMU trajectory")
        (self.cmd_traj,) = ax.plot([], [], "--", c="tab:orange", lw=1.2, label="commanded dead-reckoning")
        self.robot = Polygon(bev(ROBOT_SHAPE), closed=True, color="tab:green", zorder=6)
        ax.add_patch(self.robot)
        ax.scatter([0], [0], marker="x", c="black", zorder=6, label="start")
        ax.legend(loc="upper right", fontsize=8)

        self.title = fig.suptitle("Waiting for radar map...")
        self.pose = (0.0, 0.0, 0.0)
        self.traj_xy, self.cmd_xy = [], []
        self.finished = False

    def handle(self, msg) -> None:
        if msg is None:
            if not self.finished:
                self.finished = True
                self.title.set_text(self.title.get_text() + "   [finished]")
            return
        if msg["type"] == "map":
            self.map_points.set_offsets(bev(msg["points"]))
            return
        points = np.asarray(msg["points"], dtype=float).reshape(-1, 2)
        target = [] if msg["target"] is None else [msg["target"]]
        if msg["pose"] is not None:
            self.pose = msg["pose"]
            self.traj_xy.append(self.pose[:2])
        if msg.get("cmd_pose") is not None:
            self.cmd_xy.append(msg["cmd_pose"][:2])
        self.rel_points.set_offsets(bev(points))
        self.rel_target.set_offsets(bev(target))
        self.glob_points.set_offsets(bev(transform(points, self.pose)))
        self.glob_target.set_offsets(bev(transform(target, self.pose)))
        self.traj.set_data(*bev(self.traj_xy).T)
        self.cmd_traj.set_data(*bev(self.cmd_xy).T)
        self.robot.set_xy(bev(transform(ROBOT_SHAPE, self.pose)))
        self.title.set_text(msg.get("text", ""))
        self._fit()

    def _fit(self) -> None:
        """Grow the global view when a trajectory leaves it (never shrinks, so it does not jitter)."""
        pts = np.asarray(self.traj_xy + self.cmd_xy, dtype=float).reshape(-1, 2)
        if not len(pts):
            return
        (left, right), (bottom, top) = self.ax_glob.get_xlim(), self.ax_glob.get_ylim()
        limits = (max(left, pts[:, 1].max() + 0.5), min(right, pts[:, 1].min() - 0.5),
                  min(bottom, pts[:, 0].min() - 0.5), max(top, pts[:, 0].max() + 0.5))
        if limits != (left, right, bottom, top):
            self.ax_glob.set_xlim(limits[0], limits[1])
            self.ax_glob.set_ylim(limits[2], limits[3])


def plot_worker(messages, settings: dict, shown) -> None:
    import signal

    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl+C stops the robot in the main process only.
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Plot disabled: matplotlib not installed (run `uv sync`)", flush=True)
        return
    if plt.get_backend().lower() in NON_INTERACTIVE_BACKENDS:
        print("Plot disabled: no display (run on the Orin desktop or set DISPLAY)", flush=True)
        return
    fig = plt.figure(figsize=(14, 7), layout="constrained")
    view = PlotView(fig, settings)
    try:
        fig.canvas.manager.set_window_title("LeKiwi radar approach")
    except AttributeError:
        pass
    plt.show(block=False)
    shown.set()
    while plt.fignum_exists(fig.number):
        changed = False
        while True:
            try:
                msg = messages.get_nowait()
            except queue_module.Empty:
                break
            except (EOFError, OSError):
                msg = None
            view.handle(msg)
            changed = True
        if changed:
            fig.canvas.draw_idle()
        plt.pause(0.05)


class Plotter:
    def __init__(self, settings: dict, enabled: bool):
        self.queue = self.process = None
        if not enabled:
            return
        ctx = mp.get_context("spawn")  # Never fork a process that owns serial ports and threads.
        self.queue = ctx.Queue(maxsize=20)
        self.shown = ctx.Event()
        self.process = ctx.Process(target=plot_worker, args=(self.queue, settings, self.shown),
                                   daemon=True, name="plot")
        self.process.start()

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

def wait_sensors(imu: ImuStream, radar: RadarStream, imu_timeout=90.0, radar_timeout=30.0) -> None:
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
    while True:
        state = radar.snapshot()
        if state["available"]:
            break
        if radar.ready:
            raise RuntimeError(f"Radar unavailable: {state.get('error')}")
        if time.monotonic() > deadline:
            raise TimeoutError(f"No radar frame within {radar_timeout:.0f} s: {state.get('error')}")
        time.sleep(0.05)
    if not any(s is not None for s in state.get("snr_db") or []) and state.get("count"):
        print("Warning: radar frames carry no SNR (side-info TLV disabled?); no target can be selected.")
    print("Radar ready.", flush=True)


def gravity_up(imu: ImuStream, samples: int = 20) -> np.ndarray:
    """Unit 'up' vector in the IMU frame from the stationary accelerometer reading."""
    accel = []
    for _ in range(samples):
        accel.append(imu.snapshot()["accel_m_s2"])
        time.sleep(0.01)
    mean = np.mean(accel, axis=0)
    return mean / np.linalg.norm(mean)


def find_target(radar: RadarStream, p: ApproachParams, timeout: float = 5.0) -> Target:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        target = select_target(radar.snapshot(), p)
        if target:
            return target
        time.sleep(0.05)
    raise RuntimeError(f"No radar target with SNR >= {p.min_snr} dB within {p.min_range}-{p.max_range} m, "
                       f"|azimuth| <= {p.max_az} deg")


def capture_map(radar: RadarStream, seconds: float, radar_yaw_deg: float, az_sign: int) -> np.ndarray:
    """Robot-frame points of the frames received during `seconds` (at least one frame)."""
    clouds, last_frame = [], None
    end = time.monotonic() + seconds
    while not clouds or time.monotonic() < end:
        state = radar.snapshot()
        if state["available"] and state["frame"] != last_frame:
            last_frame = state["frame"]
            clouds.append(radar_points_robot(state, radar_yaw_deg, az_sign))
        time.sleep(0.02)
    return np.vstack(clouds)


def average_target(radar: RadarStream, p: ApproachParams, seconds: float = 1.0) -> Target | None:
    """Median of per-frame strongest targets over a short window."""
    found, last_frame = [], None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state = radar.snapshot()
        if state["available"] and state["frame"] != last_frame:
            last_frame = state["frame"]
            target = select_target(state, p)
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


def frame_message(radar: dict, target: Target | None, pose, cmd_pose, text: str, args) -> dict:
    target_xy = None
    if target is not None:
        target_xy = tuple(radar_xy_to_robot([[target.x, target.y]], args.radar_yaw, args.radar_az_sign)[0])
    return {"type": "frame", "points": radar_points_robot(radar, args.radar_yaw, args.radar_az_sign),
            "target": target_xy, "pose": pose, "cmd_pose": cmd_pose, "text": text}


def approach(imu: ImuStream, radar: RadarStream, base: Base, p: ApproachParams, args, plotter: Plotter) -> None:
    up = gravity_up(imu)
    target = find_target(radar, p)
    global_map = capture_map(radar, args.map_seconds, args.radar_yaw, args.radar_az_sign)
    plotter.send({"type": "map", "points": global_map}, block=True)
    plotter.send(frame_message(radar.snapshot(), target, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                               "Press Enter in the terminal to start", args), block=True)
    print(f"Global map: {len(global_map)} points from the first {args.map_seconds:g} s of radar frames")
    print(f"Initial target: {describe(target)}")
    print(f"Mounting: radar_yaw={args.radar_yaw} deg, radar_az_sign={args.radar_az_sign}, imu_yaw={args.imu_yaw} deg"
          f"{'  [DRY RUN: wheels stay off]' if args.dry_run else ''}")
    input("Press Enter to start the approach (Ctrl+C to abort)... ")

    start = prev = last_frame_time = last_seen = time.monotonic()
    first_imu = imu.snapshot()
    verifier = ImuVerifier(first_imu, up, args.imu_yaw)
    tracker = ImuPose(first_imu, args.imu_yaw, start)
    if tracker.d0 is None:
        print("Warning: IMU position estimate invalid; trajectory will not be drawn.")
    last_frame = last_print = last_plot = None
    imu_bad_since = None
    command = (0.0, 0.0, 0.0)
    cmd_pose = [0.0, 0.0, 0.0]
    pose = None
    travelled = 0.0
    try:
        while True:
            now = time.monotonic()
            dt, prev = now - prev, now
            if now - start > args.max_time:
                print(f"\nStopped: time limit {args.max_time:.0f} s")
                break
            # Dead-reckon the command that was in effect over the last dt (for comparison only).
            heading = math.radians(cmd_pose[2])
            cmd_pose[0] += (math.cos(heading) * command[0] - math.sin(heading) * command[1]) * dt
            cmd_pose[1] += (math.sin(heading) * command[0] + math.cos(heading) * command[1]) * dt
            cmd_pose[2] += command[2] * dt
            travelled += math.hypot(command[0], command[1]) * dt
            if travelled > args.max_travel:
                print(f"\nStopped: commanded travel limit {args.max_travel:.1f} m")
                break

            state = radar.snapshot()
            new_frame = state["available"] and state["frame"] != last_frame
            found = None
            if new_frame:
                last_frame, last_frame_time = state["frame"], now
                found = select_target(state, p, target)
                if found:
                    target, last_seen = found, now
                    x, y, theta, arrived = approach_command(target, p, args.radar_yaw, args.radar_az_sign)
                    if arrived:
                        print(f"\nArrived: {describe(target)}")
                        break
                    command = (x, y, theta)
            if now - last_frame_time > RADAR_ABORT_S:
                raise RuntimeError("Radar frames stopped")
            if now - last_seen > TARGET_ABORT_S:
                raise RuntimeError(f"Target lost for {TARGET_ABORT_S:.0f} s (last: {describe(target)})")
            if now - last_frame_time > RADAR_HOLD_S or now - last_seen > TARGET_HOLD_S:
                command = (0.0, 0.0, 0.0)
            base.send(*command)

            imu_state = imu.snapshot()
            if not imu_state.get("available") or imu_state.get("gyro_corrected_deg_s") is None:
                imu_bad_since = imu_bad_since or now
                if now - imu_bad_since > IMU_ABORT_S:
                    raise RuntimeError(f"IMU unavailable during motion: {imu_state.get('error')}")
                time.sleep(1 / LOOP_HZ)
                continue
            imu_bad_since = None
            pose = tracker.pose(imu_state, now)
            status = verifier.update(imu_state, command, now)
            if not args.dry_run and status["yaw_bad_for"] > YAW_MISMATCH_ABORT_S:
                raise RuntimeError(f"IMU yaw rate {status['yaw_rate']:+.1f} deg/s does not follow "
                                   f"theta command {command[2]:+.1f} deg/s for {YAW_MISMATCH_ABORT_S} s")

            if new_frame and (last_plot is None or now - last_plot >= 1 / PLOT_HZ):
                last_plot = now
                pose_text = ("IMU pose invalid" if pose is None else
                             f"IMU pose x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={pose[2]:+.0f}")
                text = (f"t={now - start:4.1f}s  {describe(found or target)}\n"
                        f"cmd x={command[0]:+.2f} y={command[1]:+.2f} th={command[2]:+.0f}  |  "
                        f"IMU yaw rate {status['yaw_rate']:+.0f} {'OK' if status['yaw_ok'] else 'MISMATCH'}  |  "
                        f"{pose_text}")
                plotter.send(frame_message(state, found, pose, tuple(cmd_pose), text, args))

            if last_print is None or now - last_print >= 0.25:
                last_print = now
                vel = status["velocity"]
                vel_tag = "n/a" if status["vel_ok"] is None else ("OK" if status["vel_ok"] else "MISMATCH")
                print(f"t={now - start:5.1f}s tgt r={target.range_m:4.2f} az={target.azimuth_deg:+5.1f} "
                      f"snr={target.snr_db:4.1f} | cmd x={command[0]:+.2f} y={command[1]:+.2f} "
                      f"th={command[2]:+5.1f} | imu th={status['yaw_rate']:+5.1f} "
                      f"{'OK' if status['yaw_ok'] else 'MISMATCH'} v=({vel[0]:+.2f},{vel[1]:+.2f}) {vel_tag}",
                      flush=True)
            time.sleep(max(0.0, 1 / LOOP_HZ - (time.monotonic() - now)))
        base.send(0, 0, 0)
    finally:
        print(verifier.summary())
        if pose is not None:
            print(f"End pose  IMU: x={pose[0]:+.2f} m y={pose[1]:+.2f} m yaw={pose[2]:+.0f} deg | "
                  f"commanded: x={cmd_pose[0]:+.2f} m y={cmd_pose[1]:+.2f} m yaw={cmd_pose[2]:+.0f} deg")


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


def check_axes(imu: ImuStream, radar: RadarStream, base: Base, p: ApproachParams) -> None:
    up = gravity_up(imu)
    print(f"Gravity 'up' in IMU frame: {np.round(up, 3)}")
    print("Axis check. Place one strong reflector ~1 m directly in front of the robot; keep 0.5 m clear around.")
    input("Press Enter to measure the target... ")
    before = average_target(radar, p)
    print(f"Target (front): {describe(before)}  -> expect azimuth ~0 if the radar faces forward")

    input("Press Enter to rotate CCW at +30 deg/s for 1 s... ")
    yaw, _ = drive_and_measure(imu, base, up, (0, 0, 30), 1.0)
    after = average_target(radar, p)
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
    before = average_target(radar, p)
    _, velocity = drive_and_measure(imu, base, up, (0.10, 0, 0), 1.0)
    after = average_target(radar, p)
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
    parser = argparse.ArgumentParser(description="Approach the strongest radar return; verify motion with the IMU.")
    parser.add_argument("--check-axes", action="store_true", help="measure radar/IMU mounting with short moves")
    parser.add_argument("--dry-run", action="store_true", help="read sensors and print commands; never energize wheels")
    parser.add_argument("--no-plot", action="store_true", help="disable the live BEV plot")
    parser.add_argument("--map-seconds", type=float, default=1.0,
                        help="radar frames accumulated at start as the global map (0 = first frame only)")
    parser.add_argument("--config", type=Path, help="per-robot JSON configuration overrides")
    parser.add_argument("--stop-distance", type=float, default=ApproachParams.stop_distance)
    parser.add_argument("--max-speed", type=float, default=ApproachParams.max_speed)
    parser.add_argument("--max-theta", type=float, default=ApproachParams.max_theta)
    parser.add_argument("--min-snr", type=float, default=ApproachParams.min_snr)
    parser.add_argument("--max-range", type=float, default=ApproachParams.max_range)
    parser.add_argument("--gate", type=float, default=ApproachParams.gate, help="target jump gate in m (0 disables)")
    parser.add_argument("--max-time", type=float, default=30.0)
    parser.add_argument("--max-travel", type=float, default=3.0, help="commanded travel limit in m")
    parser.add_argument("--radar-yaw", type=float, default=RADAR_YAW_DEG)
    parser.add_argument("--radar-az-sign", type=int, choices=(-1, 1), default=RADAR_AZ_SIGN)
    parser.add_argument("--imu-yaw", type=float, default=IMU_YAW_DEG)
    args = parser.parse_args()
    if args.check_axes and args.dry_run:
        parser.error("--check-axes moves the robot; it cannot be combined with --dry-run")
    if not 0 < args.max_speed <= 0.4 or not 0 < args.max_theta <= 150:
        parser.error("max speed must be in (0, 0.4] m/s and max theta in (0, 150] deg/s")

    p = ApproachParams(stop_distance=args.stop_distance, max_speed=args.max_speed, max_theta=args.max_theta,
                       min_snr=args.min_snr, max_range=args.max_range, gate=args.gate)
    plotter = Plotter(dict(max_range=p.max_range, max_az=p.max_az, stop_distance=p.stop_distance),
                      enabled=not (args.no_plot or args.check_axes))
    config = load_config(args.config)
    imu = ImuStream(config["imu"])
    radar = RadarStream(config["radar"])
    bus = ServoBus(config["arm"])
    base = Base(WheelDrive(bus), args.dry_run)
    try:
        imu.start()
        radar.start()
        wait_sensors(imu, radar)
        if args.check_axes:
            check_axes(imu, radar, base, p)
        else:
            approach(imu, radar, base, p, args, plotter)
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


class MovingLogicTest(unittest.TestCase):
    P = ApproachParams()

    @staticmethod
    def radar(*points):
        return {"points_xyzd": [[x, y, 0, 0] for x, y, _ in points], "snr_db": [s for _, _, s in points]}

    def test_selects_highest_snr_within_limits(self):
        # too close, outside azimuth, too far, and missing SNR are ignored
        radar = self.radar((0, 1, 15), (0.5, 1, 20), (0, 0.1, 40), (3, 0.5, 50), (0, 5, 60))
        radar["points_xyzd"].append([0, 2, 0, 0])
        radar["snr_db"].append(None)
        self.assertEqual(select_target(radar, self.P).snr_db, 20)
        self.assertIsNone(select_target(self.radar((0, 1, 5)), self.P))

    def test_gate_rejects_jump_to_other_object(self):
        last = Target(0, 1, 1, 0, 15)
        self.assertEqual(select_target(self.radar((0, 1.05, 12), (1, 2, 30)), self.P, last).snr_db, 12)

    def test_target_ahead_drives_straight(self):
        x, y, theta, arrived = approach_command(Target(0, 2, 2, 0, 20), self.P)
        self.assertAlmostEqual(x, self.P.max_speed)
        self.assertAlmostEqual(y, 0)
        self.assertEqual(theta, 0)
        self.assertFalse(arrived)

    def test_target_right_turns_clockwise_and_moves_right(self):
        x, y, theta, _ = approach_command(Target(0.7, 1.9, 2, 20, 20), self.P)
        self.assertLess(theta, 0)
        self.assertLess(y, 0)
        self.assertGreater(x, 0)
        _, _, theta_mirrored, _ = approach_command(Target(0.7, 1.9, 2, 20, 20), self.P, az_sign=-1)
        self.assertGreater(theta_mirrored, 0)

    def test_large_azimuth_rotates_in_place_and_arrival_stops(self):
        x, y, theta, _ = approach_command(Target(-1.5, 1.5, 2, -45, 20), self.P)
        self.assertAlmostEqual(math.hypot(x, y), 0)
        self.assertEqual(theta, self.P.max_theta)
        self.assertEqual(approach_command(Target(0, .5, .5, 0, 20), self.P), (0, 0, 0, True))
        self.assertTrue(valid_motion(dict(zip("x y theta".split(), approach_command(
            Target(0, 4, 4, 0, 20), self.P)[:3]), duration_ms=1)))

    def test_imu_frames(self):
        imu = {"velocity_m_s": [0, .1, 0], "orientation_deg": [0, 0, 90], "gyro_corrected_deg_s": [0, 0, -10]}
        np.testing.assert_allclose(body_velocity(imu, np.zeros(2), 0), [.1, 0], atol=1e-12)
        np.testing.assert_allclose(body_velocity(imu, np.zeros(2), 90), [0, .1], atol=1e-12)
        self.assertAlmostEqual(yaw_rate(imu, np.array([0, 0, -1])), 10)

    def test_radar_and_global_frames(self):
        # TI: +y ahead, +x right -> robot: +x ahead, -y right
        np.testing.assert_allclose(radar_xy_to_robot([[0, 2], [1, 0]], 0, 1, (0, 0)), [[2, 0], [0, -1]], atol=1e-12)
        np.testing.assert_allclose(radar_xy_to_robot([[1, 0]], 0, -1, (0.1, 0)), [[0.1, 1]], atol=1e-12)
        np.testing.assert_allclose(transform([[1, 0]], (1, 2, 90)), [[1, 3]], atol=1e-12)
        self.assertEqual(transform([], (1, 2, 90)).shape, (0, 2))

    def test_imu_pose_removes_start_drift(self):
        start = {"estimate_valid": True, "displacement_m": [1, 1, 0], "velocity_m_s": [.1, 0, 0],
                 "orientation_deg": [0, 0, 0]}
        tracker = ImuPose(start, 0, 0)
        later = {**start, "displacement_m": [1.3, 1, 0], "orientation_deg": [0, 0, 190]}
        x, y, yaw = tracker.pose(later, 1.0)
        self.assertAlmostEqual(x, .2)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(yaw, -170)
        self.assertAlmostEqual(ImuPose(start, 90, 0).pose(later, 1.0)[1], .2)
        self.assertIsNone(tracker.pose({**later, "estimate_valid": False}, 1.0))

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

    def test_plot_view_renders(self):
        try:
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            from matplotlib.figure import Figure
        except ImportError:
            self.skipTest("matplotlib not installed")
        fig = Figure(figsize=(10, 5))
        FigureCanvasAgg(fig)
        view = PlotView(fig, dict(max_range=4, max_az=60, stop_distance=.5))
        view.handle({"type": "map", "points": np.array([[1, 0], [2, 1]])})
        view.handle({"type": "frame", "points": [[1, 0]], "target": (1, 0), "pose": (.5, .2, 30),
                     "cmd_pose": (.4, 0, 0), "text": "moving"})
        view.handle({"type": "frame", "points": np.empty((0, 2)), "target": None, "pose": None,
                     "cmd_pose": (9, 0, 0), "text": "drifting"})
        view.handle(None)
        fig.canvas.draw()
        self.assertEqual(len(view.traj_xy), 1)
        self.assertGreater(view.ax_glob.get_ylim()[1], 9)
        self.assertTrue(view.title.get_text().endswith("[finished]"))


if __name__ == "__main__":
    main()
