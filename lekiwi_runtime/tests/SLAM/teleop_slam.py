"""Focused-window WASD/Q/E base teleoperation with supervised monocular SLAM.

Defaults to a no-hardware simulation. --live requires an explicit servo port.
Only wheel IDs 7/8/9 are configured; no arm, radar or IMU object is created.
"""

import argparse
import csv
from datetime import datetime
import json
import math
import multiprocessing as mp
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "src"))
from lekiwi_runtime.drivers.servo import ServoBus
from lekiwi_runtime.drivers.wheels import GOAL_VELOCITY, WHEEL_IDS, WheelDrive, body_to_wheel_raw, valid_motion
from lekiwi_runtime.config import load_config

ZERO = (0.0, 0.0, 0.0)
KEYS = set("wasdqe")
COMMAND_TTL = 0.30
SLAM_TTL = 2.5  # mono_usb flushes tracking.csv approximately once a second.


def key_velocity(keys, speed=0.06, turn=12.0):
    x = float("w" in keys) - float("s" in keys)
    y = float("a" in keys) - float("d" in keys)
    norm = max(1.0, math.hypot(x, y))
    return x * speed / norm, y * speed / norm, (float("q" in keys) - float("e" in keys)) * turn


def servo_settings(config_path=None, port_override=None):
    """Use the same config['arm'] bus settings as tests/test_moving.py.

    Reading arm bus configuration does not create or control an ArmController.
    An explicit port override wins; automatic bus scanning remains disabled.
    """
    arm = load_config(config_path)["arm"]
    configured_port = port_override if port_override is not None else arm.get("port")
    baudrate = arm["baudrate"]
    if type(baudrate) is not int or baudrate <= 0:
        raise ValueError("arm.baudrate must be a positive integer")
    return (Path(configured_port).expanduser() if configured_port else None), baudrate


def decode_command(packet, now):
    """Only fresh, finite, bounded commands can reach a wheel driver."""
    try:
        data = json.loads(packet)
        if data.get("kind") == "quit":
            return None
        if data.get("kind") != "velocity":
            return ZERO
        stamp = data["at"]
        velocity = tuple(data["velocity"])
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            return ZERO
        if not 0 <= now - stamp <= COMMAND_TTL:
            return ZERO
        if len(velocity) != 3 or not valid_motion(dict(zip(("x", "y", "theta", "duration_ms"), (*velocity, 1)))):
            return ZERO
        if math.hypot(*velocity[:2]) > 0.15 or abs(velocity[2]) > 30:
            return ZERO
        return velocity
    except (ValueError, TypeError, KeyError, AttributeError):
        return ZERO


def send_packet(channel, data):
    try:
        channel.send(json.dumps(data).encode())
        return True
    except (BlockingIOError, BrokenPipeError, ConnectionRefusedError, OSError):
        return False


class ContinuousWheels(WheelDrive):
    """Follow tests/test_moving.py Base.send()/stop() velocity-session design.

    Prepare once per active session, use the existing body_to_wheel_raw and
    GroupSyncWrite, then zero/release all wheels on stop or communication error.
    """
    def set_velocity(self, velocity):
        if len(velocity) != 3 or not valid_motion(dict(zip(("x", "y", "theta", "duration_ms"), (*velocity, 1)))):
            raise ValueError("Invalid velocity")
        if not any(velocity):
            self.close()
            return
        if self.mock:
            return
        import scservo_sdk as sdk
        try:
            if not self._active:
                self._prepare()
            writer = sdk.GroupSyncWrite(self.bus.port_handler, self.bus.packet_handler, GOAL_VELOCITY, 2)
            for motor_id, value in body_to_wheel_raw(*velocity).items():
                if not writer.addParam(motor_id, [value & 0xff, value >> 8]):
                    raise OSError(f"Cannot encode wheel {motor_id}")
            if writer.txPacket() != sdk.COMM_SUCCESS:
                raise OSError("Wheel velocity transmission failed")
        except BaseException:
            self.close(release_all=True)
            raise


def motor_worker(channel, port, mock, log_path, baudrate=1000000):
    """Independent command-expiry loop; never depends on Tk or SLAM threads."""
    bus = ServoBus({"port": port, "baudrate": baudrate}, motor_ids=WHEEL_IDS)
    drive = ContinuousWheels(bus, mock=mock)
    channel.setblocking(False)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    current, packet = ZERO, None
    last_report = 0.0
    try:
        with open(log_path, "x", newline="") as log:
            writer = csv.writer(log)
            writer.writerow(("host_monotonic_s", "vx_m_s", "vy_m_s", "theta_deg_s", "mock"))
            while True:
                started = time.monotonic()
                if select.select([channel], [], [], 0.05)[0]:
                    while True:
                        try:
                            incoming = channel.recv(4096)
                        except BlockingIOError:
                            break
                        if decode_command(incoming, time.monotonic()) is None:
                            return
                        packet = incoming
                now = time.monotonic()
                target = decode_command(packet, now) if packet is not None else ZERO
                if target is None:
                    return
                # An explicit stop or an expired command brakes immediately.
                if not any(target):
                    updated = ZERO
                else:
                    dt = min(max(now - started, 0.01), 0.1)
                    delta_x, delta_y = target[0] - current[0], target[1] - current[1]
                    scale = min(1.0, 0.2 * dt / max(math.hypot(delta_x, delta_y), 1e-12))
                    yaw_step = max(-45 * dt, min(45 * dt, target[2] - current[2]))
                    updated = (current[0] + delta_x * scale, current[1] + delta_y * scale, current[2] + yaw_step)
                drive.set_velocity(updated)
                current = updated
                writer.writerow((now, *current, mock))
                log.flush()
                if now - last_report >= 0.2:
                    send_packet(channel, {"kind": "status", "velocity": current})
                    last_report = now
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        send_packet(channel, {"kind": "error", "message": str(exc)})
        raise
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        try:
            drive.close(release_all=True)
            send_packet(channel, {"kind": "stopped"})
        except BaseException as exc:
            send_packet(channel, {"kind": "error", "message": "Stop failed; physical state unknown: " + str(exc)})
            raise
        finally:
            bus.close()
            channel.close()


class SlamHealth:
    def __init__(self):
        self.token = None
        self.changed = None
        self.state = "STARTING"

    def update(self, path, now):
        if path.is_file():
            with path.open("rb") as stream:
                stream.seek(max(0, path.stat().st_size - 4096))
                tail = stream.read()
            # Ignore a partially written last line.
            lines = tail.split(b"\n")[:-1]
            for line in reversed(lines):
                fields = line.decode(errors="replace").split(",")
                if len(fields) != 8:
                    continue
                try:
                    float(fields[0]); int(fields[1])
                except ValueError:
                    continue
                if fields[0] != self.token:
                    self.token, self.changed, self.state = fields[0], now, fields[2]
                break
        fresh = self.changed is not None and now - self.changed <= SLAM_TTL
        # Some translation is needed to initialize; allow NOT_INITIALIZED.
        return fresh and self.state in ("NOT_INITIALIZED", "OK")


def assert_unoccupied(port):
    result = subprocess.run(["fuser", str(port)], capture_output=True, text=True)
    if result.returncode == 0:
        raise RuntimeError(f"Servo port is already in use: {port}; PIDs: {result.stdout.strip()}")
    if result.returncode != 1:
        raise RuntimeError(f"Cannot check servo port occupancy: {result.stderr.strip()}")


class TeleopApp:
    def __init__(self, root, args):
        import tkinter as tk
        self.root, self.args = root, args
        self.keys, self.pending_release = set(), {}
        self.armed = False
        self.closing = False
        self.healthy = False
        self.health = SlamHealth()
        self.slam = None
        self.slam_log = None
        self.worker = None
        self.channel = None
        self.run = HERE / ("teleop_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self.run.mkdir(exist_ok=False)
        (self.run / "teleop.json").write_text(json.dumps({
            "live": args.live, "port": str(args.port) if args.port else None,
            "baudrate": args.baudrate,
            "speed_m_s": args.speed, "turn_deg_s": args.turn, "duration_s": args.seconds,
            "frame": "robot body: x forward, y left, theta counterclockwise",
            "command_timeout_s": COMMAND_TTL, "slam_timeout_s": SLAM_TTL,
            "wheel_ids": list(WHEEL_IDS),
            "keymap": {"w": "+vx", "s": "-vx", "a": "+vy", "d": "-vy", "q": "+theta", "e": "-theta"},
        }, indent=2), encoding="utf-8")
        self.message = tk.StringVar(value="시작 중 — 주행 비활성")
        self.motion = tk.StringVar(value="vx=0, vy=0, theta=0")
        root.title("LeKiwi WASD + Q/E SLAM" + (" [MOCK]" if not args.live else " [LIVE]"))
        tk.Label(root, text="W 전진 / S 후진 / A 왼쪽 / D 오른쪽\nQ 반시계 / E 시계 회전", font=("Sans", 16)).pack(padx=25, pady=15)
        tk.Label(root, text="키를 누르는 동안 이동 · Space 정지/해제 · Esc 전체 종료\n이 제어창을 클릭해 포커스를 유지하세요.").pack()
        tk.Label(root, text=f"속도 {args.speed:.2f} m/s / 회전 {args.turn:.1f} °/s · 로봇 기준 좌표").pack(pady=8)
        self.arm_button = tk.Button(root, text="주행 활성화", command=self.toggle_arm, state=tk.DISABLED)
        self.arm_button.pack(pady=8)
        tk.Button(root, text="정지 / 주행 해제 [Space]", command=lambda: self.disarm("사용자 정지"), bg="#efb1a9").pack(pady=8)
        tk.Label(root, textvariable=self.message, wraplength=550).pack(padx=10, pady=8)
        tk.Label(root, textvariable=self.motion).pack(pady=6)
        tk.Label(root, text="실물 구동 없음" if not args.live else f"바퀴 7·8·9만 제어: {args.port}").pack(pady=6)
        root.bind("<KeyPress>", self.key_press)
        root.bind("<KeyRelease>", self.key_release)
        root.bind("<FocusOut>", lambda event: root.after_idle(self.focus_check))
        root.protocol("WM_DELETE_WINDOW", self.close)
        try:
            if args.live:
                assert_unoccupied(args.port)
                self.slam_log = (self.run / "mono_usb.log").open("x")
                import os
                env = os.environ.copy()
                lib = str(Path.home() / "slam_ws/install/pangolin-0.8/lib")
                env["LD_LIBRARY_PATH"] = lib + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
                self.slam = subprocess.Popen([
                    str(args.binary), str(args.vocabulary), str(args.settings),
                    "--output", str(self.run / "slam"), "--seconds", str(args.seconds)
                ], stdout=self.slam_log, stderr=subprocess.STDOUT, env=env)
            context = mp.get_context("spawn")
            self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.channel.setblocking(False)
            self.worker = context.Process(target=motor_worker,
                args=(child, str(args.port) if args.port else None, not args.live,
                      str(self.run / "wheel_commands.csv"), args.baudrate))
            self.worker.start()
            child.close()
            self.started = time.monotonic()
            self.worker_seen = self.started
            print("결과:", self.run, flush=True)
            root.after(50, self.tick)
        except BaseException:
            self.close()
            raise

    def disarm(self, reason):
        self.armed = False
        self.keys.clear()
        for job in self.pending_release.values():
            self.root.after_cancel(job)
        self.pending_release.clear()
        self.arm_button.configure(text="주행 활성화")
        self.message.set(reason)
        self.send(ZERO)

    def send(self, velocity):
        return self.channel is not None and send_packet(self.channel,
            {"kind": "velocity", "at": time.monotonic(), "velocity": velocity})

    def toggle_arm(self):
        if self.armed:
            self.disarm("주행 해제")
        elif self.healthy and self.worker is not None and self.worker.is_alive():
            self.keys.clear()
            self.armed = True
            self.arm_button.configure(text="주행 해제")
            self.message.set("주행 활성: WASD/Q/E를 누르는 동안 이동")
            self.root.focus_set()

    def key_press(self, event):
        key = event.keysym.lower()
        if key == "escape":
            self.close()
        elif key == "space":
            self.disarm("Space 정지")
        elif key in KEYS:
            if key in self.pending_release:
                self.root.after_cancel(self.pending_release.pop(key))
            if self.armed:
                self.keys.add(key)

    def key_release(self, event):
        key = event.keysym.lower()
        if key in KEYS:
            # X11 autorepeat emits adjacent release/press events: cancel the
            # pending release when its matching repeat press arrives.
            if key in self.pending_release:
                self.root.after_cancel(self.pending_release.pop(key))
            self.pending_release[key] = self.root.after_idle(lambda: self.release_key(key))

    def release_key(self, key):
        self.pending_release.pop(key, None)
        self.keys.discard(key)
        if self.armed:
            self.send(key_velocity(self.keys, self.args.speed, self.args.turn))

    def focus_check(self):
        if self.closing:
            return
        focused = self.root.focus_displayof()
        if focused is None or focused.winfo_toplevel() != self.root:
            self.disarm("포커스를 잃어 정지했습니다. 제어창에서 다시 활성화하세요.")

    def tick(self):
        if self.closing:
            return
        now = time.monotonic()
        try:
            while True:
                data = json.loads(self.channel.recv(4096))
                self.worker_seen = now
                if data["kind"] == "error":
                    raise RuntimeError(data["message"])
                if data["kind"] == "status":
                    x, y, theta = data["velocity"]
                    self.motion.set(f"명령 vx={x:+.3f} m/s, vy={y:+.3f} m/s, theta={theta:+.1f} °/s")
        except BlockingIOError:
            pass
        except (OSError, ValueError, RuntimeError) as exc:
            self.message.set(str(exc)); print("ERROR:", exc, flush=True)
            self.close(); return
        if not self.worker.is_alive() or now - self.worker_seen > 1.0:
            print("Motor worker stopped or unresponsive; closing.", flush=True)
            self.close(); return
        if self.slam is not None and self.slam.poll() is not None:
            print("SLAM ended:", self.slam.returncode, "log:", self.run / "mono_usb.log", flush=True)
            self.close(); return
        if now - self.started >= self.args.seconds:
            self.close(); return
        self.healthy = not self.args.live or self.health.update(self.run / "slam/tracking.csv", now)
        if not self.healthy and self.armed:
            self.disarm("SLAM 영상 갱신 중단 또는 Tracking Lost: 정지했습니다.")
        if not self.armed:
            self.message.set("주행 비활성 | SLAM: " + (self.health.state if self.args.live else "MOCK"))
        self.arm_button.configure(state="normal" if self.healthy else "disabled")
        velocity = key_velocity(self.keys, self.args.speed, self.args.turn) if self.armed and self.healthy else ZERO
        if not self.send(velocity):
            self.disarm("제어 통신 실패")
        self.root.after(50, self.tick)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.disarm("종료 중")
        if self.channel is not None:
            send_packet(self.channel, {"kind": "quit"})
        if self.worker is not None:
            self.worker.join(timeout=3)
            if self.worker.is_alive():
                print("WARNING: wheel worker has not exited; physical stop is unconfirmed. Do not force-kill for cleanup.", file=sys.stderr, flush=True)
            elif self.worker.exitcode:
                print("ERROR: wheel worker failed; inspect stop-error messages. Physical stop is not confirmed by software.", file=sys.stderr, flush=True)
        if self.slam is not None and self.slam.poll() is None:
            self.slam.send_signal(signal.SIGINT)
            try:
                self.slam.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print("SLAM is still completing shutdown/export; it was not force-killed.", flush=True)
        if self.channel is not None:
            self.channel.close()
        if self.slam_log is not None:
            self.slam_log.close()
        self.root.destroy()
        print("종료. 결과:", self.run, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--mock", action="store_true", help="GUI·키 입력만 모의 검증; Camera/Serial 접근 없음 (기본)")
    mode.add_argument("--live", action="store_true", help="실제 Camera SLAM과 바퀴 구동")
    parser.add_argument("--port", type=Path, help="바퀴 보드의 명시적 Serial 장치")
    parser.add_argument("--config", type=Path, help="test_moving.py와 같은 장비별 robot JSON; arm.port/baudrate 사용")
    parser.add_argument("--speed", type=float, default=0.06)
    parser.add_argument("--turn", type=float, default=12)
    parser.add_argument("--seconds", type=float, default=300)
    parser.add_argument("--binary", type=Path, default=Path.home() / "slam_ws/mono_usb_build/mono_usb")
    parser.add_argument("--vocabulary", type=Path, default=Path.home() / "slam_ws/ORB_SLAM3/Vocabulary/ORBvoc.txt")
    parser.add_argument("--settings", type=Path, default=HERE / "calibration_result_20261113_48views/OV5648.yaml")
    args = parser.parse_args()
    try:
        args.port, args.baudrate = servo_settings(args.config, args.port)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    if not math.isfinite(args.speed) or not 0 < args.speed <= 0.15:
        parser.error("speed must be in (0, 0.15] m/s")
    if not math.isfinite(args.turn) or not 0 < args.turn <= 30:
        parser.error("turn must be in (0, 30] deg/s")
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 1800:
        parser.error("seconds must be in (0, 1800]")
    if args.live:
        if args.port is None or not args.port.exists():
            parser.error("--live requires an existing --port or config arm.port; no automatic Serial scan is performed")
        args.port = args.port.resolve()
        for name in ("binary", "vocabulary", "settings"):
            path = getattr(args, name).expanduser().resolve()
            if not path.is_file():
                parser.error(f"{name} not found: {path}")
            setattr(args, name, path)
    import tkinter as tk
    root = tk.Tk()
    app = TeleopApp(root, args)
    def interrupted(signum, frame):
        root.after(0, app.close)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    root.mainloop()


if __name__ == "__main__":
    main()
