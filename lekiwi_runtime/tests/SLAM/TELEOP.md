# Keyboard driving with USB monocular SLAM

Run `teleop_slam.py` from the existing runtime Python environment. No C++ rebuild,
uv sync or new package installation is needed. The wrapper launches the already
built `mono_usb` and a separate wheel worker, with a Tk keyboard-control window.
This is manual robot-body velocity control, not navigation to a SLAM world goal.

| Key held | Command in the robot body frame |
| --- | --- |
| W / S | forward +vx / backward -vx |
| A / D | left +vy / right -vy, without intentional rotation |
| Q / E | counterclockwise +theta / clockwise -theta |
| W+A, etc. | combined motion; diagonal translation is normalized |
| Space | brake, clear held keys, disarm |
| Esc / window close | stop wheels, end SLAM, export trajectory |

Translation is in m/s and theta in deg/s. Defaults are 0.06 m/s and 12 deg/s.
Direction follows the robot as it rotates; world-aligned translation would need
camera-to-body calibration and additional heading handling. Wheel geometry and
motor directions come from the existing `body_to_wheel_raw` implementation.
The continuous motor session follows `tests/test_moving.py`'s `Base.send()/stop()`:
prepare once, transmit GroupSyncWrite velocities, then zero/release the wheels.
Regression tests compare the exact motor payload and setup/stop writes with that
reference for forward/back/sideways/rotation/combined commands. R has no action.

## 1. No-hardware keyboard check

In the Orin Desktop terminal:

```bash
/home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/.venv/bin/python -B \
  /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/teleop_slam.py --mock
```

This mode opens no camera or Serial device and launches no SLAM process. Click
the control window and click **주행 활성화**, then hold WASD/Q/E. The velocity label
and wheel_commands.csv show simulated commands. Verify release, Space and loss
of window focus all stop/disarm the control as intended. The viewer is a separate
window in live mode; keyboard driving requires focus on the control window.

## 2. Identify the wheel board

```bash
/home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/.venv/bin/python -B \
  -m serial.tools.list_ports -v
```

Use the actual wheel-board device (prefer its stable `/dev/serial/by-id/...`
path). The tool never guesses a Serial port and never scans/pings the radar or
arm ports. Live mode requires an explicit existing `--port` or a configured
`arm.port`, checks it using `fuser` and configures/pings only wheel IDs 7/8/9.
As in test_moving.py, `--config /path/to/robot.json` reads the arm bus's port and
baudrate (default 1,000,000); `--port` overrides only the configured port. No arm
controller is created. Do not run
another runtime/teleop process on the same servo bus or camera. The occupancy
check reflects that moment and cannot lock out unrelated future processes.

## 3. Live driving and SLAM

Replace `ACTUAL_WHEEL_PORT` with the verified device path:

```bash
/home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/.venv/bin/python -B \
  /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/teleop_slam.py \
  --live --port ACTUAL_WHEEL_PORT --speed 0.06 --turn 12 --seconds 300
```

Close existing calibration, GStreamer and mono_usb instances before launch.
Keep the camera fixed to the base with the calibrated focus/orientation.
Wait for SLAM frames, click the control window and enable driving. Motion is
allowed while NOT_INITIALIZED (translation is needed to initialize) or OK.
LOST/RECENTLY_LOST or a stale SLAM log disarms motion; recovery requires manual
re-enabling. No robot arm torque or joint goal is changed by this tool.

Normal key release sends zero velocity. Focus loss and Space also disarm. Motor
commands expire in the independent wheel process after 0.30 seconds without fresh
GUI messages. SLAM progress is checked through tracking.csv (flushed about once
a second); the stale-progress threshold is 2.5 seconds. A SLAM process exit also
stops the session. The worker ramps up translation/rotation and does not cycle
torque on each update; stopping sends zero and releases wheel torque.

These are software stop attempts, not a hardware watchdog: a blocked SDK call,
USB failure, SIGKILL or power loss can prevent a stop packet from reaching the
motors. The program reports failed/unconfirmed cleanup instead of claiming a
physical stop. Existing wheel preparation writes velocity-mode settings using
the existing WheelDrive implementation. Start with clear space and low speed.

## Outputs

Each invocation creates `tests/SLAM/teleop_TIMESTAMP/` containing:

- `teleop.json`: mode, port, speed, coordinate frame and watchdog thresholds.
- `wheel_commands.csv`: monotonic host time and issued/simulated body velocity.
- `mono_usb.log`: live SLAM stdout/stderr, including initialization errors.
- `slam/`: the usual copied YAML, session report, tracking.csv, online camera
  poses and final keyframe trajectory from mono_usb.

Commanded velocity is not measured odometry. The wheel log uses absolute host
monotonic seconds; SLAM uses seconds relative to capture start. Do not directly
join those timestamp columns without estimating their offset. Camera SLAM has
arbitrary monocular scale; this feature does not add metric localization or map
navigation.

## No-hardware regression checks

```bash
cd /home/irsadmin/LeKiwi/lekiwi
./lekiwi_runtime/.venv/bin/python -B -m unittest discover \
  -s lekiwi_runtime/tests/SLAM -p test_teleop_slam.py -v
```

The checks exercise mapping, diagonal normalization, malformed/stale command
rejection, X11 repeat/key release, focus/Space stop, SLAM health expiry, mocked
servo failures and real-process command expiry with mocked wheels. Real GUI and
motor behavior still require operator validation. Tests also compare exact
motor writes against test_moving.py and verify per-robot bus config overrides.
