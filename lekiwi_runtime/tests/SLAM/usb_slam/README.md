# OV5648 USB Monocular SLAM

Standalone C++ runner for the existing ORB-SLAM3 library. It does not import or
start the LeKiwi Python runtime. Camera access occurs only when `mono_usb` runs.
Use the Orin Desktop terminal for the Pangolin and current-frame windows.

## 1. Restore shutdown synchronization

The installed ORB-SLAM3 revision has its worker wait commented out in
`System::Shutdown()`. The supplied patch requests worker/viewer termination and
waits before trajectory export. It avoids waiting for the viewer from its own
thread when the viewer's Stop action calls Shutdown.

Check and apply once. Stop if either command fails; do not force application.

```bash
git -C "$HOME/slam_ws/ORB_SLAM3" apply --check \
  /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/usb_slam/orb_slam3_shutdown.patch

git -C "$HOME/slam_ws/ORB_SLAM3" apply \
  /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/usb_slam/orb_slam3_shutdown.patch

cmake --build "$HOME/slam_ws/ORB_SLAM3/build" \
  --target ORB_SLAM3 --parallel 1
```

The patch has been checked for applicability; runtime shutdown behavior still
needs validation. A running global BA may take time to finish. Do not use
SIGKILL if you need final trajectory export.

## 2. Build the separate runner

```bash
cmake \
  -S /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/usb_slam \
  -B "$HOME/slam_ws/mono_usb_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DORB_SLAM3_ROOT="$HOME/slam_ws/ORB_SLAM3" \
  -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DPangolin_DIR="$HOME/slam_ws/install/pangolin-0.8/lib/cmake/Pangolin"

cmake --build "$HOME/slam_ws/mono_usb_build" --parallel 1

LD_LIBRARY_PATH="$HOME/slam_ws/install/pangolin-0.8/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$HOME/slam_ws/mono_usb_build/mono_usb" --help
```

`--help` does not open the camera or create results. Configuration rejects an
unpatched System.cc or a library older than System.cc.

## 3. Live validation (explicit camera operation)

Close calibration and GStreamer previews. Check occupancy and focus first:

```bash
fuser -v /dev/video0 /dev/video1
v4l2-ctl \
  -d /dev/v4l/by-id/usb-Arducam_Arducam_5MP_USB_Camera_YL20240927V0-video-index0 \
  --get-ctrl=focus_automatic_continuous,focus_absolute
```

Do not run if another process owns the camera. Keep the calibrated focus
(autofocus off; reported focus was 0), resolution, orientation and lens unchanged.

```bash
LD_LIBRARY_PATH="$HOME/slam_ws/install/pangolin-0.8/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$HOME/slam_ws/mono_usb_build/mono_usb" \
  "$HOME/slam_ws/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  /home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/calibration_result_20261113_48views/OV5648.yaml \
  --output "/home/irsadmin/LeKiwi/lekiwi/lekiwi_runtime/tests/SLAM/usb_$(date +%Y%m%d_%H%M%S_%N)" \
  --seconds 120
```

Remove the checkerboard; use a well-lit scene with textured objects at different
depths. Move the camera slowly with some translation. Pure rotation or a single
blank wall may prevent monocular initialization. No motor commands are sent.
Terminate early with Ctrl+C in the terminal. Camera read/SLAM work can delay
signal handling until that operation returns. Closing a window via its titlebar
is not the supported stop mechanism.

`--no-viewer` allows headless capture and logging. Do not run two instances on
the same camera. The capture buffer size is requested, but exact driver latency
is not guaranteed. Three seconds of identical decoded frames stops the run;
this cannot diagnose the underlying USB/driver problem or interrupt a blocked read.

## Results and interpretation

- `OV5648.yaml`: exact settings copy used for this run.
- `session.txt`: input paths, reported camera properties, outcome and counts.
- `tracking.csv`: every processed frame's state, tracking time, loop FPS,
  keypoints, valid-pose flag and big-map-change notification.
- `CameraTrajectoryOnlineTUM.txt`: only Tracking::OK poses, converted from Tcw
  to Twc, with `timestamp tx ty tz qx qy qz qw` ordering.
- `KeyFrameTrajectoryTUM.txt`: ORB-SLAM3 TUM keyframe export after shutdown;
  only requested if at least one valid pose was observed.

Successful validation requires a sustained OK state, map points visible in the
viewer and nonempty trajectory output. No valid pose makes the runner exit with
failure, even if capture itself worked. The viewer's Stop action may additionally
write its built-in EuRoC exports; these are different from the TUM files above.

Timestamps are host steady-clock seconds after frame reception, relative to
capture start. They are not hardware exposure timestamps or synchronized IMU
timestamps. Translation has arbitrary monocular scale and describes the camera,
not the robot body. Online poses are not retrospectively corrected after loop
closure. Atlas resets/new maps can change the world frame; neither the online
file nor concatenated keyframes should be assumed to form one continuous metric
trajectory. Sparse map display does not create a navigation occupancy map or a
saved landmark-map artifact.
