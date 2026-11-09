"""On-device sensor lifecycle, observations and supported actions."""

import time

from .config import load_config
from .drivers.arm import ArmController
from .drivers.imu import ImuStream
from .drivers.servo import ServoBus
from .drivers.wheels import WheelDrive, valid_motion
from .drivers.camera import CameraStream
from .drivers.radar import RadarStream


class OrinRobot:
    def __init__(self, config: dict | None = None, mock: bool = False):
        config = load_config() if config is None else config
        self.mock = mock
        self.cameras = {name: CameraStream(name, settings, mock)
                        for name, settings in config["cameras"].items()}
        self.imu = ImuStream(config["imu"], mock)
        self.radar = RadarStream(config["radar"], mock)
        bus = ServoBus(config["arm"])
        self.arm = ArmController(config["arm"], mock, bus)
        self.wheels = WheelDrive(bus, mock)
        self._sensors = [*self.cameras.values(), self.radar, self.imu, self.arm]

    def __enter__(self):
        try:
            for sensor in self._sensors:
                sensor.start()
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                if all(camera.ready for camera in self.cameras.values()) and self.radar.ready and self.imu.ready:
                    break
                time.sleep(0.05)
        except BaseException:
            self.close()
            raise
        return self

    def close(self) -> None:
        # Attempt every close even if one device fails during cleanup.
        errors = []
        try:
            self.wheels.close(release_all=True)
        except Exception as exc:
            errors.append(exc)
        for sensor in reversed(self._sensors):
            try:
                sensor.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("Sensor cleanup failed") from errors[0]

    def __exit__(self, *_):
        self.close()

    def observe(self) -> dict:
        return {
            "timestamp": time.time(),
            "simulated": self.mock,
            "cameras": {name: camera.snapshot_jpeg_base64()
                        for name, camera in self.cameras.items()},
            "radar": self.radar.snapshot(),
            "ego": self.arm.read_ego(),
            "imu": self.imu.snapshot(),
        }

    def execute(self, command: dict) -> bool:
        if command.get("action") == "move":
            motion = {key: command.get(key, 0) for key in ("x", "y", "theta")}
            duration = command.get("duration_ms")
            if not valid_motion({**motion, "duration_ms": duration}):
                return False
            joints = command.get("joints")
            if joints is not None:
                if not self.arm.valid_target(joints, duration):
                    return False
                # One bus and thread; wheels run only during arm interpolation.
                base_motion = self.wheels.driving(**motion) if any(motion.values()) else None
                self.arm.move(joints, duration, base_motion=base_motion)
            else:
                self.wheels.move(motion["x"], motion["y"], motion["theta"], duration)
            return True
        if command.get("action") != "wait":
            return False
        if command.get("joints") is not None or any(command.get(k, 0) for k in ("x", "y", "theta")):
            return False
        duration = command.get("duration_ms")
        if type(duration) is not int or not 0 <= duration <= 1000:
            return False
        time.sleep(duration / 1000)
        return True
