"""On-device sensor lifecycle, observations and supported actions."""

import time

from .config import load_config
from .drivers.arm import ArmEgoReader
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
        self.radar = RadarStream(config["radar"], mock)
        bus = ServoBus(config["arm"])
        self.arm = ArmEgoReader(config["arm"], mock, bus)
        self.wheels = WheelDrive(bus, mock)
        self._sensors = [*self.cameras.values(), self.radar, self.arm]

    def __enter__(self):
        try:
            for sensor in self._sensors:
                sensor.start()
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline:
                if all(camera.ready for camera in self.cameras.values()) and self.radar.ready:
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
            self.wheels.close()
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
        }

    def execute(self, command: dict) -> bool:
        if command.get("action") == "move":
            if not valid_motion(command):
                return False
            self.wheels.move(command["x"], command["y"], command["theta"], command["duration_ms"])
            return True
        if command.get("action") != "wait":
            return False
        duration = command.get("duration_ms")
        if type(duration) is not int or not 0 <= duration <= 1000:
            return False
        time.sleep(duration / 1000)
        return True
