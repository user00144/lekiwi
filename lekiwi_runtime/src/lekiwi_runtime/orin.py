"""On-device sensor lifecycle, observations and supported actions."""

import time
from copy import deepcopy

from .config import load_config
from .drivers.arm import ARM_CONTROL_REVISION, ArmController, ArmTargetNotReached
from .drivers.imu import ImuStream
from .drivers.servo import ServoBus
from .drivers.wheels import WheelDrive, valid_motion
from .drivers.camera import CameraStream
from .drivers.radar import RadarStream
from .robot import BASE_MAX_DURATION_MS


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
            joints = command.get("joints")
            if joints is not None:
                # Validate velocity independently: arm-only interpolation may
                # take up to 10 s, while any base motion still stops within 1 s.
                if (not valid_motion({**motion, "duration_ms": 1})
                        or not self.arm.valid_target(joints, duration)
                        or any(motion.values()) and duration > BASE_MAX_DURATION_MS):
                    return False
                # One bus and thread; wheels run only during arm interpolation.
                base_motion = self.wheels.driving(**motion) if any(motion.values()) else None
                self.arm.move(joints, duration, base_motion=base_motion)
            else:
                if not valid_motion({**motion, "duration_ms": duration}):
                    return False
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

    def execute_guarded(self, command: dict) -> dict:
        """Report what this driver verifies; base pose/stationarity are unknown.

        PreDispatchRejected certifies a validation failure before motor writes.
        An exact endpoint failure may report incomplete only after measured
        progress and a verified hold; it remains distinct from completion.
        Communication errors and partly transmitted commands retain uncertainty.
        """
        moving = command.get("action") == "move"
        uses_arm = moving and command.get("joints") is not None
        uses_base = moving and (not uses_arm or any(command.get(axis, 0) for axis in ("x", "y", "theta")))
        try:
            accepted = self.execute(command)
        except ArmTargetNotReached as failure:
            feedback = failure.feedback
            hold = feedback.get('hold')
            # Only this exact driver failure, after measured progress and an
            # independently verified hold, can be reconciled by fresh feedback.
            # No goal or transport exception is inferred safe from its message.
            if (type(failure) is not ArmTargetNotReached or not uses_arm or uses_base
                    or 'gripper' in command['joints'] or feedback.get('targets') != command['joints']
                    or feedback != self.arm.last_motion or feedback.get('recovery_eligible') is not True
                    or not isinstance(hold, dict) or hold.get('verified') is not True
                    or hold.get('revision') != ARM_CONTROL_REVISION
                    or hold.get('calibration_id') != self.arm.calibration_id):
                raise
            return {'outcome': 'incomplete', 'command_sent': True, 'arm_reached': False,
                    'arm_hold_verified': True, 'arm_motion': deepcopy(feedback),
                    'base_stop_command_sent': None, 'base_stop_verified': None,
                    'simulated': self.mock}
        result = {
            "outcome": "completed" if accepted else "rejected",
            "command_sent": bool(accepted and moving),
            "arm_reached": bool(self.arm.last_motion and self.arm.last_motion.get("reached")) if accepted and uses_arm else None,
            "base_stop_command_sent": True if accepted and uses_base else None,
            "base_stop_verified": False if uses_base else None,
            "simulated": self.mock,
        }
        if accepted and uses_arm:
            result["arm_motion"] = dict(self.arm.last_motion)
        return result

    def reconcile_arm_hold(self, certificate: dict) -> dict:
        """Read-only recheck of a prior hold; the guard obtains new sensors next."""
        return self.arm.reconcile_hold(certificate)
