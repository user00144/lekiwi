"""Bounded body-velocity commands for the three STS3215 omni wheels."""

import sys

import math
import time

from .servo import ServoBus


WHEEL_IDS = (7, 8, 9)
GOAL_VELOCITY = 46
TORQUE_ENABLE = 40


def valid_motion(command: dict) -> bool:
    duration = command.get("duration_ms")
    values = [command.get(key) for key in ("x", "y", "theta")]
    if type(duration) is not int or not 1 <= duration <= 1000:
        return False
    if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
        return False
    x, y, theta = values
    return math.hypot(x, y) <= 0.4 and abs(theta) <= 150


def body_to_wheel_raw(x: float, y: float, theta: float) -> dict[int, int]:
    """m/s, m/s, deg/s -> signed-magnitude speed registers, capped at 3000."""
    rotation = math.radians(theta)
    speeds = [
        (math.cos(math.radians(angle - 90)) * x
         + math.sin(math.radians(angle - 90)) * y + 0.125 * rotation)
        / 0.05 * 4096 / (2 * math.pi)
        for angle in (240, 0, 120)
    ]
    scale = max(1, max(abs(speed) for speed in speeds) / 3000)
    result = {}
    for motor_id, speed in zip(WHEEL_IDS, speeds):
        raw = round(speed / scale)
        result[motor_id] = abs(raw) | (0x8000 if raw < 0 else 0)
    return result


class WheelDrive:
    def __init__(self, bus: ServoBus, mock: bool = False):
        self.bus = bus
        self.mock = mock
        self._active = False
        self._configured_bus = None

    def _write(self, motor_id: int, address: int, value: int, size: int = 1) -> None:
        import scservo_sdk as sdk

        write = (self.bus.packet_handler.write1ByteTxRx if size == 1
                 else self.bus.packet_handler.write2ByteTxRx)
        comm, error = write(self.bus.port_handler, motor_id, address, value)
        if comm != sdk.COMM_SUCCESS or error:
            raise OSError(f"Wheel {motor_id} write {address}: communication={comm}, device={error}")

    def _prepare(self) -> None:
        import scservo_sdk as sdk

        self.bus.connect()
        if self._configured_bus is not self.bus.port_handler:
            for motor_id in WHEEL_IDS:
                _, comm, error = self.bus.packet_handler.ping(self.bus.port_handler, motor_id)
                if comm != sdk.COMM_SUCCESS or error:
                    raise OSError(f"Wheel {motor_id} is unavailable")
            self._active = True
            for motor_id in WHEEL_IDS:
                self._write(motor_id, TORQUE_ENABLE, 0)
                self._write(motor_id, 55, 0)  # Unlock EEPROM.
                self._write(motor_id, 7, 0)  # Return delay.
                self._write(motor_id, 33, 1)  # Velocity mode.
                self._write(motor_id, 9, 0, 2)
                self._write(motor_id, 11, 0, 2)
                self._write(motor_id, 85, 254)  # Maximum acceleration.
                self._write(motor_id, 55, 1)
                self._write(motor_id, 41, 254)
            self._configured_bus = self.bus.port_handler
        self._active = True
        for motor_id in WHEEL_IDS:
            self._write(motor_id, GOAL_VELOCITY, 0, 2)
            self._write(motor_id, TORQUE_ENABLE, 1)

    def move(self, x: float, y: float, theta: float, duration_ms: int) -> None:
        if not valid_motion(dict(x=x, y=y, theta=theta, duration_ms=duration_ms)):
            raise ValueError("Invalid body velocity or duration")
        print(f'Motion command x={x} y={y} theta={theta} duration_ms={duration_ms} mock={self.mock}', file=sys.stderr, flush=True)
        if self.mock:
            time.sleep(duration_ms / 1000)
            return
        import scservo_sdk as sdk

        try:
            self._prepare()
            writer = sdk.GroupSyncWrite(self.bus.port_handler, self.bus.packet_handler, GOAL_VELOCITY, 2)
            for motor_id, value in body_to_wheel_raw(x, y, theta).items():
                if not writer.addParam(motor_id, [value & 0xff, value >> 8]):
                    raise OSError(f"Cannot build wheel command for {motor_id}")
            if writer.txPacket() != sdk.COMM_SUCCESS:
                raise OSError("Wheel velocity transmission failed")
            time.sleep(duration_ms / 1000)
        finally:
            self.close()

    def close(self) -> None:
        if self.mock or not self._active:
            return
        errors = []
        for motor_id in WHEEL_IDS:
            # Attempt every wheel and torque release even after a failed stop write.
            for address, value, size in ((GOAL_VELOCITY, 0, 2), (TORQUE_ENABLE, 0, 1), (55, 1, 1)):
                try:
                    self._write(motor_id, address, value, size)
                except Exception as exc:
                    errors.append(exc)
        self._active = bool(errors)
        if errors:
            print('Wheel stop/release failed; physical state unknown', file=sys.stderr, flush=True)
            raise OSError("Wheel stop/release failed; physical state unknown") from errors[0]
        print('Wheels stopped and torque released', file=sys.stderr, flush=True)
