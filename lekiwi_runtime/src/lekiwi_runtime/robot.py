from typing import Protocol


BASE_MAX_DURATION_MS = 1000
ARM_MAX_DURATION_MS = 10000


class PreDispatchRejected(ValueError):
    """A driver validation failure certified to precede all motor writes.

    Raise only while checking inputs or read-only device state, before goal,
    torque, configuration or base commands. Communication and post-write
    failures must keep their original exception and uncertain outcome.
    """
    def __init__(self, code: str, message: str, **details):
        super().__init__(message)
        self.code = code
        self.details = details


class Robot(Protocol):
    def observe(self) -> dict: ...
    def execute(self, command: dict) -> bool: ...
