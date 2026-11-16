from typing import Protocol


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
