import importlib
from pathlib import Path
import sys


class OrinRobot:
    """Adapter for the existing ORIN/client hardware and action handlers."""

    def __init__(self, client_dir: Path, mock: bool = False):
        sys.path.insert(0, str(client_dir.resolve()))
        self.handlers = importlib.import_module("handlers")
        hardware = importlib.import_module("hardware")
        self.context = hardware.RobotContext(self.handlers.DEFAULT_ROBOT_CONFIG, mock=mock)

    def __enter__(self):
        try:
            self.context.start()
        except Exception:
            self.context.close()
            raise
        return self

    def __exit__(self, *_):
        self.context.close()

    def observe(self) -> dict:
        return self.handlers.observe(self.context)

    def execute(self, command: dict) -> bool:
        action = command.get("action")
        handler = self.handlers.ACTION_HANDLERS.get(action) if isinstance(action, str) else None
        return bool(handler and handler(self.context, command))
