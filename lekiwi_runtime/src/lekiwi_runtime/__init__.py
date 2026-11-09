import sys

from .client import RobotClient
from .robot import Robot

__all__ = ["Robot", "RobotClient"]


def main():
    import argparse
    import signal
    from pathlib import Path
    from .orin import OrinRobot
    from .config import load_config

    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="tcp://127.0.0.1:5555", help="Robot server endpoint (default: local port 5555)")
    parser.add_argument("--robot-id", default="kiwi-01")
    parser.add_argument("--config", type=Path, help="Per-robot JSON configuration overrides")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    def interrupt(signum, frame):
        # Let cleanup finish even when Ctrl+C is pressed repeatedly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    print(f'Starting robot runtime robot={args.robot_id} mock={args.mock}', file=sys.stderr, flush=True)
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with OrinRobot(load_config(args.config), mock=args.mock) as robot:
            RobotClient(robot, args.robot_id, args.server).run()
    except KeyboardInterrupt:
        pass
    except Exception:
        print('Robot runtime failed', file=sys.stderr, flush=True)
        raise
    finally:
        print(f'Robot runtime stopped robot={args.robot_id}', file=sys.stderr, flush=True)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
