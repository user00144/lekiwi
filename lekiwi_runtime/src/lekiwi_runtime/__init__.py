from .client import RobotClient
from .robot import Robot

__all__ = ["Robot", "RobotClient"]


def main():
    import argparse
    from pathlib import Path
    from .orin import OrinRobot
    from .config import load_config

    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True, help="tcp://SERVER_IP:5555")
    parser.add_argument("--robot-id", default="kiwi-01")
    parser.add_argument("--config", type=Path, help="Per-robot JSON configuration overrides")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    try:
        with OrinRobot(load_config(args.config), mock=args.mock) as robot:
            RobotClient(robot, args.robot_id, args.server).run()
    except KeyboardInterrupt:
        pass
