from .router import DEFAULT_TIMEOUT_S, RobotServer

__all__ = ["RobotServer"]


def main():
    import argparse
    import asyncio
    from .service import ServerService

    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="tcp://*:5555")
    parser.add_argument("--api", default="tcp://127.0.0.1:5556")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="Robot response timeout in seconds (default: 20; includes arm feedback and observation)")
    args = parser.parse_args()

    async def serve():
        async with RobotServer(args.bind, args.timeout) as robots:
            await ServerService(robots, args.api).run()

    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
