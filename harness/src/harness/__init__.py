def main():
    import argparse
    from pathlib import Path
    from mcp.server.fastmcp import FastMCP
    from .client import ServerClient, SnapshotClient
    from .tools import RobotTools

    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="tcp://127.0.0.1:5556")
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    client = SnapshotClient(args.snapshot) if args.snapshot else ServerClient(args.server, args.timeout)
    tools = RobotTools(client)
    mcp = FastMCP("lekiwi")
    mcp.tool(structured_output=False)(tools.observe)
    if not args.snapshot:
        mcp.tool(structured_output=False)(tools.execute)
    mcp.run()
