import sys

import asyncio
from contextlib import suppress
import json
from uuid import uuid4

import zmq
import zmq.asyncio


class RobotServer:
    """One event loop owns the ROUTER; replies are matched by robot and request ID."""

    def __init__(self, endpoint: str = "tcp://*:5555", timeout: float = 10):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.endpoint = endpoint
        self.timeout = timeout
        self._pending = {}
        self._locks = {}

    async def __aenter__(self):
        self._context = zmq.asyncio.Context()
        self._socket = self._context.socket(zmq.ROUTER)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
        try:
            self._socket.bind(self.endpoint)
        except Exception:
            self._socket.close()
            self._context.term()
            raise
        print(f'Robot router listening endpoint={self.endpoint}', file=sys.stderr, flush=True)
        self._reader = asyncio.create_task(self._receive())
        return self

    async def __aexit__(self, *_):
        self._reader.cancel()
        with suppress(asyncio.CancelledError):
            await self._reader
        for future in self._pending.values():
            if not future.done():
                future.set_exception(ConnectionError("Server closed"))
        self._socket.close()
        self._context.term()
        print('Robot router stopped', file=sys.stderr, flush=True)

    async def observe(self, robot_id: str) -> dict:
        return await self._request(robot_id, {"op": "observe"})

    async def execute(self, robot_id: str, command: dict) -> dict:
        if {"op", "request_id"} & command.keys():
            raise ValueError("command cannot override op or request_id")
        return await self._request(robot_id, {**command, "op": "execute"})

    async def _request(self, robot_id: str, payload: dict) -> dict:
        lock = self._locks.setdefault(robot_id, asyncio.Lock())
        if lock.locked():
            raise RuntimeError(f"Robot busy: {robot_id}")
        async with lock:
            request_id = uuid4().hex
            key = (robot_id.encode(), request_id)
            future = asyncio.get_running_loop().create_future()
            self._pending[key] = future
            try:
                body = json.dumps({**payload, "request_id": request_id}).encode()
                async with asyncio.timeout(self.timeout):
                    await self._socket.send_multipart([key[0], body], flags=zmq.DONTWAIT)
                    response = await future
                    if payload["op"] == "execute":
                        print(f"Execution reply robot={robot_id} request={request_id} status={response.get('status')}", file=sys.stderr, flush=True)
                    return response
            except TimeoutError:
                print(f"Robot request timed out robot={robot_id} request={request_id} op={payload['op']}; outcome unknown", file=sys.stderr, flush=True)
                raise
            finally:
                self._pending.pop(key, None)
                future.cancel()

    async def _receive(self):
        while True:
            frames = await self._socket.recv_multipart()
            if len(frames) != 2:
                continue
            try:
                reply = json.loads(frames[1])
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(reply, dict) and reply.get("type") == "hello":
                print(f"Robot registered robot={frames[0].decode(errors='replace')}", file=sys.stderr, flush=True)
                continue
            if not isinstance(reply, dict) or not isinstance(reply.get("request_id"), str):
                continue
            future = self._pending.get((frames[0], reply["request_id"]))
            if future is not None and not future.done():
                future.set_result(reply)
