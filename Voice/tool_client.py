"""G1 内の Unix socket でツールサーバーを呼ぶ。モデル・ROS の依存を持たない。"""

from __future__ import annotations

import json
import os
import socket
import time
from pathlib import Path
from typing import Any
from uuid import uuid4

DEFAULT_SOCKET = Path(f"/tmp/g1-voice-{os.getuid()}/tools.sock")
MAX_MESSAGE = 65536


def read_message(stream: Any) -> dict[str, Any]:
    line = stream.readline(MAX_MESSAGE + 1)
    if not line.endswith(b"\n") or len(line) > MAX_MESSAGE:
        raise ValueError("IPC メッセージが不完全、または大きすぎます")
    def invalid_constant(value: str) -> None:
        raise ValueError(f"JSON の非有限値は使えません: {value}")
    data = json.loads(line, parse_constant=invalid_constant)
    if not isinstance(data, dict):
        raise ValueError("IPC メッセージはオブジェクトにしてください")
    return data


class ToolClient:
    def __init__(self, path: Path = DEFAULT_SOCKET, timeout_s: float = 8.0) -> None:
        self.path, self.timeout_s = path, timeout_s

    def request(self, operation: str, **fields: Any) -> dict[str, Any]:
        request_id = uuid4().hex
        payload = json.dumps({"version": 1, "id": request_id, "op": operation, **fields}, allow_nan=False).encode() + b"\n"
        if len(payload) > MAX_MESSAGE:
            raise ValueError("IPC リクエストが大きすぎます")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(self.timeout_s)
            sock.connect(str(self.path))
            sock.sendall(payload)
            with sock.makefile("rb") as stream:
                response = read_message(stream)
        if response.get("id") != request_id:
            raise ValueError("IPC 応答の ID が一致しません")
        return response

    def call(self, name: str, arguments: dict[str, Any], *, execute: bool = False, max_age_s: float = 2.0) -> dict[str, Any]:
        return self.request("call", tool=name, arguments=arguments, execute=execute,
                            expires_at=time.monotonic() + max_age_s)

    def health(self, session: str, healthy: bool) -> dict[str, Any]:
        return self.request("health", session=session, healthy=healthy, sent_at=time.monotonic())
