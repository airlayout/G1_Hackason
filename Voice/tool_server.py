#!/usr/bin/env python3
"""G1 上で動かす、登録済みツールのローカル実行サーバー。"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import socketserver
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Voice.supervision import HealthLease, HeartbeatWire, ONBOARD_HEARTBEAT_PORT
from Voice.tool_client import DEFAULT_SOCKET, read_message
from Voice.tool_runtime import ToolResult, ToolRuntime, default_backends, load_tools


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        self.connection.settimeout(2.0)
        request_id = None
        try:
            request = read_message(self.rfile)
            request_id = request.get("id")
            result = self.server.dispatch(request)
        except Exception as exc:
            result = {"success": False, "status": "invalid_request", "message": str(exc)}
        payload = json.dumps({"id": request_id, **result}, ensure_ascii=False, allow_nan=False).encode() + b"\n"
        try:
            self.wfile.write(payload)
        except OSError:
            pass


class ToolServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path: Path, runtime: ToolRuntime, *, execute: bool = False,
                 onboard_heartbeat: bool = False, heartbeat_port: int = ONBOARD_HEARTBEAT_PORT) -> None:
        if not 1 <= heartbeat_port <= 65535 or heartbeat_port == 47311:
            raise ValueError("オンボード監視には外部操作PC用 47311 と異なるポートを指定してください")
        self.runtime, self.execute_enabled = runtime, execute
        self.onboard_heartbeat = onboard_heartbeat
        self.heartbeat_port = heartbeat_port
        self.lease = HealthLease()
        self.stop_event = threading.Event()
        self.monitor: threading.Thread | None = None
        self.socket_identity: tuple[int, int] | None = None
        self.path = path.expanduser().absolute()
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        directory = self.path.parent.stat()
        if directory.st_uid != os.getuid() or directory.st_mode & 0o077:
            raise ValueError("ツールソケットは自分が所有する mode 700 のディレクトリに置いてください")
        if self.path.exists():
            raise ValueError(f"ソケットが既に存在します。既存サーバーか残骸を確認してください: {self.path}")
        super().__init__(str(self.path), RequestHandler)
        identity = self.path.stat()
        self.socket_identity = identity.st_dev, identity.st_ino
        self.path.chmod(0o600)
        self.monitor = threading.Thread(target=self._monitor, name="voice-health", daemon=True)
        self.monitor.start()

    def _monitor(self) -> None:
        wire = HeartbeatWire()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            while not self.stop_event.wait(0.1):
                if self.onboard_heartbeat and self.execute_enabled and self.lease.alive():
                    sock.sendto(wire.next(), ("127.0.0.1", self.heartbeat_port))

    def dispatch(self, request: dict[str, Any]) -> dict[str, Any]:
        if type(request.get("version")) is not int or request["version"] != 1 or not isinstance(request.get("id"), str):
            raise ValueError("IPC の version / id が不正です")
        operation = request.get("op")
        if operation == "list":
            return {"success": True, "tools": self.runtime.describe(), "execute_enabled": self.execute_enabled}
        if operation == "health":
            ok = self.lease.update(request.get("session"), request.get("healthy"), request.get("sent_at"))
            return {"success": ok, "status": "healthy" if self.lease.alive() else "unhealthy"}
        if operation != "call":
            raise ValueError("未知の IPC 操作です")
        deadline = request.get("expires_at")
        if not isinstance(deadline, (int, float)) or not math.isfinite(deadline) or not time.monotonic() <= deadline <= time.monotonic() + 30:
            return asdict(ToolResult(str(request.get("tool", "")), False, "expired", "実行期限を過ぎた指示です"))
        execute = request.get("execute", False)
        if type(execute) is not bool or not isinstance(request.get("tool"), str):
            raise ValueError("tool / execute が不正です")
        if execute and not self.execute_enabled:
            return asdict(ToolResult(request["tool"], False, "execution_disabled", "サーバーが dry-run モードです"))
        if execute and self.onboard_heartbeat and not self.lease.alive():
            return asdict(ToolResult(request["tool"], False, "unhealthy", "音声処理の生存報告がありません"))
        result = self.runtime.call(request["tool"], request.get("arguments", {}), execute=execute)
        print("[tools] " + json.dumps(asdict(result), ensure_ascii=False), flush=True)
        return asdict(result)

    def server_close(self) -> None:
        self.stop_event.set()
        if self.monitor is not None:
            self.monitor.join(timeout=1)
        super().server_close()
        try:
            identity = self.path.stat()
            if self.socket_identity == (identity.st_dev, identity.st_ino):
                self.path.unlink()
        except FileNotFoundError:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "configs/tools.json")
    parser.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--onboard-heartbeat", action="store_true", help="期限付き音声処理の生存報告を loopback の Nav2 へ渡す")
    args = parser.parse_args(argv)
    backends = default_backends()
    runtime = ToolRuntime(load_tools(args.config), backends)
    try:
        if args.execute and any(spec.backend == "ros_service" for spec in runtime.specs.values()):
            backends["ros_service"].prepare()
        with ToolServer(args.socket, runtime, execute=args.execute, onboard_heartbeat=args.onboard_heartbeat) as server:
            print(f"[tools] 待受: {args.socket} execute={args.execute} onboard_heartbeat={args.onboard_heartbeat}", flush=True)
            notify_socket = os.environ.get("NOTIFY_SOCKET")
            if notify_socket:
                address = "\0" + notify_socket[1:] if notify_socket.startswith("@") else notify_socket
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as notifier:
                    notifier.sendto(b"READY=1", address)
            server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        return 130
    finally:
        for backend in backends.values():
            if hasattr(backend, "close"):
                backend.close()
    return 0


if __name__ == "__main__":
    def stop_on_signal(signum: int, frame: Any) -> None:
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_on_signal)
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print(f"[tools] エラー: {error}", file=sys.stderr)
        raise SystemExit(2)
