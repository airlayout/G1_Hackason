"""Small newline-delimited JSON protocol for the resident reaction worker."""
from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path
from typing import Any


STATES = frozenset({"STARTING", "READY", "EXECUTING", "FAULT", "STOPPING"})


class ResidentController:
    """Serialize reactions and expose explicit worker states."""

    def __init__(self, backend: Any, reactions: set[str]) -> None:
        self.backend = backend
        self.reactions = frozenset(reactions)
        self.state = "STARTING"
        self.fault: str | None = None
        self._lock = threading.Lock()
        self.started_at = time.monotonic()

    def start(self) -> None:
        try:
            self.backend.start()
            self.state = "READY"
        except BaseException as exc:
            self.fault = str(exc)
            self.state = "FAULT"
            raise

    def status(self) -> dict[str, Any]:
        detail = self.backend.status() if hasattr(self.backend, "status") else {}
        return {"state": self.state, "fault": self.fault,
                "reactions": sorted(self.reactions), "worker_started_at": self.started_at,
                **detail}

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        operation = request.get("operation", "execute")
        if operation == "status":
            return {"accepted": True, **self.status()}
        if operation == "preflight":
            if self.state != "READY":
                return {"accepted": False, "state": self.state,
                        "passed": False, "reason": "worker not ready"}
            if not self._lock.acquire(blocking=False):
                return {"accepted": False, "state": "EXECUTING",
                        "passed": False, "reason": "worker busy"}
            try:
                result = self.backend.preflight()
                return {"accepted": True, "state": self.state, **result}
            except BaseException as exc:
                return {"accepted": True, "state": self.state,
                        "passed": False, "reason": str(exc)}
            finally:
                self._lock.release()
        if operation == "stop":
            self.state = "STOPPING"
            self.backend.stop()
            return {"accepted": True, "state": self.state}
        if operation != "execute":
            return {"accepted": False, "state": self.state, "reason": "unknown operation"}
        reaction = request.get("reaction")
        if reaction not in self.reactions:
            return {"accepted": False, "state": self.state, "reason": "unknown reaction"}
        if self.state == "FAULT":
            return {"accepted": False, "state": self.state, "reason": self.fault}
        if not self._lock.acquire(blocking=False):
            return {"accepted": False, "state": "EXECUTING", "reason": "worker busy"}
        try:
            if self.state != "READY":
                return {"accepted": False, "state": self.state, "reason": "worker not ready"}
            received = time.monotonic()
            self.state = "EXECUTING"
            result = self.backend.execute(reaction, request, received)
            if not isinstance(result, dict):
                raise RuntimeError("backend result must be an object")
            self.state = "READY"
            return {"accepted": True, "state": self.state,
                    "worker_receive_monotonic_s": received, **result}
        except BaseException as exc:
            self.fault = str(exc)
            self.state = "FAULT"
            return {"accepted": True, "state": self.state, "status": "fail",
                    "reason": str(exc)}
        finally:
            self._lock.release()


class JsonUnixServer:
    def __init__(self, path: Path, controller: ResidentController) -> None:
        self.path = Path(path)
        self.controller = controller
        self.socket: socket.socket | None = None
        self.stopping = threading.Event()

    def serve(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self.path)); server.listen(8); server.settimeout(.2)
        self.socket = server
        try:
            while not self.stopping.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                threading.Thread(target=self._connection, args=(connection,), daemon=True).start()
        finally:
            server.close(); self.path.unlink(missing_ok=True)

    def _connection(self, connection: socket.socket) -> None:
        with connection:
            stream = connection.makefile("rwb")
            for raw in stream:
                request = {}
                try:
                    request = json.loads(raw)
                    if not isinstance(request, dict):
                        raise ValueError("request must be an object")
                    response = self.controller.handle(request)
                except BaseException as exc:
                    response = {"accepted": False, "state": self.controller.state,
                                "reason": str(exc)}
                stream.write((json.dumps(response, sort_keys=True) + "\n").encode())
                stream.flush()
                if request.get("operation") == "stop" and response.get("accepted"):
                    self.stopping.set(); break


def unix_request(path: Path, request: dict[str, Any], timeout: float = 30.) -> dict[str, Any]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout); client.connect(str(path))
        stream = client.makefile("rwb")
        stream.write((json.dumps(request, sort_keys=True) + "\n").encode()); stream.flush()
        raw = stream.readline()
    if not raw:
        raise RuntimeError("resident worker closed without a response")
    response = json.loads(raw)
    if not isinstance(response, dict):
        raise RuntimeError("resident worker response must be an object")
    return response


class DryRunBackend:
    def __init__(self, execution_seconds: float = .01) -> None:
        self.execution_seconds = execution_seconds
        self.executions = 0
        self.started = False

    def start(self) -> None:
        self.started = True

    def status(self) -> dict[str, Any]:
        return {"mode": "dry-run", "executions": self.executions}

    def preflight(self) -> dict[str, Any]:
        return {"passed": True, "mode": "dry-run"}

    def execute(self, reaction: str, request: dict[str, Any], received: float) -> dict[str, Any]:
        del request
        acquired = time.monotonic(); time.sleep(self.execution_seconds)
        clip = time.monotonic(); self.executions += 1
        return {"reaction": reaction, "status": "pass", "executed": False,
                "released": True, "returned_to_q0": True,
                "acquire_start_monotonic_s": acquired,
                "clip_start_monotonic_s": clip,
                "worker_to_acquire_s": acquired - received}

    def stop(self) -> None:
        self.started = False
