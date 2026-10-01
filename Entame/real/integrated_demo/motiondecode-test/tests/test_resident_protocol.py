from __future__ import annotations

import socket
import threading
import time

from resident_protocol import DryRunBackend, JsonUnixServer, ResidentController, unix_request


def test_state_machine_unknown_busy_and_reconnect(tmp_path):
    path = tmp_path / "worker.sock"
    backend = DryRunBackend(.08)
    controller = ResidentController(backend, {"surprise"})
    controller.start()
    server = JsonUnixServer(path, controller)
    thread = threading.Thread(target=server.serve, daemon=True); thread.start()
    deadline = time.monotonic() + 1
    while not path.exists() and time.monotonic() < deadline: time.sleep(.005)

    assert unix_request(path, {"operation": "status"})["state"] == "READY"
    assert unix_request(path, {"reaction": "unknown"})["accepted"] is False
    first = {}
    running = threading.Thread(target=lambda: first.update(
        unix_request(path, {"reaction": "surprise"})))
    running.start(); time.sleep(.02)
    busy = unix_request(path, {"reaction": "surprise"})
    assert busy == {"accepted": False, "reason": "worker busy", "state": "EXECUTING"}
    running.join(1)
    assert first["status"] == "pass"
    # A new connection works after the first client disconnects.
    assert unix_request(path, {"operation": "status"})["state"] == "READY"
    unix_request(path, {"operation": "stop"}); thread.join(1)


class FaultBackend(DryRunBackend):
    def execute(self, reaction, request, received):
        raise RuntimeError("backend failed")


def test_fault_is_latched():
    controller = ResidentController(FaultBackend(), {"surprise"}); controller.start()
    response = controller.handle({"reaction": "surprise"})
    assert response["state"] == "FAULT"
    assert controller.handle({"reaction": "surprise"})["accepted"] is False


def test_recoverable_abort_result_returns_worker_to_ready():
    class RecoverableBackend(DryRunBackend):
        def execute(self, reaction, request, received):
            return {
                "reaction": reaction, "status": "recoverable_abort",
                "classification": "RECOVERABLE_ABORT", "released": True,
                "weight_zero": True,
            }

    controller = ResidentController(RecoverableBackend(), {"surprise"})
    controller.start()
    response = controller.handle({"reaction": "surprise"})
    assert response["classification"] == "RECOVERABLE_ABORT"
    assert response["state"] == "READY"
    assert controller.state == "READY"


def test_preflight_is_read_only_and_failure_does_not_fault_worker():
    class PreflightBackend(DryRunBackend):
        def __init__(self):
            super().__init__(); self.safe = True

        def preflight(self):
            if not self.safe:
                raise RuntimeError("robot not stable")
            return {"passed": True}

    backend = PreflightBackend()
    controller = ResidentController(backend, {"surprise"}); controller.start()
    assert controller.handle({"operation": "preflight"}) == {
        "accepted": True, "state": "READY", "passed": True,
    }
    assert backend.executions == 0
    backend.safe = False
    response = controller.handle({"operation": "preflight"})
    assert response["passed"] is False
    assert response["reason"] == "robot not stable"
    assert controller.state == "READY"
    assert backend.executions == 0
