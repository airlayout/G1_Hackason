"""SDKを使わず、プロセス境界の指令検証とウォッチドッグを検証する。"""

import socket
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "real"))
from arm_bridge import ArmProxy, serve_connection


class FakeArm:
    releases = 0
    writes = 0

    def current_configuration(self):
        return ((0.0,)*29, (0, 0, 0), time.monotonic(), 0, 0)

    def current_arm(self):
        return (0.0,)*14

    def set_hold(self, current):
        pass

    def publish(self, joints, weight):
        self.writes += 1

    def release(self):
        self.releases += 1


def connection():
    client_socket, server_socket = socket.socketpair()
    proxy = ArmProxy.__new__(ArmProxy)
    proxy.socket = client_socket
    proxy.socket.settimeout(1)
    proxy.buffer, proxy.lock, proxy.weight = b"", threading.Lock(), 0.0
    arm, errors = FakeArm(), []

    def worker():
        with server_socket:
            try:
                serve_connection(server_socket, arm, timeout_s=0.05)
            except RuntimeError as exc:
                errors.append(str(exc))
    thread = threading.Thread(target=worker)
    thread.start()
    return proxy, arm, errors, thread


def test_bridge_releases_when_command_process_stalls():
    proxy, arm, errors, thread = connection()
    try:
        assert len(proxy.current_configuration()[0]) == 29
        proxy.set_hold(proxy.current_arm())
        proxy.publish((0,)*7, 0.01)
        thread.join(timeout=0.5)
        assert not thread.is_alive()
        assert arm.releases >= 1
        assert "途絶" in errors[0]
    finally:
        proxy.socket.close()
        thread.join(timeout=1)


def test_invalid_or_fast_commands_are_rejected_before_publish():
    proxy, arm, errors, thread = connection()
    try:
        proxy.set_hold(proxy.current_arm())
        proxy.publish((0,)*7, 0.01)
        with pytest.raises(RuntimeError, match="変化速度"):
            proxy.publish((0.3,)*7, 0.01)
        assert arm.writes == 1
    finally:
        proxy.socket.close()
        thread.join(timeout=1)
