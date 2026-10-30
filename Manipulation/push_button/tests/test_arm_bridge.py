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

    def connection_info(self):
        return {"mode_machine": 5, "motor_count": 35, "fsm_id": 501}

    def set_hold(self, current):
        pass

    def publish(self, joints, weight, *, right_tau=None):
        self.writes += 1
        self.last_tau = right_tau

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


def test_connection_info_is_passive_and_available_over_bridge():
    proxy, arm, errors, thread = connection()
    try:
        assert proxy.connection_info() == {"mode_machine": 5, "motor_count": 35, "fsm_id": 501}
        assert arm.writes == 0
    finally:
        proxy.close()
        thread.join(timeout=1)
    assert not errors


def test_torque_round_trip_and_invalid_packet_releases_before_publish():
    proxy, arm, errors, thread = connection()
    try:
        proxy.set_hold(proxy.current_arm())
        tau = (1., -1., .5, -.5, 0., .2, -.2)
        proxy.publish((0.,)*7, .1, right_tau=tau)
        assert arm.last_tau == tau and arm.writes == 1
        with pytest.raises(RuntimeError, match="トルクが上限"):
            proxy._rpc('publish', joints=[0.]*7, weight=.1, right_tau=[0.]*6+[6.])
        assert arm.writes == 1 and arm.releases >= 1
    finally:
        proxy.socket.close()
        thread.join(timeout=1)


def test_old_bridge_without_torque_ack_is_released(monkeypatch):
    proxy = ArmProxy.__new__(ArmProxy)
    proxy.weight = 0.
    calls = []
    monkeypatch.setattr(proxy, '_rpc', lambda op, **kwargs: calls.append(op))
    with pytest.raises(RuntimeError, match="トルク送信に未対応"):
        proxy.publish((0.,)*7, .5, right_tau=(0.,)*7)
    assert calls == ['publish', 'release']
    assert proxy.weight == 0.
