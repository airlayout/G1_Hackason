"""SdkBridgeProcess の統合テスト。実スレッド + 実Unix domain socketで検証する。

タイミング系のテストはsleepではなくwait_until()でポーリングし、CI環境差による
flakyさを抑える。ただしスレッド+実ソケットを使うため、他のテストより本質的に
時間がかかる(数百ms程度)。
"""

import os
import tempfile
import time
import unittest

import _pathfix  # noqa: F401
from ipc_transport import connect_client
from protocol import BridgeStatus, CmdPacket, StatePacket, monotonic_ns
from sdk_process_mock import MockMoveBackend, SdkBridgeConfig, SdkBridgeProcess


def wait_until(predicate, timeout=2.0, interval=0.01, message="condition not met in time"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(interval)
    raise AssertionError(message)


class SdkBridgeTestCase(unittest.TestCase):
    """速いテスト条件(cmd_rate=50Hz)で構成する。実運用値は仕様書の20Hzを想定。"""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp(prefix="g1bridge_")
        self._cmd_path = os.path.join(self._tmpdir, "cmd.sock")
        self._state_path = os.path.join(self._tmpdir, "state.sock")
        self._backend = MockMoveBackend()
        self._cfg = SdkBridgeConfig(
            cmd_sock_path=self._cmd_path,
            state_sock_path=self._state_path,
            cmd_rate_hz=50.0,  # period=0.02s
            sdk_command_duration_s=0.04,
            cmd_timeout_s=0.08,
            max_sdk_errors=3,
        )
        self._bridge = SdkBridgeProcess(self._cfg, self._backend)
        self._bridge.start()

    def tearDown(self):
        self._bridge.stop()

    def _connect_cmd_client(self):
        return connect_client(self._cmd_path, max_payload=64, timeout=1.0)

    def _connect_state_client(self):
        return connect_client(self._state_path, max_payload=96, timeout=1.0)

    def _send_cmd(self, client, seq, vx, vy, omega):
        client.send_latest(CmdPacket(seq=seq, timestamp_ns=monotonic_ns(), vx=vx, vy=vy, omega=omega).encode())

    def test_startup_sends_zero_before_any_connection(self):
        # D-11: 起動直後、接続前でも必ずゼロ速度を1回送っている
        first = self._backend.calls[0]
        self.assertEqual(first[:3], (0.0, 0.0, 0.0))
        self.assertEqual(first[3], self._cfg.sdk_command_duration_s)

    def test_duration_argument_is_always_finite(self):
        # D-27: 呼び出しの度にdurationを明示していることを確認する(continous_move相当を使わない)
        wait_until(lambda: len(self._backend.calls) >= 3)
        for call in self._backend.calls:
            duration = call[3]
            self.assertGreater(duration, 0.0)
            self.assertLessEqual(duration, self._cfg.cmd_timeout_s)

    def test_cmd_forwarded_to_backend(self):
        client = self._connect_cmd_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)
        self._send_cmd(client, 1, 0.15, 0.0, 0.05)

        wait_until(lambda: self._backend.last_call()[:3] == (0.15, 0.0, 0.05))
        client.close()

    def test_watchdog_zeroes_after_cmd_timeout(self):
        client = self._connect_cmd_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)
        self._send_cmd(client, 1, 0.15, 0.0, 0.0)
        wait_until(lambda: self._backend.last_call()[:3] == (0.15, 0.0, 0.0))

        # それ以降は何も送らない -> cmd_timeout超過でSDK側watchdogがゼロを送るはず
        wait_until(
            lambda: self._backend.last_call()[:3] == (0.0, 0.0, 0.0),
            timeout=1.0,
            message="cmd_timeout超過後もゼロ速度が送られなかった(SDK側watchdogが機能していない)",
        )
        client.close()

    def test_peer_disconnect_sets_status_disconnected_and_zero(self):
        client = self._connect_cmd_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)
        self._send_cmd(client, 1, 0.1, 0.0, 0.0)
        wait_until(lambda: self._backend.last_call()[:3] == (0.1, 0.0, 0.0))

        client.close()
        wait_until(lambda: self._bridge.status == BridgeStatus.DISCONNECTED)
        wait_until(lambda: self._backend.last_call()[:3] == (0.0, 0.0, 0.0))

    def test_consecutive_sdk_errors_trigger_fault_and_stay_until_clear(self):
        client = self._connect_cmd_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)

        self._backend.fail_next(self._cfg.max_sdk_errors)
        self._send_cmd(client, 1, 0.1, 0.0, 0.0)

        wait_until(
            lambda: self._bridge.status == BridgeStatus.FAULT,
            message="連続SDKエラーでFAULTに遷移しなかった",
        )

        # D-13: FAULT中は新しい有効な指令が来ても自動では復帰しない
        self._send_cmd(client, 2, 0.1, 0.0, 0.0)
        time.sleep(0.1)
        self.assertEqual(self._bridge.status, BridgeStatus.FAULT)
        self.assertEqual(self._backend.last_call()[:3], (0.0, 0.0, 0.0))

        # 操作者がclear_fault()した後は復帰できる
        self.assertTrue(self._bridge.clear_fault())
        self._send_cmd(client, 3, 0.1, 0.0, 0.0)
        wait_until(lambda: self._backend.last_call()[:3] == (0.1, 0.0, 0.0))

        client.close()

    def test_successful_call_resets_consecutive_error_count(self):
        # 仕様書8章: 「連続」失敗回数。成功が挟まればカウントはリセットされ、FAULTにならない
        client = self._connect_cmd_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)

        for _ in range(self._cfg.max_sdk_errors - 1):
            self._backend.fail_next(1)
            self._send_cmd(client, 1, 0.1, 0.0, 0.0)
            wait_until(lambda: self._bridge.sdk_error_count >= 1)
            wait_until(lambda: self._bridge.sdk_error_count == 0)  # 次の成功呼び出しでリセットされる

        self.assertNotEqual(self._bridge.status, BridgeStatus.FAULT)
        client.close()

    def test_state_channel_reports_status_and_pose(self):
        cmd_client = self._connect_cmd_client()
        state_client = self._connect_state_client()
        wait_until(lambda: self._bridge.status != BridgeStatus.DISCONNECTED)

        self._send_cmd(cmd_client, 1, 0.1, 0.0, 0.0)

        received = []
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(received) < 3:
            raw = state_client.recv_latest()
            if raw is not None:
                received.append(StatePacket.decode(raw))
            time.sleep(0.005)

        self.assertGreaterEqual(len(received), 1)
        self.assertTrue(any(p.status == BridgeStatus.NAVIGATING for p in received))
        # x=0スタートで前進指令を送っているので、いずれかの時点でxが正に進んでいるはず
        self.assertTrue(any(p.x > 0.0 for p in received))

        cmd_client.close()
        state_client.close()


if __name__ == "__main__":
    unittest.main()
