import os
import socket
import tempfile
import time
import unittest

import _pathfix  # noqa: F401
from ipc_transport import PeerClosed, SeqPacketServer, connect_client


def _sock_path(tmpdir: str, name: str) -> str:
    # AF_UNIXのパス長制限(108バイト前後)に収まるよう短い名前にする
    return os.path.join(tmpdir, name)


class TestSeqPacketRoundtrip(unittest.TestCase):
    def setUp(self):
        self._tmpdir = tempfile.mkdtemp(prefix="g1ipc_")
        self._path = _sock_path(self._tmpdir, "cmd.sock")
        self._server = SeqPacketServer(self._path, max_payload=64)

    def tearDown(self):
        self._server.close()

    def test_client_to_server_roundtrip(self):
        client = connect_client(self._path, max_payload=64, timeout=1.0)
        server_ep = self._server.accept_blocking(timeout=1.0)

        client.send_latest(b"hello")
        # ノンブロッキング受信なので、届くまで少し待つ
        deadline = time.monotonic() + 1.0
        received = None
        while time.monotonic() < deadline:
            received = server_ep.recv_latest()
            if received is not None:
                break
            time.sleep(0.005)
        self.assertEqual(received, b"hello")

        client.close()
        server_ep.close()

    def test_recv_latest_drains_and_returns_only_newest(self):
        client = connect_client(self._path, max_payload=64, timeout=1.0)
        server_ep = self._server.accept_blocking(timeout=1.0)

        for i in range(5):
            ok = client.send_latest(f"msg{i}".encode())
            self.assertTrue(ok)
        time.sleep(0.05)  # 全メッセージがカーネルバッファに届くのを待つ

        # 5件送っても、recv_latestは「最新の1件」だけを返す(latest-value semantics)
        received = server_ep.recv_latest()
        self.assertEqual(received, b"msg4")

        client.close()
        server_ep.close()

    def test_peer_close_detected_on_recv(self):
        client = connect_client(self._path, max_payload=64, timeout=1.0)
        server_ep = self._server.accept_blocking(timeout=1.0)

        client.close()
        time.sleep(0.05)
        with self.assertRaises(PeerClosed):
            # recv_latestは即座に例外を出さないことがあるためリトライする
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                server_ep.recv_latest()
                time.sleep(0.01)

        server_ep.close()

    def test_peer_close_detected_on_send(self):
        client = connect_client(self._path, max_payload=64, timeout=1.0)
        server_ep = self._server.accept_blocking(timeout=1.0)

        server_ep.close()
        time.sleep(0.05)
        with self.assertRaises(PeerClosed):
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                client.send_latest(b"ping")
                time.sleep(0.01)

        client.close()

    def test_send_drops_silently_when_buffer_full(self):
        client = connect_client(self._path, max_payload=64, timeout=1.0)
        server_ep = self._server.accept_blocking(timeout=1.0)
        # 送信バッファを極端に小さくして、受信側が読まない状態ですぐ満杯にする
        client._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1)

        results = [client.send_latest(f"m{i}".encode()) for i in range(50)]
        # 全部は入りきらず、どこかでFalse(破棄)が発生するはず
        self.assertIn(False, results, "送信バッファ満杯時にFalseで破棄される設計だが、一度もFalseにならなかった")

        client.close()
        server_ep.close()


if __name__ == "__main__":
    unittest.main()
