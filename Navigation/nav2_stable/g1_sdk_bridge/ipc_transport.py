"""Unix domain socket (SOCK_SEQPACKET) による cmd/state 伝送。

Planning.md D-05 の決定:
- SOCK_SEQPACKET を使う(メッセージ境界が保たれ、相手の接続断を検知できる)
- 送信側は非ブロッキング。バッファ満杯時は古いものを捨て、最新値を優先する
- cmdとstateは別ソケット(stateの高頻度配信がcmdのレイテンシに影響しないように)

役割: SDK側プロセスが両ソケットのサーバ(bind + listen)になる。
systemdで常駐管理される側が固定のアドレスを持つ方が構成が単純なため。
ROS側プロセス(safety manager)がクライアントとして接続する。
"""

from __future__ import annotations

import errno
import os
import socket
from typing import Optional


class PeerClosed(Exception):
    """相手がソケットを閉じた(プロセス終了・クラッシュ)ことを示す。"""


class SeqPacketEndpoint:
    """接続済み SOCK_SEQPACKET ソケットの薄いラッパ。"""

    def __init__(self, sock: socket.socket, max_payload: int):
        self._sock = sock
        self._sock.setblocking(False)
        self._max_payload = max_payload

    def send_latest(self, payload: bytes) -> bool:
        """非ブロッキング送信。バッファ満杯(EAGAIN)なら黙って破棄しFalseを返す。

        最新値優先の設計: 送れなかった1件は次にもっと新しい値が来て上書きされるだけなので、
        古い値を無理に送るより破棄する方が正しい。
        """
        try:
            self._sock.send(payload)
            return True
        except BlockingIOError:
            return False
        except OSError as exc:
            if exc.errno in (errno.EPIPE, errno.ECONNRESET, errno.ENOTCONN):
                raise PeerClosed(str(exc)) from exc
            raise

    def recv_latest(self) -> Optional[bytes]:
        """受信バッファに溜まっている分をすべて読み切り、最後の1件だけを返す。

        古い未処理メッセージが残っていても、常に「今の値」を使うことで
        処理遅延によるコマンド滞留を防ぐ(latest-value semantics)。
        """
        latest: Optional[bytes] = None
        while True:
            try:
                data = self._sock.recv(self._max_payload)
            except BlockingIOError:
                return latest
            except OSError as exc:
                if exc.errno in (errno.ECONNRESET, errno.ENOTCONN):
                    raise PeerClosed(str(exc)) from exc
                raise
            if data == b"":
                # 空bytesはピアが正常にshutdown/closeしたことを示す(SOCK_SEQPACKETの規約)
                raise PeerClosed("peer closed the connection")
            latest = data

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def fileno(self) -> int:
        return self._sock.fileno()


class SeqPacketServer:
    """bind + listen する側。SDK側プロセスが cmd/state それぞれで1つずつ持つ。"""

    def __init__(self, path: str, max_payload: int):
        self._path = path
        self._max_payload = max_payload
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        self._listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._listener.bind(path)
        self._listener.listen(1)
        self._listener.setblocking(False)

    def accept_if_pending(self) -> Optional[SeqPacketEndpoint]:
        """非ブロッキングでaccept。接続要求が無ければNone。"""
        try:
            conn, _ = self._listener.accept()
        except BlockingIOError:
            return None
        return SeqPacketEndpoint(conn, self._max_payload)

    def accept_blocking(self, timeout: Optional[float] = None) -> SeqPacketEndpoint:
        self._listener.settimeout(timeout)
        try:
            conn, _ = self._listener.accept()
        finally:
            self._listener.setblocking(False)
        return SeqPacketEndpoint(conn, self._max_payload)

    def close(self) -> None:
        try:
            self._listener.close()
        finally:
            try:
                os.unlink(self._path)
            except FileNotFoundError:
                pass


def connect_client(path: str, max_payload: int, timeout: float = 5.0) -> SeqPacketEndpoint:
    """ROS側プロセスがSDK側プロセスへ接続する側。"""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.settimeout(timeout)
    sock.connect(path)
    return SeqPacketEndpoint(sock, max_payload)
