"""深度付きストリーム（RGBD）の受信側。タスク3。

Perception の FrameSource を継承するので、read() は既存の約束どおり BGR の画像だけを返す
（既存のパイプラインにそのまま差し込める）。深度と内部パラメータが要るときは read_rgbd() を使う。
"""

from __future__ import annotations

from .perception_bridge import perception
from .rgbd_protocol import RgbdFrame, decode_rgbd

FrameSource = perception("camera").FrameSource


class RgbdZmqSource(FrameSource):
    def __init__(self, server_address: str, port: int = 5556, timeout_ms: int = 5000) -> None:
        self._address = server_address
        self._port = port
        self._timeout_ms = timeout_ms
        self._context = None
        self._socket = None
        self.last: RgbdFrame | None = None

    def open(self) -> None:
        import zmq

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        # 常に最新の 1 フレームだけを保持する（1 フレーム = 1 パートなので使える）
        self._socket.setsockopt(zmq.CONFLATE, True)
        self._socket.connect(f"tcp://{self._address}:{self._port}")

    def read_rgbd(self) -> RgbdFrame | None:
        """最新の RGBD フレーム。タイムアウトまでに届かなければ None。"""
        if self._socket is None:
            raise RuntimeError("open() を呼び出す前に read_rgbd() が呼ばれた")
        import zmq

        try:
            msg = self._socket.recv()
        except zmq.Again:
            return None
        self.last = decode_rgbd(msg)
        return self.last

    def read(self):  # type: ignore[no-untyped-def]
        """BGR の画像だけを返す（FrameSource の約束）。"""
        f = self.read_rgbd()
        return None if f is None else f.color_bgr

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._context is not None:
            self._context.term()
            self._context = None
