import base64
import json

import cv2
import numpy as np

from .base import FrameSource

# NOTE(プロトコルについて): ここで実装しているワイヤプロトコルは、lerobot本体の
# `src/lerobot/cameras/zmq/camera_zmq.py`(ZMQCamera)が使っているものと同一である
# ことを確認した上で、pyzmq+cv2+numpyだけで薄く再実装したもの。lerobotパッケージ
# そのもの(cyclonedds/unitree_sdk2py/pinocchio込みの重い依存)には依存しない。
#
#   - サーバ(run_g1_server.py --camera。MuJoCoシムも同じ仕組み)はZMQ PUBソケットで
#     JSON文字列 {"images": {"<camera_name>": "<base64エンコードされたJPEG>"}} を
#     配信し続ける
#   - クライアントはSUBソケットで接続し、SUBSCRIBE ""・CONFLATE=True(キューを持たず
#     常に最新フレームのみ保持)・RCVTIMEOでタイムアウトを設定して受信する
#
# 将来lerobot側でこのプロトコル(JSONのキー名やbase64+JPEGという形式)が変わった場合は、
# このファイルも追従が必要。変わっていないか確認したい場合は上記ファイルを参照。
#
# NOTE(チャンネル順について、2026-08-30 sim実測で確認・Perception/sim/README.md参照):
# 配信側はRGB順で画像をエンコードしているため、cv2.imdecode()の戻り値をそのまま使うと
# 赤と青が入れ替わって見える。FrameSourceインターフェースの契約(BGRを返す。
# webcam.py/video_file.pyと揃える)に合わせるため、ここでRGB2BGR変換を行う。


class ZmqFrameSource(FrameSource):
    """ZMQストリーム(lerobotのZMQCameraと同じプロトコル)から画像を取得する。

    sim(MuJoCoのhead_camera)・real(実機G1、run_g1_server.py --camera)の
    どちらも同じZMQストリームの仕組みを使うため、このクラス1つで共用する。
    """

    def __init__(
        self,
        server_address: str,
        port: int = 5555,
        camera_name: str = "head_camera",
        timeout_ms: int = 5000,
    ) -> None:
        self._server_address = server_address
        self._port = port
        self._camera_name = camera_name
        self._timeout_ms = timeout_ms
        self._context = None
        self._socket = None

    def open(self) -> None:
        try:
            import zmq
        except ImportError as e:
            raise RuntimeError(
                "pyzmq がインストールされていません。"
                "README.md のセットアップ手順に従って `pip install pyzmq` してください。"
            ) from e

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._socket.setsockopt(zmq.CONFLATE, True)
        self._socket.connect(f"tcp://{self._server_address}:{self._port}")

    def read(self) -> np.ndarray | None:
        if self._socket is None:
            raise RuntimeError("open() を呼び出す前に read() が呼ばれました")

        import zmq

        try:
            message = self._socket.recv_string()
        except zmq.Again:
            # タイムアウト内にフレームが届かなかった(接続断・配信停止など)
            return None

        data = json.loads(message)

        images = data.get("images", {})
        if self._camera_name in images:
            img_b64 = images[self._camera_name]
        elif images:
            img_b64 = next(iter(images.values()))
        else:
            return None

        img_bytes = base64.b64decode(img_b64)
        frame = cv2.imdecode(np.frombuffer(img_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._context is not None:
            self._context.term()
            self._context = None
