"""ZmqFrameSourceのメッセージデコード処理を、実際のZMQ接続無しで確認するテスト。

lerobotのZMQCameraと同じワイヤプロトコル(JSON文字列 {"images": {name: base64 JPEG}})を
正しくデコードできることだけを確認する。sim/real実機への接続そのものはここでは
対象外(ネットワークが必要なため)。
"""
from __future__ import annotations

import base64
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.camera.zmq_camera import ZmqFrameSource  # noqa: E402


def encode_message(camera_name: str, frame: np.ndarray) -> str:
    ok, jpeg = cv2.imencode(".jpg", frame)
    assert ok
    img_b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
    return json.dumps({"images": {camera_name: img_b64}})


class ZmqFrameSourceDecodeTest(unittest.TestCase):
    def test_read_decodes_matching_camera_name(self) -> None:
        source = ZmqFrameSource(server_address="localhost", camera_name="head_camera")
        original = np.full((48, 64, 3), 127, dtype=np.uint8)
        source._socket = MagicMock()
        source._socket.recv_string.return_value = encode_message("head_camera", original)

        frame = source.read()

        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape[:2], (48, 64))

    def test_read_converts_rgb_wire_format_to_bgr(self) -> None:
        # 配信側はRGB順でJPEGエンコードする(Perception/sim/README.mdの実測値通り)。
        # cv2.imencode()は渡された配列をBGRとみなしてエンコードするため、ここではその
        # 送信側の挙動そのままに、チャンネルごとに値が異なる配列を"生の配列"として渡す。
        raw_wire_array = np.zeros((16, 16, 3), dtype=np.uint8)
        raw_wire_array[..., 0] = 200  # 配信側にとっての "R"
        raw_wire_array[..., 1] = 50
        raw_wire_array[..., 2] = 10  # 配信側にとっての "B"

        source = ZmqFrameSource(server_address="localhost", camera_name="head_camera")
        source._socket = MagicMock()
        source._socket.recv_string.return_value = encode_message("head_camera", raw_wire_array)

        frame = source.read()

        self.assertIsNotNone(frame)
        # FrameSourceの契約(BGRを返す)に合わせてRGB2BGR変換されているはず:
        # 配信側の"R"(200)がBGR配列のB(index 2)に、"B"(10)がBGR配列のB(index 0)に来る。
        # JPEG圧縮による誤差を許容するため、許容誤差付きで比較する。
        mean_pixel = frame.reshape(-1, 3).mean(axis=0)
        self.assertAlmostEqual(mean_pixel[0], 10, delta=10)
        self.assertAlmostEqual(mean_pixel[1], 50, delta=10)
        self.assertAlmostEqual(mean_pixel[2], 200, delta=10)

    def test_read_returns_none_when_camera_name_unknown_and_no_images(self) -> None:
        source = ZmqFrameSource(server_address="localhost", camera_name="head_camera")
        source._socket = MagicMock()
        source._socket.recv_string.return_value = json.dumps({"images": {}})

        frame = source.read()

        self.assertIsNone(frame)


if __name__ == "__main__":
    unittest.main()
