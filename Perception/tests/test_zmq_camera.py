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

    def test_read_returns_none_when_camera_name_unknown_and_no_images(self) -> None:
        source = ZmqFrameSource(server_address="localhost", camera_name="head_camera")
        source._socket = MagicMock()
        source._socket.recv_string.return_value = json.dumps({"images": {}})

        frame = source.read()

        self.assertIsNone(frame)


if __name__ == "__main__":
    unittest.main()
