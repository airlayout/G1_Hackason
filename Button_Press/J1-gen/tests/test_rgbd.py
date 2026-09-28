"""深度付きストリーム（タスク3）。実機も RealSense も使わず、ダミーデータで送受信を確かめる。"""

from __future__ import annotations

import socket
import sys
import threading
import time
import types
import unittest
from unittest.mock import MagicMock

import numpy as np

from common.camera_rgbd import RgbdZmqSource
from common.config import FEATURE_DIR, load_config
from common.perception_bridge import perception
from common.rgbd_protocol import Intrinsics, RgbdFrame, decode_rgbd, encode_legacy_rgb, encode_rgbd

sys.path.insert(0, str(FEATURE_DIR / "real" / "depth_server"))
import rgbd_server  # noqa: E402

RED_BGR = np.array([0, 0, 255])


def sample_frame() -> RgbdFrame:
    color = np.full((48, 64, 3), 128, np.uint8)
    color[10:30, 20:40] = RED_BGR
    depth = (np.arange(48 * 64).reshape(48, 64) % 5000).astype(np.uint16)
    depth[0, 0] = 0
    depth[1, 1] = 65535
    intr = Intrinsics(64, 48, 61.5, 61.6, 31.5, 23.5, "brown_conrady", [0.1, 0.0, 0.0, 0.0, 0.0])
    return RgbdFrame(color, depth, 0.001, intr, timestamp=1234.5, frame_id=7)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class TestProtocol(unittest.TestCase):
    def test_roundtrip_keeps_depth_exactly(self) -> None:
        f = sample_frame()
        for comp in ("none", "zlib"):
            g = decode_rgbd(encode_rgbd(f, depth_compression=comp))
            np.testing.assert_array_equal(g.depth, f.depth)
            self.assertEqual(g.depth.dtype, np.uint16)
            self.assertEqual(g.intrinsics, f.intrinsics)
            self.assertEqual((g.frame_id, g.timestamp, g.depth_scale), (7, 1234.5, 0.001))
            # JPEG なので色は近似。赤が赤のまま（BGR）で届く
            np.testing.assert_allclose(g.color_bgr[20, 30], RED_BGR, atol=8)

    def test_depth_in_meters(self) -> None:
        g = decode_rgbd(encode_rgbd(sample_frame()))
        self.assertAlmostEqual(float(g.depth_m()[0, 5]), 0.005, places=6)

    def test_rejects_bad_messages(self) -> None:
        msg = encode_rgbd(sample_frame())
        with self.assertRaises(ValueError):
            decode_rgbd(b"XXXXXXXX" + msg[8:])
        with self.assertRaises(ValueError):
            decode_rgbd(msg[:-10])

    def test_rejects_unaligned_or_wrong_dtype(self) -> None:
        f = sample_frame()
        f.depth = f.depth[:-1]
        with self.assertRaises(ValueError):
            encode_rgbd(f)
        f = sample_frame()
        f.depth = f.depth.astype(np.float32)  # type: ignore[assignment]
        with self.assertRaises(ValueError):
            encode_rgbd(f)

    def test_legacy_rgb_is_compatible_with_zmq_frame_source(self) -> None:
        """RGB 互換の形式を、既存の ZmqFrameSource（Perception）がそのままデコードできる。"""
        rgb = np.full((48, 64, 3), 128, np.uint8)
        rgb[10:30, 20:40] = (255, 0, 0)  # 赤（RGB の並び。RealSense の rgb8 と同じ）
        src = perception("camera").ZmqFrameSource(server_address="localhost", camera_name="head_camera")
        src._socket = MagicMock()
        src._socket.recv_string.return_value = encode_legacy_rgb(rgb, "head_camera", 1.0)
        img = src.read()
        # 品質 80 の JPEG なので少し誤差が出る。色の順番が入れ替わっていれば 255 ずれる
        np.testing.assert_allclose(img[20, 30], RED_BGR, atol=20)


class TestServerLoopback(unittest.TestCase):
    def test_dummy_server_to_both_clients(self) -> None:
        cfg = load_config("depth_server.yaml")
        cfg["camera"]["source"] = "dummy"
        cfg["publish"]["bind_address"] = "127.0.0.1"
        cfg["publish"]["rgbd"]["port"] = free_port()
        cfg["publish"]["legacy_rgb"]["port"] = free_port()
        server = rgbd_server.RgbdServer(cfg)
        th = threading.Thread(target=server.run, daemon=True)
        th.start()
        try:
            with RgbdZmqSource("127.0.0.1", cfg["publish"]["rgbd"]["port"], 3000) as src:
                f = src.read_rgbd()
                self.assertIsNotNone(f)
                assert f is not None
                h, w = f.depth.shape
                self.assertEqual((h, w), (480, 640))
                self.assertEqual(int(f.depth[h // 2, int(w * 0.42)]), 600)  # 赤い箱
                self.assertEqual(int(f.depth[h - 5, w - 5]), 1000)  # 机
                self.assertEqual(int(f.depth[5, 5]), 0)  # 測れなかった画素
                np.testing.assert_allclose(f.color_bgr[h // 2, int(w * 0.42)], RED_BGR, atol=8)
                self.assertEqual(src.read().shape, (480, 640, 3))  # FrameSource の read() は BGR 画像だけ
            Z = perception("camera").ZmqFrameSource
            with Z("127.0.0.1", cfg["publish"]["legacy_rgb"]["port"], timeout_ms=3000) as z:
                img = z.read()
                self.assertIsNotNone(img)
                np.testing.assert_allclose(img[240, int(640 * 0.42)], RED_BGR, atol=8)
        finally:
            server.request_stop()
            th.join(timeout=5)
        self.assertFalse(th.is_alive())


class FakeRs(types.ModuleType):
    """pyrealsense2 の代わり（呼ばれ方を記録する）。"""

    def __init__(self) -> None:
        super().__init__("pyrealsense2")
        self.calls: list[tuple] = []
        rs = self

        class stream:  # noqa: N801
            color, depth = "color", "depth"

        class format:  # noqa: N801
            rgb8, z16 = "rgb8", "z16"

        class camera_info:  # noqa: N801
            name, serial_number = "name", "serial"

        class config:  # noqa: N801
            def enable_device(self, s: str) -> None:
                rs.calls.append(("enable_device", s))

            def enable_stream(self, *a: object) -> None:
                rs.calls.append(("enable_stream",) + a)

        intr = types.SimpleNamespace(width=640, height=480, fx=610.0, fy=611.0, ppx=320.5, ppy=241.5,
                                     model="distortion.inverse_brown_conrady", coeffs=[0.0] * 5)

        class _Frame:
            def __init__(self, arr: np.ndarray) -> None:
                self.arr = arr

            def get_data(self) -> np.ndarray:
                return self.arr

            def __bool__(self) -> bool:
                return True

        class _Frames:
            def get_color_frame(self) -> _Frame:
                return _Frame(np.zeros((480, 640, 3), np.uint8))

            def get_depth_frame(self) -> _Frame:
                return _Frame(np.full((480, 640), 777, np.uint16))

        class align:  # noqa: N801
            def __init__(self, to: str) -> None:
                rs.calls.append(("align", to))

            def process(self, frames: object) -> _Frames:
                return _Frames()

        device = MagicMock()
        device.get_info.side_effect = lambda k: {"name": "Intel RealSense D435I", "serial": "123"}[k]
        device.first_depth_sensor.return_value.get_depth_scale.return_value = 0.001

        class pipeline:  # noqa: N801
            def start(self, cfg: object) -> MagicMock:
                prof = MagicMock()
                prof.get_device.return_value = device
                prof.get_stream.return_value.as_video_stream_profile.return_value.get_intrinsics.return_value = intr
                return prof

            def wait_for_frames(self, timeout: int) -> object:
                return object()

            def stop(self) -> None:
                rs.calls.append(("stop",))

        self.stream, self.format, self.camera_info = stream, format, camera_info
        self.config, self.align, self.pipeline = config, align, pipeline


class TestRealSenseCameraWithFake(unittest.TestCase):
    def setUp(self) -> None:
        self.fake = FakeRs()
        sys.modules["pyrealsense2"] = self.fake

    def tearDown(self) -> None:
        sys.modules.pop("pyrealsense2", None)

    def _cam(self, serial: str) -> rgbd_server.RealSenseCamera:
        cfg = dict(load_config("depth_server.yaml")["camera"], serial=serial)
        cam = rgbd_server.RealSenseCamera(cfg)
        cam.start()
        return cam

    def test_serial_selects_device(self) -> None:
        self._cam("123")
        self.assertIn(("enable_device", "123"), self.fake.calls)

    def test_empty_serial_uses_first_device(self) -> None:
        self._cam("")
        self.assertFalse([c for c in self.fake.calls if c[0] == "enable_device"])

    def test_streams_aligned_to_color_and_intrinsics(self) -> None:
        cam = self._cam("")
        self.assertIn(("align", "color"), self.fake.calls)
        self.assertIn(("enable_stream", "color", 640, 480, "rgb8", 30), self.fake.calls)
        self.assertIn(("enable_stream", "depth", 640, 480, "z16", 30), self.fake.calls)
        assert cam.intrinsics is not None
        self.assertEqual((cam.intrinsics.fx, cam.intrinsics.cx, cam.intrinsics.cy), (610.0, 320.5, 241.5))
        f = cam.read()
        assert f is not None
        self.assertEqual(f.depth.dtype, np.uint16)
        self.assertEqual(int(f.depth[0, 0]), 777)
        self.assertLess(abs(f.timestamp - time.time()), 1.0)


if __name__ == "__main__":
    unittest.main()
