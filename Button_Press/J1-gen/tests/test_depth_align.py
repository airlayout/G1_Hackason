"""深度をカラーに位置合わせする計算（common/depth_align.py）と、rgbd_server の --source videohub。

実機も RealSense も videohub も使わず、テスト用の値で確かめる。カメラの値は 2026-09-29 に PC2 で読んだものに近くしてある
（深度 640x480 fx=384.8、カラー 1920x1080 を 640x360 に縮めたもの、深度 → カラーは横に 1.5 cm）。
"""

from __future__ import annotations

import sys
import threading
import types
import unittest
from typing import Any

import cv2
import numpy as np

from common.camera_rgbd import RgbdZmqSource
from common.config import FEATURE_DIR, load_config
from common.depth_align import Extrinsics, align_depth_to_color, scale_intrinsics
from common.rgbd_protocol import Intrinsics

sys.path.insert(0, str(FEATURE_DIR / "real" / "depth_server"))
import rgbd_server  # noqa: E402
from test_rgbd import free_port  # noqa: E402

DEPTH_INTR = Intrinsics(640, 480, 384.8, 384.8, 325.4, 242.1)
COLOR_1080 = Intrinsics(1920, 1080, 1380.0, 1379.0, 958.0, 545.0, "distortion.inverse_brown_conrady", [0.0] * 5)
T_DEPTH_TO_COLOR = [0.015, -0.0004, 0.0003]


def project(p: np.ndarray, intr: Intrinsics) -> tuple[float, float]:
    """カメラ座標の点を画素に投影する（確かめる側の計算）。"""
    return p[0] / p[2] * intr.fx + intr.cx, p[1] / p[2] * intr.fy + intr.cy


def box_scene(near_mm: int = 600, far_mm: int = 1200) -> np.ndarray:
    """奥の壁（far_mm）の前に、四角い箱（near_mm）がある深度。"""
    depth = np.full((480, 640), far_mm, np.uint16)
    depth[180:300, 260:380] = near_mm
    return depth


class TestScaleIntrinsics(unittest.TestCase):
    def test_third_of_1080p(self) -> None:
        c = scale_intrinsics(COLOR_1080, 640, 360)
        self.assertEqual((c.width, c.height), (640, 360))
        self.assertAlmostEqual(c.fx, 460.0)
        self.assertAlmostEqual(c.fy, 1379.0 / 3)
        # 画素の中心を基準に縮める: 元の画素 958 の中心は、縮めた画像で (958 + 0.5) / 3 − 0.5
        self.assertAlmostEqual(c.cx, (958.0 + 0.5) / 3 - 0.5)
        self.assertAlmostEqual(c.cy, (545.0 + 0.5) / 3 - 0.5)
        self.assertEqual(c.model, COLOR_1080.model)

    def test_same_ray_hits_same_place(self) -> None:
        """同じ 3 次元の点は、縮める前と後で、同じ場所（画像の幅に対する割合）に写る。"""
        c = scale_intrinsics(COLOR_1080, 640, 360)
        p = np.array([0.1, -0.05, 0.8])
        u_big, v_big = project(p, COLOR_1080)
        u, v = project(p, c)
        self.assertAlmostEqual((u + 0.5) / 640, (u_big + 0.5) / 1920)
        self.assertAlmostEqual((v + 0.5) / 360, (v_big + 0.5) / 1080)


class TestExtrinsics(unittest.TestCase):
    def test_realsense_rotation_is_column_major(self) -> None:
        """librealsense は回転を列の順に並べる: to[0] = r[0]·x + r[3]·y + r[6]·z + t[0]。"""
        r = [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # 列の順
        e = Extrinsics.from_realsense(r, [0.1, 0.2, 0.3])
        p = np.array([1.0, 2.0, 3.0])
        expected = np.array([r[0] * 1 + r[3] * 2 + r[6] * 3, r[1] * 1 + r[4] * 2 + r[7] * 3, r[2] * 1 + r[5] * 2 + r[8] * 3])
        np.testing.assert_allclose(e.rotation @ p + e.translation, expected + [0.1, 0.2, 0.3])


class TestAlign(unittest.TestCase):
    def test_same_camera_is_unchanged(self) -> None:
        """同じカメラどうし（内部パラメータが同じ、位置関係なし）なら、入力とまったく同じになる。"""
        depth = box_scene()
        depth[0:10, 0:10] = 0
        out = align_depth_to_color(depth, 0.001, DEPTH_INTR, DEPTH_INTR, Extrinsics.identity())
        np.testing.assert_array_equal(out, depth)

    def test_box_lands_where_geometry_says(self) -> None:
        """箱の角が、3 次元の点を計算で投影した位置に来る。壁も箱も穴が空かない。"""
        color = scale_intrinsics(COLOR_1080, 640, 360)
        extr = Extrinsics(np.eye(3), np.array(T_DEPTH_TO_COLOR))
        depth = box_scene()
        out = align_depth_to_color(depth, 0.001, DEPTH_INTR, color, extr)
        self.assertEqual(out.shape, (360, 640))
        self.assertEqual(out.dtype, np.uint16)

        def corner(u: float, v: float, z: float) -> tuple[float, float]:
            p = np.array([(u - DEPTH_INTR.cx) / DEPTH_INTR.fx * z, (v - DEPTH_INTR.cy) / DEPTH_INTR.fy * z, z])
            return project(p + T_DEPTH_TO_COLOR, color)

        # 箱の左上の角（深度の画素 260, 180 の左上）と右下の角（379, 299 の右下）
        u0, v0 = corner(259.5, 179.5, 0.6)
        u1, v1 = corner(379.5, 299.5, 0.6)
        box = out == 600
        ys, xs = np.nonzero(box)
        self.assertLessEqual(abs(xs.min() - u0), 1.0)
        self.assertLessEqual(abs(xs.max() - u1), 1.0)
        self.assertLessEqual(abs(ys.min() - v0), 1.0)
        self.assertLessEqual(abs(ys.max() - v1), 1.0)
        # 箱の中は全部埋まっている（網目の穴が無い）
        inner = out[int(np.ceil(v0)) + 1:int(v1) - 1, int(np.ceil(u0)) + 1:int(u1) - 1]
        self.assertTrue(np.all(inner == 600))
        # 平行移動の z（0.3 mm）は丸めると消えるので、値は 0・600・1200 だけ
        vals = set(np.unique(out).tolist())
        self.assertTrue(vals <= {0, 600, 1200}, vals)

    def test_near_object_hides_far(self) -> None:
        """カメラの位置がずれていると、奥の壁のうち箱の陰になる部分と、箱の縁が同じ画素に来る。近いほうが残る。"""
        color = DEPTH_INTR
        extr = Extrinsics(np.eye(3), np.array([0.05, 0.0, 0.0]))  # 大きめにずらす
        out = align_depth_to_color(box_scene(), 0.001, DEPTH_INTR, color, extr)
        # 箱の真ん中の行: 箱は壁より大きく右にずれる（fx·t/z が大きい）。箱の範囲に壁の値が混じらない
        row = out[240]
        box_cols = np.nonzero(row == 600)[0]
        self.assertTrue(np.all(np.diff(box_cols) == 1))  # 箱は途切れない
        self.assertFalse(np.any(row[box_cols.min():box_cols.max() + 1] == 1200))
        # 箱のずれは約 384.8·0.05/0.6 ≈ 32 画素、壁のずれは約 16 画素。箱の左側に「影」（測れない画素）ができる
        self.assertAlmostEqual(float(box_cols.min()), 260 + 384.8 * 0.05 / 0.6, delta=1.5)
        self.assertTrue(np.any(row[:box_cols.min()] == 0))

    def test_zero_and_outside_stay_zero(self) -> None:
        depth = np.zeros((480, 640), np.uint16)
        out = align_depth_to_color(depth, 0.001, DEPTH_INTR, DEPTH_INTR, Extrinsics.identity())
        self.assertFalse(out.any())
        # カラーの画面の外に写る点は捨てる（大きく横にずらすと、右端の列は画面の外に出る）
        depth[:, :] = 1000
        out = align_depth_to_color(depth, 0.001, DEPTH_INTR, DEPTH_INTR, Extrinsics(np.eye(3), np.array([0.3, 0.0, 0.0])))
        self.assertEqual(out.shape, (480, 640))
        self.assertTrue(np.all(out[:, :50] == 0))  # 右にずれた分、左端は何も来ない

    def test_rejects_wrong_input(self) -> None:
        with self.assertRaises(ValueError):
            align_depth_to_color(np.zeros((480, 640), np.float32), 0.001, DEPTH_INTR, DEPTH_INTR, Extrinsics.identity())
        with self.assertRaises(ValueError):
            align_depth_to_color(np.zeros((240, 320), np.uint16), 0.001, DEPTH_INTR, DEPTH_INTR, Extrinsics.identity())


class FakeProfile:
    def __init__(self, stream: str, w: int, h: int, fps: int, intr: Intrinsics) -> None:
        self.stream, self.w, self.h, self.f, self.intr = stream, w, h, fps, intr

    def is_video_stream_profile(self) -> bool:
        return True

    def as_video_stream_profile(self) -> FakeProfile:
        return self

    def width(self) -> int:
        return self.w

    def height(self) -> int:
        return self.h

    def fps(self) -> int:
        return self.f

    def stream_type(self) -> str:
        return self.stream

    def get_intrinsics(self) -> types.SimpleNamespace:
        i = self.intr
        return types.SimpleNamespace(width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.cx, ppy=i.cy,
                                     model=i.model, coeffs=list(i.coeffs))

    def get_extrinsics_to(self, other: FakeProfile) -> types.SimpleNamespace:
        assert (self.stream, other.stream) == ("depth", "color")
        return types.SimpleNamespace(rotation=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0], translation=list(T_DEPTH_TO_COLOR))


class FakeRsVideohub(types.ModuleType):
    """pyrealsense2 の代わり。カラーは設定（stream profile）を読まれるだけで、開かれない（開くと失敗する）。"""

    def __init__(self, depth: np.ndarray) -> None:
        super().__init__("pyrealsense2")
        self.calls: list[tuple] = []
        rs = self

        class stream:  # noqa: N801
            color, depth = "color", "depth"

        class format:  # noqa: N801
            rgb8, z16 = "rgb8", "z16"

        class camera_info:  # noqa: N801
            name, serial_number = "name", "serial"

        color_profiles = [FakeProfile("color", 640, 480, 30, Intrinsics(640, 480, 605.5, 605.1, 318.6, 254.8)),
                          FakeProfile("color", 1920, 1080, 15, COLOR_1080),
                          FakeProfile("color", 1920, 1080, 30, COLOR_1080)]
        depth_profiles = [FakeProfile("depth", 640, 480, 30, DEPTH_INTR), FakeProfile("depth", 1280, 720, 30, DEPTH_INTR)]

        class _Sensor:
            def __init__(self, profiles: list[FakeProfile]) -> None:
                self.profiles = profiles

            def get_stream_profiles(self) -> list[FakeProfile]:
                return self.profiles

        class _Device:
            def get_info(self, k: str) -> str:
                return {"name": "Intel RealSense D435I", "serial": "250122075509"}[k]

            def query_sensors(self) -> list[_Sensor]:
                return [_Sensor(depth_profiles), _Sensor(color_profiles)]

            def first_depth_sensor(self) -> Any:
                return types.SimpleNamespace(get_depth_scale=lambda: 0.001)

        class context:  # noqa: N801
            def query_devices(self) -> list[_Device]:
                return [_Device()]

        class config:  # noqa: N801
            def enable_device(self, s: str) -> None:
                rs.calls.append(("enable_device", s))

            def enable_stream(self, *a: object) -> None:
                if a[0] == "color":
                    raise RuntimeError("Device or resource busy")  # videohub が開いている
                rs.calls.append(("enable_stream",) + a)

        class _Frame:
            def get_data(self) -> np.ndarray:
                return depth

            def __bool__(self) -> bool:
                return True

        class pipeline:  # noqa: N801
            def start(self, cfg: object) -> Any:
                rs.calls.append(("start",))
                prof = types.SimpleNamespace(
                    get_device=lambda: _Device(),
                    get_stream=lambda s: types.SimpleNamespace(as_video_stream_profile=lambda: depth_profiles[0]),
                )
                return prof

            def wait_for_frames(self, timeout: int) -> Any:
                rs.calls.append(("wait",))
                return types.SimpleNamespace(get_depth_frame=lambda: _Frame())

            def stop(self) -> None:
                rs.calls.append(("stop",))

        self.stream, self.format, self.camera_info = stream, format, camera_info
        self.context, self.config, self.pipeline = context, config, pipeline


class FakeVideoClient:
    """VideoClient の代わり。赤い四角のある 1920x1080 の JPEG を返す（JPEG は BGR の並びで作る）。"""

    def __init__(self, fail_first: int = 0) -> None:
        img = np.full((1080, 1920, 3), 128, np.uint8)
        img[300:600, 600:900] = (0, 0, 255)  # 赤（BGR）
        self.jpeg = cv2.imencode(".jpg", img)[1].tobytes()
        self.fail_first = fail_first
        self.n = 0

    def GetImageSample(self) -> tuple[int, list[int]]:  # noqa: N802
        self.n += 1
        if self.n <= self.fail_first:
            return 3104, []
        return 0, list(self.jpeg)


class TestVideohubCamera(unittest.TestCase):
    def setUp(self) -> None:
        self.depth = box_scene()
        self.fake = FakeRsVideohub(self.depth)
        sys.modules["pyrealsense2"] = self.fake
        self.client = FakeVideoClient()

    def tearDown(self) -> None:
        sys.modules.pop("pyrealsense2", None)

    def _cam(self, **vh: Any) -> rgbd_server.VideohubCamera:
        cfg = load_config("depth_server.yaml")["camera"]
        cfg["videohub"].update(vh)
        cam = rgbd_server.VideohubCamera(cfg)
        cam._make_client = lambda: self.client  # type: ignore[method-assign]
        cam.start()
        return cam

    def test_opens_depth_only_and_reads_color_params(self) -> None:
        cam = self._cam()
        streams = [c for c in self.fake.calls if c[0] == "enable_stream"]
        self.assertEqual(streams, [("enable_stream", "depth", 640, 480, "z16", 30)])  # カラーは開かない
        self.assertIn(("enable_device", "250122075509"), self.fake.calls)
        # 起動直後の深度を捨てる
        self.assertEqual(self.fake.calls.count(("wait",)), 15)
        ci = cam.intrinsics
        assert ci is not None
        self.assertEqual((ci.width, ci.height), (640, 360))
        self.assertAlmostEqual(ci.fx, 460.0)
        np.testing.assert_allclose(cam._extr.translation, T_DEPTH_TO_COLOR)  # type: ignore[union-attr]

    def test_frame_is_aligned_rgb(self) -> None:
        cam = self._cam()
        f = cam.read()
        assert f is not None
        self.assertIsNone(f.color_rgb)  # read() は深度だけ
        g = cam.complete(f)
        assert g is not None and g.color_rgb is not None
        self.assertEqual(g.color_rgb.shape, (360, 640, 3))
        self.assertEqual(g.depth.shape, (360, 640))
        # 赤い四角（元の 600:900, 300:600 → 縮めて 200:300, 100:200）の中心が RGB の赤
        np.testing.assert_allclose(g.color_rgb[150, 250], [255, 0, 0], atol=10)
        # 深度は位置合わせ済み: 箱（0.6 m）と壁（1.2 m）がある
        self.assertIn(600, np.unique(g.depth))
        self.assertIn(1200, np.unique(g.depth))
        self.assertIn("位置合わせ", cam.status())

    def test_retries_until_videohub_answers(self) -> None:
        self.client = FakeVideoClient(fail_first=3)
        cam = self._cam()
        self.assertIsNotNone(cam.intrinsics)
        self.client.fail_first = 10**9  # 以降は受け取れない
        f = cam.read()
        assert f is not None
        self.assertIsNone(cam.complete(f))

    def test_different_size_from_videohub_is_scaled(self) -> None:
        """videohub が 1280x720 を返しても、縦横比が同じなら内部パラメータを縮めて使う。"""
        small = cv2.resize(cv2.imdecode(np.frombuffer(self.client.jpeg, np.uint8), cv2.IMREAD_COLOR), (1280, 720))
        self.client.jpeg = cv2.imencode(".jpg", small)[1].tobytes()
        cam = self._cam()
        ci = cam.intrinsics
        assert ci is not None
        self.assertEqual((ci.width, ci.height), (640, 360))
        self.assertAlmostEqual(ci.fx, 460.0)

    def test_server_publishes_aligned_frames(self) -> None:
        """サーバの配信ループで complete() が呼ばれ、受け取る側に 640x360 のカラーと深度が届く。"""
        cfg = load_config("depth_server.yaml")
        cfg["camera"]["source"] = "videohub"
        cfg["publish"]["bind_address"] = "127.0.0.1"
        cfg["publish"]["rgbd"]["port"] = free_port()
        cfg["publish"]["legacy_rgb"]["enabled"] = False
        cfg["publish"]["max_fps"] = 0
        server = rgbd_server.RgbdServer(cfg)
        self.assertIsInstance(server.camera, rgbd_server.VideohubCamera)
        server.camera._make_client = lambda: self.client  # type: ignore[union-attr]
        th = threading.Thread(target=server.run, daemon=True)
        th.start()
        try:
            with RgbdZmqSource("127.0.0.1", cfg["publish"]["rgbd"]["port"], 5000) as src:
                f = src.read_rgbd()
                assert f is not None
                self.assertEqual(f.color_bgr.shape, (360, 640, 3))
                self.assertEqual(f.depth.shape, (360, 640))
                self.assertEqual((f.intrinsics.width, f.intrinsics.height), (640, 360))
                np.testing.assert_allclose(f.color_bgr[150, 250], [0, 0, 255], atol=10)  # BGR で赤
        finally:
            server.request_stop()
            th.join(timeout=5)
        self.assertFalse(th.is_alive())


if __name__ == "__main__":
    unittest.main()
