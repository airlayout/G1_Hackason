"""ボトル検出 → 3D 座標 → pelvis 座標（タスク4）。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from _env import needs
from common.camera_geometry import R_LINK_OPTICAL, HeadCameraTransform
from common.config import FEATURE_DIR, load_config
from common.localize import Locator, anchor_pixel, bbox_depth, deproject, make_detector
from common.rgbd_io import load_rgbd, save_rgbd
from common.rgbd_protocol import Intrinsics, RgbdFrame

sys.path.insert(0, str(FEATURE_DIR / "sim"))
from locate_sim import locate_in_sim  # noqa: E402

INTR = Intrinsics(640, 480, 600.0, 610.0, 319.5, 239.5)


class TestDepthAndDeprojection(unittest.TestCase):
    def test_deproject(self) -> None:
        np.testing.assert_allclose(deproject(319.5, 239.5, 1.0, INTR), [0, 0, 1])
        np.testing.assert_allclose(deproject(319.5 + 60, 239.5 - 61, 2.0, INTR), [0.2, -0.2, 2.0])

    def test_bbox_depth_ignores_zero_and_out_of_range(self) -> None:
        d = np.full((100, 100), 0.5)
        d[40:60, 40:50] = 0.0  # 測れなかった
        d[40:45, 50:60] = 9.0  # 範囲外
        z, n, _ = bbox_depth(d, (30, 30, 70, 70), 0.5, 0.15, 1.5, 20)
        self.assertAlmostEqual(z, 0.5)
        self.assertGreater(n, 20)

    def test_bbox_depth_fails_on_transparent(self) -> None:
        d = np.zeros((100, 100))
        z, n, reason = bbox_depth(d, (30, 30, 70, 70), 0.5, 0.15, 1.5, 20)
        self.assertIsNone(z)
        self.assertIn("透明", reason)

    def test_anchor(self) -> None:
        self.assertEqual(anchor_pixel((10, 20, 30, 120), (0.5, 0.7)), (20, 90))


@needs('pin')
class TestTransform(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_config("robot.yaml")
        cls.t = HeadCameraTransform(cls.robot)

    def test_matches_urdf_d435_link(self) -> None:
        """設定の取り付け位置で計算した変換が、URDF の d435_link と一致する（腰を動かしても）。"""
        pin = self.t._pin
        fid = self.t.model.getFrameId("d435_link")
        for qw in ([0, 0, 0], [0.3, -0.1, 0.2], [-0.2, 0.05, -0.1]):
            r, p = self.t.pelvis_from_optical(np.array(qw))
            q = np.zeros(29)
            q[12:15] = qw
            pin.framesForwardKinematics(self.t.model, self.t.data, q)
            f = self.t.data.oMf[fid]
            np.testing.assert_allclose(p, f.translation, atol=1e-9)
            np.testing.assert_allclose(r, f.rotation @ R_LINK_OPTICAL, atol=1e-9)

    def test_optical_axis_looks_down_forward(self) -> None:
        r, _ = self.t.pelvis_from_optical(np.zeros(3))
        axis = r @ [0, 0, 1]
        self.assertGreater(axis[0], 0.6)  # 前
        self.assertLess(axis[2], -0.7)  # 下（約 47.6°）

    def test_calibration_offset_added(self) -> None:
        t2 = HeadCameraTransform(self.robot, np.array([0.01, -0.02, 0.03]))
        p = np.array([0.1, 0.0, 0.5])
        np.testing.assert_allclose(t2.to_pelvis(p, np.zeros(3)) - self.t.to_pelvis(p, np.zeros(3)),
                                   [0.01, -0.02, 0.03], atol=1e-12)


@needs('render', 'pin')
class TestSimCamera(unittest.TestCase):
    def test_deprojection_matches_mujoco_rays(self) -> None:
        """MuJoCo の深度から逆投影した点が、同じ画素へ飛ばした光線の当たる点と一致する（腰を動かしても）。"""
        import mujoco

        from common.sim_camera import SimHeadCamera
        from common.sim_scene import build_scene_model

        robot = load_config("robot.yaml")
        m = build_scene_model(robot, load_config("sim_scene.yaml"))
        d = mujoco.MjData(m)
        tr = HeadCameraTransform(robot)
        for qw in ([0, 0, 0], [0.15, 0.05, 0.1]):
            d.qpos[:] = 0
            d.qpos[12:15] = qw
            mujoco.mj_forward(m, d)
            cam = SimHeadCamera(m, d, robot)
            f = cam.render()
            dm = f.depth_m()
            for u, v in [(320, 240), (100, 300), (560, 200), (320, 420), (50, 150), (600, 440)]:
                p = tr.to_pelvis(deproject(u, v, float(dm[v, u]), f.intrinsics), np.array(qw))
                np.testing.assert_allclose(p, cam.ray_hit_pelvis(u, v), atol=0.002)
            cam.close()

    def test_locate_bottle_in_sim(self) -> None:
        r = locate_in_sim()
        self.assertIsNotNone(r["best"])
        self.assertLess(r["err_ray"], 0.002)
        self.assertLess(abs(r["err_front_x"]), 0.005)  # 胴の前面から 5 mm 以内
        self.assertLess(abs(r["err_center_y"]), 0.010)  # 左右はボトルの中心から 1 cm 以内


class FakeDetector:
    def __init__(self, bbox: tuple[float, float, float, float]) -> None:
        from common.localize import Detection

        self.det = Detection("bottle", 0.9, bbox)

    def detect(self, frame: np.ndarray) -> list:
        return [self.det]


@needs('pin')
class TestLocator(unittest.TestCase):
    def frame(self, depth_mm: int) -> RgbdFrame:
        depth = np.zeros((480, 640), np.uint16)
        depth[200:300, 300:340] = depth_mm
        return RgbdFrame(np.zeros((480, 640, 3), np.uint8), depth, 0.001, INTR, 0.0, 1)

    def test_locator_with_fake_detector(self) -> None:
        loc = Locator(load_config("robot.yaml"), load_config("localize.yaml"), FakeDetector((300, 200, 340, 300)))
        found = loc.locate(self.frame(500), np.zeros(3))
        best = loc.best(found)
        assert best is not None and best.p_optical is not None
        self.assertAlmostEqual(best.depth_m, 0.5)
        np.testing.assert_allclose(best.p_optical[2], 0.5)

    def test_no_depth_is_reported(self) -> None:
        loc = Locator(load_config("robot.yaml"), load_config("localize.yaml"), FakeDetector((300, 200, 340, 300)))
        found = loc.locate(self.frame(0), np.zeros(3))
        self.assertFalse(found[0].ok)
        self.assertIsNone(loc.best(found))
        self.assertTrue(found[0].reason)

    @needs('yolo')
    def test_yolo_detector_builds(self) -> None:
        """Perception の YoloDetector が bottle クラスで作れて、画像を処理できる（重みは初回に自動ダウンロード）。"""
        det = make_detector(load_config("localize.yaml"))
        self.assertIsInstance(det.detect(np.zeros((480, 640, 3), np.uint8)), list)


class TestRgbdIO(unittest.TestCase):
    def test_save_and_load(self) -> None:
        depth = (np.arange(480 * 640).reshape(480, 640) % 60000).astype(np.uint16)
        f = RgbdFrame(np.full((480, 640, 3), 7, np.uint8), depth, 0.001, INTR, 12.5, 3)
        with tempfile.TemporaryDirectory() as tmp:
            stem = Path(tmp) / "x"
            save_rgbd(f, stem, {"waist_q": [0.1, 0.2, 0.3]})
            g, meta = load_rgbd(stem)
        np.testing.assert_array_equal(g.depth, depth)
        self.assertEqual(g.intrinsics, INTR)
        self.assertEqual(meta["waist_q"], [0.1, 0.2, 0.3])


if __name__ == "__main__":
    unittest.main()
