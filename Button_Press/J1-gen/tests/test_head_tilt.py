"""頭カメラの向きの推定（common/head_tilt.py）: 向きを決めて作った床・机の深度から、同じ向きが求まる。"""

from __future__ import annotations

import unittest

import numpy as np

from common.camera_geometry import R_LINK_OPTICAL, HeadCameraTransform, rpy_to_matrix
from common.config import load_config
from common.head_tilt import estimate_head_tilt, imu_up_in_pelvis, judge, pelvis_tilt_deg
from common.rgbd_protocol import Intrinsics, RgbdFrame


def synth_frame(tf: HeadCameraTransform, robot_cfg: dict, pitch_deg: float, roll_deg: float,
                planes: list[tuple[float, float]]) -> RgbdFrame:
    """水平な面（pelvis の z = 高さ。範囲は (z, x が前に何 m 以上か)）を、指定の向きのカメラから見た深度。"""
    w, h = 640, 360
    intr = Intrinsics(width=w, height=h, fx=454.0, fy=454.0, cx=320.0, cy=180.0)
    r, p = tf.pelvis_from_optical(np.zeros(3))
    r_parent = tf.data.oMf[tf.parent_frame].rotation
    rot = r_parent @ rpy_to_matrix(np.radians(roll_deg), np.radians(pitch_deg), float(robot_cfg["head_camera"]["rpy"][2])) @ R_LINK_OPTICAL
    v, u = np.mgrid[0:h, 0:w]
    rays = np.stack([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, np.ones_like(u, dtype=float)], axis=-1)
    dp = rays @ rot.T
    depth = np.zeros((h, w))
    for z_plane, x_min in planes:
        t = (z_plane - p[2]) / np.where(np.abs(dp[..., 2]) < 1e-9, 1e-9, dp[..., 2])
        pts_x = p[0] + t * dp[..., 0]
        m = (t > 0.2) & (t < 3.0) & (pts_x >= x_min) & (depth == 0)
        depth[m] = t[m]
    return RgbdFrame(color_bgr=np.zeros((h, w, 3), np.uint8), depth=np.round(depth * 1000).astype(np.uint16),
                     depth_scale=0.001, intrinsics=intr, timestamp=0.0, frame_id=0)


class TestHeadTilt(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config("robot.yaml")
        cls.tf = HeadCameraTransform(cls.cfg)

    def _check(self, pitch: float, roll: float) -> None:
        # 床（pelvis −0.73 m）と、前 0.3 m から先の机の面（−0.03 m）
        f = synth_frame(self.tf, self.cfg, pitch, roll, [(-0.03, 0.30), (-0.73, -9.0)])
        planes = estimate_head_tilt(f, self.tf, self.cfg, np.zeros(3))
        good = [p for p in planes if p.residual_deg < 2.0]
        self.assertGreaterEqual(len(good), 2)  # 床と机の両方
        for p in good:
            self.assertAlmostEqual(p.pitch_deg, pitch, delta=1.0)
            self.assertAlmostEqual(p.roll_deg, roll, delta=1.5)

    def test_recovers_neck_tilted_pitch(self) -> None:
        self._check(71.0, -1.4)  # 2026-10-21（首が傾いていた）

    def test_recovers_normal_pitch(self) -> None:
        self._check(46.0, 0.0)  # 2026-10-14

    def test_judge(self) -> None:
        f = synth_frame(self.tf, self.cfg, 71.0, 0.0, [(-0.03, 0.30), (-0.73, -9.0)])
        res = judge(estimate_head_tilt(f, self.tf, self.cfg, np.zeros(3)), 47.6)
        self.assertIn("首が傾いている", res["verdict"])
        f = synth_frame(self.tf, self.cfg, 46.0, 0.0, [(-0.03, 0.30), (-0.73, -9.0)])
        res = judge(estimate_head_tilt(f, self.tf, self.cfg, np.zeros(3)), 47.6)
        self.assertIn("傾いていない", res["verdict"])
        self.assertIn("見つからない", judge([], 47.6)["verdict"])

    def test_imu_up(self) -> None:
        np.testing.assert_allclose(imu_up_in_pelvis(None), [0, 0, 1])
        self.assertAlmostEqual(pelvis_tilt_deg(np.array([1.0, 0, 0, 0])), 0.0)
        a = np.radians(3.0) / 2  # x 軸まわり 3°
        self.assertAlmostEqual(pelvis_tilt_deg(np.array([np.cos(a), np.sin(a), 0, 0])), 3.0, places=3)


if __name__ == "__main__":
    unittest.main()
