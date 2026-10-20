"""評価セット realistic の乱し（common/realism.py）のテスト。numpy と PyYAML だけで動く。"""

from __future__ import annotations

import unittest

import numpy as np

from common.config import load_config
from common.realism import ImageDelay, SensorModel, perturbed_robot_cfg, sample_realism, sway_offset
from contest.task import make_trial


class TestRealism(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_config("contest.yaml")

    def test_same_seed_same_realism_and_same_scene_as_basic(self) -> None:
        """同じ種なら乱しも同じ。盤の配置などは basic と同じ（乱しは別の乱数で決めるため）。"""
        a = make_trial(5, self.cfg, eval_set="realistic")
        b = make_trial(5, self.cfg, eval_set="realistic")
        basic = make_trial(5, self.cfg, eval_set="basic")
        self.assertEqual(a.realism.summary(), b.realism.summary())
        self.assertEqual(a.scene_cfg, basic.scene_cfg)
        self.assertEqual(a.target, basic.target)
        self.assertIsNone(basic.realism)
        self.assertNotEqual(make_trial(6, self.cfg, eval_set="realistic").realism.summary(), a.realism.summary())

    def test_values_in_range(self) -> None:
        c = self.cfg["realism"]
        for seed in range(30):
            r = sample_realism(seed, c)
            self.assertTrue(c["latency"]["image_s"][0] <= r.image_latency_s <= c["latency"]["image_s"][1])
            self.assertTrue(np.all(np.abs(r.camera_xyz_offset) <= c["camera_mount"]["xyz_m"]))
            self.assertTrue(np.all(np.abs(np.degrees(r.camera_rpy_offset)) <= c["camera_mount"]["rpy_deg"] + 1e-9))
            self.assertTrue(np.all((r.kp_scale >= 0.9) & (r.kp_scale <= 1.1)))
            self.assertLessEqual(abs(np.degrees(r.stance_yaw)), c["stance"]["yaw_deg"] + 1e-9)

    def test_sway_within_amplitude(self) -> None:
        r = sample_realism(0, self.cfg["realism"])
        offs = np.array([sway_offset(r, t) for t in np.linspace(0, 20, 2000)])
        self.assertTrue(np.all(np.abs(offs) <= r.sway_amp + 1e-12))
        self.assertGreater(np.ptp(offs[:, 0]), 0.5 * r.sway_amp[0])  # ちゃんと揺れている

    def test_depth_noise_scales_with_distance(self) -> None:
        """深度のノイズの標準偏差 ≈ sigma_at_1m × 距離²。1 mm 単位に丸める。範囲の外は 0。"""
        r = sample_realism(0, self.cfg["realism"])
        import dataclasses

        r = dataclasses.replace(r, depth_hole_ratio=0.0, depth_edge_hole_prob=0.0)
        s = SensorModel(r)
        for z in (0.4, 1.0):
            out = s.depth(np.full((240, 320), z, dtype=np.float32))
            self.assertAlmostEqual(float(out.std()), r.depth_sigma_at_1m * z ** 2, delta=0.4 * r.depth_sigma_at_1m * z ** 2 + 0.0006)
            np.testing.assert_allclose(out * 1000, np.round(out * 1000), atol=1e-3)
        self.assertTrue(np.all(s.depth(np.full((10, 10), 20.0, dtype=np.float32)) == 0))

    def test_image_delay(self) -> None:
        d = ImageDelay(latency_s=0.1, fps=10.0)
        for k in range(30):
            t = k * 0.02
            if d.due(t):
                d.push(t, t)
        self.assertAlmostEqual(d.get(0.58)[0], 0.4, places=6)  # 0.58 − 0.1 = 0.48 までに撮った最新は 0.4（10 fps）

    def test_camera_mount_only_changes_camera(self) -> None:
        from common.j1gen_bridge import j1gen_config

        rc = j1gen_config("robot.yaml")
        r = sample_realism(0, self.cfg["realism"])
        p = perturbed_robot_cfg(rc, r)
        np.testing.assert_allclose(np.asarray(p["head_camera"]["xyz"]) - rc["head_camera"]["xyz"], r.camera_xyz_offset)
        self.assertEqual(p["end_effector"], rc["end_effector"])
        self.assertNotEqual(p["head_camera"]["xyz"], rc["head_camera"]["xyz"])


if __name__ == "__main__":
    unittest.main()
