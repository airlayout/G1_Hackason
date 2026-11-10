"""押している間・戻る間もランプと扉を見る（common/lamp_watch.py と、common/elevator_press.py の使い方）。

2026-11-03 は、確認が当たってから約 3.5 秒後の 1 枚だけで、かごがその階にいる・すぐ来るとランプが消えていて、
人の目では点灯していたのに「点かず」と判定した。一度でも点灯を見ていれば「点灯した」とする。
"""

from __future__ import annotations

import unittest

import numpy as np

from _env import needs
from common.config import load_config
from common.lamp_watch import DoorWatch, LampWatch, convex_mask, hand_mask, project

K = np.array([[454.0, 0.0, 320.0], [0.0, 454.0, 180.0], [0.0, 0.0, 1.0]])
# 光学座標 = pelvis 座標（rot = 単位行列、pos = 0）の、カメラが +z を向いた簡単な配置
ROT, POS = np.eye(3), np.zeros(3)


class TestGeometry(unittest.TestCase):
    def test_project_and_behind_camera(self) -> None:
        uv, z = project(np.array([[0.1, 0.0, 1.0], [0.0, 0.0, -1.0]]), K, ROT, POS)
        np.testing.assert_allclose(uv[0], [365.4, 180.0])
        self.assertTrue(np.all(np.isnan(uv[1])))
        self.assertLess(z[1], 0)

    def test_hand_mask_only_in_region(self) -> None:
        m = hand_mask((100, 100), [(50.0, 50.0, 10.0)])
        self.assertTrue(m[50, 50])
        self.assertFalse(m[50, 65])
        part = hand_mask((100, 100), [(50.0, 50.0, 10.0)], region=(50, 0, 100, 100))
        self.assertFalse(part[50, 45])
        self.assertTrue(part[50, 55])

    def test_convex_mask(self) -> None:
        m = convex_mask((50, 50), np.array([[10.0, 10.0], [40.0, 10.0], [40.0, 30.0], [10.0, 30.0]]))
        self.assertTrue(m[20, 20])
        self.assertFalse(m[5, 20])
        self.assertEqual(int(m.sum()), 30 * 20)


class TestLampWatch(unittest.TestCase):
    def test_latches_first_lit(self) -> None:
        w = LampWatch()
        e = lambda t, lit: {"t": t, "phase": "hold", "retry": 0, "level": 0.0, "rise": 100.0 if lit else 0.0,
                            "visible": 1.0, "used": True, "lit": lit}
        self.assertFalse(w.add(e(1.0, False)))
        self.assertTrue(w.add(e(1.2, True)))
        self.assertFalse(w.add(e(1.4, True)))
        self.assertEqual(w.seen["t"], 1.2)
        self.assertEqual(w.summary()["frames"], 3)


class TestDoorWatch(unittest.TestCase):
    """扉は、▲▼ の中点の左（+y）の柱の面の続き。開くと、その範囲の深度が奥へ変わる。"""

    def setUp(self) -> None:
        # カメラは pelvis の原点で真正面を向き、柱の面は前 0.5 m（押す向き +x）
        self.w = DoorWatch({"left_m": [0.05, 0.25], "height_m": [-0.1, 0.1], "min_depth_increase_m": 0.15,
                            "min_fraction": 0.3, "min_pixels": 50})
        self.depth = np.full((360, 640), 0.5)

    def start(self) -> int:
        rot = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # 光学 (右, 下, 前) → pelvis (前, 左, 上)
        return self.w.start(np.array([0.5, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), self.depth, K, rot, POS)

    def test_open_after_two_frames(self) -> None:
        n = self.start()
        self.assertGreater(n, 1000)
        region = self.w.region
        self.assertTrue(region[:, :320].any())  # 左（ロボットの左）は画像の左
        self.assertFalse(region[:, 330:].any())
        opened = self.depth.copy()
        opened[region] = 2.0
        self.assertAlmostEqual(self.w.observe(self.depth, None, 1.0), 0.0)
        self.w.observe(opened, None, 2.0)
        self.assertIsNone(self.w.opened)  # 1 枚だけでは開いたとしない
        self.w.observe(opened, None, 2.2)
        self.assertEqual(self.w.opened["t"], 2.2)
        self.assertAlmostEqual(self.w.summary()["max_fraction"], 1.0)


@needs("mujoco", "pin")
class TestControllerUsesSeenLamp(unittest.TestCase):
    """確認のときに消えていても、押している間に点灯を見ていれば「点灯した」とし、押し直さない。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_buttons import Button3D, CallButtons
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"), load_config("elevator.yaml"))
        cls.call = CallButtons(Button3D(np.array([0.309, 0.027, 0.267]), 0.033, "up", (300, 200, 340, 240), 100, "up"),
                               Button3D(np.array([0.309, 0.027, 0.187]), 0.033, "down", (300, 300, 340, 340), 100, "down"),
                               np.array([1.0, 0.0, 0.0]))
        cls.q0 = np.zeros(29)
        cls.q0[[15, 22]] = 0.2
        cls.q0[16], cls.q0[23] = 0.25, -0.25
        cls.q0[[18, 25]] = 1.0

    def ready(self) -> None:
        c = self.ctl
        c.reset("down", 0.02, np.full(17, 60.0))
        c.call = self.call
        c._plan(self.q0, c.call)
        c.q_hold = self.q0.copy()
        c.baseline_brightness = c.baseline_level = 60.0

    def test_seen_lamp_counts_as_lit(self) -> None:
        self.ready()
        self.ctl.lamp.add({"t": 6.5, "phase": "hold", "retry": 0, "level": 232.0, "rise": 172.0, "visible": 0.6,
                           "used": True, "lit": True})
        self.assertTrue(self.ctl._check(np.full((480, 640, 3), 60, np.uint8), 9.9))
        self.assertEqual(self.ctl.outcome, "点灯した（確認のときには消えていた）")
        self.assertEqual(self.ctl.retries, 0)
        self.assertEqual(self.ctl.phase, "home")

    def test_without_seen_lamp_it_retries(self) -> None:
        self.ready()
        self.assertFalse(self.ctl._check(np.full((480, 640, 3), 60, np.uint8), 9.9))
        self.assertEqual(self.ctl.phase, "press")

    def test_hand_pixels_are_excluded(self) -> None:
        """白い手が枠に被さっても、その画素を除けば明るく見えない（11/10 の MuJoCo で、上の指が枠に被さって「点灯」と出た）。"""
        img = np.full((480, 640, 3), 60, np.uint8)
        img[300:340, 320:345] = 230  # 枠の右半分に白い手
        covered = np.zeros((480, 640), dtype=bool)
        covered[295:345, 318:350] = True
        bright, _ = self.ctl._lit_level_visible(img, (300, 300, 340, 340))
        level, visible = self.ctl._lit_level_visible(img, (300, 300, 340, 340), covered)
        self.assertGreater(bright, 200)
        self.assertAlmostEqual(level, 60.0)
        self.assertGreater(visible, 0.3)
        self.assertLess(visible, 0.8)

    def test_watch_records_frames_and_masks_the_hand(self) -> None:
        """_watch が、手の写る画素（メッシュから求める）を除いて 1 枚分を記録する。"""
        self.ready()
        self.ctl.phase = "hold"
        self.ctl._watch(np.full((360, 640, 3), 60, np.uint8), np.full((360, 640), 0.5), K, self.q0, 7.0)
        self.assertEqual(len(self.ctl.lamp.trace), 1)
        e = self.ctl.lamp.trace[0]
        self.assertFalse(e["lit"])
        self.assertGreaterEqual(e["visible"], 0.0)


if __name__ == "__main__":
    unittest.main()
