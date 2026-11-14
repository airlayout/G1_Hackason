"""ボトルを押せる範囲の地図（sim/reach_map.py）の部品。計画そのものは test_kinematics.py などで確かめている。"""

from __future__ import annotations

import sys
import unittest

import numpy as np

from _env import needs
from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "sim"))
import reach_map as rm  # noqa: E402


class TestReachMapParts(unittest.TestCase):
    def test_desk_boxes_follow_a_slanted_edge(self) -> None:
        boxes = rm.desk_boxes(0.32, 0.0, slope=-0.033, margin=0.03, strip=0.09, y_range=(-0.45, 0.0))
        self.assertEqual(len(boxes), 5)
        fronts = [b["center"][0] - b["half_size"][0] for b in boxes]
        # 左（y が大きい）ほど縁が手前（slope < 0）
        self.assertTrue(all(a > b for a, b in zip(fronts, fronts[1:])))
        # y = −0.09〜0 の箱: 縁は y = 0 側が手前（x = 0.32）、余裕 3 cm を引く
        self.assertAlmostEqual(fronts[-1], 0.32 - 0.03, places=4)
        b = boxes[-1]
        self.assertAlmostEqual(b["center"][2] + b["half_size"][2], 0.03, places=4)   # 上面 = 天板 + 余裕
        self.assertAlmostEqual(b["center"][2] - b["half_size"][2], -0.8, places=4)

    def test_classify_failures(self) -> None:
        self.assertEqual(rm.classify("経由の姿勢へ移動する途中（2/30）で腕が体にぶつかる: right_wrist_yaw_link ↔ right_hip_roll_link（距離 -13.7 mm）"), ("1", "body"))
        self.assertEqual(rm.classify("経由の姿勢へ移動する途中（8/30）で腕が体にぶつかる: right_wrist_yaw_link ↔ obstacle_0_table（距離 -4.5 mm）"), ("1", "desk"))
        self.assertEqual(rm.classify("手前の姿勢で腕が体にぶつかる: torso_link ↔ right_wrist_roll_link（距離 +9.5 mm）"), ("A", "body"))
        self.assertEqual(rm.classify("押し込みの直線の 12/90 点目で関節が 1 周期に 0.05 rad 動く"), ("J", ""))
        self.assertEqual(rm.classify("手前の点の手先が作業空間の外"), ("W", ""))
        self.assertEqual(rm.classify("手前の姿勢に届かない: 収束しない"), ("R", ""))
        # 経由の姿勢なしの直接の移動（「経由の姿勢から手前の姿勢へ移動する途中」と取り違えない）
        self.assertEqual(rm.classify("手前の姿勢へ移動する途中（16/30）で腕が体にぶつかる: torso_link ↔ right_elbow_link（距離 -3.0 mm）"), ("D", "body"))
        self.assertEqual(rm.classify("経由の姿勢から手前の姿勢へ移動する途中（17/30）で腕が体にぶつかる: right_wrist_yaw_link ↔ obstacle_3_table（距離 -1.0 mm）"), ("2", "desk"))

    @needs("pin")
    def test_bottle_visibility(self) -> None:
        from common.camera_geometry import HeadCameraTransform
        from common.config import load_config

        tf = HeadCameraTransform(load_config("robot.yaml"))
        self.assertTrue(rm.bottle_visible(tf, 0.34, -0.12, 0.0))      # 10/21 の配置に近い（見えた）
        self.assertFalse(rm.bottle_visible(tf, 0.34, -0.30, 0.0))     # 右へ寄りすぎて画像の外
        self.assertFalse(rm.bottle_visible(tf, 0.16, -0.12, 0.0))     # 近すぎて上が画像の外


if __name__ == "__main__":
    unittest.main()
