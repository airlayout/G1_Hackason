"""立った状態の深度から机の箱を作る（common/realday.py の table_boxes_from_points、replace_obstacle_group）。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.realday import load_obstacles, replace_obstacle_group, table_boxes_from_points


def synth_scene(tilt_deg: float = -5.0, top_z: float = -0.03, x0: float = 0.31) -> tuple[np.ndarray, float]:
    """机（手前の縁が x = x0 + tan(tilt) y で斜め）の天板の点、床、ボトル、下ろした手の点。"""
    rng = np.random.default_rng(1)
    a = np.tan(np.radians(tilt_deg))
    y = rng.uniform(-0.5, 0.5, 6000)
    x = rng.uniform(0.0, 0.4, 6000) + x0 + a * y
    desk = np.stack([x, y, top_z + rng.normal(0, 0.003, 6000)], axis=1)
    floor = np.stack([rng.uniform(0.15, 0.7, 3000), rng.uniform(-0.5, 0.5, 3000), rng.normal(-0.73, 0.004, 3000)], axis=1)
    bottle = np.stack([rng.normal(0.36, 0.01, 300), rng.normal(-0.17, 0.01, 300), rng.uniform(top_z, top_z + 0.2, 300)], axis=1)
    hand = np.stack([rng.normal(0.18, 0.02, 300), rng.normal(-0.19, 0.02, 300), rng.normal(-0.24, 0.01, 300)], axis=1)
    return np.vstack([desk, floor, bottle, hand]), a


class TestTableBoxes(unittest.TestCase):
    def test_follows_slanted_edge_and_ignores_floor_bottle_hand(self) -> None:
        pts, a = synth_scene()
        boxes, info = table_boxes_from_points(pts, margin_m=0.03, strip_m=0.09)
        self.assertAlmostEqual(info["z_top"], -0.03, delta=0.005)  # 床（−0.73）や手（−0.24）ではない
        self.assertGreaterEqual(len(boxes), 8)
        for b, st in zip(boxes, info["strips"]):
            c, h = np.array(b["center"]), np.array(b["half_size"])
            front = c[0] - h[0]
            # 手前の縁（余裕 3 cm 込み）は、その strip の中で一番手前の縁より少し手前（斜めなので strip の幅の分だけ）
            edge_min = 0.31 + a * st["y_hi"] if a < 0 else 0.31 + a * st["y_lo"]
            self.assertAlmostEqual(front, edge_min - 0.03, delta=0.02)  # 下位 2% の点なので、真の縁より最大 1〜2 cm 奥（余裕 3 cm の内）
            self.assertAlmostEqual(c[2] + h[2], -0.03 + 0.03, delta=0.006)  # 上面は天板 + 余裕
            self.assertAlmostEqual(c[2] - h[2], -0.03 - 0.8, delta=0.006)
            self.assertAlmostEqual(h[1], 0.09 / 2 + 0.03, places=3)
        # 斜めなので、右（y < 0）の縁のほうが奥にある（a < 0 なら y が大きいほど手前）
        fronts = [b["center"][0] - b["half_size"][0] for b in boxes]
        self.assertGreater(fronts[0], fronts[-1])

    def test_too_few_points_raises(self) -> None:
        with self.assertRaises(ValueError):
            table_boxes_from_points(np.zeros((10, 3)))

    def test_replace_group_removes_old_names(self) -> None:
        pts, _ = synth_scene()
        boxes, _ = table_boxes_from_points(pts)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "obstacles.yaml"
            replace_obstacle_group("table", boxes, "1 回目", path)
            replace_obstacle_group("other", boxes[:1], "別の物", path)
            n1 = len(load_obstacles(path))
            replace_obstacle_group("table", boxes[:3], "2 回目", path)
            names = [o["name"] for o in load_obstacles(path)]
        self.assertEqual(n1, len(boxes) + 1)
        self.assertEqual(sorted(names), ["other_1", "table_1", "table_2", "table_3"])  # 古い table_4… は残らない


if __name__ == "__main__":
    unittest.main()
