"""common/scene_spec.py（シミュレーターに依存しない部分）のテスト。numpy と PyYAML だけで動く。"""

from __future__ import annotations

import unittest

import numpy as np

from common.config import load_config
from common.scene_spec import CallButtonState, build_hall_scene, carpet_texture


class TestHallScene(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_config("elevator_hall.yaml")
        self.scene = build_hall_scene(self.cfg)

    def test_buttons_on_panel_surface(self) -> None:
        """ボタンの根元が盤の表面（ロボット側の面）にあり、面は盤の範囲の中にある。"""
        panel = next(b for b in self.scene.boxes if b.name == "hall_panel")
        panel_front = panel.center[0] - panel.half_size[0]
        for b in self.scene.buttons:
            self.assertAlmostEqual(b.base_center[0], panel_front, places=9)
            lo = panel.center[1:] - panel.half_size[1:]
            hi = panel.center[1:] + panel.half_size[1:]
            self.assertTrue(np.all(b.face_center[1:] - b.radius >= lo), b.name)
            self.assertTrue(np.all(b.face_center[1:] + b.radius <= hi), b.name)

    def test_heights_from_floor(self) -> None:
        """高さは床からの値で書き、pelvis 座標では床が −pelvis_height になる。"""
        self.assertAlmostEqual(self.scene.floor_z, -self.cfg["pelvis_height"])
        up = self.scene.button("up")
        expected = self.cfg["panel"]["center_height"] + self.cfg["buttons"]["items"][0]["offset_z"]
        self.assertAlmostEqual(up.face_center[2] - self.scene.floor_z, expected)
        self.assertGreater(up.face_center[2], self.scene.button("down").face_center[2])

    def test_panel_does_not_overlap_door(self) -> None:
        panel = next(b for b in self.scene.boxes if b.name == "hall_panel")
        frame = next(b for b in self.scene.boxes if b.name == "hall_door_frame_right")
        self.assertLess(panel.center[1] + panel.half_size[1], frame.center[1] - frame.half_size[1])

    def test_symbol_direction(self) -> None:
        """▲ は頂点が上（+z）、▼ は頂点が下。重心は面の中心。"""
        up = self.scene.button("up").symbol_triangle()
        down = self.scene.button("down").symbol_triangle()
        self.assertGreater(up[0, 1], 0.0)
        self.assertLess(down[0, 1], 0.0)
        np.testing.assert_allclose(up.mean(axis=0), 0.0, atol=1e-12)

    def test_press_depth_must_fit_travel(self) -> None:
        cfg = load_config("elevator_hall.yaml")
        cfg["buttons"]["common"]["press_depth"] = cfg["buttons"]["common"]["travel"] * 2
        with self.assertRaises(ValueError):
            build_hall_scene(cfg)


class TestColumnScene(unittest.TestCase):
    """本番に似た乗り場（configs/elevator_prod.yaml、layout: column）。"""

    def setUp(self) -> None:
        self.cfg = load_config("elevator_prod.yaml")
        self.scene = build_hall_scene(self.cfg)

    def test_buttons_in_column_holes(self) -> None:
        """ボタンは柱の面より少し手前に出ていて、柱には、ボタンの位置に穴（ボタンを囲む箱）がある。"""
        c = self.cfg["column"]
        front = self.cfg["wall"]["front_x"] - c["protrusion"]
        names = {b.name for b in self.scene.boxes}
        self.assertIn("hall_panel", names)  # ボタンとの衝突を外す相手
        for k, b in enumerate(sorted(self.scene.buttons, key=lambda b: -b.face_center[2])):
            self.assertAlmostEqual(b.base_center[0], front, places=9)
            self.assertLess(abs(b.face_center[1] - c["center_y"]), c["width"] / 2 - b.radius)
            self.assertIn(f"hall_column_hole{k}_back", names)
            # 穴を囲む箱（階段状の丸い穴）は、どれもボタンの円と重ならず、穴は円から 3 mm 以内（丸に近い）
            pieces = [x for x in self.scene.boxes if x.name.startswith(f"hall_column_hole{k}_s")]
            self.assertGreater(len(pieces), 4)
            gaps = []
            for box in pieces:
                lo, hi = box.center[1:] - box.half_size[1:], box.center[1:] + box.half_size[1:]
                d = np.linalg.norm(np.maximum(0.0, np.maximum(lo - b.face_center[1:], b.face_center[1:] - hi)))
                self.assertGreater(d, b.radius, box.name)  # 円の中に入り込まない
                gaps.append(d - b.radius)
            self.assertLess(min(gaps), 0.003)

    def test_order_and_door(self) -> None:
        z = {b.name: b.face_center[2] for b in self.scene.buttons}
        self.assertTrue(z["up"] > z["down"] > z["wc_up"] > z["wc_down"])
        right = next(b for b in self.scene.boxes if b.name == "hall_door_right")
        self.assertGreater(right.center[1] - right.half_size[1], self.cfg["column"]["width"] / 2)  # 扉は柱の左


class TestCarpet(unittest.TestCase):
    def test_texture_color_and_seam(self) -> None:
        """平均の色は設定の rgb。端どうしの差が内側の隣り合う画素の差と同じくらい（並べても継ぎ目が出ない）。"""
        floor = build_hall_scene(load_config("elevator_hall.yaml")).floor
        img = carpet_texture(floor).astype(float) / 255.0
        self.assertEqual(img.shape, (floor.texture_px, floor.texture_px, 3))
        np.testing.assert_allclose(img.mean(axis=(0, 1)), floor.rgb, atol=0.01)
        inner = np.abs(np.diff(img, axis=1)).mean()
        seam = np.abs(img[:, 0] - img[:, -1]).mean()
        self.assertLess(seam, inner * 1.5)
        np.testing.assert_array_equal(carpet_texture(floor), carpet_texture(floor))  # 同じ seed なら同じ画像


class TestCallButtonState(unittest.TestCase):
    def test_latch_until_reset(self) -> None:
        s = CallButtonState(press_depth=0.0025)
        self.assertFalse(s.update(0.001))
        self.assertFalse(s.lit)
        self.assertTrue(s.update(0.003))
        self.assertTrue(s.lit)
        self.assertFalse(s.update(0.004))  # 押し続けても 1 回だけ
        self.assertFalse(s.update(0.0))
        self.assertTrue(s.lit)  # 離しても点灯したまま
        s.reset()
        self.assertFalse(s.lit)
        self.assertTrue(s.update(0.003))
        self.assertEqual(s.press_count, 2)

    def test_hysteresis(self) -> None:
        """境目のあたりで揺れても、半分の深さより浅くならなければ押し直しにならない。"""
        s = CallButtonState(press_depth=0.0025)
        s.update(0.0026)
        for d in (0.0024, 0.0026, 0.0014, 0.0026):
            self.assertFalse(s.update(d))
        self.assertEqual(s.press_count, 1)


if __name__ == "__main__":
    unittest.main()
