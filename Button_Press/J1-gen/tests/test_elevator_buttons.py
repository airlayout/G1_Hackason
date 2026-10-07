"""エレベーターの呼びボタンを見つける（common/elevator_buttons.py）と、押す計画（common/elevator_press.py）。

見つける部分は、正面の平らな柱に灰色の丸と黒い三角を描いた作り物の画像と深度で確かめる
（カメラは柱にまっすぐ向いている。1 cm ≈ 18.75 画素）。
"""

from __future__ import annotations

import unittest

import numpy as np

from _env import needs
from common.config import load_config
from common.elevator_buttons import (
    Button3D,
    ButtonFinder,
    ButtonNotFound,
    ButtonTracker,
    CallButtons,
    ClassicButtonDetector,
    read_arrow,
    select_call_pair,
)

W, H, F = 640, 480, 600.0
WALL_X = 0.32  # 柱の面までの距離 [m]
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
PX_PER_M = F / WALL_X


class FacingCamera:
    """柱にまっすぐ向いたカメラ（光学座標: x 右・y 下・z 前 → pelvis: x 前・y 左・z 上）。"""

    def pelvis_from_optical(self, q_waist: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]]), np.zeros(3)


def scene(buttons: list[tuple[float, float, str]], diameter: float = 0.035) -> tuple[np.ndarray, np.ndarray]:
    """buttons: (左右 y [m], 上下 z [m], "up" / "down") の並び。黒い柱に灰色の丸と黒い三角を描く。"""
    import cv2

    rgb = np.full((H, W, 3), 15, np.uint8)
    depth = np.full((H, W), WALL_X, np.float32)
    r = int(round(diameter / 2 * PX_PER_M))
    for y, z, arrow in buttons:
        u, v = int(round(W / 2 - y * PX_PER_M)), int(round(H / 2 - z * PX_PER_M))
        cv2.circle(rgb, (u, v), r, (90, 90, 90), -1)
        cv2.circle(depth, (u, v), r, WALL_X - 0.002, -1)  # 2 mm 出っ張っている
        s = int(r * 0.45)
        tri = [(u - s, v + s // 2), (u + s, v + s // 2), (u, v - s)] if arrow == "up" else \
              [(u - s, v - s // 2), (u + s, v - s // 2), (u, v + s)]
        cv2.fillPoly(rgb, [np.array(tri, np.int32)], (20, 20, 20))
    return rgb, depth


class TestFindButtons(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config("elevator.yaml")
        cls.finder = ButtonFinder(FacingCamera(), ClassicButtonDetector(cls.cfg["classic"]), cls.cfg)

    def find(self, buttons: list[tuple[float, float, str]]) -> CallButtons:
        rgb, depth = scene(buttons)
        return self.finder.find(rgb, depth, K, np.zeros(3))

    def test_reads_arrows(self) -> None:
        rgb, _ = scene([(0.0, 0.0, "up"), (0.1, 0.0, "down")])
        gray = rgb.astype(float).mean(axis=2)
        cands = sorted(ClassicButtonDetector(self.cfg["classic"]).detect(rgb), key=lambda c: c.bbox[0])
        self.assertEqual(len(cands), 2)
        # 画像の左が pelvis の +y（▲ を y = 0.1 の左側に置いた）
        self.assertEqual([c.arrow for c in cands], ["down", "up"])
        self.assertEqual(read_arrow(gray, cands[0].bbox), "down")

    def test_finds_call_pair_and_normal(self) -> None:
        call = self.find([(0.0, 0.10, "up"), (0.0, 0.02, "down"), (0.0, -0.12, "up")])
        self.assertAlmostEqual(call.up.center[2], 0.10, delta=0.004)
        self.assertAlmostEqual(call.down.center[2], 0.02, delta=0.004)
        self.assertAlmostEqual(call.up.center[0], WALL_X - 0.002, delta=0.002)
        self.assertAlmostEqual(call.up.diameter_m, 0.035, delta=0.005)
        self.assertGreater(call.normal[0], 0.99)
        self.assertEqual((call.up.arrow, call.down.arrow), ("up", "down"))
        self.assertEqual(len(call.others), 1)

    def test_top_cut_off_is_rejected(self) -> None:
        """▲ が画面の外: ▼ と車いす用の ▲ は 20 cm 離れているので、一般用の組とはみなさない。"""
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.18, "down"), (0.0, -0.02, "up")])

    def test_wrong_arrow_order_is_rejected(self) -> None:
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.10, "down"), (0.0, 0.02, "up")])

    def test_single_button_is_rejected(self) -> None:
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.05, "up")])


class TestSelectAndTrack(unittest.TestCase):
    def b(self, y: float, z: float, arrow: str | None = None) -> Button3D:
        return Button3D(np.array([0.3, y, z]), 0.035, arrow, (0, 0, 1, 1), 100)

    def test_two_columns_are_ambiguous(self) -> None:
        g = load_config("elevator.yaml")["geometry"]
        with self.assertRaises(ButtonNotFound):
            select_call_pair([self.b(0.0, 0.3), self.b(0.0, 0.22), self.b(0.3, 0.3), self.b(0.3, 0.22)], g)

    def test_unknown_arrows_use_order(self) -> None:
        g = load_config("elevator.yaml")["geometry"]
        up, down = select_call_pair([self.b(0.0, 0.22), self.b(0.0, 0.3)], g)
        self.assertEqual((up.center[2], down.center[2]), (0.3, 0.22))

    def test_tracker(self) -> None:
        cfg = {"frames": 3, "max_spread_m": 0.005, "max_normal_spread_deg": 5.0}
        tr = ButtonTracker(cfg)

        def call(dz: float) -> CallButtons:
            return CallButtons(self.b(0, 0.3 + dz, "up"), self.b(0, 0.22 + dz, "down"), np.array([1.0, 0, 0]))

        self.assertIsNone(tr.add(call(0.0)))
        self.assertIsNone(tr.add(call(0.02)))  # 2 cm 動いた
        self.assertIsNone(tr.add(call(0.0)))  # ばらつきが大きいので、まだ返さない
        self.assertIsNone(tr.add(call(0.001)))  # 2 cm 動いたフレームがまだ 3 つの中にある
        merged = tr.add(call(0.0))  # 続けて 3 つそろった
        self.assertIsNotNone(merged)
        self.assertAlmostEqual(merged.up.center[2], 0.30, delta=1e-6)


@needs("mujoco", "pin")
class TestElevatorPlan(unittest.TestCase):
    """評価環境の seed 1（▼、柱まで 0.31 m）の値で、計画が立つこと。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"),
            load_config("elevator.yaml"))
        # 評価環境の始めの姿勢（両腕を下ろしている）
        cls.q0 = np.zeros(29)
        cls.q0[[15, 22]] = 0.2
        cls.q0[16], cls.q0[23] = 0.25, -0.25
        cls.q0[[18, 25]] = 1.0

    def call(self, x: float = 0.309) -> CallButtons:
        return CallButtons(Button3D(np.array([x, 0.027, 0.267]), 0.033, "up", (0, 0, 1, 1), 100, "up"),
                           Button3D(np.array([x, 0.027, 0.187]), 0.033, "down", (0, 0, 1, 1), 100, "down"),
                           np.array([1.0, 0.0, 0.0]))

    def test_plans_both_buttons(self) -> None:
        from common.elevator_press import wall_obstacle

        for target in ("up", "down"):
            self.ctl.reset(target, 0.02, np.full(17, 60.0))
            self.ctl._plan(self.q0, self.call())
            p = self.ctl.plan_
            face = self.call().button(target).center
            # 球の表面がボタンに触れる点、押す向き、押し込み終わり
            np.testing.assert_allclose(p.target_point, face - [0.008, 0, 0], atol=1e-9)
            self.assertAlmostEqual(p.end_point[0] - p.target_point[0], 0.015, places=6)
            # 指は左・上に傾いている（押す向きとは別）
            self.assertLess(p.finger_axis @ p.push_dir, np.cos(np.radians(30)))
            # 手前の姿勢は壁の箱の手前（箱の前面はボタンの面の位置）
            wall = wall_obstacle(self.call(), self.ctl.cfg["wall"])
            front = wall["center"][0] - wall["half_size"][0]
            self.assertAlmostEqual(front, face[0] + self.ctl.cfg["wall"]["margin_m"], places=9)
            self.assertLess(p.approach_point[0], front)

    def test_tilted_wall_box_follows_the_face(self) -> None:
        """面が斜め（3°）でも、箱の前面はボタンの面に沿う（手前に出っ張らない）。"""
        from common.collision import CollisionChecker
        from common.elevator_press import wall_obstacle

        n = np.array([np.cos(np.radians(3)), np.sin(np.radians(3)), 0.0])
        call = self.call()
        call.normal = n
        ob = wall_obstacle(call, self.ctl.cfg["wall"])
        np.testing.assert_allclose(ob["x_axis"], n)
        chk = CollisionChecker(self.ctl.robot_cfg, "right", 0.0, [ob])
        m, d = chk.model, chk.data
        import mujoco

        mujoco.mj_forward(m, d)
        gid = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("obstacle_0")][0]
        x_axis_world = d.geom_xmat[gid].reshape(3, 3)[:, 0]
        np.testing.assert_allclose(x_axis_world, n, atol=1e-9)

    def test_unreachable_far_wall(self) -> None:
        from common.press_planner import UnreachableError

        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        with self.assertRaises(UnreachableError):
            self.ctl._plan(self.q0, self.call(x=0.9))

    def test_fresh_images_without_capture_time(self) -> None:
        """撮った時刻が 0 のまま（評価環境の実機用の口）でも、一定の間隔ごとに新しい画像とみなす。"""
        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        interval = self.ctl.cfg["press"]["image_interval_s"]
        fresh = [self.ctl._is_fresh(k * 0.02, 0.0) for k in range(int(round(1.0 / 0.02)))]
        self.assertEqual(sum(fresh), int(round(1.0 / interval)))
        # 撮った時刻があれば、それが進んだときだけ新しい
        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        self.assertEqual([self.ctl._is_fresh(t, it) for t, it in [(0.1, 0.05), (0.12, 0.05), (0.14, 0.09)]],
                         [True, False, True])

    def test_lit_check_retries_once(self) -> None:
        self.ctl.reset("down", 0.02, np.full(17, 60.0))
        self.ctl.call = self.call()
        self.ctl._plan(self.q0, self.call())
        dark = np.zeros((480, 640, 3), np.uint8)
        self.ctl.baseline_brightness = 0.0
        depth0 = self.ctl.plan_.depth
        self.assertFalse(self.ctl._check(dark, 1.0))  # 点灯していない → 深く押し直す
        self.assertEqual(self.ctl.phase, "press")
        self.assertAlmostEqual(self.ctl.plan_.depth, depth0 + 0.01)
        self.assertTrue(self.ctl._check(dark, 2.0))  # 押し直しは 1 回まで
        self.assertIn("確かめられない", self.ctl.outcome)


if __name__ == "__main__":
    unittest.main()
