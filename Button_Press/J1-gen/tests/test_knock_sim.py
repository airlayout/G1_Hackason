"""ボトルを倒す動作（sim/knock_bottle_sim.py）を MuJoCo で確かめる。

倒れる自由物体のボトル（configs/sim_scene_knock.yaml）を、configs/knock.yaml の設定で押す。
見積もりの誤りを繰り返さないために、浅い押し込みでは倒れないこと（水 50 mL では重心が低く、倒れきる角度が約 45°）も確かめる。
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import unittest

import numpy as np

from _env import needs
from common.config import FEATURE_DIR, deep_merge, load_config

sys.path.insert(0, str(FEATURE_DIR / "sim"))

# 記録は本番のフォルダ（_local/button_press/runs）ではなく、一時フォルダに書く
LOG_DIR = tempfile.TemporaryDirectory()


def args(**kw: object) -> argparse.Namespace:
    base = dict(label="knock", arm=None, target=None, point=None, taught_pose=None, offset=None, seed_pose="",
                depth_mm=None, detector=None, gravity_scale=None, sim_detector="segmentation", confirm=False,
                log_dir=LOG_DIR.name, overlay="knock.yaml", view=False)
    base.update(kw)
    return argparse.Namespace(**base)


def scene_model(**bottle: float):  # type: ignore[no-untyped-def]
    from common.robot_model import build_spec
    from common.sim_scene import add_scene

    cfg = deep_merge(load_config("sim_scene_knock.yaml"), {"bottle": bottle})
    spec = build_spec(load_config("robot.yaml"), fixed_base=True)
    add_scene(spec, cfg)
    return spec.compile()


@needs('mujoco')
class TestKnockScene(unittest.TestCase):
    def test_free_bottle_mass_and_center_of_mass(self) -> None:
        """殻 20 g + 水 50 mL = 70 g。水は底にたまるので、重心は底から 3 cm 前後まで下がる。"""
        from common.sim_scene import BOTTLE

        m = scene_model()
        b = m.body(BOTTLE).id
        self.assertAlmostEqual(float(m.body_mass[b]), 0.07, places=6)
        self.assertTrue(0.02 < float(m.body_ipos[b][2]) < 0.04, m.body_ipos[b])
        # 水が無ければ重心はずっと高い（胴の中ほど）
        self.assertGreater(float(scene_model(water_ml=0.0).body_ipos[m.body(BOTTLE).id][2]), 0.07)

    def test_fixed_bottle_scene_is_unchanged(self) -> None:
        """従来のシーン（sim_scene.yaml）のボトルは固定のまま（自由関節が付かない）。"""
        from common.robot_model import build_spec
        from common.sim_scene import BOTTLE, add_scene

        spec = build_spec(load_config("robot.yaml"), fixed_base=True)
        add_scene(spec, load_config("sim_scene.yaml"))
        m = spec.compile()
        self.assertEqual(int(m.body_jntnum[m.body(BOTTLE).id]), 0)

    def test_bottle_stays_upright_without_a_push(self) -> None:
        import mujoco

        from common.sim_scene import bottle_tilt_deg

        m = scene_model()
        d = mujoco.MjData(m)
        for _ in range(1000):
            mujoco.mj_step(m, d)
        self.assertLess(bottle_tilt_deg(m, d), 1.0)


class TestTipCheck(unittest.TestCase):
    """倒れたかの自動判定（common/tip_check.py）。ボトルのいちばん高い点の高さを、押す前と押したあとで比べる。"""

    @staticmethod
    def frame(cap_z: float | None):  # type: ignore[no-untyped-def]
        """640x480、深度 0.8 m の背景の中に、枠 (300, 100)-(340, 300)。上端の帯の中央の列に、深度 cap_z の細い物（キャップ）。"""
        from common.rgbd_protocol import Intrinsics, RgbdFrame

        depth = np.full((480, 640), 800, dtype=np.uint16)
        if cap_z is not None:
            depth[100:112, 316:324] = int(cap_z * 1000)
        intr = Intrinsics(width=640, height=480, fx=615.0, fy=615.0, cx=319.5, cy=239.5)
        return RgbdFrame(color_bgr=np.zeros((480, 640, 3), np.uint8), depth=depth, depth_scale=0.001, intrinsics=intr,
                         timestamp=0.0, frame_id=1, camera="head")

    def setUp(self) -> None:
        from common.camera_geometry import HeadCameraTransform
        from common.tip_check import top_height  # noqa: F401

        self.robot, self.loc = load_config("robot.yaml"), load_config("localize.yaml")
        self.cfg = load_config("knock.yaml")["pipeline"]["tip_check"]
        self.tf = HeadCameraTransform(self.robot, self.loc["calibration"]["offset_pelvis_m"])
        self.qw = np.zeros(3)
        self.bbox = (300.0, 100.0, 340.0, 300.0)

    def test_uses_the_nearest_pixels_not_the_background(self) -> None:
        """帯の大半が背景（0.8 m）でも、手前のキャップの画素の高さをとる。"""
        from common.tip_check import top_height

        near, why = top_height(self.frame(0.5), self.bbox, self.tf, self.qw, self.cfg)
        self.assertIsNotNone(near, why)
        # キャップのまわりは背景の奥行きなので、背景を使っていれば近い物との高さの差が出る（奥行き 0.3 m 分、カメラは下向き）
        far, _ = top_height(self.frame(None), self.bbox, self.tf, self.qw, self.cfg)
        self.assertIsNotNone(far)
        self.assertGreater(abs(near - far), 0.05)

    def test_no_depth_is_unknown_not_a_guess(self) -> None:
        from common.tip_check import top_height

        f = self.frame(None)
        f.depth[:] = 0
        top, why = top_height(f, self.bbox, self.tf, self.qw, self.cfg)
        self.assertIsNone(top)
        self.assertIn("有効な深度", why)

    def test_states_from_the_drop_in_height(self) -> None:
        import common.tip_check as tc

        class Det:
            def detect(self, _img):  # type: ignore[no-untyped-def]
                return [type("D", (), {"bbox": (300.0, 100.0, 340.0, 300.0), "confidence": 0.9})()]

        chk = tc.TipOverCheck(self.robot, self.loc, Det(), self.cfg)
        f = self.frame(0.5)
        orig = tc.top_height
        try:
            for after, expect in ((0.148, tc.STANDING), (0.145 - 0.045, tc.LEANING), (0.015, tc.FALLEN)):
                seq = iter([0.150, after])
                tc.top_height = lambda *a, **k: (next(seq), "")  # type: ignore[assignment]
                chk.observe_before([(f, self.qw)])
                self.assertEqual(chk.evaluate([(f, self.qw)]).state, expect)
        finally:
            tc.top_height = orig  # type: ignore[assignment]

    def test_unknown_without_a_baseline_or_when_the_bottle_is_gone(self) -> None:
        import common.tip_check as tc

        class NoDet:
            def detect(self, _img):  # type: ignore[no-untyped-def]
                return []

        chk = tc.TipOverCheck(self.robot, self.loc, NoDet(), self.cfg)
        self.assertEqual(chk.evaluate([(self.frame(0.5), self.qw)]).state, tc.UNKNOWN)  # 押す前の高さが無い
        chk.top_before = 0.15
        res = chk.evaluate([(self.frame(0.5), self.qw)])
        self.assertEqual(res.state, tc.UNKNOWN)  # 押したあとに見つからない（机から落ちた、など）
        self.assertEqual(res.exit_code, tc.EXIT_TIP_UNKNOWN)
        self.assertEqual(tc.TipResult(tc.FALLEN, "").exit_code, 0)
        self.assertEqual(tc.TipResult(tc.STANDING, "").exit_code, tc.EXIT_NOT_TIPPED)


@needs('render', 'mujoco', 'pin')
class TestKnockSim(unittest.TestCase):
    def test_default_settings_knock_the_bottle_over(self) -> None:
        from knock_bottle_sim import run_knock_sim

        r, backend = run_knock_sim(args())
        backend.close()
        self.assertEqual(r.pipeline.code, 0, r.pipeline.message)
        self.assertTrue(r.tipped, f"最後の傾き {r.final_tilt_deg:.1f}°")
        self.assertGreater(r.max_tilt_deg, 60.0)
        # 深度での自動判定も「倒れた」。いちばん高い点が、立っているボトルの高さ（約 20 cm）の分だけ下がる
        self.assertEqual(r.auto_state, "fallen")
        self.assertGreater(r.auto_drop_m, 0.10)

    def test_shallow_push_does_not_knock_it_over(self) -> None:
        """押し込み 8 cm では、水 50 mL のボトルは約 44° まで傾いて戻る（倒れきる角度は約 45°）。"""
        from knock_bottle_sim import run_knock_sim

        r, backend = run_knock_sim(args(label="knock_shallow", depth_mm=80.0, target_offset=[0.0, 0.0, 0.03]))
        backend.close()
        self.assertEqual(r.pipeline.code, 0, r.pipeline.message)
        self.assertFalse(r.tipped)
        self.assertEqual(r.auto_state, "standing")  # 傾いて戻ったので、高さは変わらない
        self.assertGreater(r.max_tilt_deg, 20.0)  # 触れて傾けてはいる
        self.assertLess(r.moved_m, 0.05)          # 滑ってもいない

    def test_low_push_only_slides_it(self) -> None:
        """押す点が低い（測った点のまま）と、滑るだけで倒れない。"""
        from knock_bottle_sim import run_knock_sim

        r, backend = run_knock_sim(args(label="knock_low", target_offset=[0.0, 0.0, 0.0]))
        backend.close()
        self.assertFalse(r.tipped)
        self.assertLess(r.max_tilt_deg, 20.0)

    def test_bottle_is_still_on_the_table_edge_side(self) -> None:
        """机の手前の縁に置いたボトルを前へ倒すので、机から落ちない。"""
        from knock_bottle_sim import run_knock_sim

        r, backend = run_knock_sim(args(label="knock_edge"))
        backend.close()
        self.assertFalse(r.fell_off_table)
        self.assertTrue(np.isfinite(r.moved_m))


if __name__ == "__main__":
    unittest.main()
