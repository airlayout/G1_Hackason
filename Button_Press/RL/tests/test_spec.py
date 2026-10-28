"""学習の条件（spec.py）が、評価環境の条件の範囲を覆っているかのテスト。"""

from __future__ import annotations

import math
import unittest

import numpy as np

from button_press_rl.paths import use_yada, yada_config
from button_press_rl.proprio import ACTION_SCALE, RIGHT_ARM_JOINTS
from button_press_rl.spec import load_task_spec, offset_ranges, target_face_nominal

use_yada()
from common.scene_spec import build_hall_scene  # noqa: E402
from contest.interface import JOINT_NAMES, RIGHT_ARM_IDX  # noqa: E402
from contest.task import make_trial  # noqa: E402


class TestSpec(unittest.TestCase):
    def setUp(self) -> None:
        self.contest = yada_config("contest.yaml")
        self.scene = yada_config(self.contest["scene_file"])

    def test_offsets_cover_trials(self) -> None:
        """評価の試行（種 0〜199 の ▼）の、ボタンの位置のずれが、ロボットの立つ位置のずらし幅に入る。"""
        o = offset_ranges(self.contest, self.scene, "down")
        f0 = np.array(target_face_nominal(self.scene, "down"))
        n = 0
        for seed in range(200):
            t = make_trial(seed, self.contest, self.scene)
            if t.target != "down":
                continue
            n += 1
            d = build_hall_scene(t.scene_cfg).button("down").face_center - f0
            # ボタンが +d ずれる = ロボットが −d ずれる。立ち位置のずれ（stance）の分は外す
            self.assertTrue(o.x[0] - 1e-9 <= -d[0] <= o.x[1] + 1e-9, (seed, d))
            self.assertTrue(o.y[0] - 1e-9 <= -d[1] <= o.y[1] + 1e-9, (seed, d))
            self.assertTrue(o.z[0] - 1e-9 <= -d[2] <= o.z[1] + 1e-9, (seed, d))
        self.assertGreater(n, 50)
        self.assertAlmostEqual(o.yaw[1], math.radians(self.contest["realism"]["stance"]["yaw_deg"]))

    def test_action_scale_matches_runner(self) -> None:
        """行動の大きさ = ランナーの 1 周期の最大移動量（方策の目標が評価で丸められない）。"""
        s = self.contest["safety"]["max_joint_speed_rad_s"] / self.contest["control_hz"]
        self.assertAlmostEqual(ACTION_SCALE, s)

    def test_joint_names(self) -> None:
        self.assertEqual(RIGHT_ARM_JOINTS, tuple(JOINT_NAMES[i] for i in RIGHT_ARM_IDX))

    def test_task_spec(self) -> None:
        s = load_task_spec()
        self.assertEqual(s.target, "down")
        self.assertLess(s.press_depth, s.travel)
        self.assertAlmostEqual(s.initial_pose["right_elbow_joint"], math.radians(86.0))
        self.assertEqual(s.fingertip_link, "right_wrist_yaw_link")


if __name__ == "__main__":
    unittest.main()
