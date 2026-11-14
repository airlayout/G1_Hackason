"""強化学習でボトルを押す環境（rl/press_env.py）のテスト。"""

from __future__ import annotations

import unittest

import numpy as np
from _env import needs


@needs("mujoco", "gym")
class TestPressEnv(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from rl.press_env import OBS_DIM, PressEnv

        cls.env = PressEnv()
        cls.obs_dim = OBS_DIM

    def test_layout_in_range(self) -> None:
        """引いた配置が設定の範囲に入り、机の縁が下限より遠い。"""
        sc = self.env.cfg["scene"]
        for seed in range(20):
            _, info = self.env.reset(seed=seed)
            lay = info["layout"]
            for k in ("target_x", "bottle_y", "gap", "top_z"):
                lo, hi = sc[k] if k != "target_x" else sc["target_x"]
                self.assertTrue(lo <= lay[k] <= hi, (k, lay[k]))
            self.assertGreaterEqual(lay["target_x"] - lay["gap"], sc["table_edge_min_m"])

    def test_layout_inside_reach_sphere(self) -> None:
        """引いた配置の押す点は、右肩から reach_radius_m の球の中に入る。肩は開始の姿勢の位置（腕の付け根）。"""
        sc = self.env.cfg["scene"]
        np.testing.assert_allclose(self.env.shoulder, [-0.001, -0.100, 0.292], atol=0.005)
        for seed in range(100):
            self.env.reset(seed=seed)
            self.assertLessEqual(float(np.linalg.norm(self.env.target - self.env.shoulder)), sc["reach_radius_m"] + 1e-9)

    def test_obs_shape_and_target(self) -> None:
        layout = {"target_x": 0.35, "bottle_y": -0.15, "gap": 0.05, "top_z": 0.0}
        obs, _ = self.env.reset(seed=0, options={"layout": layout})
        self.assertEqual(obs.shape, (self.obs_dim,))
        self.assertTrue(np.all(np.isfinite(obs)))
        np.testing.assert_allclose(self.env.target, [0.35, -0.15, 0.10])
        self.assertAlmostEqual(self.env.table_edge_x, 0.30)

    def test_start_pose_contact_is_allowed(self) -> None:
        """下ろした手は股関節の当たり判定に入っているが、行動 0 で 1 秒置いても「当たった」にならない。"""
        self.env.reset(seed=0)
        self.assertTrue(self.env._start_dist, "開始の姿勢で手と股関節が接していない（前提が変わった）")
        for _ in range(50):
            _, _, terminated, truncated, info = self.env.step(np.zeros(7))
            self.assertFalse(terminated, info)
        self.assertEqual(info["hit"], "")

    def test_action_moves_target_by_max_step(self) -> None:
        self.env.reset(seed=0)
        before = self.env._q_des[self.env.arm].copy()
        self.env.step(np.full(7, 2.0))   # −1〜1 に切られる
        moved = self.env._q_des[self.env.arm] - before
        step = self.env.cfg["action"]["max_step_rad"]
        self.assertTrue(np.all(moved <= step + 1e-12))
        self.assertTrue(np.any(np.isclose(moved, step)))

    def test_timeout(self) -> None:
        self.env.reset(seed=0)
        for _ in range(self.env.max_steps):
            _, _, terminated, truncated, info = self.env.step(np.zeros(7))
            if terminated or truncated:
                break
        self.assertTrue(truncated)
        self.assertEqual(info["outcome"], "timeout")

    def test_search_penalty(self) -> None:
        """押す点の近くにいる間だけ、毎周期 search_penalty が引かれる（遠いときは引かれない）。"""
        env = self.env
        cfg_r = env.cfg["reward"]
        base = dict(cfg_r)
        try:
            for pen in (0.0, 1.0):
                cfg_r.update(search_near_m=0.05, search_penalty=pen, touch_lateral_penalty=0.0)
                env.reset(seed=1)
                _, r_far, *_ = env.step(np.zeros(7))
                env._prev_dist = float(np.linalg.norm(env.target - env.tip_pos()))
                env.target = env.tip_pos() + np.array([0.02, 0.0, 0.0])  # 指先の 2 cm 前に押す点を置く
                env._prev_dist = 0.02
                _, r_near, *_ = env.step(np.zeros(7))
                if pen == 0.0:
                    far0, near0 = r_far, r_near
                else:
                    self.assertAlmostEqual(r_far, far0, places=6)
                    self.assertAlmostEqual(r_near, near0 - 1.0, places=6)
        finally:
            cfg_r.clear()
            cfg_r.update(base)


@needs("mujoco", "gym", "pin")
class TestPressEnvScripted(unittest.TestCase):
    def test_ik_motion_succeeds(self) -> None:
        """IK で作った動き（経由 → 手前 → 押す）で、右 18 cm のボトルを押すと success になり、押す力が範囲に入る。"""
        from common.config import load_config
        from common.kinematics import ArmKinematics
        from rl.check_env import ik_goals
        from rl.press_env import PressEnv

        env = PressEnv()
        env.reset(seed=0, options={"layout": {"target_x": 0.364, "bottle_y": -0.183, "gap": 0.024, "top_z": -0.048}})
        kin = ArmKinematics(load_config("robot.yaml"), "right", load_config("press.yaml")["ik"])
        goals = ik_goals(env, kin, np.radians(load_config("pipeline.yaml")["via_pose"]))
        self.assertIsNotNone(goals)
        step = env.cfg["action"]["max_step_rad"]
        info: dict = {}
        # 最後の目標に着いたあとは、押す力が hold_s 続くまで行動 0 で待つ（None）
        for g in goals + [None]:
            for _ in range(env.max_steps):
                a = np.zeros(7) if g is None else np.clip((g - env._q_des[env.arm]) / step, -1, 1)
                _, _, terminated, truncated, info = env.step(a)
                if terminated or truncated or (g is not None and np.abs(g - env._q_des[env.arm]).max() < 1e-9):
                    break
            if terminated or truncated:
                break
        self.assertEqual(info["outcome"], "success", info)
        self.assertLessEqual(info["lateral"], env.cfg["success"]["lateral_tol_m"])


if __name__ == "__main__":
    unittest.main()
