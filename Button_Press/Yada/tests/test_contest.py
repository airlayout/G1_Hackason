"""評価環境（contest/）のテスト。"""

from __future__ import annotations

import importlib.util
import itertools
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.config import FEATURE_DIR, REPO_ROOT, load_config
from common.scene_spec import build_hall_scene
from contest.interface import UPPER_BODY_IDX, Action, load_agent
from contest.runner import SafetyFilter, _check_action
from contest.task import make_trial

MODELS = REPO_ROOT / "_local" / "button_press" / "models" / "g1_description" / "g1_29dof_rev_1_0.xml"
HAS_SIM = all(importlib.util.find_spec(m) is not None for m in ("mujoco", "pinocchio", "scipy")) and MODELS.exists()
SKIP_SIM = "mujoco / pinocchio / scipy / 公式モデルのどれかが無い"
EXAMPLE = FEATURE_DIR / "contest" / "example_agent"
N = len(UPPER_BODY_IDX)


class TestTask(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_config("contest.yaml")

    def test_same_seed_same_trial(self) -> None:
        a, b = make_trial(7, self.cfg), make_trial(7, self.cfg)
        self.assertEqual(a.target, b.target)
        self.assertEqual(a.instruction, b.instruction)
        self.assertEqual(a.scene_cfg, b.scene_cfg)
        self.assertNotEqual(make_trial(8, self.cfg).scene_cfg, a.scene_cfg)

    def test_values_in_range(self) -> None:
        r = self.cfg["randomize"]
        targets = set()
        for seed in range(50):
            t = make_trial(seed, self.cfg)
            targets.add(t.target)
            self.assertIn(t.instruction, self.cfg["instructions"][t.target])
            self.assertEqual(set(t.params), set(r))
            for key, (lo, hi) in r.items():
                self.assertTrue(lo <= t.params[key] <= hi, key)
            sc = build_hall_scene(t.scene_cfg)  # 柱と扉が重なるなど、作れない条件にならない
            z = {b.name: b.face_center[2] for b in sc.buttons}
            self.assertTrue(z["up"] > z["down"] > z["wc_up"] > z["wc_down"])  # 一般用が上、車いす用が下
        self.assertEqual(targets, {"up", "down"})


class TestSafetyAndChecks(unittest.TestCase):
    def test_limit_and_rate(self) -> None:
        lo, hi = -np.ones(29), np.ones(29)
        f = SafetyFilter(lo, hi, margin=0.1, max_step=0.05)
        f.reset(np.zeros(N))
        q, by_limit, by_rate = f(np.full(N, 5.0))
        self.assertTrue(by_limit and by_rate)
        np.testing.assert_allclose(q, 0.05)
        for _ in range(100):
            q, _, _ = f(np.full(N, 5.0))
        np.testing.assert_allclose(q, 0.9)  # 可動範囲 1.0 − 余裕 0.1 で止まる
        q, by_limit, by_rate = f(q)
        self.assertFalse(by_limit or by_rate)

    def test_bad_actions(self) -> None:
        with self.assertRaises(ValueError):
            _check_action(Action(q_target=np.zeros(N - 1)))
        with self.assertRaises(ValueError):
            _check_action(Action(q_target=np.full(N, np.nan)))
        with self.assertRaises(TypeError):
            _check_action(np.zeros(N))

    def test_load_agent_needs_make_agent(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            Path(d, "agent.py").write_text("x = 1\n")
            with self.assertRaises(AttributeError):
                load_agent(d)


@unittest.skipUnless(HAS_SIM, SKIP_SIM)
class TestExampleAgent(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from contest.robots.mujoco_robot import MujocoRobot

        cls.MujocoRobot = MujocoRobot
        cls.cfg = load_config("contest.yaml")
        cls.agent = load_agent(EXAMPLE)

    def test_corners_visible(self) -> None:
        """条件の範囲の端（柱までの距離・一般用 ▲ の高さ・柱の左右の位置の最小と最大の組み合わせ 8 通り）でも、
        一般用の 2 つのボタンを見つけられ、位置の誤差が 1 cm 以内。"""
        from contest.example_agent.agent import find_buttons

        r = self.cfg["randomize"]
        base = make_trial(0, self.cfg).scene_cfg
        for fx, h, cy in itertools.product(r["wall.front_x"], r["button_up_height"], r["column.center_y"]):
            import copy

            sc = copy.deepcopy(base)
            sc["wall"]["front_x"] = fx
            sc["column"]["center_y"] = cy
            items = {b["name"]: b for b in sc["buttons"]["items"]}
            gap = items["up"]["height"] - items["down"]["height"]
            items["up"]["height"], items["down"]["height"] = h, h - gap
            robot = self.MujocoRobot(sc, self.cfg)
            try:
                robot.advance(0.3)
                found = find_buttons(robot.observe(0.0), self.agent.cam_tf)
            finally:
                robot.close()
            self.assertEqual([b.name for b in found], ["up", "down"], f"front_x={fx}, h={h}, y={cy}")
            for b in found:
                err = np.linalg.norm(b.center - robot.scene.button(b.name).face_center)
                self.assertLess(err, 0.01, f"{b.name}: front_x={fx}, h={h}, y={cy}")

    def test_smoke_seeds_succeed(self) -> None:
        """見本のエージェントが、動作の確認用の種（seeds.yaml の smoke）で成功し、壁や柱に強くぶつからない。"""
        from contest.runner import run_episode

        for seed in load_config(FEATURE_DIR / "contest" / "seeds.yaml")["smoke"]:
            trial = make_trial(seed, self.cfg)
            robot = self.MujocoRobot(trial.scene_cfg, self.cfg)
            try:
                res = run_episode(robot, self.agent, trial, self.cfg)
            finally:
                robot.close()
            self.assertEqual(res.outcome, "success", f"seed {seed}: {res}")
            # ⚠️ 見本は、本番に似た乗り場の ▲ で、腕を上げる途中に指先が柱の横の壁をこする（30〜50 N。既知の弱点。
            #    contest/example_agent/agent.py の APPROACH_M のコメント）。強くぶつからないことだけを確かめる
            self.assertLess(res.max_contact_force_n, 80.0)


if __name__ == "__main__":
    unittest.main()
