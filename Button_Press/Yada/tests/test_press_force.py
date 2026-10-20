"""押す力の記録（common/press_force.py）と、点灯のあとも測り続けるランナー（contest/runner.py）のテスト。"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from common.config import load_config
from common.press_force import PressForceMeter, PressForceSpec, summarize_press
from contest.interface import NUM_JOINTS, UPPER_BODY_IDX, Action, Agent, Observation
from contest.robots.base import Robot, StepInfo
from contest.runner import run_episode
from contest.task import first_seeds_for_target, make_trial, seeds_for_target

SPEC = PressForceSpec(ok_lo=3.0, ok_hi=8.0, max_n=20.0, post_press_s=0.5, release_n=0.3)


class TestMeter(unittest.TestCase):
    def test_in_range_and_released(self) -> None:
        m = PressForceMeter(SPEC)
        for f in (0.0, 2.0, 5.0):
            m.update(f)
        m.mark_lit()
        m.update(4.0)
        m.mark_end(0.1)
        s = m.summary()
        self.assertEqual(s["max_press_force_n"], 5.0)
        self.assertTrue(s["press_force_in_range"])
        self.assertFalse(s["press_force_over_max"])
        self.assertTrue(s["released_after_press"])

    def test_too_strong_and_kept_pressing(self) -> None:
        m = PressForceMeter(SPEC)
        m.update(25.0)
        m.mark_lit()
        m.mark_end(10.0)
        s = m.summary()
        self.assertFalse(s["press_force_in_range"])
        self.assertTrue(s["press_force_over_max"])
        self.assertFalse(s["released_after_press"])

    def test_not_lit_and_not_measured(self) -> None:
        m = PressForceMeter(SPEC)
        m.update(1.0)
        s = m.summary()
        self.assertIsNone(s["press_force_in_range"])
        self.assertIsNone(s["released_after_press"])
        self.assertIsNone(PressForceMeter(SPEC).summary()["max_press_force_n"])  # 測れないロボット

    def test_spec_from_config(self) -> None:
        spec = PressForceSpec.from_cfg(load_config("contest.yaml")["press_force"])
        self.assertLess(spec.ok_lo, spec.ok_hi)
        self.assertLess(spec.ok_hi, spec.max_n)
        self.assertGreater(spec.post_press_s, 0.0)

    def test_summarize(self) -> None:
        rows = [
            {"outcome": "success", "max_press_force_n": 5.0, "press_force_in_range": True,
             "press_force_over_max": False, "released_after_press": True},
            {"outcome": "success", "max_press_force_n": 12.0, "press_force_in_range": False,
             "press_force_over_max": False, "released_after_press": False},
            {"outcome": "timeout", "max_press_force_n": 30.0, "press_force_over_max": True},
            {"outcome": "timeout", "max_press_force_n": None},
        ]
        s = summarize_press(rows)
        self.assertEqual(s["measured"], 3)
        self.assertEqual(s["in_range_rate"], 0.5)
        self.assertEqual(s["released_rate"], 0.5)
        self.assertEqual(s["over_max_trials"], 1)
        self.assertEqual(s["max_press_force_n_success_mean"], 8.5)
        self.assertEqual(summarize_press([{"outcome": "success", "max_press_force_n": None}]), {"measured": 0})


class TestTargetSeeds(unittest.TestCase):
    def test_pick_by_target(self) -> None:
        cfg = load_config("contest.yaml")
        down = first_seeds_for_target(5, "down", cfg)
        self.assertEqual(len(down), 5)
        self.assertTrue(all(make_trial(s, cfg).target == "down" for s in down))
        self.assertEqual(seeds_for_target(list(range(down[-1] + 1)), "down", cfg), down)
        with self.assertRaises(ValueError):
            seeds_for_target([0], "left", cfg)


class _FakeRobot(Robot):
    """advance() の lit_at 回目（0 から）に目標のボタンが点灯し、押す力は forces[回] になる模擬のロボット。

    ランナーは試行の前に 1 回 advance() する（置いた直後の揺れを収める）ので、試行の周期は 1 回ずれる。
    """

    name = "fake"
    base_enabled = False

    def __init__(self, target: str, lit_at: int, forces: list[float]):
        self.target, self.lit_at, self.forces = target, lit_at, forces
        self.k = 0

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return -np.full(NUM_JOINTS, 3.0), np.full(NUM_JOINTS, 3.0)

    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        n = len(UPPER_BODY_IDX)
        return np.ones(n), np.ones(n)

    def observe(self, t: float) -> Observation:
        return Observation(rgb=np.zeros((2, 2, 3), np.uint8), depth=np.zeros((2, 2), np.float32), K=np.eye(3),
                           q=np.zeros(NUM_JOINTS), dq=np.zeros(NUM_JOINTS), t=t)

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        pass

    def advance(self, dt: float) -> StepInfo:
        f = self.forces[min(self.k, len(self.forces) - 1)]
        lit = self.k >= self.lit_at
        self.k += 1
        lights = {"up": False, "down": False}
        lights[self.target] = lit
        return StepInfo(lit=lights, press_force={self.target: f},
                        press_force_end={self.target: f})


class _HoldAgent(Agent):
    def __init__(self, done_after: int | None = None):
        self.done_after, self.n = done_after, 0

    def reset(self, task) -> None:
        self.n = 0

    def act(self, obs: Observation) -> Action:
        self.n += 1
        return Action(q_target=np.zeros(len(UPPER_BODY_IDX)), done=self.done_after is not None and self.n > self.done_after)


class TestRunnerPostPress(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_config("contest.yaml")
        self.trial = SimpleNamespace(seed=0, target="down", instruction="下", params={}, realism=None)
        self.dt = 1.0 / float(self.cfg["control_hz"])
        self.post_steps = int(round(float(self.cfg["press_force"]["post_press_s"]) / self.dt))

    def test_keeps_measuring_after_lit(self) -> None:
        # 点灯（10 周期目）のあとに力が強くなり（12 N）、最後は離す
        forces = [0.0] * 8 + [3.0, 5.0, 5.0] + [12.0] * 10 + [0.0] * 100
        res = run_episode(_FakeRobot("down", 10, forces), _HoldAgent(), self.trial, self.cfg)
        self.assertEqual(res.outcome, "success")
        self.assertAlmostEqual(res.time_s, 10 * self.dt, places=6)  # 点灯までの時間は変わらない
        self.assertEqual(res.steps, 10 + self.post_steps)
        self.assertEqual(res.max_press_force_n, 12.0)
        self.assertFalse(res.press_force_in_range)
        self.assertTrue(res.released_after_press)

    def test_done_during_post_press_keeps_success(self) -> None:
        forces = [5.0] * 200
        res = run_episode(_FakeRobot("down", 3, forces), _HoldAgent(done_after=6), self.trial, self.cfg)
        self.assertEqual(res.outcome, "success")
        self.assertTrue(res.press_force_in_range)
        self.assertFalse(res.released_after_press)
        self.assertLess(res.steps, 4 + self.post_steps)

    def test_wrong_button_stops_at_once(self) -> None:
        forces = [5.0] * 200
        res = run_episode(_FakeRobot("up", 3, forces), _HoldAgent(), self.trial, self.cfg)
        self.assertEqual(res.outcome, "wrong")
        self.assertEqual(res.steps, 3)
        self.assertIsNone(res.press_force_in_range)


if __name__ == "__main__":
    unittest.main()
