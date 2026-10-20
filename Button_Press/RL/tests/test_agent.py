"""評価用の Agent（agent/agent.py）の形のテスト（方策と検出を差し替えて、シミュレーター無しで確かめる）。"""

from __future__ import annotations

import unittest

import numpy as np

from button_press_rl.paths import use_yada
from button_press_rl.proprio import ACTION_SCALE, NUM_ACTIONS, POLICY_OBS_DIM

use_yada()
from contest.interface import NUM_JOINTS, RIGHT_ARM_IDX, UPPER_BODY_IDX, Observation, TaskInfo  # noqa: E402


def _obs(t: float, q: np.ndarray) -> Observation:
    return Observation(rgb=np.zeros((4, 4, 3), np.uint8), depth=np.zeros((4, 4), np.float32), K=np.eye(3),
                       q=q, dq=np.zeros(NUM_JOINTS), t=t)


class TestTeacherAgent(unittest.TestCase):
    def setUp(self) -> None:
        import sys
        from pathlib import Path

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "agent"))
        import agent as rl_agent

        self.rl_agent = rl_agent
        self.seen: list[np.ndarray] = []

        def policy(x: np.ndarray) -> np.ndarray:
            self.seen.append(x.copy())
            return np.full(NUM_ACTIONS, 2.0)  # 範囲の外 → ±1 に丸められる

        self.agent = rl_agent.TeacherAgent(policy=policy, locate=lambda obs: np.array([0.3, 0.07, 0.2]))
        self.task = TaskInfo(instruction="下", target="down", sim="fake", time_limit=30.0, control_dt=0.02,
                             base_enabled=False)

    def test_shapes_and_step(self) -> None:
        self.agent.reset(self.task)
        q = np.zeros(NUM_JOINTS)
        a1 = self.agent.act(_obs(0.0, q))
        a2 = self.agent.act(_obs(0.02, q))
        self.assertEqual(a1.q_target.shape, (len(UPPER_BODY_IDX),))
        self.assertEqual(self.seen[0].shape, (POLICY_OBS_DIM,))
        self.assertEqual(self.seen[0].dtype, np.float32)
        # 右腕の目標は 1 周期に ACTION_SCALE ずつ動き、腰と左腕は動かない
        arm = [UPPER_BODY_IDX.index(i) for i in RIGHT_ARM_IDX]
        others = [k for k in range(len(UPPER_BODY_IDX)) if k not in arm]
        np.testing.assert_allclose(a2.q_target[arm] - a1.q_target[arm], ACTION_SCALE)
        np.testing.assert_allclose(a1.q_target[others], 0.0)
        self.assertFalse(a1.done)

    def test_gives_up_when_not_found(self) -> None:
        agent = self.rl_agent.TeacherAgent(policy=lambda x: np.zeros(NUM_ACTIONS), locate=lambda obs: None)
        agent.reset(self.task)
        a = agent.act(_obs(0.0, np.zeros(NUM_JOINTS)))
        self.assertTrue(a.done)


if __name__ == "__main__":
    unittest.main()
