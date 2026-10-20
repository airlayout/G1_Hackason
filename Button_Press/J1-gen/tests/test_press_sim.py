"""MuJoCo で押し込みを実行し、手先が目標に届くかを確かめる（タスク2の完了条件: 誤差 1cm 以内）。"""

from __future__ import annotations

import sys
import unittest

import numpy as np

from _env import needs
from common.arm import UnsafeTargetError
from common.config import FEATURE_DIR, load_config

sys.path.insert(0, str(FEATURE_DIR / "sim"))
from press_sim import run_press_sim  # noqa: E402

from test_kinematics import REACHABLE  # noqa: E402


@needs('mujoco', 'pin')
class TestPressSim(unittest.TestCase):
    def test_reaches_targets_within_1cm(self) -> None:
        for tgt, d in REACHABLE:
            errs = run_press_sim(np.array(tgt), np.array(d))
            self.assertLess(errs["approach"], 0.01, (tgt, errs))
            self.assertLess(errs["end"], 0.01, (tgt, errs))

    def _plan_b(self, scale: float, waist_scale: float, replan: bool) -> dict[str, float]:
        """プランB の条件（MuJoCo 側の重力補償なし、腰の保持は既定の Kp=300 / Kd=3）で押す。"""
        arm = load_config("arm.yaml")
        sim = dict(arm["sim"], emulate="lowcmd", gravity_compensation=False)
        gc = {"scale": scale, "tau_max_nm": 7.0, "waist_scale": waist_scale, "waist_tau_max_nm": 15.0}
        return run_press_sim(np.array(REACHABLE[0][0]), np.array(REACHABLE[0][1]),
                             arm_overrides={"sim": sim, "gravity_compensation": gc}, replan=replan)

    def test_plan_b_gravity_compensation_and_replan(self) -> None:
        """プランB の条件で、腕の重力補償 → 実測の腰で計算し直し → 腰の重力補償 の順に誤差が減る。"""
        none = self._plan_b(0.0, 0.0, replan=False)
        arm_only = self._plan_b(1.0, 0.0, replan=False)
        replanned = self._plan_b(1.0, 0.0, replan=True)
        waist = self._plan_b(1.0, 1.0, replan=True)
        print("\n[test] プランB 条件の押し込み終わりの誤差: "
              f"補償なし {none['end'] * 1000:.1f} mm、腕の補償 {arm_only['end'] * 1000:.1f} mm、"
              f"+計算し直し {replanned['end'] * 1000:.1f} mm、+腰の補償 {waist['end'] * 1000:.1f} mm")
        self.assertGreater(none["end"], arm_only["end"])
        self.assertLess(arm_only["end"], 0.01)
        self.assertLess(replanned["end"], 0.003)
        self.assertLess(waist["end"], 0.001)

    def test_unreachable_is_rejected_before_moving(self) -> None:
        with self.assertRaises(UnsafeTargetError):
            run_press_sim(np.array([0.30, 0.10, 0.0]), np.array([1.0, 0, 0]))


if __name__ == "__main__":
    unittest.main()
