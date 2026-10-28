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


@needs('mujoco', 'pin')
class TestPressRetreat(unittest.TestCase):
    """押し込みの途中で中止したときに戻る経路（common/press.py の PressRetreat）。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.press_planner import PressPlanner

        cfg = load_config("arm.yaml")
        cls.planner = PressPlanner.from_config(load_config("robot.yaml"), load_config("press.yaml"), cfg, "right")
        tgt, d = REACHABLE[0]
        cls.q0 = np.zeros(29)
        cls.plan = cls.planner.plan(cls.q0, np.array(tgt), np.array(d), q_via_arm=np.radians([28.6, -11.5, 0, -34.4, 0, 0, 0]))
        cls.start = np.zeros(7)

    def retreat(self, stage: str):  # type: ignore[no-untyped-def]
        from common.press import PressRetreat

        r = PressRetreat(self.plan, self.planner, self.start)
        r.stage = stage
        return r

    def test_pressing_backs_off_along_the_line_then_via_then_start(self) -> None:
        p, arm = self.plan, self.planner.kin.arm_idx
        j = len(p.press_in) - 1  # 指令は押し込み終わり
        q_meas = self.q0.copy()
        q_meas[arm] = p.press_in[j - 20]  # 実際の指先は 20 点手前で止まっている（押し当てている）
        steps = self.retreat("press")(q_meas, p.press_in[j])
        self.assertTrue(steps[0].hold)
        # 指令の点から、実際の指先に当たる点まで、直線の点を逆にたどる（1 周期の上限を守る隣どうしの点）
        np.testing.assert_allclose(steps[0].line[0], p.press_in[j - 1])
        np.testing.assert_allclose(steps[0].line[-1], p.press_in[j - 20])
        np.testing.assert_allclose(steps[1].line[0], p.press_in[j - 21])
        np.testing.assert_allclose(steps[1].line[-1], p.q_approach)
        self.assertEqual([s.label for s in steps[2:]], ["経由の姿勢へ", "開始姿勢へ"])
        self.assertFalse(steps[3].check_workspace)

    def test_not_pressing_holds_command(self) -> None:
        """実際の指先が指令より奥に無ければ（押し当てていない）、下がる段は無く、今の指令のまま止める。"""
        p, arm = self.plan, self.planner.kin.arm_idx
        q_meas = self.q0.copy()
        q_meas[arm] = p.press_in[30]
        steps = self.retreat("press")(q_meas, p.press_in[30])
        self.assertFalse(steps[0].hold)
        np.testing.assert_allclose(steps[0].line[0], p.press_in[29])

    def test_stages_before_and_after(self) -> None:
        q = self.q0.copy()
        self.assertEqual([s.label for s in self.retreat("to_via")(q, self.start)], ["開始姿勢へ"])
        self.assertEqual([s.label for s in self.retreat("to_approach")(q, self.start)], ["経由の姿勢へ", "開始姿勢へ"])
        self.assertEqual([s.label for s in self.retreat("to_start")(q, self.start)], ["開始姿勢へ"])


if __name__ == "__main__":
    unittest.main()
