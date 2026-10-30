"""押し込みを止める処理（common/press.py の ContactStop）は、押す前からある遅れを基準にして、増え方で止める（2026-10-21）。"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from common.elevator_press import ElevatorPressController
from common.press import ContactStop, _hold_index, _hold_keep, _report_press
from common.press_planner import PressPlan


class FakeKin:
    """指先は q の先頭（x）そのもの。"""

    arm_idx = [0]

    @staticmethod
    def fk_pos(q: np.ndarray) -> np.ndarray:
        return np.array([float(q[0]), 0.0, 0.0])


def make(lag_m: float = 0.005, attempt: int = 1) -> ContactStop:
    planner = SimpleNamespace(cfg={"contact_stop": {"lag_m": lag_m, "retry_lag_m": 0.008, "keep_m": 0.002}}, kin=FakeKin())
    plan = SimpleNamespace(push_dir=np.array([1.0, 0.0, 0.0]))
    return ContactStop(planner, plan, attempt)  # type: ignore[arg-type]


def q(x: float) -> np.ndarray:
    return np.array([x])


class TestContactStop(unittest.TestCase):
    def test_constant_lag_before_pushing_does_not_stop(self) -> None:
        """押す前から 4.5 mm 遅れていても（実機の重力補償 0.5）、遅れが増えなければ止めない。"""
        cs = make()
        for k in range(100):  # 指令が 1 mm ずつ進み、実際の指先も 4.5 mm 遅れて付いてくる
            self.assertFalse(cs(q(0.001 * k), q(0.001 * k - 0.0045)))
        self.assertAlmostEqual(cs.baseline_m, 0.0045)
        self.assertAlmostEqual(cs.extra_m, 0.0)
        self.assertFalse(cs.stopped)

    def test_stops_when_lag_grows_from_baseline(self) -> None:
        """指先が進まなくなって遅れが基準から lag_m（5 mm）増えたら止める。"""
        cs = make()
        self.assertFalse(cs(q(0.0), q(-0.0045)))  # 基準 4.5 mm
        self.assertFalse(cs(q(0.003), q(-0.0045)))  # 遅れ 7.5 mm（増えた分 3 mm）
        self.assertFalse(cs(q(0.0045), q(-0.0045)))  # 増えた分 4.5 mm
        self.assertTrue(cs(q(0.0055), q(-0.0045)))  # 増えた分 5.5 mm
        self.assertTrue(cs.stopped)

    def test_old_rule_would_have_stopped_at_once(self) -> None:
        """以前の見方（遅れの絶対値）なら、1 mm 進んだだけで止まっていた（2026-10-21 の押し込み）。"""
        cs = make()
        q_cmd, q_meas = q(0.001), q(-0.0045)  # 遅れ 5.5 mm
        self.assertGreater(float((FakeKin.fk_pos(q_cmd) - FakeKin.fk_pos(q_meas))[0]), 0.005)
        cs(q(0.0), q(-0.0045))
        self.assertFalse(cs(q_cmd, q_meas))

    def test_retry_pushes_harder(self) -> None:
        cs = make(attempt=2)
        cs(q(0.0), q(0.0))
        self.assertFalse(cs(q(0.007), q(0.0)))
        self.assertTrue(cs(q(0.009), q(0.0)))

    def test_no_config_never_stops_but_records(self) -> None:
        planner = SimpleNamespace(cfg={}, kin=FakeKin())
        cs = ContactStop(planner, SimpleNamespace(push_dir=np.array([1.0, 0.0, 0.0])), 1)  # type: ignore[arg-type]
        cs(q(0.0), q(0.0))
        self.assertFalse(cs(q(0.5), q(0.0)))
        self.assertAlmostEqual(cs.extra_m, 0.5)

    def test_report_shows_how_far_the_tip_went(self) -> None:
        cs = make()
        cs(q(0.0), q(-0.0045))
        cs(q(0.0055), q(-0.0045))  # 止めた
        planner = SimpleNamespace(kin=FakeKin())
        plan = SimpleNamespace(push_dir=np.array([1.0, 0.0, 0.0]), end_point=np.array([0.065, 0.0, 0.0]),
                               approach_point=np.zeros(3), press_in=[0] * 100)
        got = []
        _report_press(cs, planner, plan, q(-0.0030), 1, 7, got.append)  # type: ignore[arg-type]
        info = got[0]
        self.assertTrue(info["stopped"])
        self.assertAlmostEqual(info["planned_m"], 0.065)
        self.assertAlmostEqual(info["tip_advance_m"], 0.0015)  # 指先は -4.5 mm → -3.0 mm
        self.assertEqual((info["attempt"], info["sent"], info["total"]), (1, 7, 100))


class TestHoldAfterStop(unittest.TestCase):
    """止めたあとの保持の位置は、押す前からある遅れの分も先に置く。腕は目標よりその分だけ手前で止まるので、
    keep_m だけ先に置くと、指先が止めた所より手前へ下がって対象から離れる（2026-10-21 の実機、4.1〜4.7 mm）。"""

    BASE = 0.0045  # 押す前からある遅れ（実機の重力補償 0.5）

    def test_hold_keep_adds_baseline(self) -> None:
        cs = make()
        cs(q(0.0), q(-self.BASE))
        planner = SimpleNamespace(cfg={"contact_stop": {"keep_m": 0.002}})
        self.assertAlmostEqual(_hold_keep(planner, cs), 0.002 + self.BASE)  # type: ignore[arg-type]
        self.assertAlmostEqual(_hold_keep(planner, None), 0.002)  # type: ignore[arg-type]
        neg = make()
        neg(q(0.0), q(0.0049))  # 腕が指令より先に出ている（遅れが負。模擬ロボットで −4.9 mm）なら、その分手前に置く
        self.assertAlmostEqual(_hold_keep(planner, neg), 0.002 - 0.0049)  # type: ignore[arg-type]

    def test_tip_stays_on_the_surface_while_holding(self) -> None:
        """面（x = 10 mm）で止まったとき、保持の目標から遅れの分だけ手前で落ち着いても、指先は面より奥にある。"""
        surface = 0.010
        planner = SimpleNamespace(cfg={"contact_stop": {"keep_m": 0.002}}, kin=FakeKin())
        plan = SimpleNamespace(push_dir=np.array([1.0, 0.0, 0.0]), press_in=[q(0.0005 * k) for k in range(100)])
        cs = make()
        cs(q(0.0), q(-self.BASE))
        for keep, ok in ((_hold_keep(planner, cs), True), (0.002, False)):  # type: ignore[arg-type]
            j = _hold_index(planner, plan, 60, q(surface), keep)  # type: ignore[arg-type]
            settled = float(plan.press_in[j][0]) - self.BASE  # 保持の目標から遅れの分だけ手前で落ち着く
            self.assertEqual(settled >= surface - 0.0005, ok, f"keep {keep * 1000:.1f} mm → 指先 {settled * 1000:.1f} mm")

    def test_elevator_stop_press_adds_lag0(self) -> None:
        """エレベーター側も同じ（ElevatorPressController._stop_press）。"""
        z3 = np.zeros(3)
        plan = PressPlan(side="right", q_approach=np.zeros(7), press_in=[], press_out=[], approach_point=z3,
                         target_point=z3, end_point=np.array([0.05, 0.0, 0.0]), push_dir=np.array([1.0, 0.0, 0.0]),
                         max_ik_err_m=0.0, depth=0.015, waist_q=z3)
        ctl = SimpleNamespace(cfg={"press": {"contact_stop": {"keep_m": 0.002}}}, plan_=plan, lag0=self.BASE,
                              _note=lambda msg: None, _enter=lambda phase, t: None)
        # 指令の点 p = 20 mm、遅れ 10 mm → 実際の指先 10 mm。保持の目標は 10 + 4.5 + 2 = 16.5 mm
        end = ElevatorPressController._stop_press(ctl, np.array([0.020, 0.0, 0.0]), 0.010, 0.0)  # type: ignore[arg-type]
        self.assertAlmostEqual(float(end[0]), 0.0165)
        ctl.lag0 = None  # 押し込みの最初の周期の前（基準が無い）なら keep_m だけ
        end = ElevatorPressController._stop_press(ctl, np.array([0.020, 0.0, 0.0]), 0.010, 0.0)  # type: ignore[arg-type]
        self.assertAlmostEqual(float(end[0]), 0.012)


if __name__ == "__main__":
    unittest.main()
