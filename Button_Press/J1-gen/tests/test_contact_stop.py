"""押し込みを止める処理（common/press.py の ContactStop）は、押す前からある遅れを基準にして、増え方で止める（2026-10-21）。"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from common.press import ContactStop, _report_press


class FakeKin:
    """指先は q の先頭（x）そのもの。"""

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


if __name__ == "__main__":
    unittest.main()
