"""MuJoCo で押し込みを実行し、手先が目標に届くかを確かめる（タスク2の完了条件: 誤差 1cm 以内）。"""

from __future__ import annotations

import sys
import unittest

import numpy as np

from common.arm import UnsafeTargetError
from common.config import FEATURE_DIR, load_config

sys.path.insert(0, str(FEATURE_DIR / "sim"))
from press_sim import run_press_sim  # noqa: E402

from test_kinematics import REACHABLE  # noqa: E402


class TestPressSim(unittest.TestCase):
    def test_reaches_targets_within_1cm(self) -> None:
        for tgt, d in REACHABLE:
            errs = run_press_sim(np.array(tgt), np.array(d))
            self.assertLess(errs["approach"], 0.01, (tgt, errs))
            self.assertLess(errs["end"], 0.01, (tgt, errs))

    def test_plan_b_conditions_with_gravity_compensation(self) -> None:
        """プランB の条件（MuJoCo 側の補償なし）: 重力補償なしでは手先が下がり、倍率 1.0 で 1cm 以内に戻る。

        腰の保持は Kp=300（既定の Kp=40 では腰が倒れて監視で止まるため。README 参照）。
        """
        arm = load_config("arm.yaml")
        sim = dict(arm["sim"], emulate="lowcmd", gravity_compensation=False)
        lowcmd = dict(arm["lowcmd"])
        lowcmd["hold_kp"] = list(lowcmd["hold_kp"])
        lowcmd["hold_kp"][12:15] = [300, 300, 300]
        errs = {}
        for scale in (0.0, 1.0):
            errs[scale] = run_press_sim(
                np.array(REACHABLE[0][0]), np.array(REACHABLE[0][1]),
                arm_overrides={"sim": sim, "lowcmd": lowcmd,
                               "gravity_compensation": {"scale": scale, "tau_max_nm": 7.0}},
            )
        print(f"\n[test] プランB 条件の手先の誤差: 補償なし {errs[0.0]['end'] * 1000:.1f} mm、"
              f"倍率 1.0 {errs[1.0]['end'] * 1000:.1f} mm")
        self.assertLess(errs[1.0]["end"], 0.01, errs)
        self.assertGreater(errs[0.0]["end"], errs[1.0]["end"])

    def test_unreachable_is_rejected_before_moving(self) -> None:
        with self.assertRaises(UnsafeTargetError):
            run_press_sim(np.array([0.30, 0.10, 0.0]), np.array([1.0, 0, 0]))


if __name__ == "__main__":
    unittest.main()
