"""腕の定型動作（actions/poses.py）の確認。

- 角度が関節の可動範囲（余裕つき）に収まる
- FK の指先が、計画値（pelvis 座標）と 1 cm 以内で合う（角度を書き換えたとき計画値と食い違いに気づくため）
- sim（MuJoCo）で最後まで動き、終了コード 0 で戻る
"""

from __future__ import annotations

import sys
import unittest

import numpy as np

from _env import needs
from actions.poses import ACTIONS
from common.arm import joint_limits
from common.config import load_config
from common.realday import FingertipFK
from common.robot_model import RIGHT_ARM_IDX

ARM = list(RIGHT_ARM_IDX)


@needs("mujoco", "pin", "models")
class TestActionPoses(unittest.TestCase):
    def setUp(self) -> None:
        self.robot_cfg = load_config("robot.yaml")
        self.lower, self.upper = joint_limits(self.robot_cfg)
        self.fk = FingertipFK(self.robot_cfg, load_config("press.yaml")["ik"])
        self.margin = float(load_config("arm.yaml")["safety"]["limit_margin_rad"])

    def test_within_joint_limits(self) -> None:
        for a in ACTIONS.values():
            for s in a.steps:
                q = np.radians(s.q_deg)
                self.assertTrue(np.all(q >= self.lower[ARM] + self.margin), f"{a.name}/{s.label}: 下限")
                self.assertTrue(np.all(q <= self.upper[ARM] - self.margin), f"{a.name}/{s.label}: 上限")

    def test_planned_tip_matches_fk(self) -> None:
        for a in ACTIONS.values():
            q = np.zeros(29)
            q[ARM] = np.radians(a.steps[-1].q_deg)
            tip = self.fk.positions(q)["right"]
            self.assertLess(float(np.linalg.norm(tip - np.array(a.planned_tip_m))), 0.01, a.name)


@needs("mujoco", "pin", "models")
class TestActionSim(unittest.TestCase):
    def test_runs_in_sim(self) -> None:
        from actions.runner import main

        for name in ACTIONS:
            self.assertEqual(main(["--action", name, "--path", "sim", "--execute", "--no-confirm"]), 0, name)

    def test_waist_kp_in_sim(self) -> None:
        """腰の kp・kd を変えても sim で最後まで動く（sim は lowcmd なので arm_sdk の腰の PD は使わない。引数が通ることの確認）。"""
        from actions.runner import main

        self.assertEqual(main(["--action", "point", "--path", "sim", "--execute", "--no-confirm",
                               "--waist-kp", "150", "--waist-kd", "3"]), 0)


class TestActionArgs(unittest.TestCase):
    def test_rejects_out_of_range(self) -> None:
        """腰の kp・kd・上限の範囲外は、ロボットにつなぐ前に引数の誤りで止まる。"""
        from actions.runner import main

        for extra in (["--waist-kp", "400"], ["--waist-kp", "0"], ["--waist-kd", "6"], ["--waist-max-deg", "20"]):
            with self.assertRaises(SystemExit, msg=str(extra)):
                main(["--action", "point", "--path", "sim", *extra])


class TestActionApi(unittest.TestCase):
    def test_unverified_refused(self) -> None:
        from actions.api import perform

        with self.assertRaises(ValueError):
            perform("point_level")
        with self.assertRaises(ValueError):
            perform("nothing")

    def test_cli_refuses_unverified(self) -> None:
        from actions.api import cli

        self.assertEqual(cli(["point_level", "--sim"]), 2)


@needs("mujoco", "pin", "models")
class TestActionApiSim(unittest.TestCase):
    def test_perform_sim(self) -> None:
        from actions.api import perform

        self.assertEqual(perform("salute", path="sim", execute=True, confirm=False), 0)


if __name__ == "__main__":
    unittest.main()
