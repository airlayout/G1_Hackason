"""MuJoCo（胴体固定）で腕の指令部分を確かめる（タスク1の完了条件）。

- 指定した関節角へ動く（arm_sdk 近似・lowcmd 近似）
- Ctrl+C（SIGINT）で安全に止まる（実時間で動かして、途中で SIGINT を送る）
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path

import numpy as np

from common.arm import ArmCommander, WaistDeviationError, joint_limits, make_backend
from common.arm.gravity import GravityModel
from common.config import FEATURE_DIR, REPO_ROOT, load_config
from common.robot_model import RIGHT_ARM_IDX

ARM = np.array(RIGHT_ARM_IDX)
# 右腕の肩ロールは負が外向き（正だと上腕が胴体に当たる。2026-09-28 に MuJoCo で確認）
TARGET_DELTA = np.radians([-20, -10, 0, 15, 10, 10, 10])


class TestArmSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot_cfg = load_config("robot.yaml")
        cls.lower, cls.upper = joint_limits(cls.robot_cfg)

    def _move(self, emulate: str, gravity_comp: bool) -> float:
        cfg = load_config("arm.yaml")
        cfg["arm"] = "right"
        cfg["sim"].update(emulate=emulate, gravity_compensation=gravity_comp, realtime=False)
        be = make_backend(cfg, self.robot_cfg, dry_run=False, path="sim")
        be.open()
        with ArmCommander(be, cfg, self.lower, self.upper) as arm:
            q0 = arm.commanded_arm_q
            target = q0 + TARGET_DELTA
            arm.move_to(target)
            err = float(np.max(np.abs(be.read_state().q[ARM] - target)))
        return err

    def test_reaches_target_arm_sdk_emulation(self) -> None:
        self.assertLess(self._move("arm_sdk", True), np.radians(0.5))

    def test_reaches_target_lowcmd_emulation(self) -> None:
        self.assertLess(self._move("lowcmd", True), np.radians(0.5))

    def test_without_gravity_comp_sags_but_within_tolerance(self) -> None:
        """重力補償なし（実機の lowcmd に近い）では重さの分だけ下がる。量を記録しておく。"""
        err = self._move("lowcmd", False)
        print(f"\n[test] 重力補償なしの追従誤差 {np.degrees(err):.2f}°")
        self.assertGreater(err, np.radians(0.5))
        self.assertLess(err, np.radians(5.0))

    def test_gravity_compensation_removes_sag(self) -> None:
        """MuJoCo 側の補償を切り（実機のプランBの条件）、こちらから tau で重力補償を送ると下がりが消える。

        腰の保持は既定の Kp=300（Kp=40 だと腰が約 19° 倒れて、腰の監視で止まるため。README 参照）。
        """
        gm = GravityModel(self.robot_cfg)
        errs = {}
        for scale in (0.0, 1.0):
            cfg = load_config("arm.yaml")
            cfg["arm"] = "right"
            cfg["sim"].update(emulate="lowcmd", gravity_compensation=False, realtime=False)
            cfg["gravity_compensation"]["scale"] = scale
            be = make_backend(cfg, self.robot_cfg, dry_run=False, path="sim")
            be.open()
            with ArmCommander(be, cfg, self.lower, self.upper, gravity=gm) as arm:
                target = arm.commanded_arm_q + TARGET_DELTA
                arm.move_to(target)
                errs[scale] = float(np.max(np.abs(be.read_state().q[ARM] - target)))
        self.assertGreater(errs[0.0], np.radians(1.0))
        self.assertLess(errs[1.0], np.radians(0.2))

    def test_waist_sag_is_caught_with_weak_waist_gains(self) -> None:
        """腰の保持が公式 low_level サンプルの Kp=40 だと、腰が倒れて監視で止まる（既定を 300 にした理由）。"""
        cfg = load_config("arm.yaml")
        cfg["arm"] = "right"
        cfg["sim"].update(emulate="lowcmd", gravity_compensation=False, realtime=False)
        cfg["lowcmd"]["hold_kp"][12:15] = [60, 40, 40]
        cfg["lowcmd"]["hold_kd"][12:15] = [1, 1, 1]
        be = make_backend(cfg, self.robot_cfg, dry_run=False, path="sim")
        be.open()
        with self.assertRaises(WaistDeviationError):
            with ArmCommander(be, cfg, self.lower, self.upper) as arm:
                arm.move_to(arm.commanded_arm_q + TARGET_DELTA)

    def test_sigint_stops_safely(self) -> None:
        script = FEATURE_DIR / "sim" / "move_arm_sim.py"
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        proc = subprocess.Popen(
            [sys.executable, str(script), "--realtime"],
            cwd=REPO_ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        # weight を上げ終わって移動している最中（開始から約 3.5 秒）に送る
        deadline = time.monotonic() + 60
        lines: list[str] = []
        assert proc.stdout is not None
        for line in proc.stdout:
            lines.append(line)
            if "目標へ移動" in line:
                time.sleep(0.5)
                proc.send_signal(signal.SIGINT)
                break
            if time.monotonic() > deadline:
                break
        out, _ = proc.communicate(timeout=60)
        text = "".join(lines) + out
        self.assertEqual(proc.returncode, 130, text)
        self.assertIn("SIGINT を受信", text)
        self.assertIn("weight 1.00→0", text)
        self.assertIn("[arm] 終了", text)
        self.assertNotIn("Traceback", text)


if __name__ == "__main__":
    unittest.main()
