"""推定トルク（tau_est）から押す力を推定する（common/push_force.py）。

- 作り物のトルク（重力のトルク + Jᵀ f）から、押した力 f を取り出せる。腕が動いて重力のトルクが変わっても、押していなければ 0
- 記録から出し直す（replay）と、エージェントの中で測ったのと同じになる
- Yada の評価環境（MuJoCo）で 1 回押し、推定した力が、シミュレーターが測った本当の押す力に近い
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from _env import needs
from common.config import FEATURE_DIR, REPO_ROOT, load_config
from common.robot_model import NUM_MOTORS, RIGHT_ARM_IDX

ARM = list(RIGHT_ARM_IDX)
# 腕を前に出して押すときに近い姿勢（右腕。肩ピッチ・肩ロール・肩ヨー・肘・手首 3）
Q_PRESS = np.radians([-60.0, -10.0, 0.0, 30.0, 0.0, 0.0, 0.0])
PUSH = np.array([1.0, 0.0, 0.0])


@needs("pin", "mujoco")
class TestPushForce(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from common.arm.gravity import GravityModel
        from common.kinematics import ArmKinematics

        robot = load_config("robot.yaml")
        cls.kin = ArmKinematics(robot, "right", load_config("press.yaml")["ik"])
        cls.gravity = GravityModel(robot)

    def pose(self, delta: float = 0.0) -> np.ndarray:
        q = np.zeros(NUM_MOTORS)
        q[ARM] = Q_PRESS + delta
        return q

    def tau(self, q: np.ndarray, f: np.ndarray) -> np.ndarray:
        """止まっているときのモーターのトルク: 重力を支える分 + 指先が f で押している反力を支える分。"""
        t = self.gravity.torques(q)
        t[ARM] += self.kin.jacobian_pos(q).T @ f
        return t

    def monitor(self):  # type: ignore[no-untyped-def]
        from common.push_force import PushForceMonitor

        return PushForceMonitor(self.kin, self.gravity)

    def test_jacobian_matches_finite_difference(self) -> None:
        q = self.pose()
        J = self.kin.jacobian_pos(q)
        h = 1e-6
        for k, j in enumerate(ARM):
            dq = np.zeros(NUM_MOTORS)
            dq[j] = h
            num = (self.kin.fk_pos(q + dq) - self.kin.fk_pos(q - dq)) / (2 * h)
            np.testing.assert_allclose(J[:, k], num, atol=1e-6)

    def test_recovers_the_pushing_force(self) -> None:
        mon = self.monitor()
        q0 = self.pose()
        for _ in range(10):
            mon.remember(q0, self.tau(q0, np.zeros(3)))
        mon.start(PUSH)
        q1 = self.pose(0.03)  # 押し込んで腕の姿勢が少し変わった
        f = np.array([8.0, 1.5, -0.5])
        along = mon.measure("press", q1, self.tau(q1, f))
        self.assertAlmostEqual(along, 8.0, places=3)
        mon.measure("hold", q1, self.tau(q1, 0.5 * f))
        s = mon.finish()
        assert s is not None
        self.assertAlmostEqual(s.press_max_n, 8.0, places=3)
        self.assertAlmostEqual(s.press_side_n, float(np.hypot(1.5, 0.5)), places=3)
        self.assertAlmostEqual(s.hold_mean_n or 0.0, 4.0, places=3)
        self.assertEqual(s.baseline_samples, 10)
        self.assertIn("8.0 N", s.text())

    def test_arm_moving_without_contact_is_zero(self) -> None:
        """重力のトルクは姿勢で変わるが、その分は引くので、押していなければ 0。重力を引かないと力に見える。"""
        from common.push_force import PushForceMonitor

        q0 = self.pose()
        with_g, without_g = self.monitor(), PushForceMonitor(self.kin, None)
        for m in (with_g, without_g):
            for _ in range(10):
                m.remember(q0, self.tau(q0, np.zeros(3)))
            m.start(PUSH)
        q1 = self.pose(0.15)
        self.assertAlmostEqual(with_g.measure("press", q1, self.tau(q1, np.zeros(3))), 0.0, places=6)
        self.assertGreater(abs(without_g.measure("press", q1, self.tau(q1, np.zeros(3)))), 0.5)

    def test_no_torque_gives_nothing(self) -> None:
        mon = self.monitor()
        mon.remember(self.pose(), None)
        mon.start(PUSH)
        self.assertIsNone(mon.measure("press", self.pose(), None))
        self.assertIsNone(mon.finish())

    def test_replay_matches_the_online_monitor(self) -> None:
        """記録から出し直すと、エージェントの中で測ったのと同じ（押し直しを含む 2 回）。"""
        from common.push_force import replay

        phases = ["settle"] * 12 + ["press"] * 5 + ["hold"] * 3 + ["check"] * 12 + ["press"] * 4 + ["hold"] * 2
        n = len(phases)
        q = np.array([self.pose(0.002 * i) for i in range(n)])
        push = np.array([max(0, i - 14) * 2.0 if ph in ("press", "hold") else 0.0 for i, ph in enumerate(phases)])
        tau = np.array([self.tau(q[i], np.array([push[i], 0.0, 0.0])) for i in range(n)])
        imu = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
        force, summaries = replay(self.kin, self.gravity, phases, q, tau, imu, PUSH)
        self.assertEqual([s.attempt for s in summaries], [1, 2])
        self.assertTrue(np.isnan(force[0]))
        mon = self.monitor()
        online = []
        prev = ""
        for i, ph in enumerate(phases):
            if ph == "press" and prev != "press":
                mon.start(PUSH)
            if prev == "hold" and ph != "hold":
                online.append(mon.finish())
            if ph in ("press", "hold"):
                mon.measure(ph, q[i], tau[i], imu[i])
            mon.remember(q[i], tau[i], imu[i])
            prev = ph
        online.append(mon.finish())
        self.assertEqual([s.to_dict() for s in summaries], [s.to_dict() for s in online])  # type: ignore[union-attr]
        self.assertAlmostEqual(summaries[0].press_max_n, 2 * 2.0, places=3)  # press の最後の行（i=16）


@needs("pin", "mujoco", "render")
class TestPushForceInContest(unittest.TestCase):
    def test_estimate_is_close_to_the_simulated_force(self) -> None:
        """Yada の評価環境（MuJoCo、種 0 の ▲）で押し、エージェントが推定した押す力の最大が、シミュレーターの
        ボタンが受けた力の最大に近い（2026-11-04: 推定 10.1 N、本当 10.5 N）。重力を引かないと 15.0 N と出ていた。"""
        with tempfile.TemporaryDirectory() as tmp:
            res, out = Path(tmp) / "agent.json", Path(tmp) / "eval.json"
            env = dict(os.environ, J1GEN_RESULT_JSON=str(res), J1GEN_BODY_LOG=str(Path(tmp) / "body_log"))
            r = subprocess.run([sys.executable, str(REPO_ROOT / "Button_Press" / "Yada" / "contest" / "evaluate.py"),
                                "--agent", str(FEATURE_DIR / "contest_agent"), "--seed", "0", "--workers", "1",
                                "--out", str(out)], cwd=REPO_ROOT, capture_output=True, text=True, env=env,
                               timeout=600)
            if r.returncode != 0 and ("ModuleNotFoundError" in r.stderr or "ImportError" in r.stderr):
                self.skipTest(f"評価環境（Yada）の読み込みに必要なものが無い: {r.stderr.strip().splitlines()[-1]}")
            self.assertEqual(r.returncode, 0, r.stderr[-2000:])
            truth = json.loads(out.read_text())["results"][0]
            forces = json.loads(res.read_text())["press_forces"]
            self.assertEqual(truth["outcome"], "success")
            self.assertEqual(len(forces), 1)
            est = forces[0]["press_max_n"]
            print(f"\n[test] 押す力の最大: 推定 {est:.1f} N、シミュレーター {truth['max_press_force_n']:.1f} N")
            self.assertLess(abs(est - truth["max_press_force_n"]), 3.0)
            # 記録の列にも入る（real/press_force.py が読む）
            logs = list((Path(tmp) / "body_log").glob("*/body_log.csv"))
            self.assertEqual(len(logs), 1)
            header = logs[0].read_text(encoding="utf-8").splitlines()[0].split(",")
            self.assertIn("tau28", header)
            self.assertIn("push_force_n", header)
            meta = json.loads((logs[0].parent / "meta.json").read_text())
            self.assertIn("push_dir", meta)


if __name__ == "__main__":
    unittest.main()
