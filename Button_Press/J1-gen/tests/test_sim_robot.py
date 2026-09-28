"""ループバックの模擬ロボット（sim/sim_robot_server.py）に、実機用のスクリプトをそのまま当てて確かめる（タスク6）。

- 実機と同じ DDS のトピック（lo / domain 1）と ZMQ のカメラ（127.0.0.1）を使う
- わざと起こした故障（arm_sdk が効かない、ゼロトルク、lowstate が途切れる、機体構成の違い、腰が倒れる）で、
  中止の処理が働き、決まった終了コードで終わること

unitree_sdk2py が無い環境（CI など）ではスキップする。1 件あたり 20〜60 秒かかる。
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
import unittest

from common.config import FEATURE_DIR, REPO_ROOT
from common.dds import PeerMismatchError, check_peer, init_dds
from common.arm.types import JointState

try:
    import unitree_sdk2py  # noqa: F401

    HAVE_SDK = True
except ImportError:
    HAVE_SDK = False

PY = sys.executable
SERVER = str(FEATURE_DIR / "sim" / "sim_robot_server.py")
MOVE = str(FEATURE_DIR / "real" / "move_arm_real.py")
PRESS = str(FEATURE_DIR / "real" / "press_bottle.py")
RGBD_PORT, RGB_PORT = 15656, 15655


class SimRobot:
    def __init__(self, *fault_args: str) -> None:
        self.args = list(fault_args)

    def __enter__(self) -> "SimRobot":
        self.proc = subprocess.Popen(
            [PY, "-u", SERVER, "--rgbd-port", str(RGBD_PORT), "--rgb-port", str(RGB_PORT)] + self.args,
            cwd=REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(4.0)  # MuJoCo の読み込みと DDS の立ち上がり
        return self

    def __exit__(self, *exc: object) -> None:
        self.proc.terminate()
        try:
            self.out, _ = self.proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.out, _ = self.proc.communicate()


def final_counts(server_out: str) -> str:
    """模擬ロボットの最後の「終了: {...}」の行（途中の 5 秒ごとの表示ではなく）。"""
    lines = [ln for ln in server_out.splitlines() if "[sim_robot] 終了" in ln]
    return lines[-1] if lines else ""


def run(cmd: list[str], stdin: str = "", timeout: float = 150) -> tuple[int, str]:
    p = subprocess.run([PY] + cmd, cwd=REPO_ROOT, input=stdin, capture_output=True, text=True, timeout=timeout,
                       env=dict(os.environ, PYTHONUNBUFFERED="1"))
    return p.returncode, p.stdout + p.stderr


MOVE_ARGS = [MOVE, "--path", "arm_sdk", "--network-interface", "lo", "--execute", "--no-confirm"]


class TestPeerCheck(unittest.TestCase):
    def _st(self, marker: bool) -> JointState:
        import numpy as np

        return JointState(np.zeros(29), np.zeros(29), np.ones(29, int), 5, time.monotonic(), sim_marker=marker)

    def test_peer_mismatch(self) -> None:
        check_peer(self._st(True), "lo")
        check_peer(self._st(False), "enp3s0")
        with self.assertRaises(PeerMismatchError):
            check_peer(self._st(False), "lo")  # lo なのに目印が無い
        with self.assertRaises(PeerMismatchError):
            check_peer(self._st(True), "enp3s0")  # 実機の口なのに模擬ロボット

    def test_real_interface_cannot_use_sim_domain(self) -> None:
        with self.assertRaises(ValueError):
            init_dds(1, "enp3s0")


@unittest.skipUnless(HAVE_SDK, "unitree_sdk2py が無い")
class TestSimRobotFaults(unittest.TestCase):
    def test_normal_move(self) -> None:
        with SimRobot():
            code, out = run(MOVE_ARGS)
        self.assertEqual(code, 0, out)
        self.assertIn("相手: 模擬ロボット（目印を確認）", out)

    def test_arm_sdk_ignored_is_detected(self) -> None:
        with SimRobot("--fault", "ignore_arm_sdk"):
            code, out = run(MOVE_ARGS)
        self.assertEqual(code, 3, out)
        self.assertIn("指令どおりに動いていない", out)

    def test_zero_torque_refused_before_sending(self) -> None:
        with SimRobot("--fault", "motor_mode0") as s:
            code, out = run(MOVE_ARGS)
        self.assertEqual(code, 4, out)
        self.assertIn("ゼロトルク", out)
        self.assertIn("'arm_sdk': 0,", final_counts(s.out))  # 何も送っていない

    def test_wrong_mode_machine_refused(self) -> None:
        with SimRobot("--fault", "mode_machine") as s:
            code, out = run(MOVE_ARGS)
        self.assertEqual(code, 4, out)
        self.assertIn("mode_machine", out)
        self.assertIn("'arm_sdk': 0,", final_counts(s.out))

    def test_lowstate_dropout_stops_safely(self) -> None:
        with SimRobot("--fault", "lowstate_dropout", "--fault-after", "7"):
            code, out = run(MOVE_ARGS + ["--delta-deg", "-25", "0", "0", "20", "0", "0", "0"])
        self.assertEqual(code, 4, out)
        self.assertIn("途切れている", out)
        self.assertIn("安全終了", out)

    def test_waist_sag_stops_safely(self) -> None:
        with SimRobot("--fault", "waist_sag", "--fault-after", "6"):
            code, out = run(MOVE_ARGS + ["--delta-deg", "-25", "0", "0", "20", "0", "0", "0"])
        self.assertEqual(code, 5, out)
        self.assertIn("腰の角度が開始時から", out)
        self.assertIn("安全終了", out)


@unittest.skipUnless(HAVE_SDK, "unitree_sdk2py が無い")
class TestSimRobotPipeline(unittest.TestCase):
    def test_press_bottle_real_script(self) -> None:
        """実機用の全体のスクリプトを、模擬ロボット相手にそのまま実行する（色の検出、机を障害物に）。"""
        cam = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        cam.write(f"rgbd: {{server_address: 127.0.0.1, port: {RGBD_PORT}, timeout_ms: 5000}}\n")
        cam.close()
        with tempfile.TemporaryDirectory() as logs, SimRobot() as s:
            code, out = run([PRESS, "--path", "arm_sdk", "--network-interface", "lo", "--camera-config", cam.name,
                             "--detector", "color", "--sim-scene-obstacles", "--seed-pose", "", "--log-dir", logs,
                             "--execute", "--no-confirm"], timeout=240)
        os.unlink(cam.name)
        self.assertEqual(code, 0, out)
        self.assertIn("実測の腰の角度で計算し直した", out)
        self.assertIn("結果: 成功", out)
        self.assertTrue(final_counts(s.out), s.out[-500:])
        self.assertNotIn("'arm_sdk': 0,", final_counts(s.out))  # 指令が模擬ロボットに届いている


if __name__ == "__main__":
    unittest.main()
