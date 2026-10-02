"""模擬 G1（sim/mujoco/g1_sim_server.py）を、実機と同じ口（DDS + ZMQ）で動かすテスト。

unitree_sdk2py（と CycloneDDS）、mujoco、pinocchio、公式モデルが無ければスキップする。
DDS の口 lo と、カメラのポート 5555 / 5556 を使うので、模擬 G1 がほかに動いていないときに実行すること。
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import unittest

from common.config import FEATURE_DIR, REPO_ROOT

MODELS = REPO_ROOT / "_local" / "button_press" / "models" / "g1_description" / "g1_29dof_rev_1_0.xml"
NEEDS = ("unitree_sdk2py", "mujoco", "pinocchio", "scipy", "zmq")
HAS_ALL = all(importlib.util.find_spec(m) is not None for m in NEEDS) and MODELS.exists()


@unittest.skipUnless(HAS_ALL, f"{', '.join(NEEDS)} / 公式モデルのどれかが無い")
class TestSimServer(unittest.TestCase):
    def test_example_agent_over_dds(self) -> None:
        """見本のエージェントを、実機用の経路（run_dds.py → arm_sdk + RGB-D）で動かし、模擬 G1 が success と判定する。"""
        py = sys.executable
        client = f"{py} {FEATURE_DIR / 'contest' / 'run_dds.py'} --agent {FEATURE_DIR / 'contest' / 'example_agent'}"
        out = REPO_ROOT / "_local" / "button_press_yada" / "results" / "test_sim_server.json"
        subprocess.run([py, str(FEATURE_DIR / "contest" / "evaluate_dds.py"), "--seed", "0", "--client", client,
                        "--out", str(out)], check=True, timeout=300)
        res = json.loads(out.read_text())["results"][0]
        self.assertEqual(res["outcome"], "success", res)
        # 見本の既知の弱点（▲ で指先が柱の横の壁をこする。30〜50 N）があるので、強くぶつからないことだけを確かめる
        self.assertLess(res["max_contact_force_n"], 80.0)
        self.assertGreater(res["counts"]["arm_sdk"], 100)  # 50 Hz で指令が届いている（5 秒で約 250）


if __name__ == "__main__":
    unittest.main()
