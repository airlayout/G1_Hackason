"""事前確認 → 押す、を Enter なしで流すスクリプト（real/auto_press.py）の判定とコマンドの組み立て。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.config import FEATURE_DIR, REPO_ROOT

sys.path.insert(0, str(FEATURE_DIR / "real"))
from auto_press import color_changed, press_command, take_agent_result  # noqa: E402


def args(**kw: object) -> argparse.Namespace:
    base = dict(target="bottle", execute=False, label="", gravity_scale=0.5, button_set=None, detector=None,
                instruction="", camera_config="camera.yaml", host=None)
    base.update(kw)
    return argparse.Namespace(**base)


class TestPrechecks(unittest.TestCase):
    def test_frozen_color_is_detected(self) -> None:
        """2026-10-21 は videohub のカラーが止まっていて、2 枚の差が 0.00 だった。動いていれば 1 前後。"""
        img = np.random.default_rng(0).integers(0, 255, (36, 64, 3), dtype=np.uint8)
        self.assertEqual(color_changed(img, img.copy()), 0.0)
        noisy = np.clip(img.astype(int) + np.random.default_rng(1).integers(-3, 4, img.shape), 0, 255)
        self.assertGreater(color_changed(img, noisy.astype(np.uint8)), 0.05)


class TestPressCommand(unittest.TestCase):
    def test_bottle_dry_run_sends_nothing(self) -> None:
        cmd, env = press_command(args(), "enp129s0")
        self.assertIn("press_bottle.py", cmd[1])
        self.assertNotIn("--execute", cmd)
        self.assertEqual(env, {})

    def test_bottle_execute_skips_enter(self) -> None:
        cmd, _ = press_command(args(execute=True, gravity_scale=0.5), "enp129s0")
        self.assertIn("--execute", cmd)
        self.assertIn("--no-confirm", cmd)
        self.assertEqual(cmd[cmd.index("--gravity-scale") + 1], "0.5")
        self.assertEqual(cmd[cmd.index("--path") + 1], "arm_sdk")
        self.assertEqual(cmd[cmd.index("--network-interface") + 1], "enp129s0")

    def test_extra_args_are_passed(self) -> None:
        cmd, _ = press_command(args(extra=["--seed-pose", "press_bottle"]), "enp129s0")
        self.assertEqual(cmd[-2:], ["--seed-pose", "press_bottle"])

    def test_button(self) -> None:
        cmd, env = press_command(args(target="down", button_set="wheelchair", detector="yolo"), "enp129s0")
        self.assertIn("run_dds.py", cmd[1])
        self.assertEqual(cmd[cmd.index("--target") + 1], "down")
        self.assertEqual(cmd[cmd.index("--network-interface") + 1], "enp129s0")
        self.assertIn("--no-loco", cmd)
        self.assertIn("--dry-run", cmd)
        self.assertEqual((env["J1GEN_BUTTON_SET"], env["J1GEN_BUTTON_DETECTOR"]), ("wheelchair", "yolo"))
        cmd, _ = press_command(args(target="down", execute=True), "enp129s0")
        self.assertNotIn("--dry-run", cmd)


class TestAgentResult(unittest.TestCase):
    """エージェントが書いた結果（中止の理由など）を auto の記録に取り込む。2026-11-03 は、計画まで行かずに終わった回の
    理由が画面にしか出ず、あとから分からなかった。"""

    def test_agent_writes_and_auto_takes_it(self) -> None:
        """contest_agent/agent.py の write_result が書いたものを take_agent_result が読み、ファイルは消す。
        agent.py は Yada の common を読むので、Yada のフォルダで別のプロセスとして動かす。"""
        code = (
            "import sys, importlib.util\n"
            "from pathlib import Path\n"
            "from types import SimpleNamespace as N\n"
            "sys.path.insert(0, '.')\n"
            "sp = importlib.util.spec_from_file_location('agent', sys.argv[2])\n"
            "m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)\n"
            "ctl = N(outcome='中止: 計画できない: 腕が体に当たる', phase='done',"
            " log=[N(phase='look', message='見つけた: ...'), N(phase='look', message='中止: 計画できない: 腕が体に当たる')])\n"
            "m.write_result(Path(sys.argv[1]), ctl, Path('/x/body_log/20261103_194132_up'))\n"
            "m.write_result(Path(sys.argv[1] + '.empty'), N(outcome='', phase='press', log=[]), None)\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "20261103_194127_up_agent.json"
            r = subprocess.run([sys.executable, "-I", "-c", code, str(out), str(FEATURE_DIR / "contest_agent" / "agent.py")],
                               cwd=REPO_ROOT / "Button_Press" / "Yada", capture_output=True, text=True,
                               env={"PYTHONPATH": str(REPO_ROOT / "Button_Press" / "Yada")})
            if r.returncode != 0 and ("ModuleNotFoundError" in r.stderr or "ImportError" in r.stderr):
                self.skipTest(f"評価環境（Yada）の読み込みに必要なものが無い: {r.stderr.strip().splitlines()[-1]}")
            self.assertEqual(r.returncode, 0, r.stderr)
            res = take_agent_result(out)
            self.assertFalse(out.exists())
            self.assertEqual(res["outcome"], "中止: 計画できない: 腕が体に当たる")
            self.assertEqual([n["phase"] for n in res["notes"]], ["look", "look"])
            self.assertEqual(res["body_log"], "/x/body_log/20261103_194132_up")
            empty = take_agent_result(Path(str(out) + ".empty"))
            self.assertEqual((empty["outcome"], empty["phase"]), ("（結果が出る前に終わった）", "press"))

    def test_missing_or_broken_file_is_none(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(take_agent_result(Path(tmp) / "none.json"))
            bad = Path(tmp) / "bad.json"
            bad.write_text("{")
            self.assertIsNone(take_agent_result(bad))


if __name__ == "__main__":
    unittest.main()
