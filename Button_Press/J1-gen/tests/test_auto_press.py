"""事前確認 → 押す、を Enter なしで流すスクリプト（real/auto_press.py）の判定とコマンドの組み立て。"""

from __future__ import annotations

import argparse
import sys
import unittest

import numpy as np

from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "real"))
from auto_press import color_changed, press_command  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
