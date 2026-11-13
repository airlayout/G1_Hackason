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
from auto_press import (  # noqa: E402
    color_changed, expected_head_pitch, head_tilt_check, press_command, stand_check, take_agent_result,
)


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


class TestStandCheck(unittest.TestCase):
    """立っているか。2026-11-13 の実機の値で確かめる。"""

    def test_regular_mode_passes(self) -> None:
        """regular mode（エレベーターで使う）は、立っているだけで腰 pitch −3.3°・骨盤の傾き 2.9°。3° の一律の上限では止まった。"""
        self.assertTrue(stand_check(2.9, np.array([-0.1, 0.5, -3.3])).ok)

    def test_locked_standing_passes(self) -> None:
        self.assertTrue(stand_check(0.8, np.array([-0.1, 0.1, -0.2])).ok)

    def test_twisted_waist_is_stopped(self) -> None:
        """起動直後に腰 yaw が −13.6° だった。yaw は 3° のまま。"""
        self.assertFalse(stand_check(0.8, np.array([-13.6, 0.9, 3.3])).ok)
        self.assertFalse(stand_check(0.8, np.array([3.5, 0.0, 0.0])).ok)

    def test_roll_and_pitch_limits(self) -> None:
        self.assertFalse(stand_check(0.8, np.array([0.0, 3.5, 0.0])).ok)
        self.assertFalse(stand_check(0.8, np.array([0.0, 0.0, -6.5])).ok)

    def test_pelvis_tilt_limit(self) -> None:
        self.assertFalse(stand_check(5.5, np.array([0.0, 0.0, 0.0])).ok)

    def test_message_shows_limits(self) -> None:
        self.assertIn("上限 3/3/6°", stand_check(2.9, np.array([-0.1, 0.5, -3.3])).message)


class TestHeadTiltCheck(unittest.TestCase):
    """頭カメラの向きの事前確認。床だけが写る深度（tests/test_head_tilt.py の synth_frame）で確かめる。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.camera_geometry import HeadCameraTransform
        from common.config import load_config

        cls.cfg = load_config("robot.yaml")
        cls.tf = HeadCameraTransform(cls.cfg)

    def check(self, pitch: float | None) -> object:
        from test_head_tilt import synth_frame

        planes = [] if pitch is None else [(-0.73, -9.0)]  # 床（pelvis −0.73 m）。None は面が写らない
        f = synth_frame(self.tf, self.cfg, 50.0 if pitch is None else pitch, 0.0, planes)
        return head_tilt_check([f, f], np.zeros(3), None, (47.6, "URDF"))

    def test_neck_shifted_stops(self) -> None:
        """2026-11-03 のラボ（61.2°）・エレベーター（64.4°）は止める。"""
        for pitch in (61.2, 64.4):
            c = self.check(pitch)
            self.assertFalse(c.ok, c.message)
            self.assertIn("首", c.message)

    def test_fixed_neck_passes(self) -> None:
        """直したあとの 51〜54°（URDF から +3〜+7°）では押して点灯したので通す。"""
        for pitch in (47.6, 51.0, 54.2):
            c = self.check(pitch)
            self.assertTrue(c.ok, c.message)
            got = float(c.message.split("pitch ")[1].split("°")[0])
            self.assertAlmostEqual(got, pitch, delta=0.5)  # 求め方は 0.25° 刻み

    def test_no_floor_only_warns(self) -> None:
        c = self.check(None)
        self.assertTrue(c.ok)
        self.assertIn("測れない", c.message)

    def test_expected_pitch(self) -> None:
        """ボタンは URDF の向き。ボトルは head_camera_mount があればその値（camera.yaml は書いていないので URDF）。"""
        self.assertAlmostEqual(expected_head_pitch(args(target="up"))[0], 47.6, delta=0.1)
        self.assertEqual(expected_head_pitch(args(target="bottle"))[1], "URDF")


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
        self.assertEqual(env["J1GEN_HAND_MASK"], "0")  # 実機の手は黒いので、ランプを見るとき手の画素を除かない
        _, env = press_command(args(target="down", hand_mask=True), "enp129s0")
        self.assertEqual(env["J1GEN_HAND_MASK"], "1")
        self.assertNotIn("J1GEN_ARM", env)  # 指定しなければ elevator.yaml の press.arm のまま

    def test_waist_scan_is_always_off(self) -> None:
        """腰を回して探す（press.waist_scan）は、実機ではまだ確かめていないので、必ずオフ（強制のオフを渡す）。指定の口は無い。"""
        for kw in ({}, {"execute": True}, {"arm": "auto"}, {"button_set": "wheelchair"}):
            with self.subTest(kw=kw):
                _, env = press_command(args(target="up", **kw), "enp129s0")
                self.assertEqual(env["J1GEN_WAIST_SCAN"], "0")
        # 指定の口は無い（auto_press.py に --waist-scan の引数を作っていない）
        src = (FEATURE_DIR / "real" / "auto_press.py").read_text(encoding="utf-8")
        self.assertNotIn('add_argument("--waist', src)

    def test_arm(self) -> None:
        """--arm は、ボタンならエージェントへ環境変数 J1GEN_ARM で、ボトルなら press_bottle.py の --arm で渡す。"""
        _, env = press_command(args(target="up", arm="left"), "enp129s0")
        self.assertEqual(env["J1GEN_ARM"], "left")
        cmd, _ = press_command(args(target="bottle", arm="left"), "enp129s0")
        self.assertEqual(cmd[cmd.index("--arm") + 1], "left")


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
