"""押す流れの節目で G1 に喋らせる（common/announce.py・configs/voice.yaml・elevator_press の on_event）。

実機のスピーカーは使わない。鳴らすコマンドを、すぐ終わる・失敗する・時間のかかるコマンドに差し替えて確かめる。
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from _env import needs
from common import announce
from common.announce import Announcer, speaker_command
from common.config import FEATURE_DIR, REPO_ROOT, load_config

EVENTS = ("press_start", "lit", "retry", "not_confirmed", "gave_up")


def cfg() -> dict:
    c = copy.deepcopy(load_config("voice.yaml"))
    c["pc2"]["dir"] = "/nonexistent/g1_voice_tts/phrases"  # local の「WAV が無い」注意は出るが、鳴らすコマンドは差し替える
    return c


class TestVoiceConfig(unittest.TestCase):
    def test_every_event_has_a_phrase(self) -> None:
        phrases = load_config("voice.yaml")["phrases"]
        self.assertEqual(set(phrases), set(EVENTS))
        self.assertEqual(phrases["press_start"], "ボタンを押します")
        self.assertEqual(phrases["lit"], "点灯成功")

    def test_pc2_dir_is_under_tmp(self) -> None:
        """共用の機体の運用ルール: PC2 では /tmp の下だけを使う。"""
        self.assertTrue(load_config("voice.yaml")["pc2"]["dir"].startswith("/tmp/"))


class TestSpeakerCommand(unittest.TestCase):
    def test_local_runs_g1_speaker_on_this_machine(self) -> None:
        c = cfg()
        cmd = speaker_command("local", c, "lit")
        d = c["pc2"]["dir"]
        self.assertEqual(cmd, ["python3", f"{d}/g1_speaker.py", "--wav", f"{d}/lit.wav", "--iface", "eth0"])

    def test_ssh_runs_it_on_pc2(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=False):
            import os
            os.environ.pop("PC2", None)
            cmd = speaker_command("ssh", cfg(), "lit")
        self.assertEqual(cmd[0], "ssh")
        self.assertIn("BatchMode=yes", cmd)
        self.assertEqual(cmd[-2], "unitree@192.168.0.82")
        self.assertIn("/lit.wav", cmd[-1])

    def test_ssh_host_can_be_overridden(self) -> None:
        with mock.patch.dict("os.environ", {"PC2": "unitree@192.168.123.164"}):
            self.assertEqual(speaker_command("ssh", cfg(), "lit")[-2], "unitree@192.168.123.164")

    def test_unknown_mode(self) -> None:
        with self.assertRaises(ValueError):
            speaker_command("dds", cfg(), "lit")
        with self.assertRaises(ValueError):
            Announcer(cfg(), "dds")


class TestAnnouncer(unittest.TestCase):
    def run_with(self, command: list[str], events: list[str], wait_s: float = 5.0, c: dict | None = None) -> Announcer:
        with mock.patch.object(announce, "speaker_command", side_effect=lambda mode, cfg, key: list(command)):
            ann = Announcer(c or cfg(), "local")
            for e in events:
                ann.say(e)
            ann.close(wait_s=wait_s)
        return ann

    def test_says_in_order_and_skips_unknown_events(self) -> None:
        ann = self.run_with(["true"], ["press_start", "nothing", "lit"])
        self.assertEqual([s["event"] for s in ann.said], ["press_start", "lit"])
        self.assertTrue(all(s["ok"] for s in ann.said))
        self.assertEqual(ann.said[1]["text"], "点灯成功")

    def test_say_returns_at_once_while_playing(self) -> None:
        """50 Hz の制御を止めない: 鳴らすのに 0.5 秒かかっても、say はすぐ返る。"""
        with mock.patch.object(announce, "speaker_command", side_effect=lambda mode, c, key: ["sleep", "0.5"]):
            ann = Announcer(cfg(), "local")
            t0 = time.monotonic()
            ann.say("press_start")
            ann.say("lit")
            self.assertLess(time.monotonic() - t0, 0.05)
            ann.close(wait_s=5.0)
        self.assertEqual([s["event"] for s in ann.said], ["press_start", "lit"])
        self.assertGreaterEqual(ann.said[0]["seconds"], 0.4)

    def test_failure_is_recorded_not_raised(self) -> None:
        ann = self.run_with(["false"], ["lit"])
        self.assertFalse(ann.said[0]["ok"])
        self.assertIn("error", ann.said[0])

    def test_missing_program_is_recorded_not_raised(self) -> None:
        ann = self.run_with(["/nonexistent/python3"], ["lit"])
        self.assertFalse(ann.said[0]["ok"])

    def test_stuck_command_times_out(self) -> None:
        c = cfg()
        c["pc2"]["timeout_s"] = 0.2
        ann = self.run_with(["sleep", "5"], ["lit"], c=c)
        self.assertFalse(ann.said[0]["ok"])
        self.assertIn("timed out", ann.said[0]["error"])
        self.assertLess(ann.said[0]["seconds"], 2.0)

    def test_close_does_not_wait_forever(self) -> None:
        with mock.patch.object(announce, "speaker_command", side_effect=lambda mode, c, key: ["sleep", "3"]):
            ann = Announcer(cfg(), "local")
            ann.say("lit")
            t0 = time.monotonic()
            ann.close(wait_s=0.2)
            self.assertLess(time.monotonic() - t0, 1.0)

    def test_empty_phrase_is_silent(self) -> None:
        c = cfg()
        c["phrases"]["gave_up"] = ""
        ann = self.run_with(["true"], ["gave_up"], c=c)
        self.assertEqual(ann.said, [])


@needs("mujoco", "pin")
class TestControllerEvents(unittest.TestCase):
    """ElevatorPressController が節目を on_event に渡す（tests/test_lit_check.py の TestSnapshotHook と同じ置き方）。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"), load_config("elevator.yaml"))
        cls.q0 = np.zeros(29)
        cls.q0[[15, 22]] = 0.2
        cls.q0[16], cls.q0[23] = 0.25, -0.25
        cls.q0[[18, 25]] = 1.0

    def ready(self) -> list[str]:
        from common.elevator_buttons import Button3D, CallButtons

        c = self.ctl
        c.reset("down", 0.02, np.full(17, 60.0))
        c.on_snapshot = None
        c.call = CallButtons(Button3D(np.array([0.309, 0.027, 0.267]), 0.033, "up", (300, 200, 340, 240), 100, "up"),
                             Button3D(np.array([0.309, 0.027, 0.187]), 0.033, "down", (300, 300, 340, 340), 100, "down"),
                             np.array([1.0, 0.0, 0.0]))
        c._plan(self.q0, c.call)
        c.q_hold = self.q0.copy()
        c.baseline_brightness = c.baseline_level = 0.0
        got: list[str] = []
        c.on_event = got.append
        return got

    def tearDown(self) -> None:
        self.ctl.on_event = None  # クラスで共有するので、口を残さない

    def test_not_lit_then_retry_then_not_confirmed(self) -> None:
        got = self.ready()
        dark = np.zeros((480, 640, 3), np.uint8)
        self.ctl._check(dark, 1.0)
        self.ctl._check(dark, 2.0)
        self.assertEqual(got, ["retry", "not_confirmed"])

    def test_lit(self) -> None:
        import cv2

        got = self.ready()
        img = np.full((480, 640, 3), 30, np.uint8)
        cv2.circle(img, (320, 320), 20, (150, 150, 150), -1)
        cv2.circle(img, (320, 320), 22, (240, 240, 255), 2)
        self.ctl.baseline_level = 100.0
        self.assertTrue(self.ctl._check(img, 1.0))
        self.assertEqual(got, ["lit"])

    def test_gave_up(self) -> None:
        got = self.ready()
        self.ctl._give_up(1.0, "テスト")
        self.assertEqual(got, ["gave_up"])

    def test_press_start_before_reach(self) -> None:
        """計画が立って reach に入るときに press_start（腕を伸ばす前に言い始める）。"""
        self.ready()
        c = self.ctl
        seen: list[str] = []
        c.on_event = lambda e: seen.append(f"{e}@{c.phase}")  # 言うときの段階も記録する（look のうち = 腕が動く前）
        # 見つける・計画・立ち位置の助言は差し替え、計画が立ったあと reach に入るところだけを見る
        with mock.patch.object(c, "_plan_arm"), mock.patch.object(c, "_stand_advice", return_value=""), \
                mock.patch.object(c.finder, "find", return_value=c.call), mock.patch.object(c.tracker, "add", return_value=c.call):
            c._look(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), np.eye(3), self.q0, 1.0)
        self.assertEqual(seen, ["press_start@look"])
        self.assertEqual(c.phase, "reach")

    def test_hook_error_does_not_stop_control(self) -> None:
        self.ready()

        def boom(_: str) -> None:
            raise RuntimeError("スピーカーが無い")

        self.ctl.on_event = boom
        self.ctl._check(np.zeros((480, 640, 3), np.uint8), 1.0)  # 例外を外へ出さず、押し直しへ進む
        self.assertEqual(self.ctl.phase, "press")


class TestAutoPressVoice(unittest.TestCase):
    def test_voice_is_passed_to_the_agent(self) -> None:
        sys.path.insert(0, str(FEATURE_DIR / "real"))
        from auto_press import press_command

        base = dict(target="down", execute=False, label="", gravity_scale=0.5, button_set=None, detector=None,
                    instruction="", camera_config="camera.yaml", host=None)
        _, env = press_command(argparse.Namespace(**base, voice="local"), "eth0")
        self.assertEqual(env["J1GEN_VOICE"], "local")
        _, env = press_command(argparse.Namespace(**base, voice=None), "eth0")
        self.assertNotIn("J1GEN_VOICE", env)


class TestVoicevoxBatch(unittest.TestCase):
    """Voice/voicevox_speak.py --batch（voicevox_core が無くても、合成を差し替えて確かめる）。"""

    def setUp(self) -> None:
        sys.path.insert(0, str(REPO_ROOT / "Voice"))
        import voicevox_speak

        self.vs = voicevox_speak
        self.tmp = Path(tempfile.mkdtemp())

    def batch(self, phrases: dict) -> int:
        f = self.tmp / "p.json"
        f.write_text(json.dumps(phrases, ensure_ascii=False), encoding="utf-8")
        with mock.patch.object(self.vs, "synthesize", return_value=b"\0" * (44 + 48000)):
            return self.vs.make_batch(None, f, self.tmp / "out", 21, 1.0, 0.0, 1.0)

    def test_makes_one_wav_per_phrase(self) -> None:
        self.assertEqual(self.batch({"lit": "点灯成功", "press_start": "ボタンを押します"}), 0)
        self.assertEqual(sorted(p.name for p in (self.tmp / "out").iterdir()), ["lit.wav", "press_start.wav"])

    def test_rejects_names_unsafe_for_the_pc2_command(self) -> None:
        self.assertEqual(self.batch({"lit; rm -rf ~": "点灯成功"}), 1)
        self.assertEqual(self.batch({"lit": "  "}), 1)


if __name__ == "__main__":
    unittest.main()
