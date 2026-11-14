"""寄ってから押す（real/approach_and_press.py）。コマンドの組み立てと「押すか」の判定を確かめる（実機・カメラ不要）。"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from common.config import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "Button_Press" / "J1-gen" / "real"))
import approach_and_press as ap  # noqa: E402

RESULT = Path("/tmp/x/approach_result.json")


def parse(*argv: str):
    return ap.parse_args(list(argv))


class TestCommands(unittest.TestCase):
    def test_execute_passes_to_both(self) -> None:
        a = parse("--target", "down", "--execute")
        self.assertIn("--execute", ap.build_approach_cmd(a, RESULT))
        self.assertIn("--execute", ap.build_press_cmd(a, "right"))

    def test_dry_run_passes_to_neither(self) -> None:
        a = parse("--target", "down")
        self.assertNotIn("--execute", ap.build_approach_cmd(a, RESULT))
        self.assertNotIn("--execute", ap.build_press_cmd(a, "right"))

    def test_arm_goes_to_press_and_arms_to_approach(self) -> None:
        a = parse("--target", "up", "--arms", "right", "left")
        press = ap.build_press_cmd(a, "left")
        self.assertEqual(press[press.index("--arm") + 1], "left")
        appr = ap.build_approach_cmd(a, RESULT)
        self.assertEqual(appr[appr.index("--arms") + 1:appr.index("--arms") + 3], ["right", "left"])
        self.assertNotIn("--arms", press)

    def test_webcam_only_approach(self) -> None:
        a = parse("--target", "up", "--webcam")
        self.assertIn("--webcam", ap.build_approach_cmd(a, RESULT))
        self.assertNotIn("--webcam", ap.build_press_cmd(a, "right"))

    def test_common_options_and_result_json(self) -> None:
        a = parse("--target", "down", "--button-set", "wheelchair", "--detector", "yolo",
                  "--network-interface", "eth0", "--camera-config", "camera_onboard.yaml")
        for cmd in (ap.build_approach_cmd(a, RESULT), ap.build_press_cmd(a, "right")):
            for k, v in (("--button-set", "wheelchair"), ("--detector", "yolo"), ("--network-interface", "eth0"),
                         ("--camera-config", "camera_onboard.yaml"), ("--target", "down")):
                self.assertEqual(cmd[cmd.index(k) + 1], v)
        appr = ap.build_approach_cmd(a, RESULT)
        self.assertEqual(appr[appr.index("--result-json") + 1], str(RESULT))

    def test_no_safety_skip_and_press_args(self) -> None:
        a = parse("--target", "up", "--execute", "--press-args", "--voice", "local")
        press = ap.build_press_cmd(a, "right")
        self.assertEqual(press[-2:], ["--voice", "local"])
        for cmd in (press, ap.build_approach_cmd(a, RESULT)):
            self.assertNotIn("--skip-prechecks", cmd)
        self.assertNotIn("--press-confirm", press)


class TestDecide(unittest.TestCase):
    def test_ready_with_arm_presses(self) -> None:
        for ex in (True, False):
            self.assertTrue(ap.decide_press({"outcome": "ready", "arm": "right"}, 0, ex)[0])

    def test_ready_without_arm(self) -> None:
        self.assertFalse(ap.decide_press({"outcome": "ready", "arm": None}, 0, True)[0])

    def test_aborted_not_found_interrupted(self) -> None:
        for r, rc in (({"outcome": "aborted", "reason": "見つからない: x", "arm": None}, 1),
                      ({"outcome": "aborted", "reason": "Ctrl+C", "interrupted": True}, 130),
                      (None, 1)):
            self.assertFalse(ap.decide_press(r, rc, True)[0])

    def test_dry_run_needs_move(self) -> None:
        go, why = ap.decide_press({"outcome": "dry_run", "reason": "次の 1 回: 前へ", "arm": None}, 0, False)
        self.assertFalse(go)
        self.assertIn("寄る必要がある", why)


class TestApproachResultJson(unittest.TestCase):
    def test_option_exists(self) -> None:
        src = (REPO_ROOT / "Button_Press" / "J1-gen" / "real" / "approach.py").read_text()
        self.assertIn('"--result-json"', src)

    def test_read_result(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "r.json"
            self.assertIsNone(ap.read_result(p))
            p.write_text(json.dumps({"outcome": "ready", "arm": "left"}))
            self.assertEqual(ap.read_result(p)["arm"], "left")

    def test_help(self) -> None:
        r = subprocess.run([sys.executable, str(Path(ap.__file__)), "--help"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0)


if __name__ == "__main__":
    unittest.main()
