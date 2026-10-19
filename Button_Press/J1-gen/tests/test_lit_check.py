"""点灯の確認（common/elevator_press.py の _lit_level / _check）と、その記録（画像を残す口、contest_agent/agent.py）。

実物の写真（ラボのエレベーター、2026-10-15 のスマホ。tests/data/lab_elevator/）の切り出しで、明るい部分の値が
点灯と消灯で分かれることを確かめる。評価環境（シミュレーション）の画像だけでは分からない、縁の細い輪だけが光る点灯と、
金属の面が反射で白い消灯を含む。
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from _env import needs
from common.config import REPO_ROOT, load_config

DATA = Path(__file__).resolve().parent / "data" / "lab_elevator"
ROOT = Path(__file__).resolve().parents[1]


@needs("mujoco", "pin")
class TestLitCheckOnRealPhotos(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"), load_config("elevator.yaml"))
        cls.min_rise = float(cls.ctl.cfg["press"]["lit_check"]["min_rise"])
        cls.levels: dict[str, list[float]] = {"lit": [], "unlit": [], "glint": []}
        for c in json.loads((DATA / "manifest.json").read_text(encoding="utf-8"))["cases"]:
            rgb = cv2.cvtColor(cv2.imread(str(DATA / c["file"])), cv2.COLOR_BGR2RGB)
            cls.levels[c["kind"]].append(cls.ctl._lit_level(rgb, tuple(c["bbox"])))

    def test_fixtures_exist(self) -> None:
        self.assertTrue(all(len(v) >= 2 for v in self.levels.values()), {k: len(v) for k, v in self.levels.items()})

    def test_lit_ring_is_bright_and_unlit_is_not(self) -> None:
        """点灯（縁の輪が光る）は 220 以上、普通の消灯は 140 以下（実測は点灯 224〜230、消灯 67〜131）。"""
        self.assertGreaterEqual(min(self.levels["lit"]), 220)
        self.assertLessEqual(max(self.levels["unlit"]), 140)

    def test_rise_is_far_above_the_threshold(self) -> None:
        """押す前が普通の消灯なら、点灯したときの上がり方はしきい値（min_rise）の 2 倍以上ある。"""
        self.assertGreaterEqual(min(self.levels["lit"]) - max(self.levels["unlit"]), 2 * self.min_rise)

    def test_glint_baseline_is_a_known_limit(self) -> None:
        """現状の限界を固定する: 消灯でも金属の面が反射で白いと 190〜220 になり、押す前の値がこれだと、点灯しても
        上がり方がしきい値に届かない。実機の画像で直すことにしたら、このテストを直す。"""
        self.assertGreaterEqual(min(self.levels["glint"]), 190)
        self.assertLessEqual(max(self.levels["glint"]), 220)
        self.assertLess(min(self.levels["lit"]) - max(self.levels["glint"]), self.min_rise)


@needs("mujoco", "pin")
class TestSnapshotHook(unittest.TestCase):
    """押す前・点灯の確認の画像を渡す口（on_snapshot）。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"), load_config("elevator.yaml"))
        cls.q0 = np.zeros(29)
        cls.q0[[15, 22]] = 0.2
        cls.q0[16], cls.q0[23] = 0.25, -0.25
        cls.q0[[18, 25]] = 1.0

    def ready(self) -> None:
        from common.elevator_buttons import Button3D, CallButtons

        c = self.ctl
        c.reset("down", 0.02, np.full(17, 60.0))
        c.on_snapshot = None  # クラスで共有するので、前のテストの口を残さない
        c.call = CallButtons(Button3D(np.array([0.309, 0.027, 0.267]), 0.033, "up", (300, 200, 340, 240), 100, "up"),
                             Button3D(np.array([0.309, 0.027, 0.187]), 0.033, "down", (300, 300, 340, 340), 100, "down"),
                             np.array([1.0, 0.0, 0.0]))
        c._plan(self.q0, c.call)
        c.q_hold = self.q0.copy()
        c.baseline_brightness = c.baseline_level = 0.0

    def test_no_hook_is_fine(self) -> None:
        self.ready()
        self.assertIsNone(self.ctl.on_snapshot)
        self.ctl._check(np.zeros((480, 640, 3), np.uint8), 1.0)

    def test_check_hands_over_image_and_numbers(self) -> None:
        self.ready()
        got: list[tuple[str, np.ndarray, dict]] = []
        self.ctl.on_snapshot = lambda kind, rgb, info: got.append((kind, rgb, info))
        dark = np.zeros((480, 640, 3), np.uint8)
        self.ctl._check(dark, 1.0)  # 点灯していない → 押し直し
        self.ctl._check(dark, 2.0)
        self.assertEqual([g[0] for g in got], ["check", "check"])
        first = got[0][2]
        self.assertIs(got[0][1], dark)
        self.assertEqual((first["target"], first["retry"], first["lit"], first["moved"]), ("down", 0, False, False))
        self.assertEqual(first["bbox"], [300, 300, 340, 340])
        self.assertAlmostEqual(first["rise"], 0.0)
        self.assertEqual(got[1][2]["retry"], 1)

    def test_check_reports_a_lit_button_and_the_moved_box(self) -> None:
        self.ready()
        got: list[dict] = []
        self.ctl.on_snapshot = lambda kind, rgb, info: got.append(info)
        img = np.full((480, 640, 3), 30, np.uint8)
        cv2.circle(img, (330, 320), 20, (150, 150, 150), -1)  # 見つけたときの枠（中心 320, 320）から 10 px ずれたボタン
        cv2.circle(img, (330, 320), 22, (240, 240, 255), 2)  # 縁の輪が光る
        self.ctl.baseline_level = 100.0
        self.assertTrue(self.ctl._check(img, 1.0))
        self.assertTrue(got[0]["lit"])
        self.assertGreaterEqual(got[0]["rise"], 30)
        self.assertEqual(tuple(got[0]["found_box"]), (300, 300, 340, 340))

    def test_failing_hook_does_not_stop_the_press(self) -> None:
        self.ready()

        def boom(kind: str, rgb: np.ndarray, info: dict) -> None:
            raise OSError("disk full")

        self.ctl.on_snapshot = boom
        self.assertFalse(self.ctl._check(np.zeros((480, 640, 3), np.uint8), 1.0))
        self.assertEqual(self.ctl.phase, "press")  # 記録に失敗しても、押し直しに進む


class TestAgentSaveSnapshot(unittest.TestCase):
    """contest_agent/agent.py の BodyLog.save_snapshot が、画像と lit_log.json を書く。
    agent.py は Yada の common を読むので、Yada のフォルダで別のプロセスとして動かす。"""

    def test_writes_images_and_log(self) -> None:
        code = (
            "import sys, json, importlib.util, numpy as np\n"
            "from pathlib import Path\n"
            "sys.path.insert(0, '.')\n"
            "sp = importlib.util.spec_from_file_location('agent', sys.argv[2])\n"
            "m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)\n"
            "log = m.BodyLog(Path(sys.argv[1]) / 'run')\n"
            "rgb = np.zeros((60, 80, 3), np.uint8); rgb[10:20, 10:20] = (255, 0, 0)\n"
            "log.save_snapshot('before', rgb, {'t': 0.1, 'target': 'down', 'bbox': [5, 5, 30, 30], 'level': 10.0,"
            " 'baseline_level': 10.0, 'retry': 0})\n"
            "log.save_snapshot('check', rgb, {'t': 2.0, 'target': 'down', 'bbox': [6, 6, 31, 31], 'level': 200.0,"
            " 'baseline_level': 10.0, 'retry': 0, 'rise': 190.0, 'lit': True, 'found_box': [5, 5, 30, 30], 'moved': True})\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            r = subprocess.run([sys.executable, "-I", "-c", code, tmp, str(ROOT / "contest_agent" / "agent.py")],
                               cwd=REPO_ROOT / "Button_Press" / "Yada", capture_output=True, text=True,
                               env={"PYTHONPATH": str(REPO_ROOT / "Button_Press" / "Yada")})
            if r.returncode != 0 and ("ModuleNotFoundError" in r.stderr or "ImportError" in r.stderr):
                self.skipTest(f"評価環境（Yada）の読み込みに必要なものが無い: {r.stderr.strip().splitlines()[-1]}")
            self.assertEqual(r.returncode, 0, r.stderr)
            run = Path(tmp) / "run"
            names = sorted(p.name for p in run.iterdir())
            for n in ("lit_00_before.png", "lit_00_before_annotated.jpg", "lit_01_check.png",
                      "lit_01_check_annotated.jpg", "lit_log.json"):
                self.assertIn(n, names)
            rows = json.loads((run / "lit_log.json").read_text())
            self.assertEqual([r["kind"] for r in rows], ["before", "check"])
            self.assertTrue(rows[1]["lit"])
            back = cv2.cvtColor(cv2.imread(str(run / "lit_00_before.png")), cv2.COLOR_BGR2RGB)
            self.assertEqual(tuple(back[15, 15]), (255, 0, 0))  # 色の並び（RGB）が保たれている


if __name__ == "__main__":
    unittest.main()
