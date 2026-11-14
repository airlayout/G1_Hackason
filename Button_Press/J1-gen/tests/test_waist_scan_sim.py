"""腰を回して探す機能の評価環境の口（contest_agent/agent.py の J1GEN_WAIST_SCAN）と、確かめるスクリプト sim/waist_scan_sim.py の部品。

- J1GEN_WAIST_SCAN が "1" のときだけ waist_scan.enabled が True（未設定・"0"・ほかの値は False。yaml の値によらない）
- waist_scan_sim.py の引数の解釈・配置の作り方・集計（MuJoCo を回す 1 試行は 2 分以上かかるのでテストにしない。実際に回した結果は
  スクリプトを直接実行して見る）
"""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

from common.config import FEATURE_DIR, REPO_ROOT

ROOT = FEATURE_DIR
sys.path.insert(0, str(ROOT / "sim"))
import waist_scan_sim as w  # noqa: E402


class TestAgentWaistScanEnv(unittest.TestCase):
    """agent.py は Yada の common を読むので、Yada のフォルダで別のプロセスとして動かす。"""

    def enabled(self, env_value: str | None) -> str:
        code = (
            "import sys, importlib.util\n"
            "sys.path.insert(0, '.')\n"
            "sp = importlib.util.spec_from_file_location('agent', sys.argv[1])\n"
            "m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)\n"
            "a = m.make_agent()\n"
            "print('ENABLED=' + str(a.ctl.cfg['press']['waist_scan']['enabled']))\n"
        )
        env = {"PYTHONPATH": str(REPO_ROOT / "Button_Press" / "Yada")}
        if env_value is not None:
            env["J1GEN_WAIST_SCAN"] = env_value
        r = subprocess.run([sys.executable, "-I", "-c", code, str(ROOT / "contest_agent" / "agent.py")],
                           cwd=REPO_ROOT / "Button_Press" / "Yada", capture_output=True, text=True, env=env)
        if r.returncode != 0 and ("ModuleNotFoundError" in r.stderr or "ImportError" in r.stderr):
            self.skipTest(f"評価環境（Yada）の読み込みに必要なものが無い: {r.stderr.strip().splitlines()[-1]}")
        self.assertEqual(r.returncode, 0, r.stderr)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith("ENABLED=")]
        self.assertEqual(len(line), 1, r.stdout)
        return line[0].split("=", 1)[1]

    def test_only_one_turns_on(self) -> None:
        self.assertEqual(self.enabled("1"), "True")
        self.assertEqual(self.enabled(None), "False")
        self.assertEqual(self.enabled("0"), "False")
        self.assertEqual(self.enabled("true"), "False")  # "1" 以外はオンにしない


class TestWaistScanSimParts(unittest.TestCase):
    def test_make_jobs_both(self) -> None:
        jobs = w.make_jobs([0.32, 0.40], [-0.2, 0.2], ["up", "down"], "both", "auto", "basic")
        self.assertEqual(len(jobs), 2 * 2 * 2 * 2)
        self.assertEqual([j.scan for j in jobs[:2]], [False, True])  # 同じ配置を off → on
        self.assertEqual(jobs[0].seed, jobs[1].seed)
        self.assertEqual((jobs[0].x, jobs[0].y, jobs[0].target), (jobs[1].x, jobs[1].y, jobs[1].target))

    def test_make_jobs_on_off_and_bad(self) -> None:
        self.assertTrue(all(j.scan for j in w.make_jobs([0.4], [0.0], ["up"], "on", "auto", "basic")))
        self.assertTrue(all(not j.scan for j in w.make_jobs([0.4], [0.0], ["up"], "off", "auto", "basic")))
        with self.assertRaises(ValueError):
            w.make_jobs([0.4], [0.0], ["up"], "maybe", "auto", "basic")

    def test_place_wheelchair(self) -> None:
        scene = {"pelvis_height": 0.8, "wall": {"front_x": 1.0}, "column": {"protrusion": 0.1, "center_y": 0.0},
                 "buttons": {"items": [{"name": "up", "height": 0}, {"name": "down", "height": 0},
                                       {"name": "wc_up", "height": 0}, {"name": "wc_down", "height": 0}]},
                 "decals": [{"name": "wheelchair", "height": 0}]}
        w.place_wheelchair(scene, 0.36, -0.24)
        self.assertAlmostEqual(scene["wall"]["front_x"], 0.46)
        self.assertAlmostEqual(scene["column"]["center_y"], -0.24)
        h = {b["name"]: b["height"] for b in scene["buttons"]["items"]}
        self.assertAlmostEqual(h["up"], 1.01)
        self.assertAlmostEqual(h["down"], 0.94)

    def test_override(self) -> None:
        cfg = {"press": {"waist_scan": {"clear_x_m": 0.24}}}
        w.apply_overrides(cfg, ("press.waist_scan.clear_x_m=0.2", "press.waist_scan.yaws_deg=[5, -5]"))
        self.assertEqual(cfg["press"]["waist_scan"]["clear_x_m"], 0.2)
        self.assertEqual(cfg["press"]["waist_scan"]["yaws_deg"], [5, -5])
        with self.assertRaises(ValueError):
            w.parse_override("noequal")

    def test_classify_and_summarize(self) -> None:
        self.assertEqual(w.classify_failure("", "", [], "success"), "")
        self.assertIn("近い", w.classify_failure("", "柱に近すぎるので腰をひねって探さない（前の物まで 20 cm）", [], "gave_up"))
        row = {"x": 0.36, "y": 0.24, "target": "up", "reason": "", "lit": True, "waist_dev_max_deg": 1.0,
               "waist_round_max_deg": 0.0, "contact_force_n": 0.0}
        rows = [dict(row, scan=False, lit=False, reason="見つからない"), dict(row, scan=True)]
        s = w.summarize(rows)
        self.assertEqual(s["pairs"]["off_ng_on_ok"], 1)
        self.assertEqual(s["on"]["lit"], 1)
        self.assertEqual(s["off"]["lit"], 0)


if __name__ == "__main__":
    unittest.main()
