"""弱点のレポート（contest/report.py）と、失敗した段階の分類（runner.classify_stage）のテスト。作った結果で確かめる。"""

from __future__ import annotations

import unittest

from contest.report import build_report, condition_effects
from contest.runner import EpisodeResult, classify_stage


def _res(outcome: str, diag: dict | None = None, force: float | None = 0.0) -> EpisodeResult:
    r = EpisodeResult(seed=0, sim="mujoco", target="up", instruction="", outcome=outcome, max_contact_force_n=force)
    r.extra["diag"] = diag or {}
    return r


class TestClassify(unittest.TestCase):
    def test_stages(self) -> None:
        moved = {"max_arm_motion_rad": 1.0}
        self.assertEqual(classify_stage(_res("success")), "success")
        self.assertEqual(classify_stage(_res("wrong")), "wrong_button")
        self.assertEqual(classify_stage(_res("gave_up", {"max_arm_motion_rad": 0.01})), "no_motion")
        self.assertEqual(classify_stage(_res("gave_up", moved, force=50.0)), "collision")
        self.assertEqual(classify_stage(_res("gave_up", {**moved, "touched_target": True})), "touched_not_pressed")
        self.assertEqual(classify_stage(_res("timeout", {**moved, "min_tip_dist_target_m": 0.02})), "near_miss")
        self.assertEqual(classify_stage(_res("timeout", {**moved, "min_tip_dist_target_m": 0.2})), "not_reached")
        self.assertEqual(classify_stage(_res("timeout", moved)), "unknown")


class TestReport(unittest.TestCase):
    def _results(self) -> list[dict]:
        """カメラの角度の誤差が大きいと必ず失敗し、深度のノイズは関係ない、という作った結果。"""
        out = []
        for i in range(40):
            cam = i % 2  # 0: 小さい、1: 大きい
            noise = (i // 2) % 2
            ok = cam == 0
            out.append({"seed": i, "target": "up", "outcome": "success" if ok else "gave_up",
                        "stage": "success" if ok else "no_motion", "time_s": 5.0 if ok else None,
                        "max_contact_force_n": 0.0,
                        "extra": {"eval_set": "realistic", "diag": {},
                                  "conditions": {"camera_rpy_offset_deg": 0.2 + 1.2 * cam,
                                                 "depth_sigma_at_1m_mm": 5.0 + 5.0 * noise}}})
        return out

    def test_condition_effects_ranks_the_real_cause_first(self) -> None:
        eff = condition_effects(self._results())
        self.assertEqual(eff[0]["key"], "camera_rpy_offset_deg")
        self.assertAlmostEqual(eff[0]["diff"], -1.0)
        noise = next(e for e in eff if e["key"] == "depth_sigma_at_1m_mm")
        self.assertAlmostEqual(noise["diff"], 0.0)

    def test_report_contains_sections_and_ablation(self) -> None:
        rs = self._results()
        for r in rs[:10]:
            r["extra"]["ablation"] = "none"
            r["outcome"], r["stage"], r["time_s"] = "success", "success", 5.0
        for r in rs[10:20]:
            r["extra"]["ablation"] = "camera_mount"
            r["outcome"], r["stage"], r["time_s"] = "gave_up", "no_motion", None
        md = build_report(rs, {"agent": "x", "sim": "mujoco", "eval_set": "realistic", "ablation": True})
        for s in ("どこで失敗したか", "乱しを 1 種類ずつ", "カメラの取り付けの誤差", "-100 pt", "条件ごとの成功率", "再現"):
            self.assertIn(s, md)


if __name__ == "__main__":
    unittest.main()
