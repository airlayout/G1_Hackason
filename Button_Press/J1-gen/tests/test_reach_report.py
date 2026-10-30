"""押せる範囲のまとめ（sim/reach_report.py）の部品。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "sim"))
import reach_report as rr  # noqa: E402


class TestReachReportParts(unittest.TestCase):
    def test_spans_split_where_a_value_is_missing(self) -> None:
        self.assertEqual(rr.spans([0.18, 0.20, 0.22, 0.28, 0.30], 0.02), "18〜22・28〜30 cm")
        self.assertEqual(rr.spans([0.32], 0.02), "32 cm")
        self.assertEqual(rr.spans([], 0.02), "なし")

    def test_status_separates_camera_desk_and_body(self) -> None:
        base = {"plan_ok": "0", "visible": "0", "partner": "", "stage": "J"}
        self.assertEqual(rr.status({**base, "plan_ok": "1", "visible": "1"}), "ok")
        self.assertEqual(rr.status({**base, "plan_ok": "1"}), "novis")
        self.assertEqual(rr.status({**base, "partner": "desk"}), "desk")
        self.assertEqual(rr.status({**base, "partner": "body"}), "body")
        self.assertEqual(rr.status(base), "other")
        # エレベーターはボタンと柱の面の両方がカメラに入れば通る
        self.assertEqual(rr.status({"plan_ok": "1", "vis_buttons": "1", "vis_region": "0", "partner": ""}), "novis")

    def test_ranges_need_every_condition_for_all(self) -> None:
        rows = [{"edge": e, "y": "-0.1", "plan_ok": "1", "visible": v, "partner": "", "stage": ""}
                for e, v in (("0.30", "1"), ("0.30", "1"), ("0.32", "1"), ("0.32", "0"))]
        self.assertEqual(rr.ranges_by_y(rows, "edge", [-0.1], groups=2), [(-0.1, "30 cm", "30〜32 cm")])

    def test_bottle_section_writes_svg_and_table(self) -> None:
        rows = [{"edge": e, "gap": "0.0", "y": y, "top": "0.0", "slope": "0.0", "plan_ok": "1", "visible": "1",
                 "partner": "", "stage": "", "reason": ""} for e in ("0.30", "0.32") for y in ("-0.1", "0.0")]
        md: list[str] = []
        page = rr.bottle_section("試し", rows, md)
        self.assertIn("<svg", page)
        self.assertIn("| 中央 | 30〜32 cm | 30〜32 cm |", md)

    def test_height_block_appears_with_three_or_more_tops(self) -> None:
        rows = [{"edge": "0.30", "gap": "0.0", "y": "0.0", "top": t, "slope": "0.0", "plan_ok": "1", "visible": v,
                 "partner": "", "stage": "", "reason": ""} for t, v in (("-0.04", "1"), ("0.0", "1"), ("0.04", "0"))]
        md: list[str] = []
        page = rr.bottle_section("試し", rows, md)
        self.assertIn("机の高さごと", page)
        self.assertIn("| 通る数 | 1/1（カメラ外 0） | 1/1（カメラ外 0） | 0/1（カメラ外 1） |", md)
        self.assertNotIn("机の高さごと", rr.bottle_section("試し", rows[:2], []))


if __name__ == "__main__":
    unittest.main()
