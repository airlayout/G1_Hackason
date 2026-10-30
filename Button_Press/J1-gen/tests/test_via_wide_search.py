"""経由の姿勢を広く探し直す（sim/via_wide_search.py）の部品。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "sim"))
import via_wide_search as vw  # noqa: E402

HEADER = "edge,gap,y,top,slope,plan_ok,seed,visible,stage,partner,reason\n"


class TestViaWideSearch(unittest.TestCase):
    def test_load_cells_keeps_only_visible(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "map.csv").write_text(
                HEADER
                + "0.32,0.0,-0.15,0.0,0.0,1,taught/0-0,1,,,\n"
                + "0.16,0.0,-0.3,0.0,0.0,0,,0,1,desk,当たる\n"
                + "0.2,0.03,0.0,-0.04,,0,,1,2,desk,当たる\n",
                encoding="utf-8",
            )
            cells, base = vw.load_cells(Path(d))
        # 画像に入らないセルは数えない。slope が空なら 0
        self.assertEqual(cells, [(0.32, 0.0, -0.15, 0.0, 0.0), (0.2, 0.03, 0.0, -0.04, 0.0)])
        self.assertEqual(base[(0.2, 0.03, 0.0, -0.04, 0.0)]["partner"], "desk")

    def test_wide_via_is_inside_search_limits(self) -> None:
        # 山登りは今の広げた設定から始まるので、探す範囲の中にないと clip で別の姿勢に変わってしまう
        for v, (lo, hi) in zip(vw.WIDE_VIA, vw.LIMIT):
            self.assertTrue(lo <= v <= hi, (v, lo, hi))


if __name__ == "__main__":
    unittest.main()
