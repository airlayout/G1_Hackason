"""すき間の測定（real/measure_gap.py の gap_cm）。"""

from __future__ import annotations

import sys
import unittest

from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "real"))
from measure_gap import gap_cm  # noqa: E402


class TestGapCm(unittest.TestCase):
    def test_horizontal_ruler(self) -> None:
        # 定規 30 cm が 300 px（10 px / cm）。指先とボトルの面が 50 px 離れている → 5 cm
        r = gap_cm((100, 400), (400, 400), 30.0, (520, 300), (570, 300))
        self.assertAlmostEqual(r["cm_per_px"], 0.1)
        self.assertAlmostEqual(r["distance_cm"], 5.0)
        self.assertAlmostEqual(r["along_ref_cm"], 5.0)

    def test_along_ref_ignores_the_perpendicular_part(self) -> None:
        r = gap_cm((0, 0), (100, 0), 10.0, (0, 0), (30, 40))  # 1 px = 0.1 cm。直線 5 cm、物差し方向 3 cm
        self.assertAlmostEqual(r["distance_cm"], 5.0)
        self.assertAlmostEqual(r["along_ref_cm"], 3.0)

    def test_bad_ruler(self) -> None:
        with self.assertRaises(ValueError):
            gap_cm((1, 1), (1, 1), 30.0, (0, 0), (1, 1))


if __name__ == "__main__":
    unittest.main()
