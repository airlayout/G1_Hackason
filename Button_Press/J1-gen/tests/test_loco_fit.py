"""実機の 1 回分の記録の扱い: 柱の面の変化から動いた量を出す（common/column_face.py の motion_between）と、記録のまとめ（real/loco_fit.py）。"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.column_face import ColumnFace, motion_between
from common.config import FEATURE_DIR



def face(d: float, yaw_deg: float, left: float | None, right: float | None) -> ColumnFace:
    psi = np.radians(yaw_deg)
    n = -np.array([np.cos(psi), np.sin(psi), 0.0])
    return ColumnFace(normal=n, point=-d * n, distance_m=d, yaw_rad=psi, left_edge_m=left, right_edge_m=right,
                      n_points=1000, floor_found=True, pitch_correction_deg=0.0, floor_z_m=-0.73)


class TestMotionFromFaces(unittest.TestCase):
    def test_forward_left_turn(self) -> None:
        """正対した面の前で、前 20 cm・左 5 cm 動いて左に 5° 回ると、面は 80 cm・向き −5°・端は 5 cm 右に見える。"""
        m = motion_between(face(1.0, 0.0, 0.25, -0.20), face(0.80, -5.0, 0.20, -0.25))
        self.assertAlmostEqual(m["forward_cm"], 20.0, places=6)
        self.assertAlmostEqual(m["left_cm"], 5.0, places=6)
        self.assertAlmostEqual(m["turn_deg"], 5.0, places=6)

    def test_turned_face(self) -> None:
        """面に 10° 斜めに立っていても、体の座標での動きになる（体の前へ 20 cm 進んだ）。"""
        psi = np.radians(10.0)
        f, t = np.array([np.cos(psi), np.sin(psi)]), np.array([-np.sin(psi), np.cos(psi)])
        move = np.array([0.20, 0.0])
        m = motion_between(face(1.0, 10.0, 0.3, -0.2), face(1.0 - f @ move, 10.0, 0.3 - t @ move, -0.2 - t @ move))
        self.assertAlmostEqual(m["forward_cm"], 20.0, places=6)
        self.assertAlmostEqual(m["left_cm"], 0.0, places=6)

    def test_no_edges(self) -> None:
        m = motion_between(face(1.0, 0.0, None, None), face(0.8, 0.0, None, None))
        self.assertIsNone(m["left_cm"])
        self.assertAlmostEqual(m["forward_cm"], 20.0, places=6)  # 面に向かう向きの量


class TestLocoFit(unittest.TestCase):
    def test_suggests_rate_and_min_duration(self) -> None:
        """前 0.3 m/s: 0.4 秒は歩き出さず（1 cm）、0.6・1.0 秒は 1 秒あたり約 23 cm。案に rate と min_duration_s が出る。"""
        with tempfile.TemporaryDirectory() as tmp:
            k = 0
            for dur, moved_cm, side_cm in ((0.4, 1.0, 0.0), (0.4, 0.5, 0.0), (0.6, 14.0, -5.0), (0.6, 13.5, -4.5),
                                           (1.0, 23.0, -8.0), (1.0, 22.6, -7.5)):
                d = Path(tmp) / f"20261113_1000{k:02d}"
                d.mkdir()
                k += 1
                (d / "pulse.json").write_text(json.dumps({
                    "pulse": {"vx": 0.3, "vy": 0.0, "wz": 0.0, "duration_s": dur, "axis": "raw", "amount": 0.0},
                    "stopped": True, "settle_s": dur + 1.9,
                    "measured": {"forward_cm": moved_cm, "left_cm": side_cm, "turn_deg": None},
                    "measured_by_camera": None}))
            r = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "loco_fit.py"), "--dir", tmp],
                               capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("0.40 秒", r.stdout)
        self.assertIn("歩き出さないとみなす", r.stdout)
        self.assertIn("forward: rate 0.22", r.stdout)  # 歩き出さない 0.4 秒は当てはめに入れない
        self.assertIn("min_duration_s 0.6", r.stdout)
        self.assertIn("進む向きのずれ", r.stdout)

    def test_no_records(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            r = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "loco_fit.py"), "--dir", tmp],
                               capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 1)


if __name__ == "__main__":
    unittest.main()
