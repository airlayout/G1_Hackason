"""ボトルを押せる範囲を広げる設定（configs/reach_wide.yaml）を、MuJoCo（ボトル・机あり、物理あり）で確かめる（2026-10-29）。

今の設定では、机の縁が近い（実機と同じ深い机の箱で 32 cm 未満）、またはボトルが体の正面寄り（右 10 cm より内側）だと、計画の段階で断られる。
広げた設定（経由の姿勢と、指を左へ傾ける候補）なら通り、押し込みで指先がボトルの面まで届き、腕が机に当たらない。
"""

from __future__ import annotations

import sys
import unittest

from _env import needs
from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "sim"))
import reach_verify as rv  # noqa: E402


@needs("mujoco", "pin")
class TestReachWide(unittest.TestCase):
    def test_narrow_desk_edge_is_rejected_now_and_passes_with_reach_wide(self) -> None:
        now = rv.run_case(0.26, -0.15, 0.02, None, "t_now")
        self.assertEqual(now["code"], 2, now["message"])            # 目標を拒否した（何も送っていない）
        wide = rv.run_case(0.26, -0.15, 0.02, "reach_wide.yaml", "t_wide")
        self.assertEqual(wide["code"], 0, wide["message"])
        self.assertGreater(wide["tip_advance_mm"], 45.0)             # 指先が面（手前 50 mm）まで届く
        self.assertNotIn("arm-desk", wide["force_N"])                # 腕が机に当たらない
        self.assertLess(wide["force_N"].get("arm-bottle", 0.0), 10.0)

    def test_bottle_near_the_body_center_passes_with_a_tilted_finger(self) -> None:
        now = rv.run_case(0.30, -0.05, 0.02, None, "t_now2")
        self.assertEqual(now["code"], 2, now["message"])
        wide = rv.run_case(0.30, -0.05, 0.02, "reach_wide.yaml", "t_wide2")
        self.assertEqual(wide["code"], 0, wide["message"])
        self.assertIn("指 左", wide["plan"])                          # 指を左へ傾けた候補で通った
        self.assertGreater(wide["tip_advance_mm"], 40.0)
        self.assertLess(wide["force_N"].get("arm-bottle", 0.0), 10.0)


if __name__ == "__main__":
    unittest.main()
