"""押す動作を動画にする（sim/record_press_video.py）の部品。"""

from __future__ import annotations

import io
import sys
import unittest

from common.config import FEATURE_DIR

sys.path.insert(0, str(FEATURE_DIR / "sim"))
import record_press_video as rv  # noqa: E402


class TestStepTee(unittest.TestCase):
    def test_passes_output_through_and_tracks_arm_step(self) -> None:
        inner = io.StringIO()
        tee = rv.StepTee(inner)
        tee.write("[pipeline] 計画\n")
        self.assertEqual(tee.step, "")
        tee.write("[arm] 経由の姿勢へ移動: 最大 120.4° を 12.6 秒で\n[arm] dry-run なので\n")
        self.assertEqual(tee.step, "to via pose")
        tee.write("[arm] 押し込み（1 回目、深さ 15.0 mm）\n")
        self.assertEqual(tee.step, "pressing")
        # [arm] 以外の行は段階を変えない
        tee.write("[press] 経由の姿勢へ移動\n")
        self.assertEqual(tee.step, "pressing")
        self.assertIn("[pipeline] 計画", inner.getvalue())

    def test_return_steps_are_not_taken_for_forward_steps(self) -> None:
        tee = rv.StepTee(io.StringIO())
        tee.write("[arm] 経由の姿勢へ戻る: 最大 65.7°\n")
        self.assertEqual(tee.step, "back to via pose")
        tee.write("[arm] 手前の姿勢へ戻る\n")
        self.assertEqual(tee.step, "back to approach pose")


if __name__ == "__main__":
    unittest.main()
