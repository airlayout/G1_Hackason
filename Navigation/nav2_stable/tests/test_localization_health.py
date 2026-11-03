"""localization_health.LocalizationHealth の判定規則のテスト。"""

import math
import unittest

from localization_health import LOST, OK, UNKNOWN, LocalizationHealth


def feed(h: LocalizationHealth, values, t0: float = 0.0, dt: float = 0.5, yaw_step: float = 0.0):
    """2Hz で inlier 率を流し込み、最後の状態を返す。yaw_step は 1 回あたりの回転[rad]。"""
    t, yaw, state = t0, 0.0, h.state
    for v in values:
        state = h.update(t, yaw, v)
        t += dt
        yaw += yaw_step
    return state


class TestLocalizationHealth(unittest.TestCase):
    def test_unknown_until_window_full(self):
        h = LocalizationHealth(window=10)
        self.assertEqual(feed(h, [0.9] * 9), UNKNOWN)
        self.assertEqual(feed(h, [0.9], t0=10.0), OK)

    def test_lost_when_average_below_threshold(self):
        h = LocalizationHealth(window=10)
        feed(h, [0.9] * 10)
        self.assertEqual(h.state, OK)
        # 窓の平均が 0.70 を切るまでは OK のまま
        self.assertEqual(feed(h, [0.6] * 5, t0=10.0), OK)   # 平均 0.75
        self.assertEqual(feed(h, [0.6] * 3, t0=20.0), LOST)  # 平均 0.69

    def test_hysteresis_between_70_and_75(self):
        h = LocalizationHealth(window=10)
        feed(h, [0.6] * 10)
        self.assertEqual(h.state, LOST)
        # 0.72 は 0.70 以上だが 0.75 未満なので LOST のまま
        self.assertEqual(feed(h, [0.72] * 10, t0=10.0), LOST)
        self.assertEqual(feed(h, [0.8] * 10, t0=20.0), OK)

    def test_short_dip_does_not_trigger(self):
        """11/01 の巡回で見た「数フレームだけ 57〜65%」は 5 秒窓なら lost にならない。"""
        h = LocalizationHealth(window=10)
        feed(h, [0.92] * 10)
        self.assertEqual(feed(h, [0.65, 0.64, 0.57, 0.59, 0.69, 0.74, 0.93], t0=10.0), OK)

    def test_spinning_frames_are_ignored(self):
        h = LocalizationHealth(window=10, max_yaw_rate_deg=20.0)
        feed(h, [0.9] * 10)
        # 52°/s の旋回中（0.5 秒で 26°）に inlier が落ちても判定に使わない
        self.assertEqual(feed(h, [0.3] * 20, t0=10.0, yaw_step=math.radians(26)), OK)
        self.assertTrue(h.spinning)

    def test_slow_turn_is_judged(self):
        h = LocalizationHealth(window=10, max_yaw_rate_deg=20.0)
        feed(h, [0.9] * 10)
        # 10°/s（0.5 秒で 5°）は判定に使う
        self.assertEqual(feed(h, [0.3] * 10, t0=10.0, yaw_step=math.radians(5)), LOST)

    def test_yaw_wraparound_is_not_a_spin(self):
        h = LocalizationHealth(window=2, max_yaw_rate_deg=20.0)
        h.update(0.0, math.radians(179), 0.9)
        h.update(0.5, math.radians(-179), 0.9)   # 実際は 2° しか回っていない
        self.assertFalse(h.spinning)

    def test_reset_returns_to_unknown(self):
        h = LocalizationHealth(window=10)
        feed(h, [0.5] * 10)
        self.assertEqual(h.state, LOST)
        h.reset()
        self.assertEqual(h.state, UNKNOWN)
        self.assertIsNone(h.average)

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            LocalizationHealth(lost_below=0.8, ok_at=0.75)


if __name__ == "__main__":
    unittest.main()
