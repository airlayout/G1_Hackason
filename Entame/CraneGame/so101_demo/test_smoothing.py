import math
import random
import unittest

import smoothing


def noisy_ramp(n=200, dt=0.1, noise=0.05, seed=1):
    rng = random.Random(seed)
    times = [i * dt for i in range(n)]
    clean = [min(1.0, t / 10.0) for t in times]
    return times, clean, [c + rng.uniform(-noise, noise) for c in clean]


class OneEuroTests(unittest.TestCase):
    def test_reduces_jitter_and_stays_close(self):
        times, clean, noisy = noisy_ramp()
        out = smoothing.smooth_series(noisy, times)
        self.assertLess(smoothing.jitter(out)[0], smoothing.jitter(noisy)[0] * 0.5)
        self.assertLess(max(abs(a - b) for a, b in zip(out, clean)), 0.12)

    def test_first_value_passes_through_and_constant_stays(self):
        self.assertEqual(smoothing.smooth_series([0.7, 0.7, 0.7], [0, 0.1, 0.2]), [0.7, 0.7, 0.7])

    def test_fast_step_is_followed(self):
        times = [i * 0.05 for i in range(40)]
        out = smoothing.smooth_series([0.0] * 10 + [1.0] * 30, times)
        self.assertGreater(out[-1], 0.97)


class CriticallyDampedTests(unittest.TestCase):
    def test_converges_without_overshoot(self):
        f, xs = smoothing.CriticallyDamped(0.08), []
        f(0.0, 0.0)
        for _ in range(60):
            xs.append(f(1.0, 0.05))
        self.assertLessEqual(max(xs), 1.0 + 1e-6)
        self.assertGreater(xs[-1], 0.99)

    def test_stable_with_large_dt(self):
        f = smoothing.CriticallyDamped(0.08)
        f(0.0, 0.0)
        for _ in range(10):
            x = f(1.0, 0.25)
        self.assertTrue(math.isfinite(x))
        self.assertAlmostEqual(x, 1.0, places=2)

    def test_removes_direction_flips(self):
        f, xs = smoothing.CriticallyDamped(0.08), []
        for i in range(60):
            xs.append(f(10.0 + (0.4 if i % 2 else -0.4), 0.09))
        self.assertLess(max(xs[30:]) - min(xs[30:]), 0.8 * 0.6)       # 入力の振れ 0.8 を 6 割未満にする（実測 約 0.46）


class InterpolateTests(unittest.TestCase):
    def test_midpoint_and_ends(self):
        a, b = {"x": 0.0, "y": 10.0}, {"x": 2.0, "y": 20.0}
        self.assertEqual(smoothing.interpolate(a, b, 0.5), {"x": 1.0, "y": 15.0})
        self.assertEqual(smoothing.interpolate(a, b, 1.0), b)

    def test_missing_previous_key_uses_current(self):
        self.assertEqual(smoothing.interpolate({}, {"x": 3.0}, 0.2), {"x": 3.0})


class JitterTests(unittest.TestCase):
    def test_straight_line_has_no_jitter(self):
        self.assertEqual(smoothing.jitter([0, 1, 2, 3, 4]), (0.0, 0))

    def test_zigzag_is_counted(self):
        rms, flips = smoothing.jitter([0, 0.02, 0, 0.02, 0, 0.02], noise=0.05)
        self.assertGreater(rms, 0)
        self.assertEqual(flips, 4)

    def test_short_series(self):
        self.assertEqual(smoothing.jitter([1, 2]), (0.0, 0))


if __name__ == "__main__":
    unittest.main()
