"""安全チェック（common/arm/safety.py）の単体テスト（タスク1）。"""

from __future__ import annotations

import unittest

import numpy as np

from common.arm.safety import (
    UnsafeTargetError,
    WorkspaceBox,
    check_finite,
    check_joint_limits,
    interpolate_joint,
    rate_limit,
    smoothstep,
)


class TestSafety(unittest.TestCase):
    def test_rate_limit_caps_each_joint(self) -> None:
        q = rate_limit(np.zeros(3), np.array([1.0, -1.0, 0.001]), 0.01)
        np.testing.assert_allclose(q, [0.01, -0.01, 0.001])

    def test_smoothstep_endpoints_and_zero_velocity(self) -> None:
        self.assertEqual(smoothstep(0.0), 0.0)
        self.assertEqual(smoothstep(1.0), 1.0)
        eps = 1e-6
        self.assertLess(abs(smoothstep(eps) - smoothstep(0.0)) / eps, 1e-4)
        self.assertLess(abs(smoothstep(1.0) - smoothstep(1.0 - eps)) / eps, 1e-4)

    def test_interpolate_ends_at_target(self) -> None:
        wps = interpolate_joint(np.zeros(2), np.array([1.0, -0.5]), 1.0, 0.02)
        self.assertEqual(len(wps), 50)
        np.testing.assert_allclose(wps[-1], [1.0, -0.5])
        # 単調に近づく
        self.assertTrue(np.all(np.diff(wps[:, 0]) >= 0))

    def test_joint_limits_reject_with_margin(self) -> None:
        lo, hi = np.array([-1.0, -1.0]), np.array([1.0, 1.0])
        check_joint_limits(np.array([0.9, -0.9]), lo, hi, 0.05, ["a", "b"])
        with self.assertRaises(UnsafeTargetError):
            check_joint_limits(np.array([0.97, 0.0]), lo, hi, 0.05, ["a", "b"])

    def test_nan_rejected(self) -> None:
        with self.assertRaises(UnsafeTargetError):
            check_finite(np.array([0.0, np.nan]))

    def test_workspace_box(self) -> None:
        box = WorkspaceBox.from_config({"min": [0, -1, -1], "max": [1, 1, 1]})
        box.check(np.array([0.5, 0.0, 0.0]))
        with self.assertRaises(UnsafeTargetError):
            box.check(np.array([-0.1, 0.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
