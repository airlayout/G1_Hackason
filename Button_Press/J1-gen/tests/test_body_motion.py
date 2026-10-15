"""押している間の体（pelvis）の動きを、脚の関節角から求める（common/body_motion.py）。"""

from __future__ import annotations

import unittest

import numpy as np

from _env import needs
from common.body_motion import analyze, gravity_in_body
from common.config import load_config

# 脚の関節（motor 番号）: 左 0〜5、右 6〜11。並びは hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
HIP_PITCH, HIP_ROLL, KNEE, ANKLE_PITCH = 0, 1, 3, 4


def crouch(a: float) -> np.ndarray:
    """両脚を同じだけ曲げる（股・膝・足首で打ち消し、pelvis の向きは変えずに下がる）。"""
    q = np.zeros(29)
    for base in (0, 6):
        q[base + HIP_PITCH], q[base + KNEE], q[base + ANKLE_PITCH] = -a, 2 * a, -a
    return q


def quat_about_y(deg: float) -> np.ndarray:
    h = np.radians(deg) / 2
    return np.array([np.cos(h), 0.0, np.sin(h), 0.0])


@needs('pin')
class TestBodyMotion(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from common.body_motion import LegKinematics

        cls.legs = LegKinematics(load_config("robot.yaml"))

    def run_rows(self, qs: list[np.ndarray], imus: list[np.ndarray] | None = None, point=None):  # type: ignore[no-untyped-def]
        imus = imus or [np.array([1.0, 0, 0, 0])] * len(qs)
        return analyze(self.legs, np.arange(len(qs), dtype=float), np.array(qs), np.array(imus), ref=0,
                       point=None if point is None else np.asarray(point))

    def test_still_is_zero(self) -> None:
        m = self.run_rows([crouch(0.2)] * 3, point=[0.35, -0.2, 0.0])
        np.testing.assert_allclose(m.move_mm, 0, atol=1e-6)
        np.testing.assert_allclose(m.aim_mm, 0, atol=1e-6)
        np.testing.assert_allclose(m.feet_mm, 0, atol=1e-6)

    def test_crouch_lowers_pelvis_and_target_looks_higher(self) -> None:
        m = self.run_rows([crouch(0.1), crouch(0.3)], point=[0.35, -0.2, 0.0])
        self.assertLess(m.move_mm[1, 2], -5.0)  # 下がる
        self.assertLess(np.linalg.norm(m.rot_deg[1]), 0.01)  # 向きは変わらない
        # 部屋に固定された対象は、下がった pelvis から見ると同じだけ上に見える
        np.testing.assert_allclose(m.aim_mm[1], -m.move_mm[1], atol=1e-6)
        self.assertLess(m.feet_mm[1], 1e-6)  # 両足の位置関係は同じ

    def test_lean_moves_aim_more_than_pelvis(self) -> None:
        """足首で前に傾くと、遠い対象ほど大きくずれて見える（回転の分が距離に比例する）。"""
        q1 = np.zeros(29)
        q1[ANKLE_PITCH] = q1[6 + ANKLE_PITCH] = 0.03
        m = self.run_rows([np.zeros(29), q1], point=[0.40, -0.2, 0.0])
        self.assertGreater(np.linalg.norm(m.rot_deg[1]), 1.0)
        self.assertGreater(np.linalg.norm(m.aim_mm[1]), np.linalg.norm(m.move_mm[1]))

    def test_one_leg_moved_is_a_step(self) -> None:
        q1 = np.zeros(29)
        q1[HIP_ROLL] = 0.1  # 左脚だけ開く
        m = self.run_rows([np.zeros(29), q1])
        self.assertGreater(m.feet_mm[1], 10.0)
        self.assertIn("足を踏み替えた", m.text())

    def test_imu_tilt(self) -> None:
        m = self.run_rows([np.zeros(29)] * 2, imus=[quat_about_y(0.0), quat_about_y(2.0)])
        self.assertAlmostEqual(m.imu_tilt_deg[1], 2.0, places=4)
        np.testing.assert_allclose(gravity_in_body(np.array([1.0, 0, 0, 0])), [0, 0, -1])


if __name__ == "__main__":
    unittest.main()
