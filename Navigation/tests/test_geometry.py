from __future__ import annotations

import math
import unittest

from nav.geometry import (
    Pose2D,
    interpolate,
    normalize_angle,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class NormalizeAngleTest(unittest.TestCase):
    def test_keeps_angles_already_in_range(self):
        for angle in (0.0, 1.0, -1.0, math.pi, -math.pi + 1e-9):
            self.assertAlmostEqual(normalize_angle(angle), angle, places=9)

    def test_folds_angles_beyond_one_turn(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(normalize_angle(2.0 * math.pi + 0.5), 0.5, places=9)

    def test_maps_negative_pi_to_positive_pi(self):
        # 区間を (-pi, pi] に取ると -pi は境界の外側になる
        self.assertAlmostEqual(normalize_angle(-math.pi), math.pi, places=9)


class QuaternionTest(unittest.TestCase):
    def test_round_trips_yaw(self):
        for yaw in (0.0, 0.5, -0.5, 1.5707963, 3.0, -3.0):
            self.assertAlmostEqual(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw, places=9)

    def test_identity_quaternion_is_zero_yaw(self):
        self.assertAlmostEqual(quaternion_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0, places=9)

    def test_accepts_unnormalized_quaternion(self):
        # 実機のJSONをそのまま食わせるので、正規化されていない値も受ける
        q_x, q_y, q_z, q_w = yaw_to_quaternion(1.0)
        scaled = (q_x * 3.0, q_y * 3.0, q_z * 3.0, q_w * 3.0)
        self.assertAlmostEqual(quaternion_to_yaw(*scaled), 1.0, places=9)

    def test_rejects_zero_quaternion(self):
        with self.assertRaises(ValueError):
            quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)

    def test_ignores_roll_and_pitch(self):
        # roll 90度だけの回転は yaw 0 として読む（平面移動しか扱わないため）
        half = math.pi / 4.0
        self.assertAlmostEqual(
            quaternion_to_yaw(math.sin(half), 0.0, 0.0, math.cos(half)), 0.0, places=9
        )


class Pose2DTest(unittest.TestCase):
    def test_distance(self):
        self.assertAlmostEqual(Pose2D(0.0, 0.0).distance_to(Pose2D(3.0, 4.0)), 5.0, places=9)

    def test_heading(self):
        self.assertAlmostEqual(
            Pose2D(0.0, 0.0).heading_to(Pose2D(0.0, 2.0)), math.pi / 2.0, places=9
        )

    def test_heading_to_same_point_keeps_own_yaw(self):
        pose = Pose2D(1.0, 1.0, 0.7)
        self.assertAlmostEqual(pose.heading_to(Pose2D(1.0, 1.0, 0.0)), 0.7, places=9)


class InterpolateTest(unittest.TestCase):
    def test_midpoint(self):
        point = interpolate(Pose2D(0.0, 0.0), Pose2D(10.0, 0.0), 0.5)
        self.assertAlmostEqual(point.x, 5.0, places=9)
        self.assertAlmostEqual(point.y, 0.0, places=9)

    def test_faces_travel_direction(self):
        point = interpolate(Pose2D(0.0, 0.0), Pose2D(0.0, 5.0), 0.5)
        self.assertAlmostEqual(point.yaw, math.pi / 2.0, places=9)

    def test_clamps_ratio_outside_the_segment(self):
        start, end = Pose2D(0.0, 0.0), Pose2D(4.0, 0.0)
        self.assertAlmostEqual(interpolate(start, end, 1.5).x, 4.0, places=9)
        self.assertAlmostEqual(interpolate(start, end, -0.5).x, 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
