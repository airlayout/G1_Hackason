"""`nav/route.py`: ウェイポイント列 -> 1102 に投げる直線区間の列。

探索そのものは skimage に任せてあるので、ここで固定するのは
**1102 固有の決め事**:

- 1 区間を 8m 以内に刻む（公式上限 10m に 2m の余裕）
- 刻みの基準がずれない
- 壁を貫く区間を作らない
- 利用者が指定したウェイポイントの向きを尊重する
"""

import math
import unittest

import numpy as np

from nav.occupancy import build_grid
from nav.protocol import Pose2D
from nav.route import (
    HARD_LIMIT_M,
    MAX_SEGMENT_M,
    MIN_SEGMENT_M,
    RouteError,
    plan_route,
)
from tests.test_occupancy import box_cloud


def open_grid(x0=-15.0, x1=15.0, y0=-15.0, y1=15.0):
    """障害物の無い広い部屋。分割そのものを見るときに使う。"""

    return build_grid(box_cloud(x0, x1, y0, y1), inflation=0.0)


def split_room():
    """真ん中の壁に開口 y ∈ [0.4, 1.0] が 1 つだけある部屋。"""

    cloud = box_cloud(-5, 5, -5, 5)
    wall = np.array(
        [[0.0, y, 1.0] for y in np.arange(-4.9, 0.4, 0.02)]
        + [[0.0, y, 1.0] for y in np.arange(1.0, 4.92, 0.02)]
    )
    return build_grid(np.concatenate([cloud, wall]), inflation=0.0)


class SplittingTest(unittest.TestCase):
    def setUp(self):
        self.grid = open_grid()

    def test_a_short_straight_run_is_one_segment(self):
        segments = plan_route(self.grid, [Pose2D(-1, 0), Pose2D(2, 0)])
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0].length, 3.0)

    def test_a_long_straight_run_is_split(self):
        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)])
        self.assertEqual(len(segments), 3)  # 20m -> 3 x 6.67m

    def test_no_segment_exceeds_the_configured_limit(self):
        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)])
        for segment in segments:
            self.assertLessEqual(segment.length, MAX_SEGMENT_M + 1e-9)

    def test_no_segment_exceeds_the_official_hard_limit(self):
        segments = plan_route(self.grid, [Pose2D(-14, -14), Pose2D(14, 14)], max_segment_m=9.9)
        for segment in segments:
            self.assertLessEqual(segment.length, HARD_LIMIT_M)

    def test_the_pieces_are_equal_length(self):
        """基準を区間ごとに動かすと 2 本目以降がずれる。始点基準で一度だけ刻む。"""

        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)])
        lengths = [round(segment.length, 6) for segment in segments]
        self.assertEqual(len(set(lengths)), 1, lengths)

    def test_the_segments_are_chained_end_to_start(self):
        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0), Pose2D(10, 10)])
        for previous, following in zip(segments, segments[1:]):
            self.assertEqual((previous.target.x, previous.target.y),
                             (following.start.x, following.start.y))

    def test_the_indices_run_from_zero_without_gaps(self):
        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0), Pose2D(10, 10)])
        self.assertEqual([segment.index for segment in segments], list(range(len(segments))))

    def test_a_smaller_limit_makes_more_segments(self):
        few = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)], max_segment_m=8.0)
        many = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)], max_segment_m=2.0)
        self.assertGreater(len(many), len(few))

    def test_the_last_segment_of_a_leg_is_flagged_as_a_waypoint(self):
        segments = plan_route(self.grid, [Pose2D(-10, 0), Pose2D(10, 0)])
        self.assertTrue(segments[-1].is_waypoint)
        self.assertFalse(any(segment.is_waypoint for segment in segments[:-1]))

    def test_every_waypoint_gets_exactly_one_flagged_segment(self):
        waypoints = [Pose2D(-10, 0), Pose2D(0, 0), Pose2D(10, 0), Pose2D(10, 10)]
        segments = plan_route(self.grid, waypoints)
        self.assertEqual(sum(segment.is_waypoint for segment in segments), len(waypoints) - 1)


class HeadingTest(unittest.TestCase):
    def setUp(self):
        self.grid = open_grid()

    def test_intermediate_targets_face_the_direction_of_travel(self):
        segments = plan_route(self.grid, [Pose2D(0, 0), Pose2D(10, 0)])
        self.assertAlmostEqual(segments[0].target.yaw, 0.0)

    def test_the_users_yaw_at_a_waypoint_is_kept(self):
        """そこで何かをする向きが決まっていることがあるので上書きしない。"""

        segments = plan_route(self.grid, [Pose2D(0, 0), Pose2D(5, 0, math.pi / 2)])
        self.assertAlmostEqual(segments[-1].target.yaw, math.pi / 2)

    def test_a_turn_in_place_at_a_waypoint_is_still_commanded(self):
        """1102 は位置だけでなく姿勢も指定する。向き直しだけの区間も投げる。"""

        segments = plan_route(self.grid, [Pose2D(0, 0), Pose2D(0.0, 0.0, math.pi / 2)])
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0].target.yaw, math.pi / 2)

    def test_a_negligible_move_and_turn_produces_nothing(self):
        segments = plan_route(
            self.grid, [Pose2D(0, 0), Pose2D(MIN_SEGMENT_M / 2, 0.0, 0.001)]
        )
        self.assertEqual(segments, [])


class ObstacleTest(unittest.TestCase):
    def setUp(self):
        self.grid = split_room()

    def test_the_route_goes_through_the_opening(self):
        segments = plan_route(self.grid, [Pose2D(-3, -3), Pose2D(3, -3)])
        self.assertGreater(len(segments), 1)  # 直線では行けない
        crossing = [s for s in segments if min(s.start.x, s.target.x) < 0 < max(s.start.x, s.target.x)]
        self.assertTrue(crossing)
        for segment in crossing:
            self.assertTrue(0.3 < segment.start.y < 1.1 or 0.3 < segment.target.y < 1.1)

    def test_no_segment_crosses_a_wall(self):
        segments = plan_route(self.grid, [Pose2D(-3, -3), Pose2D(3, -3), Pose2D(3, 3), Pose2D(-3, 3)])
        for segment in segments:
            self.assertTrue(
                self.grid.is_segment_free(
                    segment.start.x, segment.start.y, segment.target.x, segment.target.y
                ),
                f"区間{segment.index}が壁を貫いている",
            )

    def test_the_bends_are_pulled_straight(self):
        """格子の経路をそのまま出すと 0.1m 刻みの 1102 が何百回も飛ぶ。"""

        segments = plan_route(self.grid, [Pose2D(-3, -3), Pose2D(3, -3)])
        self.assertLess(len(segments), 10)

    def test_an_unreachable_goal_says_so(self):
        cloud = box_cloud(-5, 5, -5, 5)
        wall = np.array([[0.0, y, 1.0] for y in np.arange(-4.95, 4.96, 0.02)])
        sealed = build_grid(np.concatenate([cloud, wall]), inflation=0.0)
        with self.assertRaises(RouteError) as caught:
            plan_route(sealed, [Pose2D(-3, 0), Pose2D(3, 0)])
        self.assertIn("たどり着ける経路が地図上に無い", str(caught.exception))


class RejectionTest(unittest.TestCase):
    def setUp(self):
        self.grid = open_grid()

    def test_a_single_waypoint_is_rejected(self):
        with self.assertRaises(RouteError):
            plan_route(self.grid, [Pose2D(0, 0)])

    def test_a_waypoint_inside_an_obstacle_is_rejected_with_its_index(self):
        grid = split_room()
        with self.assertRaises(RouteError) as caught:
            plan_route(grid, [Pose2D(-3, -3), Pose2D(0.0, -3.0)])
        self.assertIn("1番目", str(caught.exception))

    def test_a_waypoint_outside_the_map_is_rejected(self):
        with self.assertRaises(RouteError):
            plan_route(self.grid, [Pose2D(0, 0), Pose2D(100.0, 100.0)])

    def test_a_tiny_segment_limit_is_rejected(self):
        with self.assertRaises(RouteError):
            plan_route(self.grid, [Pose2D(0, 0), Pose2D(1, 0)], max_segment_m=MIN_SEGMENT_M)


if __name__ == "__main__":
    unittest.main()
