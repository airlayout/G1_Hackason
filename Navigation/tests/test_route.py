from __future__ import annotations

import math
import unittest
from functools import lru_cache

from nav.geometry import Pose2D
from nav.occupancy import build_grid
from nav.route import HARD_LIMIT_M, MAX_SEGMENT_M, RouteError, plan_route

# 壁の点をどれくらい細かく置くか[m]。格子解像度より細かくしないと穴が空く。
WALL_STEP_M = 0.05

# 通り抜けられる開口の幅[m]。膨張半径0.4mの2倍(0.8m)より広くないと通れない。
DOORWAY_M = 1.4


def _line(x0, y0, x1, y1, z=1.0):
    """(x0,y0)-(x1,y1) を WALL_STEP_M 刻みで埋めた点列。"""

    length = math.hypot(x1 - x0, y1 - y0)
    count = max(1, int(length / WALL_STEP_M))
    return [
        (x0 + (x1 - x0) * index / count, y0 + (y1 - y0) * index / count, z)
        for index in range(count + 1)
    ]


def _floor(x0, y0, x1, y1, step):
    """床の点(z=0)。障害物にはならないが「観測済み」の証拠になる。

    これが無いと占有格子は全面を未観測=通行不可とみなす。
    実際の地図（`sim_room.pcd` は床に151万点）と同じ条件を作るためのもの。
    """

    cols = int((x1 - x0) / step) + 1
    rows = int((y1 - y0) / step) + 1
    return [
        (x0 + col * step, y0 + row * step, 0.0)
        for row in range(rows)
        for col in range(cols)
    ]


@lru_cache(maxsize=None)
def open_field(half_size: float = 30.0):
    """障害物のない広い平面。床だけがある。"""

    resolution = 0.2
    return build_grid(
        _floor(-half_size, -half_size, half_size, half_size, resolution * 0.9),
        resolution=resolution,
        inflation=0.0,
        margin=1.0,
    )


@lru_cache(maxsize=None)
def room_with_doorway():
    """12m四方の閉じた部屋を x=0 の仕切り壁で2分し、y=0 に開口を1つだけ空ける。

    開口が1箇所しかないので、迂回路が「たまたま別の抜け道を通った」ことが起きない。
    """

    walls = (
        _line(-6.0, -6.0, 6.0, -6.0) + _line(-6.0, 6.0, 6.0, 6.0)
        + _line(-6.0, -6.0, -6.0, 6.0) + _line(6.0, -6.0, 6.0, 6.0)
        + _line(0.0, -6.0, 0.0, -DOORWAY_M / 2.0)
        + _line(0.0, DOORWAY_M / 2.0, 0.0, 6.0)
    )
    return build_grid(
        walls + _floor(-6.0, -6.0, 6.0, 6.0, 0.09),
        resolution=0.1,
        inflation=0.4,
        margin=1.0,
    )


class SplittingTest(unittest.TestCase):
    def test_short_leg_is_a_single_segment(self):
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(3.0, 0.0)])
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0].length, 3.0, places=6)

    def test_long_leg_is_split_under_the_limit(self):
        segments = plan_route(open_field(), [Pose2D(-12.0, 0.0), Pose2D(12.0, 0.0)])
        self.assertGreater(len(segments), 1)
        for segment in segments:
            self.assertLessEqual(segment.length, MAX_SEGMENT_M + 1e-9)
            self.assertLessEqual(segment.length, HARD_LIMIT_M)

    def test_split_pieces_are_evenly_spaced_from_the_edge_start(self):
        """刻みの基準がずれると2区間目以降の目標点が狂う。ここで固定する。"""

        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(24.0, 0.0)])
        self.assertEqual(len(segments), 3)  # 24m / 8m
        self.assertAlmostEqual(segments[0].target.x, 8.0, places=6)
        self.assertAlmostEqual(segments[1].target.x, 16.0, places=6)
        self.assertAlmostEqual(segments[2].target.x, 24.0, places=6)

    def test_segments_are_chained(self):
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(20.0, 0.0)])
        for previous, current in zip(segments, segments[1:]):
            self.assertEqual(previous.target, current.start)

    def test_indices_are_sequential_across_legs(self):
        segments = plan_route(
            open_field(), [Pose2D(0.0, 0.0), Pose2D(10.0, 0.0), Pose2D(10.0, 10.0)]
        )
        self.assertEqual([s.index for s in segments], list(range(len(segments))))

    def test_custom_segment_length_is_honoured(self):
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(9.0, 0.0)],
                              max_segment_m=3.0)
        self.assertEqual(len(segments), 3)


class WaypointTest(unittest.TestCase):
    def test_only_waypoint_segments_are_flagged(self):
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(20.0, 0.0)])
        self.assertTrue(segments[-1].is_waypoint)
        self.assertFalse(any(s.is_waypoint for s in segments[:-1]))

    def test_each_waypoint_produces_one_flagged_segment(self):
        route = [Pose2D(0.0, 0.0), Pose2D(10.0, 0.0), Pose2D(10.0, 10.0), Pose2D(0.0, 0.0)]
        segments = plan_route(open_field(), route)
        self.assertEqual(sum(1 for s in segments if s.is_waypoint), 3)

    def test_requested_yaw_is_kept_at_the_waypoint(self):
        goal = Pose2D(10.0, 0.0, math.pi)
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), goal])
        self.assertAlmostEqual(segments[-1].target.yaw, math.pi, places=9)

    def test_intermediate_points_face_the_travel_direction(self):
        segments = plan_route(open_field(), [Pose2D(0.0, 0.0), Pose2D(0.0, 20.0)])
        self.assertAlmostEqual(segments[0].target.yaw, math.pi / 2.0, places=9)

    def test_rotation_only_waypoint_still_gets_a_command(self):
        """同じ場所で向きだけ変える指示も1102として成立する。"""

        segments = plan_route(
            open_field(), [Pose2D(0.0, 0.0, 0.0), Pose2D(5.0, 0.0), Pose2D(5.0, 0.0, math.pi)]
        )
        self.assertTrue(segments[-1].is_waypoint)
        self.assertAlmostEqual(segments[-1].target.yaw, math.pi, places=9)


class ObstacleAvoidanceTest(unittest.TestCase):
    """1102は直線でしか歩かないので、壁を貫く区間を作ってはいけない。"""

    def setUp(self):
        self.grid = room_with_doorway()
        self.route = [Pose2D(-4.0, 4.0), Pose2D(4.0, 4.0)]

    def test_no_segment_crosses_an_obstacle(self):
        for segment in plan_route(self.grid, self.route):
            self.assertTrue(
                self.grid.is_segment_free(
                    segment.start.x, segment.start.y, segment.target.x, segment.target.y
                ),
                f"区間{segment.index}が障害物を貫いている",
            )

    def test_route_passes_through_the_only_doorway(self):
        segments = plan_route(self.grid, self.route)
        crossings = [
            segment
            for segment in segments
            if (segment.start.x < 0.0) != (segment.target.x < 0.0)
        ]
        self.assertEqual(len(crossings), 1, "仕切り壁は1回だけ越えるはず")
        crossing = crossings[0]
        # 越える瞬間のyが開口の中に収まっていること
        ratio = (0.0 - crossing.start.x) / (crossing.target.x - crossing.start.x)
        y_at_wall = crossing.start.y + (crossing.target.y - crossing.start.y) * ratio
        self.assertLessEqual(abs(y_at_wall), DOORWAY_M / 2.0)

    def test_shortcut_keeps_the_number_of_segments_small(self):
        """A*の格子経路をそのまま出すと0.1m刻みで数百区間になる。"""

        self.assertLess(len(plan_route(self.grid, self.route)), 15)

    def test_straight_shot_is_not_detoured(self):
        segments = plan_route(self.grid, [Pose2D(-4.0, 4.0), Pose2D(-4.0, -4.0)])
        self.assertEqual(len(segments), 1)


class RejectionTest(unittest.TestCase):
    def test_needs_at_least_two_waypoints(self):
        with self.assertRaises(RouteError):
            plan_route(open_field(), [Pose2D(0.0, 0.0)])

    def test_waypoint_inside_a_wall_is_rejected_with_its_index(self):
        grid = room_with_doorway()
        with self.assertRaises(RouteError) as caught:
            # (0.0, 4.0) は仕切り壁のど真ん中
            plan_route(grid, [Pose2D(-4.0, 4.0), Pose2D(0.0, 4.0)])
        self.assertIn("1番目", str(caught.exception))

    def test_waypoint_outside_the_map_is_rejected(self):
        with self.assertRaises(RouteError):
            plan_route(open_field(5.0), [Pose2D(0.0, 0.0), Pose2D(100.0, 100.0)])

    def test_unreachable_goal_reports_no_path(self):
        # 完全に閉じた箱の中に目標を置く
        points = []
        for index in range(int(4.0 / 0.05) + 1):
            offset = -2.0 + index * 0.05
            points += [(offset, -2.0, 1.0), (offset, 2.0, 1.0),
                       (-2.0, offset, 1.0), (2.0, offset, 1.0)]
        points += [(-9.0, -9.0, 1.0), (9.0, 9.0, 1.0)]
        grid = build_grid(points, resolution=0.1, inflation=0.4, margin=1.0)
        with self.assertRaises(RouteError):
            plan_route(grid, [Pose2D(-6.0, -6.0), Pose2D(0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
