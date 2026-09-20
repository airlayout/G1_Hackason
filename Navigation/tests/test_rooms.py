"""`sim/rooms.py`: MuJoCo の世界と占有格子が食い違わないこと。

Phase 3.5 でここを作った理由がこれ。前の sim は「地図の上での経路」と
「機体が歩く世界」が別物で、経路が正しくても歩けるとは限らなかった。
寸法を 1 か所に持ち、そこから MJCF と点群の両方を出すことで、
**食い違いが原理的に起きない**ようにしてある。それを固定する。

mujoco は要らない（MJCF はテキストとして検査する）。
"""

import math
import unittest
import xml.etree.ElementTree as ElementTree

from nav.occupancy import DEFAULT_INFLATION_M
from nav.route import plan_route
from sim.rooms import (
    EMPTY_ROOM,
    ROOMS,
    TEST_ROOM,
    Box,
    DynamicObstacle,
    get_room,
    patrol_waypoints,
)


class BoxTest(unittest.TestCase):
    def test_surface_points_sit_on_the_faces(self):
        box = Box("b", 1.0, 2.0, 0.5, 0.25)
        points = box.surface_points()
        self.assertGreater(len(points), 0)
        for x, y, _ in points:
            on_x_face = math.isclose(abs(x - 1.0), 0.5, abs_tol=1e-9)
            on_y_face = math.isclose(abs(y - 2.0), 0.25, abs_tol=1e-9)
            self.assertTrue(on_x_face or on_y_face, f"({x}, {y}) が面の上にない")

    def test_surface_points_land_in_the_obstacle_height_band(self):
        """帯（0.30〜1.80m）を外れた点は障害物にならず、壁が消える。"""

        for _, _, z in Box("b", 0, 0, 0.5, 0.5).surface_points():
            self.assertGreater(z, 0.30)
            self.assertLess(z, 1.80)

    def test_the_mjcf_geom_uses_half_sizes(self):
        element = ElementTree.fromstring(Box("b", 1.0, 2.0, 0.5, 0.25, height=2.0).to_mjcf())
        self.assertEqual(element.get("type"), "box")
        self.assertEqual([float(v) for v in element.get("size").split()], [0.5, 0.25, 1.0])
        self.assertEqual([float(v) for v in element.get("pos").split()], [1.0, 2.0, 1.0])


class RoomGeometryTest(unittest.TestCase):
    def test_the_walls_do_not_eat_into_the_inner_area(self):
        (x0, x1), (y0, y1) = TEST_ROOM.inner_x, TEST_ROOM.inner_y
        for wall in TEST_ROOM.walls:
            self.assertFalse(
                x0 < wall.x < x1 and y0 < wall.y < y1, f"{wall.name} が部屋の内側にある"
            )

    def test_the_inner_area_is_walkable(self):
        grid = TEST_ROOM.grid()
        self.assertTrue(grid.is_free(0.0, 0.0))

    def test_outside_the_walls_is_not_walkable(self):
        grid = TEST_ROOM.grid()
        self.assertFalse(grid.is_free(TEST_ROOM.inner_x[1] + 1.0, 0.0))

    def test_the_declared_obstacles_are_in_the_grid(self):
        """MJCF に置いた箱が格子にも写っていること。これが食い違うと sim の意味が無い。"""

        grid = TEST_ROOM.grid()
        for box in TEST_ROOM.boxes:
            self.assertFalse(grid.is_free(box.x, box.y), f"{box.name} が格子に無い")

    def test_the_floor_is_observed_so_the_room_is_not_all_unknown(self):
        """床の点が無いと未観測扱いになり、部屋が丸ごと通行不可になる。"""

        grid = EMPTY_ROOM.grid()
        self.assertGreater(grid.free_count, 1000)

    def test_the_room_is_one_connected_region(self):
        self.assertEqual(len(TEST_ROOM.grid().free_regions()), 1)

    def test_the_spawn_is_walkable(self):
        for room in ROOMS.values():
            with self.subTest(room=room.name):
                self.assertTrue(room.grid().is_free(room.spawn.x, room.spawn.y))


class PatrolTest(unittest.TestCase):
    def test_every_patrol_waypoint_is_walkable(self):
        for room in ROOMS.values():
            grid = room.grid()
            for pose in patrol_waypoints(room):
                with self.subTest(room=room.name, pose=pose):
                    self.assertTrue(grid.is_free(pose.x, pose.y))

    def test_the_patrol_returns_to_where_it_started(self):
        waypoints = patrol_waypoints(TEST_ROOM)
        self.assertEqual(waypoints[0], waypoints[-1])

    def test_a_route_can_be_planned_through_the_test_room(self):
        segments = plan_route(TEST_ROOM.grid(), patrol_waypoints(TEST_ROOM))
        self.assertGreater(len(segments), 0)

    def test_no_planned_segment_crosses_a_wall(self):
        grid = TEST_ROOM.grid()
        for segment in plan_route(grid, patrol_waypoints(TEST_ROOM)):
            self.assertTrue(
                grid.is_segment_free(
                    segment.start.x, segment.start.y, segment.target.x, segment.target.y
                )
            )

    def test_the_test_room_forces_the_8m_splitter_to_act(self):
        """9m の直線が 1 本あることが部屋の設計意図。無くなったら気付けるようにする。"""

        segments = plan_route(TEST_ROOM.grid(), patrol_waypoints(TEST_ROOM), max_segment_m=8.0)
        self.assertTrue(
            any(not segment.is_waypoint and segment.length > 4.0 for segment in segments)
        )
        for segment in segments:
            self.assertLessEqual(segment.length, 8.0 + 1e-9)

    def test_the_test_room_forces_a_detour_around_the_divider(self):
        """仕切りを回り込む必要があること。直線で済むなら経路計画を試せていない。"""

        grid = TEST_ROOM.grid()
        south_east, north_east = None, None
        for pose in patrol_waypoints(TEST_ROOM):
            if pose.x > 0 and pose.y < 0:
                south_east = pose
            if pose.x > 0 and pose.y > 0:
                north_east = pose
        self.assertFalse(
            grid.is_segment_free(south_east.x, south_east.y, north_east.x, north_east.y)
        )

    def test_every_corridor_is_wider_than_the_machine(self):
        """膨張後に自由なセルがあること＝機体中心が通れること。

        経路計画だけなら通る幅でも、歩容の揺れで壁に当たれば sim は落ちる。
        """

        grid = TEST_ROOM.grid(inflation=DEFAULT_INFLATION_M)
        self.assertGreater(grid.free_count, 2000)
        self.assertEqual(len(grid.free_regions()), 1)


class MjcfTest(unittest.TestCase):
    def test_the_scene_is_well_formed_xml(self):
        ElementTree.fromstring(TEST_ROOM.to_mjcf(with_robot=False))

    def test_the_scene_includes_the_robot_model(self):
        self.assertIn('<include file="g1_12dof.xml"/>', TEST_ROOM.to_mjcf())

    def test_the_scene_can_be_built_without_the_robot(self):
        self.assertNotIn("include", TEST_ROOM.to_mjcf(with_robot=False))

    def test_every_solid_becomes_a_geom(self):
        root = ElementTree.fromstring(TEST_ROOM.to_mjcf(with_robot=False))
        names = {geom.get("name") for geom in root.iter("geom")}
        for box in TEST_ROOM.solids:
            self.assertIn(box.name, names)

    def test_the_keyframe_puts_the_robot_at_the_spawn(self):
        root = ElementTree.fromstring(TEST_ROOM.to_mjcf())
        qpos = [float(v) for v in root.find("keyframe/key").get("qpos").split()]
        self.assertAlmostEqual(qpos[0], TEST_ROOM.spawn.x)
        self.assertAlmostEqual(qpos[1], TEST_ROOM.spawn.y)

    def test_the_keyframe_has_one_value_per_degree_of_freedom(self):
        """自由関節 7 + 脚 12 = 19。ずれると生成直後に転ぶ。"""

        root = ElementTree.fromstring(TEST_ROOM.to_mjcf())
        self.assertEqual(len(root.find("keyframe/key").get("qpos").split()), 19)


class DynamicObstacleTest(unittest.TestCase):
    def test_it_is_absent_before_it_appears(self):
        obstacle = DynamicObstacle(1.0, 2.0, appears_at_s=5.0, disappears_at_s=9.0)
        self.assertFalse(obstacle.is_active(4.9))
        self.assertLess(obstacle.position_at(4.9)[2], 0.0)

    def test_it_is_present_while_it_lasts(self):
        obstacle = DynamicObstacle(1.0, 2.0, appears_at_s=5.0, disappears_at_s=9.0)
        self.assertTrue(obstacle.is_active(5.0))
        self.assertGreater(obstacle.position_at(6.0)[2], 0.0)

    def test_it_is_absent_again_after_it_goes(self):
        obstacle = DynamicObstacle(1.0, 2.0, appears_at_s=5.0, disappears_at_s=9.0)
        self.assertFalse(obstacle.is_active(9.0))
        self.assertLess(obstacle.position_at(9.0)[2], 0.0)

    def test_it_stays_at_its_xy_even_when_parked(self):
        obstacle = DynamicObstacle(1.0, 2.0, appears_at_s=5.0)
        self.assertEqual(obstacle.position_at(0.0)[:2], (1.0, 2.0))

    def test_it_is_present_forever_by_default(self):
        self.assertTrue(DynamicObstacle(0.0, 0.0).is_active(10_000.0))


class LookupTest(unittest.TestCase):
    def test_rooms_are_found_by_name(self):
        self.assertIs(get_room(TEST_ROOM.name), TEST_ROOM)

    def test_an_unknown_room_says_what_exists(self):
        with self.assertRaises(SystemExit) as caught:
            get_room("no_such_room")
        self.assertIn(TEST_ROOM.name, str(caught.exception))

    def test_every_registered_room_is_keyed_by_its_own_name(self):
        for name, room in ROOMS.items():
            self.assertEqual(name, room.name)


if __name__ == "__main__":
    unittest.main()
