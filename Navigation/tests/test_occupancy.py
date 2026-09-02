"""`nav/occupancy.py`: 通行判定の格子。

計算そのものは scipy に任せてあるので、ここで固定するのは
**G1 のナビ固有の決め事**だけ:

- 床と天井を障害物にしない（高さの帯）
- 未観測を通行可にしない（地図の外へ歩かせない）
- 壁をすり抜けない（線分判定は安全側に倒す）
- 機体半径ぶん膨らませる
"""

import unittest

import numpy as np

from nav.occupancy import (
    DEFAULT_OBSTACLE_Z_MAX,
    DEFAULT_OBSTACLE_Z_MIN,
    GridSpec,
    OccupancyGrid,
    build_grid,
)


def box_cloud(x0, x1, y0, y1, *, z=1.0, step=0.05):
    """矩形の外周に点を打つ（壁のつもり）+ 内側の床。"""

    xs = np.arange(x0, x1 + step, step)
    ys = np.arange(y0, y1 + step, step)
    wall = np.concatenate([
        np.stack([xs, np.full_like(xs, y0), np.full_like(xs, z)], axis=1),
        np.stack([xs, np.full_like(xs, y1), np.full_like(xs, z)], axis=1),
        np.stack([np.full_like(ys, x0), ys, np.full_like(ys, z)], axis=1),
        np.stack([np.full_like(ys, x1), ys, np.full_like(ys, z)], axis=1),
    ])
    grid_x, grid_y = np.meshgrid(np.arange(x0, x1 + 0.1, 0.1), np.arange(y0, y1 + 0.1, 0.1))
    floor = np.stack([grid_x.ravel(), grid_y.ravel(), np.zeros(grid_x.size)], axis=1)
    return np.concatenate([wall, floor])


class GridSpecTest(unittest.TestCase):
    def setUp(self):
        self.spec = GridSpec(origin_x=-1.0, origin_y=-2.0, resolution=0.1, width=40, height=50)

    def test_the_origin_is_the_lower_left_corner_of_cell_zero(self):
        self.assertEqual(self.spec.to_cell(-1.0, -2.0), (0, 0))

    def test_a_cell_maps_back_to_its_centre(self):
        x, y = self.spec.to_world(0, 0)
        self.assertAlmostEqual(x, -0.95)
        self.assertAlmostEqual(y, -1.95)

    def test_world_to_cell_to_world_stays_inside_the_same_cell(self):
        col, row = self.spec.to_cell(0.37, 1.44)
        x, y = self.spec.to_world(col, row)
        self.assertLess(abs(x - 0.37), self.spec.resolution)
        self.assertLess(abs(y - 1.44), self.spec.resolution)

    def test_cells_outside_the_grid_are_rejected(self):
        self.assertFalse(self.spec.contains(-1, 0))
        self.assertFalse(self.spec.contains(0, 50))
        self.assertTrue(self.spec.contains(0, 0))

    def test_arrays_are_converted_too(self):
        col, row = self.spec.to_cell(np.array([-1.0, 0.0]), np.array([-2.0, 0.0]))
        self.assertEqual(list(col), [0, 10])
        self.assertEqual(list(row), [0, 20])


class BuildGridTest(unittest.TestCase):
    def test_the_floor_is_not_an_obstacle(self):
        """床の点（z=0）を障害物にすると部屋が丸ごと通れなくなる。"""

        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0)
        self.assertTrue(grid.is_free(0.0, 0.0))

    def test_points_above_the_ceiling_band_are_not_obstacles(self):
        cloud = box_cloud(-2, 2, -2, 2)
        overhead = np.array([[0.0, 0.0, DEFAULT_OBSTACLE_Z_MAX + 0.5]])
        grid = build_grid(np.concatenate([cloud, overhead]), inflation=0.0)
        self.assertTrue(grid.is_free(0.0, 0.0))

    def test_points_inside_the_band_are_obstacles(self):
        cloud = box_cloud(-2, 2, -2, 2)
        chest_high = np.array([[0.0, 0.0, (DEFAULT_OBSTACLE_Z_MIN + DEFAULT_OBSTACLE_Z_MAX) / 2]])
        grid = build_grid(np.concatenate([cloud, chest_high]), inflation=0.0)
        self.assertFalse(grid.is_free(0.0, 0.0))

    def test_outside_the_map_is_not_free(self):
        """地図に無い場所は「何も無い」ではなく「分からない」。歩かせない。"""

        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0)
        self.assertFalse(grid.is_free(100.0, 100.0))

    def test_the_unobserved_margin_around_the_room_is_not_free(self):
        """余白を自由にすると、建物の外側を回る経路が引ける（実際に踏んだバグ）。"""

        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0, margin=1.0)
        self.assertFalse(grid.is_free(-2.5, 0.0))

    def test_inflation_pushes_the_free_space_away_from_the_wall(self):
        loose = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0)
        tight = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.4)
        self.assertTrue(loose.is_free(-1.85, 0.0))
        self.assertFalse(tight.is_free(-1.85, 0.0))
        self.assertTrue(tight.is_free(0.0, 0.0))

    def test_inflation_reaches_about_the_requested_radius(self):
        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.4)
        self.assertFalse(grid.is_free(-1.75, 0.0))  # 壁から 0.25m
        self.assertTrue(grid.is_free(-1.45, 0.0))   # 壁から 0.55m

    def test_an_empty_cloud_is_rejected(self):
        with self.assertRaises(ValueError):
            build_grid(np.zeros((0, 3)))

    def test_a_non_positive_resolution_is_rejected(self):
        with self.assertRaises(ValueError):
            build_grid(box_cloud(-1, 1, -1, 1), resolution=0.0)

    def test_the_grid_shape_must_match_the_spec(self):
        spec = GridSpec(0.0, 0.0, 0.1, 10, 10)
        with self.assertRaises(ValueError):
            OccupancyGrid(spec, np.zeros((5, 5), bool))


class SegmentTest(unittest.TestCase):
    def setUp(self):
        # 部屋を真ん中で仕切る。開口は y ∈ [0.4, 1.0] だけ
        cloud = box_cloud(-2, 2, -2, 2)
        wall = np.array([[0.0, y, 1.0] for y in np.arange(-1.9, 0.4, 0.02)]
                        + [[0.0, y, 1.0] for y in np.arange(1.0, 1.92, 0.02)])
        self.grid = build_grid(np.concatenate([cloud, wall]), inflation=0.0)

    def test_a_clear_line_is_free(self):
        self.assertTrue(self.grid.is_segment_free(-1.5, 0.7, -0.5, 0.7))

    def test_a_line_through_the_wall_is_not_free(self):
        self.assertFalse(self.grid.is_segment_free(-1.5, -1.0, 1.5, -1.0))

    def test_a_line_through_the_opening_is_free(self):
        self.assertTrue(self.grid.is_segment_free(-1.5, 0.7, 1.5, 0.7))

    def test_a_diagonal_does_not_slip_through_a_corner(self):
        """壁の角をかすめる斜めの線を通してはいけない。

        この線は **Bresenham（`skimage.draw.line`）だと「通れる」になる**。
        1 行につき 1 セルしか取らないので、開口の下端 (0, 0.4) にある壁のセルを
        飛び越すため。`line_aa` はかすめたセルも列挙するので塞ぐ。
        判定を安全側に倒してあることの回帰テスト。
        """

        from skimage.draw import line

        col0, row0 = self.grid.spec.to_cell(-0.4, 0.0)
        col1, row1 = self.grid.spec.to_cell(0.4, 0.8)
        rows, cols = line(int(row0), int(col0), int(row1), int(col1))
        self.assertFalse(
            bool(self.grid.blocked[rows, cols].any()),
            "前提が崩れている: Bresenham でも塞がるなら、この線は回帰テストにならない",
        )
        self.assertFalse(self.grid.is_segment_free(-0.4, 0.0, 0.4, 0.8))

    def test_a_diagonal_well_inside_the_opening_is_free(self):
        self.assertTrue(self.grid.is_segment_free(-0.3, 0.1, 0.3, 0.7))

    def test_a_segment_leaving_the_map_is_not_free(self):
        self.assertFalse(self.grid.is_segment_free(0.0, 0.0, 100.0, 0.0))

    def test_a_zero_length_segment_on_free_space_is_free(self):
        self.assertTrue(self.grid.is_segment_free(-1.0, 0.0, -1.0, 0.0))


class RegionTest(unittest.TestCase):
    def test_a_wall_that_splits_the_room_gives_two_regions(self):
        cloud = box_cloud(-2, 2, -2, 2)
        wall = np.array([[0.0, y, 1.0] for y in np.arange(-1.95, 1.96, 0.02)])
        grid = build_grid(np.concatenate([cloud, wall]), inflation=0.0)
        self.assertEqual(len(grid.free_regions()), 2)

    def test_an_open_room_is_one_region(self):
        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0)
        self.assertEqual(len(grid.free_regions()), 1)

    def test_regions_come_back_largest_first(self):
        cloud = box_cloud(-3, 3, -2, 2)
        wall = np.array([[1.5, y, 1.0] for y in np.arange(-1.95, 1.96, 0.02)])
        regions = build_grid(np.concatenate([cloud, wall]), inflation=0.0).free_regions()
        self.assertGreater(len(regions[0]), len(regions[1]))

    def test_the_free_count_matches_the_regions(self):
        grid = build_grid(box_cloud(-2, 2, -2, 2), inflation=0.0)
        self.assertEqual(grid.free_count, sum(len(region) for region in grid.free_regions()))


class ExtraObstacleTest(unittest.TestCase):
    def setUp(self):
        self.grid = build_grid(box_cloud(-3, 3, -3, 3), inflation=0.0)

    def test_adding_an_obstacle_blocks_the_disk(self):
        blocked = self.grid.with_extra_obstacle(0.0, 0.0, 0.5)
        self.assertFalse(blocked.is_free(0.0, 0.0))
        self.assertFalse(blocked.is_free(0.3, 0.0))
        self.assertTrue(blocked.is_free(1.0, 0.0))

    def test_the_original_grid_is_not_changed(self):
        self.grid.with_extra_obstacle(0.0, 0.0, 0.5)
        self.assertTrue(self.grid.is_free(0.0, 0.0))

    def test_two_obstacles_stack(self):
        blocked = self.grid.with_extra_obstacle(0.0, 0.0, 0.4).with_extra_obstacle(1.5, 0.0, 0.4)
        self.assertFalse(blocked.is_free(0.0, 0.0))
        self.assertFalse(blocked.is_free(1.5, 0.0))
        self.assertTrue(blocked.is_free(0.75, 0.0))

    def test_dropping_an_obstacle_means_rebuilding_from_the_original(self):
        """`nav/mission.py` は迂回の憶測を一覧で持ち、要らなくなったら重ねない。

        格子側に「消す」操作を持たせないのは、実測の壁まで消せてしまうため。
        """

        with_guess = self.grid.with_extra_obstacle(0.0, 0.0, 0.5)
        self.assertFalse(with_guess.is_free(0.0, 0.0))
        self.assertTrue(self.grid.is_free(0.0, 0.0))

if __name__ == "__main__":
    unittest.main()
