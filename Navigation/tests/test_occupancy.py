from __future__ import annotations

import struct
import tempfile
import unittest
from pathlib import Path

from nav.occupancy import (
    DEFAULT_OBSTACLE_Z_MAX,
    DEFAULT_OBSTACLE_Z_MIN,
    build_grid,
    load_grid,
    load_pcd,
)


def floor(x0: float, y0: float, x1: float, y1: float, step: float = 0.05):
    """床の点(z=0)。障害物にはならないが「観測済み」の証拠になる。

    占有格子は未観測の外側を通行不可とするので、床を置かないと全面が塞がる。
    実際の地図（`sim_room.pcd` は z≒0 に151万点）と同じ条件を作るためのもの。
    """

    cols = int((x1 - x0) / step) + 1
    rows = int((y1 - y0) / step) + 1
    return [
        (x0 + col * step, y0 + row * step, 0.0)
        for row in range(rows)
        for col in range(cols)
    ]


def wall_points(x: float, y_from: float, y_to: float, step: float = 0.05):
    """x=const の壁。高さは障害物とみなす範囲の中に置く。"""

    count = int((y_to - y_from) / step) + 1
    return [(x, y_from + index * step, 1.0) for index in range(count)]


def write_binary_pcd(path: Path, points) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        for x, y, z in points:
            stream.write(struct.pack("<fff", x, y, z))


class HeightFilterTest(unittest.TestCase):
    def test_floor_and_ceiling_are_not_obstacles(self):
        room = floor(-2.0, -2.0, 2.0, 2.0)
        ceiling = [(x, y, 2.52) for x, y, _ in room]
        grid = build_grid(room + ceiling, resolution=0.1, inflation=0.0)
        self.assertTrue(grid.is_free(0.0, 0.0))
        self.assertTrue(grid.is_free(1.5, -1.5))

    def test_point_in_range_is_an_obstacle(self):
        grid = build_grid(
            floor(-2.0, -2.0, 2.0, 2.0) + [(0.0, 0.0, 1.0)], resolution=0.1, inflation=0.0
        )
        self.assertFalse(grid.is_free(0.0, 0.0))
        self.assertTrue(grid.is_free(1.0, 1.0))

    def test_range_boundaries_are_inclusive(self):
        room = floor(-2.0, -2.0, 2.0, 2.0)
        for height in (DEFAULT_OBSTACLE_Z_MIN, DEFAULT_OBSTACLE_Z_MAX):
            grid = build_grid(room + [(0.0, 0.0, height)], resolution=0.1, inflation=0.0)
            self.assertFalse(grid.is_free(0.0, 0.0), f"z={height} は障害物のはず")

    def test_above_the_range_is_not_an_obstacle(self):
        grid = build_grid(
            floor(-2.0, -2.0, 2.0, 2.0) + [(0.0, 0.0, DEFAULT_OBSTACLE_Z_MAX + 0.1)],
            resolution=0.1,
            inflation=0.0,
        )
        self.assertTrue(grid.is_free(0.0, 0.0))


class InflationTest(unittest.TestCase):
    def test_single_point_becomes_a_disk(self):
        room = floor(-4.0, -4.0, 4.0, 4.0)
        grid = build_grid(room + [(0.0, 0.0, 1.0)], resolution=0.1, inflation=0.4)
        self.assertFalse(grid.is_free(0.0, 0.0))
        self.assertFalse(grid.is_free(0.3, 0.0))  # 膨張の内側
        self.assertTrue(grid.is_free(0.6, 0.0))   # 膨張の外側

    def test_no_inflation_keeps_the_point_itself(self):
        room = floor(-4.0, -4.0, 4.0, 4.0)
        grid = build_grid(room + [(0.0, 0.0, 1.0)], resolution=0.1, inflation=0.0)
        self.assertFalse(grid.is_free(0.0, 0.0))
        self.assertTrue(grid.is_free(0.3, 0.0))


class UnobservedTest(unittest.TestCase):
    """未観測は「自由」ではなく「未知」。通行可能にしてはいけない。

    ここを自由にすると、外周の余白を通って**建物の外側を回る経路**が引ける。
    実データ(`sim_room.pcd`)で実際に起きた不具合の再発防止。
    """

    def setUp(self):
        self.grid = build_grid(floor(-2.0, -2.0, 2.0, 2.0), resolution=0.1,
                               inflation=0.0, margin=1.0)

    def test_observed_floor_is_free(self):
        self.assertTrue(self.grid.is_free(0.0, 0.0))

    def test_margin_ring_outside_the_floor_is_blocked(self):
        # margin=1.0 なので格子は広いが、床が無い外周は通れない
        self.assertFalse(self.grid.is_free(2.5, 0.0))
        self.assertFalse(self.grid.is_free(0.0, -2.5))

    def test_outside_the_grid_is_blocked(self):
        self.assertFalse(self.grid.is_free(100.0, 100.0))
        self.assertFalse(self.grid.is_free(-100.0, 0.0))

    def test_interior_hole_stays_free(self):
        """部屋の中の死角（床が写らなかった小穴）まで塞ぐと通路が消える。"""

        holed = [p for p in floor(-2.0, -2.0, 2.0, 2.0) if not (0.4 < p[0] < 0.6 and 0.4 < p[1] < 0.6)]
        grid = build_grid(holed, resolution=0.1, inflation=0.0, margin=1.0)
        self.assertTrue(grid.is_free(0.5, 0.5))


class SegmentTest(unittest.TestCase):
    def setUp(self):
        # x=0 に壁のある部屋
        self.grid = build_grid(
            floor(-3.0, -3.0, 3.0, 3.0) + wall_points(0.0, -3.0, 3.0),
            resolution=0.1,
            inflation=0.4,
        )

    def test_segment_through_a_wall_is_blocked(self):
        self.assertFalse(self.grid.is_segment_free(-2.0, 0.0, 2.0, 0.0))

    def test_segment_parallel_to_a_wall_is_free(self):
        self.assertTrue(self.grid.is_segment_free(-2.0, -2.0, -2.0, 2.0))

    def test_zero_length_segment_matches_point_check(self):
        self.assertTrue(self.grid.is_segment_free(-2.0, 0.0, -2.0, 0.0))
        self.assertFalse(self.grid.is_segment_free(0.0, 0.0, 0.0, 0.0))

    def test_diagonal_segment_cannot_tunnel_through_the_wall(self):
        self.assertFalse(self.grid.is_segment_free(-2.0, -2.0, 2.0, 2.0))


class PcdTest(unittest.TestCase):
    def test_round_trips_binary_pcd(self):
        points = [(1.0, 2.0, 3.0), (-1.5, 0.25, 1.0)]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.pcd"
            write_binary_pcd(path, points)
            loaded = load_pcd(path)
        self.assertEqual(len(loaded), 2)
        for expected, actual in zip(points, loaded):
            for a, b in zip(expected, actual):
                self.assertAlmostEqual(a, b, places=5)

    def test_load_grid_reads_a_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.pcd"
            write_binary_pcd(path, floor(-2.0, -2.0, 2.0, 2.0) + wall_points(0.0, -2.0, 2.0))
            grid = load_grid(path, resolution=0.1, inflation=0.0)
        self.assertTrue(grid.is_free(-1.0, 0.0))
        self.assertFalse(grid.is_free(0.0, 0.0))

    def test_rejects_unexpected_fields_instead_of_misreading(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.pcd"
            path.write_bytes(
                b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                b"TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\n"
                b"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA binary\n"
                + struct.pack("<ffff", 0.0, 0.0, 0.0, 1.0)
            )
            with self.assertRaises(ValueError):
                load_pcd(path)

    def test_rejects_ascii_pcd(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.pcd"
            path.write_text(
                "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                "POINTS 1\nDATA ascii\n0 0 0\n"
            )
            with self.assertRaises(ValueError):
                load_pcd(path)


class GuardTest(unittest.TestCase):
    def test_empty_point_cloud_is_an_error(self):
        with self.assertRaises(ValueError):
            build_grid([])

    def test_non_positive_resolution_is_an_error(self):
        with self.assertRaises(ValueError):
            build_grid([(0.0, 0.0, 1.0)], resolution=0.0)


if __name__ == "__main__":
    unittest.main()
