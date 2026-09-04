from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest


TOOLS_PACKAGE = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "g1_mapping_tools"
)
sys.path.insert(0, str(TOOLS_PACKAGE))

from g1_mapping_tools.density_grid import (  # noqa: E402
    DENSITY_POINT_STEP,
    DensityGrid,
    pack_density_points,
)


class DensityGridTest(unittest.TestCase):
    def test_counts_hits_and_unique_scans_separately(self) -> None:
        grid = DensityGrid(voxel_size=0.1, max_points=10, target_scan_count=4)
        grid.integrate_scan(
            [(0.01, 0.01, 0.01, 10.0), (0.02, 0.02, 0.02, 30.0)]
        )
        first = grid.snapshot()[0]
        self.assertEqual(first[5], 2)
        self.assertEqual(first[6], 1)
        self.assertAlmostEqual(first[4], 0.25)
        self.assertAlmostEqual(first[0], 0.015)
        self.assertAlmostEqual(first[3], 20.0)

        grid.integrate_scan([(0.03, 0.03, 0.03, 20.0)])
        second = grid.snapshot()[0]
        self.assertEqual(second[5], 3)
        self.assertEqual(second[6], 2)
        self.assertAlmostEqual(second[4], 0.5)

    def test_density_saturates_and_existing_voxels_keep_updating_at_limit(self) -> None:
        grid = DensityGrid(voxel_size=1.0, max_points=1, target_scan_count=2)
        grid.integrate_scan([(0.0, 0.0, 0.0, 1.0)])
        grid.integrate_scan([(2.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)])
        grid.integrate_scan([(0.0, 0.0, 0.0, 1.0)])
        point = grid.snapshot()[0]
        self.assertEqual(len(grid.voxels), 1)
        self.assertEqual(grid.dropped_new_voxels, 1)
        self.assertEqual(point[6], 3)
        self.assertEqual(point[4], 1.0)

    def test_binary_layout_matches_pointcloud_fields(self) -> None:
        value = (1.0, 2.0, 3.0, 4.0, 0.5, 6, 7)
        data = pack_density_points([value])
        self.assertEqual(len(data), DENSITY_POINT_STEP)
        self.assertEqual(struct.unpack("<fffffII", data), value)

    def test_invalid_coordinates_are_ignored(self) -> None:
        grid = DensityGrid(voxel_size=0.1, max_points=10, target_scan_count=10)
        updated = grid.integrate_scan([(float("nan"), 0.0, 0.0, 1.0)])
        self.assertEqual(updated, 0)
        self.assertEqual(grid.snapshot(), [])


if __name__ == "__main__":
    unittest.main()
