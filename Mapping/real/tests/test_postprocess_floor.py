from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

# Keep optional scientific dependencies explicit on minimal CI hosts.
_MISSING = [name for name in ("numpy", "scipy") if importlib.util.find_spec(name) is None]
if _MISSING:
    raise unittest.SkipTest(f"{', '.join(_MISSING)} が無い（床基準の点群処理が使う）")

import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT = Path(__file__).resolve().parents[1] / "quickstart" / "postprocess_floor_pcd.py"
SPEC = importlib.util.spec_from_file_location("postprocess_floor_pcd", SCRIPT)
floor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(floor)


class FloorPostprocessTest(unittest.TestCase):
    def test_no_denoise_keeps_isolated_points_without_querying_neighbors(self):
        xyz = np.array([[0., 0., 0.], [20., 20., 1.], [np.nan, 0., 0.]])
        with patch.object(floor, "cKDTree", side_effect=AssertionError("must not denoise")):
            indices, finite = floor.select_points(xyz, .15, remove_isolated=False)
        np.testing.assert_array_equal(indices, [0, 1])
        np.testing.assert_array_equal(finite, [True, True, False])

    def test_reference_frame_requires_the_same_source_and_rigid_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "processing.json"
            report = {"source_sha256": "source", "map_to_floor_matrix": np.eye(4).tolist(),
                      "ceiling_height_at_frame_origin_m": 2.8, "isolated_points_removed": 10}
            path.write_text(json.dumps(report))
            transform, reused = floor.reference_frame(path, "source")
            np.testing.assert_array_equal(transform, np.eye(4))
            self.assertNotIn("isolated_points_removed", reused)
            with self.assertRaisesRegex(ValueError, "different source"):
                floor.reference_frame(path, "other")
            report["map_to_floor_matrix"][0][0] = 2
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "rigid"):
                floor.reference_frame(path, "source")

    def test_rotated_upside_down_room_is_leveled(self):
        rng = np.random.default_rng(42)
        xy = rng.uniform(-10, 10, (10000, 2))
        room = np.r_[np.c_[xy, rng.normal(0, .003, len(xy))],
                     np.c_[xy, 2.8 + rng.normal(0, .003, len(xy))]]
        rotation = Rotation.from_euler("xyz", [170, 29, -20], degrees=True).as_matrix()
        translation = np.array([1, 2, 5])
        points = room @ rotation.T + translation
        trajectory = np.c_[np.linspace(-2, 2, 100), np.zeros(100), np.full(100, 1.2)] @ rotation.T + translation
        transform, report = floor.find_room_frame(points, rotation[:, 2], trajectory)
        corrected = points @ transform[:3, :3].T + transform[:3, 3]
        self.assertAlmostEqual(np.median(corrected[:10000, 2]), 0, delta=.01)
        self.assertAlmostEqual(np.median(corrected[10000:, 2]), 2.8, delta=.01)
        self.assertAlmostEqual(report["ceiling_height_at_frame_origin_m"], 2.8, delta=.01)
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1, places=10)

    def test_missing_ceiling_is_not_silently_invented(self):
        rng = np.random.default_rng(10)
        points = np.c_[rng.uniform(-5, 5, (10000, 2)), rng.normal(-1.2, .005, 10000)]
        with self.assertRaisesRegex(ValueError, "below AND above"):
            floor.find_room_frame(points, np.array([0., 0., 1.]), np.zeros((100, 3)))

    def test_pcd_preserves_attributes_and_strict_height_boundary(self):
        header = {"VERSION": ["0.7"], "FIELDS": ["x", "y", "z", "scan_count"],
                  "SIZE": ["4"] * 4, "TYPE": ["F", "F", "F", "U"],
                  "COUNT": ["1"] * 4, "DATA": ["binary"]}
        xyz = np.array([[1, 2, 0], [3, 4, np.nextafter(np.float32(2), np.float32(0))]], dtype="<f4")
        records = np.zeros((2, 16), dtype=np.uint8)
        records[:, :12] = xyz.view(np.uint8).reshape(2, 12)
        records[:, 12:] = np.array([17, 200], dtype="<u4").view(np.uint8).reshape(2, 4)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "crop.pcd"
            floor.write_records(path, header, records, xyz, (0, 4, 8), "f", np.eye(4), maximum=2.)
            h, offset = floor._pcd_header(path)
            data = np.memmap(path, mode="r", offset=offset, dtype=np.uint8, shape=(2, 24))
            self.assertEqual(h["FIELDS"], ["x", "y", "z", "scan_count", "height", "rgb"])
            np.testing.assert_array_equal(data[:, :16], records)
            heights = np.ndarray((2,), dtype="<f4", buffer=data, offset=16, strides=(24,))
            self.assertTrue(np.all(heights < 2))
            with self.assertRaises(FileExistsError):
                floor.write_records(path, header, records, xyz, (0, 4, 8), "f", np.eye(4), maximum=2.)
            with self.assertRaisesRegex(ValueError, "boundary"):
                bad = xyz.copy()
                bad[1, 2] = 2
                floor.write_records(Path(directory) / "bad.pcd", header, records, bad, (0, 4, 8), "f", np.eye(4), maximum=2.)


if __name__ == "__main__":
    unittest.main()
