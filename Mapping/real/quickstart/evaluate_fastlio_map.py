#!/usr/bin/env python3
"""FAST-LIO2 の A/B 地図を、その地図と同時記録した軌跡で評価する。"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def read_trajectory(path: Path) -> np.ndarray:
    """`ros2 topic echo --csv nav_msgs/Odometry` の位置列を読む。"""
    points = []
    with path.open(newline="") as stream:
        for row in csv.reader(stream):
            if len(row) < 7:
                continue
            try:
                points.append((float(row[4]), float(row[5]), float(row[6])))
            except ValueError:
                continue
    if not points:
        raise ValueError(f"軌跡を読めません: {path}")
    return np.asarray(points)


def fit_floor(points: np.ndarray) -> tuple[np.ndarray, float, int]:
    """最大水平面を床候補として返す。normal は +Z 側へ揃える。"""
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    sample = cloud.voxel_down_sample(0.10)
    plane, inliers = sample.segment_plane(
        distance_threshold=0.05, ransac_n=3, num_iterations=2000)
    normal = np.asarray(plane[:3], dtype=float)
    normal /= np.linalg.norm(normal)
    offset = float(plane[3]) / np.linalg.norm(np.asarray(plane[:3], dtype=float))
    if normal[2] < 0:
        normal = -normal
        offset = -offset
    return normal, offset, len(inliers)


def plane_coordinates(points: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """点を床面上の直交2軸へ射影する。"""
    reference = np.array([1.0, 0.0, 0.0])
    if abs(float(normal @ reference)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    axis_x = reference - normal * float(normal @ reference)
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    return np.column_stack(
        (np.einsum("ij,j->i", points, axis_x),
         np.einsum("ij,j->i", points, axis_y)))


def evaluate(map_path: Path, trajectory_path: Path) -> dict:
    points = np.asarray(o3d.io.read_point_cloud(str(map_path)).points)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    trajectory = read_trajectory(trajectory_path)
    normal, offset, floor_inliers = fit_floor(points)

    # 軌跡が床より上になる向きを正とする。
    trajectory_height = np.einsum("ij,j->i", trajectory, normal) + offset
    if np.median(trajectory_height) < 0:
        normal, offset = -normal, -offset
        trajectory_height = -trajectory_height
    height = np.einsum("ij,j->i", points, normal) + offset

    map_xy = plane_coordinates(points, normal)
    trajectory_xy = plane_coordinates(trajectory[::5], normal)
    distance, _ = cKDTree(trajectory_xy).query(map_xy, workers=-1)
    human = (height >= 0.5) & (height <= 1.9)
    follower = human & (distance >= 0.45) & (distance <= 1.8)
    structure = (~human) | (distance >= 2.5)

    core_low, core_high = np.percentile(points, [1, 99], axis=0)
    closure = float(np.linalg.norm(trajectory[-1] - trajectory[0]))
    result = {
        "map": map_path.name,
        "points": int(len(points)),
        "core_extent_xyz": [float(v) for v in core_high - core_low],
        "floor_tilt_deg": float(math.degrees(math.acos(np.clip(abs(normal[2]), 0, 1)))),
        "floor_inliers_downsampled": floor_inliers,
        "trajectory_points": int(len(trajectory)),
        "trajectory_height_median": float(np.median(trajectory_height)),
        "closure_error_m": closure,
        "human_band_points": int(human.sum()),
        "follower_band_points": int(follower.sum()),
        "follower_band_pct": float(follower.mean() * 100),
        "structure_points": int(structure.sum()),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args()
    result = evaluate(args.map, args.trajectory)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(rendered + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
