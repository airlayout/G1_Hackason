#!/usr/bin/env python3
"""Fit gravity-constrained room planes, level a PCD copy, and crop by floor height.

The supplied --up must be measured in the map frame (not assumed to be map Z).
This is a rigid frame correction, not SLAM re-optimization or loop closure.
Requires numpy/scipy. Input and existing outputs are never overwritten.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd import _pcd_header, _point_count, _xyz_offsets  # noqa: E402


def read_records(path: Path):
    header, offset = _pcd_header(path)
    if header["DATA"] != ["binary"]:
        raise ValueError("Only uncompressed binary PCD is supported")
    if set(header["FIELDS"]) & {"height", "rgb", "rgba"}:
        raise ValueError("Use an uncolored source without a height field")
    x, y, z, step, code = _xyz_offsets(header)
    count = _point_count(header)
    if path.stat().st_size != offset + count * step:
        raise ValueError("PCD payload length does not match its header")
    records = np.memmap(path, mode="r", dtype=np.uint8, offset=offset,
                        shape=(count, step))
    xyz = np.column_stack([
        np.ndarray((count,), dtype="<" + code, buffer=records, offset=i,
                   strides=(step,)) for i in (x, y, z)
    ])
    return header, records, xyz, (x, y, z), code


def fit_plane(points: np.ndarray, up: np.ndarray, mode: float, band: float):
    slab = points[np.abs(points @ up - mode) < band]
    if len(slab) < 100:
        raise ValueError("Insufficient plane support")
    sample = slab[::max(1, len(slab) // 80000)]
    fit = sample
    for _ in range(7):
        center = np.median(fit, axis=0)
        _, vectors = np.linalg.eigh((fit - center).T @ (fit - center))
        normal = vectors[:, 0]
        normal *= np.sign(normal @ up)
        offset = float(np.median(fit @ normal))
        residual = sample @ normal - offset
        limit = max(0.015, float(np.percentile(np.abs(residual), 90)))
        fit = sample[np.abs(residual) <= limit]
    angle = float(np.degrees(np.arccos(np.clip(normal @ up, -1, 1))))
    if angle > 10:
        raise ValueError("Detected plane is not consistent with measured gravity")
    residual = slab @ normal - offset
    return normal, offset, {
        "normal_in_map": normal.tolist(), "equation": "normal dot map_xyz = offset_m",
        "offset_m": offset, "candidate_points": len(slab),
        "within_5cm_points": int((np.abs(residual) < 0.05).sum()),
        "absolute_residual_p50_p95_m": np.percentile(np.abs(residual), [50, 95]).tolist(),
        "angle_to_imu_up_degrees": angle,
    }


def find_room_frame(points: np.ndarray, up: np.ndarray, trajectory: np.ndarray,
                    band: float = 0.12):
    up = np.asarray(up, dtype=float)
    if not np.isfinite(up).all() or np.linalg.norm(up) < 1e-6:
        raise ValueError("A finite nonzero measured up direction is required")
    up /= np.linalg.norm(up)
    height = points @ up
    lo, hi = np.percentile(height, [0.1, 99.9])
    edges = np.arange(np.floor(lo / 0.025) * 0.025 - 0.025, hi + 0.05, 0.025)
    counts, edges = np.histogram(height, edges)
    smooth = gaussian_filter1d(counts.astype(float), 1)
    peaks, _ = find_peaks(smooth, distance=10, prominence=max(smooth.max() * 0.02, 10))
    centers = (edges[:-1] + edges[1:]) / 2
    sensor_height = float(np.median(trajectory @ up))
    below = [i for i in peaks if centers[i] < sensor_height - 0.3]
    above = [i for i in peaks if centers[i] > sensor_height + 0.3]
    if not below or not above:
        raise ValueError("Cannot identify supported planes below AND above the sensor")
    floor_peak = max(below, key=lambda i: smooth[i])
    ceiling_peak = max(above, key=lambda i: smooth[i])
    normal, floor_offset, floor = fit_plane(points, up, centers[floor_peak], band)
    ceiling_normal, ceiling_offset, ceiling = fit_plane(points, up, centers[ceiling_peak], band)
    parallel_angle = float(np.degrees(np.arccos(np.clip(normal @ ceiling_normal, -1, 1))))
    if parallel_angle > 5:
        raise ValueError("Floor/ceiling are not sufficiently parallel for a single room frame")
    # Preserve the projected map-X heading; XY is not georeferenced.
    x_axis = np.array([1., 0., 0.])
    x_axis -= (x_axis @ normal) * normal
    if np.linalg.norm(x_axis) < 0.1:
        x_axis = np.array([0., 1., 0.]) - normal[1] * normal
    x_axis /= np.linalg.norm(x_axis)
    rotation = np.stack([x_axis, np.cross(normal, x_axis), normal])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[2, 3] = -floor_offset
    ceiling_height = float(ceiling_offset / (ceiling_normal @ normal) - floor_offset)
    trajectory_height = trajectory @ normal - floor_offset
    if not 1.8 < ceiling_height < 8:
        raise ValueError("Implausible room height; inspect planes manually")
    if not np.all((trajectory_height > 0.2) & (trajectory_height < ceiling_height - 0.2)):
        raise ValueError("Sensor trajectory does not remain between detected floor/ceiling")
    return transform, {
        "imu_up_hint_in_map": up.tolist(), "floor": floor, "ceiling": ceiling,
        "floor_ceiling_normal_angle_degrees": parallel_angle,
        "ceiling_height_at_frame_origin_m": ceiling_height,
        "angle_between_map_plus_z_and_floor_up_degrees": float(np.degrees(np.arccos(normal[2]))),
        "trajectory_height_min_p01_p50_p99_max_m": np.percentile(trajectory_height, [0, 1, 50, 99, 100]).tolist(),
        "map_to_floor_matrix": transform.tolist(),
        "transform_convention": "floor_xyz = R @ map_xyz + t; right handed, meters, floor Z=0",
        "xy_heading": "map X projected onto floor; not a surveyed geographic heading",
        "pose_reoptimized": False,
    }


def height_rgb(height: np.ndarray, maximum: float):
    t = np.clip(height / maximum, 0, 1)
    channels = [np.rint(np.clip(1.5 - np.abs(4 * t - shift), 0, 1) * 255).astype(np.uint32)
                for shift in (3, 2, 1)]
    return (channels[0] << 16) | (channels[1] << 8) | channels[2]


def select_points(xyz: np.ndarray, radius: float, *, remove_isolated: bool):
    finite = np.isfinite(xyz).all(axis=1)
    indices = np.flatnonzero(finite)
    if remove_isolated:
        # Include the query point itself, so >=3 means two OTHER neighbors.
        neighbors = cKDTree(xyz[indices]).query_ball_point(
            xyz[indices], radius, return_length=True, workers=2)
        indices = indices[neighbors >= 3]
    return indices, finite


def reference_frame(path: Path, source_hash: str):
    """Reuse only the coordinate frame, never the reference's filtered point set."""
    reference = json.loads(path.read_text())
    if reference["source_sha256"] != source_hash:
        raise ValueError("Reference frame belongs to a different source PCD")
    transform = np.asarray(reference["map_to_floor_matrix"], dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(transform[:3, :3] @ transform[:3, :3].T,
                               np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-6, rtol=0)):
        raise ValueError("Reference transform is not a finite rigid right-handed transform")
    geometry_keys = (
        "imu_up_hint_in_map", "floor", "ceiling", "floor_ceiling_normal_angle_degrees",
        "ceiling_height_at_frame_origin_m", "angle_between_map_plus_z_and_floor_up_degrees",
        "trajectory_height_min_p01_p50_p99_max_m", "map_to_floor_matrix",
        "transform_convention", "xy_heading", "pose_reoptimized", "up_hint_source",
    )
    report = {key: copy.deepcopy(reference[key]) for key in geometry_keys if key in reference}
    report["frame_reference_processing"] = str(path.resolve())
    report["frame_reference_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    report["frame_reference_note"] = "Reused frame/plane diagnostics; no reference point filtering was applied."
    return transform, report


def write_records(path: Path, header, records, xyz, offsets, code, transform,
                  *, maximum: float | None = None):
    header = copy.deepcopy(header)
    count, step = records.shape
    extra = 4 if maximum is None else 8
    output = np.empty((count, step + extra), dtype=np.uint8)
    output[:, :step] = records
    for i, offset in enumerate(offsets):
        axis = np.ndarray((count,), dtype="<" + code, buffer=output,
                          offset=offset, strides=(step + extra,))
        axis[:] = xyz[:, i]
    stored_height = xyz[:, 2].astype("<f4")
    output[:, step:step + 4] = stored_height.view(np.uint8).reshape(-1, 4)
    header["FIELDS"].append("height")
    header["SIZE"].append("4")
    header["TYPE"].append("F")
    header.setdefault("COUNT", ["1"] * (len(header["FIELDS"]) - 1)).append("1")
    if maximum is not None:
        if not np.all(stored_height < maximum):
            raise ValueError("Height boundary violated after storage rounding")
        output[:, step + 4:] = height_rgb(stored_height, maximum).astype("<u4").view(np.uint8).reshape(-1, 4)
        for key, value in (("FIELDS", "rgb"), ("SIZE", "4"), ("TYPE", "F"), ("COUNT", "1")):
            header[key].append(value)
    header["WIDTH"], header["HEIGHT"], header["POINTS"] = [str(count)], ["1"], [str(count)]
    viewpoint = np.array(header.get("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"]), dtype=float)
    translation = transform[:3, :3] @ viewpoint[:3] + transform[:3, 3]
    rotation = Rotation.from_matrix(transform[:3, :3]) * Rotation.from_quat(viewpoint[[4, 5, 6, 3]])
    q = rotation.as_quat()
    header["VIEWPOINT"] = [str(v) for v in np.r_[translation, q[[3, 0, 1, 2]]]]
    with path.open("xb") as stream:
        stream.write(b"# .PCD v0.7 - floor-aligned derived copy\n")
        for key in ("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS", "DATA"):
            if key in header:
                stream.write((key + " " + " ".join(header[key]) + "\n").encode("ascii"))
        stream.write(output.tobytes())
    _, written_offset = _pcd_header(path)
    written = np.memmap(path, mode="r", dtype=np.uint8, offset=written_offset, shape=output.shape)
    if not np.array_equal(written, output):
        raise RuntimeError("Written output verification failed")


def write_views(directory: Path, aligned: np.ndarray, keep: np.ndarray, report, *, write_config=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import yaml

    points = aligned[keep]
    lo, hi = np.percentile(points, [0.2, 99.8], axis=0)
    center = (lo + hi) / 2
    span = max(float((hi - lo)[:2].max()), 5.)
    variant = ("OctoMap only - no isolated-point filter"
               if report.get("denoise", {}).get("enabled") is False
               else "Height 0m blue - 2m red")
    config = {
        "Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"}],
        "Visualization Manager": {
            "Class": "", "Enabled": True,
            "Global Options": {"Fixed Frame": "floor_map", "Background Color": "26; 29; 35", "Frame Rate": 20},
            "Displays": [
                {"Class": "rviz_default_plugins/Grid", "Name": "Floor Z=0", "Enabled": True,
                 "Cell Size": 1, "Plane Cell Count": 50, "Color": "90; 90; 90", "Alpha": .35,
                 "Plane": "XY", "Reference Frame": "<Fixed Frame>", "Value": True},
                {"Class": "rviz_default_plugins/PointCloud2", "Name": variant, "Enabled": True,
                 "Position Transformer": "XYZ", "Color Transformer": "RGB8", "Alpha": 1,
                 "Style": "Points", "Size (Pixels)": 2, "Size (m)": .025, "Decay Time": 0,
                 "Topic": {"Value": "/g1_mapping/floor_height_lt2", "Reliability Policy": "Reliable",
                           "Durability Policy": "Volatile", "History Policy": "Keep Last", "Depth": 1},
                 "Use Fixed Frame": True, "Value": True},
            ],
            "Tools": [{"Class": "rviz_default_plugins/Interact"}, {"Class": "rviz_default_plugins/MoveCamera"},
                      {"Class": "rviz_default_plugins/Select"}, {"Class": "rviz_default_plugins/FocusCamera"}],
            "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Distance": span * 1.25,
                                  "Focal Point": {"X": float(center[0]), "Y": float(center[1]), "Z": .5},
                                  "Pitch": .8, "Yaw": .78, "Near Clip Distance": .01,
                                  "Target Frame": "<Fixed Frame>", "Name": "Current View"}},
        },
        "Window Geometry": {"Width": 1500, "Height": 950, "X": 80, "Y": 40},
    }
    if write_config:
        with (directory / "height_view.rviz").open("x") as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
    rng = np.random.default_rng(4)
    sample = points[rng.choice(len(points), min(120000, len(points)), replace=False)]
    rgb = height_rgb(sample[:, 2], report["color_range_m"][1])
    colors = np.column_stack([(rgb >> shift) & 255 for shift in (16, 8, 0)]) / 255.
    fig = plt.figure(figsize=(16, 9))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.05, 1], left=.025, right=.97,
                           bottom=.09, top=.87, hspace=.42, wspace=.17)
    ax = fig.add_subplot(grid[:, 0], projection="3d")
    ax.scatter(*sample.T, c=colors, s=.3, depthshade=False, rasterized=True)
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(-.15, 2.05),
           xlabel="Floor-frame X [m]", ylabel="Floor-frame Y [m]", zlabel="",
           title="Floor-aligned 3D point cloud / height < 2 m")
    ax.set_zticks([0, 2], labels=["0 m", "2 m"])
    ax.tick_params(axis="z", labelsize=8)
    ax.set_box_aspect((hi[0] - lo[0], hi[1] - lo[1], 2.2))
    ax.view_init(elev=45, azim=-55)
    side = fig.add_subplot(grid[0, 1])
    all_sample = aligned[::max(1, len(aligned) // 100000)]
    side.scatter(all_sample[:, 0], all_sample[:, 2], s=.2, c="#4d6478", alpha=.4)
    side.axhline(0, color="#145dbe", label="Detected floor: 0 m")
    side.axhline(report["ceiling_height_at_frame_origin_m"], color="#333333", label="Detected ceiling")
    side.axhline(2, color="#d84435", linestyle="--", label="Crop boundary: 2 m")
    side.set(xlim=(lo[0], hi[0]), ylim=(-.3, report["ceiling_height_at_frame_origin_m"] + .3),
             xlabel="Floor-frame X [m]", ylabel="Height [m]", title="Full processed map: floor / ceiling check")
    side.legend(fontsize=8)
    hist = fig.add_subplot(grid[1, 1])
    hist.hist(aligned[:, 2], bins=np.arange(-.2, 3.21, .025), color="#4d6478")
    hist.axvline(0, color="#145dbe")
    hist.axvline(2, color="#d84435", linestyle="--")
    hist.axvline(report["ceiling_height_at_frame_origin_m"], color="#333333")
    hist.set(xlabel="Height above fitted floor [m]", ylabel="Points / 2.5 cm",
             title="Measured height distribution")
    fig.suptitle(variant + "; rigid floor alignment (not loop-closure optimization)\n"
                 "Color: 0 m blue / 0.5 m cyan / 1 m green / 1.5 m yellow / 2 m red", fontsize=12)
    fig.savefig(directory / "height_preview.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output_dir", type=Path)
    frame = parser.add_mutually_exclusive_group(required=True)
    frame.add_argument("--up", nargs=3, type=float)
    frame.add_argument("--reference-processing", type=Path,
                       help="reuse a checked frame from processing.json for this exact source PCD")
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--max-height", type=float, default=2.)
    parser.add_argument("--radius", type=float, default=.15, help="conservative isolated-return removal radius")
    parser.add_argument("--up-source", default="user-supplied measured map-frame up direction")
    parser.add_argument("--no-denoise", action="store_true",
                        help="retain every finite source point before the height crop; skip neighbor filtering")
    args = parser.parse_args()
    if not np.isfinite([args.max_height, args.radius]).all() or args.max_height <= 0 or args.radius <= 0:
        raise ValueError("Height and radius must be finite and positive")
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    source_hash = hashlib.sha256(args.source.read_bytes()).hexdigest()
    header, records, xyz, offsets, code = read_records(args.source)
    trajectory = np.loadtxt(args.trajectory)
    indices, finite = select_points(xyz, args.radius, remove_isolated=not args.no_denoise)
    print(f"[DENOISE] enabled={not args.no_denoise} finite={int(finite.sum())} retained={len(indices)}", flush=True)
    clean = xyz[indices]
    if args.reference_processing:
        transform, report = reference_frame(args.reference_processing, source_hash)
    else:
        transform, report = find_room_frame(clean, np.array(args.up), trajectory[:, 1:4])
    aligned = (clean @ transform[:3, :3].T + transform[:3, 3]).astype("<" + code)
    height = aligned[:, 2].astype(np.float32)
    keep = height < args.max_height
    if not keep.any():
        raise ValueError("Height crop would remove every point")
    args.output_dir.mkdir(parents=True)
    full = args.output_dir / "map_floor_aligned.pcd"
    write_records(full, header, records[indices], aligned, offsets, code, transform)
    crop_dir = args.output_dir / "height_lt_2m"
    crop_dir.mkdir()
    shutil.copy2(full, crop_dir / "map_source.pcd")
    cropped = crop_dir / "map_floor_height_lt_2m.pcd"
    write_records(cropped, header, records[indices[keep]], aligned[keep], offsets, code,
                  transform, maximum=args.max_height)
    corrected_trajectory = trajectory.copy()
    corrected_trajectory[:, 1:4] = trajectory[:, 1:4] @ transform[:3, :3].T + transform[:3, 3]
    corrected_trajectory[:, 4:8] = (Rotation.from_matrix(transform[:3, :3]) * Rotation.from_quat(trajectory[:, 4:8])).as_quat()
    np.savetxt(args.output_dir / "trajectory_floor.tum", corrected_trajectory, fmt="%.9f")
    report.update({
        "source": str(args.source.resolve()), "source_sha256": source_hash,
        "source_points": len(xyz), "nonfinite_removed": int((~finite).sum()),
        "isolated_points_removed": int(finite.sum()) - len(indices),
        "denoise": {"enabled": not args.no_denoise,
                    "radius_m": None if args.no_denoise else args.radius,
                    "minimum_other_neighbors": None if args.no_denoise else 2},
        "processed_points": len(indices), "cropped_points": int(keep.sum()),
        "height_cut_removed": int((~keep).sum()), "criterion": f"floor-relative stored height < {args.max_height} m",
        "cropped_height_min_max_m": [float(height[keep].min()), float(height[keep].max())],
        "color": "blue -> cyan -> green -> yellow -> red", "color_range_m": [0., args.max_height],
        "up_hint_source": report.get("up_hint_source", args.up_source),
        "trajectory_source": str(args.trajectory.resolve()),
        "non_xyz_source_fields_preserved": True,
        "full_map": str(full.resolve()), "crop": str(cropped.resolve()),
        "crop_sha256": hashlib.sha256(cropped.read_bytes()).hexdigest(),
        "limitations": ["Rigid frame correction only; no loop closure or absolute XY survey alignment.",
                        "No unobserved surfaces were filled; below-floor returns were not blindly clipped."],
    })
    if hashlib.sha256(args.source.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError("Input source changed during processing")
    with (args.output_dir / "processing.json").open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    write_views(crop_dir, aligned, keep, report)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
