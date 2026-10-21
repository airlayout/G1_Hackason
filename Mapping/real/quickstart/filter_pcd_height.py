#!/usr/bin/env python3
"""Copy a binary PCD, keep Z < max-z, and append height-based RGB colors.

Original point records (including density and occupancy) remain byte-identical.
The source is never overwritten. Requires only numpy, not ROS or Open3D.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd import _pcd_header, _point_count, _xyz_offsets  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-z", type=float, default=2.0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if not np.isfinite(args.max_z):
        raise ValueError("max-z must be finite")

    header, offset = _pcd_header(args.source)
    if header["DATA"] != ["binary"]:
        raise ValueError("only uncompressed binary PCD is supported")
    if set(header["FIELDS"]) & {"rgb", "rgba"}:
        raise ValueError("input already contains a color field")
    count = _point_count(header)
    x_offset, y_offset, z_offset, step, code = _xyz_offsets(header)
    payload = np.memmap(args.source, mode="r", dtype=np.uint8, offset=offset,
                        shape=(count, step))
    coordinates = np.stack([
        np.ndarray((count,), dtype="<" + code, buffer=payload, offset=axis,
                   strides=(step,))
        for axis in (x_offset, y_offset, z_offset)
    ], axis=1)
    if not np.isfinite(coordinates).all():
        raise ValueError("input contains nonfinite coordinates")
    keep = coordinates[:, 2] < args.max_z
    kept = int(keep.sum())
    if not kept:
        raise ValueError("height filter removed every point")
    selected = coordinates[keep]
    low, high = np.percentile(selected[:, 2], [1, 99])
    t = np.clip((selected[:, 2] - low) / max(float(high - low), 1e-9), 0, 1)
    channels = [np.clip(1.5 - np.abs(4 * t - shift), 0, 1)
                for shift in (3, 2, 1)]
    red, green, blue = [np.rint(channel * 255).astype(np.uint32) for channel in channels]
    packed_rgb = (red << 16) | (green << 8) | blue
    output_records = np.empty((kept, step + 4), dtype=np.uint8)
    output_records[:, :step] = payload[keep]
    output_records[:, step:] = packed_rgb.astype("<u4").view(np.uint8).reshape(-1, 4)

    header["FIELDS"].append("rgb")
    header["SIZE"].append("4")
    header["TYPE"].append("F")
    header.setdefault("COUNT", ["1"] * (len(header["FIELDS"]) - 1)).append("1")
    header["WIDTH"], header["HEIGHT"], header["POINTS"] = [str(kept)], ["1"], [str(kept)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        stream.write(b"# .PCD v0.7 - height-filtered copy\n")
        for key in ("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT",
                    "VIEWPOINT", "POINTS", "DATA"):
            if key in header:
                stream.write((key + " " + " ".join(header[key]) + "\n").encode("ascii"))
        stream.write(output_records.tobytes())

    # Verify the complete written payload, including the strict 2 m boundary.
    _, written_offset = _pcd_header(args.output)
    written = np.memmap(args.output, mode="r", dtype=np.uint8, offset=written_offset,
                        shape=output_records.shape)
    if not np.array_equal(written, output_records):
        raise RuntimeError("written point records do not match the filtered data")
    report = {
        "source": str(args.source.resolve()), "output": str(args.output.resolve()),
        "criterion": f"map-frame Z < {args.max_z} m", "source_points": count,
        "kept_points": kept, "removed_points": count - kept,
        "height_min_m": float(selected[:, 2].min()),
        "height_max_m": float(selected[:, 2].max()),
        "color": "blue (low) -> cyan -> green -> yellow -> red (high)",
        "color_range_m": [float(low), float(high)],
        "original_fields_preserved": True,
        "output_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
