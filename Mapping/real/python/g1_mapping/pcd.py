"""依存ライブラリなしで行うPCDとセッション成果物の健全性検査。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import json
import math
import struct


@dataclass(frozen=True)
class ValidationCheck:
    name: str
    status: str
    message: str


def _pcd_header(path: Path) -> tuple[dict[str, list[str]], int]:
    header: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("PCDのDATA行がありません")
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ValueError("PCDヘッダーがASCIIではありません") from exc
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                return header, stream.tell()


def _point_count(header: dict[str, list[str]]) -> int:
    if "POINTS" in header:
        return int(header["POINTS"][0])
    return int(header.get("WIDTH", ["0"])[0]) * int(header.get("HEIGHT", ["1"])[0])


def _xyz_offsets(header: dict[str, list[str]]) -> tuple[int, int, int, int, str]:
    fields = header.get("FIELDS", [])
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCDのFIELDS/SIZE/TYPE/COUNTの長さが一致しません")
    offsets: dict[str, tuple[int, int, str]] = {}
    offset = 0
    for name, size, value_type, count in zip(fields, sizes, types, counts):
        offsets[name] = (offset, size, value_type)
        offset += size * count
    for axis in ("x", "y", "z"):
        if axis not in offsets:
            raise ValueError(f"PCDに{axis} fieldがありません")
    x, y, z = offsets["x"], offsets["y"], offsets["z"]
    if any(value[2] != "F" or value[1] not in (4, 8) for value in (x, y, z)):
        raise ValueError("x/y/zはfloat32またはfloat64である必要があります")
    if not (x[1] == y[1] == z[1]):
        raise ValueError("x/y/zのサイズが一致しません")
    return x[0], y[0], z[0], offset, "f" if x[1] == 4 else "d"


def inspect_pcd(path: Path, sample_limit: int = 100_000) -> dict[str, object]:
    header, data_offset = _pcd_header(path)
    count = _point_count(header)
    data_mode = header["DATA"][0].lower()
    x_offset, y_offset, z_offset, point_step, format_code = _xyz_offsets(header)
    finite_count = 0
    sampled_count = 0
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]

    def add_point(x: float, y: float, z: float) -> None:
        nonlocal finite_count, sampled_count
        sampled_count += 1
        if not all(math.isfinite(value) for value in (x, y, z)):
            return
        finite_count += 1
        for index, value in enumerate((x, y, z)):
            minimum[index] = min(minimum[index], value)
            maximum[index] = max(maximum[index], value)

    if data_mode == "ascii":
        fields = header["FIELDS"]
        axes = [fields.index(axis) for axis in ("x", "y", "z")]
        with path.open("rb") as stream:
            stream.seek(data_offset)
            for raw in stream:
                if sampled_count >= sample_limit:
                    break
                values = raw.decode("ascii").split()
                if len(values) < len(fields):
                    continue
                add_point(*(float(values[index]) for index in axes))
    elif data_mode == "binary":
        unpack = struct.Struct("<" + format_code).unpack_from
        stride = max(1, count // sample_limit)
        with path.open("rb") as stream:
            stream.seek(data_offset)
            for index in range(count):
                record = stream.read(point_step)
                if len(record) != point_step:
                    break
                if index % stride != 0:
                    continue
                add_point(
                    unpack(record, x_offset)[0],
                    unpack(record, y_offset)[0],
                    unpack(record, z_offset)[0],
                )
    else:
        raise ValueError(f"未対応のPCD DATA形式です: {data_mode}")

    extent = [
        maximum[index] - minimum[index] if finite_count else 0.0 for index in range(3)
    ]
    return {
        "path": str(path),
        "data_mode": data_mode,
        "points": count,
        "sampled_points": sampled_count,
        "finite_points": finite_count,
        "finite_ratio": finite_count / sampled_count if sampled_count else 0.0,
        "minimum_xyz": minimum if finite_count else [None, None, None],
        "maximum_xyz": maximum if finite_count else [None, None, None],
        "extent_xyz": extent,
        "size_bytes": path.stat().st_size,
    }


def validate_session(
    session_dir: Path,
    *,
    min_map_points: int,
    allow_partial: bool,
) -> tuple[bool, dict[str, object]]:
    checks: list[ValidationCheck] = []
    map_path = session_dir / "map" / "map_raw.pcd"
    pcd_info: dict[str, object] | None = None

    if not map_path.exists():
        checks.append(
            ValidationCheck(
                "map.pcd",
                "WARN" if allow_partial else "FAIL",
                "map_raw.pcdがありません",
            )
        )
    else:
        try:
            pcd_info = inspect_pcd(map_path)
            points = int(pcd_info["points"])
            finite_ratio = float(pcd_info["finite_ratio"])
            extent = [float(value) for value in pcd_info["extent_xyz"]]
            checks.append(
                ValidationCheck(
                    "map.points",
                    "PASS" if points >= min_map_points else "FAIL",
                    f"{points} points（必要 {min_map_points}以上）",
                )
            )
            checks.append(
                ValidationCheck(
                    "map.finite",
                    "PASS" if finite_ratio >= 0.99 else "FAIL",
                    f"有限値率 {finite_ratio:.3f}",
                )
            )
            horizontal_extent = max(extent[0], extent[1])
            checks.append(
                ValidationCheck(
                    "map.extent",
                    "PASS" if horizontal_extent >= 1.0 else "WARN",
                    f"XYZ範囲 {extent}",
                )
            )
        except (OSError, ValueError, KeyError, struct.error) as exc:
            checks.append(ValidationCheck("map.pcd", "FAIL", str(exc)))

    bag_dir = session_dir / "raw" / "rosbag2"
    metadata = bag_dir / "metadata.yaml"
    checks.append(
        ValidationCheck(
            "raw.rosbag2",
            "PASS" if metadata.exists() else ("WARN" if allow_partial else "FAIL"),
            "rosbag2 metadataあり" if metadata.exists() else "rosbag2 metadataがありません",
        )
    )

    trajectory = session_dir / "trajectory" / "trajectory.tum"
    trajectory_lines = 0
    if trajectory.exists():
        trajectory_lines = sum(
            1
            for line in trajectory.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.startswith("#")
        )
    checks.append(
        ValidationCheck(
            "trajectory",
            "PASS" if trajectory_lines >= 2 else "WARN",
            f"{trajectory_lines} poses",
        )
    )

    success = not any(check.status == "FAIL" for check in checks)
    report: dict[str, object] = {
        "schema_version": 1,
        "success": success,
        "checks": [asdict(check) for check in checks],
        "pcd": pcd_info,
    }
    report_path = session_dir / "report" / "quality.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return success, report
