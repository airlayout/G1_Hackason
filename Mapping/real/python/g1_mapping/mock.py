"""実機なしでセッションライフサイクルを検証するmock成果物。"""

from __future__ import annotations

from pathlib import Path
import math

from .session import MappingSession


def write_mock_artifacts(session: MappingSession) -> None:
    points: list[tuple[float, float, float, float]] = []
    for index in range(1200):
        angle = 2.0 * math.pi * index / 1200
        radius = 2.0 + 0.1 * math.sin(angle * 4.0)
        points.append((radius * math.cos(angle), radius * math.sin(angle), 1.0, 80.0))

    pcd_lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z intensity",
        "SIZE 4 4 4 4",
        "TYPE F F F F",
        "COUNT 1 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    pcd_lines.extend(" ".join(f"{value:.6f}" for value in point) for point in points)
    session.map_path.write_text("\n".join(pcd_lines) + "\n", encoding="ascii")

    trajectory = session.directory / "trajectory" / "trajectory.tum"
    trajectory.write_text(
        "# timestamp tx ty tz qx qy qz qw\n"
        "0.000000 0 0 0 0 0 0 1\n"
        "1.000000 1 0 0 0 0 0 1\n"
        "2.000000 0 0 0 0 0 0 1\n",
        encoding="utf-8",
    )

    bag_dir = session.directory / "raw" / "rosbag2"
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  version: 5\n  storage_identifier: mock\n",
        encoding="utf-8",
    )
