"""実機なしでセッションライフサイクルを検証するmock成果物。"""

from __future__ import annotations

from pathlib import Path
import math
import sqlite3

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
    database = sqlite3.connect(bag_dir / "mock_0.db3")
    try:
        database.executescript(
            """
            CREATE TABLE topics(
                id INTEGER PRIMARY KEY,
                name TEXT NOT NULL,
                type TEXT NOT NULL,
                serialization_format TEXT NOT NULL,
                offered_qos_profiles TEXT NOT NULL
            );
            CREATE TABLE messages(
                id INTEGER PRIMARY KEY,
                topic_id INTEGER NOT NULL,
                timestamp INTEGER NOT NULL,
                data BLOB NOT NULL
            );
            """
        )
        topics = [
            (1, "/utlidar/cloud_livox_mid360", "sensor_msgs/msg/PointCloud2"),
            (2, "/utlidar/imu_livox_mid360", "sensor_msgs/msg/Imu"),
            (3, "/g1_mapping/odom", "nav_msgs/msg/Odometry"),
            (4, "/g1_mapping/cloud_registered", "sensor_msgs/msg/PointCloud2"),
            (5, "/g1_camera/color/image/compressed", "sensor_msgs/msg/CompressedImage"),
            (6, "/g1_camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
            (7, "/g1_camera/frame_metadata", "std_msgs/msg/String"),
        ]
        database.executemany(
            "INSERT INTO topics VALUES (?, ?, ?, 'cdr', '')", topics
        )
        database.executemany(
            "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)",
            [(topic_id, topic_id * 1_000_000, b"mock") for topic_id, _, _ in topics],
        )
        database.commit()
    finally:
        database.close()
