"""記録済みの bag を再生して FAST-LIO2 で地図を作り直す（オフライン再構成）。

`raw_mapping.launch.py` は現場でライブに回すためのもの。こちらは**録ってある
db3 から作り直す**ためのもので、2 点だけ違う。

1. **入力段に CropBox を挟む**（`ENABLE_FOLLOWER_CROP=false` で外せる）
2. **`use_sim_time` を使う**。bag の再生時刻に合わせないと FAST-LIO2 が
   「未来のスキャン」を受け取ったと判断して捨てる

## なぜ入力段で落とせるのか

追従者はセンサ座標系ではほぼ定位置（真後ろ 1.0m）にいる。生 LiDAR は
`livox_frame` のままなので、**姿勢推定を一切必要とせずにマスクできる**。
「正しい姿勢を得るには追従者を消す必要があり、消すには姿勢が要る」という
循環に陥らない。内蔵 SLAM の点群は既に地図座標系へ変換済みなのでこれができない。

## なぜアダプタの前なのか

CropBox が扱えるのは `PointCloud2` だけ。`pointcloud_to_livox` が
Livox の `CustomMsg` へ変換した後では通せない。

    bag play → /utlidar/cloud_livox_mid360
                  ↓
             [CropBox negative=true]        ← ここ
                  ↓ /g1_mapping/points_masked
             pointcloud_to_livox
                  ↓ /g1_mapping/livox + /g1_mapping/imu
             fastlio_mapping → map_raw.pcd
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

RAW_POINTS_DEFAULT = "/utlidar/cloud_livox_mid360"
RAW_IMU_DEFAULT = "/utlidar/imu_livox_mid360"
MASKED_POINTS_TOPIC = "/g1_mapping/points_masked"


def _boolean_environment(name: str, default: str) -> bool:
    return os.environ.get(name, default).lower() in {"1", "true", "yes", "on"}


def _float_environment(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


def generate_launch_description() -> LaunchDescription:
    session_id = os.environ.get("MAPPING_SESSION_ID", "unknown")
    share = Path(get_package_share_directory("g1_mapping_bringup"))
    output_name = os.environ.get("MAP_OUTPUT_NAME", "map_fastlio.pcd")
    output_path = Path("/runs") / session_id / "map" / output_name

    raw_points = os.environ.get("RAW_POINTS_TOPIC", RAW_POINTS_DEFAULT)
    crop_enabled = _boolean_environment("ENABLE_FOLLOWER_CROP", "true")
    # CropBox を外したときはアダプタが生トピックを直接読む
    adapter_input = MASKED_POINTS_TOPIC if crop_enabled else raw_points

    nodes = []

    if crop_enabled:
        # ROS 2 Humble の pcl_ros はフィルタ群をビルドしていないので
        # （2.4.5 で add_library(pcl_ros_filters ...) がコメントアウト済み）、
        # 同じパラメータ名・topic 名の実装を g1_sensor_adapter に置いてある。
        nodes.append(Node(
            package="g1_sensor_adapter",
            executable="crop_box",
            name="crop_follower",
            output="screen",
            parameters=[
                str(share / "config" / "follower_crop.yaml"),
                {
                    "shape": os.environ.get("FOLLOWER_MASK_SHAPE", "rear_sector"),
                    "min_range": _float_environment("FOLLOWER_MASK_MIN_RANGE", 0.45),
                    "max_range": _float_environment("FOLLOWER_MASK_MAX_RANGE", 2.2),
                    "rear_half_angle_deg": _float_environment(
                        "FOLLOWER_MASK_REAR_ANGLE", 120.0),
                    "min_z": _float_environment("FOLLOWER_MASK_MIN_Z", -0.2),
                    "max_z": _float_environment("FOLLOWER_MASK_MAX_Z", 1.2),
                },
            ],
            remappings=[
                ("input", raw_points),
                ("output", MASKED_POINTS_TOPIC),
            ],
        ))

    nodes.append(Node(
        package="g1_sensor_adapter",
        executable="pointcloud_to_livox",
        name="g1_sensor_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_points_topic": adapter_input,
            "input_imu_topic": os.environ.get("RAW_IMU_TOPIC", RAW_IMU_DEFAULT),
            "output_points_topic": "/g1_mapping/livox",
            "output_imu_topic": "/g1_mapping/imu",
            "timestamp_mode": os.environ.get("POINT_TIMESTAMP_MODE", "auto"),
            "allow_inferred_time": _boolean_environment(
                "ALLOW_INFERRED_POINT_TIME", "true"),
            "scan_period_seconds": 0.1,
        }],
    ))

    nodes.append(Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            str(share / "config" / "fast_lio_mid360.yaml"),
            {
                "map_file_path": str(output_path),
                "use_sim_time": True,
                "mapping.extrinsic_est_en": _boolean_environment(
                    "FASTLIO_EXTRINSIC_EST", "false"),
                "common.time_sync_en": _boolean_environment(
                    "FASTLIO_TIME_SYNC", "false"),
                "common.time_offset_lidar_to_imu": _float_environment(
                    "FASTLIO_TIME_OFFSET", 0.0),
            },
        ],
        remappings=[
            ("/Odometry", "/g1_mapping/odom"),
            ("/cloud_registered", "/g1_mapping/cloud_registered"),
            ("/Laser_map", "/g1_mapping/map"),
        ],
    ))

    return LaunchDescription(nodes)
