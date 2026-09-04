"""G1 raw PointCloud2/IMUからFAST-LIO2 Mappingを起動する。"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _boolean_environment(name: str, default: str) -> bool:
    return os.environ.get(name, default).lower() in {"1", "true", "yes", "on"}


def generate_launch_description() -> LaunchDescription:
    session_id = os.environ.get("MAPPING_SESSION_ID", "unknown")
    use_sim_time = _boolean_environment("USE_SIM_TIME", "false")
    output_path = Path("/runs") / session_id / "map" / "map_raw.pcd"
    config_path = (
        Path(get_package_share_directory("g1_mapping_bringup"))
        / "config"
        / "fast_lio_mid360.yaml"
    )

    adapter = Node(
        package="g1_sensor_adapter",
        executable="pointcloud_to_livox",
        name="g1_sensor_adapter",
        output="screen",
        parameters=[
            {
                "input_points_topic": os.environ.get(
                    "RAW_POINTS_TOPIC", "/utlidar/cloud_livox_mid360"
                ),
                "input_imu_topic": os.environ.get(
                    "RAW_IMU_TOPIC", "/utlidar/imu_livox_mid360"
                ),
                "output_points_topic": "/g1_mapping/livox",
                "output_imu_topic": "/g1_mapping/imu",
                "timestamp_mode": os.environ.get("POINT_TIMESTAMP_MODE", "auto"),
                "allow_inferred_time": _boolean_environment(
                    "ALLOW_INFERRED_POINT_TIME", "true"
                ),
                "scan_period_seconds": 0.1,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            str(config_path),
            {"map_file_path": str(output_path), "use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/Odometry", "/g1_mapping/backend/odom"),
            ("/cloud_registered", "/g1_mapping/backend/cloud_registered"),
            ("/Laser_map", "/g1_mapping/backend/map"),
        ],
    )
    normalizer = Node(
        package="g1_mapping_tools",
        executable="mapping_adapter",
        name="g1_mapping_adapter",
        output="screen",
        parameters=[
            {
                "source_odom": "/g1_mapping/backend/odom",
                "source_cloud": "/g1_mapping/backend/cloud_registered",
                "global_frame": "map",
                "fallback_child_frame": "lio_body",
                "use_sim_time": use_sim_time,
            }
        ],
    )
    accumulator = Node(
        package="g1_mapping_tools",
        executable="map_accumulator",
        name="g1_map_accumulator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    return LaunchDescription([adapter, fast_lio, normalizer, accumulator])
