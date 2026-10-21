"""Shared FAST-LIO2 scan normalization and occupancy mapping for live/replay."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    parameters = {
        "use_sim_time": use_sim_time,
        "output_path": LaunchConfiguration("output_path"),
        "voxel_size": float(os.environ.get("MAP_VOXEL_SIZE", "0.05")),
        "target_scans": int(os.environ.get("DENSITY_TARGET_SCANS", "10")),
        "resolution": float(os.environ.get("OCTOMAP_RESOLUTION", "0.10")),
        "min_range": float(os.environ.get("OCTOMAP_MIN_RANGE", "0.30")),
        "max_range": float(os.environ.get("OCTOMAP_MAX_RANGE", "3.0")),
        "prob_hit": float(os.environ.get("OCTOMAP_PROB_HIT", "0.7")),
        "prob_miss": float(os.environ.get("OCTOMAP_PROB_MISS", "0.4")),
        "clamp_min": float(os.environ.get("OCTOMAP_CLAMP_MIN", "0.12")),
        "clamp_max": float(os.environ.get("OCTOMAP_CLAMP_MAX", "0.97")),
        "keep_unknown": os.environ.get("OCTOMAP_KEEP_UNKNOWN", "true").lower() in {"true", "1", "yes"},
        "max_points": int(os.environ.get("MAP_MAX_POINTS", "2000000")),
    }
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("output_path"),
        Node(package="g1_mapping_tools", executable="mapping_adapter",
             name="g1_mapping_adapter", output="screen", parameters=[{
                 "source_odom": "/g1_mapping/backend/odom",
                 "source_cloud": "/g1_mapping/backend/cloud_registered",
                 "source_sensor_pose": "/g1_mapping/backend/sensor_pose",
                 "global_frame": "map", "child_frame_override": "lio_body",
                 "use_sim_time": use_sim_time,
             }]),
        Node(package="g1_mapping_octomap", executable="occupancy_mapper",
             name="g1_occupancy_mapper", output="screen", parameters=[parameters]),
    ])
