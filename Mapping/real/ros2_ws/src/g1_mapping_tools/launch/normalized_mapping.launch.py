"""Backendのodom・登録点群を共通Mappingトピックへ接続する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    source_odom = LaunchConfiguration("source_odom")
    source_cloud = LaunchConfiguration("source_cloud")
    use_sim_time = LaunchConfiguration("use_sim_time")
    voxel_size = LaunchConfiguration("voxel_size")
    target_scan_count = LaunchConfiguration("target_scan_count")

    return LaunchDescription(
        [
            DeclareLaunchArgument("source_odom"),
            DeclareLaunchArgument("source_cloud"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("voxel_size", default_value="0.05"),
            DeclareLaunchArgument("target_scan_count", default_value="10"),
            Node(
                package="g1_mapping_tools",
                executable="mapping_adapter",
                name="g1_mapping_adapter",
                output="screen",
                parameters=[
                    {
                        "source_odom": source_odom,
                        "source_cloud": source_cloud,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package="g1_mapping_tools",
                executable="map_accumulator",
                name="g1_map_accumulator",
                output="screen",
                parameters=[
                    {
                        "voxel_size": ParameterValue(voxel_size, value_type=float),
                        "target_scan_count": ParameterValue(
                            target_scan_count, value_type=int
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
