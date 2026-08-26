"""ライブ地図または保存PCDを、同じRViz設定で表示する。"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    pcd_path = LaunchConfiguration("pcd_path")
    fixed_frame = LaunchConfiguration("fixed_frame")
    map_topic = LaunchConfiguration("map_topic")
    rviz_enabled = LaunchConfiguration("rviz")
    config_path = (
        Path(get_package_share_directory("g1_mapping_visualization"))
        / "rviz"
        / "room_mapping.rviz"
    )

    saved_mode = IfCondition(PythonExpression(["'", mode, "' == 'saved'"]))
    pcd_publisher = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="g1_mapping_pcd_publisher",
        output="screen",
        condition=saved_mode,
        parameters=[
            {
                "file_name": ParameterValue(pcd_path, value_type=str),
                "tf_frame": ParameterValue(fixed_frame, value_type=str),
                # RVizが後から参加しても確実に受信できるよう、低頻度で再配信する。
                "publishing_period_ms": 2000,
            }
        ],
        remappings=[("/cloud_pcd", map_topic)],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="g1_mapping_rviz",
        output="screen",
        condition=IfCondition(rviz_enabled),
        arguments=["-d", str(config_path), "-f", fixed_frame],
        # onboardの実トピックもRViz設定側では共通名として扱う。
        remappings=[("/g1_mapping/map", map_topic)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="live",
                choices=["live", "saved"],
                description="live: ROS graphを表示、saved: PCDを再配信して表示",
            ),
            DeclareLaunchArgument("pcd_path", default_value=""),
            DeclareLaunchArgument("fixed_frame", default_value="camera_init"),
            DeclareLaunchArgument("map_topic", default_value="/g1_mapping/map"),
            DeclareLaunchArgument("rviz", default_value="true"),
            pcd_publisher,
            rviz,
        ]
    )
