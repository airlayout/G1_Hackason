"""Nav2 + 疑似データでの動作確認用launch(実機なし)。

Planning.md「Nav2設定ファイル下書き+疑似データでの動作確認」に対応。

構成:
- map_server: A-7で生成したtest_room.yamlを配信(静的地図)
- static_transform_publisher: map -> odom (恒等変換。実際のlocalizationはPhase 2aで
  vendoringしたFAST-LIOに置き換える。このdry-runではNav2自体の配線検証が目的)
- g1_state_bridge: SDK側プロセスのstateから /odom と odom->base_link のTFを配信
- fake_sensor_publisher: 空の /scan, /g1/points_local を配信(costmapが詰まらないように)
- controller_server / planner_server / behavior_server / bt_navigator / velocity_smoother:
  仕様書の構成(Nav2生指令 -> Smoother -> /cmd_vel_smoothed)を再現
- g1_cmd_router: /cmd_vel_smoothed を安全処理してSDK側プロセスへIPC送信

SDK側プロセス(g1_sdk_bridge_mock_server 等)は本launchの対象外。別途起動しておくこと。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    g1_navigation_share = get_package_share_directory("g1_navigation")
    default_params = os.path.join(g1_navigation_share, "config", "nav2_params.yaml")
    # 既定はNav2の配線検証用の合成地図(連結した自由空間を保証)。A-7で生成した実点群由来の
    # test_room.yamlは、レイトレーシングをしていないため自由空間がほぼ連結しておらず
    # (最大連結成分3m^2未満)、経路計画のデモには使えなかった(既知の課題、Planning.md参照)。
    default_map = os.path.join(g1_navigation_share, "maps", "synthetic_room.yaml")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            # map -> odom はdry-run専用のスタンドイン(Phase 2aで本物のlocalizationに置き換える)。
            # synthetic_room.yaml の左下寄りの自由空間に疑似ロボットの起点を置く。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf",
                arguments=["-3", "-4", "0", "0", "0", "0", "map", "odom"],
            ),
            Node(
                package="g1_navigation",
                executable="fake_sensor_publisher.py",
                name="fake_sensor_publisher",
            ),
            Node(
                package="g1_state_bridge",
                executable="g1_state_bridge_node",
                name="g1_state_bridge",
            ),
            Node(
                package="g1_cmd_router",
                executable="g1_cmd_router_node",
                name="g1_cmd_router",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[params_file, {"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[params_file],
                remappings=[("cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[params_file],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[params_file],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[params_file],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                parameters=[params_file],
                remappings=[("cmd_vel", "/cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel_smoothed")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes, "use_sim_time": False}],
            ),
        ]
    )
