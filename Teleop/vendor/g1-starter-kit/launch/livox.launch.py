"""頭部 Livox LiDAR のドライバ + 表示用 TF + RViz2 を起動する。

ドライバ同梱の launch は引数を持たず、切り替えのたびにファイル編集が要るため、
必要なものを launch 引数として外に出したもの。通常は ./scripts/lidar_view.sh 経由で使う。

    ros2 launch livox.launch.py                  # 通常（PointCloud2）
    ros2 launch livox.launch.py xfer_format:=1   # SLAM 用（Livox CustomMsg）
    ros2 launch livox.launch.py rviz:=false      # 記録だけしたいとき
    ros2 launch livox.launch.py flip:=false      # 上下反転をやめる

トピック: /livox/lidar (PointCloud2), /livox/imu (sensor_msgs/Imu)
フレーム: livox_frame（センサ）, viz_base（表示用の親。下記参照）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# このファイルは <repo>/launch/ に置かれている前提
REPO = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    xfer_format = LaunchConfiguration("xfer_format")
    config = LaunchConfiguration("config")
    publish_freq = LaunchConfiguration("publish_freq")
    rviz_config = LaunchConfiguration("rviz_config")
    flip_roll = LaunchConfiguration("flip_roll")

    args = [
        DeclareLaunchArgument("xfer_format", default_value="0",
                              description="0=PointCloud2 (RViz でそのまま見える) / 1=Livox CustomMsg (FAST-LIO 等)"),
        DeclareLaunchArgument("config",
                              default_value=os.path.join(REPO, "config", "livox", "active.json"),
                              description="Livox ドライバ設定 JSON。"
                                          "tools/lidar_probe.py --write-config で生成する。"
                                          "機種(Mid-360 / Mid-360S)で形式が違うので手書きしないこと"),
        DeclareLaunchArgument("publish_freq", default_value="10.0",
                              description="点群のパブリッシュ周波数[Hz]"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config",
                              default_value=os.path.join(REPO, "config", "rviz", "lidar_view.rviz"),
                              description="残像を出したいときは lidar_view_decay.rviz を指定"),
        DeclareLaunchArgument("flip", default_value="true",
                              description="LiDAR が上下逆さま搭載のとき true。"
                                          "表示用に 180 度回した親フレーム viz_base を立てる。"
                                          "判定は tools/lidar_probe.py --orientation"),
        DeclareLaunchArgument("flip_roll", default_value="3.14159265",
                              description="viz_base→livox_frame のロール[rad]。実点群を見て符号・軸を合わせること"),
    ]

    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": xfer_format,
            "multi_topic": 0,          # 全 LiDAR で同一トピック
            "data_src": 0,             # 0 = 実機 LiDAR
            "publish_freq": publish_freq,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "lvx_file_path": "",
            "user_config_path": config,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    # 表示用の親フレーム。点群と IMU は livox_frame 内で整合しているので、
    # ここでは「見た目を正立させる」だけに使う。SLAM 側の外部パラメータは触らない。
    #
    # flip=false でも TF 自体は必ず publish する（roll=0 の恒等変換にする）。
    # RViz の Fixed Frame が viz_base 固定なので、publish を止めてしまうと
    # 正立搭載の環境では点群がまったく表示されなくなる。
    flip_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="viz_base_to_livox",
        output="log",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", PythonExpression(
                       ["'", flip_roll, "' if '", LaunchConfiguration("flip"),
                        "'.lower() in ('true', '1') else '0.0'"]),
                   "--pitch", "0", "--yaw", "0",
                   "--frame-id", "viz_base", "--child-frame-id", "livox_frame"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["--display-config", rviz_config],
    )

    return LaunchDescription(args + [driver, flip_tf, rviz])
