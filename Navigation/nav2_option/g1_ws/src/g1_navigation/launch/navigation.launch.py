"""Nav2 一式の起動。**モック(実機なし)と実機の両方をここで賄う。**

`backend:=mock`（既定）と `backend:=real` で構成が変わる。**設定ファイルを二重に
持たない**ため、実機固有の差分はすべてこの1ファイルに集約してある
（分けると必ず片方だけ直して食い違う）。

## backend による違い

| | `mock` | `real` |
|---|---|---|
| センサー | `fake_sensor_publisher`(空の点群) | **G1 内蔵の MID-360** |
| `odom→base_link` TF | `g1_state_bridge` | **`g1_slam_odom_tf.py`**(内蔵SLAM の odom) |
| `map→odom` | 静的(dry-run 用のスタンドイン) | `g1_slam_odom_tf.py` が配信(ICP 合わせの結果) |
| `/odom` | `g1_state_bridge` | `g1_slam_odom_tf.py` |
| `g1_state_bridge` の役割 | odom と TF の供給源 | **状態監視のみ**(TF を止め `/odom` を改名) |

⚠️ **`real` で `g1_state_bridge` の TF/odom を止める理由。**
SDK 側の `MoveBackend` は `SetVelocity` しか持たず、**姿勢を読む口が無い**。
つまり `g1_state_bridge` が出す姿勢は**送った指令を積分しただけ**で、
実測で約19°横に逸れる機体では位置がすぐ破綻する。実機では内蔵 SLAM の
odometry（LiDAR+IMU の実測。静止70秒でドリフト 0.9cm）を使う。
`/odom` は `/g1/sdk_odom` に改名して残す（**両者を比較できると原因究明に効く**）。

📌 **観測源のトピック名はモックと実機で同じ**(`/utlidar/cloud_livox_mid360`)。
モック側が実機に合わせている。設定を2種類持つと必ず片方だけ直して食い違う。

⚠️ **`real` を使う前に、G1 の内蔵 SLAM を `1801` で起動しておくこと**
（`tools/send_slam_api.py`）。これが無いと `/unitree/slam_mapping/odom` が出ない。

## 使い方

    # モック(実機なし)
    ros2 launch g1_navigation navigation.launch.py

    # 実機
    ros2 launch g1_navigation navigation.launch.py \\
        backend:=real map:=/path/to/room_a_map.yaml

SDK 側プロセス(`g1_sdk_bridge_real_server`)は本 launch の対象外。
**ROS 環境を継承させないため systemd から起動する**(D-07、`deploy/` 参照)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# 実機の MID-360 は G1 内蔵で、**Livox のドライバを別に立てる必要は無い**。
# このトピック名は 2026-09-04 の記録と A-10b の配線検証で実測確認したもの。
# ⚠️ **モックと実機で同じ名前を使う。** モック専用の名前にすると
# 「モックでは通るのに実機で通らない」設定の食い違いを作り込む。
# `fake_sensor_publisher` の側がこの名前に合わせている。
SENSOR_TOPIC = "/utlidar/cloud_livox_mid360"

LIFECYCLE_NODES = [
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def _launch_setup(context, *args, **kwargs):
    """引数を解決してからノード構成を組み立てる。

    `IfCondition` を撒くより、**解決済みの値で素直に分岐する**ほうが読みやすく、
    「モックの設定のまま実機を動かす」事故も起きにくい。
    """
    share = get_package_share_directory("g1_navigation")
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in ("mock", "real"):
        raise RuntimeError(f"backend は mock か real: {backend}")
    is_real = backend == "real"

    params_file = LaunchConfiguration("params_file").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    sensor_topic = LaunchConfiguration("sensor_topic").perform(context) or SENSOR_TOPIC

    def flag(name: str) -> bool:
        return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")

    # ⚠️ **記録済み bag で再生検証するときは `use_sim_time:=true` が必須。**
    # bag のタイムスタンプは過去なので、壁時計のまま動かすと TF が常に
    # 「未来を要求している」と判定されて Nav2 も鮮度監視も成立しない。
    # `ros2 bag play --clock` と対で使う。
    use_sim_time = flag("use_sim_time")

    # 記録済み bag での再生検証のため `use_sim_time` を全ノードに行き渡らせる。
    # ⚠️ `param_rewrites` の**キーはパラメータ名**であって、値の文字列置換ではない
    # (一度そう誤解して観測源のトピックが書き換わらなかった。2026-09-13)。
    # 観測源のトピック名は YAML 側を実機に合わせてあるので書き換え不要。
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"use_sim_time": str(use_sim_time)},
        convert_types=True,
    )

    nodes = []

    # --- 位置と姿勢の供給源 ---------------------------------------------------
    if is_real:
        # 内蔵 SLAM の odom → odom→base_link TF / map→odom / base_link→livox_frame。
        # ⚠️ `--map-to-odom` を渡さないと map→odom は恒等変換になる。
        # 保存地図と合わせるには `tools/match_scan_to_map_2d.py` の結果を渡すこと。
        odom_args = ["--lidar-frame", LaunchConfiguration("lidar_frame").perform(context),
                     "--lidar-yaw", LaunchConfiguration("lidar_yaw").perform(context)]
        # 手順書 §7 の2分岐。`none` は「map->odom を出さない」= map_localizer.py に任せる。
        map_to_odom = LaunchConfiguration("map_to_odom").perform(context).strip()
        if map_to_odom == "none":
            odom_args.append("--no-map-to-odom")
        else:
            parts = map_to_odom.split()
            if len(parts) != 3:
                raise RuntimeError(
                    f'map_to_odom は "dx dy yaw" の3つか `none`: {map_to_odom!r}')
            odom_args += ["--map-to-odom", *parts]
        nodes.append(Node(
            package="g1_navigation",
            executable="g1_slam_odom_tf.py",
            name="g1_slam_odom_tf",
            output="screen",
            arguments=odom_args,
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    else:
        # dry-run 専用のスタンドイン。synthetic_room の自由空間に起点を置く
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static_tf",
            arguments=["-3", "-4", "0", "0", "0", "0", "map", "odom"],
        ))
        nodes.append(Node(
            package="g1_navigation",
            executable="fake_sensor_publisher.py",
            name="fake_sensor_publisher",
        ))

    # --- SDK 側との橋渡し -----------------------------------------------------
    # ⚠️ **ソケットの置き場は systemd ユニット(D-07)が /run/g1_bridge を指定している。**
    # 実行ファイルの既定は /tmp/g1_bridge なので、ROS 側をホストでネイティブに
    # (＝コンテナの bind mount 無しで)動かす本構成では、**明示的に合わせないと
    # 繋がらない**。2026-09-15 に、systemd 版と手起動版の2本が立っていて ROS 側が
    # 手起動版に繋がっていた事故を踏んだ。arm する前は必ず1本にすること。
    sock_dir = LaunchConfiguration("bridge_sock_dir").perform(context).rstrip("/")
    nodes.append(Node(
        package="g1_state_bridge",
        executable="g1_state_bridge_node",
        name="g1_state_bridge",
        # 実機では TF を止め、/odom を改名して衝突を避ける(冒頭のコメント参照)
        parameters=[{"publish_tf": not is_real, "use_sim_time": use_sim_time,
                     "state_sock_path": f"{sock_dir}/state.sock"}],
        remappings=[("/odom", "/g1/sdk_odom")] if is_real else [],
    ))
    nodes.append(Node(
        package="g1_cmd_router",
        executable="g1_cmd_router_node",
        name="g1_cmd_router",
        output="screen",
        parameters=[{
            "heartbeat_required": flag("heartbeat_required"),
            "operator_timeout_s": float(LaunchConfiguration("operator_timeout_s").perform(context)),
            "require_tf": flag("require_tf"),
            "require_sensor": flag("require_sensor"),
            # 鮮度監視の対象も backend に合わせる(見ていないトピックを監視しても無意味)
            "sensor_topic": sensor_topic,
            "use_sim_time": use_sim_time,
            "cmd_sock_path": f"{sock_dir}/cmd.sock",
        }],
    ))

    # --- Nav2 本体 ------------------------------------------------------------
    nodes += [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[configured_params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[configured_params], remappings=[("cmd_vel", "/cmd_vel_nav")]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[configured_params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             parameters=[configured_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             parameters=[configured_params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             parameters=[configured_params],
             remappings=[("cmd_vel", "/cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel_smoothed")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{"autostart": True, "node_names": LIFECYCLE_NODES,
                          "use_sim_time": use_sim_time}]),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory("g1_navigation")
    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="mock",
            description="mock=疑似データ(実機なし) / real=G1実機。構成が変わる"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        # 既定は Nav2 の配線検証用の合成地図(連結した自由空間を保証)。
        # A-7 で生成した test_room.yaml はレイトレーシング前のもので自由空間が
        # 連結しておらず、経路計画のデモには使えない。実機では room_a_map.yaml を渡す。
        DeclareLaunchArgument(
            "map", default_value=os.path.join(share, "maps", "synthetic_room.yaml")),
        DeclareLaunchArgument(
            "sensor_topic", default_value="",
            description=f"空なら {SENSOR_TOPIC}。モックも実機も同じ名前を使う"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="記録済み bag の再生検証で true。`ros2 bag play --clock` と対で使う"),
        DeclareLaunchArgument(
            "lidar_frame", default_value="livox_frame",
            description="点群の frame_id。real でのみ使う"),
        # ⚠️ **既定は 0 に戻した(2026-09-15)。** 以前は 180 を既定にしていたが、
        # あれは「最小回転で水平化する」実装が姿勢依存の yaw 誤差を注入していたことへの
        # 対症療法で、**0 でも 180 でも合わない姿勢が実機で出た**(軸の方位 -56.4°、
        # X の行き先 -112.8°)。水平化を heading_preserving_leveling に替えて
        # 構成上ずれないようにしたので、補正は不要になった。
        DeclareLaunchArgument(
            "lidar_yaw", default_value="0",
            description="base_link->livox_frame に足す yaw[度]。既定 0 のままでよい"),
        DeclareLaunchArgument(
            "heartbeat_required", default_value="true",
            description="操作PCの生存監視(D-31)。falseにすると通信断で止まらない。ベンチ試験専用"),
        DeclareLaunchArgument(
            "operator_timeout_s", default_value="1.0",
            description="heartbeatが何秒途絶したら停止するか。会場の電波状況に応じて調整する"),
        DeclareLaunchArgument(
            "require_tf", default_value="true",
            description="TFの鮮度をREADYの条件にする(仕様書7章)。falseはベンチ試験専用"),
        DeclareLaunchArgument(
            "require_sensor", default_value="true",
            description="センサーの鮮度をREADYの条件にする(仕様書7章)。falseはベンチ試験専用"),
        # ⚠️ ここが無いと、当日の手順書 §7 の**どちらの分岐も実行できない**
        # (`g1_slam_odom_tf.py` は両オプションを持つのに launch が公開していなかった。
        #  2026-09-15 に実機で気づいた)。
        # systemd ユニット(deploy/g1-sdk-bridge.service)が RuntimeDirectory= で
        # /run/g1_bridge に作る。手起動で実行ファイルの既定を使う場合だけ /tmp/g1_bridge。
        DeclareLaunchArgument(
            "bridge_sock_dir", default_value="/run/g1_bridge",
            description="SDK側プロセスの Unix socket の置き場。systemd 運用なら既定のまま"),
        DeclareLaunchArgument(
            "map_to_odom", default_value="0 0 0",
            description="map->odom の初期値 \"dx dy yaw[rad]\"。find_map_offset.py の結果を渡す。"
                        "`none` なら map->odom を配信しない(連続localization を併用するとき)"),
        OpaqueFunction(function=_launch_setup),
    ])
