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

## 2つの運転モード（**再起動なしで切り替わる**）

| | 単純ゴール指定 | 巡回 |
|---|---|---|
| 入口 | RViz の 2D Goal Pose(`/goal_pose`) | `ros2 service call /g1/patrol/start std_srvs/srv/Trigger` |
| 準備 | 不要 | `patrol_waypoints:=<yaml>` |

📌 **`patrol_node.py` は既定で常駐するが `IDLE` で何もしない。**
`/g1/patrol/start` を呼ぶまで Goal を1件も出さないので、置いてあるだけの状態は
従来と完全に同じ挙動になる。巡回中に人が RViz から Goal を送ると、
**巡回のほうが退く**（`bt_navigator` は Goal を1件しか持てないため）。
ノードごと消したいときは `patrol:=false`。

## 使い方

    # モック(実機なし)
    ros2 launch g1_navigation navigation.launch.py

    # 実機
    ros2 launch g1_navigation navigation.launch.py \\
        backend:=real map:=/path/to/room_a_map.yaml

    # 実機 + 巡回（現地で記録したウェイポイントを読ませる）
    ros2 launch g1_navigation navigation.launch.py backend:=real \\
        map:=/path/to/room_a_map.yaml \\
        patrol_waypoints:=/home/unitree/g1_nav2/patrol_room_a.yaml

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
    # ⚠️ 既定は backend で変える。**モックは /tmp、実機(systemd)は /run**。
    # 2026-09-16: 既定を /run 固定にしたらモック構成が無言で壊れた
    # (state_bridge が繋がらない → TF が出ない → controller_server が activate できない)。
    sock_dir = LaunchConfiguration("bridge_sock_dir").perform(context).rstrip("/")
    if not sock_dir:
        sock_dir = "/run/g1_bridge" if is_real else "/tmp/g1_bridge"
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
            # ⚠️ **指令が何秒途切れたら FAULT にするか。** 既定 0.30 だと、復帰動作
            # (spin / DriveOnHeading)の切り替わりや Goal 到達の直後に必ず落ちる
            # (2026-09-24 実機で発生。約0.8m 歩いた直後に cmd_timeout で FAULT)。
            # 📌 **機体の安全は SDK 側が別に持っている。** systemd の
            # `--cmd-timeout 0.30` は据え置きなので、指令が途切れれば**機体は 0.3 秒で
            # ゼロ速度になる**。ここを伸ばして変わるのは「FAULT にして Goal ごと
            # 捨てるまでの猶予」だけで、**止まる速さは変わらない**。
            #
            # ⚠️ **2026-09-24 に 1.0 → 2.0 秒へ（利用者の判断）。** 1.0 でも復帰動作の
            # 切り替わりで足りなかったため。**上限は heartbeat と同じ 2.0 まで**とする。
            # これ以上にすると他の番人（heartbeat 2.0 / センサー鮮度 1.0 / TF 鮮度 0.5）の
            # ほうが先に発火するので、この番人は名目だけになる。
            # ⚠️ 伸ばすと増えるリスクは2つ:
            #   1. **無人のまま再開する窓が広がる**。長く詰まったあと Nav2 が
            #      **古い姿勢で計算した指令**を出して動き出しうる（FAULT なら Goal ごと捨てる）
            #   2. **状態表示が嘘をつく時間が伸びる**。ROS 側が死んでも 2 秒間は
            #      `NAVIGATING` のままで、UI も「走行中」と出し続ける
            "cmd_timeout": float(LaunchConfiguration("cmd_timeout").perform(context)),
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
        # ⚠️⚠️ **`cmd_vel` の付け替えが要る**(2026-09-24 に実機で判明)。
        # `behavior_server`(spin / backup / wait)は既定で **`/cmd_vel`** に出すが、
        # 下流は `controller_server` に合わせて `/cmd_vel_nav` → velocity_smoother →
        # `/cmd_vel_smoothed` と繋がっており、`g1_cmd_router` は最後だけを見ている。
        # 付け替えが無いと**復帰動作の指令はどこにも届かない**:
        #   実機ログ: `/cmd_vel 非ゼロ (vx=0.000, wz=1.000)` の裏で
        #             `/cmd_vel_smoothed` は**ゼロ**のまま
        # その結果 **BT が復帰動作に入るたびに指令が必ず途切れ**、`cmd_timeout` で
        # FAULT → Goal 取り消し → 人が clear_fault するまで停止、を繰り返していた。
        # ⚠️ **`cmd_timeout` をいくら伸ばしても直らない**（spin は 10 秒級で粘るため）。
        # 本家 nav2_bringup も velocity_smoother を使う構成では同じ付け替えをしている。
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             parameters=[configured_params],
             remappings=[("cmd_vel", "/cmd_vel_nav")]),
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

    # --- 巡回モード -----------------------------------------------------------
    # ⚠️ **ここで巡回が始まるわけではない。** ノードは IDLE で待つだけで、
    # `/g1/patrol/start` を呼ぶまで Goal を1件も出さない。だから既定で常駐させても
    # 単純ゴール指定モードの邪魔をしない（両モードを再起動なしで選べるようにするため）。
    if flag("patrol"):
        waypoints = LaunchConfiguration("patrol_waypoints").perform(context).strip()
        if not waypoints:
            # 既定は backend で変える。実機の room_a は**空のひな形**なので、
            # 現地で `tools/record_waypoints.py` を回すまで巡回は start を拒否する。
            waypoints = os.path.join(
                share, "config",
                "patrol_room_a.yaml" if is_real else "patrol_synthetic.yaml")
        nodes.append(Node(
            package="g1_navigation",
            executable="patrol_node.py",
            name="g1_patrol",
            output="screen",
            parameters=[{
                "waypoints_file": waypoints,
                "loop": flag("patrol_loop"),
                "dwell_s": float(LaunchConfiguration("patrol_dwell_s").perform(context)),
                "on_failure": LaunchConfiguration("patrol_on_failure").perform(context),
                "teach_output": LaunchConfiguration("patrol_teach_output").perform(context),
                # ⚠️ **既定 false。** 立ち上げただけで機体が歩き出さないようにする
                "autostart": flag("patrol_autostart"),
                "use_sim_time": use_sim_time,
            }],
        ))
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
        # 連結しておらず、経路計画のデモには使えない。
        # ⚠️ 実機では **room_a_map_20260911_edited.yaml** を渡す（2026-09-24）。
        # 実地の目視で通路と判断した 28 箇所を開けた版（maps/grids/EDITS.md）。
        # 手編集していない版・9/07 版・Sorasta 版も残してある。
        DeclareLaunchArgument(
            "map", default_value=os.path.join(share, "maps", "synthetic_room.yaml")),
        DeclareLaunchArgument(
            "cmd_timeout", default_value="2.0",
            description="指令の途切れを何秒で FAULT にするか(2026-09-24 に 0.30 → 1.0 → 2.0)"),
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
            "bridge_sock_dir", default_value="",
            description="SDK側プロセスの Unix socket の置き場。"
                        "空なら backend で決まる（real→/run/g1_bridge, mock→/tmp/g1_bridge）"),
        DeclareLaunchArgument(
            "map_to_odom", default_value="0 0 0",
            description="map->odom の初期値 \"dx dy yaw[rad]\"。find_map_offset.py の結果を渡す。"
                        "`none` なら map->odom を配信しない(連続localization を併用するとき)"),
        # --- 巡回モード（単純ゴール指定モードと併存する）----------------------
        DeclareLaunchArgument(
            "patrol", default_value="true",
            description="巡回ノードを常駐させる。⚠️ 常駐するだけで走り出さない"
                        "(/g1/patrol/start を呼ぶまで IDLE)。false でノードごと消す"),
        DeclareLaunchArgument(
            "patrol_waypoints", default_value="",
            description="巡回路の yaml。空なら backend で決まる"
                        "（real→config/patrol_room_a.yaml, mock→config/patrol_synthetic.yaml）"),
        DeclareLaunchArgument(
            "patrol_loop", default_value="true",
            description="最後の点まで行ったら1点目に戻る。false なら1周で終わる"),
        # ⚠️ **既定 0。** 1.3 秒を超えて止まると `cmd_timeout` で FAULT に落ちる
        # （velocity_timeout 1.0 + cmd_timeout 0.30。2026-09-16 モックで実測）。
        DeclareLaunchArgument(
            "patrol_dwell_s", default_value="0.0",
            description="各点で止まる秒数。⚠️ 1.3秒を超えると FAULT に落ちる。"
                        "点ごとに yaml の dwell_s で上書きできる"),
        # ⚠️ 空なら ~/g1_nav2/patrol_taught.yaml（無ければ ~/patrol_taught.yaml）。
        # **既定を patrol_waypoints にはしない。** 実機の既定は install/share 配下
        # （--symlink-install でリポジトリの実体）なので、上書きするとリポジトリが汚れる。
        DeclareLaunchArgument(
            "patrol_teach_output", default_value="",
            description="教示モード(RViz の Publish Point)の書き出し先 yaml"),
        DeclareLaunchArgument(
            "patrol_on_failure", default_value="skip",
            description="到達できない点の扱い。skip=飛ばして次へ / stop=巡回を止める"),
        # ⚠️ **既定 false のまま運用すること。** true にすると launch しただけで
        # 機体が歩き出す。発進ゲートと heartbeat は効くが、人の「開始」の意思が抜ける。
        DeclareLaunchArgument(
            "patrol_autostart", default_value="false",
            description="⚠️ 起動後に自動で巡回を始める。ベンチ/モック専用"),
        OpaqueFunction(function=_launch_setup),
    ])
