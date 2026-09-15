#!/usr/bin/env bash
# **PC2（機体の Orin NX）で MOLA-LO を live で動かす。**
#
#   bash run_mola_live.sh                      # 地図なし（odometry のみ。疎通確認用）
#   bash run_mola_live.sh --map <map.mm>       # 事前地図の上で測位のみ
#   bash run_mola_live.sh --map <map.mm> --init "[x, y, 0.0, yaw, 0.0, 0.0]"
#   G1_PC2_SECONDS=60 bash run_mola_live.sh    # 動かす秒数（既定 0 = 無限）
#   G1_ODOM_TOPIC="" bash run_mola_live.sh     # 脚 odom を渡さない（事前情報なしの従来動作）
#
# ## なぜ PC2 で動かすのか
#
# 機体を無線で歩かせるとき、Mac のコンテナからだと DDS が AP を越えられない
# （multicast は L2 を越えない）。PC2 なら LiDAR と同じ機体内部の網に居るので
# ネットワークの問題が消える。描画も `foxglove_bridge` なら 0.06 コアで済む。
#
# ## 前提（2026-09-14 に全部踏んで潰した）
#
# 1. `~/jammy_ros/env.sh` が在ること（`pc2/setup_jammy_ros.sh` が作る）
# 2. ⚠️ **`jrun` が LD_LIBRARY_PATH を export しないこと（罠 5）。**
#    export すると `ros2 launch` の子で focal の /bin/sh が落ち、
#    「mola-cli が SIGSEGV」に見える。`pc2/write_jammy_env.sh` で直してある。
# 3. ⚠️ **`base_link -> livox_frame` の静的 TF を誰かが出すこと。**
#    コンテナでは `odom_to_tf.py` が出しているが PC2 には無いので、
#    ここで `static_transform_publisher` を起こす。無いと MOLA は
#    `lookupSensorPose ... "livox_frame" passed to lookupTransform` を延々と出し、
#    **スキャンは届いているのに 1 枚も処理されない**。
#    ⚠️ `ignore_lidar_pose_from_tf:=true` は代用にならない
#    （あれは「LiDAR は base_link の原点」とみなす指定で、取付角を渡せない）。
# 4. ⚠️ **既定のパイプライン `lidar3d-default.yaml` は使えない。**
#    arm64 Humble の mp2p_icp は nanoflann 1.4.2 で建っているので
#    `RKNN search requires nanoflann>=1.5.1` で**最初の 1 スキャンで落ちる**。
#    プロジェクトの `mola/g1_lidar3d_icp_imu.yaml` はこれを避けてある。
#
# ## 脚 odom を事前情報として渡す（2026-09-15 追加）
#
# 純正の脚 odometry `/dog_odom`（`nav_msgs/Odometry`・約 1008 Hz・静止 60 s で xy 2.7 mm）を
# MOLA に食わせる。launch 引数は **`odom_topic_name`**（`ros2-lidar-odometry.launch.py`
# の 384-388 行目で宣言され、環境変数 `MOLA_ODOM_TOPIC` になる）。
# 経路は BridgeROS2 → `CObservationRobotPose` → `state_estimation`
# （`mola::state_estimation_simple::StateEstimationSimple`）→ LidarOdometry の
# `state_.navstate_fuse` → **ICP の prior（コスト項）**。初期値だけではない。
#
# ⚠️ **`forward_ros_tf_odom_to_mola`（既定 False）とは排他。** 両方立てると launch の
#    `validate_odometry_sources()` が RuntimeError で止まる（同 75-118 行目）。
# ⚠️ ラベルの regex（`do_process_odometry_labels_re`）の既定は `.*` なので
#    `odom_sensor_label` は何でも通る（`Parameters.h` 75 行目）。
# ⚠️ `publish_tf_from_robot_pose_observations` は yaml で **false 固定**なので
#    （`mola-cli-launchs/lidar_odometry_ros2.yaml` 117 行目）、
#    base_link に親が 2 つ生える心配は無い。
#
# ## 取付値（live）
#
# `nav_stack.sh` の live と同じ `rpy 177.93 3.32 0 deg` / `xyz 0 0 1.228`。
# ⚠️ **offline（内蔵 odom が base_link）とは別の値**。取り違えると落ちずに数字だけ悪くなる。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
ROS="$JAMMY/rootfs/opt/ros/humble"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
PIPELINE="${G1_MOLA_PIPELINE:-$CFG/mola/g1_lidar3d_icp_imu.yaml}"
LIDAR_TOPIC="${G1_LIDAR_TOPIC:-/utlidar/cloud_livox_mid360}"
IMU_TOPIC="${G1_IMU_TOPIC:-/utlidar/imu_livox_mid360}"
# 脚 odom（上の「脚 odom を事前情報として渡す」を読むこと）。
# ⚠️ `:-` ではなく `-` を使う。**空文字を「渡さない」の意味にするため**
#    （`:-` だと空文字が既定値に化けて切れなくなる）。
ODOM_TOPIC="${G1_ODOM_TOPIC-/dog_odom}"
ODOM_LABEL="${G1_ODOM_SENSOR_LABEL:-odom_legs}"
# 機体の内部網。⚠️ wlan0 ではない（LiDAR は PC1 から eth0 側に出ている）
DDS_NIC="${G1_PC2_DDS_NIC:-eth0}"
DOMAIN="${G1_PC2_DOMAIN:-0}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"
# live の取付値（上の注記）
LIVOX_XYZ="${G1_LIVOX_XYZ:-0 0 1.228}"
LIVOX_RPY_DEG="${G1_LIVOX_RPY_DEG:-177.93 3.32 0}"

MAP=""; INIT_POSE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --map)  MAP="$2"; shift 2 ;;
        --init) INIT_POSE="$2"; shift 2 ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[pc2] 知らない引数: $1" >&2; exit 2 ;;
    esac
done

say() { echo "[pc2] $*"; }

[ -f "$JAMMY/env.sh" ] || { echo "[pc2] env.sh が無い: $JAMMY/env.sh（setup_jammy_ros.sh を先に）" >&2; exit 2; }
[ -f "$PIPELINE" ]     || { echo "[pc2] パイプラインが無い: $PIPELINE" >&2; exit 2; }
[ -z "$MAP" ] || [ -f "$MAP" ] || { echo "[pc2] 地図が無い: $MAP" >&2; exit 2; }

# shellcheck source=/dev/null
. "$JAMMY/env.sh"
export ROS_DOMAIN_ID="$DOMAIN"
# ⚠️ **NIC を 2 つにしたいときは `G1_PC2_DDS_URI` で丸ごと差し替える。**（2026-09-15）
#    eth0 だけだと participant の locator が 192.168.123.164 になり、AP 越しの
#    コンテナ（192.168.123.201）からは DDS が見えない。wlan0 も列挙すると見える。
if [ -n "${G1_PC2_DDS_URI:-}" ]; then
    export CYCLONEDDS_URI="$G1_PC2_DDS_URI"
else
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$DDS_NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi

# ── 静的 TF（前提 3）───────────────────────────────────────────────
# rpy[deg] -> rad。static_transform_publisher はラジアンを取る
read -r RX RY RZ <<EOF
$LIVOX_XYZ
EOF
read -r RR RP RYAW <<EOF
$(awk -v v="$LIVOX_RPY_DEG" 'BEGIN{split(v,a," "); printf "%.6f %.6f %.6f", a[1]*3.141592653589793/180, a[2]*3.141592653589793/180, a[3]*3.141592653589793/180}')
EOF
say "静的 TF base_link->livox_frame: xyz ($RX, $RY, $RZ) / rpy ${LIVOX_RPY_DEG} deg"
# ⚠️ **`jrun <実行ファイル>` で呼ばないこと（2026-09-14 に踏んだ）。**
# write_jammy_env.sh は $ROS/bin と $ROS/lib の ELF を**全部ラップ**するので、
# このパスは `#!/bin/sh` のラッパになっている。ローダにシェルスクリプトを
# 渡すと `error while loading shared libraries` で落ちる。
# `ros2 run` なら execve でラッパが正しく走る（Nav2 のノードと同じ経路）。
jros2 run tf2_ros static_transform_publisher \
    --x "$RX" --y "$RY" --z "$RZ" --roll "$RR" --pitch "$RP" --yaw "$RYAW" \
    --frame-id base_link --child-frame-id livox_frame > /tmp/pc2_stf.log 2>&1 &
STF_PID=$!
cleanup() { kill "$STF_PID" 2>/dev/null; wait "$STF_PID" 2>/dev/null; }
# ⚠️ 孤児を残さない。run_stage.sh で一度ディスクを埋めた前例がある
trap cleanup EXIT INT TERM
sleep 2
kill -0 "$STF_PID" 2>/dev/null || { echo "[pc2] 静的 TF が起動しない（/tmp/pc2_stf.log）" >&2; exit 1; }

# ── MOLA ────────────────────────────────────────────────────────────
ARGS="mola_lo_pipeline:=$PIPELINE lidar_topic_name:=$LIDAR_TOPIC imu_topic_name:=$IMU_TOPIC"
ARGS="$ARGS use_imu_for_lio:=True use_mola_gui:=False use_rviz:=False"
ARGS="$ARGS min_nearby_poses_occupied:=2 ignore_lidar_pose_from_tf:=false"
if [ -n "$ODOM_TOPIC" ]; then
    ARGS="$ARGS odom_topic_name:=$ODOM_TOPIC odom_sensor_label:=$ODOM_LABEL"
    say "脚 odom を事前情報に使う: $ODOM_TOPIC（ラベル $ODOM_LABEL）"
else
    say "⚠️ 脚 odom を渡さない（事前情報なし）。G1_ODOM_TOPIC が空"
fi
if [ -n "$MAP" ]; then
    # 測位のみ。地図は更新しない。map -> base_link を出させる
    ARGS="$ARGS start_mapping_enabled:=False mola_initial_map_mm_file:=$MAP"
    ARGS="$ARGS publish_localization_following_rep105:=False"
    ARGS="$ARGS initial_localization_method:=InitLocalization::FixedPose"
    [ -n "$INIT_POSE" ] && ARGS="$ARGS initial_pose:=\"$INIT_POSE\""
    say "測位のみ: 地図 $MAP"
    [ -n "$INIT_POSE" ] && say "初期姿勢: $INIT_POSE"
else
    say "⚠️ 地図なし（odometry のみ）。疎通確認にだけ使うこと"
fi

say "MOLA を起こす（${DDS_NIC} / domain ${DOMAIN}）"
# ⚠️ `timeout jros2 ...` は使えない。**jros2 はシェル関数**なので timeout が
# 実行ファイルとして探して `No such file or directory` になる（2026-09-14 に踏んだ）。
# ⚠️ 止めるときは **SIGINT**。SIGTERM だと内側の ros2 launch が子を孤児にする。
eval jros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py "$ARGS" &
MOLA_PID=$!
TIMER_PID=""
if [ "$SECONDS_LIMIT" != "0" ]; then
    say "${SECONDS_LIMIT} 秒で止める"
    ( sleep "$SECONDS_LIMIT"; kill -INT "$MOLA_PID" 2>/dev/null ) &
    TIMER_PID=$!
fi
wait "$MOLA_PID"
RC=$?
[ -n "$TIMER_PID" ] && kill "$TIMER_PID" 2>/dev/null
exit "$RC"
