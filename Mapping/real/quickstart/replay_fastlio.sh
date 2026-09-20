#!/usr/bin/env bash
# 記録済みの bag を再生して FAST-LIO2 で地図を作り直す。
#
# 内蔵 SLAM の点群は姿勢が座標に焼き込まれていて作り直せない。生 LiDAR + IMU から
# 姿勢を推定し直せるようになったのは 2026-09-04 に IMU を記録できるようになってから。
#
# 入力段で追従者を CropBox で落とす。生 LiDAR は livox_frame（センサ座標系）のままなので、
# 姿勢推定を一切必要とせずにマスクできる＝鶏卵問題にならない。
#
#   ./quickstart/replay_fastlio.sh <session_id>
#   ./quickstart/replay_fastlio.sh <session_id> --no-crop     # マスク無しで比較
#   ./quickstart/replay_fastlio.sh <session_id> --duration 60 # 先頭だけで試す
#   ./quickstart/replay_fastlio.sh <session_id> --shape rear_sector \
#       --min-range 0.45 --max-range 1.8 --rear-angle 90 --min-z -0.2 --max-z 1.2
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(dirname "$SCRIPT_DIR")"
IMAGE="${G1_MAPPING_IMAGE:-g1-mapping-raw:local}"

SESSION="${1:?使い方: $0 <session_id> [--no-crop] [--duration 秒] [--rate 倍率]}"
shift
CROP=true
OUTPUT_NAME=map_fastlio.pcd
DURATION=""
START=""
RATE=1.0
MASK_SHAPE=rear_sector
MASK_MIN_RANGE=0.45
MASK_MAX_RANGE=2.2
MASK_REAR_ANGLE=120.0
MASK_MIN_Z=-0.2
MASK_MAX_Z=1.2
EXTRINSIC_EST=false
TIME_SYNC=false
TIME_OFFSET=0.0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-crop)  CROP=false; OUTPUT_NAME=map_fastlio_nocrop.pcd; shift ;;
        --duration) DURATION="$2"; shift 2 ;;
        --rate)     RATE="$2"; shift 2 ;;
        --start)    START="$2"; shift 2 ;;
        --output)   OUTPUT_NAME="$2"; shift 2 ;;
        --shape)    MASK_SHAPE="$2"; shift 2 ;;
        --min-range) MASK_MIN_RANGE="$2"; shift 2 ;;
        --max-range) MASK_MAX_RANGE="$2"; shift 2 ;;
        --rear-angle) MASK_REAR_ANGLE="$2"; shift 2 ;;
        --min-z) MASK_MIN_Z="$2"; shift 2 ;;
        --max-z) MASK_MAX_Z="$2"; shift 2 ;;
        --extrinsic-est) EXTRINSIC_EST="$2"; shift 2 ;;
        --time-sync) TIME_SYNC="$2"; shift 2 ;;
        --time-offset) TIME_OFFSET="$2"; shift 2 ;;
        *) echo "不明な引数: $1" >&2; exit 2 ;;
    esac
done

BAG_DIR="$REAL_DIR/runs/$SESSION/raw/rosbag2"
[[ -d "$BAG_DIR" ]] || { echo "[ERROR] bag が無い: $BAG_DIR" >&2; exit 1; }

PLAY_ARGS=(--clock 200 --rate "$RATE")
[[ -n "$START" ]] && PLAY_ARGS+=(--start-offset "$START")
# ROS 2 Humble の rosbag2 には --playback-duration が無い（2026-09-04 に実測）。
# 途中で止めたいときはコンテナ内の timeout で切る。
PLAY_PREFIX=""
[[ -n "$DURATION" ]] && PLAY_PREFIX="timeout --signal=INT $DURATION"

echo "session : $SESSION"
echo "CropBox : $CROP"
[[ "$CROP" == true ]] && echo "マスク   : shape=$MASK_SHAPE range=$MASK_MIN_RANGE-$MASK_MAX_RANGE angle=+/-$MASK_REAR_ANGLE z=$MASK_MIN_Z-$MASK_MAX_Z"
echo "出力    : runs/$SESSION/map/$OUTPUT_NAME"
echo "再生    : rate=$RATE ${DURATION:+duration=${DURATION}s}"
echo "FAST-LIO: extrinsic_est=$EXTRINSIC_EST time_sync=$TIME_SYNC offset=$TIME_OFFSET"
echo "------------------------------------------------------------"

# --clock はシミュレーション時刻を配信する。これが無いと use_sim_time のノードが
# 時刻 0 のまま止まる。200 は配信レート[Hz]で、既定の 10Hz だと 10Hz の
# スキャンに対して粗すぎる。
docker run --rm -i \
    -v "$REAL_DIR/runs:/runs" \
    -v "$REAL_DIR/ros2_ws/src/g1_mapping_bringup/launch/offline_mapping.launch.py:/opt/g1_ws/install/share/g1_mapping_bringup/launch/offline_mapping.launch.py:ro" \
    -v "$REAL_DIR/ros2_ws/src/g1_mapping_bringup/config/follower_crop.yaml:/opt/g1_ws/install/share/g1_mapping_bringup/config/follower_crop.yaml:ro" \
    -e MAPPING_SESSION_ID="$SESSION" \
    -e ENABLE_FOLLOWER_CROP="$CROP" \
    -e MAP_OUTPUT_NAME="$OUTPUT_NAME" \
    -e FOLLOWER_MASK_SHAPE="$MASK_SHAPE" \
    -e FOLLOWER_MASK_MIN_RANGE="$MASK_MIN_RANGE" \
    -e FOLLOWER_MASK_MAX_RANGE="$MASK_MAX_RANGE" \
    -e FOLLOWER_MASK_REAR_ANGLE="$MASK_REAR_ANGLE" \
    -e FOLLOWER_MASK_MIN_Z="$MASK_MIN_Z" \
    -e FOLLOWER_MASK_MAX_Z="$MASK_MAX_Z" \
    -e FASTLIO_EXTRINSIC_EST="$EXTRINSIC_EST" \
    -e FASTLIO_TIME_SYNC="$TIME_SYNC" \
    -e FASTLIO_TIME_OFFSET="$TIME_OFFSET" \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    -e PLAY_PREFIX="$PLAY_PREFIX" \
    "$IMAGE" bash -c "
set -eo pipefail
ros2 launch g1_mapping_bringup offline_mapping.launch.py &
LAUNCH_PID=\$!

# 各A/B地図と同じ座標系の軌跡を残す。内蔵SLAMのodomをFAST-LIO2地図へ
# そのまま重ねると座標系が違い、ゴースト指標を誤判定するため。
mkdir -p /runs/$SESSION/trajectory
ODOM_CSV=/runs/$SESSION/trajectory/\${MAP_OUTPUT_NAME%.pcd}_odom.csv
ros2 topic echo --csv /g1_mapping/odom > \$ODOM_CSV &
ODOM_PID=\$!

# ノードが購読を張る前に再生を始めると先頭のスキャンを取りこぼす。
# トピックが現れるまで待つ（固定の sleep では機械の速さに依存して不安定）。
for i in \$(seq 60); do
    ros2 topic list 2>/dev/null | grep -q '/g1_mapping/livox' && break
    sleep 1
done

${PLAY_PREFIX} ros2 bag play /runs/$SESSION/raw/rosbag2 ${PLAY_ARGS[*]} || true

# **PCD は /map_save サービスで書かせる。**
# FAST-LIO2 が終了時に書く先は map_file_path ではなく
# ROOT_DIR/PCD/scans.pcd（= /opt/g1_ws/src/fast_lio/PCD/、コンテナ内でマウント外）
# なので、そのまま終わらせると地図が消える。laserMapping.cpp:1173 で確認した。
# map_file_path を使うのは save_to_pcd() だけで、それを呼ぶのは /map_save のみ。
echo '[replay] 再生完了。/map_save で PCD を書き出します ...'
ros2 service call /map_save std_srvs/srv/Trigger

kill -INT \$ODOM_PID 2>/dev/null || true
wait \$ODOM_PID 2>/dev/null || true

echo '[replay] 終了します ...'
kill -INT \$LAUNCH_PID 2>/dev/null || true
# FAST-LIO2 の ROS2 版は別スレッドを回しており SIGINT で素直に落ちないことがある。
# PCD は既に書けているので、待ちすぎずに打ち切る。
for i in \$(seq 20); do kill -0 \$LAUNCH_PID 2>/dev/null || break; sleep 1; done
kill -9 \$LAUNCH_PID 2>/dev/null || true
"

OUT="$REAL_DIR/runs/$SESSION/map/$OUTPUT_NAME"
echo "------------------------------------------------------------"
if [[ -f "$OUT" ]]; then
    echo "[OK] $OUT  ($(du -h "$OUT" | cut -f1))"
else
    echo "[ERROR] PCD が書かれていません: $OUT" >&2
    exit 1
fi
