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
SETTINGS_TOPICS="$(PYTHONPATH="$REAL_DIR/python${PYTHONPATH:+:$PYTHONPATH}" python3 -c \
    'from g1_mapping.config import Settings; s=Settings.load(); print(s.raw_points_topic); print(s.raw_imu_topic)')"
POINTS_TOPIC="${SETTINGS_TOPICS%%$'\n'*}"
IMU_TOPIC="${SETTINGS_TOPICS#*$'\n'}"
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
REPLAY_RECORD="${OUTPUT_NAME%.pcd}_$(date -u +%Y%m%dT%H%M%SZ)"

BAG_DIR="$REAL_DIR/runs/$SESSION/raw/rosbag2"
[[ -d "$BAG_DIR" ]] || { echo "[ERROR] bag が無い: $BAG_DIR" >&2; exit 1; }

echo "session : $SESSION"
echo "CropBox : $CROP"
[[ "$CROP" == true ]] && echo "マスク   : shape=$MASK_SHAPE range=$MASK_MIN_RANGE-$MASK_MAX_RANGE angle=+/-$MASK_REAR_ANGLE z=$MASK_MIN_Z-$MASK_MAX_Z"
echo "出力    : runs/$SESSION/map/$OUTPUT_NAME"
echo "再生    : rate=$RATE ${DURATION:+duration=${DURATION}s}"
echo "FAST-LIO: extrinsic_est=$EXTRINSIC_EST time_sync=$TIME_SYNC offset=$TIME_OFFSET"
echo "------------------------------------------------------------"

MAP_ENV_ARGS=()
[[ -f "$REAL_DIR/.env" ]] && MAP_ENV_ARGS+=(--env-file "$REAL_DIR/.env")
for key in MAP_VOXEL_SIZE MAP_MAX_POINTS DENSITY_TARGET_SCANS \
    OCTOMAP_RESOLUTION OCTOMAP_MIN_RANGE OCTOMAP_MAX_RANGE OCTOMAP_PROB_HIT \
    OCTOMAP_PROB_MISS OCTOMAP_CLAMP_MIN OCTOMAP_CLAMP_MAX OCTOMAP_KEEP_UNKNOWN; do
    if value="$(printenv "$key")"; then MAP_ENV_ARGS+=(-e "$key=$value"); fi
done

# 記録に /clock があればそれを再生し、無ければbagの再生時計を生成する。
# センサヘッダーの時刻とdb3の記録時刻が異なるシミュレーションにも対応する。
docker run --rm -i \
    "${MAP_ENV_ARGS[@]}" \
    -v "$REAL_DIR/runs:/runs" \
    -v "$REAL_DIR/ros2_ws/src/g1_mapping_bringup/launch:/opt/g1_ws/install/share/g1_mapping_bringup/launch:ro" \
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
    -e RAW_POINTS_TOPIC="$POINTS_TOPIC" \
    -e RAW_IMU_TOPIC="$IMU_TOPIC" \
    -e REPLAY_RECORD="$REPLAY_RECORD" \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    -e REPLAY_RATE="$RATE" \
    -e REPLAY_START="$START" \
    -e REPLAY_DURATION="$DURATION" \
    -v "$REAL_DIR/docker/replay-occupancy.sh:/opt/g1/bin/replay-occupancy.sh:ro" \
    "$IMAGE" bash /opt/g1/bin/replay-occupancy.sh

OUT="$REAL_DIR/runs/$SESSION/map/$OUTPUT_NAME"
echo "[REBUILD] ./mapctl rebuild $SESSION --method octomap --bag-dir runs/$SESSION/derived/$REPLAY_RECORD/rosbag2 --output runs/$SESSION/map/${OUTPUT_NAME%.pcd}_rebuilt.pcd"
echo "------------------------------------------------------------"
if [[ -f "$OUT" ]]; then
    echo "[OK] $OUT  ($(du -h "$OUT" | cut -f1))"
else
    echo "[ERROR] PCD が書かれていません: $OUT" >&2
    exit 1
fi
