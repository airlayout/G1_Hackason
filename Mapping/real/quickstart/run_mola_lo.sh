#!/usr/bin/env bash
# MOLA-LO をオフラインで記録に当て、TUM 形式の軌跡を出す。
#
# コンテナ rviz の中で実行する:
#   docker exec rviz bash /work/G1_Hackason/Mapping/real/quickstart/run_mola_lo.sh <SESSION> [--only-first-n N]
#
# 出力: runs/<SESSION>/mola/{traj.txt, run.log, <SESSION>.simplemap}
#
# 2026-09-07 に踏んだ詰まりどころ 4 件を全部織り込んである（詳細は
# docs/plan/2026-09-07_2-mola-lo-map-alignment.md §3 段 2）:
#   1. G1 は /tf を出さない          -> --base-link-frame-id livox_frame
#   2. センサラベルは CLI では届かない -> MOLA_LIDAR_NAME / MOLA_IMU_NAME
#   3. libmola_metric_maps.so が無い  -> ros-humble-mola-metric-maps が必要
#   4. RKNN が nanoflann>=1.5.1 要求  -> GICP を避け ICP パイプライン（下記 yaml）
# ROS の setup.bash は未定義変数を触るので -u は使わない（AMENT_TRACE_SETUP_FILES で落ちる）
set -eo pipefail

SESSION="${1:?usage: run_mola_lo.sh <SESSION> [extra mola args...]}"
shift || true

WORK=/work/G1_Hackason/Mapping/real
RUN="$WORK/runs/$SESSION"
BAG="$RUN/raw/rosbag2"
OUT="$RUN/${MOLA_OUT_NAME:-mola}"   # 対照実験は MOLA_OUT_NAME=mola_nograv などで分ける
PIPELINE="$WORK/quickstart/mola/g1_lidar3d_icp_imu.yaml"

[ -d "$BAG" ] || { echo "no bag: $BAG" >&2; exit 1; }
[ -f "$PIPELINE" ] || { echo "no pipeline: $PIPELINE" >&2; exit 1; }
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash

# 詰まり 3 の再発防止（コンテナを作り直すと消える）
if ! [ -e /opt/ros/humble/lib/aarch64-linux-gnu/libmola_metric_maps.so ]; then
  echo "[run_mola_lo] libmola_metric_maps.so が無い。ros-humble-mola-metric-maps を入れる" >&2
  sudo apt-get install -y ros-humble-mola-metric-maps >&2
fi

# 詰まり 2: LidarOdometry 側のラベルはパイプライン YAML の環境変数でしか渡らない
export MOLA_LIDAR_NAME="${MOLA_LIDAR_NAME:-/utlidar/cloud_livox_mid360}"
export MOLA_IMU_NAME="${MOLA_IMU_NAME:-/utlidar/imu_livox_mid360}"
# Livox の非反復スキャン向けに文書が指示している値
export MOLA_MIN_NEARBY_POSES_OCCUPIED="${MOLA_MIN_NEARBY_POSES_OCCUPIED:-2}"

echo "[run_mola_lo] session=$SESSION"
echo "[run_mola_lo] pipeline=$PIPELINE"
echo "[run_mola_lo] MOLA_IMU_GRAVITY_CORRECTION=${MOLA_IMU_GRAVITY_CORRECTION:-true(既定)}"
echo "[run_mola_lo] MOLA_IMU_GRAVITY_SIGMA_DEG=${MOLA_IMU_GRAVITY_SIGMA_DEG:-2.0(既定)}"

set -x
mola-lidar-odometry-cli \
  -c "$PIPELINE" \
  --state-estimator-param-file \
     /opt/ros/humble/share/mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml \
  --input-rosbag2 "$BAG" \
  --base-link-frame-id livox_frame \
  --lidar-sensor-label "$MOLA_LIDAR_NAME" \
  --imu-sensor-label "$MOLA_IMU_NAME" \
  --output-simplemap "$OUT/$SESSION.simplemap" \
  --output-tum-path "$OUT/traj.txt" \
  "$@" 2>&1 | grep -v "ETA=" | tee "$OUT/run.log"
set +x

# 測位のみモード（段 3）が読む .mm を作る。
# MOLA_SAVE_MM は「最後の局所地図」しか出さない（LO の局所地図は移動窓）ので、
# 全域の地図は .simplemap から sm2mm で作る。
# -l libmola_metric_maps.so が要る（詰まり 3 と同じプラグイン）。
if [ -f "$OUT/$SESSION.simplemap" ]; then
  echo "[run_mola_lo] sm2mm で全域の .mm を作る"
  sm2mm -i "$OUT/$SESSION.simplemap" -o "$OUT/map_full.mm" \
        -l libmola_metric_maps.so --no-progress-bar 2>&1 | grep -E "Final map|Writing|keyframes"
fi

echo "[run_mola_lo] 出力: $OUT"
wc -l "$OUT/traj.txt"
ls -la "$OUT"
