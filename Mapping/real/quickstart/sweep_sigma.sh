#!/usr/bin/env bash
# 段 2b: **MOLA の事前の σ を再生で掃く。**
#
# 実機では同じ入力を繰り返せない。09-12 の blend 掃引は同じ条件が 6 倍ばらつき、
# 旋回のたびに向きが 200° 変わって各点が違う壁を見ていたので結論が出なかった。
# `mola-lidar-odometry-cli` は bag を**直接**食う（`ros2 bag play` も `/clock` も要らない）ので、
# **同じ入力を何度でも再現できる**。
#
# ⚠️ **CLI の rosbag2 入力には odometry のセンサ定義が無い**（バイナリに埋め込まれた
#    dataset yaml を実読して確認。lidar / gps / imu の 3 つだけ）。
#    つまり**この掃引に L2（/dog_odom の事前）は入っていない**。σ が効くのは
#    StateEstimationSimple 自身の等速度モデルの予測に対してである。
#    L2 込みの再現が要るなら `ros2 launch mola_lidar_odometry ...` 側（nav_stack.sh）を使う。
#
# 使い方（コンテナの外から）:
#   bash quickstart/sweep_sigma.sh <RUN_DIR名> [出力先]
#
#   G1_SIGMA_SET="10.0:0.1 2.0:0.1 0.5:0.05" bash quickstart/sweep_sigma.sh click_20260912T125919
#   （"ANGACC:ANG" の並び。ANGACC が既定 10.0 の項で、dt=0.1 s でも 57.6° を作っている犯人）
set -uo pipefail

NAME="${G1_RVIZ_NAME:-rviz}"
RUN="${1:?usage: sweep_sigma.sh <run_dir_name> [out_subdir]}"
OUT_SUB="${2:-sigma_sweep}"
WORK=/work/G1_Hackason/Mapping/real
BAG="$WORK/runs/$RUN/${G1_BAG_SUB:-bag}"
OUT="$WORK/runs/$RUN/$OUT_SUB"

# ANGACC:ANG の組。既定 -> 段 2a の導出値まで 2 桁を刻む
SIGMA_SET="${G1_SIGMA_SET:-10.0:0.1 2.0:0.1 0.5:0.05 0.1:0.02 0.00575:0.00438}"
BASE_FRAME="${G1_MOLA_BASE_FRAME:-base_link}"

# ── 測位のみモード（事前地図 map.mm の中で自分を出す）──────────────────
# 既定の LO モードは**自分で作った局所地図**に合わせる。live の崩壊は
# **事前地図 map.mm への scan-to-map** で起きたので、そちらも同じ入り口で測れるようにする。
# ⚠️ `map_full.mm`（sm2mm 製）では測位できない。ICP が見る localmap 層が無く品質 0.00 に張り付く
LOC_ENV=""
if [ "${G1_LOC_MODE:-0}" = "1" ]; then
    MAP="${G1_MOLA_MAP:-$WORK/runs/20260906T135940_UiS_room_v3/mola_floor0/map.mm}"
    docker exec "$NAME" test -f "$MAP" || { echo "地図が無い: $MAP" >&2; exit 1; }
    # [x y z yaw pitch roll]（角度は度）。**bag の最初の /tf をそのまま種にする**のが
    # 一番忠実。ここを地図の datum にすると「11 秒古い場所から始まる」型を踏む
    LOC_ENV="-e MOLA_LOAD_MM=$MAP -e MOLA_MAPPING_ENABLED=false"
    LOC_ENV="$LOC_ENV -e MOLA_INITIAL_X=${G1_INIT_X:-4.3614} -e MOLA_INITIAL_Y=${G1_INIT_Y:--0.3993}"
    LOC_ENV="$LOC_ENV -e MOLA_INITIAL_Z=${G1_INIT_Z:-0.0276} -e MOLA_INITIAL_YAW=${G1_INIT_YAW:--34.674}"
    LOC_ENV="$LOC_ENV -e MOLA_INITIAL_PITCH=${G1_INIT_PITCH:--0.004} -e MOLA_INITIAL_ROLL=${G1_INIT_ROLL:-2.515}"
    echo "[sweep] 測位のみモード: $MAP"
fi

docker exec "$NAME" test -d "$BAG" || { echo "bag が無い: $BAG" >&2; exit 1; }
docker exec -u ubuntu "$NAME" mkdir -p "$OUT"

for pair in $SIGMA_SET; do
    angacc="${pair%%:*}"; ang="${pair##*:}"
    tag="angacc${angacc}_ang${ang}"
    echo "=== σ_ANGACC=$angacc  σ_ANG=$ang  ==="
    # shellcheck disable=SC2086
    docker exec -u ubuntu $LOC_ENV \
        -e MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC="$angacc" \
        -e MOLA_NAVSTATE_SIGMA_ANG="$ang" \
        -e MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC="${G1_SIGMA_LINACC:-1.0}" \
        -e MOLA_NAVSTATE_SIGMA_POSITION="${G1_SIGMA_POS:-0.5}" \
        -e MOLA_LIDAR_NAME=/utlidar/cloud_livox_mid360 \
        -e MOLA_IMU_NAME=/utlidar/imu_livox_mid360 \
        -e MOLA_MIN_NEARBY_POSES_OCCUPIED=2 \
        "$NAME" bash -c "source /opt/ros/humble/setup.bash && \
        mola-lidar-odometry-cli \
          -c $WORK/quickstart/mola/g1_lidar3d_icp_imu.yaml \
          --state-estimator-param-file \
            /opt/ros/humble/share/mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml \
          --input-rosbag2 $BAG \
          --base-link-frame-id $BASE_FRAME \
          ${G1_LOC_MODE:+-l libmola_metric_maps.so} \
          --lidar-sensor-label /utlidar/cloud_livox_mid360 \
          --imu-sensor-label /utlidar/imu_livox_mid360 \
          --output-tum-path $OUT/$tag.tum \
          > $OUT/$tag.log 2>&1"
    rc=$?
    n=$(docker exec -u ubuntu "$NAME" bash -c "wc -l < $OUT/$tag.tum 2>/dev/null || echo 0")
    echo "  rc=$rc  軌跡 $n 行  -> $OUT/$tag.tum"
    [ "$rc" != "0" ] && docker exec -u ubuntu "$NAME" tail -5 "$OUT/$tag.log"
done
echo "出力: $OUT"
