#!/usr/bin/env bash
# **実時間で再生して MOLA を回す。**`mola-lidar-odometry-cli` の再生との違いを埋めるため。
#
# ## なぜ要るか（2026-09-12 の掃引で分かったこと）
#
# CLI 再生は σ を 10.0 -> 0.1 に締めても、レートを 9.9 -> 1.8 Hz に落としても、
# LO / 測位のみのどちらでも、**live の崩壊（M1 = 5.91 m / M2 = -91.7%）を再現しない**
# （どれも M1 ~ 0.28 m / M2 ~ +0.7%）。CLI は自分のペースで全スキャンを処理するので、
# 残っている差は
#   1. **実時間の計算予算**（live は RViz2 175〜204% + Nav2 + 録画と 4 コアを取り合っていた）
#   2. **L2（/dog_odom の事前）**。⚠️ **CLI の rosbag2 入力には odometry のセンサ定義が無い**
#      ので、掃引には L2 が入っていない。live は L2 ON だった
# の 2 つだけである。ここはその 2 つを同時に載せる唯一の経路。
#
# ⚠️ **bag の /tf は再生しない。** 当時の MOLA の出力がそのまま流れて、
#    これから測る MOLA の出力と衝突する（`base_link` の親が 2 つになる）。
#    `/tf_static` は再生する（map->odom 恒等・base_link->livox_frame・base_footprint が入っており、
#    odom_to_tf.py が live で流していたものと同じ値である）。
#
# 使い方:
#   bash quickstart/replay_realtime.sh click_20260912T125919 rt_base
#   G1_RT_LOAD=2 bash quickstart/replay_realtime.sh click_20260912T125919 rt_load2   # CPU を 2 本食わせる
#   G1_LEG_ODOM=0 bash quickstart/replay_realtime.sh click_20260912T125919 rt_nol2
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"
NAME="$G1_RVIZ_NAME"
RUN="${1:?usage: replay_realtime.sh <run_dir_name> <out_subdir>}"
OUT_SUB="${2:-rt}"
WORK=/work/G1_Hackason/Mapping/real
BAG="$WORK/runs/$RUN/${G1_BAG_SUB:-bag}"
OUT="$WORK/runs/$RUN/$OUT_SUB"
MAP="${G1_MOLA_MAP:-$WORK/runs/20260906T135940_UiS_room_v3/mola_floor0/map.mm}"
PIPELINE="$WORK/quickstart/mola/g1_lidar3d_icp_imu.yaml"
RATE="${G1_RT_RATE:-1}"
DELAY="${G1_BAG_DELAY:-18}"
LOAD="${G1_RT_LOAD:-0}"
# live と同じ [x y z yaw pitch roll]（度）。既定は click の bag の最初の /tf
# ⚠️ **initial_pose の \" \" を外さないこと。** 外すと ros2 launch はカンマ区切りの
# **リスト**として受け取り、生成する YAML が `wrongly indented double-quoted scalar` で壊れ、
# MOLA は `Access non-existing map key params` で**黙って即終了する**（/tf が 1 本も出ない）。
# 逆に空白を消すと `ros2: error: unrecognized arguments` になる。**空白つき + クォート**が正解
INIT="${G1_MOLA_INIT_POSE:-[4.3614, -0.3993, 0.0276, -34.674, -0.004, 2.515]}"
LEG="${G1_LEG_ODOM:-1}"
LEG_ARGS=""
[ "$LEG" = "1" ] && LEG_ARGS="odom_topic_name:=/dog_odom odom_sensor_label:=odom_legs"

DDS="$(g1_dds_uri offline)"
say() { echo "[rt] $*"; }
dex() { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
        bash -c "source /opt/ros/humble/setup.bash && $*"; }
# 事前の σ。**ここを振るのが段 2b の本体**（負荷をかけて初めて意味を持つ）
SIG_ENV="-e MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC=${G1_SIGMA_ANGACC:-10.0}"
SIG_ENV="$SIG_ENV -e MOLA_NAVSTATE_SIGMA_ANG=${G1_SIGMA_ANG:-0.1}"
SIG_ENV="$SIG_ENV -e MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC=${G1_SIGMA_LINACC:-1.0}"
SIG_ENV="$SIG_ENV -e MOLA_NAVSTATE_SIGMA_POSITION=${G1_SIGMA_POS:-0.5}"
# 欠測がこれを超えると **事前そのものが消える**（"Not able to use velocity motion model"）。
# σ をどう締めてもこの区間では効かないので、CPU 競合下では本命の調整つまみになる
SIG_ENV="$SIG_ENV -e MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL=${G1_MAX_VEL_TIME:-0.75}"

spawn() { local log="$1"; shift
    # shellcheck disable=SC2086
    docker exec -d -u ubuntu $SIG_ENV -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
        bash -c "source /opt/ros/humble/setup.bash && $* > $OUT/$log 2>&1"; }

docker exec "$NAME" test -d "$BAG" || { echo "bag が無い: $BAG" >&2; exit 1; }
docker exec "$NAME" test -f "$MAP" || { echo "地図が無い: $MAP" >&2; exit 1; }
bash "$HERE/nav_stack.sh" stop >/dev/null 2>&1
docker exec -u ubuntu "$NAME" bash -c "rm -rf $OUT && mkdir -p $OUT"

say "L2=$LEG / 負荷 $LOAD 本(nice ${G1_RT_LOAD_NICE:-0}) / ${RATE}倍速 / σ_ANGACC=${G1_SIGMA_ANGACC:-10.0} σ_ANG=${G1_SIGMA_ANG:-0.1}"

# CPU の競合を作る（live の RViz2 175〜204% + Nav2 を模す）。**MOLA より先に走らせる**
if [ "$LOAD" != "0" ]; then
    for i in $(seq 1 "$LOAD"); do
        # G1_RT_LOAD_NICE=19 にすると負荷が MOLA に道を譲る。
        # 「CPU を空ければ直るのか」＝ 運用で直せるのかを測るための対照
        docker exec -d "$NAME" nice -n "${G1_RT_LOAD_NICE:-0}" bash -c 'while :; do :; done' >/dev/null
    done
    say "負荷 $LOAD 本を投入した"
fi

# ── 測位エンジンを選ぶ ────────────────────────────────────────────────
# mola    … MOLA-LO の測位のみモード（事前地図 map.mm・単層の scan-to-map）
# fastlio … FAST-LIO2 単体（C5 に同梱の内側の層。iEKF・200 Hz の IMU で姿勢を伝播）
#           ⚠️ 事前地図を読まないので出すのは odometry（camera_init -> body）。
#           M1（純回転中の端から端の変位）と M2（正味 Δyaw の比）は**相対量**なので
#           odometry の座標系のままで測れる
ENGINE="${G1_RT_ENGINE:-mola}"
if [ "$ENGINE" = "fastlio" ]; then
    say "[1] FAST-LIO2 を起こす（C2。事前地図なしの odometry）"
    spawn mola.log "source /work/third_party/ws_c5/install/setup.bash && \
        ros2 launch fast_lio mapping.launch.py \
        config_path:=/work/G1_Hackason/Mapping/real/quickstart/fastlio \
        config_file:=g1_mid360_pc2.yaml rviz:=false use_sim_time:=true"
    # ⚠️ このフォークは topic を /Odometry_loc に改名している（本家は /Odometry）
    RECORD_TOPIC="/Odometry_loc"
else
    RECORD_TOPIC="/tf"
    say "[1] MOLA を測位のみモードで起こす（map -> base_link を直接出す）"
    spawn mola.log "ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
    mola_lo_pipeline:=$PIPELINE \
    lidar_topic_name:=/utlidar/cloud_livox_mid360 \
    imu_topic_name:=/utlidar/imu_livox_mid360 \
    use_imu_for_lio:=True $LEG_ARGS \
    min_nearby_poses_occupied:=2 \
    start_mapping_enabled:=False \
    mola_initial_map_mm_file:=$MAP \
    initial_localization_method:=InitLocalization::FixedPose \
    initial_pose:=\"$INIT\" \
    publish_localization_following_rep105:=False \
    mola_bridge_odometry_frame:=mola_odom \
    ignore_lidar_pose_from_tf:=false \
    use_mola_gui:=False use_rviz:=False use_sim_time:=true"
fi

# ⚠️ **MOLA が立ったことを確かめてから進む。** 設定を 1 文字間違えると
# `mola-cli` は数百 ms で終了し、そのあと再生も録画も**成功したように**流れて
# 空の bag が残る（2026-09-12 に 2 回踏んだ）。ここで落とす
PROC_PAT="mola-cl[i]"; [ "$ENGINE" = "fastlio" ] && PROC_PAT="fastlio_mappin[g]"
until docker exec "$NAME" bash -c "pgrep -f \"$PROC_PAT\" >/dev/null" 2>/dev/null; do
    sleep 1
    if grep -q "Entering main MOLA" <(docker exec -u ubuntu "$NAME" cat "$OUT/mola.log" 2>/dev/null); then :; fi
    if docker exec -u ubuntu "$NAME" grep -q "has finished cleanly\|Error:" "$OUT/mola.log" 2>/dev/null; then
        say "⛔ MOLA が起動に失敗した:"; docker exec -u ubuntu "$NAME" grep -m5 -A3 "Error\|error" "$OUT/mola.log"; exit 1
    fi
done
say "    MOLA 起動を確認した"

say "[2] $RECORD_TOPIC を録る"
spawn record.log "ros2 bag record -o $OUT/tf_bag $RECORD_TOPIC --use-sim-time"

say "[3] 記録を再生する（⚠️ /tf は再生しない。当時の MOLA の出力と衝突するため）"
spawn bagplay.log "ros2 bag play $BAG --rate $RATE --clock --delay $DELAY \
    --topics /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /dog_odom /tf_static"

# 再生が終わるまで待つ。⚠️ `sleep N` を連ねるとブロックされるので until で回す
say "[4] 再生の終了を待つ"
# ⚠️ **ここを `do :; done` の素の spin にしないこと（2026-09-13 に踏んだ）。**
# 1 周ごとに `docker exec` を fork するので、待っている間ずっと Mac 側に
# 1 コア近い負荷を掛ける ＝ **測定器が測定対象を邪魔する**。2 秒ごとに緩める
until docker exec "$NAME" bash -c 'pgrep -f "ros2 bag pla[y]" >/dev/null' 2>/dev/null; do sleep 1; done
until ! docker exec "$NAME" bash -c 'pgrep -f "ros2 bag pla[y]" >/dev/null' 2>/dev/null; do sleep 2; done

say "[5] 止める"
docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null
docker exec "$NAME" bash -c 'pkill -INT -f "mola-cl[i]"; pkill -INT -f "ros2-lidar-odometr[y]"; pkill -INT -f "fastlio_mappin[g]"; pkill -INT -f "mapping.launch.p[y]"' 2>/dev/null
# ⚠️ 負荷は**確実に**落とす。残ると次の測定が全部ずれる（09-12 に 2 本取り逃した）。
# `pkill` のパターンに自分のコマンド行が当たらないよう [] で括る
if [ "$LOAD" != "0" ]; then
    for _ in 1 2 3; do
        docker exec "$NAME" bash -c 'pkill -KILL -f "do :; don[e]"' 2>/dev/null
        docker exec "$NAME" bash -c 'pgrep -f "do :; don[e]" >/dev/null' 2>/dev/null || break
    done
fi
until ! docker exec "$NAME" bash -c 'pgrep -f "ros2 bag recor[d]" >/dev/null' 2>/dev/null; do sleep 1; done

n=$(docker exec -u ubuntu "$NAME" bash -c "ls $OUT/tf_bag/*.db3 2>/dev/null | wc -l")
say "出力: $OUT （tf_bag $n 個）"
# 空の bag を「成功」として返さない
# ⚠️ コンテナに sqlite3 の CLI は入っていない（python3 の sqlite3 モジュールは在る）
msgs=$(docker exec -u ubuntu "$NAME" python3 -c "
import sqlite3,glob
db=glob.glob('$OUT/tf_bag/*.db3')[0]
print(sqlite3.connect('file:%s?mode=ro'%db,uri=True).execute('SELECT COUNT(*) FROM messages').fetchone()[0])" 2>/dev/null || echo 0)
say "録れた /tf: $msgs 件"
[ "${msgs:-0}" -lt 10 ] && { say "⛔ /tf がほとんど録れていない。MOLA の出力を確認する"; exit 1; }
docker exec -u ubuntu "$NAME" bash -c "grep -c 'Not able to use velocity motion model' $OUT/mola.log 2>/dev/null || echo 0" \
    | sed 's/^/[rt] velocity motion model を使えなかった回数: /'
