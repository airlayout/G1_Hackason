#!/usr/bin/env bash
# `map_localizer.py` の**照合1回にかかる時間**を、記録済み bag で測る（機体は動かさない）。
#
#     bench_localizer.sh <bag> <地図.yaml> <dx> <dy> <yaw[rad]> [秒数=120] [match_rate=10] [出力dir]
#
# <dx> <dy> <yaw> はその bag の map→odom（find_map_offset.py の結果）。
# 局所探索なので、ずれていると点が地図の外に出て、本番と違う計算量になる。
# 0911_robag（とその3トピック版）＋ room_a_map_20260911_edited なら `-0.30 0 0`
# （2026-11-03 に find_map_offset.py --yaw-range 180 で求めた。50cm以内 100%）:
#
#     bench_localizer.sh ~/bench_0911_180s maps/grids/room_a_map_20260911_edited.yaml -0.30 0 0
#
# ## なぜ要るのか（2026-11-03）
#
# 照合は既定 2Hz（`--match-rate`）だが、**2 の根拠はどこにも無い**。その場旋回の
# ドリフトに追いつけない疑いがあり（README §2）、上げられるかを PC2 の実測で決める。
#
# ## 機体に影響しないための作り
#
# - **ROS_DOMAIN_ID=77 ＋ ROS_LOCALHOST_ONLY=1** で閉じる。実機の点群（domain 0）と混ざらず、
#   こちらの再生も機体側へ出ない。機体の電源が入っていても測れる
# - `map_localizer.py` は `--observe-only`（map→odom の TF を出さない）
# - 送るのは bag の3トピックだけ。移動系の指令は一切出さない
#
# ## 動かす場所
#
# - **PC2**: `~/.pixi/bin/pixi` があれば pixi の Humble で動かす（CycloneDDS）
# - **/opt/ros/humble がある PC**: それを使う
# - **どちらも無い PC**: Docker の `osrf/ros:humble-desktop` で動かす（FastDDS、ネットワーク無し）
#
# ⚠️ 照合の点数は `--max-points`（既定 1500）で頭打ちなので、計算量は点数よりも
# 探索の窓（粗 7×7×7 ＋ 細 5×5×5 = 468 候補）で決まる。
set -u

if [ $# -lt 5 ]; then
    sed -n 2,8p "$0" >&2
    exit 2
fi
BAG="$(realpath "$1")"; MAP_YAML="$(realpath "$2")"
DX="$3"; DY="$4"; YAW="$5"
SECS="${6:-120}"; RATE="${7:-10}"
OUT="$(realpath -m "${8:-./bench_localizer_$(date +%Y%m%d_%H%M%S)}")"
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"
G1_DIR="${G1_NAV2_DIR:-/home/unitree/g1_nav2}"
mkdir -p "$OUT"

# --- 外側: 環境を選んで、自分自身を内側として呼び直す -----------------------------
if [ -z "${G1_BENCH_INNER:-}" ]; then
    ARGS=("$BAG" "$MAP_YAML" "$DX" "$DY" "$YAW" "$SECS" "$RATE" "$OUT")
    if [ -x "$PIXI" ]; then
        echo "[bench] PC2 の pixi(Humble) で動かす"
        cd "$G1_DIR/pc2_humble" || exit 1
        exec "$PIXI" run env G1_BENCH_INNER=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            bash "$TOOLS/bench_localizer.sh" "${ARGS[@]}"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        echo "[bench] /opt/ros/humble で動かす"
        G1_BENCH_INNER=1 exec bash "$TOOLS/bench_localizer.sh" "${ARGS[@]}"
    else
        echo "[bench] Docker(osrf/ros:humble-desktop) で動かす"
        BAG_DIR="$(dirname "$BAG")"; MAP_DIR="$(dirname "$MAP_YAML")"
        exec docker run --rm --network none \
            -e G1_BENCH_INNER=1 \
            -v "$TOOLS:/tools:ro" -v "$BAG_DIR:/bagdir:ro" -v "$MAP_DIR:/mapdir:ro" -v "$OUT:/out" \
            osrf/ros:humble-desktop \
            bash -c "source /opt/ros/humble/setup.bash && bash /tools/bench_localizer.sh \
                /bagdir/$(basename "$BAG") /mapdir/$(basename "$MAP_YAML") \
                $DX $DY $YAW $SECS $RATE /out"
    fi
fi

# --- 内側: 再生して測る ---------------------------------------------------------
[ -f /opt/ros/humble/setup.bash ] && [ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI   # PC2 の eth0 固定の設定を持ち込まない（localhost で閉じるため）
cd "$OUT" || exit 1
PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

echo "[bench] 出力: $OUT  (domain=$ROS_DOMAIN_ID localhost only, RMW=${RMW_IMPLEMENTATION:-既定})"
echo "[bench] CPU: $(nproc) コア / $(grep -m1 -E 'model name|Model' /proc/cpuinfo | cut -d: -f2 | xargs)"

# 地図と TF を先に上げる。g1_slam_odom_tf.py の自動校正は bag の IMU で行う
python3 "$TOOLS/publish_map_file.py" "$MAP_YAML" > map.log 2>&1 &
PIDS+=($!)
python3 "$TOOLS/g1_slam_odom_tf.py" --no-map-to-odom --ros-args -p use_sim_time:=true > tf.log 2>&1 &
PIDS+=($!)
python3 "$TOOLS/map_localizer.py" --initial "$DX" "$DY" "$YAW" --observe-only \
    --match-rate "$RATE" --timing-csv "$OUT/timing.csv" --log-csv "$OUT/drift.csv" \
    --ros-args -p use_sim_time:=true > localizer.log 2>&1 &
PIDS+=($!)
sleep 5   # 距離場の作成（数秒）を待つ

ros2 bag play "$BAG" --clock --rate 1.0 \
    --topics /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /unitree/slam_mapping/odom \
    > bag.log 2>&1 &
PIDS+=($!)
echo "[bench] ${SECS} 秒再生する（照合 ${RATE}Hz を指定）"
sleep "$SECS"
cleanup
trap - EXIT

grep -m1 "占有セル" map.log
grep -m1 -E "自動校正した" tf.log || echo "[bench] ⚠️ TF の自動校正の行が無い（tf.log を確認）"
grep -m1 "距離場の作成が終わった" localizer.log || echo "[bench] ⚠️ 距離場ができていない（localizer.log を確認）"
python3 "$TOOLS/localizer_timing_summary.py" "$OUT/timing.csv"
