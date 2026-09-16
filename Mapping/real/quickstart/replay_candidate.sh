#!/usr/bin/env bash
# **記録 1 本を、測位候補を差し替えて実時間で再生し直す。**軌跡を候補ごとに録る。
#
#   bash quickstart/replay_candidate.sh click4_20260913 amcl    click4_amcl
#   bash quickstart/replay_candidate.sh click4_20260913 fastlio click4_fastlio
#
# MOLA は `replay_realtime.sh` が入口（こちらでは扱わない）。
# GLIM は `pc2/replay_glim_loc.sh` が入口（コンテナの中で走る）。
#
# ## なぜ要るのか（2026-09-15）
#
# 作業ログ 09-15_1 の §5 に click2/3/4 の動画を 3 本載せたが、**あれは MOLA しか
# 写っていない**。記録に入っている `/tf` は 09-13 に動いていた MOLA の出力だからである。
# 候補を比べるには、**同じ記録を候補ごとに再生し直して `/tf` を録り直す**しかない。
#
# ## 関門（2026-09-15 に確認済み）
#
# 09-12 の掃引では `mola-lidar-odometry-cli` の再生が live の崩壊を再現しなかった
# （σ を 2 桁振っても M1 ~ 0.28 m）。**実時間 ＋ 脚 odom** で再生すると再現する:
#
#   live   M3 25.4 % / M4 1.376 m/s / 正味 17.128 m / 脚odom差 +1.443 m
#   再生   M3 25.2 % / M4 1.632 m/s / 正味 17.028 m / 脚odom差 +1.343 m
#
# だから**このスクリプトは必ず等倍（`--rate 1`）で回す**。速めると比較が壊れる。
#
# ## ⚠️ 罠
#
# - **bag の `/tf` を再生してはいけない。** 当時の MOLA の `map -> base_link` が流れて、
#   これから測る候補の出力と `base_link` の親が 2 つになる。
# - **bag の `/tf_static` も再生してはいけない。** この記録の `/tf_static` には
#   `map -> odom` の**静的な恒等**が入っている（09-09 の段 A の手当て）。
#   AMCL は `map -> odom` を**動的に**出すので、静的と衝突する。
#   代わりに候補ごとに要る静的 TF をこちらで出す。
# - **候補は 1 つずつ動かすこと。** 2 つが同時に `map -> base_link` を出すと `/tf` が壊れる。
#   さらに CPU を取り合って条件が変わる（それ自体が調査対象の変数である）。
# - 出力は **`/tf` と `/tf_static` だけ録る**。LiDAR を録り直すと 1 本 800 MB を超える。
#   採点と描画はスキャンを**元の記録から**読む。
# - `pkill -f` / `pgrep -f` は必ずブラケットで書く（`mola-cl[i]`）。素で書くと自分が死ぬ。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"
NAME="$G1_RVIZ_NAME"
RUN="${1:?usage: replay_candidate.sh <run_dir_name> <amcl|fastlio> [out_name]}"
CAND="${2:?候補を指定する: amcl | fastlio}"
OUT_NAME="${3:-${RUN}_${CAND}}"

WORK=/work/G1_Hackason/Mapping/real
BAG="$WORK/runs/$RUN/${G1_BAG_SUB:-bag}"
OUT="$WORK/runs/_replay/$OUT_NAME"
QS="$WORK/quickstart"
MAPDIR="$WORK/runs/20260906T135940_UiS_room_v3/map"
RATE="${G1_RT_RATE:-1}"
DELAY="${G1_BAG_DELAY:-14}"
LOAD="${G1_RT_LOAD:-0}"
# base_link の初期姿勢（map 系・水平・床面）: "x y yaw_deg"
INIT="${G1_INIT_XYYAW:?G1_INIT_XYYAW=\"x y yaw_deg\" が要る}"
# 取付値。live と同じでなければならない
LIVOX_XYZ="${G1_LIVOX_XYZ:-0 0 1.228}"
LIVOX_RPY_DEG="${G1_LIVOX_RPY_DEG:-177.93 3.32 0}"

DDS="$(g1_dds_uri offline)"
say() { echo "[cand] $*"; }
dex() { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
        bash -c "source /opt/ros/humble/setup.bash && $*"; }
spawn() { local log="$1"; shift
    docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
        bash -c "source /opt/ros/humble/setup.bash && $* > $OUT/$log 2>&1"; }

docker exec "$NAME" test -d "$BAG" || { echo "bag が無い: $BAG" >&2; exit 1; }

# ── 前の回の残りを必ず落とす（残ると次の測定が全部ずれる）──────────
KILL_RE='[c]loud_to_scan\.py|[d]og_odom_to_tf\.py|nav2_amcl/amc[l]|nav2_map_server/map_serve[r]|[f]astlio_mapping|[g]lobal_localization_node|[c]5_stf_|ros2 bag pla[y]|ros2 bag recor[d]|mola-cl[i]'
stop_all() {
    docker exec "$NAME" bash -c "pkill -INT -f '$KILL_RE'" 2>/dev/null
    sleep 3
    docker exec "$NAME" bash -c "pkill -KILL -f '$KILL_RE'" 2>/dev/null
    # 負荷は**確実に**落とす（09-12 に残して 2 本取り逃した）
    docker exec "$NAME" bash -c 'pkill -KILL -f "do :; don[e]"' 2>/dev/null
    return 0
}
stop_all
docker exec -u ubuntu "$NAME" bash -c "rm -rf $OUT && mkdir -p $OUT"

say "候補 $CAND / 記録 $RUN / ${RATE}倍速 / 負荷 $LOAD 本 / 初期姿勢 ($INIT)"

if [ "$LOAD" != "0" ]; then
    for _ in $(seq 1 "$LOAD"); do
        docker exec -d "$NAME" bash -c 'while :; do :; done' >/dev/null
    done
    say "負荷 $LOAD 本を投入した"
fi

read -r EX EY EZ <<EOF
$LIVOX_XYZ
EOF
read -r ERD EPD EYD <<EOF
$LIVOX_RPY_DEG
EOF
d2r() { awk -v d="$1" 'BEGIN{printf "%.9f", d*3.141592653589793/180}'; }
ER="$(d2r "$ERD")"; EP="$(d2r "$EPD")"; EYW="$(d2r "$EYD")"
read -r IX IY IYAW_DEG <<EOF
$INIT
EOF

# ── 候補を起こす ───────────────────────────────────────────────────
case "$CAND" in
amcl)
    # 鎖: map --(amcl)--> odom --(dog_odom_to_tf)--> base_link
    # ⚠️ dog_odom_to_tf は TF を `get_clock().now()` で打つので **--use-sim-time が必須**。
    #    付け忘れると打刻が 2 日ずれて AMCL の lookup が全部落ちる（落ちずに黙る）。
    # ⚠️ --rate は live の 100 ではなく 200 にする。spin_once は 1 周に 1 件しか
    #    処理しないので、/clock と /dog_odom を分け合って実効が半分になる
    say "[1] dog_odom_to_tf.py（odom->base_link・sim time）"
    spawn odomtf.log "python3 $QS/pc2/dog_odom_to_tf.py --rate 200 --use-sim-time"
    say "[2] cloud_to_scan.py（3D -> /scan・帯 1.30〜1.80 m）"
    spawn scan.log "python3 $QS/pc2/cloud_to_scan.py --min-height 1.30 --max-height 1.80 --range-max 20.0"
    say "[3] map_server（nav_map_run.yaml）"
    spawn mapserver.log "ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=$MAPDIR/nav_map_run.yaml -p use_sim_time:=true -p frame_id:=map"
    IYAW="$(d2r "$IYAW_DEG")"
    say "[4] amcl（初期姿勢 x=$IX y=$IY yaw=${IYAW_DEG} deg）"
    spawn amcl.log "ros2 run nav2_amcl amcl --ros-args --params-file $QS/pc2/amcl_live.yaml \
        -p use_sim_time:=true -p set_initial_pose:=true \
        -p initial_pose.x:=$IX -p initial_pose.y:=$IY -p initial_pose.z:=0.0 -p initial_pose.yaw:=$IYAW"
    PROC_PAT="nav2_amcl/amc[l]"
    LIFECYCLE=1
    ;;
fastlio)
    # 鎖: map --(open3d_loc)--> odom --(静的恒等)--> camera_init --(FAST-LIO)--> body --(静的)--> base_link
    # 幾何は run_fastlio_loc_live.sh と同じ計算（要点 1 と 2。角度規約が 2 つある）
    GEO="$(python3 - "$LIVOX_XYZ" "$LIVOX_RPY_DEG" "$INIT" <<'PY'
import sys, math
xyz  = [float(v) for v in sys.argv[1].split()]
rpyd = [float(v) for v in sys.argv[2].split()]
ib   = [float(v) for v in sys.argv[3].split()]
def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def Rx(a): c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def Ry(a): c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def Rz(a): c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def ros_rpy(r,p,y): return mm(Rz(y), mm(Ry(p), Rx(r)))
def to_ros_rpy(R):
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    return math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])
def to_xyz_intrinsic(R):   # R = Rx(a)Ry(b)Rz(c) <- open3d_loc の規約（ROS とは別物）
    b = math.asin(max(-1.0, min(1.0, R[0][2])))
    return math.atan2(-R[1][2], R[2][2]), b, math.atan2(-R[0][1], R[0][0])
ER = ros_rpy(math.radians(rpyd[0]), math.radians(rpyd[1]), math.radians(rpyd[2]))
ERi = [[ER[j][i] for j in range(3)] for i in range(3)]
Eti = [-v for v in mv(ERi, xyz)]
r,p,y = to_ros_rpy(ERi)
MR = mm(Rz(math.radians(ib[2])), ER)
Mt = [a+b for a,b in zip([ib[0], ib[1], 0.0], mv(Rz(math.radians(ib[2])), xyz))]
a,b2,c = to_xyz_intrinsic(MR)
print("[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]" % (Mt[0],Mt[1],Mt[2],
      math.degrees(a),math.degrees(b2),math.degrees(c)))
print("%.6f %.6f %.6f" % tuple(Eti))
print("%.9f %.9f %.9f" % (r,p,y))
PY
)" || { echo "[cand] 幾何の計算に失敗した" >&2; exit 1; }
    INITPOSE="$(echo "$GEO" | sed -n 1p)"
    read -r BX BY BZ <<EOF
$(echo "$GEO" | sed -n 2p)
EOF
    read -r BR BP BYW <<EOF
$(echo "$GEO" | sed -n 3p)
EOF
    say "[1] 静的 TF 3 本（odom->camera_init / body->base_link / base_link->imu_link）"
    spawn stf.log "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
        --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id camera_init \
        --ros-args -p use_sim_time:=true -r __node:=c5_stf_odom"
    spawn stf2.log "ros2 run tf2_ros static_transform_publisher --x $BX --y $BY --z $BZ \
        --roll $BR --pitch $BP --yaw $BYW --frame-id body --child-frame-id base_link \
        --ros-args -p use_sim_time:=true -r __node:=c5_stf_base"
    spawn stf3.log "ros2 run tf2_ros static_transform_publisher --x $EX --y $EY --z $EZ \
        --roll $ER --pitch $EP --yaw $EYW --frame-id base_link --child-frame-id imu_link \
        --ros-args -p use_sim_time:=true -r __node:=c5_stf_imu"
    sleep 3
    say "[2] FAST-LIO2（内側の odometry）"
    spawn fastlio.log "source /work/third_party/ws_c5/install/setup.bash && \
        ros2 run fast_lio fastlio_mapping --ros-args --params-file $QS/pc2/fastlio_loc/fastlio_g1.yaml \
        -p use_sim_time:=true"
    sleep 3
    say "[3] open3d_loc（事前地図への ICP。initialpose = T_map_odom）"
    say "    initialpose $INITPOSE"
    # ⚠️ initialpose とカルマンは**コマンドラインで渡す**（yaml からは入らない。要点 3・4）
    spawn o3dloc.log "source /work/third_party/ws_c5/install/setup.bash && \
        ros2 run open3d_loc global_localization_node --ros-args \
        --params-file $QS/pc2/fastlio_loc/open3d_loc_g1.yaml \
        -p use_sim_time:=true \
        -p path_map:=$MAPDIR/map_octomap_r4_s5_floor0.pcd \
        -p initialpose:=$INITPOSE \
        -p 'kf_baselink2map/x:=[0.001,0.002]' \
        -p 'kf_baselink2map/y:=[0.001,0.005]' \
        -p 'kf_baselink2map/z:=[0.00001,0.04]'"
    PROC_PAT="[g]lobal_localization_node"
    LIFECYCLE=0
    ;;
*)  echo "[cand] 知らない候補: ${CAND}（amcl | fastlio）" >&2; exit 2 ;;
esac

# ── 立ったことを確かめてから進む ───────────────────────────────────
# ⚠️ 設定を 1 文字間違えると数百 ms で終了し、そのあと再生も録画も**成功したように**
#    流れて空の bag が残る（09-12 に 2 回踏んだ）。ここで落とす
for i in $(seq 1 30); do
    docker exec "$NAME" bash -c "pgrep -f \"$PROC_PAT\" >/dev/null" 2>/dev/null && break
    [ "$i" = 30 ] && { say "⛔ $CAND が起動しない。$OUT のログを見る"; stop_all; exit 1; }
    sleep 1
done
say "    $CAND の起動を確認した"

# ── 再生（⚠️ /tf も /tf_static も再生しない）───────────────────────
say "[4] 記録を再生する（/tf・/tf_static は流さない）"
spawn bagplay.log "ros2 bag play $BAG --rate $RATE --clock --delay $DELAY \
    --topics /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /dog_odom"

# ⚠️ **/clock が流れてから lifecycle を上げる。** 先に activate すると AMCL の
#    時刻が 0 のままで、再生が始まった瞬間に 2 日ぶん跳ぶ
until docker exec "$NAME" bash -c 'pgrep -f "ros2 bag pla[y]" >/dev/null' 2>/dev/null; do sleep 1; done
if [ "${LIFECYCLE:-0}" = "1" ]; then
    sleep $((DELAY + 3))
    say "[5] lifecycle を上げる（map_server -> amcl）"
    # ⚠️ リダイレクトは**コンテナの中**に置く。`dex ... > path` だと Mac 側に書こうとして
    #    `/work/...` が無いので落ちる。`timeout` はサービスが出ないときの保険
    dex "timeout 60 ros2 run nav2_util lifecycle_bringup map_server > $OUT/lifecycle.log 2>&1" \
        || say "⚠️ map_server の bringup が返らなかった"
    dex "timeout 60 ros2 run nav2_util lifecycle_bringup amcl >> $OUT/lifecycle.log 2>&1" \
        || say "⚠️ amcl の bringup が返らなかった"
    say "    activated"
fi

say "[6] /tf と /tf_static を録る"
spawn record.log "ros2 bag record -o $OUT/tf_bag /tf /tf_static --use-sim-time"

say "[7] 再生の終了を待つ"
# ⚠️ 素の spin にしないこと。1 周ごとに docker exec を fork して Mac 側に
#    1 コア近い負荷を掛ける ＝ 測定器が測定対象を邪魔する
until ! docker exec "$NAME" bash -c 'pgrep -f "ros2 bag pla[y]" >/dev/null' 2>/dev/null; do sleep 2; done

say "[8] 止める"
docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null
until ! docker exec "$NAME" bash -c 'pgrep -f "ros2 bag recor[d]" >/dev/null' 2>/dev/null; do sleep 1; done
stop_all

msgs=$(docker exec -u ubuntu "$NAME" python3 -c "
import sqlite3,glob
g=glob.glob('$OUT/tf_bag/*.db3')
print(sqlite3.connect('file:%s?mode=ro'%g[0],uri=True).execute('SELECT COUNT(*) FROM messages').fetchone()[0] if g else 0)" 2>/dev/null || echo 0)
say "録れた /tf: $msgs 件 -> $OUT"
[ "${msgs:-0}" -lt 10 ] && { say "⛔ ほとんど録れていない。$OUT のログを見る"; exit 1; }
exit 0
