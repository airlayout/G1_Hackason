#!/usr/bin/env bash
# **記録を再生して GLIM の事前地図測位を検証する（Mac のコンテナの中で動かす）。**
#
#   docker exec rviz bash /work/G1_Hackason/Mapping/real/quickstart/pc2/replay_glim_loc.sh \
#       --bag <再生する bag> --init "X Y YAW_DEG" --out <記録先> [--ct] [--seconds N]
#
# ⚠️ **これは PC2 用ではない。**PC2 で live に動かすのは run_glim_loc_live.sh。
#    こちらは jammy コンテナ（/opt/glim_loc に入れたビルド）を前提にしている。
#
# ## なぜ bag の /tf を捨てるのか
#
# 記録には**そのとき動かしていた測位の /tf**（AMCL の map->odom や MOLA の map->base_link）が
# 入っている。そのまま流すと odom と base_link の親が 2 つになり TF が壊れる。
# ⇒ `--remap /tf:=/tf_bag /tf_static:=/tf_static_bag` で外に逃がす。
# 真値は再生前に記録から読んで `--init` に渡す（still_report.py と同じ鎖の辿り方）。
set -uo pipefail

GLIM=/opt/glim_loc
CFG=/work/G1_Hackason/Mapping/real/quickstart/pc2/glim_loc
MAP=/work/G1_Hackason/Mapping/real/runs/20260906T135940_UiS_room_v3/glim/map
FRAME=/work/G1_Hackason/Mapping/real/runs/20260906T135940_UiS_room_v3/glim/map_frame.txt
BAG=""; INIT=""; OUT=""; CT=0; SECONDS_LIMIT=0
# ⚠️ 2026-09-16 に床基準へ定義し直した（旧 xyz 0 0 1.228 / rpy 177.93 3.32 0 は内蔵 odom 基準で
#    base_link が床から 5.90 度傾く）。詳細は dog_odom_to_tf.py の注記。
LIVOX_XYZ="-0.112579 -0.057151 1.209672"
LIVOX_RPY_DEG="-179.401 -1.9437 0.0321"

while [ $# -gt 0 ]; do
    case "$1" in
        --bag) BAG="$2"; shift 2 ;;
        --init) INIT="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --map) MAP="$2"; shift 2 ;;
        --ct) CT=1; shift ;;
        --seconds) SECONDS_LIMIT="$2"; shift 2 ;;
        *) echo "[replay] 知らない引数: $1" >&2; exit 2 ;;
    esac
done
[ -n "$BAG" ] && [ -n "$INIT" ] && [ -n "$OUT" ] || { echo "[replay] --bag --init --out が要る" >&2; exit 2; }
[ -d "$MAP" ] || { echo "[replay] 事前地図が無い: $MAP" >&2; exit 2; }
[ -f "$FRAME" ] || { echo "[replay] 変換が無い: ${FRAME}（align_glim_to_map.py を先に走らせる）" >&2; exit 2; }
[ "$CT" = 1 ] && CFG="$CFG/ct"

say() { echo "[replay] $*"; }
# ⚠️ ブラケットで自己一致を外す（素で書くと自分を殺す）
KILL_RE='[g]lim_rosnode|[g]lim_stf_|[r]os2 bag play|[r]os2 bag record'
stop_all() {
    pkill -INT -f "$KILL_RE" 2>/dev/null; sleep 2
    pkill -KILL -f "$KILL_RE" 2>/dev/null; sleep 1
    return 0
}
trap stop_all EXIT INT TERM
stop_all

# shellcheck source=/dev/null
. "$FRAME"          # G1_GLIM_MAP_XYZ / G1_GLIM_MAP_RPY_DEG

# ⚠️ ROS の setup.bash は `set -u` の下で AMENT_TRACE_SETUP_FILES の未定義参照で落ちる
set +u; source /opt/ros/humble/setup.bash; set -u
export LD_LIBRARY_PATH=$GLIM/lib:${LD_LIBRARY_PATH:-}
export ROS_DOMAIN_ID=${G1_REPLAY_DOMAIN:-43}

# ── 初期姿勢を GLIM の規約へ直す ────────────────────────────────
# GLIM の /initialpose は **glim_map 系の「いまのサブマップ原点」の姿勢**である
# （localization.cpp:186-196 が prebuilt_submaps との距離でいちばん近い物を選ぶ）。
# サブマップ原点は IMU 系で、T_lidar_imu が単位なので livox_frame と同じ。
#   initialpose = T_map_glimmap^-1 · T_map_baselink · (base_link -> livox_frame)
POSE="$(python3 - "$G1_GLIM_MAP_XYZ" "$G1_GLIM_MAP_RPY_DEG" "$LIVOX_XYZ" "$LIVOX_RPY_DEG" "$INIT" <<'PY'
import sys, math
import numpy as np
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    Rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]); Ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz@Ry@Rx
def T(R,t):
    M=np.eye(4); M[:3,:3]=R; M[:3,3]=t; return M
gx=[float(v) for v in sys.argv[1].split()]; gr=[math.radians(float(v)) for v in sys.argv[2].split()]
ex=[float(v) for v in sys.argv[3].split()]; er=[math.radians(float(v)) for v in sys.argv[4].split()]
ib=[float(v) for v in sys.argv[5].split()]
G = T(rpy(*gr), gx)                                  # map -> glim_map
E = T(rpy(*er), ex)                                  # base_link -> livox_frame
B = T(rpy(0.0, 0.0, math.radians(ib[2])), [ib[0], ib[1], 0.0])   # map -> base_link
M = np.linalg.inv(G) @ B @ E
R = M[:3,:3]
# 行列 -> quaternion
w = math.sqrt(max(0.0, 1+R[0,0]+R[1,1]+R[2,2]))/2
x = (R[2,1]-R[1,2])/(4*w); y = (R[0,2]-R[2,0])/(4*w); z = (R[1,0]-R[0,1])/(4*w)
print("%.6f %.6f %.6f %.9f %.9f %.9f %.9f" % (M[0,3],M[1,3],M[2,3],x,y,z,w))
PY
)" || { echo "[replay] 初期姿勢の計算に失敗" >&2; exit 1; }
read -r PX PY_ PZ QX QY QZ QW <<EOF
$POSE
EOF
say "base_link 初期姿勢 (map 系): $INIT"
say "initialpose (glim_map 系): $PX $PY_ $PZ / q $QX $QY $QZ $QW"

# ── 静的 TF 2 本 ─────────────────────────────────────────────────
read -r GX GY GZ <<EOF
$G1_GLIM_MAP_XYZ
EOF
read -r GR GP GYW <<EOF
$(awk -v v="$G1_GLIM_MAP_RPY_DEG" 'BEGIN{split(v,a," ");printf "%.9f %.9f %.9f",a[1]*3.141592653589793/180,a[2]*3.141592653589793/180,a[3]*3.141592653589793/180}')
EOF
read -r EX EY EZ <<EOF
$LIVOX_XYZ
EOF
read -r ER EP EYW <<EOF
$(awk -v v="$LIVOX_RPY_DEG" 'BEGIN{split(v,a," ");printf "%.9f %.9f %.9f",a[1]*3.141592653589793/180,a[2]*3.141592653589793/180,a[3]*3.141592653589793/180}')
EOF
ros2 run tf2_ros static_transform_publisher --x "$GX" --y "$GY" --z "$GZ" \
    --roll "$GR" --pitch "$GP" --yaw "$GYW" --frame-id map --child-frame-id glim_map \
    --ros-args -r __node:=glim_stf_map > /tmp/glim_stf.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --x "$EX" --y "$EY" --z "$EZ" \
    --roll "$ER" --pitch "$EP" --yaw "$EYW" --frame-id base_link --child-frame-id livox_frame \
    --ros-args -r __node:=glim_stf_livox >> /tmp/glim_stf.log 2>&1 &
sleep 2
say "静的 TF 2 本（map->glim_map / base_link->livox_frame）"

# ── GLIM ─────────────────────────────────────────────────────────
"$GLIM/lib/glim_ros/glim_rosnode" --ros-args \
    -p config_path:="$CFG" -p localization:=true -p map_path:="$MAP" \
    > /tmp/glim_loc.log 2>&1 &
GPID=$!
sleep 6
kill -0 "$GPID" 2>/dev/null || { echo "[replay] glim_rosnode が起動しない（/tmp/glim_loc.log）" >&2; tail -20 /tmp/glim_loc.log; exit 1; }
say "glim_rosnode 起動（config: ${CFG}）"

ros2 service call /glim_ros/load_map std_srvs/srv/Trigger "{}" > /tmp/glim_loadmap.log 2>&1
grep -q "Successuflly load map" /tmp/glim_loadmap.log \
    && say "事前地図を読んだ" || { echo "[replay] 地図の読み込みに失敗"; cat /tmp/glim_loadmap.log; exit 1; }

# ── 再生 ─────────────────────────────────────────────────────────
say "再生: $BAG"
ros2 bag play "$BAG" --rate "${G1_REPLAY_RATE:-0.5}" --remap /tf:=/tf_bag /tf_static:=/tf_static_bag \
    > /tmp/glim_play.log 2>&1 &
PLAY=$!

# ⚠️ **odometry が 1 フレームも出す前に /initialpose を投げると握り潰される**
#    （glim_ros.cpp:266 "latest frame is null. Abort relocalize"）。
#    投げっぱなしにせず、ログに再定位が現れるまで数回試す。
POSE_MSG="{header: {frame_id: 'map'}, pose: {pose: {position: {x: $PX, y: $PY_, z: $PZ}, \
   orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}}}"
# ⚠️ 二度投げると二度目の再定位が走り、**overlap 0.0 の偽の一致で上書きされる**
#    （2026-09-15 実測）。受理を確認してから次を投げる。
RELOC=0
for i in 1 2 3 4 5 6 7 8 9 10; do
    if grep -q "Create_relocalization_factors" /tmp/glim_loc.log; then
        RELOC=1; say "/initialpose 受理（${i} 回目の確認）"; break
    fi
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "$POSE_MSG" > /tmp/glim_initpose.log 2>&1
    sleep 2
done
[ "$RELOC" = 1 ] || say "⚠️ 再定位が始まらなかった（/tmp/glim_loc.log を見る）"

# ── 記録は**再定位が済んでから**始める ───────────────────────────
# ⚠️ 再定位の前は map->odom が単位のままなので base_link は見当違いの場所に居る。
#    そこから記録すると「滑り」に**再定位の跳び**（実測 10.7 m）がそのまま入り、
#    測位の良し悪しと無関係な数字になる（2026-09-15 に 1 度読み違えた）。
rm -rf "$OUT"
# ⚠️ **長い記録では点群を録り直さないこと。** click4（259 s）で 1 本 1.3 GB になり、
#    候補 3 つ × 記録 3 本でディスクが尽きる。採点も描画もスキャンは**元の記録から**
#    読めるので、既定を軌跡だけにし、点群が要るときだけ環境変数で足す
#    （`G1_GLIM_RECORD_TOPICS="/tf /tf_static /utlidar/cloud_livox_mid360"`）。
# shellcheck disable=SC2086
ros2 bag record -o "$OUT" ${G1_GLIM_RECORD_TOPICS:-/tf /tf_static} \
    > /tmp/glim_record.log 2>&1 &
sleep 2
say "記録開始（再定位の後）"

if [ "$SECONDS_LIMIT" != "0" ]; then
    ( sleep "$SECONDS_LIMIT"; kill -INT "$PLAY" 2>/dev/null ) &
fi
wait "$PLAY" 2>/dev/null
sleep 3
say "再生おわり。記録: $OUT"
grep -E "relocaliz|overlap score|Load prebuilt|map is too far" /tmp/glim_loc.log | tail -20
stop_all
