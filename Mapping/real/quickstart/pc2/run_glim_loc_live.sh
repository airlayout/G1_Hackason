#!/usr/bin/env bash
# **PC2（機体の Orin NX）で GLIM の事前地図測位を live で動かす。**
#
#   bash run_glim_loc_live.sh --init "0.703 12.966 -57.5"   # x[m] y[m] yaw[deg]（base_link・map 系）
#   bash run_glim_loc_live.sh --map <GLIM の dump ディレクトリ>
#   G1_PC2_SECONDS=120 bash run_glim_loc_live.sh --init "..."   # 秒数を切る（既定 0 = 無限）
#   bash run_glim_loc_live.sh --stop                          # 全部落とす
#
# ログ: /tmp/pc2_glim.log /tmp/pc2_glim_stf.log /tmp/pc2_glim_loadmap.log /tmp/pc2_glim_init.log
#
# ## 何が動くのか
#
#   glim_rosnode (localization)  /utlidar/cloud_livox_mid360 ＋ /utlidar/imu_livox_mid360
#                                → TF `glim_map -> odom` と `odom -> base_link`
#   static_transform_publisher   TF `map -> glim_map`（GLIM 地図を既存 map 系へ乗せる）
#   static_transform_publisher   TF `base_link -> livox_frame`（取付・上下逆さま）
#
# **出力の契約**: `/tf` に `map -> base_link`。鎖は
#
#   map --(静的)--> glim_map --(GLIM localization)--> odom --(GLIM odometry)--> base_link
#
# ## ⚠️ 設計上の要点（全部ソースを読んで実測で確かめた。触る前に読むこと）
#
# 1. **地図は起動時には読まれない。** `map_path` を渡しても `setup_localization()` は
#    値を覚えるだけで、実際の読み込みは **`~/load_map` サービス**が来たときだけ
#    （glim_ros.cpp:232-243）。このスクリプトが起動後に叩く。
#
# 2. **再定位は `/initialpose` でしか始まらない。** しかも
#    **odometry が 1 フレームも出す前に投げると握り潰される**
#    （glim_ros.cpp:266 "latest frame is null. Abort relocalize"）。
#    ⇒ ログに受理が出るまで投げ直す。⚠️ 受理後に投げ直してはいけない。
#    二度目の再定位が overlap 0.0 の偽の一致で上書きする（2026-09-15 実測）。
#
# 3. **`/initialpose` は base_link の姿勢ではない。** 中身は
#    **glim_map 系の「いまのサブマップ原点」の姿勢**で、`create_relocalization_factors`
#    がこれとの距離でいちばん近い事前サブマップを選ぶ（localization.cpp:186-196）。
#    サブマップ原点は IMU 系、`T_lidar_imu` は単位なので livox_frame と同じ。よって
#        initialpose = T_map_glimmap^-1 · T_map_baselink · (base_link -> livox_frame)
#    このスクリプトが `--init` の base_link 姿勢からこれを計算する。
#
# 4. **`map_frame_id` は "map" ではなく "glim_map"**（config_ros.json）。
#    GLIM の地図の原点は記録の最初の IMU 姿勢（重力整列）で、既存の floor0 系とは
#    一致しない。**地図は触らず、静的 TF `map -> glim_map` で吸収する。**
#    値は `align_glim_to_map.py` が両者の軌跡から求めて `map_frame.txt` に書く
#    （実測: 対応 5442 点・位置 RMSE 0.056 m・回転中央値 0.607 deg）。
#
# 5. **IMU は必須。** `libodometry_estimation_cpu.so` は IMU 無しでは動かず、
#    IMU 無し用の `libodometry_estimation_ct.so` は**事前地図のサブマップと規約が違う**
#    （`sub_mapping.cpp:617` が回転を捨てるのに passthrough は捨てない）。
#    live の機体は `/utlidar/imu_livox_mid360` を 200 Hz で出しているので問題ない。
#    ⚠️ IMU の加速度は **g 単位**。`config_ros.json` の `acc_scale: 9.80665` が要る。
#
# 6. **終了時に `terminate called without an active exception` で core を吐く。**
#    `Localization` の再定位スレッドが join されないまま落ちるため。
#    **測位そのものには影響しない**（仕事は全部終わっている）。2026-09-15 実測。
#
# ## ⚠️ 安全
#
# **これは測位だけである。`/cmd_vel` は出さない。機体は動かない。**
# DDS domain 0 ＋ NIC eth0 に出るので、**実行前に /tmp/pc2_lock を取ること。**
# ⚠️ **MOLA や AMCL と同時に起こさない。** `map -> odom` / `map -> base_link` と
#    衝突して TF の親が 2 つになる。
set -uo pipefail

JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
WS="${G1_GLIM_ROOT:-$JAMMY/ws_glim}"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
GLIM_CFG="${G1_GLIM_CFG:-$CFG/glim_loc}"
MAP="${G1_GLIM_MAP:-$CFG/map/glim_map}"
FRAME="${G1_GLIM_FRAME:-$CFG/map/glim_map_frame.txt}"
DDS_NIC="${G1_PC2_DDS_NIC:-eth0}"
DOMAIN="${G1_PC2_DOMAIN:-0}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"
# live の取付値。⚠️ run_mola_live.sh / run_fastlio_loc_live.sh と同じ値でなければならない
LIVOX_XYZ="${G1_LIVOX_XYZ:-0 0 1.228}"
LIVOX_RPY_DEG="${G1_LIVOX_RPY_DEG:-177.93 3.32 0}"
# base_link の初期姿勢（map 系・水平・床面）: x y yaw_deg
INIT_BASELINK="${G1_GLIM_INIT:-0.703 12.966 -57.5}"

STOP=0
while [ $# -gt 0 ]; do
    case "$1" in
        --init)  INIT_BASELINK="$2"; shift 2 ;;
        --map)   MAP="$2"; shift 2 ;;
        --frame) FRAME="$2"; shift 2 ;;
        --stop)  STOP=1; shift ;;
        -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[glim] 知らない引数: $1" >&2; exit 2 ;;
    esac
done

say() { echo "[glim] $*"; }

# ⚠️ 括弧は**自己一致を外す**ため（素で書くと自分の ssh やシェルが死ぬ。09-15 に 3 回踏んだ）
KILL_RE='[g]lim_rosnode|[g]lim_stf_'

stop_all() {
    pkill -INT -f "$KILL_RE" 2>/dev/null
    sleep 3
    pkill -TERM -f "$KILL_RE" 2>/dev/null
    sleep 2
    pkill -KILL -f "$KILL_RE" 2>/dev/null
    sleep 1
    return 0
}

if [ "$STOP" = "1" ]; then
    stop_all
    say "止めた。残り $(pgrep -c -f "$KILL_RE" 2>/dev/null || echo 0) 個"
    exit 0
fi

[ -f "$JAMMY/env.sh" ]      || { echo "[glim] env.sh が無い: $JAMMY/env.sh" >&2; exit 2; }
[ -x "$WS/bin/glim_rosnode" ] || { echo "[glim] 実行ファイルが無い: $WS/bin/glim_rosnode" >&2; exit 2; }
[ -f "$GLIM_CFG/config.json" ] || { echo "[glim] 設定が無い: $GLIM_CFG/config.json" >&2; exit 2; }
[ -d "$MAP" ]               || { echo "[glim] 事前地図が無い: $MAP" >&2; exit 2; }
[ -f "$MAP/graph.txt" ]     || { echo "[glim] 事前地図に graph.txt が無い: $MAP" >&2; exit 2; }
[ -f "$FRAME" ]             || { echo "[glim] 変換が無い: $FRAME（align_glim_to_map.py が出す）" >&2; exit 2; }

# shellcheck source=/dev/null
. "$FRAME"          # G1_GLIM_MAP_XYZ / G1_GLIM_MAP_RPY_DEG

# ── 幾何の計算（要点 3）─────────────────────────────────────────
# 出力 1 行: initialpose の x y z qx qy qz qw（glim_map 系）
POSE="$(python3 - "$G1_GLIM_MAP_XYZ" "$G1_GLIM_MAP_RPY_DEG" "$LIVOX_XYZ" "$LIVOX_RPY_DEG" "$INIT_BASELINK" <<'PY'
import sys, math
def mm(A, B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A, v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def Rx(a): c, s = math.cos(a), math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def Ry(a): c, s = math.cos(a), math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def Rz(a): c, s = math.cos(a), math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
def rpy(r, p, y): return mm(Rz(y), mm(Ry(p), Rx(r)))            # ROS の規約 Rz*Ry*Rx
gx = [float(v) for v in sys.argv[1].split()]
gr = [math.radians(float(v)) for v in sys.argv[2].split()]
ex = [float(v) for v in sys.argv[3].split()]
er = [math.radians(float(v)) for v in sys.argv[4].split()]
ib = [float(v) for v in sys.argv[5].split()]
GR, Gt = rpy(*gr), gx                                            # map -> glim_map
ER, Et = rpy(*er), ex                                            # base_link -> livox_frame
BR, Bt = rpy(0.0, 0.0, math.radians(ib[2])), [ib[0], ib[1], 0.0] # map -> base_link
# map -> livox_frame
LR, Lt = mm(BR, ER), [a+b for a, b in zip(Bt, mv(BR, Et))]
# glim_map -> livox_frame = (map->glim_map)^-1 * (map->livox_frame)
GRi = [[GR[j][i] for j in range(3)] for i in range(3)]
R = mm(GRi, LR)
t = mv(GRi, [a-b for a, b in zip(Lt, Gt)])
tr = R[0][0] + R[1][1] + R[2][2]
if tr > 0:
    w = math.sqrt(1.0 + tr) / 2.0
    x, y, z = (R[2][1]-R[1][2])/(4*w), (R[0][2]-R[2][0])/(4*w), (R[1][0]-R[0][1])/(4*w)
elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
    s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
    w, x, y, z = (R[2][1]-R[1][2])/s, 0.25*s, (R[0][1]+R[1][0])/s, (R[0][2]+R[2][0])/s
elif R[1][1] > R[2][2]:
    s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
    w, x, y, z = (R[0][2]-R[2][0])/s, (R[0][1]+R[1][0])/s, 0.25*s, (R[1][2]+R[2][1])/s
else:
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
    w, x, y, z = (R[1][0]-R[0][1])/s, (R[0][2]+R[2][0])/s, (R[1][2]+R[2][1])/s, 0.25*s
print("%.6f %.6f %.6f %.9f %.9f %.9f %.9f" % (t[0], t[1], t[2], x, y, z, w))
PY
)" || { echo "[glim] 幾何の計算に失敗した" >&2; exit 1; }
read -r PX PY_ PZ QX QY QZ QW <<EOF
$POSE
EOF
[ -n "${QW:-}" ] || { echo "[glim] 幾何の出力が空" >&2; exit 1; }

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

# ws_glim の .so を見せるためにライブラリパスを伸ばす。
# ⚠️ LD_LIBRARY_PATH は **export しない**（README の罠 5）。--library-path だけで足りる
jrun_ws() { local exe="$1"; shift
    env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS:$WS/lib" "$exe" "$@"; }

say "事前地図: $MAP"
say "設定: $GLIM_CFG"
say "base_link 初期姿勢 (x y yaw_deg): $INIT_BASELINK"
say "map -> glim_map: xyz ($G1_GLIM_MAP_XYZ) / rpy ($G1_GLIM_MAP_RPY_DEG) deg"
say "initialpose (glim_map 系): $PX $PY_ $PZ / q $QX $QY $QZ $QW"

PIDS=""
CLEANED=0
cleanup() {
    [ "$CLEANED" = "1" ] && return
    CLEANED=1
    for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
    stop_all
    say "後始末おわり。残り $(pgrep -c -f "$KILL_RE" 2>/dev/null || echo 0) 個"
}
trap cleanup EXIT INT TERM

# ── 静的 TF 2 本 ────────────────────────────────────────────────
# ⚠️ `jrun <実行ファイル>` で呼ばないこと。write_jammy_env.sh が $ROS/bin の ELF を
#    全部 `#!/bin/sh` のラッパにしているので、ローダに渡すと落ちる。`ros2 run` を使う。
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

jros2 run tf2_ros static_transform_publisher --x "$GX" --y "$GY" --z "$GZ" \
    --roll "$GR" --pitch "$GP" --yaw "$GYW" \
    --frame-id map --child-frame-id glim_map \
    --ros-args -r __node:=glim_stf_map > /tmp/pc2_glim_stf.log 2>&1 &
PIDS="$PIDS $!"
jros2 run tf2_ros static_transform_publisher --x "$EX" --y "$EY" --z "$EZ" \
    --roll "$ER" --pitch "$EP" --yaw "$EYW" \
    --frame-id base_link --child-frame-id livox_frame \
    --ros-args -r __node:=glim_stf_livox >> /tmp/pc2_glim_stf.log 2>&1 &
PIDS="$PIDS $!"
sleep 3
for p in $PIDS; do
    kill -0 "$p" 2>/dev/null || { echo "[glim] 静的 TF が起動しない（/tmp/pc2_glim_stf.log）" >&2; exit 1; }
done
say "静的 TF 2 本 OK（map->glim_map / base_link->livox_frame）"

# ── GLIM ────────────────────────────────────────────────────────
jrun_ws "$WS/bin/glim_rosnode" --ros-args \
    -p config_path:="$GLIM_CFG" \
    -p localization:=true \
    -p map_path:="$MAP" \
    > /tmp/pc2_glim.log 2>&1 &
G_PID=$!; PIDS="$PIDS $G_PID"
sleep 8
kill -0 "$G_PID" 2>/dev/null || { echo "[glim] glim_rosnode が起動しない（/tmp/pc2_glim.log）" >&2; exit 1; }
say "glim_rosnode 起動"

# ── 地図を読む（要点 1）──────────────────────────────────────────
jros2 service call /glim_ros/load_map std_srvs/srv/Trigger "{}" > /tmp/pc2_glim_loadmap.log 2>&1
if grep -q "Successuflly load map" /tmp/pc2_glim_loadmap.log; then
    say "事前地図を読んだ（$(grep -c "^" /tmp/pc2_glim_loadmap.log) 行）"
else
    echo "[glim] 地図の読み込みに失敗（/tmp/pc2_glim_loadmap.log）" >&2
    cat /tmp/pc2_glim_loadmap.log >&2
    exit 1
fi

# ── 再定位（要点 2）─────────────────────────────────────────────
POSE_MSG="{header: {frame_id: 'map'}, pose: {pose: {position: {x: $PX, y: $PY_, z: $PZ}, \
 orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}}}"
RELOC=0
for i in 1 2 3 4 5 6 7 8 9 10; do
    if grep -q "Create_relocalization_factors" /tmp/pc2_glim.log; then
        RELOC=1; break
    fi
    jros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "$POSE_MSG" > /tmp/pc2_glim_init.log 2>&1
    sleep 2
done
if [ "$RELOC" = 1 ]; then
    say "再定位が走った: $(grep -m1 'best overlap score' /tmp/pc2_glim.log || echo '（スコア未出力）')"
else
    say "⚠️ 再定位が始まらなかった。/tmp/pc2_glim.log を見ること"
fi

say "ログ: /tmp/pc2_glim.log /tmp/pc2_glim_stf.log"
say "確認: jros2 run tf2_ros tf2_echo map base_link"

if [ "$SECONDS_LIMIT" != "0" ]; then
    say "${SECONDS_LIMIT} 秒で止める"
    # ⚠️ 送り先は**このスクリプト自身**で、信号は **SIGTERM**（README の罠 8）。
    #    背景に置かれた子は SIGINT を無視するので kill -INT $G_PID は効かない
    ( sleep "$SECONDS_LIMIT"; kill -TERM $$ 2>/dev/null ) &
fi
wait "$G_PID"
