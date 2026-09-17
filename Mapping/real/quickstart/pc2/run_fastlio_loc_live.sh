#!/usr/bin/env bash
# **PC2（機体の Orin NX）で C5（FAST-LIO2 ＋ open3d_loc）を live で動かす。**
#
#   bash run_fastlio_loc_live.sh                       # 点対面 ICP（既定）
#   bash run_fastlio_loc_live.sh --gicp                # GICP 版（C5-b）
#   bash run_fastlio_loc_live.sh --init "0.703 12.966 -57.5"   # base_link の初期姿勢
#   bash run_fastlio_loc_live.sh --map <prior.pcd>
#   G1_PC2_SECONDS=60 bash run_fastlio_loc_live.sh     # 秒数を切る（既定 0 = 無限）
#   bash run_fastlio_loc_live.sh --stop                # 止める
#
# ログ: /tmp/pc2_fastlio.log /tmp/pc2_o3dloc.log /tmp/pc2_c5_stf.log
#
# ## これは何か
#
# 2 層構成の事前地図測位（計画書 2026-09-12_2 の候補 C5）。
#
#   LiDAR+IMU ──> FAST-LIO2 (iEKF, 10 Hz, 漂う)  ──> camera_init -> body
#                          │ /cloud_registered_1
#                          v
#   事前地図(.pcd) ──> open3d_loc (ICP, 2.5 Hz)  ──> map -> odom
#
# **出力の契約**: `/tf` に `map -> base_link`。次の鎖で出る。
#
#   map --(open3d_loc)--> odom --(静的恒等)--> camera_init --(FAST-LIO)--> body --(静的)--> base_link
#
# ## ⚠️ 設計上の要点（全部ソースを読んで確かめた。触る前に読むこと）
#
# 1. **`initialpose` は base_link の姿勢ではない。** `mat_odom2map_ = mat_initialpose_`
#    （global_localization.cpp:775）なので中身は **T_map_odom**。
#    FAST-LIO の `camera_init` は重力整列されない（IMU_Processing.hpp:195 の
#    整列行はコメントアウト・実読）ので t=0 で **odom ≡ livox_frame**。よって
#        initialpose = T_map_baselink(t0) · (base_link -> livox_frame)
#    このスクリプトが `--init` の base_link 姿勢からこれを計算する。
#
# 2. **角度の規約が 2 つある。** open3d_loc の Euler2Matrix3d は **Rx·Ry·Rz**
#    （global_localization.cpp:425-433）で、ROS の rpy（Rz·Ry·Rx）とは**別物**。
#    取り違えても落ちない。数字だけ静かに悪くなる。ここで両方を使い分ける。
#
# 3. **`initialpose` を渡さないと SEGV する。** 既定が空ベクトルなのに
#    `initialpose_[3]` を読む（同 361-370）。2026-09-15 にコンテナで実際に落とした。
#
# 4. **カルマンのパラメータはパラメータファイルからは入らない。** コードが宣言している
#    名前が `kf_baselink2map/x`（**スラッシュ**・同 319-321）で、yaml の入れ子は
#    `kf_baselink2map.x`（ドット）になるため**一致しない**。出荷時の
#    `loc_param_g1.yaml` は入れ子なので**効いていない**（実測: kf_x が [0,0] と出る）。
#    ここでは `-p "kf_baselink2map/x:=[...]"` とコマンドラインで渡す。
#
# 5. **`base_link -> imu_link` と `motion_link -> base_link` は TF から読まれる**
#    （同 402/406）。読めないと未初期化の行列が使われる。
#    ⚠️ 上流は宣言のみで初期化しておらず**ゴミが TF へ伝播する**ので、
#    我々のビルドでは**コンストラクタで単位行列に初期化するパッチを当ててある**。
#    それでも `base_link -> imu_link` は**必ず出すこと**（ここで出している）。
#    これが無いと `mat_baselink2odom_` が単位行列になり、取付角が丸ごと落ちる。
#
# 6. **`motion_link` の静的 TF は出さない。** open3d_loc が初期化後に
#    `map -> motion_link` を**動的に**流す（同 575-581）ので、静的でも親を付けると
#    親が 2 つになって TF が壊れる。5. のパッチで単位行列に落ちるので
#    motion_link ≡ base_link になり、実害なく重複するだけになる。
#
# 7. **FAST-LIO の外部パラメータは単位行列＋推定 OFF**（fastlio_g1.yaml の注記）。
#    `body ≡ livox_frame` が崩れると 2. の静的 TF が食い違う。
#
# ## ⚠️ 安全
#
# **これは測位だけである。`/cmd_vel` は出さない。機体は動かない。**
# DDS domain 0 ＋ NIC eth0 に出るので、**実行前に /tmp/pc2_lock を取ること。**
set -uo pipefail

JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
WS="${G1_C5_ROOT:-$JAMMY/ws_c5}"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
FASTLIO_YAML="${G1_FASTLIO_YAML:-$CFG/fastlio_loc/fastlio_g1.yaml}"
O3D_YAML="${G1_O3D_YAML:-$CFG/fastlio_loc/open3d_loc_g1.yaml}"
MAP="${G1_C5_MAP:-$CFG/map/map_octomap_r4_s5_floor0.pcd}"
DDS_NIC="${G1_PC2_DDS_NIC:-eth0}"
DOMAIN="${G1_PC2_DOMAIN:-0}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"
# live の取付値。⚠️ run_mola_live.sh と同じ値でなければならない
# ⚠️ **2026-09-16 に床基準へ定義し直した。** 旧値 xyz (0,0,1.228) / rpy (177.93, 3.32, 0) は
# 機体内蔵 odom 基準で、これで定義した base_link は**床に対して 5.90 度傾く**
# （生スキャンの床平面フィット 4411 点・残差 3.3 mm で実測。法線 (0.0917, 0.0465, 0.9947)）。
# その結果 map->base_link に pitch -5.1 度が定常で乗り、preflight.sh の
# 「pitch は ±3 度」に落ちる値だった。yaw を動かさない水平軸まわりの最小回転 5.901 度で補正。
# 検算: 新しい値で床を測り直すと 傾き 0.046 度 / 高さ +0.00002 m。
# 補正後の実機 map->base_link は roll -0.32 / pitch +0.08 度（補正前 +2.43 / -5.06）。
LIVOX_XYZ="${G1_LIVOX_XYZ:--0.112579 -0.057151 1.209672}"
LIVOX_RPY_DEG="${G1_LIVOX_RPY_DEG:--179.401 -1.9437 0.0321}"
# base_link の初期姿勢（map 座標・水平・床面）: x y yaw_deg
INIT_BASELINK="${G1_C5_INIT:-0.703 12.966 -57.5}"
NODE_BIN="global_localization_node"

say() { echo "[c5] $*"; }

stop_all() {
    # ⚠️ SIGINT で落とす。SIGTERM だと子が孤児になる
    pkill -INT -f "$WS/bin/global_localization_node" 2>/dev/null
    pkill -INT -f "$WS/bin/fastlio_mapping" 2>/dev/null
    pkill -INT -f "c5_stf_" 2>/dev/null
    sleep 2
    pkill -KILL -f "$WS/bin/global_localization_node" 2>/dev/null
    pkill -KILL -f "$WS/bin/fastlio_mapping" 2>/dev/null
    return 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        --gicp)  NODE_BIN="global_localization_node_gicp"; shift ;;
        --init)  INIT_BASELINK="$2"; shift 2 ;;
        --map)   MAP="$2"; shift 2 ;;
        --stop)  say "止める"; stop_all; say "残り: $(pgrep -cf "$WS/bin/" 2>/dev/null || true)"; exit 0 ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[c5] 知らない引数: $1" >&2; exit 2 ;;
    esac
done

[ -f "$JAMMY/env.sh" ]   || { echo "[c5] env.sh が無い: $JAMMY/env.sh" >&2; exit 2; }
[ -x "$WS/bin/$NODE_BIN" ] || { echo "[c5] 実行ファイルが無い: $WS/bin/$NODE_BIN" >&2; exit 2; }
[ -x "$WS/bin/fastlio_mapping" ] || { echo "[c5] fastlio_mapping が無い: $WS/bin" >&2; exit 2; }
[ -f "$FASTLIO_YAML" ]   || { echo "[c5] 設定が無い: $FASTLIO_YAML" >&2; exit 2; }
[ -f "$O3D_YAML" ]       || { echo "[c5] 設定が無い: $O3D_YAML" >&2; exit 2; }
[ -f "$MAP" ]            || { echo "[c5] 事前地図が無い: $MAP" >&2; exit 2; }

# ── 幾何の計算（要点 1 と 2）─────────────────────────────────────
# 出力 3 行: initialpose 配列 / body->base_link の xyz / 同 rpy(rad)
GEO="$(python3 - "$LIVOX_XYZ" "$LIVOX_RPY_DEG" "$INIT_BASELINK" <<'PY'
import sys, math
xyz  = [float(v) for v in sys.argv[1].split()]
rpyd = [float(v) for v in sys.argv[2].split()]
ib   = [float(v) for v in sys.argv[3].split()]
def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mv(A,v): return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]
def Rx(a): c,s=math.cos(a),math.sin(a); return [[1,0,0],[0,c,-s],[0,s,c]]
def Ry(a): c,s=math.cos(a),math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]
def Rz(a): c,s=math.cos(a),math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]
# ROS の rpy 規約: R = Rz(yaw)Ry(pitch)Rx(roll)
def ros_rpy(r,p,y): return mm(Rz(y), mm(Ry(p), Rx(r)))
def to_ros_rpy(R):
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    return math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])
def to_xyz_intrinsic(R):   # R = Rx(a)Ry(b)Rz(c)  <- open3d_loc の規約
    b = math.asin(max(-1.0, min(1.0, R[0][2])))
    return math.atan2(-R[1][2], R[2][2]), b, math.atan2(-R[0][1], R[0][0])
# E: base_link -> livox_frame
ER = ros_rpy(math.radians(rpyd[0]), math.radians(rpyd[1]), math.radians(rpyd[2]))
Et = xyz
# E^-1 : body(=livox_frame) -> base_link
ERi = [[ER[j][i] for j in range(3)] for i in range(3)]
Eti = [-v for v in mv(ERi, Et)]
r,p,y = to_ros_rpy(ERi)
# initialpose = T_map_baselink(t0) · E
MR = mm(Rz(math.radians(ib[2])), ER)
Mt = [a+b for a,b in zip([ib[0], ib[1], 0.0], mv(Rz(math.radians(ib[2])), Et))]
a,b2,c = to_xyz_intrinsic(MR)
print("[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]" % (Mt[0],Mt[1],Mt[2],
      math.degrees(a),math.degrees(b2),math.degrees(c)))
print("%.6f %.6f %.6f" % tuple(Eti))
print("%.9f %.9f %.9f" % (r,p,y))
PY
)" || { echo "[c5] 幾何の計算に失敗した" >&2; exit 1; }
INITPOSE="$(echo "$GEO" | sed -n 1p)"
BL_XYZ="$(echo "$GEO" | sed -n 2p)"
BL_RPY="$(echo "$GEO" | sed -n 3p)"
[ -n "$INITPOSE" ] && [ -n "$BL_RPY" ] || { echo "[c5] 幾何の出力が空" >&2; exit 1; }

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

# ws_c5 の型サポートを見せるためにライブラリパスを伸ばす。
# ⚠️ LD_LIBRARY_PATH は **export しない**（罠 5）。--library-path だけで足りる
jrun_ws() { local exe="$1"; shift
    env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS:$WS/lib" "$exe" "$@"; }

say "実装: $NODE_BIN"
say "事前地図: $MAP"
say "base_link 初期姿勢 (x y yaw_deg): $INIT_BASELINK"
say "initialpose (= T_map_odom, Rx*Ry*Rz deg): $INITPOSE"
say "静的 TF body->base_link: xyz ($BL_XYZ) / rpy ($BL_RPY) rad"

PIDS=""
cleanup() {
    for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
    sleep 1
    for p in $PIDS; do kill -KILL "$p" 2>/dev/null; done
    return 0
}
trap cleanup EXIT INT TERM

# ── 静的 TF 3 本（要点 5・6）────────────────────────────────────
# ⚠️ `jrun <実行ファイル>` で呼ばないこと。write_jammy_env.sh が $ROS/bin の ELF を
#    全部 `#!/bin/sh` のラッパにしているので、ローダに渡すと落ちる。`ros2 run` を使う。
read -r EX EY EZ <<EOF
$LIVOX_XYZ
EOF
read -r ER EP EYW <<EOF
$(awk -v v="$LIVOX_RPY_DEG" 'BEGIN{split(v,a," ");printf "%.9f %.9f %.9f",a[1]*3.141592653589793/180,a[2]*3.141592653589793/180,a[3]*3.141592653589793/180}')
EOF
read -r BX BY BZ <<EOF
$BL_XYZ
EOF
read -r BR BP BYW <<EOF
$BL_RPY
EOF

jros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id odom --child-frame-id camera_init \
    --ros-args -r __node:=c5_stf_odom > /tmp/pc2_c5_stf.log 2>&1 &
PIDS="$PIDS $!"
jros2 run tf2_ros static_transform_publisher --x "$BX" --y "$BY" --z "$BZ" \
    --roll "$BR" --pitch "$BP" --yaw "$BYW" \
    --frame-id body --child-frame-id base_link \
    --ros-args -r __node:=c5_stf_base >> /tmp/pc2_c5_stf.log 2>&1 &
PIDS="$PIDS $!"
jros2 run tf2_ros static_transform_publisher --x "$EX" --y "$EY" --z "$EZ" \
    --roll "$ER" --pitch "$EP" --yaw "$EYW" \
    --frame-id base_link --child-frame-id imu_link \
    --ros-args -r __node:=c5_stf_imu >> /tmp/pc2_c5_stf.log 2>&1 &
PIDS="$PIDS $!"
sleep 3
for p in $PIDS; do
    kill -0 "$p" 2>/dev/null || { echo "[c5] 静的 TF が起動しない（/tmp/pc2_c5_stf.log）" >&2; exit 1; }
done
say "静的 TF 3 本 OK"

# ── FAST-LIO2（内側）────────────────────────────────────────────
jrun_ws "$WS/bin/fastlio_mapping" --ros-args --params-file "$FASTLIO_YAML" \
    > /tmp/pc2_fastlio.log 2>&1 &
FL_PID=$!; PIDS="$PIDS $FL_PID"
sleep 3
kill -0 "$FL_PID" 2>/dev/null || { echo "[c5] FAST-LIO が起動しない（/tmp/pc2_fastlio.log）" >&2; exit 1; }
say "FAST-LIO2 起動"

# ── open3d_loc（外側）──────────────────────────────────────────
# ⚠️ initialpose とカルマンは**コマンドラインで渡す**（要点 3・4）
jrun_ws "$WS/bin/$NODE_BIN" --ros-args --params-file "$O3D_YAML" \
    -p "path_map:=$MAP" \
    -p "initialpose:=$INITPOSE" \
    -p "kf_baselink2map/x:=[0.001,0.002]" \
    -p "kf_baselink2map/y:=[0.001,0.005]" \
    -p "kf_baselink2map/z:=[0.00001,0.04]" \
    -r __node:=global_localization_node \
    > /tmp/pc2_o3dloc.log 2>&1 &
O3_PID=$!; PIDS="$PIDS $O3_PID"
sleep 3
kill -0 "$O3_PID" 2>/dev/null || { echo "[c5] open3d_loc が起動しない（/tmp/pc2_o3dloc.log）" >&2; exit 1; }
say "open3d_loc 起動（地図の読み込みに数秒かかる）"

if [ "$SECONDS_LIMIT" != "0" ]; then
    say "${SECONDS_LIMIT} 秒で止める"
    # ⚠️ **SIGINT だけでは止まらない。** open3d_loc の初期化待ちループ
    #    （"Waiting for Odometry_loc..." / "Waiting for cloud_registered_1..."）は
    #    rclcpp::ok() を見ずに sleep しているので、シグナルで抜けない。
    #    2026-09-15 に実測（40 秒指定が 300 秒たっても止まらなかった）。
    #    猶予のあとに **SIGKILL まで上げる**。
    ( sleep "$SECONDS_LIMIT"; kill -INT "$O3_PID" "$FL_PID" 2>/dev/null; \
      sleep 5; kill -KILL "$O3_PID" "$FL_PID" 2>/dev/null ) &
    PIDS="$PIDS $!"
fi
say "動作中。ログ: /tmp/pc2_fastlio.log /tmp/pc2_o3dloc.log"
wait "$O3_PID"
exit $?
