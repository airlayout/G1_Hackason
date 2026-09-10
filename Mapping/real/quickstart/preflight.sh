#!/usr/bin/env bash
# **実機に触る前の単一のゲート。** 全部 OK でなければ足を繋がない。
#
#   bash quickstart/preflight.sh          # 全部見る
#   bash quickstart/preflight.sh --quiet  # NG だけ出す
#
# ## なぜ要るのか
#
# 2026-09-09 のセッションで**同じ確認を手で 4 回繰り返した。**現地でこれをやると
# 実機の時間を食う。しかも今日引っかかった 3 つは全部「落ちないので気づけない」型で、
# **見るべき所を知らないと通ってしまう**:
#
#   - map -> base_link の pitch が 12°/z が 1.27m（経路が機体系でずれて右へ逸れる）
#   - nav2.log の Extrapolation Error（controller が経路を変換できず /cmd_vel が出ない）
#   - local costmap がフル版を出さない（静止中は原点が動かないため）
#
# ⚠️ **これは経路の確認ではない。** 経路は check_link.sh が見る。こちらは
# 「スタックが上がった後、歩かせてよい状態か」だけを見る。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=preflight

QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1
FAIL=0
NAME="$G1_RVIZ_NAME"
DDS="$(g1_dds_uri live)"

step() { [ "$QUIET" = "1" ] || { echo; echo "── $*"; }; }
ok()   { [ "$QUIET" = "1" ] || g1_ok "$*"; }
bad()  { g1_bad "$*"; FAIL=1; }
warn() { [ "$QUIET" = "1" ] || g1_warn "$*"; }

# コンテナ内で ROS のコマンドを 1 つ流す
ros() {
    docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
        bash -c "source /opt/ros/humble/setup.bash && $*" 2>/dev/null
}
# ⚠️ hz は --no-daemon を受け付けない。SIGTERM だと集計を印字しないので -s INT
#
# ⚠️ **`hz` は必ず窓いっぱい待つ。**（早く終わる術は無い）ここが preflight の
# 所要のほぼ全部だった（12 s x 3 本 = 36 s）。窓はトピックのレートで決める:
# 2 通来れば数字は出るので、**10 Hz なら 3 s、0.67 Hz なら 8 s**あれば足りる。
# 一律に縮めると遅いトピック（global costmap 0.67 Hz）で「取れない」に化ける。
RATE_SCALE="${G1_PREFLIGHT_RATE_SCALE:-1.0}"   # 1 未満で更に急ぐ / 1 超で慎重に
rate() {  # rate <topic> [窓の秒数]
    local w
    w="$(awk -v b="${2:-5}" -v s="$RATE_SCALE" 'BEGIN{v=b*s; printf "%d", (v<2?2:v)}')"
    ros "stdbuf -oL timeout -s INT $w ros2 topic hz $1" \
        | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | head -1
}
# 購読者・発行者の数
count() { ros "timeout 10 ros2 topic info --no-daemon $1" \
          | sed -n "s/^$2 count: //p" | head -1; }

echo "=============================================================="
echo " 歩かせる前の確認"
echo "=============================================================="

# ── 1. 測位 ─────────────────────────────────────────────────────
step "1. 測位（MOLA-LO）"
Q="$(ros "stdbuf -oL timeout -s INT 12 ros2 topic echo --once /lidar_odometry/pose_quality" \
     | sed -n 's/^data: //p' | head -1)"
if [ -z "$Q" ]; then
    bad "ICP 品質が取れない（MOLA が居ない）"
elif awk "BEGIN{exit !($Q >= 0.70)}"; then
    ok "ICP 品質 ${Q}（実測の常用域 0.83〜0.85）"
else
    bad "ICP 品質 $Q が低い。地図の層が違うか初期姿勢が合っていない"
fi
R="$(rate /tf 3)"
if [ -n "$R" ] && awk "BEGIN{exit !($R >= 8.0)}"; then
    ok "/tf $R Hz（LiDAR が 10Hz なのでこの水準）"
else
    bad "/tf のレートが低い（${R:-取れない}）"
fi

# ── 2. base_link の定義（今日の落とし穴）─────────────────────────
step "2. base_link が水平・床面か（⚠️ ここが今日の落とし穴）"
TF="$(ros "timeout 3 ros2 run tf2_ros tf2_echo map base_link" \
      | grep -m1 -A3 'Translation' | tr '\n' ' ')"
Z="$(echo "$TF" | sed -n 's/.*Translation: \[[^,]*, [^,]*, \([-0-9.]*\)\].*/\1/p')"
PITCH="$(echo "$TF" | sed -n 's/.*RPY (degree) \[[^,]*, \([-0-9.]*\),.*/\1/p')"
if [ -z "$Z" ] || [ -z "$PITCH" ]; then
    bad "map -> base_link が引けない"
else
    if awk "BEGIN{exit !(($Z<0.20)&&($Z>-0.20))}"; then
        ok "base_link の高さ z = $Z m（床面）"
    else
        bad "base_link の高さ z = $Z m。**0 付近でないと経路が機体系でずれる**"
        echo "     → live の LIVOX_XYZ / 初期姿勢が offline 用になっていないか"
    fi
    if awk "BEGIN{exit !(($PITCH<3.0)&&($PITCH>-3.0))}"; then
        ok "base_link の pitch = $PITCH °（水平）"
    else
        bad "base_link の pitch = $PITCH °。**経路が前方にずれて右へ逸れる**"
        echo "     → LIVOX_RPY_DEG が内蔵 odom 基準の値（pitch -8.41）になっていないか"
    fi
fi
# ⚠️ **`... | grep -q` にしてはいけない**（2026-09-10 に踏んだ）。grep -q は一致した
# 時点で終わるので、流し続けている上流の tf2_echo が SIGPIPE(141) で死ぬ。
# `set -o pipefail` はパイプライン全体を 141 にするので、**引けているのに「引けない」**
# と出る（実測 PIPESTATUS=141 0 ＝ grep 自身は一致している）。
# 変数に受けてから case で見る。パイプを作らないので取り違えようがない。
OB="$(ros "timeout 3 ros2 run tf2_ros tf2_echo odom base_link")"
case "$OB" in
    *Translation*) ok "odom -> base_link も引ける" ;;
    *)             bad "odom -> base_link が引けない（map -> odom の静的変換が出ていない）" ;;
esac

# ── 3. コストマップ ─────────────────────────────────────────────
step "3. コストマップ"
# local は 1.67 Hz / global は 0.67 Hz。**2 通来れば出る**ので窓はこの程度でよい
for spec in "/local_costmap/costmap 5" "/global_costmap/costmap 8"; do
    t="${spec% *}"; w="${spec##* }"
    R="$(rate "$t" "$w")"
    if [ -n "$R" ]; then
        ok "$t $R Hz"
    else
        bad "$t が周期的に届かない"
        echo "     → always_send_full_costmap が両方に入っているか（静止中は local も出ない）"
    fi
done
# ⚠️ **1 回だけ取る。**以前はセル数と占有数で `ros2 topic echo --once` を
# 2 回打っており、同じメッセージを 2 度取りに行っていた（往復が丸ごと無駄）
CELLS="$(ros "stdbuf -oL timeout -s INT 15 ros2 topic echo --once --field data /local_costmap/costmap" \
         | tr ',[]' '   ' | tr ' ' '\n' | grep -E '^-?[0-9]+$' || true)"
NZ="$(printf '%s\n' "$CELLS" | grep -cE '^-?[0-9]+$' || true)"
OCC="$(printf '%s\n' "$CELLS" | awk '$1+0>=99{n++} END{print n+0}')"
if [ "${NZ:-0}" -gt 0 ] && [ "${OCC:-0}" -gt 0 ]; then
    ok "local costmap の中身: 全 $NZ セル中 占有 $OCC"
else
    bad "local costmap の中身が全部 0（LiDAR が層に入っていない）"
fi

# ── 4. Nav2 の健全性 ───────────────────────────────────────────
step "4. Nav2"
# ⚠️ **ログを読む前にプロセスの生存を見る。** nav2.log は前回の起動のものが残るので、
# 止まっている Nav2 のログを読んで「Extrapolation Error 0 件」と言ってしまう
# （2026-09-09 にこのスクリプトのテストで踏んだ）。
if docker exec "$NAME" pgrep -f "nav2_bt_navigator/bt_navigato[r]" >/dev/null 2>&1; then
    ok "bt_navigator が動いている"
    # ⚠️ grep -c は 0 件でも "0" を印字して **exit 1** する。
    # `|| echo 0` を足すと "0" が 2 行になり数値比較が壊れる。head -1 で 1 行に絞る
    logcount() {
        docker exec "$NAME" grep -c "$1" /home/ubuntu/nav2.log 2>/dev/null | head -1
    }
    EX="$(logcount 'Extrapolation Error')"; EX="${EX:-0}"
    if [ "$EX" = "0" ]; then
        ok "nav2.log の Extrapolation Error 0 件"
    else
        bad "Extrapolation Error $EX 件。**controller が経路を変換できず /cmd_vel が出ない**"
        echo "     → local_costmap.global_frame が odom になっていないか（map にする）"
    fi
    DR="$(logcount 'dropping message')"; DR="${DR:-0}"
    [ "$DR" = "0" ] && ok "LiDAR の落とし 0 件" || warn "LiDAR を $DR 件落としている"
else
    bad "bt_navigator が居ない。Nav2 が起動していない"
    echo "     → G1_USE_MOLA=1 bash quickstart/nav_stack.sh live"
    echo "     （nav2.log は前回の起動のものが残るので、ここでログは読まない）"
fi
PS="$(count /planner_selector Subscription)"
if [ "${PS:-0}" -ge 1 ]; then
    ok "/planner_selector の購読者 ${PS}（BT が差し替わっている）"
else
    bad "/planner_selector に購読者が居ない。**Smac2D を選べない**"
    echo "     → bt_navigator の default_nav_to_pose_bt_xml が navigate_g1.xml か"
fi

# ── 5. 足が繋がっているか ───────────────────────────────────────
step "5. 足（⚠️ 意味を取り違えないこと）"
CS="$(count /cmd_vel Subscription)"
if [ "${CS:-0}" = "0" ]; then
    ok "/cmd_vel の購読者 0 → **足は繋がっていない**（段 D はこの状態で測る）"
else
    warn "/cmd_vel の購読者 $CS → **足が繋がっている。指令を投げれば歩く**"
fi

# ── 6. 逸脱ガード ───────────────────────────────────────────────
step "6. 逸脱ガード"
if docker exec "$NAME" pgrep -f "stray_guar[d].py" >/dev/null 2>&1; then
    ok "stray_guard.py が常駐している"
else
    warn "stray_guard.py が居ない。**RViz からクリックするなら起こすこと**"
    echo "     → docker exec -d ... python3 .../stray_guard.py --no-sim-time"
fi

echo
echo "=============================================================="
if [ "$FAIL" = "0" ]; then
    echo " 判定: 歩かせてよい"
else
    echo " 判定: **NG を潰すまで足を繋がない**"
fi
echo "=============================================================="
exit "$FAIL"
