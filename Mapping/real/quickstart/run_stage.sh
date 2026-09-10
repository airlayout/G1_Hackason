#!/usr/bin/env bash
# 実機で 1 本走らせ、**走行中の bag を録って重畳まで出す**。現地の手数を減らすため。
#
#   bash quickstart/run_stage.sh ahead 1.0        # 今の向きに真っ直ぐ 1.0m を 1 本
#   bash quickstart/run_stage.sh goal 3.2 1.5     # map 系の絶対座標へ（事前検算つき）
#   bash quickstart/run_stage.sh wp 3             # waypoint から 3 本（--range で選ぶ）
#
# ## なぜ要るのか
#
# 2026-09-09 のセッションでは 1 本ごとに「bag を録る」「走らせる」「重畳を測る」を
# 手で並べていた。**そして重畳を録り忘れたので歩行中の測位を測れなかった**
# （静止の 89.2% しか手元に無い）。順番を焼き込む。
#
# ⚠️ **preflight.sh を先に通すこと。**このスクリプトは状態を確認しない。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=stage

NAME="$G1_RVIZ_NAME"
DDS="$(g1_dds_uri live)"
SESSION="${G1_SESSION:-20260906T135940_UiS_room_v3}"
RUNS="/work/G1_Hackason/Mapping/real/runs"
MAP="${G1_NAV_MAP_YAML:-$RUNS/$SESSION/map/nav_map.yaml}"
WP="${G1_WAYPOINTS:-$RUNS/$SESSION/measure_20260908/waypoints.json}"
PLANNER="${G1_PLANNER:-Smac2D}"
TIMEOUT="${G1_GOAL_TIMEOUT:-40}"
MARGIN="${G1_MAX_STRAY:-0.5}"
VENV="$G1_REPO_ROOT/G1_Hackason/Navigation/.venv/bin/python"
NAV2_YAML="${G1_NAV2_YAML:-$G1_REPO_ROOT/G1_Hackason/Navigation/nav2/g1_nav2.yaml}"
# yaml を読めないときの保険。**黙って 0 にしない**（0 だと検査が素通りする）
FALLBACK_XY_TOL=0.30
FALLBACK_PLANNER_TOL=0.50
TAG="${G1_TAG_NAME:-$(date +%Y%m%dT%H%M%S)}"
OUT="$RUNS/stage_$TAG"

MODE="${1:-}"
say() { echo "[stage] $*"; }
die() { echo "[stage] $*" >&2; exit 1; }

ros()  { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
rosd() { docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }

# ゴールの許容とプランナの許容を **yaml から読む**（焼き込むと必ずずれる）
read_tolerances() {
    [ -x "$VENV" ] && [ -f "$NAV2_YAML" ] || return 1
    "$VENV" - "$NAV2_YAML" "$PLANNER" <<'PY' 2>/dev/null
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
ctrl = cfg["controller_server"]["ros__parameters"]
gc = next(v for v in ctrl.values() if isinstance(v, dict) and "xy_goal_tolerance" in v)
print(gc["xy_goal_tolerance"], cfg["planner_server"]["ros__parameters"][sys.argv[2]]["tolerance"])
PY
}

# **短すぎるゴールを弾く（2026-09-10 に実機で踏んだ）。**
#
# `ahead 0.5` を投げたら `到達 1/1` と出たのに、bag の map -> base_link 119 サンプル
# 11.83 秒で**開始点からの最大距離は 0.049 m** だった。**1 歩も歩いていない。**
# 機構はこう:
#
#   * Smac2D は `tolerance`（0.50 m）以内で一番近い到達可能点を返す
#   * ゴールが 0.50 m 先だと、**機体の現在地そのもの**がその点の候補になる
#   * すると経路は 1 点だけ。コントローラは経路の終端と機体を比べるので
#     **1.6 ms で「Reached the goal!」**と言い、bt_navigator が Goal succeeded を返す
#
# 意味のある移動量は `D - xy_goal_tolerance` なので、これがプランナの許容を
# 超えていないと「動かずに成功」が原理的に起こり得る。**距離で弾くのが確実。**
guard_ahead_distance() {
    local d="$1" xy pt min short src="yaml"
    if ! read -r xy pt < <(read_tolerances); then
        xy="$FALLBACK_XY_TOL"; pt="$FALLBACK_PLANNER_TOL"; src="既定値（yaml を読めなかった）"
    fi
    min="$(awk -v a="$xy" -v b="$pt" 'BEGIN{printf "%.2f", a+b}')"
    say "ゴールの許容 ${xy} m / ${PLANNER} の許容 ${pt} m → 最短 ${min} m 超（${src}）"
    if awk -v d="$d" -v m="$min" 'BEGIN{exit !(d > m)}'; then
        return 0
    fi
    if [ "${G1_ALLOW_SHORT_AHEAD:-0}" = 1 ]; then
        say "⚠️ ${d} m は最短 ${min} m を超えていない。**G1_ALLOW_SHORT_AHEAD=1 なので続ける**"
        say "⚠️ この回は「動かずに到達」が起こり得る。**合否の根拠に使わないこと**"
        return 0
    fi
    short="$(awk -v d="$d" -v x="$xy" 'BEGIN{printf "%.2f", d-x}')"
    die "ahead ${d} m は短すぎる（最短 ${min} m 超）。**動かずに「到達」と出る。**
       意味のある移動量は ${d} - ${xy} = ${short} m で、$PLANNER の許容 ${pt} m に収まってしまう。
       → ahead 1.0 以上にする。どうしても短くしたいなら G1_ALLOW_SHORT_AHEAD=1"
}

case "$MODE" in
  ahead) D="${2:?距離[m] を渡すこと}"
         guard_ahead_distance "$D"
         NAV_ARGS="--ahead $D --tries 1" ;;
  goal)  GX="${2:?X Y を渡すこと}"; GY="${3:?X Y を渡すこと}"
         NAV_ARGS="" ;;                            # ゴールは下で作る
  wp)    N="${2:-3}"
         NAV_ARGS="--waypoints $WP --tries $N --range ${G1_RANGE:-1.8 3.2}" ;;
  *)     die "使い方: run_stage.sh {ahead D | goal X Y | wp N}" ;;
esac

# ── 絶対座標のときは歩く前に経路を検算する ───────────────────────────
if [ "$MODE" = "goal" ]; then
    say "歩く前に ComputePathToPose で検算する（${GX}, ${GY}）"
    if ! ros "python3 /work/G1_Hackason/Mapping/real/quickstart/check_planning.py \
              --goal $GX $GY --tries 3 --no-sim-time --planner $PLANNER" \
         2>&1 | grep -vE 'TF_OLD_DATA|Possible reasons|buffer_core' | tail -20; then
        die "検算で落ちた。**歩かせない**"
    fi
    # waypoints を使わず 1 点だけを渡すため、その場で候補ファイルを作る
    ros "python3 -c \"import json,sys;json.dump([{'x':$GX,'y':$GY,'clearance':float('nan')}],open('/tmp/one_wp.json','w'))\""
    NAV_ARGS="--waypoints /tmp/one_wp.json --tries 1 --range 0.1 99"
fi

# ── 走行中の bag を録る（**これが歩行中の重畳の素**）─────────────────
say "記録を開始する -> $OUT/bag"
ros "mkdir -p $OUT"
rosd "cd $OUT && ros2 bag record -o bag /utlidar/cloud_livox_mid360 /tf /tf_static \
      > $OUT/bagrecord.log 2>&1"
sleep 3

say "走らせる（$MODE / planner $PLANNER / 逸脱ガード余裕 ${MARGIN}m / 上限 ${TIMEOUT}s）"
ros "cd /work/G1_Hackason/Mapping/real/quickstart && \
     python3 check_navigation.py $NAV_ARGS --no-sim-time --planner-id $PLANNER \
       --max-stray $MARGIN --timeout $TIMEOUT --record $OUT" \
    2>&1 | grep -vE 'TF_OLD_DATA|Possible reasons|buffer_core|rcutils|overwritten|serdata|reset_error|^<<<|^>>>' | tail -14

# ⚠️ bag は SIGINT で閉じさせる。SIGTERM だとキャッシュを書き出さずに死ぬ
say "記録を止める"
docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null || true
sleep 4

# ── 歩行中の重畳を出す（**静止の値では判定にならない**）───────────────
say "歩行中の重畳を測る（既知の立脚静止は 89.2%）"
# コンテナの /work は Mac の $G1_REPO_ROOT。**パスを焼かずにここで読み替える**
tolocal() { printf '%s\n' "${1/#\/work/$G1_REPO_ROOT}"; }
LOCAL_OUT="$(tolocal "$OUT")"
if [ -x "$VENV" ] && [ -d "$LOCAL_OUT/bag" ]; then
    # ⚠️ 解析は Mac 側でやる（コンテナに numpy も scipy も無い）
    "$VENV" "$HERE/measure_overlay.py" "$LOCAL_OUT/bag" "$(tolocal "$MAP")" 2>&1 | tail -8
else
    say "⚠️ 重畳を測れない（venv か bag が無い）。手で: "
    say "   Navigation/.venv/bin/python quickstart/measure_overlay.py <bag> <nav_map.yaml>"
fi

say "全部ここに: $LOCAL_OUT"
