#!/usr/bin/env bash
# 実機で歩かせ、**走行中の bag を録って重畳まで出す**。現地の手数を減らすため。
#
#   bash quickstart/run_stage.sh ahead 1.0              # 今の向きに真っ直ぐ 1.0m
#   bash quickstart/run_stage.sh ahead 1.0 --repeat 3   # 続けて 3 本（1 本ずつ別フォルダ）
#   bash quickstart/run_stage.sh goal 3.2 1.5           # map 系の絶対座標へ（事前検算つき）
#   bash quickstart/run_stage.sh wp 3                   # waypoint から 3 本（--range で選ぶ）
#
# ## なぜ要るのか
#
# 2026-09-09 のセッションでは 1 本ごとに「bag を録る」「走らせる」「重畳を測る」を
# 手で並べていた。**そして重畳を録り忘れたので歩行中の測位を測れなかった**
# （静止の 89.2% しか手元に無い）。順番を焼き込む。
#
# ## 実験段階の方針（2026-09-10 に決めた）
#
# **ここは実験を回す道具であって、安全装置ではない。歩き出す前で止めない。**
# 商用ではないので「歩く前の関門」は置かず、**気になることは警告で出してそのまま走る**。
# そのかわり**走った後の裏取りは落とさない**（check_navigation.py の verify_arrival）。
# 機体を止める役目は走行中の逸脱ガード（stray_guard.py / --max-stray）だけが持つ。
#
# 速さのつまみ。既定は 2026-09-10 の実測から決めた（実測の 8〜60 倍の余裕がある）:
#
#   G1_BAG_WARMUP_S   1.0  bag が購読を終えるまで実測 **0.12 s**（旧 3 s 固定）
#   G1_BAG_FLUSH_S    1.0  bag の書き出し完了まで実測 **0.015 s**（旧 4 s 固定）
#   G1_NAV_WARMUP_S   2.0  check_navigation の TF 溜め（旧 5 s 固定）
#   G1_REPEAT         1    --repeat と同じ
#
# ⚠️ **preflight.sh は先に通しておくこと。**このスクリプトは状態を確認しない。
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

BAG_WARMUP_S="${G1_BAG_WARMUP_S:-1.0}"
BAG_FLUSH_S="${G1_BAG_FLUSH_S:-1.0}"
NAV_WARMUP_S="${G1_NAV_WARMUP_S:-2.0}"
REPEAT="${G1_REPEAT:-1}"

TAG="${G1_TAG_NAME:-$(date +%Y%m%dT%H%M%S)}"

say() { echo "[stage] $*"; }
die() { echo "[stage] $*" >&2; exit 1; }

ros()  { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
rosd() { docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
# コンテナの /work は Mac の $G1_REPO_ROOT。**パスを焼かずにここで読み替える**
tolocal() { printf '%s\n' "${1/#\/work/$G1_REPO_ROOT}"; }

# ── --repeat N を引数から抜く ──────────────────────────────────────────
# ⚠️ 配列は使わない。macOS の /bin/bash は 3.2 で、`set -u` の下では
# 空配列の "${a[@]}" がエラーになる。位置パラメータを回して取り除く
_n=$#
while [ "$_n" -gt 0 ]; do
    case "$1" in
        # ⚠️ _n は「まだ回していない元の引数の数」。末尾に --repeat だけ置かれた場合、
        # $2 には**回り込んだ別の引数**が入っているので、$2 の中身では判定できない
        --repeat) [ "$_n" -ge 2 ] || die "--repeat には回数が要る（例: --repeat 3）"
                  REPEAT="$2"; shift 2; _n=$((_n - 2)) ;;
        *)        set -- "$@" "$1"; shift; _n=$((_n - 1)) ;;
    esac
done
case "$REPEAT" in
    ''|*[!0-9]*) die "--repeat は正の整数で（受け取ったのは: ${REPEAT}）" ;;
esac
[ "$REPEAT" -ge 1 ] || die "--repeat は 1 以上で"

MODE="${1:-}"

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

# **短すぎるゴールは結果が嘘になる（2026-09-10 に実機で踏んだ）。**
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
# 超えていないと「動かずに成功」が原理的に起こり得る。
#
# ⚠️ **実験段階なので止めない。警告だけ出して走る。**「動かずに成功」は
# check_navigation.py の verify_arrival() が走行後に幾何で捕まえるので見逃さない。
warn_if_ahead_too_short() {
    local d="$1" xy pt min short src="yaml"
    if ! read -r xy pt < <(read_tolerances); then
        xy="$FALLBACK_XY_TOL"; pt="$FALLBACK_PLANNER_TOL"; src="既定値（yaml を読めなかった）"
    fi
    min="$(awk -v a="$xy" -v b="$pt" 'BEGIN{printf "%.2f", a+b}')"
    if awk -v d="$d" -v m="$min" 'BEGIN{exit !(d > m)}'; then
        return 0
    fi
    short="$(awk -v d="$d" -v x="$xy" 'BEGIN{printf "%.2f", d-x}')"
    say "⚠️ ahead ${d} m は短い（実質の移動量 ${d} - ${xy} = ${short} m ≦ ${PLANNER} の許容 ${pt} m / ${src}）"
    say "⚠️ **1 歩も歩かずに「到達」と出ることがある。**走行後の実移動量を必ず見ること"
}

case "$MODE" in
  ahead) D="${2:?距離[m] を渡すこと}"
         warn_if_ahead_too_short "$D"
         NAV_ARGS="--ahead $D --tries 1" ;;
  goal)  GX="${2:?X Y を渡すこと}"; GY="${3:?X Y を渡すこと}"
         NAV_ARGS="" ;;                            # ゴールは下で作る
  wp)    N="${2:-3}"
         NAV_ARGS="--waypoints $WP --tries $N --range ${G1_RANGE:-1.8 3.2}" ;;
  *)     die "使い方: run_stage.sh {ahead D | goal X Y | wp N} [--repeat N]" ;;
esac

# ── 絶対座標のときは歩く前に経路を検算する（繰り返しの外で 1 回だけ）──────
if [ "$MODE" = "goal" ]; then
    say "歩く前に ComputePathToPose で検算する（${GX}, ${GY}）"
    # ⚠️ **落ちても止めない**（実験段階）。検算は材料であって関門ではない
    ros "python3 /work/G1_Hackason/Mapping/real/quickstart/check_planning.py \
              --goal $GX $GY --tries 3 --no-sim-time --planner $PLANNER" \
         2>&1 | grep -vE 'TF_OLD_DATA|Possible reasons|buffer_core' | tail -20 \
      || say "⚠️ 検算が通らなかった。**そのまま走らせる**（守るのは逸脱ガード）"
    # waypoints を使わず 1 点だけを渡すため、その場で候補ファイルを作る
    ros "python3 -c \"import json,sys;json.dump([{'x':$GX,'y':$GY,'clearance':float('nan')}],open('/tmp/one_wp.json','w'))\""
    NAV_ARGS="--waypoints /tmp/one_wp.json --tries 1 --range 0.1 99"
    [ "$REPEAT" -gt 1 ] && say "⚠️ goal の繰り返しは 2 本目以降「既に着いている点」が相手になる"
fi

# ── 1 本ぶん。**途中で止めない。**結果は SUMMARY に 1 行足すだけ ──────────
SUMMARY=""
run_once() {
    local out="$1" local_out rc ov ov10 reach fake
    local_out="$(tolocal "$out")"

    say "記録を開始する -> $out/bag"
    ros "mkdir -p $out"
    rosd "cd $out && ros2 bag record -o bag /utlidar/cloud_livox_mid360 /tf /tf_static \
          > $out/bagrecord.log 2>&1"
    sleep "$BAG_WARMUP_S"

    say "走らせる（$MODE / planner $PLANNER / 逸脱ガード余裕 ${MARGIN}m / 上限 ${TIMEOUT}s）"
    # ⚠️ 画面には末尾だけ出すが、**全文は navigate.log に残す**（後から追える）
    ros "cd /work/G1_Hackason/Mapping/real/quickstart && \
         python3 check_navigation.py $NAV_ARGS --no-sim-time --planner-id $PLANNER \
           --warmup $NAV_WARMUP_S --max-stray $MARGIN --timeout $TIMEOUT --record $out" \
        2>&1 | grep -vE 'TF_OLD_DATA|Possible reasons|buffer_core|rcutils|overwritten|serdata|reset_error|^<<<|^>>>' \
             | tee "$local_out/navigate.log" | tail -14
    rc=${PIPESTATUS[0]}

    # ⚠️ bag は SIGINT で閉じさせる。SIGTERM だとキャッシュを書き出さずに死ぬ
    say "記録を止める"
    docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null || true
    sleep "$BAG_FLUSH_S"

    # ── 歩行中の重畳を出す（**静止の値では判定にならない**）───────────────
    say "歩行中の重畳を測る（既知の立脚静止は 89.2%）"
    ov="-"; ov10="-"
    if [ -x "$VENV" ] && [ -d "$local_out/bag" ]; then
        # ⚠️ 解析は Mac 側でやる（コンテナに numpy も scipy も無い）。109 枚で 0.54 s
        "$VENV" "$HERE/measure_overlay.py" "$local_out/bag" "$(tolocal "$MAP")" \
            > "$local_out/overlay.txt" 2>&1
        tail -8 "$local_out/overlay.txt"
        ov="$(sed -n 's/.*占有セルに乗った割合 *\([0-9.]*\) %.*/\1/p' "$local_out/overlay.txt" | head -1)"
        ov10="$(sed -n 's/.*まで許した割合 *\([0-9.]*\) %.*/\1/p'     "$local_out/overlay.txt" | head -1)"
    else
        say "⚠️ 重畳を測れない（venv か bag が無い）。手で: "
        say "   Navigation/.venv/bin/python quickstart/measure_overlay.py <bag> <nav_map.yaml>"
    fi

    reach="$(sed -n 's/^== 到達 \(.*\) ==$/\1/p' "$local_out/navigate.log" 2>/dev/null | head -1)"
    fake=""
    grep -q "着いていない回が" "$local_out/navigate.log" 2>/dev/null && fake="  ⚠️動かずに成功"
    SUMMARY="$SUMMARY
  $(basename "$out")  到達 ${reach:-?}  重畳 ${ov:--} % (±10cm ${ov10:--} %)  rc=${rc}${fake}"
}

for i in $(seq 1 "$REPEAT"); do
    if [ "$REPEAT" -gt 1 ]; then
        echo; say "──── ${i} / ${REPEAT} 本目 ────"
        run_once "$RUNS/stage_${TAG}_r${i}"
    else
        run_once "$RUNS/stage_${TAG}"
    fi
done

echo
say "まとめ$SUMMARY"
say "全部ここに: $(tolocal "$RUNS")/stage_${TAG}*"
