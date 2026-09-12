#!/usr/bin/env bash
# **静止の対照**を録る。歩かせない。
#
#   bash quickstart/record_still.sh                  # 60 秒を 1 本
#   bash quickstart/record_still.sh 90               # 90 秒を 1 本
#   bash quickstart/record_still.sh 60 --repeat 3    # 60 秒を 3 本（別フォルダ）
#
# ## なぜ要るのか
#
# 門番 G0（見かけの速さ）の**誤報率の根拠が 11.8 秒の 1 本しかない**（2026-09-11、段 4）。
# しかも余裕を決めているのは歩行側ではなく**静止中の推定の震え**で、
# 端から端が 0.03 m しか動いていない記録でも 0.5 秒窓の中央値は 0.335 m/s 出る。
# つまり「どこまで震えるか」が合否を直接決めるのに、標本が 1 本しかない。
#
# 静止は**歩かせずに録れる**ので、`run_stage.sh`（歩く）から切り離してここに置く。
#
# ## 前提
#
#   G1_USE_MOLA=1 bash quickstart/nav_stack.sh live   # MOLA が map->base_link を出す
#   bash quickstart/preflight.sh                      # 「歩かせてよい」まで通しておく
#
# ⚠️ **機体は立たせておくこと。**座っていると LiDAR の高さが変わり、
# 歩行時の条件と比べられなくなる（preflight の base_link z が床面かで見る）。
#
# ⚠️ **録っている間は機体に触らない。**人が押すと「静止の対照」でなくなる。
# 本当に静止していたかは、録り終えたあとに端から端の変位で裏を取る（下の判定）。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=still

NAME="$G1_RVIZ_NAME"
DDS="$(g1_dds_uri live)"
SESSION="${G1_SESSION:-20260906T135940_UiS_room_v3}"
RUNS="/work/G1_Hackason/Mapping/real/runs"
VENV="$G1_REPO_ROOT/G1_Hackason/Navigation/.venv/bin/python"

# 重畳の基準地図。**run_stage.sh と同じ既定**（間引いていない旧 nav_map）。
# clean を基準にすると静止でも 43.9% が天井になり合格線 85% を引けない（2026-09-11 実測）。
OVERLAY_REF="${G1_OVERLAY_REF_MAP:-$RUNS/$SESSION/map/old/nav_map.yaml}"

# 記録するトピックは **record_topics.txt が唯一の定義**。ここにも直書きしない。
# 読めなければ**録り始める前に**落とす（黙って減らすと 09-10 の IMU 落ちを繰り返す）。
TOPICS_FILE="$HERE/record_topics.txt"
REC_TOPICS="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' "$TOPICS_FILE" 2>/dev/null | sed 's/ *$//')"
[ -n "$REC_TOPICS" ] || g1_die "$TOPICS_FILE から記録トピックを読めない"

BAG_WARMUP_S="${G1_BAG_WARMUP_S:-1.0}"
BAG_FLUSH_S="${G1_BAG_FLUSH_S:-1.0}"
REPEAT="${G1_REPEAT:-1}"
TAG="${G1_TAG_NAME:-$(date +%Y%m%dT%H%M%S)}"

# 「静止だった」と認めてよい端から端の変位 [m]。既知の静止（09-10）は 0.049 m。
# ⚠️ これは合否ではなく**素材の検品**。超えたら素材として使わない
STILL_MAX_DRIFT="${G1_STILL_MAX_DRIFT:-0.15}"

say() { echo "[still] $*"; }

ros()  { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
rosd() { docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
# コンテナの /work は Mac の $G1_REPO_ROOT。パスを焼かずにここで読み替える
tolocal() { printf '%s\n' "${1/#\/work/$G1_REPO_ROOT}"; }

# ── 引数 ──────────────────────────────────────────────────────────────
# ⚠️ 配列は使わない。macOS の /bin/bash は 3.2
_n=$#
while [ "$_n" -gt 0 ]; do
    case "$1" in
        --repeat) [ "$_n" -ge 2 ] || g1_die "--repeat には回数が要る（例: --repeat 3）"
                  REPEAT="$2"; shift 2; _n=$((_n - 2)) ;;
        *)        set -- "$@" "$1"; shift; _n=$((_n - 1)) ;;
    esac
done
DUR="${1:-60}"
case "$DUR" in    ''|*[!0-9]*) g1_die "秒数は正の整数で（受け取ったのは: ${DUR}）" ;; esac
case "$REPEAT" in ''|*[!0-9]*) g1_die "--repeat は正の整数で（受け取ったのは: ${REPEAT}）" ;; esac
[ "$DUR"    -ge 1 ] || g1_die "秒数は 1 以上で"
[ "$REPEAT" -ge 1 ] || g1_die "--repeat は 1 以上で"

# ── 録り始める前に「測位が生きているか」だけ見る ──────────────────────
# ⚠️ 止めはしないが、MOLA が出ていなければ /tf に map->base_link が入らず
# **録っても門番の測定に使えない**（それを録り終えてから知るのは遅い）
if ! ros "ros2 topic list 2>/dev/null" 2>/dev/null | grep -qx "/tf"; then
    say "⚠️ /tf が見えない。MOLA が上がっているか確かめること（nav_stack.sh live）"
fi
if [ ! -f "$(tolocal "$OVERLAY_REF")" ]; then
    say "⚠️ 重畳の基準地図が無い: ${OVERLAY_REF}"
    say "⚠️ そのまま録るが**重畳は測れない**。G1_OVERLAY_REF_MAP で指すこと"
fi

SUMMARY=""

record_once() {
    local out="$1" local_out drift ov
    local_out="$(tolocal "$out")"

    say "記録を開始する -> ${out}/bag（${REC_TOPICS}）"
    ros "mkdir -p $out"
    rosd "cd $out && ros2 bag record -o bag $REC_TOPICS \
          > $out/bagrecord.log 2>&1"
    sleep "$BAG_WARMUP_S"

    say "${DUR} 秒 静止して録る。**機体に触らないこと**"
    sleep "$DUR"

    # ⚠️ bag は SIGINT で閉じさせる。SIGTERM だとキャッシュを書き出さずに死ぬ
    say "記録を止める"
    docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null || true
    sleep "$BAG_FLUSH_S"

    # ── 本当に静止していたか（素材の検品）────────────────────────────
    drift="?"
    if [ -x "$VENV" ] && [ -d "$local_out/bag" ]; then
        # ⚠️ 読み手は eval_guard_speed.py から**借りる**（同じ CDR の解き方を 2 つ持たない）
        drift="$("$VENV" - "$local_out/bag" "$HERE" <<'PY' 2>/dev/null
import sys, math
from pathlib import Path
sys.path.insert(0, sys.argv[2])          # quickstart/ を渡してもらう（パスを焼かない）
import numpy as np
from eval_guard_speed import read_tf_track, apparent_speed
t = read_tf_track(Path(sys.argv[1]))
d = math.hypot(t[-1, 1] - t[0, 1], t[-1, 2] - t[0, 2])
sp, _ = apparent_speed(t)
print("{:.3f} {:.1f} {:d} {:.3f}".format(d, t[-1, 0] - t[0, 0], len(t), float(np.median(sp))))
PY
)"
        [ -n "$drift" ] || drift="?"
    fi

    # ── 重畳（静止なので既知の 78.2% と比べられる）──────────────────
    ov="-"
    if [ -x "$VENV" ] && [ -d "$local_out/bag" ]; then
        "$VENV" "$HERE/measure_overlay.py" "$local_out/bag" "$(tolocal "$OVERLAY_REF")" \
            > "$local_out/overlay.txt" 2>&1
        ov="$(sed -n 's/.*占有セルに乗った割合 *\([0-9.]*\) %.*/\1/p' "$local_out/overlay.txt" | head -1)"
        tail -6 "$local_out/overlay.txt"
    fi

    SUMMARY="$SUMMARY
  $(basename "$out")  変位/長さ/件数/速さ中央 ${drift:-?}  重畳 ${ov:--} %"
}

for i in $(seq 1 "$REPEAT"); do
    if [ "$REPEAT" -gt 1 ]; then
        echo; say "──── ${i} / ${REPEAT} 本目 ────"
        record_once "$RUNS/still_${TAG}_r${i}"
    else
        record_once "$RUNS/still_${TAG}"
    fi
done

echo
say "まとめ（変位 [m] / 長さ [s] / /tf 件数 / 見かけの速さの中央値 [m/s]）:${SUMMARY}"
say "⚠️ 変位が ${STILL_MAX_DRIFT} m を超えた本は静止の対照に使わない（誰かが触っている）"
echo
say "門番 G0 を測り直す:"
say "  Navigation/.venv/bin/python quickstart/eval_guard_speed.py \\"
say "    --walk runs/stage_20260910T182639_r1 runs/stage_20260910T182639_r2 runs/stage_20260910T182639_r3 \\"
say "    --still $(tolocal "$RUNS")/still_${TAG}* \\"
say "    --nav2-yaml Navigation/nav2/g1_nav2.yaml"
