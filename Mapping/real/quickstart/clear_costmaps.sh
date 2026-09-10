#!/usr/bin/env bash
# コストマップに溜まった**消え残りの障害物**を落とす。**preflight の前に 1 回打つ。**
#
#   bash quickstart/clear_costmaps.sh
#
# ## なぜ要るのか（2026-09-10 に実機で踏んだ）
#
# 障害物層（voxel_layer）の印は、そのセルを**貫くレイが後から来ないと消えない**。
# LiDAR の死角や、人が通り過ぎた跡は残り続ける。スタックを 30 分ほど立ち上げたまま
# にしたら、機体まわり ±3 m の LETHAL 1,305 セルのうち **384 セルが
# 「事前地図にも無く、その時 LiDAR が見てもいない」消え残り**になっていた。
#
# **そのうちの 1 つがちょうどゴールのセルだった。** すると:
#
#   1. Smac2D はゴールへ行けない
#   2. `tolerance: 0.5` の中で一番近い到達可能点を返す = **機体の現在地**
#   3. コントローラは経路の終端と機体を比べるので **1.6 ms で「Reached the goal!」**
#   4. `到達 1/1` と出る。**1 度も歩いていないのに。**
#
# ⚠️ **走行のたびに自動で消すことはしない。** 毎回消すと「いま何が見えているか」が
# 走行ごとに失われ、消え残りが育っていることに気づけなくなる。**明示的に 1 手打つ。**
#
# ⚠️ 消した直後にライブの障害物は 10 Hz ですぐ戻る。つまりこれで消えるのは
# **消え残りだけ**で、本物の障害物（人・家具・壁）は残る。だから preflight の
# 「local costmap の中身が全 0 でない」は消した後も通る。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=clear

MODE="${1:-live}"
RADIUS="${G1_CLEAR_STATS_RADIUS:-3.0}"
# 消した後にライブの障害物が戻るのを待つ秒数。costmap の update は 1〜5 Hz なので
# 1 周期では足りない。**短くすると「消えた」ではなく「まだ戻っていない」を見てしまう**
SETTLE_S="${G1_CLEAR_SETTLE_S:-3}"

case "$MODE" in
    live|offline) ;;
    *) g1_die "使い方: clear_costmaps.sh [live|offline]" ;;
esac

g1_container_running || g1_die "コンテナ $G1_RVIZ_NAME が動いていない"

STATS="python3 /work/G1_Hackason/Mapping/real/quickstart/costmap_stats.py"
SIMFLAG=""
[ "$MODE" = live ] && SIMFLAG="--no-sim-time"

stats() {
    for t in /global_costmap/costmap /local_costmap/costmap; do
        g1_exec "$MODE" "$STATS $t --radius $RADIUS $SIMFLAG" 2>&1 \
            | grep -vE 'TF_OLD_DATA|Possible reasons|buffer_core' || true
    done
}

g1_say "消す前"
stats

for svc in /global_costmap/clear_entirely_global_costmap \
           /local_costmap/clear_entirely_local_costmap; do
    # ⚠️ `ros2 service call` は応答が来ないと待ち続ける。上限を付ける
    if g1_exec "$MODE" \
        "timeout 15 ros2 service call $svc nav2_msgs/srv/ClearEntireCostmap '{}'" \
        >/dev/null 2>&1; then
        g1_ok "$svc"
    else
        g1_bad "$svc が失敗した（Nav2 が上がっているか）"
    fi
done

g1_say "ライブの障害物が戻るのを $SETTLE_S 秒待つ"
sleep "$SETTLE_S"

g1_say "消した後"
stats

cat <<'EOM'

  読み方: **減った分が消え残り**（＝ 誰も見ていないのに通れないと言っていたセル）。
          残った分は本物（壁・人・家具）なので、これは消えないのが正しい。
          減らないなら消え残りは無い。次は preflight.sh へ。
EOM
