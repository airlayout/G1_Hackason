#!/usr/bin/env bash
# 11-04（画像だけの生徒 V2）: 種（＝扉の組）を替えて collect_vis_1104.sh を続けて回す。1 回目だけ 2 分の空き待ち。
# 使い方（OMEN の ~/work_inoue/doorman6）: SEEDS="11 12 13" NUM_ENVS=256 MINUTES=4 MAX_FRAMES=12500 bash collect_seeds_1104.sh <出力の親> [hydra 引数...]
#   止め札: ~/work_inoue/STOP_DOOR_VIS_COLLECT（種の境目で止まる）
set -uo pipefail
ROOT="$1"; shift
mkdir -p "$ROOT"
first=1
for s in ${SEEDS:?}; do
  [ -e "$HOME/work_inoue/STOP_DOOR_VIS_COLLECT" ] && { echo "=== 止め札"; break; }
  [ -d "$ROOT/s$s/frames" ] && [ "$(ls "$ROOT/s$s/frames" | wc -l)" -gt 0 ] && { echo "=== s$s は済み"; continue; }
  SKIP=$([ "$first" = 1 ] && echo 0 || echo 1)
  SEED="$s" SKIP_GATE="$SKIP" bash "$HOME/work_inoue/doorman6/collect_vis_1104.sh" "$ROOT/s$s" "$@"
  first=0
  sleep 5
done
echo "=== [$(date '+%m-%d %T')] collect_seeds 終わり 枚数 $(ls "$ROOT"/s*/frames/shard_*.npz 2>/dev/null | wc -l) 束"
