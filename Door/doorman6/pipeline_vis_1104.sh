#!/usr/bin/env bash
# 11-04（画像だけの生徒）: 記録集め（collect_seeds_1104.sh）の終わりを待ち、V2（画像の網と扉の頭の学習）、V3（DAgger）を続けて回す。
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash pipeline_vis_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
#   WAIT_FOR: 終わりを待つ記録集めの出力（既定 vis_1104/ds1.out）。DATA: 学習に使う frames の並び（空白区切り）
#   EPOCHS（既定 8）。SKIP_DAGGER=1: V2 だけ。DAgger の設定は exp_vis_1104.sh の環境変数（NUM_ENVS・ROUNDS・ROUND_MIN）
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
WAIT_FOR="${WAIT_FOR:-$V/ds1.out}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
log "記録集めの終わりを待つ: $WAIT_FOR"
until grep -aq "collect_seeds 終わり" "$WAIT_FOR" 2>/dev/null; do sleep 20; done
sleep 10
DATA="${DATA:-$(ls -d "$V"/ds1/s*/frames "$V"/n128a/frames 2>/dev/null | tr '\n' ' ')}"
OUT="$V/pretrain_$(date +%m%d_%H%M).pt"
log "V2 の学習 data=$DATA out=$OUT"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=13G -p MemorySwapMax=0 -p CPUWeight=20 \
  env PYTHONPATH="$B/VIRAL6_vis" PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py \
  --data $DATA --out "$OUT" --epochs "${EPOCHS:-8}" --batch 256 < /dev/null
[ -f "$OUT" ] || { log "V2 の出力が無い（学習が落ちた）"; exit 1; }
log "V2 終わり $OUT"
[ "${SKIP_DAGGER:-0}" = 1 ] && exit 0
cd "$B" || exit 1
log "V3 DAgger を始める"
ENC="$OUT" bash "$B/exp_vis_1104.sh" "${DAGGER_NAME:-d1}"
log "V3 終わり"
