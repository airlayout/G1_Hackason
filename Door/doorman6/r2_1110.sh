#!/usr/bin/env bash
# 11-10 手順 R2（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 8.8 節）: s2f を一時停止して、画像の網（カラー＋深度 → 扉の値 17）を
# シムの画（ds_dep の 12 束・8.4 万枚）に実物の画（11-06 18:33 の 7,341 枚・正解はライダー）を混ぜて学び直す（教師あり・パッチ 0103）。
# 確かめは 11-06 18:59 の 4,750 枚（学習に使わない）。終わったら（失敗しても）一時停止の札を外して s2f を再開する。
#   1. 比べる元: 網は pretrain_rgbd_1106_1117.pt・頭は実機の e2b（init_e2b.pt）を、同じ束で測るだけ（--eval_only）
#   2. 学び直し: init_e2b.pt から EPOCHS エポック・学習率 LR・実物の割合 FRAC
# RAM の空きが RAM_MIN_MB を切ったら学び直しを止める（共用機・s2f の再開の分を残す）
# 使い方（OMEN）: bash ~/work_inoue/doorman6/r2_1110.sh   [EPOCHS=4 LR=1e-4 FRAC=0.25 TAG=r2_real25 SKIP_BASE=1]
set -uo pipefail
PY="$HOME/miniforge3/envs/doorman6/bin/python"
V="$HOME/work_inoue/doorman6/VIRAL6_vis"
O="$HOME/work_inoue/truth_1110"
DS="$HOME/work_inoue/doorman6/vis_1104/ds_dep"
EPOCHS="${EPOCHS:-4}"; LR="${LR:-1e-4}"; FRAC="${FRAC:-0.25}"; TAG="${TAG:-r2_real25}"; RAM_MIN_MB="${RAM_MIN_MB:-1500}"
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] r2: $*"; }

cd "$HOME/work_inoue/doorman" || exit 1
log "s2f を一時停止する（$PAUSE_FILE）"
if ! ./pause_training.sh on; then
  log "学習が 3 分で止まらない。札を外して終わる"; ./pause_training.sh off; exit 1
fi
WATCH=""
finish() {
  [ -n "$WATCH" ] && kill "$WATCH" 2>/dev/null
  cd "$HOME/work_inoue/doorman" && ./pause_training.sh off
  log "s2f を再開した（札を外した）"
}
trap finish EXIT

( while sleep 5; do
    a=$(free -m | awk '/^Mem:/{print $7}')
    if [ "${a:-0}" -lt "$RAM_MIN_MB" ]; then
      echo "=== [$(date '+%m-%d %T')] r2: RAM の空き ${a} MB < ${RAM_MIN_MB} MB で学び直しを止める"
      pkill -f "[d]oor_vision_pretrain.py"
    fi
  done ) &
WATCH=$!

DATA="$(ls -d "$DS"/s*/frames | tr '\n' ' ')"  # 元の学び直し（pipeline_dep_1106.sh）と同じ並び＝同じ確かめの扉
cd "$V" || exit 1
if [ "${SKIP_BASE:-0}" != 1 ]; then
  log "比べる元を測る（init_e2b.pt・--eval_only）"
  # shellcheck disable=SC2086
  PYTHONPATH=. PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py --data $DATA --init "$O/init_e2b.pt" --eval_only \
    --real_val "$O/real_185929.npz" "$O/real_183336.npz" --out "$O/base_e2b_full.pt" || log "⚠️ 比べる元の測りが失敗した"
fi
log "学び直し（${EPOCHS} エポック・学習率 ${LR}・実物の割合 ${FRAC}）→ $O/${TAG}.pt"
# shellcheck disable=SC2086
PYTHONPATH=. PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py --data $DATA --init "$O/init_e2b.pt" \
  --epochs "$EPOCHS" --lr "$LR" --real_data "$O/real_183336.npz" --real_frac "$FRAC" \
  --real_val "$O/real_185929.npz" "$O/real_183336.npz" --out "$O/${TAG}.pt"
rc=$?
log "学び直し 終わり（rc=$rc）"
exit $rc
