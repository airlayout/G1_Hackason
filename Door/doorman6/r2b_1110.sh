#!/usr/bin/env bash
# 11-10 手順 R2 の続き（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 8.8 節）: s2f を 1 回だけ一時停止し、画像の網の学び直しを
# いくつかまとめて回す（パッチ 0103）。終わったら（失敗しても）一時停止の札を外して s2f を再開する。
#   c1 写り込む物の真似（--occlude 0.5）・実物なし → 18:33・18:59 のどちらも学習に使わない試験になる
#   c2 写り込む物の真似＋実物 18:33 の読める枚（地点 0 と移動中・real_183336_S_clean.npz）を 25%
#   c3 実物 18:33 の読める枚を 25%（写り込む物の真似なし）＝ c2 との差で真似の効きを分ける
# どれも網は pretrain の重み・頭は実機の e2b（init_e2b.pt）から 4 エポック・学習率 1e-4。確かめは 18:59 と 18:33 の全部の枚
# 使い方（OMEN）: bash ~/work_inoue/doorman6/r2b_1110.sh   [RUNS="c1 c2 c3" EPOCHS=4 LR=1e-4]
set -uo pipefail
PY="$HOME/miniforge3/envs/doorman6/bin/python"
V="$HOME/work_inoue/doorman6/VIRAL6_vis"
O="$HOME/work_inoue/truth_1110"
DS="$HOME/work_inoue/doorman6/vis_1104/ds_dep"
EPOCHS="${EPOCHS:-4}"; LR="${LR:-1e-4}"; RUNS="${RUNS:-c1 c2 c3}"; RAM_MIN_MB="${RAM_MIN_MB:-1500}"
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] r2b: $*"; }

run_args() {  # 実験の名前 → 学び直しの引数
  case "$1" in
    c1) echo "--occlude 0.5" ;;
    c2) echo "--occlude 0.5 --real_data $O/real_183336_S_clean.npz --real_frac 0.25" ;;
    c3) echo "--real_data $O/real_183336_S_clean.npz --real_frac 0.25" ;;
    *) return 1 ;;
  esac
}
for r in $RUNS; do run_args "$r" > /dev/null || { log "知らない実験 $r"; exit 2; }; done

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
      echo "=== [$(date '+%m-%d %T')] r2b: RAM の空き ${a} MB < ${RAM_MIN_MB} MB で学び直しを止める"
      pkill -f "[d]oor_vision_pretrain.py"
    fi
  done ) &
WATCH=$!

DATA="$(ls -d "$DS"/s*/frames | tr '\n' ' ')"  # 元の学び直しと同じ並び＝同じ確かめの扉
cd "$V" || exit 1
rc_all=0
for r in $RUNS; do
  log "$r: $(run_args "$r")（${EPOCHS} エポック・学習率 ${LR}）→ $O/r2b_$r.pt"
  # shellcheck disable=SC2046,SC2086
  PYTHONPATH=. PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py --data $DATA --init "$O/init_e2b.pt" \
    --epochs "$EPOCHS" --lr "$LR" $(run_args "$r") \
    --real_val "$O/real_185929_S.npz" "$O/real_183336_S.npz" --out "$O/r2b_$r.pt"
  rc=$?
  log "$r 終わり（rc=$rc）"
  [ "$rc" -ne 0 ] && rc_all=$rc
done
exit $rc_all
