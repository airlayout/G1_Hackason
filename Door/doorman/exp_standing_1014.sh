#!/usr/bin/env bash
# ③ の続き（2026-10-14）: 評価で立った姿勢から始めると成功がどれだけ変わるか（パッチ 0042、++env.config.reset_from_dataset.enabled=False）。
# 既定の評価は人の動きのデータ（LAFAN-G1）の姿勢から始まり、128 台中 5 台は傾いた姿勢（24〜68°）から倒れて 1 秒以内に打ち切られる。
# 実機は立った姿勢から始まるので、その条件で測る。exp_s24L_1014.sh（①②④の実験）が終わってから回す（GPU は 1 枚）。
# 比べる相手: 反復 39,150 の把手が右の扉 102/128（79.7%、データの姿勢から）・把手が左＋鏡写し（exp_s24L の 0）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_standing_1014.sh > exp_standing_1014.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/s24a_r4_it39150/last.pt"
RULES="++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90"
STAND="++env.config.reset_from_dataset.enabled=False"
export IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" COUNT_ARGS="--criterion through"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "exp_s24L_1014.sh の終わりを待つ"
until grep -q "exp_s24L_done" exp_s24L_1014.log 2>/dev/null; do sleep 60; done
sleep 70
for SPEC in "right:door_en1154_g13_shaped:" "left_mirror:door_en1154_g13_shaped_left:++env.config.mirror_left_doors=True"; do
  NAME="${SPEC%%:*}"; REST="${SPEC#*:}"; TASK="${REST%%:*}"; EXTRA="${REST#*:}"
  log "立った姿勢から・$NAME task=$TASK"
  EXTRA_ARGS="++env.config.task.name=$TASK $RULES $STAND $EXTRA" ./eval_from_start.sh "$CK" 256 > "eval_standing_${NAME}.log" 2>&1
  grep -a "eval_from_start out=\|区間 s 以上\|end eval_record" "eval_standing_${NAME}.log" | sed "s/^/  [$NAME] /"
  sleep 61
done
log "exp_standing_done"
