#!/usr/bin/env bash
# 段 2-4a・実物の向き（2026-10-14 のユーザー判断「向きを直す」「①〜④をトライ」）。どれが効いたかを分けて読めるよう 1 つずつ足す:
#   0. 学習前の評価: 反復 39,150 を、把手が左の扉（door_en1154_g13_shaped_left）＋鏡写しで 256 回（向きの影響だけ）
#   A. 把手が左＋鏡写し、②（30 分ごとに 256 回で評価して最良を拾う）。報酬・学習の歩幅は段 2-4a と同じ（対照）
#   B. A ＋ ①窓の報酬（扉を開ける点は 80° で頭打ち、85° を越えると減点 −3.0）
#   C. B ＋ ④学習の 1 歩を小さく（desired_kl 0.005 → 0.0025）
# どれも反復 39,150（段 2-4a で 79.7%）から 30 分 × 4 回。合格でも止めず（PASS_RATE=1.1）最後まで回して比べる。
# ③（すぐ打ち切られる回の原因）は Mac で記録から調べる（GPU は使わない）
# OMEN の前の人（ユーザー本人）がいても回す（IDLE_GATE=0、10-13 のユーザー判断）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_s24L_1014.sh > exp_s24L_1014.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/s24a_r4_it39150/last.pt"
TASK=door_en1154_g13_shaped_left
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-3.0"
SMALL="++algo.config.desired_kl=0.0025"
export IDLE_GATE=0 MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=1.1 EVAL_N=256
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "0. 学習前の評価（把手が左・鏡写し・256 回）"
EXTRA_ARGS="++env.config.task.name=$TASK ${RULES//,/ } $MIRROR" COUNT_ARGS="--criterion through" \
  ./eval_from_start.sh "$CK" 256 > eval_s24L_r0.log 2>&1
grep -a "eval_from_start out=\|区間 s 以上\|end eval_record" eval_s24L_r0.log | sed "s/^/  [r0] /"

for SPEC in "s24LA:$CAP3,$RULES,$MIRROR" "s24LB:$CAP3,$RULES,$MIRROR,$WINDOW" "s24LC:$CAP3,$RULES,$MIRROR,$WINDOW,$SMALL"; do
  NAME="${SPEC%%:*}"; EXTRA="${SPEC#*:}"
  [ -e "$STOP_FILE" ] && { log "止め札があるので $NAME の前で止める"; break; }
  log "$NAME start extra=$EXTRA"
  ./stage_chain.sh "$CK" "$NAME:$TASK:4:30:through:$EXTRA"
  L=$(ls -td chain/*/ | head -1)
  log "$NAME done chain=$L"
  cut -f1-6 "$L/results.tsv" | sed "s/^/  [$NAME] /"
done
log "exp_s24L_done"
