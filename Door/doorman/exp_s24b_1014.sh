#!/usr/bin/env bash
# 段 2-4b（2026-10-14 のユーザー判断「2-4b を実施して」）: 実物のレバー（約 1.0 m・バックセット 64 mm・長さ 約 142 mm・四角）を
# 学習の範囲に入れる。2-4a の扉は 0.85〜0.95 m・80〜150 mm・110〜140 mm・丸で、実物が外れていた。ばらつきは 1 つずつ足す（10.7 の教訓）:
#   b1. レバーの位置（高さ 0.85〜1.10 m・バックセット 30〜100 mm）door_en1154_g13_shaped_left_lpos。教師は高さとバックセットを数値で見ている
#   b2. さらにレバーの形（長さ 100〜160 mm・断面 丸／四角）door_en1154_g13_shaped_left_lever。長さと断面は観測に無く、触って分かるだけ
# 出発は段 2-4a の合格（反復 40,600）。学習の引数は 2-4a の B と同じ（押す力 50 N・通り抜け 70°・90° で失敗・鏡写し・窓の報酬 −3.0）。
# 各段: まず学習なしで評価（EVAL_FIRST=1＝回 0）。80% 以上なら学習を飛ばす。届かなければ 30 分学習 → 評価 を最大 4 回。
# 合格したら、その重みから次の段へ（stage_chain.sh）。評価は 256 回・立った姿勢から・時間 2 倍（段 2-4a の成功値と同じ条件）。
# 最後に、最後に合格した重みを 2-4a の扉でも評価する（範囲を広げたせいで元の扉が落ちていないか）。
# 在席の判定は既定のまま（IDLE_GATE=1。画面の前に人がいる間は待つ）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_s24b_1014.sh > exp_s24b_1014.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/s24a_pass_it40600/last.pt"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-3.0"
TRAIN="$CAP3,$RULES,$MIRROR,$WINDOW"
export EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export MINUTES=30 MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=0.8 EVAL_N=256 EVAL_FIRST=1
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "exp_s24b start ckpt=$CK"
./stage_chain.sh "$CK" \
  "s24b1:door_en1154_g13_shaped_left_lpos:4:30:through:$TRAIN" \
  "s24b2:door_en1154_g13_shaped_left_lever:4:30:through:$TRAIN"
L=$(ls -td chain/*/ | head -1)
log "chain done chain=$L"
cut -f1-6 "$L/results.tsv" | sed "s/^/  [chain] /"

KEPT=$(grep -o 'kept=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
if [ -n "$KEPT" ] && [ -f "$KEPT/last.pt" ]; then
  log "2-4a の扉で評価し直す ckpt=$KEPT/last.pt"
  COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left $EVAL_EXTRA" \
    ./eval_from_start.sh "$KEPT/last.pt" 256 > "$L/eval_s24a_door_regression.log" 2>&1
  grep -a "eval_from_start out=" "$L/eval_s24a_door_regression.log" | sed "s/^/  [2-4a の扉] /"
else
  log "合格した重みが無いので 2-4a の扉での評価は飛ばす"
fi
log "exp_s24b_done"
