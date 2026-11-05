#!/usr/bin/env bash
# 11-04（画像だけの生徒 V3→V4）: DAgger の連鎖（d1）を区切りの境目で止め、最新の重みを 2 通りに評価し、PPO（V4）の連鎖に切り替える
# （11-04 ユーザー了承「評価して PPO に切り替える」）。
#   ① 生徒だけで運転（学習に無い扉 seed 999・最初の区間から・256 台）  ② 同じ・頭の頷き 4°
#   PPO の出発の重み = 生徒＋f3sc の評価係（make_vis_ppo_ckpt.py、std は f3sc × 0.5）。PPO の 1 区切り目で落ちたら DAgger に戻す
# 使い方（OMEN の ~/work_inoue/doorman6）: ENC=<V2 の出力> setsid nohup bash switch_ppo_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
#   ROUND: 止める区切りの番号（いまの連鎖の中の番号。既定 1）
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
LOG="$V/pipeline.log"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TEACHER="$HOME/work_inoue/doorman/ckpt/start_1103_0807/last.pt"
ENC="${ENC:?}"
ROUND="${ROUND:-1}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
# 止める区切りの始まりの時刻（最後の「区切り ROUND seed=」の行）＋ 30 分の 20 秒前に止め札を置く
START_LINE=$(grep -a "区切り $ROUND seed=" "$LOG" | tail -1)
START_T=$(date -d "$(echo "$START_LINE" | sed -E 's/^=== \[11-([0-9]+) ([0-9:]+)\].*/2026-11-\1 \2/')" +%s)
STOP_AT=$(( START_T + ${ROUND_MIN:-30} * 60 - 20 ))
log "切り替えの見張り: 区切り $ROUND の始まり $(date -d @$START_T +%T)・止め札は $(date -d @$STOP_AT +%T)"
while [ "$(date +%s)" -lt "$STOP_AT" ]; do sleep 5; done
touch "$HOME/work_inoue/STOP_DOOR_VIS"
log "止め札を置いた。DAgger の連鎖の終わりを待つ"
while pgrep -f "[e]xp_vis_1104.sh d1" > /dev/null; do sleep 5; done
[ -f "$B/exp_vis_1104.sh.new" ] && mv "$B/exp_vis_1104.sh.new" "$B/exp_vis_1104.sh"
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_d1/*/*/last.pt | head -1)
LAST_SEED=$(grep -ao "区切り [0-9]* seed=[0-9]*" "$LOG" | tail -1 | grep -o "seed=[0-9]*" | cut -d= -f2)
TAG="$(date +%m%d_%H%M)"
log "評価 ck=$CK"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_student" 256 > "$V/eval_${TAG}_student.log" 2>&1
log "① 生徒: $(tail -1 "$V/eval_${TAG}_student.log")"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_nod4" 256 ++env.config.vision_feat.nod_deg=4.0 > "$V/eval_${TAG}_nod4.log" 2>&1
log "② 頷き 4°: $(tail -1 "$V/eval_${TAG}_nod4.log")"
PCK="$V/ppo_start_${TAG}.pt"
(cd "$B/VIRAL6_vis" && PYTHONPATH=. "$PY" gr00t/rl/scripts/make_vis_ppo_ckpt.py "$CK" "$TEACHER" "$PCK" --std-scale 0.5)
[ -f "$PCK" ] || { log "PPO の出発の重みが作れない"; exit 1; }
rm -f "$HOME/work_inoue/STOP_DOOR_VIS"
log "V4 PPO の連鎖を始める ck=$PCK"
EXP=wbmanip/door_open_homie_29dof_vision_f3_ppo-lstm CK="$PCK" SEED0=2000 ENC="$ENC" NUM_ENVS=256 ROUNDS="${ROUNDS:-40}" ROUND_MIN=30 \
  bash "$B/exp_vis_1104.sh" p1
if tail -5 "$LOG" | grep -aq "最初の区切りで落ちた"; then
  log "PPO の 1 区切り目で落ちた → DAgger に戻す ck=$CK seed0=$LAST_SEED"
  rm -f "$HOME/work_inoue/STOP_DOOR_VIS"
  CK="$CK" SEED0="$LAST_SEED" ENC="$ENC" NUM_ENVS=256 ROUNDS=40 ROUND_MIN=30 bash "$B/exp_vis_1104.sh" d1
fi
log "switch_ppo 終わり"
