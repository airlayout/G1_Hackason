#!/usr/bin/env bash
# 11-07（ユーザー「区切りの境目で学習を 6 分ほど止めて評価を挟む」）: 生徒 e4（DAgger）の区切りの終わりの少し前に
# exp_vis_1104.sh を一時停止（SIGSTOP）し、学習を区切りの終わりと同じ形（SIGINT）で止めて、最新の重みを層 v2 の脚で
# 256 回評価してから、台本を再開（SIGCONT）する。台本からは区切りがふつうに終わったように見える（時刻は END を過ぎている）。
# exp_vis_1104.sh は IGNORE_PAUSE=1 で一時停止の札を見ず、止め札 STOP_DOOR_VIS は連鎖ごと終わらせるので使えない。
# 評価の引数は night_1107.sh の e4_resp と同じ。
# 使い方（OMEN の ~/work_inoue/doorman6）: AT=1613 setsid nohup bash mid_eval_1107.sh >> vis_1104/mid_eval_1107.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"; V="$B/vis_1104"; D="$HOME/work_inoue/doorman"
log() { echo "=== [$(date "+%m-%d %T")] mid_eval: $*" | tee -a "$V/night_1107.log"; }
AT="${AT:?止める時刻 HHMM}"
FIT="$D/fit_loco_resp_1107.txt"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) log "応え方の層の値が無い: $FIT"; exit 1 ;; esac
read -r -a RESP <<< "$RESP_EVAL"
SW=(++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1)
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"; NOD0=++env.config.vision_feat.nod_deg=0.0
COFF0="++env.config.vision_feat.color_off_prob=0.0"

target=$(date -d "$(date +%F) ${AT:0:2}:${AT:2:2}" +%s)
while [ "$(date +%s)" -lt "$target" ]; do sleep 5; done
EXP_PID=$(pgrep -f "^bash $B/exp_vis_1104.sh e4" | head -1)
[ -n "$EXP_PID" ] || { log "exp_vis_1104.sh e4 が動いていない"; exit 1; }
TR_PID=$(pgrep -P "$EXP_PID" -f "train_agent_trl" | head -1)
[ -n "$TR_PID" ] || { log "区切りの学習が動いていない（境目の最中）。何もしない"; exit 1; }
kill -STOP "$EXP_PID"
trap 'kill -CONT "$EXP_PID" 2>/dev/null; log "台本を再開した"' EXIT
log "台本 $EXP_PID を一時停止。学習 $TR_PID を止める"
kill -INT -- -"$TR_PID" 2>/dev/null
for _ in $(seq 30); do kill -0 "$TR_PID" 2>/dev/null || break; sleep 1; done
kill -KILL -- -"$TR_PID" 2>/dev/null
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e4/*/*/last.pt | head -1)
log "評価する重み: $CK（保存 $(date -r "$CK" +%T)）"
tag=$(date +%m%d_%H%M); lg="$V/eval_${tag}_e4mid_resp.log"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${tag}_e4mid_resp" 256 "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}" > "$lg" 2>&1
log "評価 e4mid_resp: $(tail -n 1 "$lg")"
