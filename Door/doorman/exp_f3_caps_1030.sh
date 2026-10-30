#!/usr/bin/env bash
# 段 CAPS（10-30 ユーザー「b かな」）。計画書 docs/plan/2026-10-26-g1-door-stand-foot-load-check.md の「CAPS」。
#
# 出発点: F3'（f3kn4096、腕の罰 −0.2＋歩く罰 −0.3）のいちばん新しい回の重み。引数は F3' と同じで、CAPS の時間の項（パッチ 0084）だけを足す:
#   ++algo.config.caps_temporal_coef=CAPS_W（腕 14＝出力の 3〜16 番の、同じ回の隣り合う時刻の出力の平均の差の 2 乗和を損失に足す）
# NAME（既定 f3l）を MIN_ROUNDS 回（既定 3）回し、平均の成功率を F3' の最後の REF_N 回（既定 9）の平均と比べる。
#   落ちが DROP（既定 5.0）pt より小さければ CAPS を、そうでなければ F3' を、それぞれの最後の重みから ROUNDS_CONT 回（既定 9999＝止めるまで）
#   続ける。手先の震えが減ったかは別に測る（jitter3_all.py）
# 回は段の名前ごとに chain/*/results.tsv から数えるので、止めて組み直しても済んだ回は回らない（exp_f3_phaseF_1029.sh と同じ）。
# 台数: NUM_ENVS（既定 4,096）・MEM_MAX（既定 12G）・PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True（段 F と同じ）
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く（今の回の評価まで済ませて止まり、次の段も見張りも始めない）
# 使い方（OMEN の ~/work_inoue/doorman で）: CAPS_W=<重み> setsid nohup ./exp_f3_caps_1030.sh >> logs_f3_caps_1030.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
export NUM_ENVS="${NUM_ENVS:-4096}" MEM_MAX="${MEM_MAX:-12G}"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
MIN_ROUNDS="${MIN_ROUNDS:-3}"
REF_N="${REF_N:-9}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
[ -n "${CAPS_W:-}" ] || { log "CAPS_W（CAPS の時間の項の重み）を渡す"; exit 1; }

# F3' と同じ条件（exp_f3_phaseF_1029.sh の 3 本目 f3k: 段 E の引数＋腕の罰 −0.2＋歩く罰 −0.3）
CAP="++algo.config.max_noise_std=0.26"
WALK="++env.config.walk_away_start_m=1.0,++rewards.reward_scales.walk_away_in_swing=-40.0"
LONG="++env.config.max_stage_time=[250|100|100|100|600|200]"
NOPULL="++env.config.target_root_distance_swing_scale=0.0"
F3K_MORE="$CAP,$WALK,$LONG,$NOPULL,++rewards.reward_scales.penalty_arm_action_change=-0.2,++rewards.reward_scales.penalty_walk_in_hold=-0.3"
CAPS_MORE="$F3K_MORE,++algo.config.caps_temporal_coef=${CAPS_W},++algo.config.caps_temporal_start=3,++algo.config.caps_temporal_end=17"
SFX=""; [ "$NUM_ENVS" != 512 ] && SFX="n${NUM_ENVS}"
REF="f3k${SFX}"
NAME_C="${NAME:-f3l}${SFX}"

phase_rates() { cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { printf "%.4f\n", $6 * 100 }'; }
phase_n() { phase_rates "$1" | wc -l | tr -d ' '; }
phase_mean() { phase_rates "$1" | awk '{ t += $1; n++ } END { if (n) printf "%.1f", t / n; else print "0" }'; }
# 段の最後の n 回（反復の順）の平均の成功率（%）
phase_last_mean() {
  cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { printf "%s\t%.4f\n", $4, $6 * 100 }' |
    sort -n | tail -"$2" | awk -F'\t' '{ t += $2; n++ } END { if (n) printf "%.1f", t / n; else print "0" }'
}
# 段のいちばん新しい回の重み（反復がいちばん大きい回。まだ回が無ければ F3' のいちばん新しい回の重み）
phase_ck() {
  local ck
  ck=$(cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { print $4 "\t" $7 }' | sort -n | tail -1 | cut -f2)
  [ -n "$ck" ] || ck=$(cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$REF" '$1 == s { print $4 "\t" $7 }' | sort -n | tail -1 | cut -f2)
  echo "$ck"
}
# 引数: 名前・回数・出発点・足す引数。止め札があれば、次の段も見張りも始めずに終わる
run_rounds() {
  log "段 $1 を $2 回回す ck=$3 n=${NUM_ENVS} mem_max=${MEM_MAX} more=$4"
  export NAME="$1" TRAIN_MORE="$4"
  CK="$3" ROUNDS="$2" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので、次の段も見張りも始めずに終わる"; exit 0; fi
}

START=$(phase_ck "$NAME_C")
[ -f "$START" ] || { log "出発点の重みが無い: $START"; exit 1; }
log "段 CAPS start caps_w=$CAPS_W name=$NAME_C ref=$REF ck=$START n=$NUM_ENVS min_rounds=$MIN_ROUNDS alloc=$PYTORCH_CUDA_ALLOC_CONF"
N=$(phase_n "$NAME_C")
[ "$N" -lt "$MIN_ROUNDS" ] && run_rounds "$NAME_C" $(( MIN_ROUNDS - N )) "$START" "$CAPS_MORE"
A=$(phase_mean "$NAME_C"); AR=$(phase_last_mean "$REF" "$REF_N")
if awk -v a="$A" -v b="$AR" -v t="${DROP:-5.0}" 'BEGIN { exit !(a > b - t) }'; then
  log "比べた: ${NAME_C}（CAPS ${CAPS_W}）$(phase_n "$NAME_C") 回の平均 ${A}% 対 ${REF} の最後の ${REF_N} 回の ${AR}% → 落ちが ${DROP:-5.0} pt より小さいので 続ける段: ${NAME_C}"
  CN="$NAME_C"; CM="$CAPS_MORE"
else
  log "比べた: ${NAME_C}（CAPS ${CAPS_W}）$(phase_n "$NAME_C") 回の平均 ${A}% 対 ${REF} の最後の ${REF_N} 回の ${AR}% → ${DROP:-5.0} pt 以上落ちたので 続ける段: ${REF}"
  CN="$REF"; CM="$F3K_MORE"
fi
run_rounds "$CN" "${ROUNDS_CONT:-9999}" "$(phase_ck "$CN")" "$CM"
[ "${NO_KEEPALIVE:-0}" = 1 ] && { log "NO_KEEPALIVE=1 なので見張りを始めずに終わる"; exit 0; }
log "続きは見張り（f3_keepalive_1026.sh）に任せる（name=${NAME:-} more=${TRAIN_MORE:-}）"
exec ./f3_keepalive_1026.sh
