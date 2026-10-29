#!/usr/bin/env bash
# 段 F（10-29 ユーザー「段 E の 8 回目をベースとして、段 F と、強化学習による改善をストップというまで継続して」）。
# 計画書 docs/plan/2026-10-26-g1-door-stand-foot-load-check.md の「G1 腕」「G2 脚」。
#
# 出発点: 段 E の 8 回目（反復 54,950・88.7%、BEST 既定 ckpt/f3fn4096_it54950_887）。報酬・扉・持ち時間は段 E と同じ
#   （出発点の重みが学んだ条件を変えない。段 E の始めは報酬を変えた直後に 3 回落ち込んだ）。どちらの段にも足すもの:
#   G2 脚の掛け金（パッチ 0082）: 胴と把手の水平距離 0.3 m 未満かつ扉 10° 未満で立つ HOMIE に固定、扉 10° 以上か 0.5 m で放す。
#     その回で扉が一度開いたら、もう止めない（10-29 に記録へ当てはめて決めた）
#   F1（f3g）: ＋G1 腕の出力の変化への罰（パッチ 0081）の重み ARM_W1（既定 −0.2）
#   F2（f3h）: ＋G1 の重み ARM_W2（既定 −0.5）
# F1・F2 は MIN_ROUNDS 回（既定 3）ずつ。その後、平均の成功率が高い方を、その最後の重みから ROUNDS_CONT 回（既定 9999＝止めるまで）
# 続ける（同じなら軽い F1）。終わったら f3_keepalive_1026.sh が続きを始め直す。
# 比べる相手: 同じ出発点から段 E のまま続けた 3 回（段 E の 9〜11 回目: 74.6・80.1・80.9%、平均 78.5%）。
# 回は段の名前ごとに chain/*/results.tsv から数えるので、止めて組み直しても済んだ回は回さない（exp_f3_phases_1028.sh と同じ）。
#
# 台数: NUM_ENVS（既定 4,096）・MEM_MAX（既定 12G）。4,096 台は PyTorch の断片化で CUDA のメモリ不足が出たので（10-28 23 時）、
#   PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True を既定で付ける
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く（今の回の評価まで済ませて止まり、次の段も見張りも始めない）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_f3_phaseF_1029.sh >> logs_f3_phaseF_1029.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
export NUM_ENVS="${NUM_ENVS:-4096}" MEM_MAX="${MEM_MAX:-12G}"
BEST="${BEST:-$HOME/work_inoue/doorman/ckpt/f3fn4096_it54950_887/last.pt}"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
MIN_ROUNDS="${MIN_ROUNDS:-3}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# 段 E と同じ条件（exp_f3_phases_1028.sh の段 E）
CAP="++algo.config.max_noise_std=0.26"
WALK="++env.config.walk_away_start_m=1.0,++rewards.reward_scales.walk_away_in_swing=-40.0"
LONG="++env.config.max_stage_time=[250|100|100|100|600|200]"
NOPULL="++env.config.target_root_distance_swing_scale=0.0"
# G2 脚の掛け金（パッチ 0082）
LATCH="++env.config.stand_latch=True,++env.config.stand_latch_enter_m=0.3,++env.config.stand_latch_exit_m=0.5"
LATCH="$LATCH,++env.config.stand_latch_door_deg=10.0,++env.config.stand_latch_once_opened=True"
BASE_MORE="$CAP,$WALK,$LONG,$NOPULL,$LATCH"
SFX=""; [ "$NUM_ENVS" != 512 ] && SFX="n${NUM_ENVS}"
# 名前;足す引数（「,」区切り）
PHASES=(
  "f3g${SFX};$BASE_MORE,++rewards.reward_scales.penalty_arm_action_change=${ARM_W1:--0.2}"
  "f3h${SFX};$BASE_MORE,++rewards.reward_scales.penalty_arm_action_change=${ARM_W2:--0.5}"
)

# 段の全部の回の成功率（%、1 行 1 回）。results.tsv の 1 列目が段の名前の行を集める
phase_rates() { cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { printf "%.4f\n", $6 * 100 }'; }
phase_n() { phase_rates "$1" | wc -l | tr -d ' '; }
phase_mean() { phase_rates "$1" | awk '{ t += $1; n++ } END { if (n) printf "%.1f", t / n; else print "0" }'; }
# 段のいちばん新しい回の重み（反復がいちばん大きい回。まだ回が無ければ出発点の重み）
phase_ck() {
  local ck
  ck=$(cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { print $4 "\t" $7 }' | sort -n | tail -1 | cut -f2)
  echo "${ck:-$BEST}"
}
# 引数: 名前・回数・出発点・足す引数。止め札があれば、次の段も見張りも始めずに終わる
run_rounds() {
  log "段 $1 を $2 回回す ck=$3 n=${NUM_ENVS} mem_max=${MEM_MAX} more=$4"
  export NAME="$1" TRAIN_MORE="$4"
  CK="$3" ROUNDS="$2" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので、次の段も見張りも始めずに終わる"; exit 0; fi
}

[ -f "$BEST" ] || { log "出発点の重みが無い: $BEST"; exit 1; }
log "段 F start best=$BEST n=$NUM_ENVS min_rounds=$MIN_ROUNDS alloc=$PYTORCH_CUDA_ALLOC_CONF"
for P in "${PHASES[@]}"; do
  IFS=';' read -r PNAME PMORE <<< "$P"
  N=$(phase_n "$PNAME")
  [ "$N" -lt "$MIN_ROUNDS" ] && run_rounds "$PNAME" $(( MIN_ROUNDS - N )) "$(phase_ck "$PNAME")" "$PMORE"
done
IFS=';' read -r N1 M1 <<< "${PHASES[0]}"
IFS=';' read -r N2 M2 <<< "${PHASES[1]}"
A1=$(phase_mean "$N1"); A2=$(phase_mean "$N2")
if awk -v a="$A2" -v b="$A1" 'BEGIN { exit !(a > b) }'; then CN="$N2"; CM="$M2"; else CN="$N1"; CM="$M1"; fi
log "比べた: ${N1} $(phase_n "$N1") 回の平均 ${A1}%・${N2} $(phase_n "$N2") 回の平均 ${A2}% → 続ける段: ${CN}"
run_rounds "$CN" "${ROUNDS_CONT:-9999}" "$(phase_ck "$CN")" "$CM"
[ "${NO_KEEPALIVE:-0}" = 1 ] && { log "NO_KEEPALIVE=1 なので見張りを始めずに終わる"; exit 0; }
log "続きは見張り（f3_keepalive_1026.sh）に任せる（name=${NAME:-} more=${TRAIN_MORE:-}）"
exec ./f3_keepalive_1026.sh
