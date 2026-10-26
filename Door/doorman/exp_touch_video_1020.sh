#!/usr/bin/env bash
# 段 5（10-20）: レバーにやさしく触れる — 条件ごとの成功と失敗を動画で見る（作業ログ用）。
# 条件: 元の方策（45,150）・① 3 回目・② 3 回目・①＋② 3 回目・A 2 回目（② を続けた）・B 1 回目（速さの罰は構えた後だけ）
# どれも 8 台を描画つきで評価する（外から固定のカメラ、960×720）。力は物理 1 回ごとの最大を記録（lever_force_substeps）。
# 8 台に成功と失敗の両方が無ければ、種を変えてもう 8 台撮る（最大 SEEDS 個）。
# 出力: records/start_<日時>/video/env###_<ok|stageN>.mp4・states_*.npz・summary.json。どの記録がどの条件かは touch_logs/video_1020.tsv
# 使い方（OMEN の ~/work_inoue/doorman で）: ./exp_touch_video_1020.sh [条件...]（省くと全部）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
TASK=door_en1154_g13_shaped_left_weight
EVAL_BASE="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.record_states.pregrasp_diag=True ++env.config.lever_force_substeps=True ++env.config.lever_lagrange.update=False"
declare -A CKS=(
  [base]=ckpt/s25rl_best_it45150_ovr6/last.pt
  [cap]=ckpt/s5cap_r3_it46600/last.pt
  [lag]=ckpt/s5lag_r3_it46500/last.pt
  [both]=ckpt/s5both_r3_it46500/last.pt
  [lagA]=ckpt/s5lagA_r2_it47450/last.pt
  [lagB]=ckpt/s5lagB_r1_it46950/last.pt
)
ORDER="base cap lag both lagA lagB"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# ① と ①＋② の 3 回目の重みを、上書きされない場所へ控える
keep() {
  local name="$1" chain="$2" round="$3" dest="$4"
  [ -f "$dest/last.pt" ] && return 0
  local ck
  ck=$(awk -F'\t' -v r="$round" 'NR>1 && $3==r {print $7}' "$chain/results.tsv")
  mkdir -p "$dest" && cp "$ck" "$(dirname "$ck")"/{config,meta}.yaml "$dest/" && log "控えた $name <- $ck"
}
keep cap chain/1019_1819 3 ckpt/s5cap_r3_it46600
keep both chain/1019_2143 3 ckpt/s5both_r3_it46500

mkdir -p touch_logs
# CLOSE=1: 撮った回と同じ種で、レバーの横に置いたカメラだけ撮り直す（評価は毎回同じに再現する。後ろのカメラではレバーが体の陰になる）
if [ "${CLOSE:-0}" = 1 ]; then
  for C in ${*:-$ORDER}; do
    CK="$(readlink -f "${CKS[$C]}")"
    for S in $(awk -F'\t' -v c="$C" '$1==c {print $2}' touch_logs/video_1020.tsv); do
      evlog="touch_logs/video_close_${C}_seed$S.log"
      VIDEO=1 RENDER_W=640 RENDER_H=480 RENDER_EYE="[-0.2,-0.65,1.15]" RENDER_TARGET="[-0.12,0.3,1.0]" IDLE_GATE="${IDLE_GATE:-1}" \
        COUNT_ARGS="--criterion through" MINUTES=30 EXTRA_ARGS="$EVAL_BASE ++seed=$S" ./eval_from_start.sh "$CK" "${NUM_ENVS:-8}" > "$evlog" 2>&1
      out=$(grep -ao 'eval_from_start out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
      log "[$C 横 seed=$S] $(grep -ao 'success=[^ ]* = [0-9.]*%' "$evlog" | tail -1) out=$out"
      [ -n "$out" ] && printf '%s\t%s\t%s\n' "$C" "$S" "$out" >> touch_logs/video_close_1020.tsv
    done
  done
  log "exp_touch_video_1020 done"
  exit 0
fi
for C in ${*:-$ORDER}; do
  CK="$(readlink -f "${CKS[$C]}")"
  [ -f "$CK" ] || { log "[$C] 重みが無い: ${CKS[$C]}"; continue; }
  for S in ${SEEDS:-0 1 2}; do
    evlog="touch_logs/video_${C}_seed$S.log"
    VIDEO=1 RENDER_W=960 RENDER_H=720 IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=30 \
      EXTRA_ARGS="$EVAL_BASE ++seed=$S" ./eval_from_start.sh "$CK" "${NUM_ENVS:-8}" > "$evlog" 2>&1
    out=$(grep -ao 'eval_from_start out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
    vids=$(grep -ao 'videos:.*' "$evlog" | tail -1)
    log "[$C seed=$S] $(grep -ao 'success=[^ ]* = [0-9.]*%' "$evlog" | tail -1) out=$out $vids"
    [ -n "$out" ] && printf '%s\t%s\t%s\n' "$C" "$S" "$out" >> touch_logs/video_1020.tsv
    # 成功（_ok）と失敗（_stage）の両方が撮れたら次の条件へ
    if echo "$vids" | grep -q "_ok" && echo "$vids" | grep -q "_stage"; then break; fi
  done
done
log "exp_touch_video_1020 done"
