#!/usr/bin/env bash
# 純正に似せた脚（10-30 ユーザー「今の方策を純正に似せた脚で評価と、再学習させて」）。パッチ 0085。
#
# 実機の脚を HOMIE から純正（Unitree の歩行）に替えるため、シミュレータの脚の受け方を純正に寄せる:
#   切り替え: 歩くと立つの切り替えの線を 0.5 → 0.1（観測の尺度。実際の前後 0.05 m/s・旋回 0.2 rad/s）。純正は小さな指令でも歩く
#   遅れ: 脚に届く歩きの指令を回ごとに 0〜15 歩（0〜0.3 秒）遅らせる。実機の純正は 前後 0.2・横 0.1・旋回 0.0 秒（10-29 の記録）
#   倍率: 脚に届く歩きの指令に回ごと・軸ごとに 0.8〜1.2 倍（指令どおりに進む割合の違い）
#   隠す: 生徒の観測から脚と腰（HOMIE が動かす 15 関節）の角度・速さ・行動を 0 に（学習し直すときだけ）
# 段 A: 今の方策（CAPS の CK）を 3 つの条件で 256 回ずつ評価する（隠すのは学習で慣らしていないので入れない）
#   A1 切り替えだけ・A2 遅れ 0.2 秒だけ・A3 全部（学習し直しと同じ脚）。比べる元は同じ重みの 17:58 の評価 84.0%
# 段 B: 同じ重みから、A3＋隠すで学習し直す（段の名前 f3sn4096。F3' と CAPS の引数はそのまま）。評価は重みの隣の設定を読むので、
#   純正に似せた脚のまま測る。止め札（~/work_inoue/STOP_DOOR_F3）を置くまで続ける
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_stocklegs_1030.sh >> logs_stocklegs_1030.log 2>&1 < /dev/null &
#   段 A を飛ばす: SKIP_A=1
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
export NUM_ENVS="${NUM_ENVS:-4096}" MEM_MAX="${MEM_MAX:-12G}"
# 自分の札（gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3" NEED_MARGIN_MB="${NEED_MARGIN_MB:-1024}"
CK="${CK:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3ln4096/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261030_173052/last.pt}"
TASK=door_en1154_g13_shaped_left_weight
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
RESULTS="$HOME/work_inoue/doorman/stocklegs_eval_1030.tsv"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
[ -f "$CK" ] || { log "出発点の重みが無い: $CK"; exit 1; }
grep -qF "_init_stock_legs" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/base_task/homie_base.py" ||
  { log "パッチ 0085 が VIRAL6_fix に当たっていない"; exit 1; }

# F3' と CAPS の引数（exp_f3_caps_1030.sh と同じ）
CAP="++algo.config.max_noise_std=0.26"
WALK="++env.config.walk_away_start_m=1.0,++rewards.reward_scales.walk_away_in_swing=-40.0"
LONG="++env.config.max_stage_time=[250|100|100|100|600|200]"
NOPULL="++env.config.target_root_distance_swing_scale=0.0"
F3K_MORE="$CAP,$WALK,$LONG,$NOPULL,++rewards.reward_scales.penalty_arm_action_change=-0.2,++rewards.reward_scales.penalty_walk_in_hold=-0.3"
CAPS_MORE="$F3K_MORE,++algo.config.caps_temporal_coef=1.0,++algo.config.caps_temporal_start=3,++algo.config.caps_temporal_end=17"
# 純正に似せた脚（構える・つかむ段階の歩く罰も、同じ切り替えの線で数える）
STOCK_MORE="++algo.config.homie_switch_threshold=0.1,++env.config.walk_in_hold_threshold=0.1,++env.config.loco_cmd_delay_steps_range=[0|15],++env.config.loco_cmd_gain_range=[0.8|1.2]"
MASK_MORE="++env.config.actor_obs_mask_lower_body=True"

EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
# 引数: 条件の名前・足す hydra の引数（空白区切り）
eval_cond() {
  local name="$1" more="$2" evlog rec rate succ
  evlog="$HOME/work_inoue/doorman/logs_stocklegs_eval_${name}.log"
  log "段 A 評価 ${name} ck=$CK more=$more"
  # 評価は 256 台なので 9 GB（10-26〜29 の評価と同じ）。再起動の後は空きが 12.5 GB で、12 GB＋余裕では始まらない
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" MAX_WAIT_MIN="${MAX_WAIT_MIN:-240}" MEM_MAX="${EVAL_MEM_MAX:-9G}" \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $more" ./eval_from_start.sh "$CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  rate=$("$PY" -c "import json,sys; print(json.load(open(sys.argv[1]))['success_rate'])" "$rec/summary.json" 2>/dev/null || echo none)
  succ=$("$PY" -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"{d['success']}/{d['episodes_from_stage0']}\")" "$rec/summary.json" 2>/dev/null || echo none)
  printf '%s\t%s\t%s\t%s\t%s\n' "$name" "$succ" "$rate" "$rec" "$more" >> "$RESULTS"
  log "段 A 評価 ${name} success=$succ rate=$rate record=$rec"
}

rm -f "$STOP_FILE"
log "純正に似せた脚 start ck=$CK n=$NUM_ENVS stock=$STOCK_MORE mask=$MASK_MORE"
if [ "${SKIP_A:-0}" != 1 ]; then
  [ -f "$RESULTS" ] || printf 'cond\tsuccess\trate\trecord\tmore\n' > "$RESULTS"
  eval_cond A1_switch "++algo.config.homie_switch_threshold=0.1"
  eval_cond A2_delay "++env.config.loco_cmd_delay_steps_range=[10,10]"
  eval_cond A3_all "$(echo "$STOCK_MORE" | tr ',' ' ' | tr '|' ',')"
  if [ -e "$STOP_FILE" ]; then log "止め札があるので、段 B を始めずに終わる"; exit 0; fi
fi

log "段 B: f3sn4096 を学習し直す ck=$CK"
export NAME=f3sn4096 TRAIN_MORE="$CAPS_MORE,$STOCK_MORE,$MASK_MORE"
CK="$CK" ROUNDS="${ROUNDS:-9999}" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
log "段 B が終わった"
