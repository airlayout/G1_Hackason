#!/usr/bin/env bash
# 11-08: 生徒 e4（DAgger＝模倣学習）を night_1107.sh の 07:00 の止めの後も続ける。引数は night_1107.sh の段 3 と同じ
# （教師 f3t3 の区切り 11・実機に合わせた純正の応え方の層 v2・持ち時間・回の上限）。最新の e4 の重みから始め、
# 止め札 ~/work_inoue/STOP_DOOR_VIS_ALL が置かれるまで連鎖を起こし直す。区切りごとの重みは別々の日時の dir に残るので、
# 止めた後にいくつかを評価して最良を選ぶ（学習中に評価は挟まない）。
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash cont_e4_1108.sh >> vis_1104/cont_e4_1108.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"; D="$HOME/work_inoue/doorman"; W="$HOME/work_inoue"
FIT="$D/fit_loco_resp_1107.txt"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) echo "応え方の層の値が無い: $FIT"; exit 1 ;; esac
LONGEP=++env.config.max_episode_length_s=45
ENC="$B/vis_1104/pretrain_rgbd_1106_1117.pt"
NOD=++env.config.vision_feat.nod_deg=4.0
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"
CAMS="++simulator.config.cameras.camera_types=[{rgb:true},{depth:true}]"
COFF0="++env.config.vision_feat.color_off_prob=0.0"
T1="++env.config.shoulder_roll_press_max_nm=6.0"
TEACHER="$B/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt"
log() { echo "=== [$(date "+%m-%d %T")] cont_e4: $*"; }
newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_e4/*/*/last.pt 2>/dev/null | head -1; }
[ -f "$TEACHER" ] && [ -f "$ENC" ] || { log "教師か網の重みが無い"; exit 1; }
E1=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e1b/*/*/last.pt | head -1)
TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$E1"
TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
rm -f "$W/STOP_DOOR_VIS_ALL" "$W/STOP_DOOR_VIS"
CK=$(newest); SEED0="${SEED0:-4840}"
for attempt in $(seq 0 6); do
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  log "e4 連鎖 $attempt teacher=$TEACHER ck=$CK seed0=$SEED0"
  STAGE_TIME=500,200,200,200,600,400 TEACHER="$TEACHER" ENC="$ENC" CK="$CK" NUM_ENVS=256 ROUNDS=48 ROUND_MIN=30 SEED0="$SEED0" \
    EXTRA_ARGS="$NOD $VDELAY $CAMS $COFF0 $T1 $TRK $RESP_EVAL $LONGEP" bash "$B/exp_vis_1104.sh" e4
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  NEW=$(newest)
  if [ -z "$NEW" ] || [ "$NEW" = "$CK" ]; then log "e4 連鎖 $attempt が新しい重みを残さずに終わった。起こし直さない"; break; fi
  CK="$NEW"; SEED0=$(( SEED0 + 20 )); log "e4 連鎖 $attempt が止め札の前に終わった。最新の重みから起こし直す"
done
log "終わり e4=$(newest)"
