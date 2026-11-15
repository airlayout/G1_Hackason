#!/usr/bin/env bash
# 11-12 夜（計画 docs/plan/2026-11-12-g1-door-script-shaped-rl.md の S3・ユーザー「sim でできるところまで進めて」「判断を委譲する」）:
#   s3a（台本の形の腕だけの段・扉 3° 96.5%）から教師 f3t3 への引き継ぎは 3°・6°・10° で 52・52・62.5%（s3_eval_1112.tsv）で線 80% に届かない。
#   → 引き継いだ後を受け持つ教師を学び直す（強化学習 PPO・段の名前 f3t4）。
#   前座: 凍らせた s3a の最良（ckpt/start_1112_1603）が扉 10° まで動かす（パッチ 0109 prelude_policy_path・学習でも評価でも同じ）
#   学ぶ方策: f3t3 の重みから。扉 10° で受け取り、通り抜け（扉の先 1.5 m）まで。記憶は回の始めから影で回る
#   環境は s3a と同じ（台本の構え・手の先で押す形・palm_press・体の回りの外乱・実物の壁・純正の応え方の層）。すき間の罰は入れない（前座は凍っている）
#   完了は通り抜け（arm_stage.complete_through）・回の長さ 45 s（f3t3 と同じ）・数え方 armthrough
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash s3b_1112.sh >> logs_s3b_1112.log 2>&1 < /dev/null &
#   続きから: CK=<f3t4 の last.pt>。止め札 ~/work_inoue/STOP_DOOR_F3 で区切りの境目に止まる
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] s3b: $*"; }
CK="${CK:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt}"
PRE="${PRE:-$HOME/work_inoue/doorman/ckpt/start_1112_1603/last.pt}"
HDEG="${HDEG:-10.0}"
FIT="$HOME/work_inoue/doorman/fit_loco_resp_1107.txt"
TR="$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/trl/trainer/ppo_trainer_homie_api.py"
export TASK=door_en1154_g13_shaped_left_weight_realwall
[ -f "$CK" ] && [ -f "$PRE" ] || { log "重みが無い: $CK / $PRE"; exit 1; }
grep -qF "_load_prelude_policy" "$TR" || { log "パッチ 0109 が VIRAL6_fix に当たっていない"; exit 1; }
grep -qF "\"armthrough\"" count_success.py || { log "count_success.py に armthrough が無い"; exit 1; }
STOP="$HOME/work_inoue/STOP_DOOR_F3"
if [ -e "$STOP" ]; then rm -f "$STOP"; log "前の止め札を外した"; fi
T1="++env.config.shoulder_roll_press_max_nm=6.0"
PEN="++rewards.reward_scales.penalty_shoulder_roll_press=-0.01"
RESP_CHAIN=$(grep -A1 "学習の連鎖の引数" "$FIT" | tail -1)
case "$RESP_CHAIN" in *loco_resp.enabled=true*) ;; *) log "応え方の層の値が無い: $FIT"; exit 1 ;; esac
SWITCH="++algo.config.homie_switch_threshold=0.1,++env.config.walk_in_hold_threshold=0.1"
ARM="++env.config.arm_stage.enabled=True,++env.config.arm_stage.start_range=[-0.42|-0.33|-0.40|-0.25|-12.0|12.0]"
ARM="$ARM,++env.config.staged_reset_ratios=[1.0|0.0|0.0|0.0|0.0|0.0]"
ARM="$ARM,++env.config.arm_stage.complete_door_deg=null,++env.config.arm_stage.complete_through=True"
ARM="$ARM,++env.config.arm_stage.free_legs_when_unlatched=True,++env.config.arm_stage.release=palm_press"
ARM="$ARM,++env.config.arm_stage.handover_door_deg=$HDEG,++algo.config.prelude_policy_path=$PRE"
ARM="$ARM,++env.config.door_sensor.loc_pos_bias_m=0.03,++env.config.door_sensor.loc_yaw_bias_deg=1.0"
YD="++env.config.yaw_disturb.enabled=True,++env.config.yaw_disturb.amp_deg=[0.0|15.0],++env.config.yaw_disturb.start_s=[1.0|2.5],++env.config.yaw_disturb.dur_s=[0.3|0.7],++env.config.yaw_disturb.sign=lever_side,++env.config.yaw_disturb.prob=0.5"
SHAPE="++env.config.grasp_target_offset=[-0.04|0.0|-0.01],++env.config.grasp_target_offset_along_lever=0.01,++env.config.pregrasp_offset=[0.0|0.0|0.12]"
STAGE="++env.config.max_stage_time=[500|200|200|200|600|400],++env.config.max_episode_length_s=45"
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log "学び直し f3t4 を始める ck=$CK 前座=$PRE 引き継ぐ角度=$HDEG"
SKIP_A=1 MASK=0 COMPLETE_THROUGH=1 TERM_UP=0.9 B_NAME="${B_NAME:-f3t4}" AUTO_ENVS=1 IDLE_GATE="${IDLE_GATE:-1}" CK="$CK" CRIT=armthrough \
  STOCK_MORE="$SWITCH,$RESP_CHAIN" LONG_MORE="$STAGE" EXTRA_MORE="$T1,$PEN,$ARM,$YD,$SHAPE,++env.config.reset_from_dataset.enabled=False" ./exp_stocklegs_1030.sh
log "終わり"
