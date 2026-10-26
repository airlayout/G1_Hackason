#!/usr/bin/env bash
# シミュレータと実機の食い違いの直し（10-26）の V11・V12。評価だけ（学習しない）。作業ツリーは VIRAL6_fix（パッチ 0069〜0073）。
#
# V12: 今の方策（45,150）を直した条件で評価し、どれだけ落ちるかを数える（学習し直す量の見込み）。直しの組ごとに切り分ける。
#   条件のほかは exp_lidar_carry_1023.sh の carry0 と同じ（学習の扉・256 回・立った姿勢から・時間 2 倍・可動域の端から 0.05 rad・
#   扉の角度の雑音・測位の誤差 0・通り抜けで数える）。10-17・10-23 の「測位の誤差 0」は 87.1%
#     old    古い条件（立つ HOMIE に指令をそのまま渡す eval.zero_stand_cmd=False。直しは何も入れない）。87% 前後なら作業ツリーが同じ
#     z8     直し 8 だけ（立つ HOMIE の指令を 0。既定）
#     A      8＋当たりの形
#     B2     8＋腕の積み上げの切り方（B）＋送る目標の 1 周期 0.7 rad
#     phys   8＋armature（1）・遅れ（5）・関節の摩擦（6）・膝 120（7）
#     dr4    8＋乱択（4。評価では押さない）
#     all    全部（8・A・B・2・1・5・6・7・4・3・C。3 と C は罰なので評価には効かないが、測り具を入れた形で回す）
# V11: 直し B・8 を入れた評価で正解の記録を取り直す（8 台・観測の雑音 0）。check_golden.py（Mac）で door_core と突き合わせる
#
# 札: 一時停止の札（PAUSE_DOORMAN_TRAINING）は 10-24 から置かれている（ユーザー）。KEEP_PAUSE=1 で、置くだけで外さない。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup bash exp_simfix_v11v12_1026.sh > logs_simfix_v11v12_1026.log 2>&1 &
#   一部だけ: RUNS="old all" bash exp_simfix_v11v12_1026.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
SENSOR="++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0"
LOC0="++env.config.door_sensor.loc_pos_bias_m=0.0 ++env.config.door_sensor.loc_pos_noise_m=0.0 ++env.config.door_sensor.loc_yaw_bias_deg=0.0 ++env.config.door_sensor.loc_yaw_noise_deg=0.0"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 $SENSOR $LOC0"
FIX="$HOME/work_inoue/doorman6/simfix_args.sh"
# BASE_ARGS と同じ鍵（clip_targets_to_limits_margin）は二重に渡さない
fix() { "$FIX" "$@" | tr ' ' '\n' | grep -v '^++env.config.clip_targets_to_limits_margin=' | tr '\n' ' '; }
echo "=== [$(date +%T)] exp_simfix_v11v12_1026 start"
run() {
  case " ${RUNS:-old z8 A B2 phys dr4 all golden Bonly 2only Bj p1 p5 p6 p7} " in *" $1 "*) ;; *) return 0 ;; esac
  echo "=== [$(date +%T)] $1"
  EVAL_REPO=VIRAL6_fix KEEP_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" \
    ./eval_from_start.sh "$CK" "${3:-256}" 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
run old "++algo.config.eval.zero_stand_cmd=False"
run z8 ""
run A "$(fix A)"
run B2 "$(fix B 2)"
run phys "$(fix 1 5 6 7)"
run dr4 "$(fix 4)"
run all "$(fix)"
# 10-26 追加: B と 2 の切り分け（δ の判断材料）と、物理の 4 項目の切り分け（学習し直しの見込み）
run Bonly "$(fix B)"
run 2only "$(fix 2)"
run Bj "++env.config.arm_accum_guard_delta=[0.2,0.2,0.3,0.3,0.3,0.2,0.2,0.2,0.2,0.3,0.3,0.3,0.2,0.2] ++env.config.arm_accum_guard_margin=0.05"
run p1 "$(fix 1)"
run p5 "$(fix 5)"
run p6 "$(fix 6)"
run p7 "$(fix 7)"
GOLDEN="$HOME/work_inoue/doorman/records/golden_simfix_1026.npz"
NOISE0="++obs.noise_scales.dof_pos=0.0 ++obs.noise_scales.dof_vel=0.0 ++obs.noise_scales.arm_dof_torque=0.0 ++obs.noise_scales.e_homie_dof_pos=0.0 ++obs.noise_scales.f_homie_dof_vel=0.0"
run golden "$(fix B) $NOISE0 ++algo.config.eval.golden_trace_path=$GOLDEN" 8
echo "=== [$(date +%T)] exp_simfix_v11v12_1026 done"
