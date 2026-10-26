#!/usr/bin/env bash
# L3（10-23、ライダーだけの本線）: 枠を「最初に一度測り、以後は odometry で運ぶ」とき、運ぶ誤差が成功率をどれだけ下げるか。評価だけ。
#
# 実機では止まっている間にライダーで枠を測り、歩き出した後は脚の odometry（/dog_odom）か FAST_LIO で運ぶ
# （計画書 2026-10-21-g1-door-odom-measurement.md の L1〜L3）。運ぶ模型はパッチ 0067（door_sensor.carry.*）。
# 扉の角度は 10-17 の実測の模型（揺れ 0.2°・片寄り ±1.5°・欠け 0）。測位の誤差（loc_*）は 0 にして、運ぶ模型だけを効かせる。
# 条件のほかは exp_locsplit_1017.sh と同じ（学習の扉・256 回・立った姿勢から・時間 2 倍・可動域の端から 0.05 rad・通り抜けで数える）。
#
#   carry0   運ぶ模型を入れて誤差 0。10-17 の「測位の誤差 0」87.1% と同じなら作りが正しい
#   raw      純正の歩行で測った /dog_odom のまま（前後 0.955・横 0.92・回ごと σ 0.015、向き σ 0.5%）＋最初の測定の片寄り 1 cm・0.5°
#   corr     倍率を直した残り（平均 1.0・σ 0.025）＋同じ片寄り
#   refit    運ばない。毎回当て直す（測位の片寄り 1 cm・揺れ 0.5 cm・向き 0.5°・0.2°）
#   start2   運ぶ誤差 0・最初の測定の片寄りだけ 2 cm・1°
#   lat085   横 0.85・前後 0.92（HOMIE の歩き方がもっと悪い場合）＋片寄り 1 cm・0.5°
#   fastlio  倍率 σ 0.005・向き σ 0.2%・酔歩 0.5 cm/√s＋片寄り 1 cm・0.5°（FAST_LIO の見込み。L2 の数字で置き換える）
#
# 札: 映像の生徒の学習を止める札（PAUSE_DOORMAN_TRAINING）を置いて回し、最初から札があったときは終わっても外さない。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup bash exp_lidar_carry_1023.sh > logs_lidar_carry_1023.log 2>&1 &
#   一部だけ: RUNS="carry0 raw" bash exp_lidar_carry_1023.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
source ./gate.sh
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
SENSOR="++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0"
loc() {  # 位置の片寄り, 位置の揺れ, 向きの片寄り, 向きの揺れ
  echo "++env.config.door_sensor.loc_pos_bias_m=$1 ++env.config.door_sensor.loc_pos_noise_m=$2 ++env.config.door_sensor.loc_yaw_bias_deg=$3 ++env.config.door_sensor.loc_yaw_noise_deg=$4"
}
carry() {  # 最初の位置の片寄り, 最初の向きの片寄り, 前後の倍率, 横の倍率, 倍率の σ, 向きの倍率の σ, 酔歩
  local k=++env.config.door_sensor.carry
  echo "$k.enabled=True $k.start_pos_bias_m=$1 $k.start_yaw_bias_deg=$2 $k.scale_fwd_mean=$3 $k.scale_lat_mean=$4 $k.scale_sigma=$5 $k.yaw_scale_sigma=$6 $k.pos_rw_m=$7 $(loc 0.0 0.0 0.0 0.0)"
}
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 $SENSOR"

had_pause=0
[ -f "$PAUSE_FILE" ] && had_pause=1
echo "=== [$(date +%T)] exp_lidar_carry_1023 start（札は始めから $([ "$had_pause" = 1 ] && echo 有った・終わっても外さない || echo 無かった・終わったら外す)）"
for _ in 1 2 3; do ./pause_training.sh on && break; done
trap '[ "$had_pause" = 0 ] && ./pause_training.sh off; echo "=== [$(date +%T)] exp_lidar_carry_1023 exit"' EXIT

run() {
  case " ${RUNS:-carry0 raw corr refit start2 lat085 fastlio} " in *" $1 "*) ;; *) return 0 ;; esac
  echo "=== [$(date +%T)] $1"
  KEEP_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" \
    ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
run carry0 "$(carry 0.0 0.0 1.0 1.0 0.0 0.0 0.0)"
run raw "$(carry 0.01 0.5 0.955 0.92 0.015 0.005 0.0)"
run corr "$(carry 0.01 0.5 1.0 1.0 0.025 0.005 0.0)"
run refit "$(loc 0.01 0.005 0.5 0.2)"
run start2 "$(carry 0.02 1.0 1.0 1.0 0.0 0.0 0.0)"
run lat085 "$(carry 0.01 0.5 0.92 0.85 0.015 0.005 0.0)"
run fastlio "$(carry 0.01 0.5 1.0 1.0 0.005 0.002 0.005)"
echo "=== [$(date +%T)] exp_lidar_carry_1023 done"
