#!/usr/bin/env bash
# 11-09（案 A・計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md の手順 3・4。ユーザー「引き継ぎに従い学習を実施して」）:
#   段 2 レバーを腕だけで学ぶ（強化学習 PPO。パッチ 0097 の env.config.arm_stage）。構えの位置に立たせて段 1（構える）から始め、
#   脚は毎歩 立つ HOMIE（実機の段 2 は純正で立ったまま＝歩きの指令 0）、扉が 10° 開いたら（段 4 に入ったら）完了。
#   構えの位置: 教師 f3t3 がレバーに触れた時（段 2 に入った時）の pelvis の 10〜90% 点を広げた範囲。扉の面から前 −0.48〜−0.32 m・
#   レバーの付け根から横 −0.15〜−0.01 m（レバーの反対側へ）・向き −65〜−25°（右へ回って左肩を前へ）。11-09 の 256 回の評価の記録
#   1. 教師 f3t3 をこの場面で 256 回評価（学習なし。手順 3 の比べる元）
#   2. f3t3 から学び直す（段の名前 s2b。脚・腕・扉の引数は f3t3〔t3_1107.sh〕と同じ。30 分の学習 → 256 回の評価を止め札まで）
#      回の上限は 10 秒（段の持ち時間は f3t3 と同じ [500,200,200,200,600,400] 歩＝観測の尺度を変えない）。途中から始める回は
#      段 2・3 だけ（段 4 から始めると始めから完了になる）。段 4 の「止まっていると罰」は 0（脚を止めたまま 10° に届くと罰になる）
#   12:55 の変更（パッチ 0098・ユーザー「引き継ぐ角度を下げる」）: 2 区切り（1 時間）でレバーを下げきるまでは覚えたが、扉は 0.1° から
#      動かず（押すと体が後ろへ最大 15 cm ずれ、手がレバーを手前へ引いた）、10° の成功は 0/256。完了を「扉が DOOR_DEG（既定 3）°」に
#      下げ（ラッチが外れて扉が動いた所で f3t3 に引き継ぐ）、数え方は arm3、段の名前は s2c。完了の後の遅れは 50 → 10 歩
#      （段 3 の持ち時間の終わり際に完了しても、遅れの間に持ち時間切れで打ち切られにくくする）。DOOR_DEG= で 10° の段の判定に戻す
#   14:15 の変更（パッチ 0099・ユーザー「ラッチが外れたら脚の固定を外す」）: s2c の 2 区切りでも 3° は 4 → 3 → 0/256（レバーを
#      下げきるのは 74% のまま）。レバーが 30° 以上（掛け金が縁まで引っ込んだ）になったら、その回の終わりまで歩く指令を通す。
#      段の名前は s2d。FREE_LEGS=0 で毎歩止める（s2c と同じ）
#   17:40 の変更（ユーザー「始めの前後を −0.50〜−0.37 m に狭める」）: s2d は 5 区切りで 76.6 → 83.2%。扉に近い始め（−0.37〜
#      −0.32 m）だけがレバーに 59〜64% しか触れない（構える点〔当てる点の 14 cm 手前・12 cm 上〕が胸と重なる見込み）。
#      START_RANGE="[-0.50|-0.37|-0.15|-0.01|-65.0|-25.0]" B_NAME=s2e で s2d の重みから続ける（段 1 は扉から 0.43 m 前後に止める前提）
#   11-10 の変更（ユーザー「レバーを下げたか: 案 1」「左に寄って立つ: 案 3」「引き継ぎは f3t3（2 つの方策）」「横の始めの範囲を網の誤差の分
#      だけ広げてよい」）: s2e の 13 区切り目（学習に無い扉で 97.3%・実機の立ち位置では 84.4%）から、RELEASE=palm_press・
#      DOOR_NOISE="0.06|2.0"・START_RANGE="[-0.50|-0.37|-0.20|0.11|-65.0|-25.0]"・B_NAME=s2f で学び直す。完了は 3° のまま（そこで f3t3 へ）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup ./s2b_1109.sh >> logs_s2b_1109.log 2>&1 < /dev/null &
#   止め札: ~/work_inoue/STOP_DOOR_F3（区切りの境目で止まる）。評価を飛ばす: SKIP_EVAL=1。続きから: SKIP_EVAL=1 CK=<s2c の last.pt>
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] s2b: $*"; }
CK="${CK:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt}"
FIT="${FIT:-$HOME/work_inoue/doorman/fit_loco_resp_1107.txt}"
DOOR_ENV="$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/door/door_open_homie.py"
TASK=door_en1154_g13_shaped_left_weight
T1="++env.config.shoulder_roll_press_max_nm=6.0"
PEN="++rewards.reward_scales.penalty_shoulder_roll_press=-0.01"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
RESP_CHAIN=$(grep -A1 '学習の連鎖の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) log "応え方の層の値が無い: $FIT"; exit 1 ;; esac
SWITCH="++algo.config.homie_switch_threshold=0.1,++env.config.walk_in_hold_threshold=0.1"
# 腕だけの段（0097）。連鎖の書き方: 引数は「,」区切り・リストの中のカンマは「|」
START_RANGE="${START_RANGE:-[-0.48|-0.32|-0.15|-0.01|-65.0|-25.0]}"  # 前 x0|x1・横 y0|y1・向き 0|1（連鎖の書き方で | 区切り）
ARM="++env.config.arm_stage.enabled=True,++env.config.arm_stage.start_range=$START_RANGE"
ARM="$ARM,++env.config.staged_reset_ratios=[0.7|0.0|0.15|0.15|0.0|0.0],++rewards.reward_scales.penalty_standing_still=0.0"
DOOR_DEG="${DOOR_DEG-3.0}"  # 0098。空にすると段 4（10°）で完了（s2b と同じ）
if [ -n "$DOOR_DEG" ]; then
  ARM="$ARM,++env.config.arm_stage.complete_door_deg=$DOOR_DEG,++env.config.reset_on_complete_delay=10"
  CRIT=arm3 NAME_DEFAULT=s2c
else
  CRIT=arm10 NAME_DEFAULT=s2b
fi
FREE_LEGS="${FREE_LEGS:-1}"  # 0099。1 でレバーが下がって掛け金が外れたら脚を放す（0 で毎歩止める＝s2b・s2c と同じ）
if [ "$FREE_LEGS" = 1 ]; then
  ARM="$ARM,++env.config.arm_stage.free_legs_when_unlatched=True"
  NAME_DEFAULT=s2d
fi
# 11-10（パッチ 0100・ユーザー「レバーを下げたか: 案 1」）: RELEASE=palm_press で、脚を放す引き金を実機でも測れる量（手のひらの位置＋手と
#   レバーの力）にする（既定 lever_true＝シムの真のレバーの角度）。DOOR_NOISE="位置 m|向き °" で扉の値の回ごとの片寄りを替える
#   （例 "0.06|2.0"＝11-09 午前に実測した画像の網の誤差。空なら既定の σ3 cm・σ1°）
RELEASE="${RELEASE:-lever_true}"
case "$RELEASE" in lever_true) ;; palm_press) ARM="$ARM,++env.config.arm_stage.release=palm_press" ;;
  *) log "RELEASE は lever_true か palm_press: $RELEASE"; exit 1 ;; esac
if [ -n "${DOOR_NOISE:-}" ]; then
  ARM="$ARM,++env.config.door_sensor.loc_pos_bias_m=${DOOR_NOISE%%|*},++env.config.door_sensor.loc_yaw_bias_deg=${DOOR_NOISE##*|}"
fi
ARM_TRAIN="$ARM,++env.config.reset_from_dataset.enabled=False"  # 評価は EVAL_EXTRA が同じ鍵を持つ（重ねて渡さない）
STAGE="++env.config.max_stage_time=[500|200|200|200|600|400],++env.config.max_episode_length_s=10"
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
[ -f "$CK" ] || { log "出発点の重みが無い: $CK"; exit 1; }
grep -qF "arm_stage_config" "$DOOR_ENV" || { log "パッチ 0097 が VIRAL6_fix に当たっていない"; exit 1; }
grep -qF "\"$CRIT\"" count_success.py || { log "count_success.py に $CRIT が無い"; exit 1; }
if [ -n "$DOOR_DEG" ]; then
  grep -qF "complete_door_deg" "$DOOR_ENV" || { log "パッチ 0098 が VIRAL6_fix に当たっていない"; exit 1; }
fi
if [ "$FREE_LEGS" = 1 ]; then
  grep -qF "free_legs_when_unlatched" "$DOOR_ENV" || { log "パッチ 0099 が VIRAL6_fix に当たっていない"; exit 1; }
fi
if [ "$RELEASE" = palm_press ]; then
  grep -qF "_arm_stage_palm_press" "$DOOR_ENV" || { log "パッチ 0100 が VIRAL6_fix に当たっていない"; exit 1; }
fi
grep -qF 'CRIT:-through' exp_f3_retrain_1026.sh || { log "exp_f3_retrain_1026.sh が CRIT を読まない"; exit 1; }
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
if [ "${SKIP_EVAL:-0}" != 1 ]; then
  log "評価 出発点＋腕だけの段 ck=$CK crit=$CRIT"
  ARM_EVAL=$(echo "$ARM" | tr ',' ' ' | tr '|' ',')
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion $CRIT" MAX_WAIT_MIN=120 MEM_MAX=9G \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $ARM_EVAL $T1 $(echo "$SWITCH" | tr ',' ' ') $RESP_EVAL" \
    ./eval_from_start.sh "$CK" 256 > logs_s2b_1109_eval_base.log 2>&1
  log "評価 終わり: $(tail -n 2 logs_s2b_1109_eval_base.log | tr "\n" " ")"
fi
B_NAME="${B_NAME:-$NAME_DEFAULT}"
log "学び直し $B_NAME を始める（数え方 $CRIT・完了の扉の角度 ${DOOR_DEG:-段 4}）"
SKIP_A=1 MASK=0 COMPLETE_THROUGH=1 TERM_UP=0.9 B_NAME="$B_NAME" AUTO_ENVS=1 IDLE_GATE="${IDLE_GATE:-1}" CK="$CK" CRIT="$CRIT" \
  STOCK_MORE="$SWITCH,$RESP_CHAIN" LONG_MORE="$STAGE" EXTRA_MORE="$T1,$PEN,$ARM_TRAIN" ./exp_stocklegs_1030.sh
log "終わり"
