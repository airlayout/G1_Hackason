#!/usr/bin/env bash
# 11-06 夜（ユーザー「解析と必要に応じて学習を開始して。ストップと言うまで自律的に進めて」）: 鈍い脚（倍率 0.3〜0.8・遅れ 5〜28 歩。
# 実機の純正の歩行の実測 前後 0.52・左右 0.36・旋回 0.61 倍）で教師と生徒を学び直す。計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.5
#   1. 教師 f3t2（t2_1106.sh・強化学習 PPO）の区切りの評価が SWITCH_RATE 以上になるか SWITCH_AT を過ぎたら、止め札 STOP_DOOR_F3 を
#      置いて区切りの境目で止める。評価の最も良かった区切りの重みを教師にする（評価は区切りごとに振れる）
#   2. 生徒 e2b を鈍い脚で評価（学習なし。比べる元。e2b は元の脚で 84.8%）
#   3. 生徒を e2b の重みから、鈍い脚・新しい教師で学び直す（DAgger＝模倣学習。段の名前 e3。ほかの引数は e2b と同じ）。STOP_AT に止め札
#   4. e3 を鈍い脚で評価 → 実機用に書き出す（vis_export/door_vision_e3*）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash night_1106.sh >> vis_1104/night_1106.log 2>&1 < /dev/null &
#   既定: SWITCH_RATE=0.88 SWITCH_AT=0130 STOP_AT=0700。止める: ~/work_inoue/STOP_NIGHT_1106 を置く（どの段でも次の境目で抜ける）
set -uo pipefail
B="$HOME/work_inoue/doorman6"; V="$B/vis_1104"; D="$HOME/work_inoue/doorman"; W="$HOME/work_inoue"
T2LOG="$D/logs_t2_1106.log"
SWITCH_RATE="${SWITCH_RATE:-0.88}"; SWITCH_AT="${SWITCH_AT:-0130}"; STOP_AT="${STOP_AT:-0700}"
LEGS=("++env.config.loco_cmd_delay_steps_range=[5,28]" "++env.config.loco_cmd_gain_range=[0.3,0.8]")
E2B="$B/VIRAL6_vis/logs_rl/vis_e2b/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261106_173309/last.pt"
ENC="$B/vis_1104/pretrain_rgbd_1106_1117.pt"
NOD=++env.config.vision_feat.nod_deg=4.0; NOD0=++env.config.vision_feat.nod_deg=0.0
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"
CAMS="++simulator.config.cameras.camera_types=[{rgb:true},{depth:true}]"
COFF0="++env.config.vision_feat.color_off_prob=0.0"
T1="++env.config.shoulder_roll_press_max_nm=6.0"
QUIT="$W/STOP_NIGHT_1106"
log() { echo "=== [$(date "+%m-%d %T")] night: $*"; }
# HHMM → 秒。夜をまたぐので、正午より後に呼んだら翌日の時刻とみなす
at_epoch() { local hm="$1" day; day=$(date +%F); [ "$(date +%-H)" -ge 12 ] && day=$(date -d tomorrow +%F); date -d "$day ${hm:0:2}:${hm:2:2}" +%s; }
quit() { [ -e "$QUIT" ] && { log "止め札 $QUIT があるので抜ける（$1）"; exit 0; }; }
ev() {  # 名前・重み・足す引数…
  local name="$1" ck="$2" tag; shift 2; tag=$(date +%m%d_%H%M)
  bash "$B/eval_vis_1104.sh" "$ck" "$V/eval_${tag}_$name" 256 "$@" > "$V/eval_${tag}_$name.log" 2>&1
  log "評価 $name: $(tail -n 1 "$V/eval_${tag}_$name.log")"
}
[ -f "$E2B" ] && [ -f "$ENC" ] || { log "e2b か網の重みが無い"; exit 1; }

# 1. 教師を止める時を待つ
SW=$(at_epoch "$SWITCH_AT")
log "教師 f3t2 を待つ（${SWITCH_RATE} 以上か $(date -d @"$SW" '+%m-%d %H:%M') まで）"
while true; do
  quit "教師を待つ間"
  best=$(grep -a "stage=f3t2 round=[0-9]* iter=" "$T2LOG" | sed -n 's/.*round=\([0-9]*\) iter=[0-9]* success=[0-9/]* rate=\([0-9.]*\).*/\2 \1/p' | sort -rn | head -1)
  if [ -n "$best" ] && awk -v r="${best%% *}" -v s="$SWITCH_RATE" 'BEGIN { exit !(r >= s) }'; then log "教師が線に届いた（率 区切り）= $best"; break; fi
  [ "$(date +%s)" -ge "$SW" ] && { log "切り替えの時刻（最良の 率 区切り = ${best:-なし}）"; break; }
  pgrep -f "[t]2_1106.sh" > /dev/null || { log "t2_1106.sh が先に終わっていた（最良 = ${best:-なし}）"; break; }
  sleep 60
done
touch "$W/STOP_DOOR_F3"
log "教師に止め札を置いた。区切りの境目で止まるのを待つ"
while pgrep -f "[t]2_1106.sh" > /dev/null; do sleep 30; done
# 区切り r の重み = 区切り r+1 の起こし直しの resume= ／最後の区切りは chain_done の last_ckpt=
best=$(grep -a "stage=f3t2 round=[0-9]* iter=" "$T2LOG" | sed -n 's/.*round=\([0-9]*\) iter=[0-9]* success=[0-9/]* rate=\([0-9.]*\).*/\2 \1/p' | sort -rn | head -1)
[ -n "$best" ] || { log "f3t2 の区切りの評価が無い"; exit 1; }
r="${best##* }"
TEACHER=$(grep -a "stage=f3t2 task=.* round=$(( r + 1 ))/" "$T2LOG" | tail -1 | sed -n 's/.* resume=\([^ ]*\).*/\1/p')
[ -n "$TEACHER" ] || TEACHER=$(grep -a "chain_done" "$T2LOG" | tail -1 | sed -n 's/.*last_ckpt=\([^ ]*\).*/\1/p')
[ -f "$TEACHER" ] || { log "教師の重みが見つからない（区切り $r）: $TEACHER"; exit 1; }
cp -p "$TEACHER" "$W/doorman/ckpt/f3t2_best_round${r}.pt"
log "教師 = f3t2 の区切り $r（率 ${best%% *}）: $TEACHER（写し ckpt/f3t2_best_round${r}.pt）"
quit "教師の後"

# 2. 生徒 e2b を鈍い脚で（比べる元）
ev e2b_slowlegs "$E2B" "$VDELAY" "$NOD0" "$COFF0" "${LEGS[@]}"
quit "e2b の評価の後"

# 3. 生徒 e3（DAgger）。17:50 の止め札が残っているので外してから
rm -f "$W/STOP_DOOR_VIS_ALL" "$W/STOP_DOOR_VIS"
ST=$(at_epoch "$STOP_AT")
( until [ "$(date +%s)" -ge "$ST" ] || [ -e "$QUIT" ]; do sleep 30; done
  touch "$W/STOP_DOOR_VIS_ALL" "$W/STOP_DOOR_VIS"; log "生徒に止め札を置いた" ) &
E1=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e1b/*/*/last.pt | head -1)
TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$E1"
TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_e3/*/*/last.pt 2>/dev/null | head -1; }
CK="$E2B"; SEED0=4600
for attempt in $(seq 0 6); do
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  log "e3 連鎖 $attempt teacher=$TEACHER ck=$CK seed0=$SEED0"
  LEGS_DELAY=5,28 LEGS_GAIN=0.3,0.8 TEACHER="$TEACHER" ENC="$ENC" CK="$CK" NUM_ENVS=256 ROUNDS=48 ROUND_MIN=30 SEED0="$SEED0" \
    EXTRA_ARGS="$NOD $VDELAY $CAMS $COFF0 $T1 $TRK" bash "$B/exp_vis_1104.sh" e3
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  NEW=$(newest)
  if [ -z "$NEW" ] || [ "$NEW" = "$CK" ]; then log "e3 連鎖 $attempt が新しい重みを残さずに終わった。起こし直さない"; break; fi
  CK="$NEW"; SEED0=$(( SEED0 + 20 )); log "e3 連鎖 $attempt が止め札の前に終わった。最新の重みから起こし直す"
done
E3=$(newest)
[ -n "$E3" ] || { log "e3 の重みが無い"; exit 1; }

# 4. 評価 → 書き出し
ev e3_slowlegs "$E3" "$VDELAY" "$NOD0" "$COFF0" "${LEGS[@]}"
log "書き出す ck=$E3"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=3G -p CPUWeight=20 \
  env CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONPATH=".:$B/real_vis" \
  "$HOME/miniforge3/envs/doorman6/bin/python" "$B/real_vis/export_vision_student.py" --ckpt "$E3" --viral . \
  --out-dir "$B/vis_export" --name door_vision_e3 2>&1 | grep -a "\[export\]"
log "終わり e3=$E3: $(ls "$B"/vis_export/door_vision_e3* 2>/dev/null | tr '\n' ' ')"
