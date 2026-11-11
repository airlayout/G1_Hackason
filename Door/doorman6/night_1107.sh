#!/usr/bin/env bash
# 11-07（案 A・計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.6）: 実機に合わせた純正の応え方（パッチ 0095）で、
# 教師 f3t3（t3_1107.sh・強化学習 PPO）→ 生徒 e4（DAgger＝模倣学習）とつなぐ。night_1106.sh と同じ流れ。
#   1. 教師の区切りの評価が SWITCH_RATE 以上か SWITCH_AT を過ぎたら、止め札 STOP_DOOR_F3 → 評価の最も良かった区切りを教師にする
#   2. 生徒 e2b・e3 をその脚で評価（学習なし）→ 良い方を生徒の出発点にする
#   3. 生徒 e4 を DAgger で学び直す（脚・持ち時間・回の上限は教師と同じ。ほかの引数は e2b・e3 と同じ）。STOP_AT に止め札
#   4. e4 をその脚で評価 → 実機用に書き出す（vis_export/door_vision_e4*）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash night_1107.sh >> vis_1104/night_1107.log 2>&1 < /dev/null &
#   既定: SWITCH_RATE=0.88 SWITCH_AT=1400 STOP_AT=0700。全部止める: ~/work_inoue/STOP_NIGHT_1107 を置く
set -uo pipefail
B="$HOME/work_inoue/doorman6"; V="$B/vis_1104"; D="$HOME/work_inoue/doorman"; W="$HOME/work_inoue"
T3LOG="$D/logs_t3_1107.log"
FIT="${FIT:-$D/fit_loco_resp_1107.txt}"
SWITCH_RATE="${SWITCH_RATE:-0.88}"; SWITCH_AT="${SWITCH_AT:-1400}"; STOP_AT="${STOP_AT:-0700}"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) echo "応え方の層の値が無い: $FIT"; exit 1 ;; esac
read -r -a RESP <<< "$RESP_EVAL"
SW=(++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1)
LONGEP=++env.config.max_episode_length_s=45
E2B="$B/VIRAL6_vis/logs_rl/vis_e2b/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261106_173309/last.pt"
E3=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e3/*/*/last.pt 2>/dev/null | head -1)
ENC="$B/vis_1104/pretrain_rgbd_1106_1117.pt"
NOD=++env.config.vision_feat.nod_deg=4.0; NOD0=++env.config.vision_feat.nod_deg=0.0
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"
CAMS="++simulator.config.cameras.camera_types=[{rgb:true},{depth:true}]"
COFF0="++env.config.vision_feat.color_off_prob=0.0"
T1="++env.config.shoulder_roll_press_max_nm=6.0"
QUIT="$W/STOP_NIGHT_1107"
log() { echo "=== [$(date "+%m-%d %T")] night7: $*"; }
at_epoch() { local hm="$1" day; day=$(date +%F); [ "$(date +%-H)" -ge 12 ] && day=$(date -d tomorrow +%F); date -d "$day ${hm:0:2}:${hm:2:2}" +%s; }
quit() { [ -e "$QUIT" ] && { log "止め札 $QUIT があるので抜ける（$1）"; exit 0; }; }
RATE=""
ev() {  # 名前・重み・足す引数…（成功の割合を RATE に返す）
  local name="$1" ck="$2" tag lg; shift 2; tag=$(date +%m%d_%H%M); lg="$V/eval_${tag}_$name.log"
  bash "$B/eval_vis_1104.sh" "$ck" "$V/eval_${tag}_$name" 256 "$@" > "$lg" 2>&1
  RATE=$(tail -n 1 "$lg" | sed -n 's/.*= *\([0-9.]*\)%.*/\1/p')
  log "評価 $name: $(tail -n 1 "$lg")"
}
best_round() { grep -a "stage=f3t3 round=[0-9]* iter=" "$T3LOG" | sed -n 's/.*round=\([0-9]*\) iter=[0-9]* success=[0-9/]* rate=\([0-9.]*\).*/\2 \1/p' | sort -rn | head -1; }
[ -f "$E2B" ] && [ -f "$ENC" ] || { log "e2b か網の重みが無い"; exit 1; }

# 1. 教師
SWE=$(at_epoch "$SWITCH_AT")
log "教師 f3t3 を待つ（${SWITCH_RATE} 以上か $(date -d @"$SWE" '+%m-%d %H:%M') まで）"
while true; do
  quit "教師を待つ間"
  best=$(best_round)
  if [ -n "$best" ] && awk -v r="${best%% *}" -v s="$SWITCH_RATE" 'BEGIN { exit !(r >= s) }'; then log "教師が線に届いた（率 区切り）= $best"; break; fi
  [ "$(date +%s)" -ge "$SWE" ] && { log "切り替えの時刻（最良の 率 区切り = ${best:-なし}）"; break; }
  pgrep -f "[t]3_1107.sh" > /dev/null || { log "t3_1107.sh が先に終わっていた（最良 = ${best:-なし}）"; break; }
  sleep 60
done
touch "$W/STOP_DOOR_F3"
log "教師に止め札を置いた。区切りの境目で止まるのを待つ"
while pgrep -f "[t]3_1107.sh" > /dev/null; do sleep 30; done
best=$(best_round)
[ -n "$best" ] || { log "f3t3 の区切りの評価が無い"; exit 1; }
r="${best##* }"
TEACHER=$(grep -a "stage=f3t3 task=.* round=$(( r + 1 ))/" "$T3LOG" | tail -1 | sed -n 's/.* resume=\([^ ]*\).*/\1/p')
[ -n "$TEACHER" ] || TEACHER=$(grep -a "chain_done" "$T3LOG" | tail -1 | sed -n 's/.*last_ckpt=\([^ ]*\).*/\1/p')
[ -f "$TEACHER" ] || { log "教師の重みが見つからない（区切り $r）: $TEACHER"; exit 1; }
cp -p "$TEACHER" "$D/ckpt/f3t3_best_round${r}.pt"
log "教師 = f3t3 の区切り $r（率 ${best%% *}）: $TEACHER（写し ckpt/f3t3_best_round${r}.pt）"
quit "教師の後"

# 2. 生徒の出発点（実機に合わせた脚で e2b・e3 を測る）
ev e2b_resp "$E2B" "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}"
R2B="${RATE:-0}"; CK="$E2B"
if [ -n "$E3" ]; then
  ev e3_resp "$E3" "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}"
  awk -v a="${RATE:-0}" -v b="$R2B" 'BEGIN { exit !(a > b) }' && CK="$E3"
fi
log "生徒の出発点: $CK"
quit "生徒の評価の後"

# 3. 生徒 e4（DAgger）
rm -f "$W/STOP_DOOR_VIS_ALL" "$W/STOP_DOOR_VIS"
ST=$(at_epoch "$STOP_AT")
( until [ "$(date +%s)" -ge "$ST" ] || [ -e "$QUIT" ]; do sleep 30; done
  touch "$W/STOP_DOOR_VIS_ALL" "$W/STOP_DOOR_VIS"; log "生徒に止め札を置いた" ) &
E1=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e1b/*/*/last.pt | head -1)
TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$E1"
TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_e4/*/*/last.pt 2>/dev/null | head -1; }
SEED0=4800
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
E4=$(newest)
[ -n "$E4" ] || { log "e4 の重みが無い"; exit 1; }

# 4. 評価 → 書き出し
ev e4_resp "$E4" "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}"
log "書き出す ck=$E4"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=3G -p CPUWeight=20 \
  env CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONPATH=".:$B/real_vis" \
  "$HOME/miniforge3/envs/doorman6/bin/python" "$B/real_vis/export_vision_student.py" --ckpt "$E4" --viral . \
  --out-dir "$B/vis_export" --name door_vision_e4 2>&1 | grep -a "\[export\]"
log "終わり e4=$E4: $(ls "$B"/vis_export/door_vision_e4* 2>/dev/null | tr '\n' ' ')"
