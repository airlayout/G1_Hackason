#!/usr/bin/env bash
# 11-08（計画書 docs/plan/2026-11-06-g1-door-depth-rl.md 8 章・案 A・ユーザー了承「この計画で実施して」）:
#   生徒 e4（DAgger＝模倣学習）を、自分の報酬の PPO（強化学習）＋真似の錨＋追う段の補助の損失で磨く（パッチ 0096）。
#   R1: 走っている e4 の連鎖（cont_e4_1108.sh）を今の区切りの終わりで止め、続きの区切り 3・6 と最新を評価する（層 v2 の脚・
#       seed 999・256 回・07:00 と同じ引数）。区切り 10（07:00 の評価 70.3%）と比べて最良を出発点にし、出発点は段の持ち時間で
#       打ち切らない評価（reset_on_overtime=False・CRITERION=passed）でも測る
#   R2: 出発の重みを組み立てる（actor＝出発点の生徒・critic と env＝教師 f3t3・std は f3t3 × 0.5。make_vis_ppo_ckpt.py）
#   R3: 強化学習の連鎖 r1（30 分の区切り・区切りごとに扉の種を替える・256 台）。区切り 0 の最初の 50 反復は評価係の慣らし
#   R4: 区切り 3 の後に 1 回だけ学習を止めて評価する（今の評価と段の持ち時間で打ち切らない評価）。出発点の評価以上なら続ける。
#       下回れば止めて手当てを見直す（ここで終わる）
#   その後は止め札 ~/work_inoue/STOP_DOOR_VIS_ALL まで続ける（落ちたら最新の重みから起こし直す）。止めた後に区切りの重みを
#   いくつか評価して最良を選ぶ（学習中に評価は挟まない）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash rl_r1_1108.sh >> vis_1104/rl_r1_1108.log 2>&1 < /dev/null &
#   START_CK=<生徒の last.pt>: R1 の候補の評価を飛ばしてこの重みから（e4 は止める。出発点の 2 本の評価は回す）
#   START_PT=<組み立て済みの出発の重み> START_THROUGH=<出発点の今の評価> START_PASSED=<打ち切らない評価>: R1・R2 を飛ばして R3 から
#     （11-08 13:13 に区切り 0 が教師の clear_rollout で落ちた後の起こし直し）
#   GATE_CK=<r1 の重み> SEED0=<続きの最初の種> START_THROUGH=… START_PASSED=…: R1〜R3 を飛ばして R4 の関門から
#     （11-08 の R4 が画面の作業で空き RAM が足りず 60 分で諦めた後の起こし直し）。GATE_TAG で評価の名前、EVAL_WAIT_MIN で評価の空き待ちの上限（分）
#   EVAL_CKS="名前=重み …": 学習せず、並べた重みを 2 つの数え方で評価するだけ（止めた後の R5）
set -uo pipefail
B="$HOME/work_inoue/doorman6"; D="$HOME/work_inoue/doorman"; W="$HOME/work_inoue"; V="$B/vis_1104"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
FIT="$D/fit_loco_resp_1107.txt"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) echo "応え方の層の値が無い: $FIT"; exit 1 ;; esac
read -r -a RESP <<< "$RESP_EVAL"
LONGEP=++env.config.max_episode_length_s=45
ENC="$B/vis_1104/pretrain_rgbd_1106_1117.pt"
NOD=++env.config.vision_feat.nod_deg=4.0; NOD0=++env.config.vision_feat.nod_deg=0.0
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"
CAMS="++simulator.config.cameras.camera_types=[{rgb:true},{depth:true}]"
COFF0="++env.config.vision_feat.color_off_prob=0.0"
T1="++env.config.shoulder_roll_press_max_nm=6.0"
SW=(++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1)
TEACHER="$B/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt"
EXP=wbmanip/door_open_homie_29dof_vision_f3_ppo_distill-lstm
E4_R10="$B/VIRAL6_vis/logs_rl/vis_e4/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261108_063331/last.pt"
E4_R10_EVAL="$V/eval_1108_0700_e4_resp"
GATE_ROUNDS=4  # R4: 区切り 0〜3 の後に評価する
log() { echo "=== [$(date "+%m-%d %T")] rl_r1: $*"; }
newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_r1/*/*/last.pt 2>/dev/null | head -1; }
rate() { "$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$1/summary.json" 2>/dev/null || echo NA; }
[ -f "$TEACHER" ] && [ -f "$ENC" ] || { log "教師か網の重みが無い"; exit 1; }

# 評価 1 本（約 7 分）。$1 重み・$2 出力の名前・$3 数え方（through・passed）・残りは足す引数。成功率を返す
ev() {
  local ck="$1" out="$V/$2" crit="$3"; shift 3
  CRITERION="$crit" MINUTES=40 MAX_WAIT_MIN="${EVAL_WAIT_MIN:-60}" bash "$B/eval_vis_1104.sh" "$ck" "$out" 256 "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}" "$@" \
    > "$out.log" 2>&1
  rate "$out"
}
# 今の評価と、段の持ち時間で打ち切らない評価の 2 本。結果は R_THROUGH・R_PASSED に
ev_both() {
  R_THROUGH=$(ev "$1" "eval_$2" through)
  R_PASSED=NA
  [ "$R_THROUGH" = NA ] || R_PASSED=$(ev "$1" "eval_${2}_noot" passed ++env.config.reset_on_overtime=False)
  log "評価 $2: 今の評価 $R_THROUGH・段の持ち時間で打ち切らない評価 $R_PASSED（$1）"
}

# 止めた後に区切りの重みを評価するだけ（R5・11-09）。EVAL_CKS="名前=重み 名前=重み …"
if [ -n "${EVAL_CKS:-}" ]; then
  for kv in $EVAL_CKS; do ev_both "${kv#*=}" "${kv%%=*}"; done
  log "評価だけ 終わり"
  exit 0
fi

# --- R1: e4 を区切りの終わりで止める ---
touch "$W/STOP_DOOR_VIS_ALL"  # cont_e4 は連鎖の間でだけ見る＝起こし直させない
if pgrep -f "[c]ont_e4_1108.sh" > /dev/null; then
  LINE=$(grep -a "区切り [0-9]* seed=" "$V/cont_e4_1108.log" | tail -1)
  T0=$(date -d "$(echo "$LINE" | sed -E 's/^=== \[11-([0-9]+) ([0-9:]+)\].*/2026-11-\1 \2/')" +%s)
  STOP_AT=$(( T0 + 30 * 60 - 20 ))
  log "e4 を区切りの終わりで止める: 区切りの始まり $(date -d @$T0 +%T)・止め札 $(date -d @$STOP_AT +%T)"
  while [ "$(date +%s)" -lt "$STOP_AT" ]; do sleep 5; done
  touch "$W/STOP_DOOR_VIS"
  while pgrep -f "[c]ont_e4_1108.sh" > /dev/null; do sleep 5; done
fi
rm -f "$W/STOP_DOOR_VIS" "$W/STOP_DOOR_VIS_ALL"
log "e4 が止まった: $(grep -a "区切り [0-9]* 終わり" "$V/cont_e4_1108.log" | tail -1 | cut -c1-60)"

if [ -n "${GATE_CK:-}" ]; then
  CK="$GATE_CK"; START_THROUGH="${START_THROUGH:?}"; START_PASSED="${START_PASSED:?}"; SEED0="${SEED0:?}"
  [ -f "$CK" ] || { log "関門で測る重みが無い: $CK"; exit 1; }
  log "R1〜R3 を飛ばして R4 の関門から: 重み $CK・続きの種 $SEED0・出発点の評価 $START_THROUGH・打ち切らない評価 $START_PASSED"
elif [ -n "${START_PT:-}" ]; then
  CK="$START_PT"; START_THROUGH="${START_THROUGH:?}"; START_PASSED="${START_PASSED:?}"
  [ -f "$CK" ] || { log "出発の重みが無い: $CK"; exit 1; }
  log "R1・R2 を飛ばす: 出発の重み $CK・出発点の評価 $START_THROUGH・打ち切らない評価 $START_PASSED"
else
# --- R1: 出発点を選ぶ ---
if [ -n "${START_CK:-}" ]; then
  BEST="$START_CK"; BEST_TAG=start
else
  # 08:07 の連鎖（最後の「e4 連鎖 0」）の区切りの終わりの重みを順に。区切り 3・6 と最新を評価する
  mapfile -t CKS < <(awk '/cont_e4: e4 連鎖 0/{n=NR} {l[NR]=$0} END{for(i=n;i<=NR;i++) print l[i]}' "$V/cont_e4_1108.log" \
    | grep -a "区切り [0-9]* 終わり" | sed -E 's/.* last=//')
  log "e4 の続きの区切り ${#CKS[@]} 個"
  BEST="$E4_R10"; BEST_TAG=e4r10; BEST_RATE=$(rate "$E4_R10_EVAL")
  log "区切り 10（07:00）: $BEST_RATE"
  for i in $(printf "%s\n" 3 6 $(( ${#CKS[@]} - 1 )) | sort -un); do  # 最新が 3・6 と重なれば 1 回だけ
    [ "$i" -ge 0 ] && [ "$i" -lt "${#CKS[@]}" ] || continue
    ck="${CKS[$i]}"
    [ -f "$ck" ] || { log "続きの区切り $i の重みが無い: $ck"; continue; }
    r=$(ev "$ck" "eval_1108_e4c$i" through)
    log "続きの区切り $i: $r（$ck）"
    if [ "$r" != NA ] && "$PY" -c 'import sys; sys.exit(0 if float(sys.argv[1]) >= float(sys.argv[2]) else 1)' "$r" "$BEST_RATE"; then
      BEST="$ck"; BEST_TAG="e4c$i"; BEST_RATE="$r"
    fi
  done
fi
log "出発点: $BEST_TAG（$BEST）"
ev_both "$BEST" "1108_r1start_$BEST_TAG"
START_THROUGH="$R_THROUGH"; START_PASSED="$R_PASSED"
[ "$START_THROUGH" != NA ] || { log "出発点の評価が出ない。止める"; exit 1; }

# --- R2: 出発の重み ---
CK="$V/r1_start_$(date +%m%d_%H%M).pt"
(cd "$B/VIRAL6_vis" && PYTHONPATH=. "$PY" gr00t/rl/scripts/make_vis_ppo_ckpt.py "$BEST" "$TEACHER" "$CK" --std-scale 0.5)
[ -f "$CK" ] || { log "出発の重みが作れない"; exit 1; }
fi

run_chain() {  # $1 区切りの数・$2 最初の種
  EXP="$EXP" STAGE_TIME=500,200,200,200,600,400 TEACHER="$TEACHER" ENC="$ENC" CK="$CK" NUM_ENVS=256 ROUNDS="$1" ROUND_MIN=30 \
    SEED0="$2" EXTRA_ARGS="$NOD $VDELAY $CAMS $COFF0 $T1 $RESP_EVAL $LONGEP" bash "$B/exp_vis_1104.sh" r1
}

# --- R3: 区切り 0〜3 ---
if [ -z "${GATE_CK:-}" ]; then
  SEED0=6000
  log "R3 強化学習 r1 を始める ck=$CK 区切り $GATE_ROUNDS 個 seed0=$SEED0"
  run_chain "$GATE_ROUNDS" "$SEED0"
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && { log "止め札で止めた"; exit 0; }
  NEW=$(newest)
  [ -n "$NEW" ] && [ "$NEW" != "$CK" ] || { log "r1 が新しい重みを残さなかった。止める"; exit 1; }
  CK="$NEW"; SEED0=$(( SEED0 + GATE_ROUNDS ))
fi

# --- R4: 途中の関門 ---
ev_both "$CK" "${GATE_TAG:-1108_r1gate}"
if [ "$R_THROUGH" = NA ] || ! "$PY" -c 'import sys; sys.exit(0 if float(sys.argv[1]) >= float(sys.argv[2]) else 1)' "$R_THROUGH" "$START_THROUGH"; then
  log "R4 の関門を通らない（$R_THROUGH < 出発点 $START_THROUGH）。止めて手当てを見直す"
  exit 0
fi
log "R4 の関門を通った（$R_THROUGH ≥ 出発点 $START_THROUGH・打ち切らない評価 $R_PASSED／出発点 $START_PASSED）。続ける"

# --- 止め札まで続ける ---
for attempt in $(seq 0 6); do
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  log "r1 連鎖 $attempt ck=$CK seed0=$SEED0"
  run_chain 48 "$SEED0"
  [ -e "$W/STOP_DOOR_VIS_ALL" ] && break
  NEW=$(newest)
  if [ -z "$NEW" ] || [ "$NEW" = "$CK" ]; then log "r1 連鎖 $attempt が新しい重みを残さずに終わった。起こし直さない"; break; fi
  CK="$NEW"; SEED0=$(( SEED0 + 48 )); log "r1 連鎖 $attempt が止め札の前に終わった。最新の重みから起こし直す"
done
log "終わり r1=$(newest)"
