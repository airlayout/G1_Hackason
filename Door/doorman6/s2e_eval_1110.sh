#!/usr/bin/env bash
# 11-10（案 A・計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 手順 4 の合格の線。ユーザー「評価してよい」）:
#   腕だけの段 s2e の候補 2 つ（13 区切り目 96.5%・19 区切り目 96.1%。どちらも学習と同じ扉の組での値）を、学習に無い扉（種 999）で
#   評価して良い方を選び、その重みで前回の反省（11-09 午前の実機・午後の作業ログ）を確かめる条件をさらに 3 つ回す。最後に動画。
#   E1・E2: 候補 2 つ・既定の扉の値の雑音（学習と同じ。位置の片寄り σ3 cm・向き σ1°）
#   E3: 扉の位置の片寄り σ6 cm・向き σ2°（11-09 午前に実測した画像の網の誤差。止まって前 4.4/6.7 cm・左右 3.4/10.9 cm）
#   E4: 始めを左（壁の側）へ。付け根から横 +0.01〜+0.11 m（11-09 の実機は pelvis が開口の左の縁の真上＝付け根の約 6 cm 左）
#   E5: 純正の歩行の効かない帯を前後・左右 0.12 m/s に（11-09 の掃引で 0.1〜0.15 m/s。シムの層 v2 は前後 0.05）
#   動画: 良い方・種 999 から 8 台ずつ、失敗が 2 本そろうまで（最大 3 回。種 999・1000・1001）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash s2e_eval_1110.sh >> logs_s2e_eval_1110.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] s2e_eval: $*"; }
CK13="$HOME/work_inoue/doorman/ckpt/start_1110_0148/last.pt"
CK19="$HOME/work_inoue/doorman/ckpt/start_1110_0506/last.pt"
OUT="$HOME/work_inoue/doorman/s2e_eval_1110.tsv"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
[ -f "$CK13" ] && [ -f "$CK19" ] || { log "候補の重みが無い"; exit 1; }
[ -f "$OUT" ] || printf 'name\tck\tseed\trate\tsuccess\trecord\tmore\n' > "$OUT"

# 評価 1 本。$1 名前・$2 重み・$3 種・残りは足す引数。成功率を RATE に、記録の置き場を REC に
ev() {
  local name="$1" ck="$2" seed="$3"; shift 3
  local evlog="logs_s2e_eval_1110_${name}.log"
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS ++seed=$seed $*" ./eval_from_start.sh "$ck" 256 > "$evlog" 2>&1
  REC=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  RATE=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$REC/summary.json" 2>/dev/null || echo NA)
  local succ
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$REC/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$ck" "$seed" "$RATE" "$succ" "$REC" "$*" >> "$OUT"
  log "$name success=$succ rate=$RATE record=$REC more=$*"
}

ev E1_r13 "$CK13" 999; R13=$RATE
ev E2_r19 "$CK19" 999; R19=$RATE
BEST="$CK13"; BEST_NAME=r13
"$PY" -c 'import sys; sys.exit(0 if float(sys.argv[2]) > float(sys.argv[1]) else 1)' "$R13" "$R19" 2>/dev/null && { BEST="$CK19"; BEST_NAME=r19; }
log "良い方 $BEST_NAME（r13 $R13・r19 $R19）ck=$BEST"
ev "E3_${BEST_NAME}_noise" "$BEST" 999 ++env.config.door_sensor.loc_pos_bias_m=0.06 ++env.config.door_sensor.loc_yaw_bias_deg=2.0
ev "E4_${BEST_NAME}_left" "$BEST" 999 "++env.config.arm_stage.start_range=[-0.50,-0.37,0.01,0.11,-65.0,-25.0]"
ev "E5_${BEST_NAME}_dead" "$BEST" 999 "++env.config.loco_resp.dead=[0.12,0.12,0.0]"

# 動画: 失敗が 2 本そろうまで 8 台ずつ
FAILS=0
for seed in 999 1000 1001; do
  vlog="logs_s2e_eval_1110_video_${seed}.log"
  VIDEO=1 RENDER_W=960 RENDER_H=720 EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 \
    EXTRA_ARGS="$BASE_ARGS ++seed=$seed" ./eval_from_start.sh "$BEST" 8 > "$vlog" 2>&1
  VREC=$(grep -o 'out=[^ ]*' "$vlog" | tail -1 | cut -d= -f2)
  n=$(ls "$VREC"/video/env*_stage*.mp4 2>/dev/null | wc -l)
  FAILS=$((FAILS + n))
  log "動画 seed=$seed record=$VREC 失敗 $n 本（計 $FAILS）: $(ls "$VREC"/video 2>/dev/null | tr '\n' ' ')"
  [ "$FAILS" -ge 2 ] && break
done
log "終わり"
