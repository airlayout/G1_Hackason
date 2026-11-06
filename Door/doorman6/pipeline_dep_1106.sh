#!/usr/bin/env bash
# 11-06（カラー＋深度、計画 docs/plan/2026-11-06-g1-door-depth-rl.md の D2・D4・D5）: 肩 roll を直した教師（f3t1）に扉を開けさせて
# 頭のカラーと深度を集め、網（DoorRGBDEncoder）と扉の頭を学び、DAgger で生徒を 2 段で学ぶ。
#   0 確かめ: 16 台・3 分だけ集めて、深度が記録に入り距離が扉に応えるかを見る（dep_check_1106.py）。だめなら止める
#   1 画像集め: 種 21〜32・256 台・1 種 7,000 枚（扉 3,072 枚）。深度は揺れを足す前の値を残す
#   2 網の先の学習: 8 エポック・カラーを落とす割合 0.4（確かめは「カラー＋深度」と「深度だけ」）
#   3 DAgger e1: 教師から・頷き 0〜4°・特徴の遅れ 2〜4 歩・カラーを落とす回 3 割・2 区切り（各 30 分）→ 評価 2 通り
#   4 DAgger e2: e1 から追う段（記憶 128）を足す（扉の頭を固める・隠れ状態を溜める）。止め札まで区切りを続ける（落ちたら起こし直す）
#   評価は学習に無い扉（seed 999・256 回）で「カラー＋深度（落とさない）」と「深度だけ（全部落とす）」。どちらも遅れあり
# 使い方（OMEN の ~/work_inoue/doorman6）: TEACHER=<f3t1 の評価済みの last.pt> setsid nohup bash pipeline_dep_1106.sh >> vis_1104/pipeline_dep.log 2>&1 < /dev/null &
#   止め札: ~/work_inoue/STOP_DOOR_VIS（区切りの境目）・~/work_inoue/STOP_DOOR_VIS_ALL（e2 の起こし直しもしない）・~/work_inoue/STOP_DOOR_VIS_COLLECT（集める段）
#   途中から: FROM=pretrain（集めた記録 DS を使う）・FROM=e1・FROM=e2（E1=<e1 の last.pt>・ENC=<網の .pt>）
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TEACHER="${TEACHER:?教師（肩 roll を直した f3t1）の last.pt を TEACHER に渡す}"
DS="${DS:-$V/ds_dep}"
FROM="${FROM:-check}"
STOP="$HOME/work_inoue/STOP_DOOR_VIS"
STOP_ALL="$HOME/work_inoue/STOP_DOOR_VIS_ALL"
CAM="[{rgb:true},{depth:true}]"
T1="++env.config.shoulder_roll_press_max_nm=6.0"
NOD=++env.config.vision_feat.nod_deg=4.0
NOD0=++env.config.vision_feat.nod_deg=0.0
DELAY="++env.config.vision_feat.delay_steps=[2,4]"
CAMS="++simulator.config.cameras.camera_types=$CAM"
COFF="++env.config.vision_feat.color_off_prob=0.3"
COFF_NONE="++env.config.vision_feat.color_off_prob=0.0"
COFF_ALL="++env.config.vision_feat.color_off_prob=1.0"
log() { echo "=== [$(date '+%m-%d %T')] dep: $*"; }
[ -f "$TEACHER" ] || { log "教師が無い: $TEACHER"; exit 1; }
grep -qF "DoorRGBDEncoder" "$B/VIRAL6_vis/gr00t/rl/envs/door/door_vision_feat.py" || { log "パッチ 0094 が VIRAL6_vis に無い"; exit 1; }
grep -qF "shoulder_roll_press_max_nm" "$B/VIRAL6_vis/gr00t/rl/envs/base_task/finger_primitive_base.py" || { log "パッチ 0093 が VIRAL6_vis に無い"; exit 1; }
rm -f "$STOP" "$STOP_ALL" "$HOME/work_inoue/STOP_DOOR_VIS_COLLECT"
log "始める from=$FROM teacher=$TEACHER ds=$DS"

collect() {  # collect <出力> <種の並び> <台数> <分> <枚数>
  CKPT="$TEACHER" CAM_TYPES="$CAM" SEEDS="$2" NUM_ENVS="$3" MINUTES="$4" MAX_FRAMES="$5" MEM_MAX=12G \
    bash "$B/collect_seeds_1104.sh" "$1" ++env.config.vision_dump.depth=True "$T1"
}

if [ "$FROM" = check ]; then
  log "0 確かめ（16 台・3 分）"
  collect "$V/ds_dep_check" "5" 16 3 1500 > "$V/ds_dep_check.out" 2>&1
  tail -1 "$V/ds_dep_check.out"
  "$PY" "$B/dep_check_1106.py" "$V/ds_dep_check" "$V/ds_dep_check.png" 2>&1 | tee "$V/ds_dep_check.txt"
  grep -q "測れた割合" "$V/ds_dep_check.txt" || { log "確かめで深度が読めない。止める"; exit 1; }
  FROM=collect
fi

if [ "$FROM" = collect ]; then
  log "1 画像集め → $DS"
  collect "$DS" "21 22 23 24 25 26 27 28 29 30 31 32" 256 6 7000 > "$V/ds_dep.out" 2>&1
  tail -1 "$V/ds_dep.out"
  FROM=pretrain
fi

if [ "$FROM" = pretrain ]; then
  DATA="$(ls -d "$DS"/s*/frames | tr '\n' ' ')"
  ENC="$V/pretrain_rgbd_$(date +%m%d_%H%M).pt"
  log "2 網の先の学習 data=$DATA out=$ENC"
  cd "$B/VIRAL6_vis" || exit 1
  XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=13G -p MemorySwapMax=0 -p CPUWeight=20 \
    env PYTHONPATH="$B/VIRAL6_vis" PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py \
    --data $DATA --out "$ENC" --epochs 8 --batch 256 --color_off 0.4 < /dev/null
  [ -f "$ENC" ] || { log "網の先の学習の出力が無い（落ちた）"; exit 1; }
  log "2 終わり $ENC"
  FROM=e1
fi
ENC="${ENC:?網の .pt を ENC に渡す}"
cd "$B" || exit 1

ev() {  # ev <名前> <重み> <引数...>
  local name="$1" ck="$2"
  shift 2
  bash "$B/eval_vis_1104.sh" "$ck" "$V/eval_${TAG}_$name" 256 "$@" > "$V/eval_${TAG}_$name.log" 2>&1
  log "評価 $name: $(tail -n 1 "$V/eval_${TAG}_$name.log")"
}

if [ "$FROM" = e1 ]; then
  log "3 DAgger e1（2 区切り・頷き・遅れ・カラーを落とす回 3 割）"
  TEACHER="$TEACHER" ENC="$ENC" NUM_ENVS=256 ROUNDS=2 ROUND_MIN=30 SEED0=4000 \
    EXTRA_ARGS="$NOD $DELAY $CAMS $COFF $T1" bash "$B/exp_vis_1104.sh" e1
  E1=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_e1/*/*/last.pt 2>/dev/null | head -1)
  [ -n "$E1" ] || { log "e1 の重みが無い"; exit 1; }
  TAG=$(date +%m%d_%H%M)
  ev e1_rgbd "$E1" "$DELAY" "$NOD0" "$COFF_NONE"
  ev e1_depthonly "$E1" "$DELAY" "$NOD0" "$COFF_ALL"
  FROM=e2
fi
E1="${E1:?e1 の last.pt を E1 に渡す}"

if [ "$FROM" = e2 ]; then
  [ -e "$STOP_ALL" ] && { log "止め札があるので e2 を始めない"; exit 0; }
  TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$E1"
  TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
  newest() { ls -t "$B"/VIRAL6_vis/logs_rl/vis_e2/*/*/last.pt 2>/dev/null | head -1; }
  log "4 DAgger e2（追う段 128・e1 から）"
  CK=""
  SEED0=4200
  for attempt in $(seq 0 "${RESTARTS:-6}"); do
    [ -e "$STOP_ALL" ] && break
    log "e2 連鎖 $attempt ck=${CK:-なし（e1 から）} seed0=$SEED0"
    TEACHER="$TEACHER" ENC="$ENC" CK="$CK" NUM_ENVS=256 ROUNDS=48 ROUND_MIN=30 SEED0="$SEED0" \
      EXTRA_ARGS="$NOD $DELAY $CAMS $COFF $T1 $TRK" bash "$B/exp_vis_1104.sh" e2
    [ -e "$STOP_ALL" ] && break
    NEW=$(newest)
    if [ -z "$NEW" ] || [ "$NEW" = "$CK" ]; then
      log "e2 連鎖 $attempt が新しい重みを残さずに終わった（設定の誤りの疑い）。起こし直さない"
      break
    fi
    CK="$NEW"
    SEED0=$(( SEED0 + 20 ))
    log "e2 連鎖 $attempt が止め札の前に終わった。最新の重みから起こし直す"
  done
  touch "$STOP_ALL"
  E2=$(newest)
  [ -n "$E2" ] || { log "e2 の重みが無い"; exit 1; }
  TAG=$(date +%m%d_%H%M)
  ev e2_rgbd "$E2" "$DELAY" "$NOD0" "$COFF_NONE"
  ev e2_depthonly "$E2" "$DELAY" "$NOD0" "$COFF_ALL"
fi
log "終わり"
