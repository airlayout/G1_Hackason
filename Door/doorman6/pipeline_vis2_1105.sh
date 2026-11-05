#!/usr/bin/env bash
# 11-05 朝（ユーザー「カラーの画角に直して続ける」）: 11-04 の画像の生徒は赤外線の画角 87°×56° で学んでいた。
# カラーの 69°×42°（開口 2.6035×1.4621）で、画像集め → V2（画像の網と扉の頭）→ V3 DAgger（2 区切り）→ 評価 → V4 PPO（512 台）を続けて回す。
#   画像集め: 種 11〜19・256 台・1 種 12,500 枚（11-04 の ds1 と同じ扉）。DAgger は頭の頷き 0〜4° を最初から入れる
#   評価: 学習に無い扉（seed 999）・256 台で ① 生徒だけ ② 頷き 4°。PPO は keep3_vis_1105.sh（落ちたら 16 台ずつ減らす・今日 18:00 まで）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash pipeline_vis2_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
#   止め札: ~/work_inoue/STOP_DOOR_VIS_COLLECT（集める段）・~/work_inoue/STOP_DOOR_VIS（DAgger・PPO）・~/work_inoue/STOP_DOOR_VIS_ALL（PPO の見張り）
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TEACHER="$HOME/work_inoue/doorman/ckpt/start_1103_0807/last.pt"
DS="$V/ds2"
NOD=++env.config.vision_feat.nod_deg=4.0
log() { echo "=== [$(date '+%m-%d %T')] vis2: $*"; }

log "画像集め（カラーの画角）→ $DS"
SEEDS="11 12 13 14 15 16 17 18 19" NUM_ENVS=256 MINUTES=6 MAX_FRAMES=12500 MEM_MAX=12G \
  bash "$B/collect_seeds_1104.sh" "$DS" > "$V/ds2.out" 2>&1
tail -1 "$V/ds2.out"
DATA="$(ls -d "$DS"/s*/frames | tr '\n' ' ')"
ENC="$V/pretrain_color_$(date +%m%d_%H%M).pt"
log "V2 の学習 data=$DATA out=$ENC"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=13G -p MemorySwapMax=0 -p CPUWeight=20 \
  env PYTHONPATH="$B/VIRAL6_vis" PYTHONUNBUFFERED=1 "$PY" gr00t/rl/scripts/door_vision_pretrain.py \
  --data $DATA --out "$ENC" --epochs 8 --batch 256 < /dev/null
[ -f "$ENC" ] || { log "V2 の出力が無い（学習が落ちた）"; exit 1; }
log "V2 終わり $ENC"

cd "$B" || exit 1
log "V3 DAgger（2 区切り・頷き 0〜4°）"
ENC="$ENC" NUM_ENVS=256 ROUNDS=2 ROUND_MIN=30 SEED0=3000 EXTRA_ARGS="$NOD" bash "$B/exp_vis_1104.sh" d2
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_d2/*/*/last.pt 2>/dev/null | head -1)
[ -n "$CK" ] || { log "DAgger の重みが無い"; exit 1; }

TAG="$(date +%m%d_%H%M)"
log "評価 ck=$CK"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_d2" 256 > "$V/eval_${TAG}_d2.log" 2>&1
log "① 生徒: $(tail -1 "$V/eval_${TAG}_d2.log")"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_d2_nod4" 256 "$NOD" > "$V/eval_${TAG}_d2_nod4.log" 2>&1
log "② 頷き 4°: $(tail -1 "$V/eval_${TAG}_d2_nod4.log")"

PCK="$V/ppo_start_color_${TAG}.pt"
(cd "$B/VIRAL6_vis" && PYTHONPATH=. "$PY" gr00t/rl/scripts/make_vis_ppo_ckpt.py "$CK" "$TEACHER" "$PCK" --std-scale 0.5)
[ -f "$PCK" ] || { log "PPO の出発の重みが作れない"; exit 1; }
log "V4 PPO（512 台）を始める ck=$PCK"
CK0="$PCK" BASE_NAME=q NUM_ENVS=512 ENC="$ENC" bash "$B/keep3_vis_1105.sh"
