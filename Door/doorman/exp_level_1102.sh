#!/usr/bin/env bash
# 11-02: 打ち切りの線（上半身の関節の速さの上限 = termination_level × 20 rad/s）を元の設計の 1.0（20 rad/s）に戻して評価する。
# 線は重みの env_state_dict に保存され、評価も学習の再開も設定より先にこれを読む（設定の initial_level では変わらない）。
# 重みの写しの env_state_dict.termination_level だけを書き換える（方策の重みはそのまま）。4・8・10 回目の 3 本
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid -f nohup ./exp_level_1102.sh >> logs_level_1102.log 2>&1 < /dev/null
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 札は自分の PAUSE_DOOR_F3（連鎖の学習もこれを見る）。gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない
# （11-02 14:25〜16:09 の評価は渡し忘れて、ユーザーの札を置いては外していた）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
LEVEL="${LEVEL:-1.0}"
for pair in r4:start_1102_1058 r8:start_1102_1314 r10:start_1102_1422; do
  name="${pair%%:*}"; src="ckpt/${pair#*:}"
  dst="ckpt/lvl${LEVEL}_${name}"
  mkdir -p "$dst" && cp "$src"/config.yaml "$src"/meta.yaml "$dst"/ || { log "写せない: $src"; continue; }
  "$PY" - "$src/last.pt" "$dst/last.pt" "$LEVEL" <<'PYEOF' || { log "書き換えに失敗: $src"; continue; }
import sys, torch
src, dst, level = sys.argv[1], sys.argv[2], float(sys.argv[3])
c = torch.load(src, map_location="cpu", weights_only=False)
before = float(c["env_state_dict"]["termination_level"])
c["env_state_dict"]["termination_level"] = torch.tensor(level)
torch.save(c, dst)
print(f"termination_level {before:.4f} -> {level}")
PYEOF
  log "評価 ${name} level=${LEVEL}"
  KEEP_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN=60 EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA" ./eval_from_start.sh "$HOME/work_inoue/doorman/$dst/last.pt" 256 > "logs_level_1102_${name}.log" 2>&1
  out=$(grep -o 'out=[^ ]*' "logs_level_1102_${name}.log" | tail -1 | cut -d= -f2)
  log "評価 ${name} level=${LEVEL} out=${out} $(grep -o 'success=[^ ]*' "logs_level_1102_${name}.log" | tail -1)"
  [ -n "$out" ] && [ -f "$out/summary.json" ] && nice "$PY" "$HOME/work_inoue/doorman/term_analysis.py" "$out"
done
./pause_training.sh off
log "level_1102 が終わった"
