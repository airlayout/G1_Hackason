#!/usr/bin/env bash
# V2（10-16）: 推定のモデルを学習する（door_vision_train.py）。共用機なので gate.sh で空きを待ってから GPU を使う。
# 使い方: OUT=v2model/run1 EPOCHS=8 ./exp_v2_train_1016.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
OUT="${OUT:-v2model/run1}"
NEED_MB="${NEED_MB:-6000}"
source ./gate.sh
echo "=== [$(date +%T)] wait machine free for v2 train"
wait_machine_free || { echo "=== [$(date +%T)] gate_timeout"; exit 1; }
echo "=== [$(date +%T)] start v2 train out=$OUT"
setsid systemd-run --user --scope --quiet -p MemoryMax="${MEM_MAX:-9G}" -p MemorySwapMax=0 -p CPUWeight=20 \
  ~/miniforge3/envs/doorman6/bin/python door_vision_train.py \
  --train v2data/pitch32 v2data/pitch38 v2data/pitch44 v2data/pitch50 --test v2data/pitch41 \
  --out "$OUT" --epochs "${EPOCHS:-8}" ${EXTRA:-}
echo "=== [$(date +%T)] v2 train done rc=$?"
