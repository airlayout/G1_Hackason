#!/usr/bin/env bash
# 実験 A（10-21）: V2 に扉の寸法の出力を足し、扉ごとに 5 つに分けて学習・検証する（door_vision_geom_train.py）。
# 実験 B（exp_geomnoise_1021.sh、Isaac Sim）と同時に回すと GPU とメモリ（15 GB）が足りないので、B の終わりを待ってから始める。
# 共用機なので gate.sh で空きを待ち、メモリの上限 9 GB の中で回す（exp_v2_train_1016.sh と同じ）。
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
OUT="${OUT:-v2model/geom_1021}"
DIRS="v2data/pitch32 v2data/pitch38 v2data/pitch41 v2data/pitch44 v2data/pitch50 records/v2dagger_1016/v2/vision"
WAIT_LOG="${WAIT_LOG:-logs/exp_geomnoise_1021.log}"
echo "=== [$(date +%T)] 実験 B の終わりを待つ（$WAIT_LOG）"
until grep -q "exp_geomnoise_1021 done" "$WAIT_LOG" 2>/dev/null; do sleep 60; done
NEED_MB="${NEED_MB:-6000}"
source ./gate.sh
for k in $(seq 0 $(( ${FOLDS:-5} - 1 ))); do
  echo "=== [$(date +%T)] wait machine free for fold $k"
  wait_machine_free || { echo "=== [$(date +%T)] gate_timeout"; exit 1; }
  echo "=== [$(date +%T)] start fold $k"
  setsid systemd-run --user --scope --quiet -p MemoryMax="${MEM_MAX:-9G}" -p MemorySwapMax=0 -p CPUWeight=20 \
    ~/miniforge3/envs/doorman6/bin/python door_vision_geom_train.py --dirs $DIRS \
    --fold "$k" --folds "${FOLDS:-5}" --out "$OUT/fold$k" --epochs "${EPOCHS:-8}" ${EXTRA:-}
  echo "=== [$(date +%T)] fold $k done rc=$?"
done
~/miniforge3/envs/doorman6/bin/python summarize_geom_folds.py "$OUT"
echo "=== [$(date +%T)] exp_geom_train_1021 done"
