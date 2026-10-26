#!/usr/bin/env bash
# 段 6 (a)（10-21）: GR00T N1.6 を OMEN（RTX 5060 Ti 16 GB・RAM 15 GB・共用機）で微調整できるかを試す。
#
# 合格の線（計画書 D11）: メモリ不足で落ちずに 1,000 ステップ回る・損失が下がる・推論が 10 Hz 以上
# 計画書の設定: 拡散ヘッドを凍結し projector だけ学習・bf16・1 枚に 1〜4・勾配を溜めて実効 32・データの先読みを減らす
#
#   STEPS=20 BS=2 TAG=probe ./gr00t_probe_1021.sh        まず短く回して GPU と RAM の山を測る
#   STEPS=1000 BS=2 ACC=16 TAG=try1000 ./gr00t_probe_1021.sh
#
# データは公式の G1 の例（unitree_g1.LMPnPAppleToPlateDC、0.38 GB）。始めの重みは G1 用の微調整済み（GR00T-N1.6-G1-PnPAppleToPlate、
# 埋め込み unitree_g1）。どちらも 3B で、メモリの使い方は扉のデータでも同じ。共用の ~/Isaac-GR00T のコードは触らず、出力は ~/work_inoue/gr00t_1021/
# ⚠️ per_device_train_batch_size = global_batch_size / num_gpus（勾配を溜める回数では割らない）。BS が 1 枚に載せる数
set -uo pipefail
W="$HOME/work_inoue/gr00t_1021"
cd "$HOME/Isaac-GR00T" || exit 1
STEPS="${STEPS:-20}"; BS="${BS:-2}"; ACC="${ACC:-1}"; TAG="${TAG:-probe}"
DIFF_FLAG="${DIFF_FLAG:---no-tune-diffusion-model}"
BASE="$(ls -d "$HOME"/.cache/huggingface/hub/models--nvidia--GR00T-N1.6-G1-PnPAppleToPlate/snapshots/*/ | head -1)"
DATA="$W/data/x_embodiment_sim/unitree_g1.LMPnPAppleToPlateDC"
mkdir -p "$W/logs" "$W/out"
NEED_MB="${NEED_MB:-12000}"
source "$HOME/work_inoue/doorman/gate.sh"
echo "=== [$(date +%T)] wait machine free（$TAG）"
wait_machine_free || { echo "=== [$(date +%T)] gate_timeout"; exit 1; }
# GPU のメモリ [MiB]・使用率 [%]・空き RAM [MB] を 1 秒ごとに残す（山を後で見る）
( while true; do echo "$(date +%T) $(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ') $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 1; done ) > "$W/logs/${TAG}_mon.log" &
MON=$!
# GPU の山が 15.7 GB と 16 GB にぎりぎり（10-21 の probe3）。長く回すと断片化で落ちうるので、断片化しにくい確保の仕方にする
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
echo "=== [$(date +%T)] start $TAG steps=$STEPS bs=$BS acc=$ACC $DIFF_FLAG"
setsid systemd-run --user --scope --quiet -p MemoryMax="${MEM_MAX:-13G}" -p MemorySwapMax=0 -p CPUWeight=20 \
  .venv/bin/python "$W/launch_finetune_omen.py" \
  --base-model-path "$BASE" --dataset-path "$DATA" --embodiment-tag UNITREE_G1 --num-gpus 1 \
  --output-dir "$W/out/$TAG" --max-steps "$STEPS" --global-batch-size "$BS" --gradient-accumulation-steps "$ACC" \
  $DIFF_FLAG --tune-projector --dataloader-num-workers 1 --save-steps "${SAVE_STEPS:-100000}" --save-total-limit 1 \
  --learning-rate 1e-4 --warmup-ratio 0.05 ${EXTRA:-}
rc=$?
kill "$MON" 2>/dev/null
echo "=== [$(date +%T)] $TAG done rc=$rc  GPU の山 $(awk '{split($2,a,","); if (a[1]>m) m=a[1]} END {print m}' "$W/logs/${TAG}_mon.log") MiB・空き RAM の底 $(awk 'NR==1{m=$3} {if ($3<m) m=$3} END {print m}' "$W/logs/${TAG}_mon.log") MB"
