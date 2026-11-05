#!/usr/bin/env bash
# 11-05（ユーザー「PC2 の電源を入れたら、すぐに実機検証に移れるように準備しておいて」）: カラーの画角で学び直した DAgger（d2）の
# 評価（pipeline_vis2_1105.sh の ② 頷き 4°）が終わったら、その重みを実機用に書き出す（Door/real/export_vision_student.py）。
# 出力: ~/work_inoue/doorman6/vis_export/door_vision_d2.npz・_encoder.npz・_encoder.onnx・_meta.json（Mac へは scp で持ってくる）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash export_after_d2_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
LOG="$B/vis_1104/pipeline.log"
log() { echo "=== [$(date '+%m-%d %T')] export_d2: $*"; }
log "d2 の評価の終わりを待つ"
until grep -aq "vis2: ② 頷き 4°" "$LOG"; do
  grep -aq "vis2: .*\(落ちた\|無い\|作れない\)" "$LOG" && { log "連鎖が途中で止まったので書き出さない"; exit 1; }
  sleep 20
done
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_d2/*/*/last.pt | head -1)
IMG=$(ls "$B"/vis_1104/ds2/s19/frames/shard_*.npz | head -1)
log "書き出す ck=$CK"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=3G -p CPUWeight=20 \
  env CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONPATH=".:$B/real_vis" \
  "$HOME/miniforge3/envs/doorman6/bin/python" "$B/real_vis/export_vision_student.py" --ckpt "$CK" --viral . \
  --images "$IMG" --out-dir "$B/vis_export" --name door_vision_d2 2>&1 | grep -a "\[export\]"
log "終わり: $(ls "$B"/vis_export/door_vision_d2* 2>/dev/null | tr '\n' ' ')"
