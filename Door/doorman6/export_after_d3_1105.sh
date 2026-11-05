#!/usr/bin/env bash
# 11-05 夕（ユーザー「G1 が起動しているので並行して準備できそうな箇所があれば実施しておいて」）: 追う段つきの DAgger（d3）の
# 評価（run_d3_1105.sh の最後の 2 回）が終わったら、その重みを実機用に書き出す（Door/real/export_vision_student.py）。
# 書き出しは成功の高さによらずする（ファイルを作るだけ）。実機に載せるかは d2 の 83.6%・82.4% と比べて決める。
# 出力: ~/work_inoue/doorman6/vis_export/door_vision_d3.npz・_encoder.npz・_encoder.onnx・_meta.json（Mac へは scp で持ってくる）
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash export_after_d3_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
LOG="$B/vis_1104/pipeline.log"
log() { echo "=== [$(date '+%m-%d %T')] export_d3: $*"; }
log "d3 の評価の終わりを待つ"
until grep -aq "d3: 終わり" "$LOG"; do
  grep -aq "d3: \(重みが無い\|d2 の重みか ENC が無い\)" "$LOG" && { log "d3 が途中で止まったので書き出さない"; exit 1; }
  sleep 20
done
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_d3/*/*/last.pt | head -1)
IMG=$(ls "$B"/vis_1104/ds2/s19/frames/shard_*.npz | head -1)
log "書き出す ck=$CK"
cd "$B/VIRAL6_vis" || exit 1
XDG_RUNTIME_DIR="/run/user/$(id -u)" systemd-run --user --scope --quiet -p MemoryMax=3G -p CPUWeight=20 \
  env CUDA_VISIBLE_DEVICES= OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONPATH=".:$B/real_vis" \
  "$HOME/miniforge3/envs/doorman6/bin/python" "$B/real_vis/export_vision_student.py" --ckpt "$CK" --viral . \
  --images "$IMG" --out-dir "$B/vis_export" --name door_vision_d3 2>&1 | grep -a "\[export\]"
log "終わり: $(ls "$B"/vis_export/door_vision_d3* 2>/dev/null | tr '\n' ' ')"
