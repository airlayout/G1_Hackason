#!/usr/bin/env bash
# **PC2 の上で**動かす。段 6 (b) 映像の生徒の推論の速さを、CPU の糸の数を変えて測る（機体は動かさない。DDS も触らない）。
# TensorRT の trtexec があれば、画像の部分（student_encoder.onnx）を Orin の GPU でも測る（fp16・fp32）。
# golden の画像を入れて出た画像の特徴も書き出し、summary で golden と比べる（fp16 の精度落ちを見る）。
#
#   bash bench_vision_student.sh [書き出しの場所（既定 ~/door_real/stage6_va/export_rand）]
#   THREADS="1 4 8" ITERS=200 TRT=0 bash bench_vision_student.sh ...   # TRT=0 で GPU を測らない
#   BENCH_TAG=<名前>  で出力先の名前を決める（deploy_vision_bench.sh が渡す。無ければ時刻）
#
# 出力: <書き出し>/bench_<名前>/（env.txt・t<糸>.json・t<糸>.log・trt_*.log・trt_*_out.json・summary.txt）
# 終了コード: どれかの計測が失敗したか golden が合わなければ 1
# ⚠️ 速さは電源の設定（nvpmodel）と他の処理の重さで変わる。env.txt に両方を残す。測る間は記録や Nav2 を止めておく
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIR="${1:-$HOME/door_real/stage6_va/export_rand}"
PY="${PY:-$HOME/miniforge3/envs/lerobot/bin/python}"
OUT="$DIR/bench_${BENCH_TAG:-$(date +%m%d_%H%M%S)}"
say() { echo "[bench] $*"; }
fail=0

[ -f "$DIR/student_full.ts" ] || { say "書き出しが無い: $DIR"; exit 1; }
[ -x "$PY" ] || { say "Python が無い: $PY"; exit 1; }
mkdir -p "$OUT"
{
  echo "date: $(date -Is)"; echo "host: $(hostname)  $(uname -srm)"; echo "nproc: $(nproc)"
  echo "online CPU: $(cat /sys/devices/system/cpu/online 2>/dev/null)"
  echo "uptime: $(uptime)"; free -m
  nv="$(nvpmodel -q 2>/dev/null | tr '\n' ' ')"
  echo "nvpmodel: ${nv:-取れない（無いか root が要る）}"
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
  for c in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq; do printf '%s ' "$(cat "$c" 2>/dev/null)"; done; echo "(kHz)"
  echo "python: $("$PY" -c 'import sys, torch; print(sys.version.split()[0], torch.__version__)' 2>&1 | tail -1)"
  echo "cv2: $("$PY" -c 'import cv2; print(cv2.__version__)' 2>&1 | tail -1)"
  echo "重い処理（CPU の上位 8）:"; ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -9
} > "$OUT/env.txt" 2>&1
say "環境: $(grep -E '^(host|online CPU|nvpmodel|python):' "$OUT/env.txt" | tr '\n' ' ')"

for n in ${THREADS:-1 2 4 8}; do
  OMP_NUM_THREADS="$n" OPENBLAS_NUM_THREADS="$n" "$PY" "$HERE/bench_vision_student.py" \
    --dir "$DIR" --threads "$n" --iters "${ITERS:-200}" --paced --out "$OUT/t$n.json" > "$OUT/t$n.log" 2>&1
  rc=$?
  grep "^\[bench\]" "$OUT/t$n.log"
  if [ "$rc" != 0 ]; then
    say "⚠️ 糸 ${n} で失敗（終了コード ${rc}）。ログの最後:"; tail -3 "$OUT/t$n.log" | sed 's/^/    /'; fail=1
  fi
done

TRTEXEC="$(command -v trtexec || ls /usr/src/tensorrt/bin/trtexec 2>/dev/null || true)"
if [ "${TRT:-1}" = 1 ] && [ -n "$TRTEXEC" ]; then
  for prec in fp16 fp32; do
    flag=""; [ "$prec" = fp16 ] && flag="--fp16"
    say "TensorRT ${prec}（エンジンを作るので 1〜3 分かかる）"
    if ! "$TRTEXEC" --onnx="$DIR/student_encoder.onnx" $flag --warmUp=1000 --duration=10 --percentile=99 \
        --loadInputs=image:"$DIR/golden_image0.bin" --exportOutput="$OUT/trt_${prec}_out.json" \
        --saveEngine="$OUT/encoder_$prec.engine" > "$OUT/trt_$prec.log" 2>&1; then
      say "⚠️ trtexec ${prec} が失敗（$OUT/trt_$prec.log の最後）"; tail -3 "$OUT/trt_$prec.log" | sed 's/^/    /'; fail=1
    fi
  done
elif [ "${TRT:-1}" = 1 ]; then
  say "trtexec が無いので GPU は測らない（dpkg -l | grep -i tensorrt で入っているか確かめる）"
fi

"$PY" "$HERE/bench_vision_student.py" --summary "$OUT" > "$OUT/summary.txt" 2>&1 || fail=1
cat "$OUT/summary.txt"
say "結果: $OUT"
exit "$fail"
