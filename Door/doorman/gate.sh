#!/usr/bin/env bash
# run_bench.sh・run_eval_record.sh・run_train_segments.sh が source する。OMEN（共用機）で GPU を使ってよいかの判定。
#
# 始めてよい条件（wait_machine_free）: 10 秒おきに 12 回（2 分）続けて
#   - 自分以外の GPU の使用が OTHERS_GPU_MB 以下（既定 2048 MiB。画面の描画で数百 MiB は常に使われている）
#   - 空きメモリ > NEED_MB
#   - IDLE_GATE=1 のときは、画面（seat0）の前の人が離席している（GNOME の IdleHint か画面ロック）
# 待ちの上限は MAX_WAIT_MIN 分（既定 720）。超えたら 1 を返す。
#
# 2026-10-06 のレビューの指摘: nvidia-smi --query-compute-apps は描画のプロセスを数えない。
# そこで nvidia-smi の表（Type C と G の両方）から、自分のプロセス以外の使用量を足し合わせる。

OTHERS_GPU_MB="${OTHERS_GPU_MB:-2048}"
MAX_WAIT_MIN="${MAX_WAIT_MIN:-720}"
# 一時停止の札。このファイルがある間は始めず、走っていれば止めて譲る（移植の確認などで GPU を使うとき。pause_training.sh）
PAUSE_FILE="${PAUSE_FILE:-$HOME/work_inoue/PAUSE_DOORMAN_TRAINING}"

# 画面（seat0）の前の人が離席しているか。画面のセッションが無ければ離席扱い
desk_idle() {
  local s
  s=$(loginctl list-sessions --no-legend 2>/dev/null | awk '$4=="seat0"{print $1}' | while read -r id; do
        [ "$(loginctl show-session "$id" -p Class --value 2>/dev/null)" = user ] && echo "$id"; done | head -1)
  [ -z "$s" ] && return 0
  loginctl show-session "$s" -p IdleHint -p LockedHint 2>/dev/null | grep -q "=yes"
}

# 自分（引数の pgid のプロセス）以外が GPU で使っているメモリ [MiB]。nvidia-smi が失敗したら 999999（使えない扱い）
others_gpu_mb() {
  local mine="${1:-}" pids table
  table=$(nvidia-smi 2>/dev/null) || { echo 999999; return; }
  pids=""
  [ -n "$mine" ] && pids=$(ps -o pid= -g "$mine" 2>/dev/null | tr -d ' ' | tr '\n' ' ')
  echo "$table" | awk -v mine=" $pids " '
    $1=="|" && ($6=="C" || $6=="G" || $6=="C+G") && $NF=="|" {
      pid=$5; mem=$(NF-1); sub(/MiB/, "", mem);
      if (index(mine, " " pid " ") == 0) s += mem
    }
    END { print s+0 }'
}

mem_avail_mb() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }

wait_machine_free() {
  local ok=0 waited=0
  while [ "$ok" -lt 12 ]; do
    local others avail idle_ok=1
    others=$(others_gpu_mb)
    avail=$(mem_avail_mb)
    if [ "${IDLE_GATE:-0}" = 1 ] && ! desk_idle; then idle_ok=0; fi
    [ -e "$PAUSE_FILE" ] && [ "${IGNORE_PAUSE:-0}" != 1 ] && idle_ok=0  # 評価・確認は IGNORE_PAUSE=1 で札を無視する
    if [ "$others" -le "$OTHERS_GPU_MB" ] && [ "$avail" -gt "$NEED_MB" ] && [ "$idle_ok" -eq 1 ]; then
      ok=$((ok+1))
    else
      ok=0
    fi
    sleep 10
    waited=$((waited+10))
    if [ "$waited" -ge $((MAX_WAIT_MIN*60)) ]; then
      echo "=== [$(date +%T)] gate: ${MAX_WAIT_MIN} 分待っても空かなかった（others_gpu=${others}MiB avail=${avail}MB idle_ok=${idle_ok}）"
      return 1
    fi
  done
  return 0
}

# 走行中に譲るべきか（譲る理由を出して 0 を返す）。引数は自分の学習の pgid
should_yield() {
  local mine="$1" others
  if [ -e "$PAUSE_FILE" ]; then echo "paused_by_flag"; return 0; fi
  if [ "${IDLE_GATE:-0}" = 1 ] && ! desk_idle; then echo "person_returned"; return 0; fi
  others=$(others_gpu_mb "$mine")
  if [ "$others" -gt "$OTHERS_GPU_MB" ]; then echo "others_gpu_${others}MiB"; return 0; fi
  return 1
}

# 背景で起動したプロセスは SIGINT を無視したまま始まる（非対話の bash の仕様。exec でも引き継ぐ）。
# 既定に戻してから exec する。Python はこれで SIGINT を KeyboardInterrupt として受ける
SIGINT_DEFAULT_EXEC=(python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')
