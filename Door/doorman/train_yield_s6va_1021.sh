#!/usr/bin/env bash
# 段 6 (b)（10-21）: 映像の生徒の DAgger を、共用機の他の人に譲りながら、決めた時間だけ学習する（OMEN、~/work_inoue/doorman6）。
#
# 学習の間、30 秒ごとに gate.sh の should_yield で「譲るべきか」を見る:
#   画面（seat0）の前に人がいる（GNOME のセッションが離席でもロックでもない）・他の人の GPU の使用が 2 GiB を超える・札（PAUSE_FILE）がある
# 譲るときは学習の python に SIGINT を送り（90 秒で止まらなければ自分のプロセスの木だけ強制的に止める）、
# 空くのを待って（NOT_BEFORE の時刻より後に gate.sh の wait_machine_free が通ったら）最新の last.pt から続きを学習する。
# last.pt は 100 回（約 8 分）ごとに書かれるので、譲るたびに失うのは最大その分。学習した時間の合計が <合計の分> に届いたら終わる。
#
# 使い方:
#   bash train_yield_s6va_1021.sh <合計の分> [始めの last.pt]
#   ADOPT=1 ADOPT_STARTED="2026-10-21 17:06:20" … すでに動いている exp_campitch_1021.sh dagger を引き取って見張る（始めた時刻から数える）
#   NOT_BEFORE="2026-10-21 22:00" … 譲った後、この日時までは再開しない（10-21 のユーザー指示：他の人は遅くとも 22 時に使い終わる）
#     ⚠️ 時刻だけ（22:00）にすると、日付が変わってから譲ったときに次の日の 22 時まで待つので、日時で書く
# 終了コード: 合計に届けば 0。学習が続けて 2 回すぐに落ちた・空きを待ちきれなかったら 1
set -uo pipefail
W6="$HOME/work_inoue/doorman6"
cd "$W6" || exit 1
source "$HOME/work_inoue/doorman/gate.sh"
export IDLE_GATE=1 NEED_MB="${NEED_MB:-11000}" MAX_WAIT_MIN="${MAX_WAIT_MIN:-900}"
TOTAL_MIN="${1:?合計の分}"
FIRST_CKPT="${2:-}"
RUNS="$W6/VIRAL6/logs_rl/port6/wbmanip"
LOGDIR="$W6/train_yield/$(date +%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$LOGDIR/yield.log"; }
kill_tree() { local c; for c in $(pgrep -P "$1"); do kill_tree "$c"; done; kill -9 "$1" 2>/dev/null; }
latest_ckpt() { ls -t "$RUNS"/door_open_homie_29dof_dagger-lstm_lstm-*/last.pt 2>/dev/null | head -1; }
# 自分の学習だけを指す（俯角の乱択の引数は自分の学習にしか無い。timeout にも同じ文字列があるので先頭が python のものに絞る）
train_pid() { pgrep -f "^python .*train_agent_trl\.py .*camera_pitch_range_deg" | head -1; }
runner_pid() { pgrep -f "^bash .*exp_campitch_1021\.sh dagger" | head -1; }

wait_not_before() {
  [ -n "${NOT_BEFORE:-}" ] || return 0
  local nb now
  nb=$(date -d "$NOT_BEFORE" +%s) || return 0
  now=$(date +%s)
  [ "$now" -lt "$nb" ] || return 0
  log "${NOT_BEFORE} まで再開しない（あと $(( (nb - now) / 60 )) 分）"
  sleep $(( nb - now ))
}

stop_training() {
  local tp rp
  tp="$(train_pid)"; rp="$(runner_pid)"
  [ -n "$tp" ] && kill -INT "$tp" 2>/dev/null
  for _ in $(seq 1 18); do [ -z "$(train_pid)" ] && break; sleep 5; done
  [ -n "$(train_pid)" ] && log "SIGINT で 90 秒たっても止まらない → 自分のプロセスの木だけ強制的に止める"
  [ -n "$rp" ] && kill_tree "$rp"
  [ -n "$(train_pid)" ] && kill_tree "$(train_pid)"
  sleep 10
  log "止めた（GPU $(nvidia-smi --query-gpu=memory.used --format=csv,noheader | tr -d ' ')・最新の last.pt $(latest_ckpt)）"
}

done_sec=0
quick_fail=0
first=1
log "start total=${TOTAL_MIN} 分 adopt=${ADOPT:-0} not_before=${NOT_BEFORE:-なし} first_ckpt=${FIRST_CKPT:-最新}"
while [ "$done_sec" -lt $(( TOTAL_MIN * 60 )) ]; do
  if [ "$first" = 1 ] && [ "${ADOPT:-0}" = 1 ] && [ -n "$(runner_pid)" ]; then
    start=$(date -d "${ADOPT_STARTED:?ADOPT_STARTED に始めた時刻}" +%s)
    log "動いている学習を引き取る（runner $(runner_pid)・python $(train_pid)・始めた時刻 ${ADOPT_STARTED}）"
  else
    rem_min=$(( (TOTAL_MIN * 60 - done_sec + 59) / 60 ))
    if [ "$rem_min" -lt 15 ]; then log "残り ${rem_min} 分は短いので回さない"; break; fi
    [ "$first" = 1 ] || wait_not_before
    log "空くのを待つ（画面の前の人・他の人の GPU・空きメモリ ${NEED_MB} MB）"
    wait_machine_free || { log "空きを ${MAX_WAIT_MIN} 分待ちきれなかった → 止める"; exit 1; }
    ck="$(latest_ckpt)"
    [ "$first" = 1 ] && [ -n "$FIRST_CKPT" ] && ck="$FIRST_CKPT"
    log "学習を始める（残り ${rem_min} 分・続きの重み ${ck:-なし}）"
    start=$(date +%s)
    RESUME_CKPT="$ck" MINUTES="$rem_min" setsid bash "$W6/exp_campitch_1021.sh" dagger \
      > "$LOGDIR/run_$(date +%m%d_%H%M).log" 2>&1 < /dev/null &
    sleep 30
  fi
  first=0
  yielded=""
  while [ -n "$(runner_pid)" ]; do
    tp="$(train_pid)"
    if [ -n "$tp" ]; then
      sid=$(ps -o sid= -p "$tp" 2>/dev/null | tr -d ' ')
      if [ -n "$sid" ] && reason=$(should_yield "$sid"); then yielded="$reason"; break; fi
    fi
    sleep 30
  done
  ran=$(( $(date +%s) - start ))
  done_sec=$(( done_sec + ran ))
  if [ -n "$yielded" ]; then
    log "譲る（${yielded}）。この回 $(( ran / 60 )) 分・合計 $(( done_sec / 60 ))/${TOTAL_MIN} 分"
    stop_training
    continue
  fi
  log "学習が終わった。この回 $(( ran / 60 )) 分・合計 $(( done_sec / 60 ))/${TOTAL_MIN} 分"
  if [ "$ran" -lt 900 ]; then
    quick_fail=$(( quick_fail + 1 ))
    log "15 分もたずに終わった（${quick_fail} 回目。$LOGDIR/run_*.log を見る）"
    [ "$quick_fail" -lt 2 ] || { log "続けて 2 回すぐに落ちた → 止める"; exit 1; }
  else
    quick_fail=0
  fi
done
log "done 合計 $(( done_sec / 60 )) 分 学習した・最新の last.pt $(latest_ckpt)"
