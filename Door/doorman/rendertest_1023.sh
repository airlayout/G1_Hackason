#!/usr/bin/env bash
# 10-23（ユーザー承認）: 描画の回数を制御 1 回に 1 回（render_interval 4）にしたときの速さと画を、今の 4 回（render_interval 1）と比べる。
# 4 周目の評価が終わって鎖が 5 周目の学習に入る（札 PAUSE_DOORMAN_TRAINING で待たされる）ところで、同じ重みから 16 台で
#   ri4: render_interval 4 で 9 分 → ri1: render_interval 1 で 7 分
# を回し、1 秒あたりの歩数と、描いた画のくっきり具合（輪郭の強さ＝ラプラシアンの分散の中央）を比べる（analyze_rendertest.py）。
# 1.3 倍以上速く、くっきり具合の差が ±15% 以内なら、s6va_extra_args.txt に ++simulator.config.sim.render_interval=4 を書く
# （exp_campitch_1021.sh が次の起動から読む＝5 周目の学習から効く）。最後に必ず札を外す。
# 試しが書いた重みのフォルダは、続きの学習が拾わないよう logs_rl/port6/smoke_ri_1023/ へ移す。
#
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash rendertest_1023.sh > tests/rendertest_1023.log 2>&1 < /dev/null &
set -uo pipefail
W6="$HOME/work_inoue/doorman6"
cd "$W6" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
PAUSE="$HOME/work_inoue/PAUSE_DOORMAN_TRAINING"
CHAIN="$W6/chain_s6va/1022_1744/chain.log"
RUNS="$W6/VIRAL6/logs_rl/port6/wbmanip"
OUT="$W6/tests/rendertest_1023"
mkdir -p "$OUT"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$OUT/rendertest.log"; }
trap 'rm -f "$PAUSE"; log "札を外した（5 周目の学習が再開する）"' EXIT

log "4 周目の評価が終わって、鎖が 5 周目の学習に入るのを待つ"
for _ in $(seq 1 720); do grep -q "round=4 学習の追加の引数" "$CHAIN" && break; sleep 30; done
grep -q "round=4 学習の追加の引数" "$CHAIN" || { log "6 時間待っても 5 周目に入らない → 試さずに終わる"; exit 1; }
sleep 60
if pgrep -f "^python .*(train|eval)_agent_trl" > /dev/null; then log "学習か評価が動いている → 試さずに終わる"; exit 1; fi
BASE_CK="$(ls -t "$RUNS"/door_open_homie_29dof_dagger-lstm_lstm-*/last.pt | head -1)"
log "出発の重み: $BASE_CK"

run() {  # $1=名前 $2=render_interval $3=分
  local name="$1" ri="$2" min="$3" newest
  log "試し ${name}（render_interval ${ri}・${min} 分）を始める"
  IGNORE_PAUSE=1 RESUME_CKPT="$BASE_CK" MINUTES="$min" \
    TRAIN_EXTRA="++env.config.enable_staged_reset=True ++simulator.config.sim.render_interval=$ri ++domain_rand.camera_pitch_debug_dir=$OUT/cam_$name ++domain_rand.camera_pitch_debug_every=40" \
    bash "$W6/exp_campitch_1021.sh" dagger > "$OUT/run_$name.log" 2>&1
  grep -o "log: [^ ]*" "$OUT/run_$name.log" | tail -1 | cut -d" " -f2 > "$OUT/trainlog_$name.txt"
  newest="$(ls -td "$RUNS"/door_open_homie_29dof_dagger-lstm_lstm-*/ | head -1)"
  newest="${newest%/}"
  if [ "$newest" != "$(dirname "$BASE_CK")" ]; then
    mkdir -p "$W6/VIRAL6/logs_rl/port6/smoke_ri_1023"
    mv "$newest" "$W6/VIRAL6/logs_rl/port6/smoke_ri_1023/" && log "試しの重みのフォルダを移した: $(basename "$newest")"
  fi
  log "試し $name 終わり（$(tail -1 "$OUT/run_$name.log" | cut -c1-120)）"
}
run ri4 4 9
run ri1 1 7

if "$PY" "$W6/analyze_rendertest.py" "$OUT" > "$OUT/result.txt" 2>&1; then
  cat "$OUT/result.txt" | tee -a "$OUT/rendertest.log"
  if grep -q "^判定: 入れる" "$OUT/result.txt"; then
    echo "++simulator.config.sim.render_interval=4" > "$W6/s6va_extra_args.txt"
    log "s6va_extra_args.txt に render_interval=4 を書いた（5 周目の学習から効く）"
  else
    log "線に届かないので、描画の設定はそのまま"
  fi
else
  cat "$OUT/result.txt" | tee -a "$OUT/rendertest.log"
  log "比べられなかったので、描画の設定はそのまま"
fi
[ "$(ls -t "$RUNS"/door_open_homie_29dof_dagger-lstm_lstm-*/last.pt | head -1)" = "$BASE_CK" ] \
  && log "最新の重みが出発の重みに戻っている" || log "⚠️ 最新の重みが出発の重みと違う（試しのフォルダが残っている？）"
