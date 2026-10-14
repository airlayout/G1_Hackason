#!/usr/bin/env bash
# 学習中のチェックポイントを「最初の区間から」評価し、成功率を数える（段 2-1 の合否）。
# 学習（run_train_segments.sh）を札で止めてから評価し、終わったら（失敗しても）札を外して学習を再開させる。
#
# 学習中の goal_reached は、回の半分を途中の区間から始める（staged_reset）ので合否に使えない。
# 評価も既定では途中から始めるので ++env.config.enable_staged_reset=False で切る。
# DoorMan の評価の goal_reached はドアの環境では数えていないので、記録から count_success.py で数える。
#
# 使い方（OMEN の ~/work_inoue/doorman で）:
#   ./eval_from_start.sh [チェックポイント（既定: 6.0.1 の学習の最新の last.pt）] [台数 128＝回数]
# 空くのを待つ上限は MAX_WAIT_MIN 分（既定 30）。IDLE_GATE=0 で画面の前の人の在席を見ない（既定 1＝離席するまで待つ。共用機なので 0 は人に確かめてから）
# 追加の hydra 引数は EXTRA_ARGS に空白区切りで（例: EXTRA_ARGS=++env.config.record_states.pregrasp_diag=True）
# 成功の定義は COUNT_ARGS で count_success.py に渡す（例: COUNT_ARGS="--criterion door30"＝段 2-2 の合格の線）
# VIDEO=1: Isaac Sim の描画で台ごとに mp4 を書く（DoorMan の render_results。カメラは扉の手前の右後ろに固定）。
#   描画はメモリを食うので台数は 8 程度にする（例: VIDEO=1 ./eval_from_start.sh "" 8）。動画は records/<日時>/video/ に
#   env<番号>_ok.mp4（通り抜けた）・env<番号>_stage<届いた区間>.mp4 の名前で置く
# 出力: records/start_<日時>/（記録・eval.log・summary.json）。最後の行は「=== ... eval_from_start success=a/b rate=r」
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
WORK6="$HOME/work_inoue/doorman6"
LOGROOT="$WORK6/VIRAL6/logs_rl/door29"
NUM_ENVS="${2:-128}"   # 各 env の最初の 1 回だけ数えるので、回数＝台数
PY="$HOME/miniforge3/envs/doorman6/bin/python"
cd "$BASE" || exit 1
log() { echo "=== [$(date +%T)] $*"; }

VIDEO_ARGS=""
if [ "${VIDEO:-0}" = 1 ]; then
  : "${MEM_MAX:=11G}"; export NEED_MARGIN_MB="${NEED_MARGIN_MB:-512}"  # 描画あり。共用機は RAM 15 GiB なので余裕は小さめ
  VIDEO_ARGS="++simulator.config.render_results=True ++simulator.config.render_width=640 ++simulator.config.render_height=480 ++simulator.config.render_eye=[-2.6,-1.4,1.7] ++simulator.config.render_target=[-0.2,-0.1,0.9]"
fi
SRC="${1:-$(find "$LOGROOT" -name last.pt -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)}"
[ -f "$SRC" ] || { log "checkpoint が無い: ${SRC:-（見つからない）}"; exit 2; }
TAG="start_$(date +%m%d_%H%M)"
OUT="$BASE/records/$TAG"
mkdir -p "$BASE/ckpt/$TAG" "$OUT"
cp "$SRC" "$BASE/ckpt/$TAG/last.pt"   # 学習が書き換えても評価の対象がずれないように写す
cp "$(dirname "$SRC")"/{config,meta}.yaml "$BASE/ckpt/$TAG/" || { log "config.yaml・meta.yaml が無い（評価は隣の設定を読む）"; exit 2; }
log "eval_from_start ckpt=$SRC copied=$BASE/ckpt/$TAG/last.pt n=$NUM_ENVS"

trap './pause_training.sh off >/dev/null; log "札を外した（学習は空きを確かめて再開する）"' EXIT
for _ in 1 2 3; do ./pause_training.sh on && break; done  # 走り始めて 5 分以内の区切りは止めるのが遅れる

WORK="$WORK6" REPO=VIRAL6 ENV_NAME=doorman6 IGNORE_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" MAX_WAIT_MIN="${MAX_WAIT_MIN:-30}" MEM_MAX="${MEM_MAX:-8G}" \
  ./run_eval_record.sh "$BASE/ckpt/$TAG/last.pt" "$OUT" "$NUM_ENVS" "$NUM_ENVS" "${MINUTES:-20}" \
  ++env.config.enable_staged_reset=False ++algo.config.eval.eval_num_envs_episodes=True ${EXTRA_ARGS:-} \
  ${VIDEO_ARGS:+$VIDEO_ARGS ++env.config.save_rendering_dir=$OUT/video}
"$PY" "$BASE/count_success.py" "$OUT" ${COUNT_ARGS:-} --json "$OUT/summary.json" | tee "$OUT/summary.txt"
RC=${PIPESTATUS[0]}
if [ -n "$VIDEO_ARGS" ] && [ -f "$OUT/summary.json" ]; then
  "$PY" - "$OUT" <<'PYEOF'
import glob, json, os, re, sys
out = sys.argv[1]
per_env = {e["env"]: e for e in json.load(open(f"{out}/summary.json"))["per_env"]}
for f in glob.glob(f"{out}/video/*_viewer_*.mp4"):
    i = int(re.search(r"_viewer_(\d+)\.mp4$", f).group(1))
    e = per_env.get(i)
    tag = "ok" if e and e["success"] else (f"stage{e['max_stage']}" if e else "unfinished")
    os.rename(f, f"{out}/video/env{i:03d}_{tag}.mp4")
print("videos:", " ".join(sorted(os.path.basename(f) for f in glob.glob(f"{out}/video/env*.mp4"))))
PYEOF
fi
SUCCESS=$(grep -o '成功[^:]*: [0-9]*/[0-9]* = [0-9.]*%' "$OUT/summary.txt" | sed 's/.*: //')
log "eval_from_start out=$OUT success=${SUCCESS:-none} rc=$RC"
exit "$RC"
