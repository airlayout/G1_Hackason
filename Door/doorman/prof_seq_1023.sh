#!/usr/bin/env bash
# 10-23（ユーザー指示「1, 2, 3 項を検証して」）: 映像の生徒の学習の速さを上げる手を、prof_1023.sh で 1 つずつ測る。
#   base2 : 今の設定（比べる基準。10-23 の base は経過時間の記録が無いので測り直す）
#   ri4up : 項目 3。描画を制御 1 回に 1 回（render_interval 4）＋カメラの更新も制御 1 回に 1 回（update_period 0.02）
#   enc16 : 項目 1。学習の段で詰め物の画像を ResNet に通さない＋ResNet を bf16 で（16 台）
#   enc32 : 項目 1。同じ設定で 32 台（メモリに入るか・速さが台数に比例して伸びるか）
#   gcfz16: 項目 2 で見つけた手。学習の前に gc.freeze（毎回の gc.collect が時間の約 16%）
#   all32 : 効いた手を全部重ねた候補（32 台・詰め物を通さない・bf16・gc.freeze）
# 試し用の写し（VIRAL6_speed）で回すので、止めている学習の重みとは混ざらない。最後に compare_prof.py で並べる。
#
# 使い方（OMEN の ~/work_inoue/doorman6）: setsid nohup bash prof_seq_1023.sh > /dev/null 2>&1 < /dev/null &
#   経過は tests/prof_seq_1023.log、比べた表は tests/prof_seq_1023_result.txt
set -uo pipefail
B="$HOME/work_inoue/doorman6"
cd "$B" || exit 1
L="$B/tests/prof_seq_1023.log"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$L"; }
CK16="$HOME/work_inoue/doorman/ckpt/s6va_r4/last.pt"
CK32="$B/tmp/ck_s6va_r4_n32.pt"
ENC=(++algo.config.vision_encode_valid_only=True ++algo.config.vision_bf16=True)
export TOTAL="${TOTAL:-480}" SEC="${SEC:-440}"
DIRS=()

run() {  # $1=名前 $2=台数 $3=出発の重み 残り=足す hydra の引数
  local tag="$1" n="$2" ck="$3" d
  shift 3
  log "${tag}（${n} 台）を始める: $*"
  TAG="$tag" NUM_ENVS="$n" CK="$ck" bash "$B/prof_1023.sh" "$@" >> "$L" 2>&1
  d="$(ls -td "$B"/tests/prof_"${tag}"_* 2>/dev/null | head -1)"
  [ -n "$d" ] && DIRS+=("$d")
  log "${tag} 終わり（$(grep -ac "Learning iteration" "$d/train.log" 2>/dev/null) 回学習した・$d）"
}

run base2 16 "$CK16"
run ri4up 16 "$CK16" ++simulator.config.sim.render_interval=4 ++simulator.config.cameras.camera_update_period=0.02
run enc16 16 "$CK16" "${ENC[@]}"
run enc32 32 "$CK32" "${ENC[@]}"
run gcfz16 16 "$CK16" ++algo.config.gc_freeze=True
run all32 32 "$CK32" "${ENC[@]}" ++algo.config.gc_freeze=True

"$HOME/miniforge3/envs/doorman6/bin/python" "$B/compare_prof.py" "$B/VIRAL6_speed" "${DIRS[@]}" > "$B/tests/prof_seq_1023_result.txt" 2>&1
log "全部終わり（表: tests/prof_seq_1023_result.txt）"
