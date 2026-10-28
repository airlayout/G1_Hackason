#!/usr/bin/env bash
# 段 A・B の結果（作業ログ 2026-10-27 の 13 章）を受けた、候補 1〜3 を 1 つずつ足す試し（10-28 ユーザー「1〜3 項を実施して」）。
# どの段も同じ最良の重み（反復 53,750・77.0%）から始めるので、隣どうしの段の差が、足した直しの効き目になる。
#
#   段 C（f3d）: 雑音の上限＋候補 1「歩き去りの罰を早く・強く」（1.2 m → WALK_START 既定 1.0 m、重み −12 → WALK_W 既定 −40）
#   段 D（f3e）: 段 C＋候補 3「押し開けの持ち時間を評価と同じに」（学習の段階 4 の持ち時間 300 → SWING_STEPS 既定 600 歩）
#   段 E（f3f）: 段 D＋候補 2「押し開けの間は扉の先の目標へ近づく報酬を止める」（パッチ 0080、段階 4 の倍率 0.5 → 0）
#
# 回数（10-28 昼 ユーザー「おすすめで進めて」）: 段 C・D は MIN_ROUNDS 回（既定 3）回したところで判定する。
#   その段の全部の回の平均の成功率が、比べる相手から DECIDE_PT（既定 5.0）ポイント以上離れていれば次の段へ。
#   離れていなければ、最大の ROUNDS_C・ROUNDS_D 回（既定 6）まで、その段の最後の重みから続ける。
#   比べる相手は、段 C は REF_C（既定 74.7＝512 台の段 C の 4 回の平均）、段 D は段 C の全部の回の平均。
#   段 E は判定せず ROUNDS_E 回（既定 9999＝止めるまで）。終わったら f3_keepalive_1026.sh が段 E の続きを始め直す。
#   回は段の名前ごとに chain/*/results.tsv から数えるので、止めて組み直しても前の回が数に入る（足りない回だけを前の最後の重みから回す）。
#   FROM_PHASE（C・D・E、既定 C）で、その段から始める（前の段は回さず、回の数え直しにだけ使う）。
#
# 台数（10-28 11 時〜）: NUM_ENVS（既定 512）と MEM_MAX（既定 9G）は stage_chain.sh・run_train_segments.sh・見張りがそのまま受け継ぐ。
#   評価は台数に依らず 256 回（eval_from_start.sh）。台数を変えた段どうしは、学習の量（30 分で集める歩数）も変わる
#
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く（今の回の評価まで済ませて止まり、次の段も見張りも始めない）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_f3_phases_1028.sh >> logs_f3_phases_1028.log 2>&1 < /dev/null &
#   台数を変えるとき: NUM_ENVS=4096 MEM_MAX=12G setsid nohup ./exp_f3_phases_1028.sh …
#   4,096 台は GPU の余りが 1 GB を切る。10-28 23 時に 11 回回した後、PyTorch の断片化（確保したまま使っていない 1.25 GiB）で
#   始めて 3〜4 分の CUDA out of memory が 2 回続いた。PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True を前に足して起こす
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
BEST="${BEST:-$HOME/work_inoue/doorman/ckpt/f3_best_it53750/last.pt}"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
MIN_ROUNDS="${MIN_ROUNDS:-3}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

CAP="++algo.config.max_noise_std=${NOISE_MAX:-0.26}"
WALK="++env.config.walk_away_start_m=${WALK_START:-1.0},++rewards.reward_scales.walk_away_in_swing=${WALK_W:--40.0}"
# 段の持ち時間の並びの中のカンマは「|」で書く（stage_chain.sh がカンマに戻す）
LONG="++env.config.max_stage_time=[250|100|100|100|${SWING_STEPS:-600}|200]"
NOPULL="++env.config.target_root_distance_swing_scale=0.0"
# 512 台でないときは段の名前に台数を付ける（f3d → f3dn4096。重みの置き場 logs_rl/door_<名前> と記録を 512 台の回と混ぜない）
SFX=""; [ "${NUM_ENVS:-512}" != 512 ] && SFX="n${NUM_ENVS}"
# 段;名前;最大の回数;足す引数（「;」で区切る。引数の側は「,」区切り）
PHASES=(
  "C;f3d${SFX};${ROUNDS_C:-6};$CAP,$WALK"
  "D;f3e${SFX};${ROUNDS_D:-6};$CAP,$WALK,$LONG"
  "E;f3f${SFX};${ROUNDS_E:-9999};$CAP,$WALK,$LONG,$NOPULL"
)

# 段の全部の回の成功率（%、1 行 1 回）。連鎖の置き場の名前の順に、results.tsv の 1 列目が段の名前の行を集める
phase_rates() { cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { printf "%.4f\n", $6 * 100 }'; }
phase_n() { phase_rates "$1" | wc -l | tr -d ' '; }
phase_mean() { phase_rates "$1" | awk '{ t += $1; n++ } END { if (n) printf "%.1f", t / n }'; }
# 段のいちばん新しい回の重み（まだ回が無ければ最良の重み）。続きは前の最後の重みから回すので、反復がいちばん大きい回を選ぶ
# （置き場の名前の並びは言語の設定で 1028_1200_01 が 1028_1200 より前に来ることがあるので、並びには頼らない）
phase_ck() {
  local ck
  ck=$(cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$1" '$1 == s { print $4 "\t" $7 }' | sort -n | tail -1 | cut -f2)
  echo "${ck:-$BEST}"
}
# 2 つの平均が DECIDE_PT ポイント以上離れているか
is_clear() { awk -v a="$1" -v b="$2" -v t="${DECIDE_PT:-5.0}" 'BEGIN { d = a - b; if (d < 0) d = -d; exit !(d >= t) }'; }

# 引数: 名前・回数・出発点・足す引数。止め札があれば、次の段も見張りも始めずに終わる
run_rounds() {
  log "段 $1 を $2 回回す ck=$3 n=${NUM_ENVS:-512} mem_max=${MEM_MAX:-9G} more=$4"
  # 最後の段の名前と引数は、見張り（f3_keepalive_1026.sh）が続きを始めるときに受け継ぐ
  export NAME="$1" TRAIN_MORE="$4"
  CK="$3" ROUNDS="$2" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので、次の段も見張りも始めずに終わる"; exit 0; fi
}

[ -f "$BEST" ] || { log "出発点の重みが無い: $BEST"; exit 1; }
STARTED=0; PREV=""
for P in "${PHASES[@]}"; do
  IFS=';' read -r PKEY PNAME PMAX PMORE <<< "$P"
  if [ "$STARTED" = 0 ] && [ "$PKEY" != "${FROM_PHASE:-C}" ]; then PREV="$PNAME"; continue; fi
  STARTED=1
  if [ "$PKEY" = E ]; then
    run_rounds "$PNAME" "$PMAX" "$(phase_ck "$PNAME")" "$PMORE"
    continue
  fi
  MIN=$(( MIN_ROUNDS < PMAX ? MIN_ROUNDS : PMAX ))
  N=$(phase_n "$PNAME")
  [ "$N" -lt "$MIN" ] && run_rounds "$PNAME" $(( MIN - N )) "$(phase_ck "$PNAME")" "$PMORE"
  N=$(phase_n "$PNAME"); MEAN=$(phase_mean "$PNAME")
  if [ "$PKEY" = C ]; then REF="${REF_C:-74.7}"; REF_TXT="${REF}%"; else REF=$(phase_mean "$PREV"); REF_TXT="${PREV} の ${REF}%"; fi
  DIFF=$(awk -v a="$MEAN" -v b="${REF:-0}" 'BEGIN { d = a - b; if (d < 0) d = -d; printf "%.1f", d }')
  if [ -n "$REF" ] && is_clear "$MEAN" "$REF"; then
    log "判定 ${PNAME}: ${N} 回の平均 ${MEAN}% 対 ${REF_TXT}（差 ${DIFF}）→ 次の段へ"
  elif [ "$N" -lt "$PMAX" ]; then
    log "判定 ${PNAME}: ${N} 回の平均 ${MEAN}% 対 ${REF_TXT:-比べる相手なし}（差 ${DIFF}）→ ${PMAX} 回まで続ける"
    run_rounds "$PNAME" $(( PMAX - N )) "$(phase_ck "$PNAME")" "$PMORE"
  else
    log "判定 ${PNAME}: ${N} 回の平均 ${MEAN}% 対 ${REF_TXT}（差 ${DIFF}）→ 最大の ${PMAX} 回に届いているので次の段へ"
  fi
  PREV="$PNAME"
done
[ "${NO_KEEPALIVE:-0}" = 1 ] && { log "NO_KEEPALIVE=1 なので見張りを始めずに終わる"; exit 0; }
log "最後の段の続きは見張り（f3_keepalive_1026.sh）に任せる（name=${NAME:-} more=${TRAIN_MORE:-}）"
exec ./f3_keepalive_1026.sh
