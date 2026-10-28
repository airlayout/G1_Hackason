#!/usr/bin/env bash
# 段 A・B の結果（作業ログ 2026-10-27 の 13 章）を受けた、候補 1〜3 を 1 つずつ足す試し（10-28 ユーザー「1〜3 項を実施して」）。
# どの段も同じ最良の重み（反復 53,750・77.0%）から始めるので、隣どうしの段の差が、足した直しの効き目になる。
#
#   段 C（f3d）: 雑音の上限＋候補 1「歩き去りの罰を早く・強く」（1.2 m → WALK_START 既定 1.0 m、重み −12 → WALK_W 既定 −40）
#   段 D（f3e）: 段 C＋候補 3「押し開けの持ち時間を評価と同じに」（学習の段階 4 の持ち時間 300 → SWING_STEPS 既定 600 歩）
#   段 E（f3f）: 段 D＋候補 2「押し開けの間は扉の先の目標へ近づく報酬を止める」（パッチ 0080、段階 4 の倍率 0.5 → 0）
# 段 C・D は ROUNDS_C・ROUNDS_D 回（既定 6 回＝約 3.4 時間ずつ）。段 E は ROUNDS_E 回（既定 9999＝止めるまで）で、
# 終わったら f3_keepalive_1026.sh が段 E の続きを始め直す。
#
# 台数（10-28 11 時〜）: NUM_ENVS（既定 512）と MEM_MAX（既定 9G）は stage_chain.sh・run_train_segments.sh・見張りがそのまま受け継ぐ。
#   評価は台数に依らず 256 回（eval_from_start.sh）。台数を変えた段どうしは、学習の量（30 分で集める歩数）も変わる
#
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く（今の回の評価まで済ませて止まり、次の段も見張りも始めない）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_f3_phases_1028.sh >> logs_f3_phases_1028.log 2>&1 < /dev/null &
#   台数を変えるとき: NUM_ENVS=1024 MEM_MAX=12G setsid nohup ./exp_f3_phases_1028.sh …
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
BEST="${BEST:-$HOME/work_inoue/doorman/ckpt/f3_best_it53750/last.pt}"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

CAP="++algo.config.max_noise_std=${NOISE_MAX:-0.26}"
WALK="++env.config.walk_away_start_m=${WALK_START:-1.0},++rewards.reward_scales.walk_away_in_swing=${WALK_W:--40.0}"
# 段の持ち時間の並びの中のカンマは「|」で書く（stage_chain.sh がカンマに戻す）
LONG="++env.config.max_stage_time=[250|100|100|100|${SWING_STEPS:-600}|200]"
NOPULL="++env.config.target_root_distance_swing_scale=0.0"
# 512 台でないときは段の名前に台数を付ける（f3d → f3dn4096。重みの置き場 logs_rl/door_<名前> と記録を 512 台の回と混ぜない）
SFX=""; [ "${NUM_ENVS:-512}" != 512 ] && SFX="n${NUM_ENVS}"
# 名前;回数;足す引数（「;」で区切る。引数の側は「,」区切り）
PHASES=(
  "f3d${SFX};${ROUNDS_C:-6};$CAP,$WALK"
  "f3e${SFX};${ROUNDS_D:-6};$CAP,$WALK,$LONG"
  "f3f${SFX};${ROUNDS_E:-9999};$CAP,$WALK,$LONG,$NOPULL"
)

[ -f "$BEST" ] || { log "出発点の重みが無い: $BEST"; exit 1; }
for P in "${PHASES[@]}"; do
  IFS=';' read -r PNAME PROUNDS PMORE <<< "$P"
  log "段 ${PNAME} を始める ck=$BEST rounds=${PROUNDS} n=${NUM_ENVS:-512} mem_max=${MEM_MAX:-9G} more=${PMORE}"
  # 最後の段の名前と引数は、見張り（f3_keepalive_1026.sh）が続きを始めるときに受け継ぐ
  export NAME="$PNAME" TRAIN_MORE="$PMORE"
  CK="$BEST" ROUNDS="$PROUNDS" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので、次の段も見張りも始めずに終わる"; exit 0; fi
done
[ "${NO_KEEPALIVE:-0}" = 1 ] && { log "NO_KEEPALIVE=1 なので見張りを始めずに終わる"; exit 0; }
log "最後の段の続きは見張り（f3_keepalive_1026.sh）に任せる（name=${NAME} more=${TRAIN_MORE}）"
exec ./f3_keepalive_1026.sh
