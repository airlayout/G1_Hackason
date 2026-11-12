#!/usr/bin/env bash
# **Mac の上で**動かす（11-12 夜・作業ログ 2026-11-12 の 7 章の 1・2）: 台本の定数を門番（script_gate.py）に通し、
# 通ったら定数と <定数>.ok を PC2 の ~/door_real/ へ送る。PC2 の走らせ役は起こしたまま、次の d で読み直す。
#
#   DOOR=0.94,2.05,0.99,0.064 PELVIS_Z=0.73 bash Door/real/push_params.sh [Door/real/script_params.json]
#   GEOM=Door/real/door_geom.json …   扉の周りの寸法を当日に測り直した時（一緒に送る）
#   無線・踏み台: G1_PC2_HOST=192.168.0.82 G1_JUMP=ubuntu@100.99.102.70 …（deploy_to_pc2.sh と同じ）
#
# 落ちたら送らない（PC2 の古い定数と .ok はそのまま＝前に通った定数で走る）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$HERE/../../Mapping/real/quickstart/live/_common.sh"
PARAMS="${1:-$HERE/script_params.json}"
: "${DOOR:?DOOR=幅,高さ,レバーの高さ,バックセット を渡す（走らせ役の DOOR と同じ文字列）}"
PZ="${PELVIS_Z:-0.75}"
PY="${PY:-$HERE/../../Navigation/.venv/bin/python}"
say() { echo "[push] $*"; }
ARGS=("$PARAMS" --door "$DOOR" --pelvis-z "$PZ"); [ -n "${GEOM:-}" ] && ARGS+=(--geom "$GEOM")
(cd "$HERE" && "$PY" script_gate.py "${ARGS[@]}") || { say "門番に落ちた: 送らない"; exit 1; }
JUMP=(); [ -n "${G1_JUMP:-}" ] && JUMP=(-J "$G1_JUMP")
FILES=("$PARAMS" "$PARAMS.ok"); [ -n "${GEOM:-}" ] && FILES+=("$GEOM")
scp -O -q ${JUMP[@]+"${JUMP[@]}"} -i "$G1_KEY" -o BatchMode=yes -o IdentitiesOnly=yes "${FILES[@]}" "$G1_PC2_USER@$G1_PC2_HOST:door_real/" \
  || { say "scp 失敗（PC2 の電源・G1_PC2_HOST・G1_JUMP）"; exit 1; }
for f in "${FILES[@]}"; do
  want=$(md5 -q "$f")
  got=$(ssh ${JUMP[@]+"${JUMP[@]}"} -i "$G1_KEY" -o BatchMode=yes -o IdentitiesOnly=yes "$G1_PC2_USER@$G1_PC2_HOST" "md5sum ~/door_real/$(basename "$f")" | cut -d' ' -f1)
  [ "$want" = "$got" ] && say "送った: $(basename "$f")" || { say "⚠️ md5 が合わない: $(basename "$f")"; exit 1; }
done
say "次の d で読み直す（./stock_cmd.sh a → d）"
