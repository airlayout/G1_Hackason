#!/usr/bin/env bash
# 3D LiDAR のコストマップを実測する（層数 x 水平ビーム数のグリッドを網羅）。
#
# 1 条件ごとにプロセスを作り直す。MultiMeshRayCaster はシーンに登録された
# 全センサがまとめて更新されるため、1 プロセスに複数のセンサを作ると
# 1 条件の計測に他条件のコストが混入する（実際にこれで誤った結果を出した）。
#
# 1 条件あたり Isaac Sim の起動に 1〜2 分かかるため、全 24 条件で 30〜50 分。
#
# 使い方:
#   bash run_cost_map.sh              # 全グリッド（24 条件）
#   bash run_cost_map.sh --quick      # 実用候補のみ（6 条件）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

TSV="$SCRIPT_DIR/logs/cost_map.tsv"
LOG="$SCRIPT_DIR/logs/cost_map.log"

if [[ "${1:-}" == "--quick" ]]; then
    CHANNELS=(1 8 16 32)
    BEAMS=(180 360)
else
    CHANNELS=(1 4 8 16 32 64)
    BEAMS=(90 180 360 720)
fi

# 集計しなおすので前回の結果は退避する
if [[ -f "$TSV" ]]; then
    mv "$TSV" "$TSV.$(date +%Y%m%d_%H%M%S).bak"
fi
printf 'channels\tbeams\ttotal_beams\tmedian_ms\tmin_ms\tcast_ms\thit_pct\n' > "$TSV"

TOTAL=$(( ${#CHANNELS[@]} * ${#BEAMS[@]} ))
N=0
echo "[INFO] コストマップ実測を開始します: $TOTAL 条件 (log: $LOG)"
: > "$LOG"

for ch in "${CHANNELS[@]}"; do
    for bm in "${BEAMS[@]}"; do
        N=$(( N + 1 ))
        echo "[INFO] ($N/$TOTAL) ${ch} 層 x ${bm} 水平 を計測中..."
        # 1 条件が失敗しても残りは続ける（原因はログに残る）
        if ! "$ISAAC_SIM/python.sh" src/probe_lidar3d_cost.py \
                --viz none --channels "$ch" --beams "$bm" --tsv "$TSV" \
                >> "$LOG" 2>&1; then
            echo "[WARN] ($N/$TOTAL) ${ch} 層 x ${bm} 水平 が失敗しました（$LOG を確認）"
        fi
    done
done

echo
echo "[INFO] 実測完了。結果:"
echo
column -t -s $'\t' "$TSV"
echo
echo "[INFO] TSV: $TSV"
