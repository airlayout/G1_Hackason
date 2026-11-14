#!/usr/bin/env bash
# 強化学習を別の PC（メンバーの PC など）で動かすための一式を、1 つの tar.gz にまとめる。
# リポジトリと同じ並びで入れる（common/config.py がリポジトリ直下を J1-gen の 2 つ上として探すため）:
#   Button_Press/J1-gen/{common,configs,rl}/ と _local/button_press/models/g1_description/（公式モデル、約 19 MB）
#
#   bash Button_Press/J1-gen/rl/make_bundle.sh            # → _local/rl/bundle/g1_rl_<日時>.tar.gz
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
MODELS="_local/button_press/models/g1_description"

if [ ! -f "$REPO_ROOT/$MODELS/g1_29dof_rev_1_0.xml" ]; then
    echo "[make_bundle] 公式モデルが無い: $MODELS（先に bash Button_Press/J1-gen/sim/fetch_models.sh）" >&2
    exit 1
fi
OUT_DIR="$REPO_ROOT/_local/rl/bundle"
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/g1_rl_$(date +%Y%m%d_%H%M%S).tar.gz"
tar -C "$REPO_ROOT" --exclude='__pycache__' -czf "$OUT" \
    Button_Press/J1-gen/common Button_Press/J1-gen/configs Button_Press/J1-gen/rl "$MODELS"
echo "[make_bundle] 作った: $OUT（$(du -h "$OUT" | cut -f1)）"
