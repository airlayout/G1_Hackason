#!/usr/bin/env bash
# 敬礼・指さしを 1 コマンドで動かす。
#
#   bash Button_Press/J1-gen/real/act.sh salute                  # dry-run（何も送らない）
#   bash Button_Press/J1-gen/real/act.sh salute --execute        # 実機に送る（確認モード。人が打つ。緊急時はリモコンの L2+B）
#   bash Button_Press/J1-gen/real/act.sh point --execute --no-confirm
#   bash Button_Press/J1-gen/real/act.sh point --sim --execute --no-confirm   # sim（MuJoCo）
#
# 動作: salute / point（実機で確かめた）。他は --allow-unverified が要る。NIC は環境変数 G1_NIC か --nic（既定は configs/arm.yaml）。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-$REPO_ROOT/G1_HuggingFace/venv/bin/python}"
exec "$PY" "$HERE/act.py" "$@"
