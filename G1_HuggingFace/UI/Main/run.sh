#!/usr/bin/env bash
# 警備システム UI を起動する。
#
#   bash run.sh                 # モック + サンプル動画（実機も ROS も要らない）
#   bash run.sh --nav http      # Docker 内の ROS アダプタに繋ぐ（アダプタは未実装）
#   bash run.sh --port 9000
#
# ⚠️ **ROS 環境を source したシェルで叩かないこと。** このプロセスは操作PC の素の venv
#    （G1_HuggingFace/venv）で動く。ROS を継承すると opencv/numpy が衝突しうる。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/../../venv/bin/python"

if [[ ! -x "$VENV" ]]; then
  echo "[起動] venv が見つかりません: $VENV" >&2
  echo "       G1_HuggingFace/README.md の手順で作ってください" >&2
  exit 1
fi

if ! "$VENV" -c "import ultralytics" 2>/dev/null; then
  echo "[起動] ultralytics が入っていません。次で入れてください:" >&2
  echo "       $VENV -m pip install -r $HERE/../../../Perception/requirements.txt" >&2
  exit 1
fi

# ⚠️ ここで cd するのは、YOLO の重み(yolo26n.pt)をこの場所から拾わせるため。
#    別の場所から起動すると ultralytics が重みを再ダウンロードする
cd "$HERE"
# -u はログを即座に出すため（リダイレクトするとバッファされて何も見えない）
exec "$VENV" -u server.py "$@"
