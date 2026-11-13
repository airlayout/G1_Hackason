#!/usr/bin/env bash
# SDK が入っている G1 の Python を使う。ROS / モデル用 Python とは分ける。
set -euo pipefail
VOICE_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${VOICE_SDK_PYTHON:-/usr/bin/python3}" "$VOICE_REPO_ROOT/Entame/real/arm_action_tool.py" "$@"
