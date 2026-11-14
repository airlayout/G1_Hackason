#!/usr/bin/env bash
# 機体で準備済みのボタン押し環境を使い、その作業ツリーは変更しない。
set -euo pipefail
target="${1:?bottle または down を指定してください}"
shift
case "$target" in
  bottle|down) ;;
  *) echo "未登録の対象です: $target" >&2; exit 2 ;;
esac
button_repo="${VOICE_BUTTON_REPO:-/home/unitree/button_press/repo}"
button_python="${VOICE_BUTTON_PYTHON:-/home/unitree/button_press/venv/bin/python}"
button_script="$button_repo/Button_Press/J1-gen/real/auto_press.py"
if [[ ! -x "$button_python" || ! -f "$button_script" ]]; then
  echo "VOICE_BUTTON_PYTHON と VOICE_BUTTON_REPO に準備済みの環境を指定してください" >&2
  exit 2
fi
cd "$button_repo"
# ROS のライブラリや Python モジュールを SDK 用プロセスに持ち込まない。
exec env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u ROS_DISTRO -u RMW_IMPLEMENTATION -u CYCLONEDDS_URI \
  "$button_python" "$button_script" --target "$target" \
  --network-interface "${VOICE_BUTTON_INTERFACE:-eth0}" "$@"
