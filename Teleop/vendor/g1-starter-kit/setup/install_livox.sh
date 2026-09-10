#!/usr/bin/env bash
# 【root 不要】頭部 LiDAR（Livox）のドライバをビルドする。
#
#   bash setup/install_livox.sh
#
# 前提: setup/install_apt.sh が完了していること（ROS 2 Humble と PCL が必要）。
#
# 作るもの:
#   ~/.local/lib/liblivox_lidar_sdk_*   … Livox-SDK2（sudo を使わないため ~/.local に入れる）
#   ~/ws_livox/                          … livox_ros_driver2 の ROS 2 ワークスペース
#
# なぜ ~/.local に入れるのか:
#   Livox-SDK2 は既定で /usr/local にインストールされ、sudo が必要になる。
#   livox_ros_driver2 の CMakeLists は /usr/local/lib を直接指定しているが、
#   CMake は CMAKE_LIBRARY_PATH も探すので、そこを渡せば sudo 無しで解決できる。
#   実行時は LD_LIBRARY_PATH が必要（scripts/lib.sh が設定する）。

set -euo pipefail

log() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()  { printf '\033[32m[OK]\033[0m %s\n' "$*"; }
skip(){ printf '\033[90m[済]\033[0m %s\n' "$*"; }
die() { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }

[ -f /opt/ros/humble/setup.bash ] \
  || die "ROS 2 Humble がありません。先に bash setup/install_apt.sh を実行してください。"

# conda の python が ROS のビルドと衝突するので外す
if [[ "$PATH" == *conda* || "$PATH" == *miniforge* ]]; then
  PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E 'conda|miniforge' | paste -sd:)"
  export PATH
fi
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
ok "ROS 2 Humble / python3=$(command -v python3)"

# ---------------------------------------------------------------------------
log "Livox-SDK2 をビルド（~/.local へ・sudo 不要）"
if [ -f "$HOME/.local/lib/liblivox_lidar_sdk_shared.so" ]; then
  skip "既にビルド済み"
else
  [ -d "$HOME/Livox-SDK2/.git" ] \
    || git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$HOME/Livox-SDK2"
  mkdir -p "$HOME/Livox-SDK2/build"
  cd "$HOME/Livox-SDK2/build"
  cmake .. -DCMAKE_INSTALL_PREFIX="$HOME/.local" -DCMAKE_BUILD_TYPE=Release > /dev/null
  make -j"$(nproc)" > /dev/null
  make install > /dev/null
  ok "ビルドしました"
fi
[ -f "$HOME/.local/lib/liblivox_lidar_sdk_shared.so" ] \
  || die "Livox-SDK2 の共有ライブラリが作られていません。"

# ---------------------------------------------------------------------------
log "livox_ros_driver2 をビルド"
mkdir -p "$HOME/ws_livox/src"
if [ ! -d "$HOME/ws_livox/src/livox_ros_driver2/.git" ]; then
  git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    "$HOME/ws_livox/src/livox_ros_driver2"
fi

cd "$HOME/ws_livox/src/livox_ros_driver2"
# このリポジトリは ROS1/ROS2 で package.xml を差し替える作りになっている
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/ 2>/dev/null || true

cd "$HOME/ws_livox"
colcon build --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  -DCMAKE_LIBRARY_PATH="$HOME/.local/lib" \
  -DCMAKE_INCLUDE_PATH="$HOME/.local/include" \
  2>&1 | tail -5

[ -f "$HOME/ws_livox/install/setup.bash" ] || die "ビルドに失敗しました。"

# ---------------------------------------------------------------------------
log "検証"
set +u
# shellcheck disable=SC1091
source "$HOME/ws_livox/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
set -u

ros2 pkg executables livox_ros_driver2 | grep -q livox_ros_driver2_node \
  && ok "livox_ros_driver2_node が見つかりました" \
  || die "実行体が見つかりません。"

LIB="$(find "$HOME/ws_livox/install" -name 'liblivox_ros_driver2.so' | head -1)"
if ldd "$LIB" | grep -q "not found"; then
  echo "  未解決の依存:"; ldd "$LIB" | grep "not found"
  die "共有ライブラリが解決できません。LD_LIBRARY_PATH の設定を確認してください。"
fi
ok "共有ライブラリの依存はすべて解決"

log "完了。次は LiDAR の機種を自動検出します"
cat <<'EOF'
  ロボットに有線で繋いだ状態で:

    python3 tools/lidar_probe.py --write-config
    ./scripts/lidar_view.sh

  ⚠️ Livox には Mid-360 と Mid-360S があり、設定ファイルの形式が違います。
     間違えるとドライバは「初期化成功」と表示したまま何も出しません。
     lidar_probe.py が機種を判定して正しい設定を作るので、必ず先に実行してください。
EOF
