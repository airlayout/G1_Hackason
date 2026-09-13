#!/usr/bin/env bash
# PC2(Ubuntu 20.04 focal) で jammy(22.04) の ROS 2 Humble + MOLA を動かす。
# **コンテナも root も使わない**（計画 §5.1 の案 B）。
#
#   ssh g1 'bash ~/mapping_tools/setup_jammy_ros.sh'
#   ssh g1 '. ~/jammy_ros/env.sh && "$ROS/bin/mola-lidar-odometry-cli" --help'
#
# ## なぜこれが要るか
#
# MOLA は `packages.ros.org` の **jammy の deb でしか配られていない**
# （conda/RoboStack には mola も mrpt も無い。linux-64 にも無い＝そもそも入っていない）。
# そして jammy の .so は **GLIBC_2.34 / GLIBCXX_3.4.30 / CXXABI_1.3.13** を要求するが、
# focal が持つのは **2.31 / 3.4.28 / 1.3.12**。glibc の互換は一方向なので素では動かない。
#
# ## どうやるか
#
# jammy の deb を `dpkg -x` で prefix に展開し（root 不要）、**jammy の ld.so を
# 明示指定して起動する**。system の /etc/apt にも /var/lib/dpkg にも一切触らない。
# **撤退は `rm -rf ~/jammy_ros` だけ**（pixi.toml と同じ思想）。
#
# ⚠️ pixi の環境は触らない。あちらは cmd_vel_bridge / foxglove_bridge が使っている。
set -uo pipefail

ROOT="${JAMMY_ROOT:-$HOME/jammy_ros}"
PREFIX="$ROOT/rootfs"
PKGS="${JAMMY_PKGS:-ros-humble-mola-lidar-odometry ros-humble-rmw-cyclonedds-cpp \
ros-humble-mola-bridge-ros2 ros-humble-ros2launch ros-humble-ros2bag \
ros-humble-rosbag2-storage-default-plugins ros-humble-tf2-ros \
ros-humble-mola-state-estimation-smoother ros-humble-mola-metric-maps}"

say() { echo "[jammy] $*"; }

# ── 1. system と完全に分離した apt を用意する ──────────────────────
mkdir -p "$ROOT"/etc/apt/sources.list.d "$ROOT"/etc/apt/preferences.d \
         "$ROOT"/var/lib/apt/lists/partial "$ROOT"/var/cache/apt/archives/partial \
         "$ROOT"/var/lib/dpkg
# dpkg status を空にする ＝「何も入っていない」と思わせて閉包を全部引く。
# これで jammy の libc6 まで含む自己完結した prefix になる
[ -f "$ROOT/var/lib/dpkg/status" ] || : > "$ROOT/var/lib/dpkg/status"

cat > "$ROOT/etc/apt/sources.list" <<EOF
deb [trusted=yes] http://ports.ubuntu.com/ubuntu-ports jammy main universe
deb [trusted=yes] http://ports.ubuntu.com/ubuntu-ports jammy-updates main universe
deb [trusted=yes] http://packages.ros.org/ros2/ubuntu jammy main
EOF

cat > "$ROOT/aptopts" <<EOF
-o Dir::Etc::sourcelist=$ROOT/etc/apt/sources.list
-o Dir::Etc::sourceparts=$ROOT/etc/apt/sources.list.d
-o Dir::Etc::preferences=$ROOT/etc/apt/preferences
-o Dir::Etc::preferencesparts=$ROOT/etc/apt/preferences.d
-o Dir::State=$ROOT/var/lib/apt
-o Dir::State::status=$ROOT/var/lib/dpkg/status
-o Dir::Cache=$ROOT/var/cache/apt
-o APT::Architecture=arm64
-o APT::Architectures::=arm64
-o Debug::NoLocking=1
EOF
OPTS=$(cat "$ROOT/aptopts")

say "索引を取得する（system の apt には触らない）"
apt-get $OPTS update >/dev/null 2>&1 || { echo "[jammy] apt update に失敗" >&2; exit 1; }

say "閉包を解決して取得する"
# shellcheck disable=SC2086
apt-get $OPTS install --no-install-recommends --download-only -y $PKGS >/dev/null 2>&1 \
    || { echo "[jammy] 取得に失敗" >&2; exit 1; }
say "  deb $(ls "$ROOT"/var/cache/apt/archives/*.deb | wc -l) 個 / $(du -sh "$ROOT"/var/cache/apt/archives | cut -f1)"

say "展開する（root 不要）"
mkdir -p "$PREFIX"
for deb in "$ROOT"/var/cache/apt/archives/*.deb; do dpkg -x "$deb" "$PREFIX" 2>/dev/null; done
say "  prefix $(du -sh "$PREFIX" | cut -f1)"

bash "$(dirname "${BASH_SOURCE[0]}")/write_jammy_env.sh" "$ROOT" || exit 1

say "完了。使い方: . $ROOT/env.sh"
