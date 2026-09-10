#!/usr/bin/env bash
# 項目2 — LiDAR による計測を見せる。
#
#   bash Demo/01_lidar.sh            # 実機が居れば live、居なければ replay
#   bash Demo/01_lidar.sh live       # 実機のライブ点群（実機が要る）
#   bash Demo/01_lidar.sh replay     # 記録の再生（実機は要らない）
#
# 引数なしのときは**実機が居なければ黙って replay に落とす**。当日の保険。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

UBUNTU_DIR="${REPO_DIR}/Mapping/real/ubuntu"
MODE="${1:-auto}"
shift || true

if [ "${MODE}" = "auto" ]; then
    if robot_reachable; then
        _info "G1 が見つかりました → ライブで出します"
        MODE="live"
    else
        _warn "G1 が見つかりません → 記録の再生に切り替えます"
        MODE="replay"
    fi
fi

case "${MODE}" in
    live)   exec bash "${UBUNTU_DIR}/lidar_live.sh" "$@" ;;
    replay) exec bash "${UBUNTU_DIR}/lidar_replay.sh" "$@" ;;
    *)      _die "使い方: bash Demo/01_lidar.sh [live|replay]" ;;
esac
