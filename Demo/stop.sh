#!/usr/bin/env bash
# デモで起動したものを全部止める。
#
#   bash Demo/stop.sh
#
# 各スクリプトは Ctrl-C で自分の後片付けをするので、普段はこれを使わなくてよい。
# ターミナルを閉じてしまった / Isaac Sim が残った、というときの逃げ道。
#
# ⚠️ **手で pkill を打たないこと。** 止める signal は送る先で逆になるうえ、
#    pgrep -f は自分自身と `bash -c` のラッパにマッチする。除外を忘れて
#    呼び出し元の ssh セッションごと殺した前例がある（2026-09-09、exit 255）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

_head "デモを止める"

# Isaac Sim + Nav2 — 正しい作法を知っているスクリプトに任せる
stop_isaac_nav2

# RViz2 と bag 再生と静的 TF。ここは自前で探すしかないが、
# パターンを絞り、自分自身と bash -c のラッパを必ず除外する。
_info "RViz2 / bag 再生 / 静的 TF を止めます"
LEFTOVERS="$(pgrep -af 'rviz2|ros2 bag play|static_transform_publisher' 2>/dev/null \
    | grep -v 'bash -c' \
    | grep -v 'Demo/stop.sh' \
    | awk -v self="$$" -v parent="$PPID" '$1 != self && $1 != parent {print $1}' || true)"

if [ -z "${LEFTOVERS}" ]; then
    _ok "残っているものはありません"
else
    # shellcheck disable=SC2086
    stop_tracked_pids ${LEFTOVERS}
fi

_head "確認"
STILL="$(pgrep -af 'rviz2|ros2 bag play|static_transform_publisher|run_g1_twin|component_container_isolated' 2>/dev/null \
    | grep -v 'bash -c' \
    | grep -v 'Demo/stop.sh' \
    | awk -v self="$$" -v parent="$PPID" '$1 != self && $1 != parent {print $0}' || true)"
if [ -z "${STILL}" ]; then
    _ok "全部止まりました"
else
    _warn "まだ残っています:"
    printf '%s\n' "${STILL}"
fi
