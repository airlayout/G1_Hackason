#!/usr/bin/env bash
# 足りない apt パッケージを入れる。**何度実行してもよい**（入っていれば飛ばす）。
#
#   bash Demo/setup/install.sh
#   bash Demo/setup/install.sh --dry-run   # 何を入れるか見るだけ
#
# ⚠️ **sudo が要る操作はこのスクリプトに集約してある。**
#    VSCode の統合ターミナルでは sudo のパスワード入力が通らないことがある。
#    普通のターミナル（GNOME Terminal 等）で実行すること。
#
# ⚠️ **キットの setup/install_apt.sh は呼ばない。**
#    あちらは :35 で jammy 以外を die する。デモ機は 24.04（noble）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/lib.sh"

DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

# **このリポジトリのスクリプトが実際に使うものだけ**を並べる。
# 「あると便利そう」で足さないこと（入れたものは当日の障害の原因になりうる）。
PACKAGES=(
    ros-jazzy-rmw-cyclonedds-cpp   # 実演3 ライブ。G1 の DDS を購読する
    ros-jazzy-tf2-ros              # 実演3。記録に /tf が無いので静的 TF を出す
    ros-jazzy-rviz2                # 実演3・3
    ros-jazzy-nav2-bringup         # 実演2
)

_head "入っているか調べる"
MISSING=()
for p in "${PACKAGES[@]}"; do
    if [ "$(dpkg-query -W -f='${Status}' "${p}" 2>/dev/null | grep -c 'install ok installed')" -gt 0 ]; then
        _ok "${p}"
    else
        _ng "${p}"
        MISSING+=("${p}")
    fi
done

# sqlite3 の storage プラグイン（記録が version 4 / sqlite3 のため）
if [ -f /opt/ros/jazzy/lib/librosbag2_storage_sqlite3.so ]; then
    _ok "rosbag2 の sqlite3 プラグイン"
else
    _warn "rosbag2 の sqlite3 プラグインが見つかりません。記録の再生ができない可能性があります"
fi

if [ "${#MISSING[@]}" -eq 0 ]; then
    _head "結果"
    _ok "足りないものはありません"
    exit 0
fi

_head "入れるもの"
printf '  %s\n' "${MISSING[@]}"

if [ "${DRY}" = "1" ]; then
    _info "--dry-run なので入れません"
    exit 0
fi

printf '\n'
# ssh 越しだと sudo のパスワードが打てない。先に見て、駄目なら手順だけ出す。
if ! sudo -n true 2>/dev/null; then
    _warn "sudo にパスワードが要ります。"
    printf '\n  \033[1mデモ機のターミナルで直接\033[0m 次を実行してください:\n\n'
    printf '    sudo apt-get update\n'
    printf '    sudo apt-get install -y %s\n\n' "${MISSING[*]}"
    printf '  （ssh 越しや VSCode の統合ターミナルでは通らないことがあります）\n'
    exit 1
fi
_info "sudo apt-get install を実行します"
sudo apt-get update
sudo apt-get install -y "${MISSING[@]}"

_head "確認"
FAILED=0
for p in "${MISSING[@]}"; do
    if [ "$(dpkg-query -W -f='${Status}' "${p}" 2>/dev/null | grep -c 'install ok installed')" -gt 0 ]; then
        _ok "${p}"
    else
        _ng "${p} が入りませんでした"
        FAILED=1
    fi
done
exit "${FAILED}"
