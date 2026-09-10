#!/usr/bin/env bash
# Mac に置いてある LiDAR の記録（3.0GB）をデモ機へ持ってくる。
#
#   bash Mapping/real/ubuntu/fetch_bag.sh                 # 既定の記録
#   bash Mapping/real/ubuntu/fetch_bag.sh <記録名>        # 別の記録
#   MAC_HOST=user@host bash Mapping/real/ubuntu/fetch_bag.sh
#
# Mapping/real/runs/ は .gitignore 済み。git には乗らないので毎回これで持ってくる。
# **有線で引くこと。** 無線だと 3GB に時間がかかる。
#
# ## ⚠️ Mac 側の SSH が既定で切ってある
#
# macOS の「リモートログイン」は既定で無効。この状態だと引く向き（デモ機 → Mac）は
# `Connection refused` になる（2026-09-10 実測）。2 つの逃げ道がある。
#
#   (a) Mac の システム設定 → 一般 → 共有 → リモートログイン を入れてから、これを実行する
#   (b) **Mac 側から押す。** このスクリプトが下に印字するコマンドをそのまま Mac で打つ
#
# デモ当日にどちらでも通せるよう、繋がらなければ (b) の手順を出して終わる。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../Demo" && pwd)/lib.sh"

RUN_NAME="${1:-20260906T135940_UiS_room_v3}"
MAC_HOST="${MAC_HOST:-inouereo@192.168.123.202}"
MAC_REPO="${MAC_REPO:-git_research/physical_ai/G1_Hackason}"
REL="Mapping/real/runs/${RUN_NAME}/raw/rosbag2"

SRC="${MAC_HOST}:${MAC_REPO}/${REL}/"
DST="${REPO_DIR}/${REL}/"

_head "LiDAR の記録を持ってくる"
_info "元 : ${SRC}"
_info "先 : ${DST}"

if [ -f "${DST}/metadata.yaml" ]; then
    _ok "すでにあります"
    du -sh "${DST}"
    exit 0
fi

MAC_IP="${MAC_HOST##*@}"
if ! tcp_probe "${MAC_IP}" 22 4; then
    _ng "Mac の SSH（${MAC_IP}:22）に繋がりません"
    printf '\n  macOS の「リモートログイン」は既定で無効です。どちらかで進めてください。\n\n'
    printf '  \033[1m(a) Mac 側から押す（設定を変えなくてよい）\033[0m\n'
    printf '      Mac のターミナルで:\n\n'
    printf '        cd ~/%s\n' "${MAC_REPO}"
    printf '        rsync -a --partial \\\n'
    printf '          %s/ \\\n' "${REL}"
    printf '          %s:G1_Hackason/%s/\n\n' "$(whoami)@${DEMO_HOST_IP}" "${REL}"
    printf '      ⚠️ macOS の rsync は openrsync（2.6.9 相当）。--info=progress2 は\n'
    printf '         使えないので付けないこと（usage エラーで黙って終わる）。\n\n' 
    printf '  \033[1m(b) Mac でリモートログインを入れてから、これをやり直す\033[0m\n'
    printf '      システム設定 → 一般 → 共有 → リモートログイン\n\n'
    exit 1
fi

_info "3.0GB あります。有線なら 2〜4 分ほど。"
mkdir -p "${DST}"
# --partial で途中から再開できる
rsync -a --partial --info=progress2 "${SRC}" "${DST}"

if [ -f "${DST}/metadata.yaml" ]; then
    _ok "取得しました"
    grep -E "^  (version|storage_identifier|message_count):" "${DST}/metadata.yaml" || true
    du -sh "${DST}"
else
    _die "metadata.yaml がありません。転送に失敗しています。"
fi
