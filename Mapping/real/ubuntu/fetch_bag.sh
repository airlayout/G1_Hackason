#!/usr/bin/env bash
# Mac に置いてある LiDAR の記録（3.0GB）をデモ機へ持ってくる。
#
#   bash Mapping/real/ubuntu/fetch_bag.sh                 # 既定の記録
#   bash Mapping/real/ubuntu/fetch_bag.sh <記録名>        # 別の記録
#   MAC_HOST=user@host bash Mapping/real/ubuntu/fetch_bag.sh
#
# Mapping/real/runs/ は .gitignore 済み。git には乗らないので毎回これで持ってくる。
# **有線で引くこと。** 無線だと 3GB に時間がかかる。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../Demo" && pwd)/lib.sh"

RUN_NAME="${1:-20260906T135940_UiS_room_v3}"
MAC_HOST="${MAC_HOST:-inouereo@192.168.123.202}"
MAC_REPO="${MAC_REPO:-git_research/physical_ai/G1_Hackason}"

SRC="${MAC_HOST}:${MAC_REPO}/Mapping/real/runs/${RUN_NAME}/raw/rosbag2/"
DST="${REPO_DIR}/Mapping/real/runs/${RUN_NAME}/raw/rosbag2/"

_head "LiDAR の記録を持ってくる"
_info "元 : ${SRC}"
_info "先 : ${DST}"
_info "3.0GB あります。有線なら 2〜4 分ほど。"

mkdir -p "${DST}"
# --partial で途中から再開できる。--progress で進み具合を出す。
rsync -a --partial --progress "${SRC}" "${DST}"

if [ -f "${DST}/metadata.yaml" ]; then
    _ok "取得しました"
    grep -E "^  (version|storage_identifier|message_count):" "${DST}/metadata.yaml" || true
    du -sh "${DST}"
else
    _die "metadata.yaml がありません。転送に失敗しています。"
fi
