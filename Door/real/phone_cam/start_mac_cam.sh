#!/usr/bin/env bash
# **Mac の上で**動かす。連係カメラ（iPhone を Mac のカメラにする）で機体を録る。11-13 から撮影の本筋（mac_cam.py）。
# iPhone は Mac の近くに三脚か挟み具で留める。Safari の撮影のページ（start_phone_cam.sh）は閉じておく（カメラを取り合う）。
#
#   bash Door/real/phone_cam/start_mac_cam.sh                    # PC2 の走らせ役の a で自動で録る（接続先は deploy_to_pc2.sh と同じ環境変数）
#   bash Door/real/phone_cam/start_mac_cam.sh --test-record 10   # 10 秒だけ録って終わる（初めの試し・時計の合わせ）
#   FEED=0 bash Door/real/phone_cam/start_mac_cam.sh             # PC2 を見ない（スペースで録る／止める）
#   G1_JUMP=ubuntu@100.99.102.70 bash Door/real/phone_cam/start_mac_cam.sh   # PC2 へ直接入れない日は OMEN を踏み台に
#
# キー: 1 当たった・2 滑った・3 後ずさり・4 良い・m メモ・スペース 手で録る／止める・q 終わる
# 置き場は Door/real/logs_real/phone/<日付>/（DATA= で替える）。回の後は phone_sync.py <cam_….json> [--npz …]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-$ROOT/Navigation/.venv/bin/python}"
[ -x "$PY" ] || PY=python3
DATA="${DATA:-$HERE/../logs_real/phone/$(date +%Y%m%d)}"
mkdir -p "$DATA"
ARGS=(--data "$DATA")
[ "${FEED:-1}" = 0 ] && ARGS+=(--no-feed)
# 画面の出力をファイルにも残す。tee -i は Ctrl-C で先に死なない。キーは端末から直に読む（標準入力は端末のまま）
LOG="$DATA/mac_cam_$(date +%H%M%S).log"
echo "[cam] 画面の出力は $LOG にも残す"
"$PY" -u "$HERE/mac_cam.py" "${ARGS[@]}" "$@" 2>&1 | tee -ia "$LOG"
