#!/usr/bin/env bash
# **Mac の上で**動かす。証明書を確かめ、撮影の受け口（phone_server.py）を起こし、撮影のページの URL の QR を Mac の画面に出す。
# Ctrl-C で止める。iPhone と Mac は同じ網（G1 の Wi-Fi「Physical_AI」・Mac は 192.168.0.x）に居ること。
#
#   bash Door/real/phone_cam/start_phone_cam.sh            # PC2 の走らせ役の出力も見る（接続先は deploy_to_pc2.sh と同じ環境変数）
#   FEED=0 bash Door/real/phone_cam/start_phone_cam.sh     # 撮るだけ（機体の電源が切れている時・ページの試し）
#   G1_JUMP=ubuntu@100.99.102.70 bash Door/real/phone_cam/start_phone_cam.sh   # PC2 へ直接入れない日は OMEN を踏み台に
#
# 置き場は Door/real/logs_real/phone/<日付>/（DATA= で替える）。QR は ~/.config/g1_phone_cam/url_qr.png（鍵を含むので git の外）
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-$ROOT/Navigation/.venv/bin/python}"
[ -x "$PY" ] || PY=python3
bash "$HERE/make_cert.sh"
DATA="${DATA:-$HERE/../logs_real/phone/$(date +%Y%m%d)}"
mkdir -p "$DATA"
QR="$HOME/.config/g1_phone_cam/url_qr.png"
rm -f "$QR"
ARGS=(--data "$DATA" --qr "$QR")
[ "${FEED:-1}" = 0 ] && ARGS+=(--no-feed)
( for _ in 1 2 3 4 5 6 7 8 9 10; do [ -f "$QR" ] && { open "$QR"; exit 0; }; sleep 0.5; done ) &
# 画面の出力（電話からの接続の跡・握手で断った理由・録画の受け取り）をファイルにも残す。tee -i は Ctrl-C で先に死なない
LOG="$DATA/server_$(date +%H%M%S).log"
echo "[phone] 画面の出力は $LOG にも残す"
"$PY" -u "$HERE/phone_server.py" "${ARGS[@]}" 2>&1 | tee -ia "$LOG"
