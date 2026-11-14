#!/usr/bin/env bash
# 指定した言葉を、G1 本体のスピーカーで喋らせる（操作PCで実行。中身は voicevox_speak.py --play）。
#
#   bash Voice/say.sh "こんにちは。私はジーワンです"
#   bash Voice/say.sh "ゆっくり話します" --speed 0.9      # 2 つ目からは voicevox_speak.py にそのまま渡す
#   STYLE=11 bash Voice/say.sh "玄野武宏の声です"           # 声を変える（スタイル ID は --list-speakers）
#
# 既定の声は voicevox_speak.py の既定（剣崎雌雄、スタイル 21、pitch 0.05、抑揚は VOICEVOX の標準）。
# 環境変数: STYLE・PITCH・INTONATION（声）、VOICEVOX_PY（voicevox_core を入れた python）、PC2（ssh の宛先）
#   例: STYLE=32 は白上虎太郎「わーい」
# ⚠️ 外に出す場面（発表・動画など）では「VOICEVOX:剣崎雌雄」のような、使った声のクレジット表記が要る
set -euo pipefail

if [ $# -lt 1 ] || [ -z "$1" ]; then
    echo "使い方: bash Voice/say.sh \"喋らせる言葉\" [voicevox_speak.py のオプション]" >&2
    exit 2
fi
text="$1"
shift

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${VOICEVOX_PY:-$HOME/voicevox_core/venv/bin/python}"
voice=()
[ -n "${STYLE:-}" ] && voice+=(--style "$STYLE")
[ -n "${PITCH:-}" ] && voice+=(--pitch "$PITCH")
[ -n "${INTONATION:-}" ] && voice+=(--intonation "$INTONATION")
exec "$PY" "$HERE/voicevox_speak.py" --play "${voice[@]}" \
    --host "${PC2:-unitree@192.168.0.82}" "$@" "$text"
