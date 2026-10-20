#!/usr/bin/env bash
# 教師が学習していく様子の動画を作る（scripts/record_progress.py）。画面なしで録画する。
#   bash Button_Press/RL/scripts/record_progress.sh
#   bash Button_Press/RL/scripts/record_progress.sh --iters 0 600 1200 1499 2700 --clip_s 8
# 2 回に分けて録画し（タイトルと各時点 / たくさんの G1 を並べてカメラを引く場面とまとめ）、ffmpeg で 1 本につなぐ。
# 動画は _local/button_press_rl/videos/progress_<日時>.mp4
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
# Isaac Sim の起動の設定は先に置く（空白を含むので、--iters 0 400 ... のあとに置くと、argparse が回数の続きと読む）
ARGS=()
if ! printf '%s\n' "$@" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="$ISAAC_KIT_ARGS")
fi
ARGS+=("$@")
cd "$RL_OUT_DIR"
mkdir -p videos
STAMP="$(date +%Y%m%d_%H%M%S)"
PART1="videos/.part_clips_$STAMP.mp4"
PART2="videos/.part_crowd_$STAMP.mp4"
OUT="videos/progress_$STAMP.mp4"
"$ISAAC_SIM/python.sh" "$RL_DIR/scripts/record_progress.py" --headless --enable_cameras --part clips --out "$PART1" "${ARGS[@]}"
"$ISAAC_SIM/python.sh" "$RL_DIR/scripts/record_progress.py" --headless --enable_cameras --part crowd --out "$PART2" "${ARGS[@]}"
printf "file '%s'\nfile '%s'\n" "$(basename "$PART1")" "$(basename "$PART2")" > "videos/.list_$STAMP.txt"
(cd videos && ffmpeg -loglevel error -y -f concat -safe 0 -i ".list_$STAMP.txt" -c copy "$(basename "$OUT")")
rm -f "$PART1" "$PART2" "videos/.list_$STAMP.txt"
echo "[video] できた: $RL_OUT_DIR/$OUT"
