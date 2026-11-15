#!/usr/bin/env bash
# Button pressing supports push only and defaults to kinematic validation.
# Align once before a straight press; bounded joint torque correction remains.
# CLI arguments can select --button-press-mode feedback for the earlier mode.
# G1 setup routes over SSH.
# Readiness uses the shared bottle collection limits; target movement is logged.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
VALIDATION=()
for ARG in "$@"; do
    case "$ARG" in
        check|precontact|--check)
            echo 'ボタン押しはpushのみ対応しています。' >&2
            exit 2 ;;
        push) VALIDATION=(--button-validation kinematic) ;;
    esac
done
exec env PYTHONUNBUFFERED=1 python3 "$HERE/robust_session.py" \
    --readiness-policy collection --button-feedback joint --button-policy collection \
    --button-press-mode straight_trial \
    --planning-budget-s 300 "${VALIDATION[@]}" "$@" --task button
