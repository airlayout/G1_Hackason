#!/usr/bin/env bash
# Explicit entry point: align once at 5cm, then advance without repeated holds.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MODE=push
ARGS=()
for ARG in "$@"; do
    case "$ARG" in
        --check) MODE=check ;;
        *) ARGS+=("$ARG") ;;
    esac
done
exec bash "$HERE/run_robust_button.sh" "$MODE" "${ARGS[@]}" --button-press-mode straight_trial
