#!/usr/bin/env bash
# Same real button launcher, with a selectable free-arm speed multiplier.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MODE=push
ARGS=()
for ARG in "$@"; do
    case "$ARG" in
        --check) MODE=check ;;
        --prepare-only)
            echo '旧 --prepare-only は廃止しました。転送のみの場合は --check --deploy-only を指定してください。' >&2
            exit 2 ;;
        *) ARGS+=("$ARG") ;;
    esac
done
exec bash "$HERE/run_robust_button.sh" "$MODE" --arm-speed-multiplier "${ARM_SPEED_MULTIPLIER:-5}" "${ARGS[@]}"
