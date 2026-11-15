#!/usr/bin/env bash
# Explicit entry point: align once at 5cm, then advance without repeated holds.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$HERE/run_robust_button.sh" push "$@" --button-press-mode straight_trial
