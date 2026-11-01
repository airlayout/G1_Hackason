#!/usr/bin/env bash
# Explicit elevator-button press: current RGB-D -> three scenarios -> press -> return -> lamp check.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$HERE/run_robust_button.sh" push "$@"
