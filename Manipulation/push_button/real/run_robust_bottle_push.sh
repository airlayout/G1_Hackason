#!/usr/bin/env bash
# Explicit full push: approach -> contact -> 5 cm push -> return -> release.
# Planning and execution both use the same 5 cm target and current calibration.
# Unmatched image/state timestamps are recaptured before planning; 3 passes start motion immediately.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$HERE/run_robust_bottle.sh" push "$@" --push-distance 0.05
