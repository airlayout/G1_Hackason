#!/usr/bin/env bash
# Current RGB-D -> nominal feedback simulation -> press -> return -> lamp check.
# Defaults: 15mm, shared bottle collection readiness, continuous joint correction.
# Target-position changes are logged; search stops at the first valid path.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$HERE/run_robust_button.sh" push "$@"
