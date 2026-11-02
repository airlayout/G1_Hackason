#!/usr/bin/env bash
# Explicit elevator-button press: current RGB-D -> four scenarios -> press -> return -> lamp check.
# Defaults: 15mm; farther +3mm accepts 11.75mm depth; nominal/left/right require 13.5mm.
# Uses bottle-equivalent monitoring and synchronous streaming; search stops at the first valid path.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$HERE/run_robust_button.sh" push "$@"
