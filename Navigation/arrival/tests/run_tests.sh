#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
ARRIVAL_PYTHON="${PYTHON:-python3}"
if ! "$ARRIVAL_PYTHON" -c 'import numpy, pytest' 2>/dev/null; then
    "$ARRIVAL_PYTHON" -m pip install --quiet numpy pytest
fi
"$ARRIVAL_PYTHON" -m pytest Navigation/arrival/tests -q
