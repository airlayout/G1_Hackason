#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
"${PYTHON:-python3}" -m pytest Manipulation/wrist_press/tests -q
