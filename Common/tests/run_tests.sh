#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMON_DIR="$(cd "$HERE/.." && pwd)"

bash -n "$COMMON_DIR"/network/*.sh
python3 -m py_compile "$COMMON_DIR"/network/*.py

bash "$HERE/test_setup_ethernet_for_g1.sh"
