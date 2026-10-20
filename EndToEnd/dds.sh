#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
e2e_launch sim/dds.py "$@"
