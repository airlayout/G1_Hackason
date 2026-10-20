#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/project_env.sh"
export PYTHONPATH="$E2E_PROJECT_PATH${PYTHONPATH:+:$PYTHONPATH}"
exec "$E2E_ROOT/.venv/bin/python" "$E2E_ROOT/examples/dds_controller.py" "$@"
