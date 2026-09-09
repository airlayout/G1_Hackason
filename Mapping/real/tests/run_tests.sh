#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$REAL_DIR/python${PYTHONPATH:+:$PYTHONPATH}"

python3 -m unittest discover -s "$SCRIPT_DIR" -p 'test_*.py' -v
bash -n "$REAL_DIR/mapctl" "$REAL_DIR"/docker/*.sh "$REAL_DIR"/scripts/*.sh
docker compose --env-file "$REAL_DIR/.env.example" \
    -f "$REAL_DIR/compose.yaml" --profile onboard --profile raw --profile sim config --quiet
