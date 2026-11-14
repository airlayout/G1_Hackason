#!/usr/bin/env bash
# Fetch a finished G1 recording, build a map locally, and copy the map back.
set -euo pipefail
REAL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SESSION="${1:?usage: reconstruct_onboard.sh SESSION_ID}"
[[ "$SESSION" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid session ID' >&2; exit 1; }
REMOTE="${G1_RECORD_SSH:-unitree@100.78.135.14}"
LOCAL="$REAL_DIR/runs/$SESSION"
[[ ! -e "$LOCAL" ]] || { echo "Local session already exists: $LOCAL" >&2; exit 1; }
ssh "$REMOTE" "test ! -e /home/unitree/mapping_runs/$SESSION/map/map_fastlio.pcd" || { echo 'Remote output already exists or G1 is unavailable' >&2; exit 1; }
rsync -a "$REMOTE:/home/unitree/mapping_runs/$SESSION/" "$LOCAL/"
/usr/bin/python3 - "$LOCAL/state.json" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    state = json.load(f)
if state.get('status') != 'completed':
    raise SystemExit('G1 recording was not completed successfully')
PY
"$REAL_DIR/quickstart/replay_onboard_native.sh" "$SESSION"
[[ -s "$LOCAL/map/map_fastlio.pcd" ]] || { echo 'Map output missing' >&2; exit 1; }
rsync -a --ignore-existing "$LOCAL/map/" "$REMOTE:/home/unitree/mapping_runs/$SESSION/map/"
rsync -a --ignore-existing "$LOCAL/trajectory/" "$REMOTE:/home/unitree/mapping_runs/$SESSION/trajectory/"
rsync -a --ignore-existing "$LOCAL/derived/" "$REMOTE:/home/unitree/mapping_runs/$SESSION/derived/"
echo "[DONE] G1 map: /home/unitree/mapping_runs/$SESSION/map/map_fastlio.pcd"
