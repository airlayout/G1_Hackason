#!/usr/bin/env bash
# Runs inside the raw image. Replay sensors only, record canonical scan/pose pairs.
set -euo pipefail
SESSION_ID="${MAPPING_SESSION_ID:?MAPPING_SESSION_ID is required}"
REPLAY_ID="${REPLAY_RECORD:?REPLAY_RECORD is required}"
OUTPUT_NAME="${MAP_OUTPUT_NAME:?MAP_OUTPUT_NAME is required}"
SESSION_DIR="${MAPPING_RUNS_DIR:-/runs}/$SESSION_ID"
REPLAY_DIR="$SESSION_DIR/derived/$REPLAY_ID"
mkdir -p "$SESSION_DIR/trajectory" "$REPLAY_DIR"

ros2 launch g1_mapping_bringup offline_mapping.launch.py &
LAUNCH_PID=$!
ros2 topic echo --csv /g1_mapping/odom > "$SESSION_DIR/trajectory/${OUTPUT_NAME%.pcd}_odom.csv" &
ODOM_PID=$!
ros2 bag record -o "$REPLAY_DIR/rosbag2" \
    /g1_mapping/cloud_registered /g1_mapping/sensor_pose /g1_mapping/odom &
RECORD_PID=$!

READY=false
for attempt in $(seq 60); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null || ! kill -0 "$RECORD_PID" 2>/dev/null; then
        echo '[ERROR] replay pipeline or recorder exited' >&2
        exit 1
    fi
    if ros2 service list | grep -q '^/g1_mapping/save_map$'; then READY=true; break; fi
    sleep 1
done
[[ "$READY" == true ]] || { echo '[ERROR] mapper did not become ready' >&2; exit 1; }
sleep 2

PLAY_ARGS=(--rate "${REPLAY_RATE:-1.0}")
[[ -n "${REPLAY_START:-}" ]] && PLAY_ARGS+=(--start-offset "$REPLAY_START")
PLAY_TOPICS=("$RAW_POINTS_TOPIC" "$RAW_IMU_TOPIC")
# Simulation sensor headers use the recorded /clock, not rosbag's wall recording time.
# Real recordings without /clock use rosbag's generated playback clock.
RECORDED_CLOCK="$(python3 - "$SESSION_DIR/raw/rosbag2" <<'PY'
import sqlite3
import sys
from pathlib import Path
found = False
for path in Path(sys.argv[1]).glob('*.db3'):
    with sqlite3.connect(f'file:{path}?mode=ro', uri=True) as db:
        found = bool(db.execute("SELECT 1 FROM messages m JOIN topics t ON m.topic_id=t.id "
                                "WHERE t.name='/clock' LIMIT 1").fetchone())
    if found:
        break
print('true' if found else 'false')
PY
)"
if [[ "$RECORDED_CLOCK" == true ]]; then
    PLAY_TOPICS+=(/clock)
else
    PLAY_ARGS+=(--clock 200)
fi
PLAY_COMMAND=(ros2 bag play "$SESSION_DIR/raw/rosbag2" "${PLAY_ARGS[@]}"
    --topics "${PLAY_TOPICS[@]}")
if [[ -n "${REPLAY_DURATION:-}" ]]; then
    PLAY_COMMAND=(timeout --signal=INT "$REPLAY_DURATION" "${PLAY_COMMAND[@]}")
fi
STATUS=0
"${PLAY_COMMAND[@]}" || STATUS=$?
if [[ "$STATUS" != 0 && "$STATUS" != 124 ]]; then
    echo "[ERROR] bag replay failed ($STATUS)" >&2
    exit "$STATUS"
fi
sleep 2
ros2 run g1_mapping_tools save_occupancy_map --timeout 45
cp "$SESSION_DIR/map/${OUTPUT_NAME%.pcd}.octomap.json" "$REPLAY_DIR/occupancy.json"

kill -TERM "$RECORD_PID" "$ODOM_PID" 2>/dev/null || true
wait "$RECORD_PID" 2>/dev/null || true
wait "$ODOM_PID" 2>/dev/null || true
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
for attempt in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
kill -9 "$LAUNCH_PID" 2>/dev/null || true
