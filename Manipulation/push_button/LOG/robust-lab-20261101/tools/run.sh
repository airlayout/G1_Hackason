#!/usr/bin/env bash
# Run from the operator PC. The camera stream remains on G1 PC2.
set -euo pipefail
SESSION=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$SESSION/../../.." && pwd)
cd "$REPO"
SDK_PY=/home/ubuntu/miniforge3/envs/tv/bin/python
VISION_PY="$REPO/_local/bottle_push/venv/bin/python"
MODE=${1:-help}
if [[ "$MODE" != preview && "$MODE" != precontact && "$MODE" != push ]]; then
  echo 'Usage: bash run.sh preview|precontact|push [robust CLI options]'
  echo 'Motion modes require an explicit --calibrated and interactive YES.'
  echo 'Verify physical B-model table placement and current calibration first.'
  exit 2
fi
shift
EXTRA=("$@")
CALIBRATED=false
for ARG in "${EXTRA[@]}"; do
  [[ "$ARG" != --calibrated ]] || CALIBRATED=true
  case "$ARG" in
    --execute|--output|--output=*|--state-json|--state-json=*|--arm-socket|--arm-socket=*|--task|--task=*|--motion-planner|--motion-planner=*|--trial-config|--trial-config=*)
      echo "run.sh manages this argument: $ARG" >&2; exit 2 ;;
  esac
done
if [[ "$MODE" != preview && "$CALIBRATED" != true ]]; then
  echo 'Check the current camera/hand calibration and table placement, then pass --calibrated.' >&2
  exit 2
fi
RUN=$(mktemp -d "$SESSION/${MODE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")
echo "Logs: $RUN"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl
export YOLO_CONFIG_DIR="$REPO/_local/bottle_push/ultralytics"
"$SDK_PY" "$SESSION/preflight.py" --trial-config "$SESSION/trial.json" --output-dir "$RUN" "${EXTRA[@]}"
ARGS=(Manipulation/push_button/real/run_push.py --task bottle --motion-planner robust
      --trial-config "$SESSION/trial.json" --output "$RUN/result.json")
if [[ "$MODE" == preview ]]; then
  "$VISION_PY" "${ARGS[@]}" --state-json "$RUN/state.json" "${EXTRA[@]}"
  exit $?
fi
SOCKET="$RUN/arm.sock"
BRIDGE_PID=
cleanup() {
  if [[ -n "$BRIDGE_PID" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill -TERM "$BRIDGE_PID"
    wait "$BRIDGE_PID" || true
  fi
}
trap cleanup EXIT
"$SDK_PY" -u Manipulation/push_button/real/arm_bridge.py --network-interface enp129s0 \
  --arm --socket "$SOCKET" >"$RUN/bridge.log" 2>&1 &
BRIDGE_PID=$!
for ((N=0; N<50; N++)); do
  [[ ! -S "$SOCKET" ]] || break
  kill -0 "$BRIDGE_PID" 2>/dev/null || { cat "$RUN/bridge.log" >&2; exit 1; }
  sleep .1
done
[[ -S "$SOCKET" ]] || { cat "$RUN/bridge.log" >&2; exit 1; }
[[ "$MODE" != precontact ]] || ARGS+=(--stop-before-contact)
"$VISION_PY" "${ARGS[@]}" --arm-socket "$SOCKET" --execute "${EXTRA[@]}"
