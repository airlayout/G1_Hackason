#!/usr/bin/env bash
# Console のテスト。実機も ssh も要らない。
set -euo pipefail
cd "$(dirname "$0")"
python3 -m unittest -v test_console test_camera_proc test_lidar test_local test_auth test_web test_motion test_tasks test_nav test_topics test_caution test_ctl_local test_hub_stop test_control
