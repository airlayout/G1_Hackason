#!/usr/bin/env bash
# 配置後の G1 上で使う入口。VOICE_HOME/environment に実際の配置先を設定する。
set -euo pipefail
VOICE_HOME="${VOICE_HOME:-/home/unitree/g1-voice-control}"
set -a
source "$VOICE_HOME/environment"
set +a
command="${1:-status}"
shift || true
case "$command" in
  start)
    mode=dry-run
    if [[ "${1:-}" == --execute && $# == 1 ]]; then
      mode=execute
    elif [[ $# != 0 ]]; then
      echo "使い方: voice start [--execute]" >&2; exit 2
    fi
    systemctl --user stop g1-voice-control-listener g1-voice-control-tools
    # 停止時に残った、自分の待受のないソケットだけを片付ける。
    "$VOICE_MODEL_PYTHON" - "$VOICE_SOCKET" <<'PY'
import errno, os, pathlib, socket, stat, sys
path = pathlib.Path(sys.argv[1])
if path.exists():
    info = path.lstat()
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
        raise SystemExit("ソケットの所有者と種類を確認してください")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
        probe.settimeout(1)
        try:
            probe.connect(str(path))
        except OSError as exc:
            if exc.errno != errno.ECONNREFUSED:
                raise
            path.unlink()
        else:
            raise SystemExit("既存のツールサーバーが待受中です")
PY
    printf '%s\n' "$mode" > "$VOICE_HOME/config/mode"
    systemctl --user start g1-voice-control-listener
    echo "音声待受の起動を開始しました: $mode"
    ;;
  stop)
    systemctl --user stop g1-voice-control-listener g1-voice-control-tools
    ;;
  status)
    systemctl --user --no-pager --full status g1-voice-control-listener g1-voice-control-tools
    ;;
  logs)
    exec journalctl _SYSTEMD_USER_UNIT=g1-voice-control-listener.service \
      _SYSTEMD_USER_UNIT=g1-voice-control-tools.service -n 40 -f
    ;;
  tools|listener)
    mode="$(cat "$VOICE_HOME/config/mode")"
    flags=()
    case "$mode" in
      execute) flags=(--execute) ;;
      dry-run) ;;
      *) echo "不正な実行モードです: $mode" >&2; exit 2 ;;
    esac
    if [[ "$command" == tools ]]; then
      exec bash "$VOICE_REPO_ROOT/Voice/deploy/run_tools.sh" \
        --config "$VOICE_REPO_ROOT/Voice/configs/tools.all.json" --socket "$VOICE_SOCKET" \
        --onboard-heartbeat "${flags[@]}"
    fi
    exec "$VOICE_MODEL_PYTHON" "$VOICE_REPO_ROOT/Voice/onboard_voice.py" \
      --model "$VOICE_MODEL_PATH" --device cpu --torch-threads 4 \
      --config "$VOICE_TASK_CONFIG" --tools "$VOICE_REPO_ROOT/Voice/configs/tools.all.json" \
      --socket "$VOICE_SOCKET" --interface eth0 --processing-timeout 330 \
      --output-dir "$VOICE_HOME/runs" "${flags[@]}"
    ;;
  *) echo "使い方: voice {start [--execute]|stop|status|logs}" >&2; exit 2 ;;
esac
