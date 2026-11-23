#!/usr/bin/env bash
# OMEN で MediaMTX（スマホの映像の受け口）・送信ページ・表示のサーバーを起こす・止める・様子を見る
#
#   bash start_mediamtx.sh            起こす（起きていれば何もしない）
#   bash start_mediamtx.sh stop       止める（送信ページ・表示のサーバーも）
#
# 表示のサーバー（rerun --serve-web）は口を固定して常に動かす（viewer_hub.py）:
#   ブラウザ http://<IP>:9090/?url=ws://<IP>:9877、道具からは 127.0.0.1:9876
#   bash start_mediamtx.sh status     届いている流れ（名前・コーデック・解像度・受けたバイト数）
#
# 証明書は無ければ作る。OMEN の LAN の IP が変わったら作り直す（スマホの警告もやり直し）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${WORK:-$HOME/work_inoue/phone_slam}"
MTX_DIR="$WORK/mediamtx"
CFG="$HERE/mediamtx_phone.yml"
LOG="$MTX_DIR/mediamtx.log"
TS_IP="${TS_IP:-100.99.102.70}"   # Tailscale（Mac から見るとき）
PAGE_PORT="${PAGE_PORT:-8443}"     # 背面カメラ・H.264 で始まる送信ページ（publish/index.html）
PAGE_LOG="$MTX_DIR/publish_page.log"
HUB_PY="${HUB_PY:-$HOME/miniforge3/envs/droid/bin/python}"
HUB_LOG="$MTX_DIR/viewer_hub.log"
HUB_MEM="${HUB_MEM:-1GB}"   # 後からつないだブラウザのために抱える量（超えたら古い物から捨てる）

log() { printf '[mediamtx] %s\n' "$*"; }

lan_ip() {
  # Physical_AI_5G の 192.168.0.x を探す（無線の名前は機械ごとに違うので IP で選ぶ）
  ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -m1 '^192\.168\.0\.' || true
}

running_pid() { pgrep -x mediamtx -u "$(id -u)" | head -1 || true; }
page_pid() { pgrep -u "$(id -u)" -f "[s]erve_publish.py" | head -1 || true; }
# python -m rerun は rerun_cli/rerun に入れ替わる（「-m rerun」では見つからない）
hub_pid() { pgrep -u "$(id -u)" -f "[r]erun --serve-web" | head -1 || true; }

start_hub() {
  [[ -n "$(hub_pid)" ]] && return
  setsid nohup "$HUB_PY" -m rerun --serve-web --bind 0.0.0.0 --port 9876 --web-viewer-port 9090 --ws-server-port 9877 \
    --server-memory-limit "$HUB_MEM" --hide-welcome-screen >>"$HUB_LOG" 2>&1 </dev/null &
  sleep 2
  [[ -n "$(hub_pid)" ]] || { log "表示のサーバーが起きなかった。$HUB_LOG:"; tail -n 5 "$HUB_LOG"; }
}

start_page() {
  [[ -n "$(page_pid)" ]] && return
  setsid nohup python3 "$HERE/serve_publish.py" --cert "$MTX_DIR/server.crt" --key "$MTX_DIR/server.key" \
    --port "$PAGE_PORT" >"$PAGE_LOG" 2>&1 </dev/null &
  sleep 0.5
  [[ -n "$(page_pid)" ]] || { log "送信ページが起きなかった。$PAGE_LOG:"; tail -n 5 "$PAGE_LOG"; }
}

make_cert() {
  local ip="$1"
  if [[ -s "$MTX_DIR/server.crt" ]] && \
     openssl x509 -in "$MTX_DIR/server.crt" -noout -ext subjectAltName 2>/dev/null | grep -q "IP Address:$ip"; then
    return
  fi
  log "証明書を作る（IP $ip と $TS_IP）"
  openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
    -keyout "$MTX_DIR/server.key" -out "$MTX_DIR/server.crt" -subj "/CN=omen" \
    -addext "subjectAltName=IP:$ip,IP:$TS_IP,DNS:omen" 2>/dev/null
}

cmd_start() {
  [[ -x "$MTX_DIR/mediamtx" ]] || { log "$MTX_DIR/mediamtx が無い（README の 1 を先に）"; exit 1; }
  local pid; pid="$(running_pid)"
  if [[ -n "$pid" ]]; then log "もう動いている（pid $pid）"; start_page; start_hub; cmd_urls; return; fi
  local ip; ip="$(lan_ip)"
  [[ -n "$ip" ]] || { log "192.168.0.x の IP が無い（OMEN が Physical_AI_5G に居ない）"; exit 1; }
  make_cert "$ip"
  cd "$MTX_DIR"
  # 設定の webrtcAdditionalHosts は環境変数で上書きする（IP が変わっても設定を書き換えない）
  # 記録は追記する（起こし直しで前の記録を消さない）。詳しくは MTX_LOGLEVEL=debug bash start_mediamtx.sh
  MTX_WEBRTCADDITIONALHOSTS="$ip,$TS_IP" setsid nohup ./mediamtx "$CFG" >>"$LOG" 2>&1 </dev/null &
  sleep 1.5
  pid="$(running_pid)"
  if [[ -z "$pid" ]]; then log "起きなかった。$LOG の最後:"; tail -n 20 "$LOG"; exit 1; fi
  log "起こした（pid $pid・記録 $LOG）"
  start_page
  start_hub
  cmd_urls
}

cmd_urls() {
  local ip; ip="$(lan_ip)"
  echo "  スマホから送る: https://$ip:$PAGE_PORT/?cam=cam1   （背面・H.264 で始まる。2 台目は cam=cam2）"
  echo "      付属の送信ページ: https://$ip:8889/cam1/publish   （カメラとコーデックを手で選ぶ）"
  echo "  地図・較正の表示（口は固定）: http://$TS_IP:9090/?url=ws://$TS_IP:9877   （同じ Wi-Fi なら $ip）"
  echo "  生の映像を見る: https://$ip:8889/cam1   （Mac から Tailscale なら https://$TS_IP:8889/cam1）"
  echo "  OMEN の中で読む: rtsp://127.0.0.1:8554/cam1"
}

cmd_stop() {
  local hp; hp="$(hub_pid)"
  # python -m rerun は親と子（rerun_cli/rerun）の 2 つになるので両方止める
  [[ -n "$hp" ]] && pkill -u "$(id -u)" -f "[r]erun --serve-web" && log "表示のサーバーを止めた"
  local pp; pp="$(page_pid)"
  [[ -n "$pp" ]] && kill "$pp" && log "送信ページを止めた（pid $pp）"
  local pid; pid="$(running_pid)"
  if [[ -z "$pid" ]]; then log "動いていない"; return; fi
  kill "$pid"; log "止めた（pid $pid）"
}

cmd_status() {
  local pid; pid="$(running_pid)"
  [[ -n "$pid" ]] || { log "動いていない"; exit 1; }
  curl -s http://127.0.0.1:9997/v3/paths/list | python3 -c "$STATUS_PY"
}

# 届いている流れを 1 行ずつ出す（MediaMTX の API v3 の paths/list）
STATUS_PY='
import json, sys
items = json.load(sys.stdin).get("items", [])
if not items:
    print("  届いている流れは無い")
for p in items:
    name = p["name"]
    src = (p.get("source") or {}).get("type", "-")
    tracks = p.get("tracks2") or p.get("tracks") or []
    codecs = [t.get("codec", "?") if isinstance(t, dict) else t for t in tracks]
    mb = p.get("bytesReceived", 0) / 1e6
    readers = len(p.get("readers", []))
    print("  %s: ready=%s 送り手=%s トラック=%s 受けた=%.1f MB 読み手=%d" % (name, p.get("ready"), src, codecs, mb, readers))
'

case "${1:-start}" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  urls) cmd_urls ;;
  *) echo "usage: $0 [start|stop|status|urls]" >&2; exit 2 ;;
esac
