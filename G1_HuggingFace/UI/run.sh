#!/usr/bin/env bash
# 実機 G1 のカメラに繋いだ状態で UI を一発で立ち上げる。
#
#   bash run.sh                  # 有線リンク → カメラ配信 → UI → ブラウザ
#   bash run.sh --no-browser     # URL を出すだけ
#   bash run.sh --device <名前>  # by-id の安定名を自分で指定する
#   bash run.sh --port 9000      # 以降の引数は UI 本体(Main/run.sh)へ渡す
#
# ⚠️ **航法はモックのまま。** 実機の Nav2 には繋がない（映像だけが本物）。
#    実 Nav2 に繋ぐときは README の「② ROS に繋ぐ」を使うこと。
#
# ⚠️ **Ctrl-C で G1 側のカメラ配信も止める。** 2026-09-23 に「操作PC 側だけ落として
#    機体に配信が残り、その後リンクが切れて止められなくなる」のを踏んだため。
#    既に動いていた配信も、このコマンドが止める（最後にその旨を出す）。
#
# ⚠️ **ROS 環境を source したシェルで叩かないこと**（Main/run.sh と同じ理由）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/../venv/bin/python"
HOST="${G1_UI_HOST:-g1}"                 # ssh の宛先(~/.ssh/config の別名)
LINK="${G1_UI_LINK:-g1-link}"            # nmcli の有線接続名
PC2_IP="${G1_UI_PC2_IP:-192.168.123.164}"
REMOTE_DIR=/home/unitree/g1_ui_camera
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8)
# ⚠️ pkill のパターンは括弧付きのまま使う。`camera_server.py` と書くと
#    ssh のコマンド行自身に当たって**自分のシェルごと落ちる**（README 参照）。
CAM_PATTERN='g1_ui_camer[a]'

OPEN_BROWSER=1
DEVICE=""
PORT_OVERRIDE=""
UI_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-browser) OPEN_BROWSER=0; shift ;;
    --device)     DEVICE="${2:?--device には名前が要る}"; shift 2 ;;
    --host)       HOST="${2:?--host には宛先が要る}"; shift 2 ;;
    # ⚠️ **UI 本体へ素通しするだけでは足りない**(2026-09-24 に発見)。
    #    待ち受けの確認・ポートの空き検査・片付けが config.yaml の値のままになり、
    #    「UI は 9000 で立っているのに 8090 を 60 秒叩いて die する」になっていた。
    --port)       UI_ARGS+=("$1" "${2:?--port には番号が要る}")
                  PORT_OVERRIDE="$2"; shift 2 ;;
    -h|--help)    sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)            UI_ARGS+=("$1"); shift ;;
  esac
done

step() { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }
ok()   { printf '  \033[32m✅\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m⚠️\033[0m  %s\n' "$1"; }
die()  { printf '  \033[31m❌ %s\033[0m\n' "$1" >&2; exit 1; }

UI_PID=""
REMOTE_READY=0   # ssh が通ってから 1。ここに来る前の失敗では機体側を触らない
cleanup() {
  trap - INT TERM EXIT
  step "片付け"
  if [[ -n "$UI_PID" ]] && kill -0 "$UI_PID" 2>/dev/null; then
    kill "$UI_PID" 2>/dev/null || true
    for _ in $(seq 20); do kill -0 "$UI_PID" 2>/dev/null || break; sleep 0.2; done
    ok "UI を止めた"
  fi
  # 保険。ポートが残っているなら本体が別プロセスで生きている
  if ss -lnt 2>/dev/null | grep -q ":${UI_PORT:-8090} "; then
    pkill -f 'UI/Main/server.p[y]' 2>/dev/null || true
    warn "UI の待ち受けが残っていたので落とした"
  fi
  if [[ "$REMOTE_READY" != "1" ]]; then
    printf '  機体側には何も触っていない\n'
    return
  fi
  # ⚠️ ここでリンクが既に切れていることがある（ケーブルを抜かれた等）。
  #    そのときは止められないので、**黙って成功したことにしない**。
  if timeout 20 "${SSH[@]}" "$HOST" "pkill -f '$CAM_PATTERN'" >/dev/null 2>&1; then
    ok "G1 のカメラ配信を止めた"
  elif timeout 20 "${SSH[@]}" "$HOST" "pgrep -f '$CAM_PATTERN' >/dev/null" >/dev/null 2>&1; then
    warn "G1 のカメラ配信が残っている。次のどちらかで止めること:"
    printf "     ssh %s 'pkill -f %s'\n" "$HOST" "$CAM_PATTERN"
  else
    ok "G1 のカメラ配信は動いていない"
  fi
  printf '  有線リンク(%s)は上げたままにしてある。切るなら nmcli connection down %s\n' "$LINK" "$LINK"
}
trap cleanup INT TERM EXIT

# --- 0. 前提 ---------------------------------------------------------------
step "0. 前提の確認"
[[ -x "$VENV" ]] || die "venv が無い: $VENV（G1_HuggingFace/README.md の手順で作る）"
"$VENV" -c "import ultralytics" 2>/dev/null \
  || die "ultralytics が入っていない: $VENV -m pip install -r ../../Perception/requirements.txt"

# config から映像ソースと待ち受け先を読む（UI 本体と同じ設定を見るため）
# ⚠️ **`|| true` を外さないこと。** set -e 下では read が EOF で 1 を返した時点で
#    スクリプトが即死し、下の die に**到達しない**。config が壊れていると
#    「無言で終了コード 1」になって原因が分からなくなる(2026-09-24 に実測)。
VIDEO_SOURCE=""; UI_HOST=""; UI_PORT=""
read -r VIDEO_SOURCE UI_HOST UI_PORT < <("$VENV" - "$HERE/Main/config.yaml" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
print(cfg["video"]["source"], cfg["server"]["host"], cfg["server"]["port"])
PY
) || true
[[ -n "$VIDEO_SOURCE" && -n "$UI_HOST" && -n "$UI_PORT" ]] \
  || die "config.yaml を読めなかった: $HERE/Main/config.yaml"
# ①の --port はここで効かせる(UI_URL とポート検査より前)
UI_PORT="${PORT_OVERRIDE:-$UI_PORT}"
if [[ "$VIDEO_SOURCE" != "zmq" ]]; then
  die "config.yaml の video.source が '$VIDEO_SOURCE' になっている。実機に繋ぐなら 'zmq' にすること"
fi
UI_URL="http://$UI_HOST:$UI_PORT"
if ss -lnt 2>/dev/null | grep -q ":$UI_PORT "; then
  die "$UI_PORT が既に使われている（UI が二重に上がる）。先に止めるか --port で変えること"
fi
ok "venv・ultralytics・video.source=zmq"

# --- 1. 有線リンク ----------------------------------------------------------
# ⚠️ ケーブルを挿しただけでは IP が付かない。2026-09-23 は 1 セッション中に 2 回落ちた。
step "1. 有線リンク"
if ping -c1 -W2 "$PC2_IP" >/dev/null 2>&1; then
  ok "$PC2_IP に届いている"
else
  # ⚠️ --wait を付けないと、ケーブルが抜けているとき既定の 90 秒待たされる
  nmcli --wait 10 connection up "$LINK" >/dev/null 2>&1 \
    || warn "nmcli connection up $LINK に失敗した（ケーブルが抜けている可能性）"
  # ⚠️ NetworkManager は「接続済み」と言いながら IP だけ付けていることがある。
  #    ケーブルが抜けているとこうなる（2026-09-23 実測）ので、物理リンクを直接見る。
  IFACE="$(nmcli -g connection.interface-name connection show "$LINK" 2>/dev/null || true)"
  if [[ -n "$IFACE" && "$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)" != "1" ]]; then
    die "$IFACE にリンクが無い（carrier=0）。**LAN ケーブルが抜けているか、G1 側が落ちている**"
  fi
  ping -c3 -W2 "$PC2_IP" >/dev/null 2>&1 \
    || die "$PC2_IP に届かない。ケーブルと $LINK を確認すること（ip -br addr）"
  ok "$LINK を上げた（$PC2_IP に届いた）"
fi
timeout 20 "${SSH[@]}" "$HOST" true >/dev/null 2>&1 || die "ssh $HOST に入れない"
REMOTE_READY=1
ok "ssh $HOST"

# --- 2. カメラ配信（G1 本体） ----------------------------------------------
step "2. G1 のカメラ配信"
if timeout 20 "${SSH[@]}" "$HOST" "pgrep -f '$CAM_PATTERN' >/dev/null"; then
  ok "既に動いている（使い回す。⚠️ 終了時にはこれも止める）"
else
  # 配信スクリプトを最新にする（リポジトリ側が正）
  LOCAL_MD5="$(md5sum "$HERE/Camera/camera_server.py" | cut -d' ' -f1)"
  REMOTE_MD5="$(timeout 20 "${SSH[@]}" "$HOST" "md5sum $REMOTE_DIR/camera_server.py 2>/dev/null | cut -d' ' -f1" || true)"
  if [[ "$LOCAL_MD5" != "$REMOTE_MD5" ]]; then
    timeout 30 "${SSH[@]}" "$HOST" "mkdir -p $REMOTE_DIR"
    scp -q "$HERE/Camera/camera_server.py" "$HOST:$REMOTE_DIR/"
    ok "camera_server.py を送った"
  fi

  # ⚠️ **/dev/videoN で指定しない。** USB が再認識されると番号が変わる
  #    （2026-09-23 に配信中の video6 → video7 を踏んだ）。by-id の安定名を使う。
  if [[ -z "$DEVICE" ]]; then
    mapfile -t FOUND < <(timeout 20 "${SSH[@]}" "$HOST" \
      "ls /dev/v4l/by-id/*webcam*-video-index0 2>/dev/null" || true)
    case "${#FOUND[@]}" in
      1) DEVICE="${FOUND[0]}" ;;
      0) die "by-id に webcam が無い。ssh $HOST 'python3 $REMOTE_DIR/camera_server.py --list' で確認し --device で渡すこと" ;;
      *) die "webcam が ${#FOUND[@]} 台ある。--device で選ぶこと: ${FOUND[*]}" ;;
    esac
  fi
  timeout 30 "${SSH[@]}" "$HOST" \
    "nohup setsid python3 $REMOTE_DIR/camera_server.py --device '$DEVICE' \
     > $REMOTE_DIR/camera.log 2>&1 < /dev/null & echo ok" >/dev/null
  ok "起動した: $(basename "$DEVICE")"
fi

# 配信が本当に立ったかを ZMQ の待ち受けで確かめる（プロセスの有無だけでは足りない）
for _ in $(seq 20); do
  timeout 10 "${SSH[@]}" "$HOST" "ss -lnt 2>/dev/null | grep -q ':5555 '" && break
  sleep 0.5
done || true
if timeout 10 "${SSH[@]}" "$HOST" "ss -lnt 2>/dev/null | grep -q ':5555 '"; then
  ok "5555 で待ち受けている"
else
  timeout 10 "${SSH[@]}" "$HOST" "tail -5 $REMOTE_DIR/camera.log" || true
  die "配信が立たなかった（上は $REMOTE_DIR/camera.log の末尾）"
fi

# --- 3. UI 本体 -------------------------------------------------------------
step "3. UI"
bash "$HERE/Main/run.sh" "${UI_ARGS[@]+"${UI_ARGS[@]}"}" &
UI_PID=$!
for _ in $(seq 120); do
  curl -sf -m 2 "$UI_URL/api/state" >/dev/null 2>&1 && break
  kill -0 "$UI_PID" 2>/dev/null || die "UI が落ちた（上のログを見ること）"
  sleep 0.5
done
curl -sf -m 3 "$UI_URL/api/state" >/dev/null 2>&1 || die "UI が $UI_URL に応答しない"
ok "$UI_URL で応答している"

# --- 4. 実機の絵が届いているかを確かめる ------------------------------------
# ⚠️ ここが肝。UI が上がっただけでは「実機の映像が来ている」ことにはならない。
step "4. 実機の映像"
VISION=""
for _ in $(seq 20); do
  VISION="$(curl -sf -m 3 "$UI_URL/api/state" | "$VENV" -c '
import json, sys
v = json.load(sys.stdin).get("vision", {})
print(int(bool(v.get("alive"))), v.get("fps") or 0)
' 2>/dev/null || true)"
  [[ "${VISION%% *}" == "1" && "${VISION##* }" != "0" ]] && break
  sleep 0.5
done
if [[ "${VISION%% *}" == "1" ]]; then
  ok "届いている（${VISION##* } fps）"
else
  warn "まだ届いていない。$REMOTE_DIR/camera.log と UI のログを見ること"
fi

# --- 5. ブラウザ ------------------------------------------------------------
if [[ "$OPEN_BROWSER" == "1" ]] && command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$UI_URL" >/dev/null 2>&1 || warn "ブラウザを開けなかった。$UI_URL を自分で開くこと"
fi

printf '\n\033[1m%s\033[0m を開く。Ctrl-C で UI と G1 の配信をまとめて止める\n' "$UI_URL"
printf '\033[33m⚠️\033[0m  航法はモック。画面の地図・現在地・巡回路・状態は**作り物**（本物は映像だけ）\n'
wait "$UI_PID"
