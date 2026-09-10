#!/usr/bin/env bash
# Quest 用の WiFi を、ホスト PC 自身がアクセスポイントになって提供する。
#
#   ./scripts/quest_ap.sh up                 # AP を立てる（SSID/パスワード/URL を表示）
#   ./scripts/quest_ap.sh up --channel 1     # 2.4GHz のチャネルを指定（既定: 空いている方）
#   ./scripts/quest_ap.sh status             # 接続中の端末と信号強度
#   ./scripts/quest_ap.sh down               # AP を止める（元の WiFi に戻る）
#
# なぜこれが要るのか（docs/02-network.md）:
#   施設の WiFi はクライアント分離で Quest → PC が通らない。スマホのテザリングは
#   チャネルを選べず、2.4GHz の混雑帯に入ると RTT が 200ms を超える。6GHz 単独の
#   テザリングは PC 側から発見できない。PC 自身が AP になれば 1 ホップ・分離なし・
#   チャネル選択可で、同じ部屋なら RTT は一桁〜数十 ms に収まる。
#
# 制約:
#   ・AP 中、この PC はインターネットに出られない（WiFi が 1 枚しか無いため）。
#     ロボット制御は有線なので影響しない。
#   ・多くのアダプタは 5GHz で AP を立てられない（規制上 NO-IR）。2.4GHz で立てる。
#   ・要 sudo（nmcli）。

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

CON="g1-teleop-ap"
SSID="${G1_AP_SSID:-G1TELEOP}"
CHANNEL=""
PASSWORD="${G1_AP_PASSWORD:-}"
CMD="${1:-}"; shift || true

while [ $# -gt 0 ]; do
  case "$1" in
    --channel)  CHANNEL="$2"; shift ;;
    --ssid)     SSID="$2"; shift ;;
    --password) PASSWORD="$2"; shift ;;
    *) _die "不明な引数: $1" ;;
  esac
  shift
done

load_config
DEV="$(detect_wifi_iface)"
[ -n "$DEV" ] || _die "WiFi インターフェースが見つかりません。config/g1.env の G1_WIFI_IFACE を設定してください。"

pick_channel() {
  # 2.4GHz の 1 / 6 / 11 のうち、見えている AP が最も少ないものを選ぶ。
  local best=11 min=999999 c n
  for c in 1 6 11; do
    n="$(nmcli -f CHAN dev wifi list --rescan no 2>/dev/null | awk -v ch="$c" 'NR>1 && $1==ch' | wc -l)"
    if [ "$n" -lt "$min" ]; then min="$n"; best="$c"; fi
  done
  echo "$best"
}

quest_url() {
  local ip
  ip="$(ip -4 -br addr show "$DEV" 2>/dev/null | grep -oP '\d+\.\d+\.\d+\.\d+(?=/)' | head -1)"
  [ -n "$ip" ] && echo "https://${ip}:8012/?ws=wss://${ip}:8012"
}

case "$CMD" in
  up)
    sudo -v || _die "sudo に失敗しました。"
    [ -n "$CHANNEL" ] || CHANNEL="$(pick_channel)"
    [ -n "$PASSWORD" ] || PASSWORD="$(tr -dc 'a-z0-9' </dev/urandom | head -c 12)"
    sudo nmcli connection delete "$CON" >/dev/null 2>&1 || true
    sudo nmcli device wifi hotspot ifname "$DEV" con-name "$CON" ssid "$SSID" password "$PASSWORD" >/dev/null
    sudo nmcli connection modify "$CON" 802-11-wireless.band bg 802-11-wireless.channel "$CHANNEL"
    sudo nmcli connection up "$CON" >/dev/null
    sleep 2
    _ok "AP を立てました"
    cat <<EOF

  SSID       : $SSID
  パスワード : $PASSWORD
  チャネル   : $CHANNEL (2.4GHz)
  PC の IP   : $(ip -4 -br addr show "$DEV" | awk '{print $3}')

  Quest を上の SSID に接続してください（「インターネットなし」でも接続を維持）。
  Quest のブラウザで開く URL:
    $(quest_url)

  ※ この PC はインターネットに出られなくなります。戻すには: $0 down
EOF
    ;;
  down)
    sudo -v || _die "sudo に失敗しました。"
    sudo nmcli connection down "$CON" >/dev/null 2>&1 || true
    _ok "AP を止めました（保存済みの WiFi に自動で戻ります）"
    ;;
  status)
    if ! nmcli -t -f NAME,DEVICE connection show --active | grep -q "^$CON:"; then
      echo "AP は動いていません。起動: $0 up"; exit 1
    fi
    echo "AP: $SSID  on $DEV"
    ip -4 -br addr show "$DEV"
    echo; echo "接続中の端末:"
    if command -v iw >/dev/null; then
      sudo iw dev "$DEV" station dump 2>/dev/null \
        | awk '/^Station/{mac=$2} /signal:/{sig=$2} /connected time/{print "  " mac "  " sig " dBm  " $3 " 秒"}'
    fi
    ip neigh show dev "$DEV" | awk '$0 !~ /FAILED|INCOMPLETE/ {print "  " $1 "  " $3}'
    echo; echo "Quest のブラウザで開く URL:"; echo "  $(quest_url)"
    echo; echo "遅延の確認: ping -c 20 -i 0.2 <Quest の IP>   （平均 10ms 未満なら健全）"
    ;;
  *)
    sed -n '2,8p' "$0"; exit 1 ;;
esac
