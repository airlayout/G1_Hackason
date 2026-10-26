#!/usr/bin/env bash
# Quest 用の WiFi を、ホスト PC 自身がアクセスポイントになって提供する。
#
#   ./scripts/quest_ap.sh up                 # AP を立てる（SSID/パスワード/URL を表示）
#   ./scripts/quest_ap.sh up --channel 1     # 2.4GHz のチャネルを指定（既定: 空いている方）
#   ./scripts/quest_ap.sh up --address 10.0.42.1  # AP 側の PC の IP を指定
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
#   ・nmcli の操作権限が要る。polkit で許可されていれば sudo は不要
#     （`nmcli general permissions` で settings.modify.system が yes なら通る）。
#   ・PC 自身の IP は config/g1.env の G1_AP_ADDRESS（未設定なら NetworkManager の
#     既定 10.42.0.1）。Quest には同じ /24 から DHCP で配られる。

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

CON="g1-teleop-ap"
# 既定値は load_config（config/g1.env の読み込み）の後に入れる。
# ここで環境変数を読んでしまうと、設定ファイルの値が無視される。
SSID=""
CHANNEL=""
PASSWORD=""
ADDRESS=""
CMD="${1:-}"; shift || true

while [ $# -gt 0 ]; do
  case "$1" in
    --channel)  CHANNEL="$2"; shift ;;
    --ssid)     SSID="$2"; shift ;;
    --password) PASSWORD="$2"; shift ;;
    --address)  ADDRESS="$2"; shift ;;
    *) _die "不明な引数: $1" ;;
  esac
  shift
done

load_config
# 優先順位: コマンドライン引数 > config/g1.env > 組み込みの既定値
[ -n "$SSID" ]     || SSID="${G1_AP_SSID:-G1TELEOP}"
[ -n "$PASSWORD" ] || PASSWORD="$G1_AP_PASSWORD"
[ -n "$ADDRESS" ]  || ADDRESS="$G1_AP_ADDRESS"
DEV="$(detect_wifi_iface)"
[ -n "$DEV" ] || _die "WiFi インターフェースが見つかりません。config/g1.env の G1_WIFI_IFACE を設定してください。"

# nmcli をどう呼ぶかを決める。
#
# NetworkManager の操作は polkit で許可されていれば sudo は要らない。
# settings.modify.system と network-control が yes なら素の nmcli で通る。
# SSH 経由では sudo が tty を持てず失敗するので、権限を見て使い分ける。
NM_SUDO=""
nm_init() {
  local perms
  perms="$(nmcli general permissions 2>/dev/null)"
  if echo "$perms" | grep -Eq "settings\.modify\.system +yes" \
     && echo "$perms" | grep -Eq "network-control +yes"; then
    _ok "NetworkManager の操作は polkit で許可されています（sudo 不要）"
    NM_SUDO=""
    return
  fi
  NM_SUDO="sudo"
  sudo -v || _die "sudo に失敗しました。
     デスクトップのネイティブ端末（Ctrl+Alt+T）で実行してください。
     SSH 経由では sudo が tty を持てません。"
}

# nmcli の実行（必要なときだけ sudo を付ける）
nm() {
  if [ -n "$NM_SUDO" ]; then sudo nmcli "$@"; else nmcli "$@"; fi
}

# AP 側のアドレスを "IP/プレフィクス" に正規化する。
# 空なら何も出さない（NetworkManager の既定 10.42.0.1/24 のままにする）。
normalize_address() {
  local a="$1"
  [ -n "$a" ] || return 0
  [[ "$a" == */* ]] || a="$a/24"
  [[ "$a" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}/[0-9]{1,2}$ ]] \
    || _die "G1_AP_ADDRESS の書式が不正です: $1
     例: 10.0.42.1 または 10.0.42.1/24"
  # ロボットの有線と衝突すると DDS の経路が壊れる
  case "$a" in
    192.168.123.*) _die "G1_AP_ADDRESS がロボットの有線サブネットと衝突します: $1
     192.168.123.0/24 以外を選んでください（例: 10.0.42.1）。" ;;
  esac
  echo "$a"
}

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
    nm_init
    [ -n "$CHANNEL" ] || CHANNEL="$(pick_channel)"
    if [ -n "$PASSWORD" ]; then
      # WPA2 は 8〜63 文字。範囲外だと nmcli が分かりにくい形で失敗する。
      [ "${#PASSWORD}" -ge 8 ] && [ "${#PASSWORD}" -le 63 ] \
        || _die "G1_AP_PASSWORD の長さが ${#PASSWORD} 文字です。WPA2 は 8〜63 文字です。"
    else
      PASSWORD="$(tr -dc 'a-z0-9' </dev/urandom | head -c 12)"
      _warn "G1_AP_PASSWORD が未設定なので、今回だけのパスワードを生成しました。
     毎回同じにするには config/g1.env の G1_AP_PASSWORD を設定してください。"
    fi
    nm connection delete "$CON" >/dev/null 2>&1 || true
    nm device wifi hotspot ifname "$DEV" con-name "$CON" ssid "$SSID" password "$PASSWORD" >/dev/null
    nm connection modify "$CON" 802-11-wireless.band bg 802-11-wireless.channel "$CHANNEL"
    # hotspot は ipv4.method=shared で作られる。アドレスを明示すると
    # その /24 を dnsmasq が配るので、URL を毎回同じにできる。
    ADDRESS_CIDR="$(normalize_address "$ADDRESS")"
    if [ -n "$ADDRESS_CIDR" ]; then
      nm connection modify "$CON" ipv4.method shared ipv4.addresses "$ADDRESS_CIDR"
    fi
    nm connection up "$CON" >/dev/null
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
    nm_init
    nm connection down "$CON" >/dev/null 2>&1 || true
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
      sudo -n iw dev "$DEV" station dump 2>/dev/null \
        | awk '/^Station/{mac=$2} /signal:/{sig=$2} /connected time/{print "  " mac "  " sig " dBm  " $3 " 秒"}'
    fi
    ip neigh show dev "$DEV" | awk '$0 !~ /FAILED|INCOMPLETE/ {print "  " $1 "  " $3}'
    echo; echo "Quest のブラウザで開く URL:"; echo "  $(quest_url)"
    echo; echo "遅延の確認: ping -c 20 -i 0.2 <Quest の IP>   （平均 10ms 未満なら健全）"
    ;;
  *)
    sed -n '2,8p' "$0"; exit 1 ;;
esac
