#!/usr/bin/env bash
# テレオペ用ネットワークの切り替え — Mac から OMEN と G1 の両方を SSH で操作する。
#
#   ./net.sh status         3つの面の現状を出す（変更しない）
#   ./net.sh teleop         AP を上げ、G1 を AP に寄せる（テレオペ前）
#   ./net.sh internet       G1 を Fujitsu に戻し、AP を下ろす（開発・apt/pip のとき）
#   ./net.sh ap-up          AP だけ上げる          ./net.sh ap-down   AP だけ下ろす
#   ./net.sh g1-ap          G1 だけ AP に寄せる     ./net.sh g1-net    G1 だけ Fujitsu に戻す
#   ./net.sh dds            DDS の疎通を確認する（キットの preflight を呼ぶ・やや遅い）
#
# 構成（docs/plan/g1-stack-architecture.html の「図 1b」）:
#   面A  Mac en0 ── Fujitsu ── Internet         （G1 も必要なときだけ借りる）
#   面B  Mac en8 ── スイッチ ── OMEN ── G1      有線 192.168.123.0/24 · 制御は必ずここ
#   面C  Quest / G1 wlan0 ── OMEN の AP         2.4GHz ch6 · 上流なし
#
# 触らないもの: 有線（面B）。制御・DDS・LiDAR は常に有線に乗る。
# キットの scripts/quest_ap.sh は使わない。あれは connection delete → hotspot で
# プロファイルを作り直し、パスフレーズを再生成してしまう。既存プロファイルを流用する。

set -uo pipefail

OMEN="${G1_OMEN_SSH:-ubuntu@192.168.123.200}"
G1="${G1_PC2_SSH:-g1}"
AP_CON="${G1_AP_CON:-g1-teleop-ap}"
G1_AP_CON="${G1_AP_CLIENT_CON:-g1-teleop-client}"
G1_NET_CON="${G1_NET_CON:-Fujitsu_free_Wi-Fi}"
AP_SSID="${G1_AP_SSID:-G1TELEOP}"
OMEN_WIFI="${G1_OMEN_WIFI:-wlp128s20f3}"
G1_WIFI="${G1_PC2_WIFI:-wlan0}"
WIRED_HOSTS="${G1_WIRED_HOSTS:-192.168.123.161 192.168.123.164 192.168.123.120}"
G1_WIFI_MAC="${G1_PC2_WIFI_MAC:-fc:23:cd:92:9e:32}"
G1_WIFI_IP_FALLBACK="${G1_PC2_WIFI_IP:-10.42.0.76}"
G1_KEY="${G1_PC2_KEY:-${HOME}/.ssh/id_ed25519_g1}"
G1_USER="${G1_PC2_USER:-unitree}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8)
RESCAN_COOLDOWN=12   # nmcli は連続 rescan を拒否する。12 秒以上空ける

c_ok=$'\033[32m'; c_ng=$'\033[31m'; c_wn=$'\033[33m'; c_b=$'\033[1m'; c_z=$'\033[0m'
ok()   { printf '  %s[OK]%s   %s\n'   "${c_ok}" "${c_z}" "$*"; }
ng()   { printf '  %s[NG]%s   %s\n'   "${c_ng}" "${c_z}" "$*"; }
warn() { printf '  %s[WARN]%s %s\n'   "${c_wn}" "${c_z}" "$*"; }
head_() { printf '\n%s=== %s ===%s\n' "${c_b}" "$*" "${c_z}"; }
die()  { ng "$*"; exit 1; }

omen() { ssh "${SSH_OPTS[@]}" "${OMEN}" "$1" 2>&1; }

# G1 へは 2 経路ある。有線が落ちても AP 経由なら OMEN を踏み台にして届く。
# 2026-09-14 に実際これで救われた（立たせた動作で有線が抜け、無線だけが残った）。
G1_SSH=("${SSH_OPTS[@]}" "${G1}")
G1_VIA="未解決"
g1() { ssh "${G1_SSH[@]}" "$1" 2>&1; }

resolve_g1() {
  if ssh "${SSH_OPTS[@]}" "${G1}" true >/dev/null 2>&1; then
    G1_SSH=("${SSH_OPTS[@]}" "${G1}"); G1_VIA="有線"; return 0
  fi
  # AP のリースから G1 の無線アドレスを引く（DHCP なので固定しない）
  local ip
  ip="$(omen "ip neigh show dev ${OMEN_WIFI} | awk '\$3==\"${G1_WIFI_MAC}\" {print \$1}' | head -1")"
  case "${ip}" in 10.42.0.*) : ;; *) ip="${G1_WIFI_IP_FALLBACK}" ;; esac
  if ssh "${SSH_OPTS[@]}" -J "${OMEN}" -i "${G1_KEY}" "${G1_USER}@${ip}" true >/dev/null 2>&1; then
    G1_SSH=("${SSH_OPTS[@]}" -J "${OMEN}" -i "${G1_KEY}" "${G1_USER}@${ip}")
    G1_VIA="無線 ${ip}（OMEN 経由）"
    warn "有線で G1 に届かない。無線経路にフォールバックした — ケーブルとスイッチのポートを確認する"
    warn "この状態では DDS・制御・テレオペは動かない（OMEN が .161 に届かないため）"
    return 0
  fi
  return 1
}

# ---------- 前提の確認 ----------

check_ssh() {
  omen 'true' >/dev/null 2>&1 || die "OMEN (${OMEN}) に鍵で SSH できない。~/.ssh/config と鍵を確認する"
  resolve_g1 || die "G1 に有線 (${G1}) でも無線 (OMEN 経由) でも SSH できない。電源とケーブル、~/.ssh/config の Host g1 を確認する"
}

# polkit が無いと nmcli は "Not authorized" で黙って失敗する。今日いちばん時間を食った所。
check_polkit() {
  local host="$1" need missing
  case "${host}" in
    omen) need="network-control wifi.share.protected"
          missing="$(omen 'nmcli general permissions' | awk '
            /network-control/        && $2!="yes" {print "network-control"}
            /wifi.share.protected/   && $2!="yes" {print "wifi.share.protected"}')" ;;
    g1)   need="network-control"
          missing="$(g1 'nmcli general permissions' | awk '
            /network-control/        && $2!="yes" {print "network-control"}')" ;;
  esac
  [ -z "${missing}" ] && return 0

  ng "${host} の polkit 権限が足りない: ${missing}"
  if [ "${host}" = omen ]; then
    cat <<'EOF'
  OMEN で 1 回だけ実行する（polkit 1.x = JS ルール）:
    sudo tee /etc/polkit-1/rules.d/50-nm-sudo-group.rules >/dev/null <<'RULE'
    polkit.addRule(function(action, subject) {
        if (subject.isInGroup("sudo") &&
            (action.id == "org.freedesktop.NetworkManager.network-control" ||
             action.id == "org.freedesktop.NetworkManager.wifi.share.protected" ||
             action.id == "org.freedesktop.NetworkManager.wifi.share.open" ||
             action.id == "org.freedesktop.NetworkManager.settings.modify.system")) {
            return polkit.Result.YES;
        }
    });
    RULE
EOF
  else
    cat <<'EOF'
  G1 で 1 回だけ実行する（polkit 0.105 = .pkla 形式。JS ルールは効かない）:
    sudo tee /etc/polkit-1/localauthority/50-local.d/50-g1-nm.pkla >/dev/null <<'RULE'
    [Allow NetworkManager control for sudo group]
    Identity=unix-group:sudo
    Action=org.freedesktop.NetworkManager.network-control;org.freedesktop.NetworkManager.wifi.scan;org.freedesktop.NetworkManager.settings.modify.system
    ResultAny=yes
    RULE
EOF
  fi
  exit 1
}

# ---------- 面C: AP ----------

ap_up() {
  check_polkit omen
  if omen "nmcli -t -f NAME connection show --active" | grep -qx "${AP_CON}"; then
    ok "AP は既に稼働中（${AP_CON}）"
  else
    local out; out="$(omen "nmcli connection up ${AP_CON}")"
    case "${out}" in
      *successfully*) : ;;
      *) die "AP の起動に失敗: ${out}" ;;
    esac
  fi
  local info; info="$(omen "iw dev ${OMEN_WIFI} info")"
  local ssid chan
  ssid="$(printf '%s\n' "${info}" | awk '/ssid/{print $2}')"
  chan="$(printf '%s\n' "${info}" | sed -n 's/.*channel \([0-9]*\) (\([0-9]*\) MHz).*/\1 (\2 MHz)/p')"
  [ "${ssid}" = "${AP_SSID}" ] || die "AP の SSID が想定と違う: ${ssid:-なし}"
  ok "AP 稼働: ${ssid} / ch ${chan}"
  warn "この間 OMEN はインターネットに出られない（無線が 1 枚しかない）"
}

ap_down() {
  check_polkit omen
  if omen "nmcli -t -f NAME connection show --active" | grep -qx "${AP_CON}"; then
    omen "nmcli connection down ${AP_CON}" >/dev/null
  fi
  sleep 6
  local conn; conn="$(omen "nmcli -t -f NAME,DEVICE connection show --active" | grep ":${OMEN_WIFI}\$" | cut -d: -f1)"
  if [ -n "${conn}" ]; then ok "AP 停止 → OMEN は ${conn} に復帰"; else warn "AP は停止したが OMEN の無線が未接続"; fi
}

# ---------- G1 の無線 ----------

# SSID がスキャンに出ていないと nmcli は (53) The Wi-Fi network could not be found で落ちる。
g1_wait_ssid() {
  local want="$1" i=0
  while [ "${i}" -lt 4 ]; do
    if g1 "nmcli -t -f SSID dev wifi list" | grep -qx "${want}"; then return 0; fi
    g1 "nmcli device wifi rescan" >/dev/null 2>&1
    sleep "${RESCAN_COOLDOWN}"
    i=$((i + 1))
  done
  g1 "nmcli -t -f SSID dev wifi list" | grep -qx "${want}"
}

g1_switch() {
  local con="$1" ssid="$2"
  check_polkit g1
  if ! g1_wait_ssid "${ssid}"; then die "G1 から ${ssid} が見えない。AP が上がっているか・電波が届くか確認する"; fi
  local out; out="$(g1 "nmcli connection up ${con}")"
  case "${out}" in
    *successfully*) : ;;
    *) die "G1 の切り替えに失敗: ${out}" ;;
  esac
  sleep 4
  local link ip
  link="$(g1 "iw dev ${G1_WIFI} link")"
  ip="$(g1 "ip -4 -br addr show ${G1_WIFI}" | awk '{print $3}')"
  ok "G1 wlan0 → $(printf '%s\n' "${link}" | awk '/SSID/{print $2}') / $(printf '%s\n' "${link}" | awk '/signal/{print $2, $3}') / ${ip:-アドレス未取得}"
}

# ---------- 面B: 有線（読むだけ。絶対に変更しない） ----------

check_wired() {
  local bad=0 h
  for h in ${WIRED_HOSTS}; do
    if ! omen "ping -c 1 -W 1 ${h} >/dev/null 2>&1 && echo alive" | grep -qx alive; then
      warn "OMEN → ${h} 応答なし"; bad=1
    fi
  done
  [ "${bad}" -eq 0 ] && ok "有線 (${WIRED_HOSTS// /, }) すべて到達"
}

# ---------- コマンド ----------

cmd_status() {
  check_ssh
  head_ "面 A — インターネット"
  local mac_ip omen_conn omen_https
  mac_ip="$(ipconfig getifaddr en0 2>/dev/null)"
  printf '  Mac en0            : %s\n' "${mac_ip:-未接続}"
  omen_conn="$(omen "nmcli -t -f NAME,DEVICE connection show --active" | grep ":${OMEN_WIFI}\$" | cut -d: -f1)"
  printf '  OMEN %-14s: %s\n' "${OMEN_WIFI}" "${omen_conn:-未接続}"
  omen_https="$(omen 'curl -s -o /dev/null --max-time 8 -w "%{http_code}" https://github.com')"
  printf '  OMEN のインターネット: http_code=%s%s\n' "${omen_https}" \
    "$([ "${omen_https}" = 200 ] && echo '' || echo '（AP 稼働中なら想定どおり）')"

  head_ "面 B — 有線（制御・DDS・LiDAR）"
  printf '  Mac en8            : %s\n' "$(ipconfig getifaddr en8 2>/dev/null || echo 未接続)"
  printf '  OMEN enp129s0      : %s\n' "$(omen "ip -4 -br addr show enp129s0" | awk '{print $3}')"
  printf '  G1 eth0            : %s\n' "$(g1 "ip -4 -br addr show eth0" | awk '{print $3}')"
  check_wired

  head_ "面 C — AP"
  if omen "nmcli -t -f NAME connection show --active" | grep -qx "${AP_CON}"; then
    printf '  AP                 : 稼働中 %s\n' "$(omen "iw dev ${OMEN_WIFI} info" | awk '/ssid/{s=$2} /channel/{c=$2} END{print s" ch"c}')"
    omen "sudo -n /usr/sbin/iw dev ${OMEN_WIFI} station dump 2>/dev/null" \
      | awk '/^Station/{m=$2} /signal:/{s=$2} /tx bitrate/{print "  接続端末           : " m "  " s " dBm  " $3 " " $4}'
  else
    printf '  AP                 : 停止\n'
  fi
  local g1_wifi_con g1_wifi_ip
  g1_wifi_con="$(g1 "nmcli -t -f NAME,DEVICE connection show --active" | grep ":${G1_WIFI}\$" | cut -d: -f1)"
  g1_wifi_ip="$(g1 "ip -4 -br addr show ${G1_WIFI}" | awk '{print $3}')"
  printf '  G1 %-16s: %s  %s\n' "${G1_WIFI}" "${g1_wifi_con:-未接続}" "${g1_wifi_ip}"

  head_ "G1 への経路"
  printf '  SSH の経路         : %s\n' "${G1_VIA}"
  omen "ping -c 3 -W 1 192.168.123.164 2>&1 | tail -1" | sed 's/^/  有線   /'
  if [ -n "${g1_wifi_ip}" ] && [ "${g1_wifi_con}" = "${G1_AP_CON}" ]; then
    omen "ping -c 3 -W 1 ${g1_wifi_ip%/*} 2>&1 | tail -1" | sed 's/^/  無線   /'
    warn "制御・DDS・LiDAR は有線に乗せる。無線 RTT は揺れる（実測 2.9 / 15.9 / 38.5 ms avg）"
  fi
  echo
}

cmd_teleop() {
  check_ssh
  head_ "1/3 AP を上げる"; ap_up
  head_ "2/3 G1 を AP に寄せる"; g1_switch "${G1_AP_CON}" "${AP_SSID}"
  head_ "3/3 有線が無傷か確認する"; check_wired
  printf '\n%sテレオペ用のネットワークが整いました。%s DDS は ./net.sh dds で確認できます。\n' "${c_b}" "${c_z}"
  warn "G1 はこの間インターネットに出られない。apt/pip を使うなら ./net.sh internet で戻す"
}

cmd_internet() {
  check_ssh
  # 順序が大事。AP を先に落とすと G1 の無線が行き先を失う。
  head_ "1/3 G1 を ${G1_NET_CON} に戻す"; g1_switch "${G1_NET_CON}" "${G1_NET_CON}"
  head_ "2/3 AP を下ろす"; ap_down
  head_ "3/3 有線が無傷か確認する"; check_wired
  echo
}

cmd_dds() {
  check_ssh
  head_ "DDS 疎通（キットの preflight を呼ぶ）"
  omen 'cd ~/g1-starter-kit && timeout 90 ./scripts/preflight.sh --skip-scan' \
    | grep -E "OK|WARN|NG|受信|IMU|機体構成" | sed 's/^/  /'
}

case "${1:-}" in
  status)   cmd_status ;;
  teleop)   cmd_teleop ;;
  internet) cmd_internet ;;
  ap-up)    check_ssh; ap_up ;;
  ap-down)  check_ssh; ap_down ;;
  g1-ap)    check_ssh; g1_switch "${G1_AP_CON}" "${AP_SSID}" ;;
  g1-net)   check_ssh; g1_switch "${G1_NET_CON}" "${G1_NET_CON}" ;;
  dds)      cmd_dds ;;
  *)        sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
