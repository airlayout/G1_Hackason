#!/usr/bin/env bash
# **Mac の上で**動かす。撮影のページを https で配るための、手元だけの認証局と Mac の証明書を作る（~/.config/g1_phone_cam/。git の外）。
#
#   bash Door/real/phone_cam/make_cert.sh          # 無ければ作る・Mac の IP が変わっていれば Mac の証明書だけ作り直す
#   bash Door/real/phone_cam/make_cert.sh --force  # Mac の証明書を作り直す（認証局はそのまま＝iPhone は入れ直さなくてよい）
#
# iPhone の Safari はカメラを https でしか使わせない。iPhone には認証局（ca.crt）を 1 度だけ入れて信頼する:
#   1. iPhone で http://<Mac の IP>:8080/ を開き「証明書を入れる」→「設定」→「プロファイルがダウンロードされました」→ インストール
#   2. 「設定」→「一般」→「情報」→「証明書信頼設定」→ g1_phone_cam をオン
#   外す時: 「設定」→「一般」→「VPN とデバイス管理」→ g1_phone_cam → プロファイルを削除
# iOS の決まり: 証明書の名前は SAN に書く・serverAuth・有効期間 825 日以内。認証局の鍵はこの Mac から出さない。
set -euo pipefail
CONF="$HOME/.config/g1_phone_cam"
mkdir -p "$CONF"
chmod 700 "$CONF"
cd "$CONF"
say() { echo "[cert] $*"; }

ips=("127.0.0.1")
for ifc in en0 en1 en2 en3 en4 en5 en6 en7 en8; do
  ip=$(ipconfig getifaddr "$ifc" 2>/dev/null || true)
  [ -n "$ip" ] && ips+=("$ip")
done
ts_ip=$( (tailscale ip -4 2>/dev/null || true) | head -1)
[ -n "$ts_ip" ] && ips+=("$ts_ip")
host="$(scutil --get LocalHostName 2>/dev/null || hostname -s).local"
san="DNS:localhost,DNS:${host}"
for ip in "${ips[@]}"; do san="${san},IP:${ip}"; done

if [ ! -f ca.key ] || [ ! -f ca.crt ]; then
  say "認証局を作る（iPhone には 1 度だけ入れる）"
  openssl genrsa -out ca.key 2048 2>/dev/null
  chmod 600 ca.key
  openssl req -x509 -new -key ca.key -sha256 -days 820 -out ca.crt -subj "/CN=g1_phone_cam/O=G1_Hackason" \
    -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=critical,keyCertSign,cRLSign"
fi

if [ "${1:-}" != "--force" ] && [ -f server.crt ] && [ -f server.san ] && [ "$(cat server.san)" = "$san" ] \
   && openssl x509 -checkend 604800 -noout -in server.crt >/dev/null; then
  say "Mac の証明書はそのまま使える（${san}）"
  exit 0
fi

say "Mac の証明書を作る: ${san}"
openssl genrsa -out server.key 2048 2>/dev/null
chmod 600 server.key
openssl req -new -key server.key -out server.csr -subj "/CN=${host}" 2>/dev/null
cat > server.ext <<EOF
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage=serverAuth
subjectAltName=${san}
EOF
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 397 -sha256 \
  -extfile server.ext 2>/dev/null
rm -f server.csr
echo "$san" > server.san
openssl verify -CAfile ca.crt server.crt
