#!/usr/bin/env bash
# FastDDS の initial peers 設定を作る。**マルチキャストが通らない網で使う。**
#
# ## なぜ要るのか
#
# DDS は起動時にマルチキャスト(239.255.0.1:7400)で相手を探す。スマホのテザリングや
# ゲスト用 AP はクライアント同士のマルチキャストを遮断するので、**ping は通るのに
# DDS だけ繋がらない**という状態になる(2026-09-15 に実機で踏んだ。トピックが
# 自分のぶん 2 件しか見えなかった)。
#
# ## ⚠️ マルチキャストを止めてはいけない
#
# `initialPeersList` は**追加**であって置き換えではない。ここが重要で、
# **PC2 は eth0 のマルチキャストで G1 本体(PC1 の CycloneDDS)のトピックを
# 見つけている**。マルチキャストを切ったり Discovery Server 方式にすると、
# `/utlidar/cloud_livox_mid360` などセンサー一式が取れなくなる。
#
# ## 使い方
#
#     ./make_fastdds_peers.sh 10.53.222.78 /tmp/fastdds_peers.xml
#     export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_peers.xml
#
# 相手の IP は**テザリングの DHCP で毎回変わる**。セッションごとに作り直すこと。
# 複数指定するときは IP をカンマで区切る。
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "使い方: $0 <相手のIP[,IP...]> [出力先(既定 /tmp/fastdds_peers.xml)]" >&2
    exit 2
fi
PEERS="$1"
OUT="${2:-/tmp/fastdds_peers.xml}"

{
    echo '<?xml version="1.0" encoding="UTF-8" ?>'
    echo '<dds xmlns="http://www.eprosima.com">'
    echo '  <profiles>'
    echo '    <participant profile_name="g1_peers" is_default_profile="true">'
    echo '      <rtps><builtin><initialPeersList>'
    # ⚠️⚠️ **マルチキャストアドレスを必ず先頭に入れること**(2026-09-15 実機で踏んだ)。
    # FastDDS は initialPeersList を指定すると**既定のマルチキャスト宛 announce を
    # 置き換える**。こちらが announce しなくなると G1 本体(PC1)は我々を知れず、
    # データを送ってこない。**受信側の設定なのに送信が止まる**という分かりにくい壊れ方で、
    # 実際にトピックが 153 件 → 見えない、に変わった。
    echo "        <locator><udpv4><address>239.255.0.1</address></udpv4></locator>"
    IFS=',' read -ra IPS <<< "$PEERS"
    for ip in "${IPS[@]}"; do
        echo "        <locator><udpv4><address>${ip}</address></udpv4></locator>"
    done
    echo '      </initialPeersList></builtin></rtps>'
    echo '    </participant>'
    echo '  </profiles>'
    echo '</dds>'
} > "$OUT"

echo "[peers] 書いた: $OUT (相手=$PEERS)"
echo "[peers] export FASTRTPS_DEFAULT_PROFILES_FILE=$OUT して ROS を起動すること"
