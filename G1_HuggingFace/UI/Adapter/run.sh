#!/usr/bin/env bash
# ROS アダプタを Docker（ROS 2 Humble）の中で起動する。**操作PC で叩く。**
#
#   bash run.sh                  # 既定 127.0.0.1:8081
#   bash run.sh --port 9081
#   G1_DOMAIN=7 bash run.sh      # ROS_DOMAIN_ID を変える
#
# 起動したら UI 側を http に切り替える:
#   bash ../Main/run.sh --nav http
#
# ⚠️ **操作PC に ROS 2 は入っていない。** だから ROS に触るのはこのコンテナの中だけ。
#    UI サーバ本体は素の venv で動き、ここへは HTTP で問い合わせる。
# ⚠️ イメージは Mapping トラックのもの。**他トラックの資産に依存している**ので、
#    向こうが作り直したら動かなくなりうる。そのときは G1_UI_IMAGE で差し替える。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$(cd "$HERE/.." && pwd)"

IMAGE="${G1_UI_IMAGE:-g1-mapping-visualization:local}"
# ⚠️ G1 と同じ網に出ている有線のインターフェース名。`ip -br addr` で確かめること
IFACE="${G1_UI_IFACE:-enp3s0}"
DOMAIN="${G1_DOMAIN:-${ROS_DOMAIN_ID:-0}}"
NAME="${G1_UI_ADAPTER_NAME:-g1-ui-adapter}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[アダプタ] イメージが無い: $IMAGE" >&2
  echo "           Mapping/real/docker/ で作るか、G1_UI_IMAGE で指定してください" >&2
  exit 1
fi

# 既に動いていたら片付ける（付けっぱなしで二重起動すると port が埋まる）
docker rm -f "$NAME" >/dev/null 2>&1 || true

# ⚠️ CycloneDDS を有線に固定する。WiFi 側を選ばれると「トピックは見えるのに
#    データが来ない」壊れ方をする（PC2 側も同じ理由で eth0 に固定している）
DDS_CFG="$(mktemp -t cyclonedds_ui_XXXX.xml)"
sed "s/@IFACE@/$IFACE/" "$HERE/cyclonedds_operator.xml" > "$DDS_CFG"
trap 'rm -f "$DDS_CFG"' EXIT
if ! ip -br addr show "$IFACE" 2>/dev/null | grep -q UP; then
  echo "[アダプタ] ⚠️ $IFACE が上がっていない。nmcli connection up g1-link を先に" >&2
fi

echo "[アダプタ] image=$IMAGE ROS_DOMAIN_ID=$DOMAIN iface=$IFACE"
# --network host: DDS の discovery をホスト側の ROS と同じ網に置く
# --ipc host:     共有メモリ転送のため（Navigation の mock テストと同じ作法）
# UI ディレクトリを渡すのは、契約 (Main/nav/base.py) を共有して読むため
exec docker run --rm --name "$NAME" --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -e PYTHONUNBUFFERED=1 \
  -e CYCLONEDDS_URI="file:///dds.xml" \
  -v "$DDS_CFG:/dds.xml:ro" \
  -v "$UI_DIR:/ui:ro" \
  "$IMAGE" \
  bash -lc 'source /opt/ros/humble/setup.bash && exec python3 /ui/Adapter/ros_adapter.py '"$*"
