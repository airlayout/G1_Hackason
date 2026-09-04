#!/usr/bin/env bash
# PC2 に ROS 2 Humble 環境を pixi で用意する。root 不要、既存の /opt/ros/foxy に触らない。
#
#   scp -r humble_env g1:~/ && ssh g1 'bash ~/humble_env/setup_pc2.sh'
#
# なぜ foxy ではなく Humble が必要か:
#   foxy 同梱の cyclonedds 0.7.0 は、Unitree(0.10.2) が撒く discovery パケットを
#   ddsi_plist_init_frommsg で解釈中に SIGSEGV する。ros2 topic list も rviz2 も
#   rosbridge も、domain 0 の eth0 に参加した瞬間に同じ死に方をする。
#   RTPS は未知パラメータの読み飛ばしを規格で要求しているので 0.7.0 側の実装バグ。
#   0.10 世代（Humble）を持ち込むと解決する（2026-09-03 実機で確認）。
#
# なぜ conda ではなく pixi か:
#   プロジェクトローカルに閉じる（base 環境も ~/.condarc も汚さない）。
#   pixi.lock で版が固定される。撤退はディレクトリ削除のみ。
set -euo pipefail

PIXI_VER_URL=https://github.com/prefix-dev/pixi/releases/latest/download
ASSET=pixi-aarch64-unknown-linux-musl.tar.gz
PROJ="${1:-$HOME/g1_humble}"

if ! command -v "$HOME/.pixi/bin/pixi" >/dev/null 2>&1; then
    echo "[setup] pixi を取得します"
    cd /tmp
    rm -f "$ASSET" "$ASSET.sha256"
    curl -fsSL -o "$ASSET"        "$PIXI_VER_URL/$ASSET"
    curl -fsSL -o "$ASSET.sha256" "$PIXI_VER_URL/$ASSET.sha256"
    want=$(grep -oE '^[0-9a-f]{64}' "$ASSET.sha256" || awk '{print $1}' "$ASSET.sha256")
    got=$(sha256sum "$ASSET" | awk '{print $1}')
    [ "$want" = "$got" ] || { echo "[setup] sha256 不一致。中止します" >&2; exit 1; }
    echo "[setup] sha256 一致"
    mkdir -p "$HOME/.pixi/bin"
    tar -xzf "$ASSET" -C "$HOME/.pixi/bin"
    chmod +x "$HOME/.pixi/bin/pixi"
else
    echo "[setup] pixi は既にあります"
fi
"$HOME/.pixi/bin/pixi" --version

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$PROJ"
cp "$SRC/pixi.toml" "$PROJ/pixi.toml"
# lock があれば版を固定して再現する
[ -f "$SRC/pixi.lock" ] && cp "$SRC/pixi.lock" "$PROJ/pixi.lock"

echo "[setup] 環境を構築します（約1.2GB。初回は数分）"
cd "$PROJ"
"$HOME/.pixi/bin/pixi" install

echo "[setup] 完了: $PROJ"
echo "[setup] 確認: cd $PROJ && ~/.pixi/bin/pixi run ros2 topic list"
