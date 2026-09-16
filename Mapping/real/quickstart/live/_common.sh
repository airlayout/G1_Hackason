#!/usr/bin/env bash
# `live/` の共通部。**2026-09-16 に実機で踏んだ罠を全部ここに閉じ込めてある。**
#
# 読み込むだけ。単体では何もしない:
#   . "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

# ── 宛先 ───────────────────────────────────────────────────────────
G1_PC2_HOST="${G1_PC2_HOST:-10.42.0.76}"          # PC2（無線）。有線は 192.168.123.164
G1_AP_HOST="${G1_AP_HOST:-192.168.123.200}"       # AP を出している Ubuntu（OMEN）
G1_AP_USER="${G1_AP_USER:-ubuntu}"
G1_PC2_USER="${G1_PC2_USER:-unitree}"
G1_KEY="${G1_KEY:-$HOME/.ssh/id_ed25519_g1}"      # ⚠️ ~/.ssh/config の Host g1 は有線向き
G1_VM_IP="${G1_VM_IP:-192.168.123.201}"           # コンテナ（col0）
G1_CONTAINER="${G1_CONTAINER:-rviz}"

# ⚠️ **PC2 の DDS は必ず eth0 に固定する。**
# 未設定だと CycloneDDS が自動選択で **wlan0 を掴み**、PC1 が出す LiDAR も
# Nav2 の /cmd_vel も見えなくなる（2026-09-16 実測。cmd_vel_bridge がこれで
# 足に繋がっていなかった）。2 NIC にしても直らない（live/README.md の表）。
G1_DDS_ETH0='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
# コンテナ側は col0。
G1_DDS_COL0='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="col0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

# ⚠️ **記録は /tmp に置かない。** 2026-09-16 に電源が落ちて歩行 2 回分を失った。
G1_RUN_DIR="${G1_RUN_DIR:-\$HOME/g1_runs}"        # PC2 側で展開させるので \$ をエスケープ

say()  { printf '[live] %s\n' "$*" >&2; }
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*" >&2; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$*" >&2; }
bad()  { printf '  \033[31mNG\033[0m   %s\n' "$*" >&2; }
die()  { printf '[live] ⛔ %s\n' "$*" >&2; exit 1; }

# ── PC2 でコマンドを走らせる ────────────────────────────────────────
pc2() { ssh -i "$G1_KEY" -o BatchMode=yes -o ConnectTimeout=15 \
            -o IdentitiesOnly=yes "$G1_PC2_USER@$G1_PC2_HOST" "$@"; }
ap()  { ssh -o BatchMode=yes -o ConnectTimeout=10 "$G1_AP_USER@$G1_AP_HOST" "$@"; }

# PC2 で ROS 環境を整えて走らせる。
# ⚠️ `jros2` は**シェル関数**なので `timeout jros2 ...` は動かない
#    （timeout は実行ファイルしか起動できない。2026-09-16 に踏んだ）。
pc2_ros() {
    pc2 "export ROS_DOMAIN_ID=0
         export CYCLONEDDS_URI='$G1_DDS_ETH0'
         . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
         $*"
}

# コンテナでコマンドを走らせる。
# ⚠️ コンテナの /bin/sh は **dash** なので `/dev/tcp` が使えない。必ず bash。
ctr() { docker exec -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" \
              "$G1_CONTAINER" bash -c "$*"; }
ctr_bg() { docker exec -d -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" \
                 "$G1_CONTAINER" bash -c "$*"; }

# ── TCP 疎通（ping は入っていない環境がある）──────────────────────
# ⚠️ **このスクリプトを zsh で走らせない。** zsh に `/dev/tcp` は無い。
#    `#!/usr/bin/env bash` を守ること（2026-09-16 に誤判定した）。
tcp_ok() { (exec 3<>"/dev/tcp/$1/$2") 2>/dev/null && { exec 3<&-; return 0; }; return 1; }

# ── プロセスを名前で止める（自分を巻き添えにしない）──────────────
# ⚠️ `pkill -f foo` も `pkill -f fo[o]` も、**呼び出し側のコマンド行に foo が
#    含まれていると自分を殺す**（2026-09-16 に 4 回踏んだ。ssh が落ちて
#    「止まったのか分からない」状態になる）。
#    文字列を分割して組み立て、PID を取ってから kill する。
pc2_kill() {  # pc2_kill <前半> <後半> [シグナル]
    local sig="${3:-TERM}"
    pc2 "P=\$(ps -eo pid,args | grep -F \"\$(printf '%s%s' '$1' '$2')\" \
              | grep -v ' grep ' | awk '{print \$1}')
         for p in \$P; do kill -$sig \"\$p\" 2>/dev/null; done
         sleep 2
         N=\$(ps -eo args | grep -cF \"\$(printf '%s%s' '$1' '$2')\")
         echo \"  残り \$((N-1)) 個\""
}
ctr_kill() { docker exec "$G1_CONTAINER" bash -c \
    "P=\$(ps -eo pid,args | grep -F \"\$(printf '%s%s' '$1' '$2')\" \
         | grep -v ' grep ' | awk '{print \$1}')
     for p in \$P; do kill -9 \"\$p\" 2>/dev/null; done; sleep 1"; }
