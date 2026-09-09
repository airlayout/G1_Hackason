#!/usr/bin/env bash
# **Mac の上で**動かす。Nav2 方式で足を繋ぐのに要るものを PC2 へ配る。
#
#   bash Navigation/real/deploy_to_pc2.sh            # 配って md5 で検証する
#   bash Navigation/real/deploy_to_pc2.sh check      # 配らずに差分だけ見る
#   bash Navigation/real/deploy_to_pc2.sh status     # PC2 で何が動いているか見る
#
# ## なぜスクリプトにするのか
#
# 2026-09-09 まで**手作業だった**（計画書に scp の 1 行が書いてあるだけ）。
# 手で配ると次の 2 つが起きる:
#   1. **配ったつもりで古いまま。** scp の成功は「中身が同じ」の証明にならない
#      （権限や途中切断でも 0 を返す経路がある）。ここでは md5 で突き合わせる
#   2. **置き場所を取り違える。** PC2 には 2 つの置き場があり、役割が違う:
#        ~/nav_tools/     … python 本体（cmd_vel_bridge.py / loco_driver.py）
#        ~/mapping_tools/ … PC2 上で叩く shell（start_*.sh）。既存の流儀に合わせる
#
# ## ⚠️ このスクリプトは足を動かさない
#
# 配るだけで、`loco_driver.py` は**起こさない**。あれは `--arm` を付けないと
# 指令を捨てるし、付けたら歩く。**人が支え、停止手段を手に持ってから**、
# 自分の手で起こすこと（計画書 §6）。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SSH_HOST="${G1_SSH_HOST:-g1}"          # ~/.ssh/config の別名。IP は書かない
NAV_TOOLS="${G1_NAV_TOOLS:-nav_tools}"          # $HOME 相対
MAP_TOOLS="${G1_MAP_TOOLS:-mapping_tools}"

# 配るもの: ローカルのパス と 置き場（$HOME 相対）
FILES=(
    "$HERE/cmd_vel_bridge.py|$NAV_TOOLS"
    "$HERE/loco_driver.py|$NAV_TOOLS"
    "$HERE/start_cmd_vel_bridge.sh|$MAP_TOOLS"
)

say() { echo "[deploy] $*"; }
MODE="${1:-deploy}"

# ⚠️ BatchMode=yes を付ける。付けないと鍵が無いときに**パスワードを聞いて止まる**
# （自動実行だと入力できないまま待ち続ける。2026-09-09 に踏んだ）。
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "$SSH_HOST")

if ! "${SSH[@]}" true 2>/dev/null; then
    echo "[deploy] ssh $SSH_HOST に鍵で入れない" >&2
    echo "   → ~/.ssh/config の Host $SSH_HOST と IdentityFile を確認する" >&2
    echo "   → 経路そのものは Mapping/real/quickstart/check_link.sh で見る" >&2
    exit 1
fi
say "ssh $SSH_HOST OK（$("${SSH[@]}" hostname)）"

if [ "$MODE" = "status" ]; then
    say "PC2 で動いているもの:"
    # パターンは [] で括る。括らないと pgrep が自分のコマンド行に当たる
    "${SSH[@]}" 'pgrep -af "cmd_vel_bridg[e]|loco_drive[r]" || echo "  （どちらも動いていない）"'
    say "⚠️ loco_driver が動いているなら **足は繋がっている**。ゴールを投げる前に確認すること"
    exit 0
fi

# 突き合わせ。md5 の取り方が Mac(md5 -q) と Linux(md5sum) で違う
local_md5() { md5 -q "$1" 2>/dev/null || md5sum "$1" | cut -d' ' -f1; }

RC=0
for spec in "${FILES[@]}"; do
    src="${spec%%|*}"; dst_dir="${spec#*|}"
    base="$(basename "$src")"
    want="$(local_md5 "$src")"
    have="$("${SSH[@]}" "md5sum ~/$dst_dir/$base 2>/dev/null | cut -d' ' -f1" || true)"
    if [ "$want" = "$have" ]; then
        say "同じ   $dst_dir/$base"
        continue
    fi
    if [ "$MODE" = "check" ]; then
        say "違う   $dst_dir/${base}（PC2 側 ${have:-無し}）"
        RC=1
        continue
    fi
    "${SSH[@]}" "mkdir -p ~/$dst_dir"
    scp -q -o BatchMode=yes "$src" "$SSH_HOST:$dst_dir/$base"
    got="$("${SSH[@]}" "md5sum ~/$dst_dir/$base | cut -d' ' -f1")"
    if [ "$want" != "$got" ]; then
        echo "[deploy] **配ったのに md5 が合わない**: $dst_dir/$base" >&2
        echo "   ローカル $want / PC2 $got" >&2
        exit 1
    fi
    say "配った $dst_dir/$base"
done

if [ "$MODE" = "check" ]; then
    [ "$RC" = "0" ] && say "PC2 は最新" || say "⚠️ 差分がある。引数なしで実行すると配る"
    exit "$RC"
fi

cat <<'NEXT'

[deploy] 次にやること（この順で）

  1. Mac 側でスタックを上げる（まだ足は繋がらない）
       cd G1_Hackason/Mapping/real
       G1_USE_MOLA=1 bash quickstart/nav_stack.sh live

  2. PC2 の ROS 側だけ起こす（**足はまだ動かない**）
       ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh'

  3. ゴールを 1 つ投げて /cmd_vel に非ゼロが出るのを確かめる（段 D）
       RViz2 の「2D Goal Pose」、または
       python3 check_navigation.py --waypoints <wp>.json --tries 1 \
           --no-sim-time --planner-id Smac2D --timeout 30

  4. ⚠️ **ここから足が動く。人が支え、停止手段を手に持つこと**（計画書 §6）
       ssh -t g1 'python3 ~/nav_tools/loco_driver.py \
           --network-interface eth0 --max-vy 0 --arm'

  止めるとき
       ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh stop'
       bash quickstart/nav_stack.sh stop
NEXT
