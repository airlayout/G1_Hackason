#!/usr/bin/env bash
# Nav2 を止める。Jetson（PC2）で、コンソールのタスク「nav2-down」から実行する。
#
# 警備班の撤収手順（Navigation/nav2_stable/safety/runbook_nav2_session.md §10）のうち、
# sudo のパスワードが要らない部分だけを、同じ順序で行う。
#   ①発進ゲートを閉じる ... sudo が要る。人が行う（D-07。ここでは確認と警告だけ）
#   ②記録を止める        ... pkill -f 'ros2 bag record'
#   （Nav2 本体・自己位置・橋渡しノードを落とす。start_nav.sh と同じ一覧）
#   ③内蔵 SLAM を止める  ... send_slam_api.py 1901（機体は動かない API だけ）
#   ④g1-sdk-bridge を止める ... sudo が要る。人が行う
#
# 発進ゲート（G1_ARM=--arm）は決して開けない。閉じようともしない（sudo が要るうえ、人の判断）。
# ゲートが開いたままなら、止められる部分は止めたうえで終了コード 3 を返し、画面に「失敗」と出す。
#
# 使い方: nav2_down.sh [--dry-run]    --dry-run は何も止めず、やることを表示する
# 環境変数（試験用）: G1_BRIDGE_ENV（既定 /etc/default/g1-sdk-bridge）、G1_NAV2_TOOLS（既定 /home/unitree/g1_nav2/tools）、G1_PYTHON（既定 /usr/bin/python3）
set -u

DRY=0
case "${1:-}" in
    --dry-run) DRY=1 ;;
    "") ;;
    *) echo "未知の引数: $1" >&2; exit 2 ;;
esac

BRIDGE_ENV="${G1_BRIDGE_ENV:-/etc/default/g1-sdk-bridge}"
TOOLS="${G1_NAV2_TOOLS:-/home/unitree/g1_nav2/tools}"
PY="${G1_PYTHON:-/usr/bin/python3}"
# start_nav.sh / g1up.sh の掃除と同じ一覧（角括弧は自分自身の pkill に一致しないための書き方）
NAV_PATTERNS=("ros2 launch g1_navigatio[n]" "g1_slam_odom_t[f].py" "g1_state_bridge_nod[e]"
              "g1_cmd_router_nod[e]" "patrol_nod[e].py" "envs/default/lib/nav[2]_" "map_localize[r].py")

run() { if [ "$DRY" = 1 ]; then echo "  [dry] $*"; else "$@"; fi; }

gate_open() {
    # ブリッジの起動引数に --arm があるか、設定ファイルが G1_ARM=--arm か。どちらかなら「開」
    ps -eo args= 2>/dev/null | grep -E 'g1_sdk_bridge_real_serve[r]' | grep -q -e '--arm' && return 0
    grep -q '^G1_ARM=--arm' "$BRIDGE_ENV" 2>/dev/null && return 0
    return 1
}

if [ "$DRY" = 1 ]; then echo "=== Nav2 を止める（dry-run。何も止めない） ==="; else echo "=== Nav2 を止める ==="; fi
GATE_OPEN=0
if gate_open; then
    GATE_OPEN=1
    echo "⚠️  発進ゲートが開いている。最優先は人がゲートを閉じること（このタスクは閉じない）:"
    echo "      sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' $BRIDGE_ENV"
    echo "      sudo systemctl restart g1-sdk-bridge"
else
    echo "発進ゲートは閉じている"
fi

echo "--- ② 記録を止める"
run pkill -f 'ros2 bag recor[d]' || true

echo "--- Nav2 と橋渡しノードを落とす"
for pat in "${NAV_PATTERNS[@]}"; do
    run pkill -f "$pat" || true
done
[ "$DRY" = 1 ] || sleep 3

echo "--- ③ 内蔵 SLAM を止める（1901）"
if [ "$DRY" = 1 ]; then
    echo "  [dry] cd $TOOLS && $PY send_slam_api.py 1901"
elif [ -f "$TOOLS/send_slam_api.py" ]; then
    ( cd "$TOOLS" && "$PY" send_slam_api.py 1901 ) || echo "⚠️  1901 の送信に失敗した。内蔵 SLAM が動いたままかもしれない"
else
    echo "⚠️  $TOOLS/send_slam_api.py が無い。1901 を送れていない"
fi

echo "--- 残っているプロセス"
LEFT=$(ps -eo args= 2>/dev/null | grep -E -e 'ros2 launch g1_navigatio[n]' -e 'envs/default/lib/nav[2]_' -e 'ros2 bag recor[d]' || true)
if [ -n "$LEFT" ] && [ "$DRY" = 0 ]; then
    echo "⚠️  まだ動いているプロセスがある:"
    echo "$LEFT" | cut -c1-160 | sed 's/^/      /'
else
    echo "なし"
fi

echo "--- ④ 人が行うこと（このタスクは行わない）"
echo "      sudo systemctl stop g1-sdk-bridge"

if [ "$GATE_OPEN" = 1 ]; then
    echo "❌ 発進ゲートが開いたまま。Nav2 は止めたが、ゲートを閉じるまで撤収は終わっていない"
    exit 3
fi
echo "完了"
