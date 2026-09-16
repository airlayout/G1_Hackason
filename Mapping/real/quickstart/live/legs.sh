#!/usr/bin/env bash
# **足を繋ぐ／外す。** up.sh から分けてあるのは、ここだけが機体を動かすから。
#
#   bash quickstart/live/legs.sh --dry-run   # SDK を呼ばない。指令が見えるだけ
#   bash quickstart/live/legs.sh --arm       # 本番。**機体が歩く**
#   bash quickstart/live/legs.sh --stop      # 外す
#   bash quickstart/live/legs.sh --status    # 繋がっているかだけ見る
#
# ⚠️ **--arm の前に、リモコンで止められる人が横に居ること。**
#
# ## なぜ 2 プロセスなのか
#
# PC2 では rclpy と unitree_sdk2py が同じ Python に同居できない（2026-09-06 実測）。
#
#   [Nav2] --/cmd_vel--> [cmd_vel_bridge.py (pixi/py3.11)] --UDP--> [loco_driver.py (SDK/py3.8)] --> 足
#
# **安全機構は全部 loco_driver.py 側にある**（発進ゲート・ウォッチドッグ・速度クランプ・
# 後退の禁止）。止められるのがあちらだけなので、判断もあちらに寄せてある。
# ROS 側が落ちれば UDP が途切れ、あちらが自分で止める。
#
# ## ⚠️ 2026-09-16 に踏んだ罠
#
# `cmd_vel_bridge.py` を素で起動すると **CYCLONEDDS_URI が未設定**になり、
# CycloneDDS が自動選択で **wlan0 を掴む**。Nav2 の `/cmd_vel` は eth0 側なので
# **購読者 0 件のまま、Nav2 は計算しているのに足が一切動かない**。
# 症状が「何も起きない」なので気づきにくい。ここでは必ず eth0 を渡す。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"

BRIDGE_PY='cmd_vel_brid''ge.py'      # ⚠️ 分割して書く（pkill の自己一致よけ）
DRIVER_PY='loco_driv''er.py'

status() {
    say "足の状態"
    pc2 "for p in '$BRIDGE_PY' '$DRIVER_PY'; do
           n=\$(ps -eo args | grep -F \"\$p\" | grep -v ' grep ' | grep -cF python)
           printf '  %-20s %s\n' \"\$p\" \"\$([ \$n -gt 0 ] && echo 稼働中 || echo 停止)\"
         done"
    # ⚠️ ここが 0 件なら「繋がっているつもりで繋がっていない」。必ず見ること。
    pc2 "cat > /tmp/_cv.sh <<'EOS'
. \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='$G1_DDS_ETH0'
jros2 topic info --no-daemon /cmd_vel 2>&1 | grep -E 'Publisher|Subscription'
EOS
         bash /tmp/_cv.sh" | sed 's/^/  /'
}

case "${1:-}" in
    --status) status; exit 0 ;;
    --stop)
        say "足を外す"
        pc2_kill "$BRIDGE_PY" "" TERM
        pc2_kill "$DRIVER_PY" "" INT      # SDK 側は INT で StopMove を通す
        status; exit 0 ;;
    --dry-run|--arm) MODE="$1" ;;
    *) sed -n '2,10p' "$0" >&2; exit 2 ;;
esac

if [ "$MODE" = "--arm" ]; then
    printf '\n' >&2
    printf '  \033[31m⚠️  これから機体が歩きます。\033[0m\n' >&2
    printf '  リモコンで止められる人が横に居ますか？ [yes と入力] ' >&2
    read -r ans
    [ "$ans" = "yes" ] || die "中止した"
fi

# ── SDK 側（安全機構はこちら）────────────────────────────────────────
say "SDK 側 $DRIVER_PY を起こす（${MODE}）"
pc2 "nohup setsid python3 \$HOME/nav_tools/$DRIVER_PY \
       --network-interface eth0 $MODE > \$HOME/g1_runs/loco.log 2>&1 < /dev/null &
     sleep 6; tail -3 \$HOME/g1_runs/loco.log | sed 's/^/     /'"

# ── ROS 側（⚠️ eth0 を必ず渡す）──────────────────────────────────────
say "ROS 側 $BRIDGE_PY を起こす（CYCLONEDDS_URI を eth0 に固定）"
pc2 "cd \$HOME/g1_humble
     nohup setsid env ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
       CYCLONEDDS_URI='$G1_DDS_ETH0' \
       \$HOME/.pixi/bin/pixi run python \$HOME/nav_tools/$BRIDGE_PY \
       > \$HOME/g1_runs/cmdvel.log 2>&1 < /dev/null &
     sleep 25; tail -2 \$HOME/g1_runs/cmdvel.log | sed 's/^/     /'"

printf '\n'
status
printf '\n'
say "⚠️ 上の **Subscription count が 1 以上**であることを必ず確かめる。"
say "   0 のままなら足は繋がっていない（Nav2 は計算しているのに動かない）。"
[ "$MODE" = "--arm" ] && say "RViz2 の「2D Goal Pose」でクリックすれば歩きます。"
