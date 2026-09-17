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
    # ⚠️ **PC2 側で grep しない**（`_common.sh` の注記。自己マッチで 4 回誤診した）。
    # 旧実装は `grep -v ' grep '` に**偶然守られていた**だけで、
    # パターンを 1 つでも変えれば「常に稼働中」と出る形だった。
    for py in "$BRIDGE_PY" "$DRIVER_PY"; do
        n="$(pc2_count "$py")"
        printf '  %-20s %s\n' "$py" "$([ "${n:-0}" -gt 0 ] && echo 稼働中 || echo 停止)" >&2
    done
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
        # ⚠️⚠️ **INT で死んだか必ず確かめる。**`--dry-run` のドライバは SIGINT を
        # 無視するので、`--stop` しても**ポート 47600 を掴んだまま残る**
        # （2026-09-17 に 19 分残し、次の `--arm` が
        #  `OSError: [Errno 98] Address already in use` で失敗した）。
        # しかも `cmd_vel_bridge` だけは起動に成功するので、`--status` は
        # 「両方稼働中・Subscription count 1」と出て**繋がったように見える**。
        # 指令は素振りのドライバへ流れ、**機体は動かない**。
        pc2_kill_procs 'loco_driver|cmd_vel_bridge' KILL
        HELD="$(pc2 "ss -lun 2>/dev/null | grep -c 47600 || true" 2>/dev/null | tr -d '[:space:]')"
        note "47600 を掴む socket ${HELD:-0} 個"
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

# ── 先に 47600 が空いているか見る ────────────────────────────────────
# ⚠️ **ここで止めないと危ない誤解が生まれる。**ポートが埋まっているとドライバは
# bind に失敗して死ぬが、橋は起動に成功するので `--status` が
# 「両方稼働中・Subscription count 1」と出る。**指令の行き先が古いドライバ**になる。
# ⚠️ **`|| echo 0` と繋いではいけない（`|| true` にする）。**
# `grep -c` も `pgrep -c` も、0 件のとき「0」を印字した上で**終了コード 1** を返す。
# `|| echo 0` を足すと数字が 2 個出て `"00"` になり、`!= "0"` が**必ず真になる**
# ＝ ポートが空いていても「埋まっている」と出て中止する。
# 2026-09-17 に preflight.sh で直した直後、ここで同じ型を再発させた。
# 同じ注記が quickstart/apriltag/record_registration.sh にもある（既知の罠）。
HELD="$(pc2 "ss -lun 2>/dev/null | grep -c 47600 || true" 2>/dev/null | tr -d '[:space:]')"
if [ "${HELD:-0}" != "0" ]; then
    say "⚠️ 127.0.0.1:47600 が既に埋まっている（古い $DRIVER_PY が残っている）"
    pc2_procs "$DRIVER_PY" | cut -c1-110 | sed 's/^/     /' >&2
    die "先に bash quickstart/live/legs.sh --stop を実行すること"
fi

# ── SDK 側（安全機構はこちら）────────────────────────────────────────
say "SDK 側 $DRIVER_PY を起こす（${MODE}）"
pc2 "nohup setsid python3 \$HOME/nav_tools/$DRIVER_PY \
       --network-interface eth0 $MODE > \$HOME/g1_runs/loco.log 2>&1 < /dev/null &
     sleep 6; tail -3 \$HOME/g1_runs/loco.log | sed 's/^/     /'"

# ⚠️ **bind できたかを確かめてから橋を起こす。**失敗したまま橋を起こすと
# 「繋がっているつもりで繋がっていない」状態になる（上の注記）。
pc2 "grep -q 'Address already in use\|Traceback' \$HOME/g1_runs/loco.log && exit 9 || exit 0" \
    || die "$DRIVER_PY が起動できていない（~/g1_runs/loco.log を見る）。橋は起こさない"

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
