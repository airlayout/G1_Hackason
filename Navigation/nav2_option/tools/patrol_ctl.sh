#!/usr/bin/env bash
# 巡回モードの操作。**ROS 環境を source 済みのシェルで叩く**（PC2 でも操作PCでもよい）。
#
#   ./patrol_ctl.sh start    # 巡回を開始/再開する（止まった点の次からではなく、その点から）
#   ./patrol_ctl.sh pause    # いまの Goal を畳んで止まる。index は保つ
#   ./patrol_ctl.sh stop     # 止めて1点目に戻す
#   ./patrol_ctl.sh skip     # いまの点を諦めて次へ
#   ./patrol_ctl.sh status   # いまの状態を1行で
#   ./patrol_ctl.sh watch    # 状態を流し続ける
#
# ## RViz で巡回路を引く（教示モード）
#
#   ./patrol_ctl.sh teach    # TEACH に入る
#   （RViz の **「Publish Point」**で回りたい順に地面をクリック。
#     引いた形は PatrolRoute として見える）
#   ./patrol_ctl.sh undo     # 直前の1点を取り消す
#   ./patrol_ctl.sh save     # yaml に書き、そのまま巡回路として読み込む（再起動不要）
#   ./patrol_ctl.sh cancel   # 全部捨てて TEACH を抜ける
#
# ⚠️⚠️ **教示に「2D Goal Pose」を使わないこと。** `bt_navigator` が `/goal_pose` を
#    直接購読しているので、**クリックした瞬間に機体が本当にそこへ歩き出す**。
# ⚠️ 向きは「次の点へ向かう方位」が自動で入る。特定の向きで止まりたい点は
#    yaml の `yaw_deg` を書き換えるか `record_waypoints.py` で取り直す。
# ⚠️ クリックするのは**地図の上の座標**。地図が古いと実際には通れない点を作れてしまう。
#    確実なのは `record_waypoints.py`（機体を実際にその場所へ立たせて拾う）。
#
# ⚠️ **`start` は `bridge_status` が `NAVIGATING` でないと断られる。** 断り文に理由が入る。
#
#   | bridge | 意味 | すること |
#   |---|---|---|
#   | `READY`        | TF もセンサーも健全。**だが走行許可がまだ** | `ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"` |
#   | `STANDBY`      | TF かセンサーがまだ | 手順書 §5.3 に戻る |
#   | `DISCONNECTED` | SDK 側プロセスに繋がっていない | ブリッジの起動と `bridge_sock_dir` |
#   | `FAULT`        | 通信断・cmd_timeout など | 原因を潰して `clear_fault` |
#
#   ⚠️ 発進ゲート（`G1_ARM=--arm`）は `bridge_status` に出ない。**ゲートが閉じたままでも
#   `NAVIGATING` にはなる**（機体が動かないだけ）。ゲートは `pgrep -af g1_sdk_bridge_real_server`。
#
# ⚠️ 単純ゴール指定モードに戻すのに `stop` は要らない。**RViz から Goal を送れば
#    巡回のほうが退く**（patrol_node.py の yield_to_manual_goal）。
set -o pipefail   # ⚠️ set -u は ROS の setup.bash を壊す

CMD="${1:-status}"

# 教示系は打ちやすい短い名前にする（サービス名は teach_* ）
case "$CMD" in
  save)   CMD=teach_save ;;
  undo)   CMD=teach_undo ;;
  cancel) CMD=teach_cancel ;;
esac

case "$CMD" in
  start|pause|stop|skip|teach|teach_save|teach_undo|teach_cancel)
    # ⚠️ `tail -1` は空行を拾う。**応答行を明示的に抜く**こと
    ros2 service call "/g1/patrol/$CMD" std_srvs/srv/Trigger "{}" \
      | sed -n "s/.*Trigger_Response(\(.*\))$/\1/p"
    ;;
  status)
    timeout 3 ros2 topic echo --once --full-length /g1/patrol/status std_msgs/msg/String 2>/dev/null \
      | sed -n 's/^data: //p' \
      || echo "❌ /g1/patrol/status が来ない。巡回ノードが上がっていない（patrol:=false？）"
    ;;
  watch)
    ros2 topic echo --full-length /g1/patrol/status std_msgs/msg/String | sed -n 's/^data: //p'
    ;;
  *)
    sed -n '2,40p' "$0" >&2
    exit 2
    ;;
esac
