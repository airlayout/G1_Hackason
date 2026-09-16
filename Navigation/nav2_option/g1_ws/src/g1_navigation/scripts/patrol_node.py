#!/usr/bin/env python3
"""巡回モード。**ウェイポイントの列を順に回る。単純ゴール指定モードと併存する。**

## 2つのモードの関係（ここが設計の肝）

| | 単純ゴール指定モード | 巡回モード |
|---|---|---|
| 入口 | RViz の **2D Goal Pose**（`/goal_pose`） | `/g1/patrol/start` サービス |
| 経路 | `bt_navigator` の `NavigateToPose` | 同左。**このノードが順に投げるだけ** |
| 切替 | — | **再起動は要らない**。サービスを呼ぶだけ |

📌 **このノードは常駐するが、既定では `IDLE` で何もしない。**
`/g1/patrol/start` を呼ぶまで Goal を1件も送らない。したがって
「置いてあるだけ」の状態では従来の単純ゴール指定モードと完全に同じ挙動になる。

📌 **手動の Goal が常に勝つ。** 巡回中に RViz から Goal を送ると、このノードは
自分の巡回を畳んで `HOLD` に退く（`yield_to_manual_goal`）。`bt_navigator` は
Goal を1件しか持てないので、**退かないと「人が送った Goal を巡回が奪い返す」**
という最悪の挙動になる。人の指示が優先されるべきなので、こちらが退く。

## 安全側の約束

- **`FAULT` / `E_STOP` / `DISCONNECTED` / `STANDBY` では巡回を始めない**
  （`/g1/bridge_status` を見る）。走行中にそうなったら **`HOLD` に落ちる**
- ⚠️ **自動では復帰しない。** 復帰は人が `/g1/patrol/start` を呼ぶこと。
  `clear_fault` した瞬間に巡回が勝手に再開すると、人は「復帰させた」だけの
  つもりなので驚きが大きい（`g1_cmd_router` が Goal をキャンセルするのと同じ思想）
- **発進ゲート（`G1_ARM`）と heartbeat（D-31）は素通りしない。** このノードは
  `NavigateToPose` に Goal を送るだけで、速度指令は従来どおり
  `velocity_smoother → g1_cmd_router → SDK` を通る。**安全機構は一切迂回しない**

## ⚠️ 内蔵SLAM の16分停止との関係

内蔵SLAM が落ちると TF が止まり、`g1_cmd_router` が `FAULT` に落ちるので、
**巡回は自動的に `HOLD` する**（暴走しない）。再定位のあと
`/g1/patrol/start` を呼べば**止まった地点の次から**続きを回る（`index` を保つ）。

## RViz で巡回路を引く（教示モード）

    ./tools/patrol_ctl.sh teach     # TEACH に入る。以後 RViz の「Publish Point」が点になる
    # RViz で回りたい順に地面をクリックしていく（/g1/patrol/route に見える）
    ./tools/patrol_ctl.sh save      # yaml に書き、そのまま巡回路として読み込む
    ./tools/patrol_ctl.sh undo      # 直前の1点を取り消す
    ./tools/patrol_ctl.sh cancel    # 全部捨てて TEACH を抜ける

⚠️⚠️ **「2D Goal Pose」は教示に使えない。** `bt_navigator` が `/goal_pose` を直接
購読しているので、**クリックした瞬間に機体が本当にそこへ歩き出す**。
RViz は同じツールを2つ置けるが、どちらも「2D Goal Pose」という同じ名前で
並んでしまい現場で取り違える。そこで**別ツールの「Publish Point」**
(`/clicked_point`) を使う。

📌 **向き(`yaw`)は「次の点へ向かう方位」を自動で入れる。** 巡回では普通それが
正しいし、クリックのたびに矢印を引かせるより速い。特定の向きで止まりたい点は
あとから yaml の `yaw_deg` を書き換えるか、`tools/record_waypoints.py` で取り直す。

⚠️ **RViz でクリックするのは「地図の上の座標」**。room_a の地図は 2026-09-07 取得で
現状と合っていないので、**機体を実際にそこへ立たせて拾う `record_waypoints.py` の
ほうが確実**。教示でざっと引いて、危ない点だけ取り直すのが実務的。

## ウェイポイントファイル

    frame_id: map          # 省略時 map
    waypoints:
      - {name: sw, x: -3.0, y: -3.5, yaw_deg: 90, dwell_s: 3.0}

`yaw_deg` は**度**（人が手で書くので rad より読み違えにくい）。`dwell_s` は
その点で止まる秒数。

⚠️ **点ごとの `dwell_s` はパラメータの `dwell_s` より優先される。** 巡回路に
書いてあると `patrol_dwell_s:=...` は**黙って無視される**（2026-09-16 に、これで
「FAULT が再現しない＝問題なし」という真逆の結論を出しかけた）。
"""

import json
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

# `g1_cmd_router` が出す状態名のうち、**走ってよいもの**。
#
# ⚠️⚠️ **`NAVIGATING` だけ。`READY` を入れてはいけない。**
# 状態機械はこうなっている（`g1_sdk_bridge/safety_manager.cpp`）:
#
#   DISCONNECTED → STANDBY → READY            ← TF とセンサーが健全になっただけ
#                              ↓ enable_navigation(true)
#                          NAVIGATING          ← **ここで初めて速度指令が SDK へ通る**
#
# `READY` は「走ってよい」ではなく「走る準備が整った」。この段階では
# `OnNavTwist` が `SendZero()` を返すので、巡回を始めても**機体は1mmも動かないまま
# Goal が abort され、全点を空振りで消化する**（2026-09-16 のモック確認で実際に踏んだ。
# `bridge: READY` のまま start が通ってしまった）。
BRIDGE_OK_STATES = ("NAVIGATING",)


def yaw_to_quat(yaw: float):
    """z 軸まわりだけの回転をクォータニオンにする。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Waypoint:
    def __init__(self, idx: int, raw: dict):
        self.name: str = str(raw.get("name", f"wp{idx}"))
        try:
            self.x = float(raw["x"])
            self.y = float(raw["y"])
        except (KeyError, TypeError, ValueError) as e:
            raise RuntimeError(f"ウェイポイント {idx} に x/y が無いか数値でない: {raw!r}") from e
        # yaw_deg（推奨）と yaw（rad）の両方を受ける
        if "yaw_deg" in raw:
            self.yaw = math.radians(float(raw["yaw_deg"]))
        else:
            self.yaw = float(raw.get("yaw", 0.0))
        self.dwell_s: Optional[float] = (
            float(raw["dwell_s"]) if raw.get("dwell_s") is not None else None)

    def __str__(self) -> str:
        return f"{self.name}({self.x:.2f},{self.y:.2f},{math.degrees(self.yaw):.0f}deg)"


class PatrolNode(Node):
    def __init__(self):
        super().__init__("g1_patrol")

        self.waypoints_file = self.declare_parameter("waypoints_file", "").value
        self.loop = self.declare_parameter("loop", True).value
        # ⚠️⚠️ **既定 0。各点で長く止まると機体が FAULT に落ちる。**
        # Goal が終わると `controller_server` は指令を出さなくなる。`velocity_smoother` は
        # velocity_timeout(既定 1.0 秒)ぶん余分に出してからゼロへ落として黙る。そこから
        # `cmd_timeout`(0.30 秒)で `g1_cmd_router` が FAULT にする。つまり
        # **止まっていられるのは 1.3 秒ほどしかない**。
        # 2026-09-16 のモック確認: `dwell_s=5.0` で**1点目の直後に FAULT**
        # (`hold_reason: bridge=FAULT`)。`dwell_s=1.0` なら 5 周回っても落ちなかった。
        # 長く止まりたいならこの制約自体を片付ける必要がある
        # (findings/patrol_mode.md §6 に選択肢を3つ並べてある)。
        self.dwell_s = self.declare_parameter("dwell_s", 0.0).value
        # 1点あたりの上限。超えたらキャンセルして失敗扱いにする。
        # ⚠️ Nav2 側の progress checker(20秒)より必ず長くすること。短いと
        # 「Nav2 は諦めていないのにこちらが切る」ので原因が追いにくくなる。
        self.goal_timeout_s = self.declare_parameter("goal_timeout_s", 180.0).value
        self.max_retries = int(self.declare_parameter("max_retries", 1).value)
        # ⚠️ **1.0 秒。上の `dwell_s` と同じ 1.3 秒の壁がここにも効く。**
        # 失敗して 3 秒待つ実装にしていたら、その間に cmd_timeout で FAULT に落ちて
        # 「再試行のはずが HOLD」になる。バックオフを伸ばすなら §6 の判断が先。
        self.retry_dwell_s = self.declare_parameter("retry_dwell_s", 1.0).value
        # skip=その点を諦めて次へ / stop=巡回を止める
        self.on_failure = self.declare_parameter("on_failure", "skip").value
        self.autostart = self.declare_parameter("autostart", False).value
        self.start_delay_s = self.declare_parameter("start_delay_s", 10.0).value
        self.require_bridge_ok = self.declare_parameter("require_bridge_ok", True).value
        self.yield_to_manual_goal = self.declare_parameter("yield_to_manual_goal", True).value
        self.bridge_status_topic = self.declare_parameter(
            "bridge_status_topic", "/g1/bridge_status").value
        # 診断が何秒来なければ「見えていない」と判断するか
        self.bridge_stale_s = self.declare_parameter("bridge_stale_s", 3.0).value
        # 教示モードの書き出し先。⚠️ **既定を waypoints_file にはしない。**
        # 実機の既定は install/share 配下（--symlink-install でリポジトリの実体）なので、
        # 上書きするとリポジトリが汚れるし、元のひな形も失う。
        self.teach_output = self.declare_parameter("teach_output", "").value
        if not self.teach_output:
            home = os.path.expanduser("~")
            base = os.path.join(home, "g1_nav2")
            self.teach_output = os.path.join(
                base if os.path.isdir(base) else home, "patrol_taught.yaml")

        if self.on_failure not in ("skip", "stop"):
            raise RuntimeError(f"on_failure は skip か stop: {self.on_failure}")

        self.frame_id = "map"
        self.waypoints: List[Waypoint] = []
        self.load_error: Optional[str] = None
        self._load_waypoints()

        self.state = "IDLE"            # IDLE / RUNNING / HOLD / DONE / TEACH
        self.index = 0
        self.loop_count = 0
        self.retries = 0
        self.last_result = ""
        self.hold_reason = ""
        self._goal_handle = None
        self._goal_in_flight = False   # send_goal_async 〜 結果受領 まで
        self._goal_sent_at = 0.0
        self._wait_until = 0.0         # dwell / retry の待ち
        self._bridge_state = ""
        self._bridge_stamp = 0.0
        self._teach: List[Dict[str, float]] = []   # 教示中に溜めた点
        self._autostart_at = time.monotonic() + float(self.start_delay_s)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(
            DiagnosticArray, self.bridge_status_topic, self._on_bridge_status, 10)
        # ⚠️ **購読するだけ。ここから Goal は送らない。** 人が RViz から送った Goal を
        # 検出して巡回を畳むためだけに見ている。
        self.create_subscription(PoseStamped, "/goal_pose", self._on_manual_goal, 10)

        # ⚠️ 教示は **「Publish Point」** を使う。「2D Goal Pose」は bt_navigator が
        # 直接購読しているので、クリックした瞬間に機体が本当に歩き出してしまう。
        self.create_subscription(PointStamped, "/clicked_point", self._on_clicked_point, 10)

        self._pub_status = self.create_publisher(String, "/g1/patrol/status", 10)
        # 巡回路を RViz で見えるようにする。
        # ⚠️⚠️ **transient_local だけに頼らないこと。** 2026-09-16 のモック確認で、
        # CycloneDDS では**あとから張った購読に latched の1件が届かなかった**
        # (先に購読を張れば届く)。RViz は Nav2 より後に立ち上げるのが普通なので、
        # これだと**巡回路が見えない**。**1Hz で出し直す**ことで依存を断つ
        # (4点の Path なので負荷は無視できる)。durability はそのまま残してある
        # (届く環境では即座に出るので、あって損はない)。
        self._pub_route = self.create_publisher(
            Path, "/g1/patrol/route",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "/g1/patrol/start", self._srv_start)
        self.create_service(Trigger, "/g1/patrol/pause", self._srv_pause)
        self.create_service(Trigger, "/g1/patrol/stop", self._srv_stop)
        self.create_service(Trigger, "/g1/patrol/skip", self._srv_skip)
        self.create_service(Trigger, "/g1/patrol/teach", self._srv_teach)
        self.create_service(Trigger, "/g1/patrol/teach_save", self._srv_teach_save)
        self.create_service(Trigger, "/g1/patrol/teach_undo", self._srv_teach_undo)
        self.create_service(Trigger, "/g1/patrol/teach_cancel", self._srv_teach_cancel)

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_status)
        # ⚠️ 遅れて立ち上がる RViz のために出し直す（上の publisher のコメント参照）
        self.create_timer(1.0, self._publish_route)

        self._publish_route()
        if self.load_error:
            self.get_logger().error(f"❌ ウェイポイントを読めない: {self.load_error}")
        self.get_logger().info(
            f"g1_patrol 起動。{len(self.waypoints)} 点 / loop={self.loop} / "
            f"IDLE のまま待機する（/g1/patrol/start で開始）")
        if self.autostart:
            self.get_logger().warn(
                f"⚠️ autostart=true。{self.start_delay_s:.0f} 秒後に自動で巡回を始める")
        longest = max([self.dwell_s, self.retry_dwell_s]
                      + [w.dwell_s for w in self.waypoints if w.dwell_s is not None],
                      default=0.0)
        if longest > 1.0:
            self.get_logger().warn(
                f"⚠️ 待ち時間が {longest:.1f} 秒ある。**1.3 秒を超えると cmd_timeout で "
                f"FAULT に落ちて巡回が HOLD する**（2026-09-16 モックで実測）。"
                "findings/patrol_mode.md §6 を読むこと")

    # --- ウェイポイント ----------------------------------------------------
    def _load_waypoints(self) -> None:
        path = self.waypoints_file
        if not path:
            self.load_error = "waypoints_file が未指定"
            return
        if not os.path.exists(path):
            self.load_error = f"ファイルが無い: {path}"
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f) or {}
            self.frame_id = str(doc.get("frame_id", "map"))
            raw = doc.get("waypoints") or []
            self.waypoints = [Waypoint(i, w) for i, w in enumerate(raw)]
        except Exception as e:  # noqa: BLE001 - 原因をそのまま status に出したい
            self.load_error = f"{type(e).__name__}: {e}"
            return
        if not self.waypoints:
            self.load_error = f"ウェイポイントが0点: {path}"
            return
        self.load_error = None
        self.get_logger().info(
            f"ウェイポイント {len(self.waypoints)} 点を読んだ（frame={self.frame_id}）: "
            + " → ".join(str(w) for w in self.waypoints))

    # --- 外から来るもの ----------------------------------------------------
    def _on_bridge_status(self, msg: DiagnosticArray) -> None:
        for st in msg.status:
            if st.name == "g1_cmd_router":
                self._bridge_state = st.message
                self._bridge_stamp = time.monotonic()

    def _bridge_ok(self) -> tuple:
        """(走ってよいか, 理由)。"""
        if not self.require_bridge_ok:
            return True, ""
        if not self._bridge_state:
            return False, f"{self.bridge_status_topic} が来ていない"
        age = time.monotonic() - self._bridge_stamp
        if age > self.bridge_stale_s:
            return False, f"{self.bridge_status_topic} が {age:.1f} 秒途絶"
        if self._bridge_state not in BRIDGE_OK_STATES:
            return False, f"bridge={self._bridge_state}"
        return True, ""

    def _on_manual_goal(self, msg: PoseStamped) -> None:
        if self.state != "RUNNING" or not self.yield_to_manual_goal:
            return
        self.get_logger().warn(
            "⚠️ 手動の Goal を検出した。**巡回を畳んで HOLD に退く**"
            "（bt_navigator は Goal を1件しか持てないため）。"
            "巡回に戻すときは /g1/patrol/start")
        self._to_hold("手動Goalに譲った")

    # --- サービス ----------------------------------------------------------
    def _srv_start(self, req, res):
        del req
        if self.state == "TEACH":
            res.success = False
            res.message = (f"教示中（{len(self._teach)} 点）。"
                           "teach_save で確定するか teach_cancel で捨ててから start すること")
            return res
        if self.load_error:
            res.success, res.message = False, f"ウェイポイントが無い: {self.load_error}"
            return res
        ok, why = self._bridge_ok()
        if not ok:
            res.success = False
            res.message = (f"走行できる状態ではない（{why}）。"
                           "bridge=READY なら enable_navigation がまだ、"
                           "STANDBY なら TF/センサーがまだ、FAULT なら clear_fault が要る")
            return res
        if self.state == "RUNNING":
            res.success, res.message = True, f"すでに巡回中（{self.index}/{len(self.waypoints)}）"
            return res
        if self.state == "DONE":
            self.index, self.loop_count = 0, 0
        self.retries = 0
        self._wait_until = 0.0
        self.hold_reason = ""
        self.state = "RUNNING"
        self.get_logger().info(f"▶ 巡回開始（{self.index + 1}/{len(self.waypoints)} 点目から）")
        res.success = True
        res.message = f"巡回を開始する（{self.index + 1}/{len(self.waypoints)} 点目から）"
        return res

    def _srv_pause(self, req, res):
        del req
        if self.state != "RUNNING":
            res.success, res.message = True, f"巡回していない（{self.state}）"
            return res
        self._to_hold("pause")
        res.success, res.message = True, f"一時停止した（次は {self.index + 1} 点目）"
        return res

    def _srv_stop(self, req, res):
        del req
        was_teaching = self.state == "TEACH"
        n_teach = len(self._teach)
        self._cancel_current("stop")
        self._teach = []
        self.state = "IDLE"
        self.index, self.loop_count, self.retries = 0, 0, 0
        self.hold_reason = ""
        self._publish_route()
        if was_teaching:
            res.message = f"教示をやめた（{n_teach} 点は捨てた）"
        else:
            res.message = "巡回を止めた。次の start は1点目から"
        self.get_logger().info(f"■ {res.message}")
        res.success = True
        return res

    def _srv_skip(self, req, res):
        del req
        if not self.waypoints:
            res.success, res.message = False, "ウェイポイントが無い"
            return res
        skipped = self.waypoints[self.index].name
        self._cancel_current("skip")
        self._advance()
        res.success = True
        res.message = f"{skipped} を飛ばした（次は {self.waypoints[self.index].name}）"
        self.get_logger().info(f"⏭ {res.message}")
        return res

    # --- 状態遷移 ----------------------------------------------------------
    def _to_hold(self, reason: str) -> None:
        self._cancel_current(reason)
        self.state = "HOLD"
        self.hold_reason = reason
        self.get_logger().warn(
            f"⏸ HOLD（{reason}）。**自動では再開しない。** "
            f"/g1/patrol/start で {self.index + 1} 点目から再開する")

    def _cancel_current(self, reason: str) -> None:
        del reason  # ログは呼び出し側で出している
        self._wait_until = 0.0
        if self._goal_handle is not None and self._goal_in_flight:
            self._goal_handle.cancel_goal_async()
        self._goal_in_flight = False
        self._goal_handle = None

    def _advance(self) -> None:
        self.retries = 0
        self.index += 1
        if self.index >= len(self.waypoints):
            self.index = 0
            self.loop_count += 1
            if not self.loop:
                self.state = "DONE"
                self.get_logger().info("🏁 巡回を1周して終わり（loop=false）")

    # --- 本体 --------------------------------------------------------------
    def _tick(self) -> None:
        if self.autostart and self.state == "IDLE" and not self.load_error:
            if time.monotonic() >= self._autostart_at:
                self.autostart = False  # 一度だけ
                req = Trigger.Request()
                res = self._srv_start(req, Trigger.Response())
                if not res.success:
                    self.get_logger().error(f"autostart できない: {res.message}")

        if self.state != "RUNNING":
            return

        ok, why = self._bridge_ok()
        if not ok:
            self._to_hold(why)
            return

        if self._goal_in_flight:
            if time.monotonic() - self._goal_sent_at > self.goal_timeout_s:
                self.get_logger().warn(
                    f"⌛ {self.waypoints[self.index].name} が "
                    f"{self.goal_timeout_s:.0f} 秒で終わらない。キャンセルして失敗扱いにする")
                self._cancel_current("timeout")
                self._on_failed("timeout")
            return

        if time.monotonic() < self._wait_until:
            return

        self._send_current_goal()

    def _send_current_goal(self) -> None:
        if not self._client.server_is_ready():
            # Nav2 がまだ上がりきっていないだけ。次の tick で再試行する。
            # ⚠️ 黙って待ち続けると「RUNNING なのに何も起きない」になるので必ず出す。
            self.get_logger().warn(
                "navigate_to_pose がまだ出ていない（bt_navigator が activate 済みか）",
                throttle_duration_sec=5.0)
            return
        wp = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        qx, qy, qz, qw = yaw_to_quat(wp.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_in_flight = True
        self._goal_sent_at = time.monotonic()
        self.get_logger().info(
            f"→ {self.index + 1}/{len(self.waypoints)} {wp} "
            f"（周回 {self.loop_count + 1}、再試行 {self.retries}）")
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Goal を送れなかった: {e}")
            self._goal_in_flight = False
            self._on_failed("send_failed")
            return
        if not handle.accepted:
            self.get_logger().error("Goal が拒否された（bt_navigator が受け付けない）")
            self._goal_in_flight = False
            self._on_failed("rejected")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        if not self._goal_in_flight:
            return  # 自分でキャンセル済み。後始末はキャンセル側でやってある
        self._goal_in_flight = False
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"結果を受け取れなかった: {e}")
            self._on_failed("result_error")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            wp = self.waypoints[self.index]
            dwell = wp.dwell_s if wp.dwell_s is not None else self.dwell_s
            self.last_result = "SUCCEEDED"
            self.get_logger().info(f"✅ {wp.name} 到達。{dwell:.1f} 秒待つ")
            self._advance()
            self._wait_until = time.monotonic() + float(dwell)
            return

        if status == GoalStatus.STATUS_CANCELED:
            # 自分で切ったのでなければ、外（`g1_cmd_router` の FAULT 時キャンセルか人）が
            # 切った。**勝手に再送しない。**
            self.last_result = "CANCELED"
            if self.state == "RUNNING":
                self._to_hold("外部からGoalがキャンセルされた")
            return

        self.last_result = f"ABORTED(status={status})"
        self._on_failed(self.last_result)

    def _on_failed(self, why: str) -> None:
        if self.state != "RUNNING":
            return
        wp = self.waypoints[self.index]
        if self.retries < self.max_retries:
            self.retries += 1
            self.get_logger().warn(
                f"⚠️ {wp.name} に失敗（{why}）。{self.retry_dwell_s:.1f} 秒後に "
                f"再試行 {self.retries}/{self.max_retries}")
            self._wait_until = time.monotonic() + float(self.retry_dwell_s)
            return
        if self.on_failure == "stop":
            self.get_logger().error(f"❌ {wp.name} に失敗（{why}）。on_failure=stop なので止める")
            self._to_hold(f"{wp.name} に到達できない: {why}")
            return
        self.get_logger().error(f"❌ {wp.name} に失敗（{why}）。飛ばして次へ（on_failure=skip）")
        self._advance()
        self._wait_until = time.monotonic() + float(self.retry_dwell_s)

    # --- 教示モード（RViz の Publish Point で巡回路を引く）-------------------
    def _srv_teach(self, req, res):
        del req
        if self.state == "TEACH":
            res.success, res.message = True, f"すでに教示中（{len(self._teach)} 点）"
            return res
        if self.state == "RUNNING":
            # 人が明示的に教示へ入るのだから、巡回は畳む（手動が勝つのと同じ思想）
            self.get_logger().warn("⚠️ 巡回中だったので畳んで教示に入る")
            self._cancel_current("teach")
        self.state = "TEACH"
        self._teach = []
        self.hold_reason = ""
        self._publish_route()
        self.get_logger().info(
            f"✏️ 教示モード。RViz の **「Publish Point」**で回りたい順にクリックする"
            f"（⚠️「2D Goal Pose」ではない。あれは本当に歩き出す）。"
            f"書き出し先: {self.teach_output}")
        res.success = True
        res.message = (f"教示モード。RViz の「Publish Point」でクリック → save で確定。"
                       f"書き出し先 {self.teach_output}")
        return res

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if self.state != "TEACH":
            return   # 教示中でなければ何もしない（誤クリックで何も起きない）
        frame = msg.header.frame_id or self.frame_id
        if frame != self.frame_id:
            self.get_logger().warn(
                f"⚠️ クリックが {frame} 座標系で来た（巡回路は {self.frame_id}）。"
                "RViz の Fixed Frame を map にすること。この点は捨てる")
            return
        self._teach.append({"x": round(float(msg.point.x), 3),
                            "y": round(float(msg.point.y), 3)})
        self._publish_route()
        self.get_logger().info(
            f"✏️ {len(self._teach)} 点目 ({self._teach[-1]['x']}, {self._teach[-1]['y']})")

    def _srv_teach_undo(self, req, res):
        del req
        if self.state != "TEACH":
            res.success, res.message = False, "教示中ではない"
            return res
        if not self._teach:
            res.success, res.message = False, "取り消す点が無い"
            return res
        dropped = self._teach.pop()
        self._publish_route()
        res.success = True
        res.message = f"({dropped['x']}, {dropped['y']}) を取り消した（残り {len(self._teach)} 点）"
        self.get_logger().info(f"↩ {res.message}")
        return res

    def _srv_teach_cancel(self, req, res):
        del req
        if self.state != "TEACH":
            res.success, res.message = False, "教示中ではない"
            return res
        n = len(self._teach)
        self._teach = []
        self.state = "IDLE"
        self._publish_route()
        res.success, res.message = True, f"{n} 点を捨てて教示をやめた"
        self.get_logger().info(f"🗑 {res.message}")
        return res

    def _teach_with_yaw(self) -> List[dict]:
        """教示した点に「次の点へ向かう方位」を入れて返す。

        📌 巡回では普通それが正しい向きで、クリックのたびに矢印を引かせるより速い。
        最後の点は、周回なら1点目へ向かう方位、1周きりなら**手前の区間の方位**を引き継ぐ。
        """
        pts = self._teach
        out: List[dict] = []
        last_yaw = 0.0
        for i, p in enumerate(pts):
            nxt = None
            if i + 1 < len(pts):
                nxt = pts[i + 1]
            elif self.loop and len(pts) > 1:
                nxt = pts[0]
            if nxt is not None:
                last_yaw = math.atan2(nxt["y"] - p["y"], nxt["x"] - p["x"])
            out.append({"name": f"wp{i + 1}", "x": p["x"], "y": p["y"],
                        "yaw_deg": round(math.degrees(last_yaw), 1)})
        return out

    def _srv_teach_save(self, req, res):
        del req
        if self.state != "TEACH":
            res.success, res.message = False, "教示中ではない"
            return res
        if len(self._teach) < 2:
            res.success, res.message = False, f"点が {len(self._teach)} しかない（2点以上要る）"
            return res
        points = self._teach_with_yaw()
        try:
            os.makedirs(os.path.dirname(self.teach_output) or ".", exist_ok=True)
            with open(self.teach_output, "w", encoding="utf-8") as f:
                f.write("# RViz の「Publish Point」で引いた巡回路（patrol_node.py の教示モード）\n")
                f.write("# yaw_deg は「次の点へ向かう方位」を自動で入れたもの。\n")
                f.write("# ⚠️ 地図の上でクリックした座標。地図が古いと実際には通れないことがある。\n")
                f.write("#    怪しい点は tools/record_waypoints.py で取り直すこと。\n")
                f.write(f"frame_id: {self.frame_id}\n")
                f.write("waypoints:\n")
                for p in points:
                    f.write(f"  - {{name: {p['name']}, x: {p['x']}, y: {p['y']}, "
                            f"yaw_deg: {p['yaw_deg']}}}\n")
        except OSError as e:
            res.success, res.message = False, f"書けなかった: {e}"
            self.get_logger().error(res.message)
            return res

        # そのまま使えるように差し替える（**再起動を要らなくするのが教示モードの眼目**）
        self.waypoints = [Waypoint(i, w) for i, w in enumerate(points)]
        self.load_error = None
        self._teach = []
        self.state = "IDLE"
        self.index, self.loop_count, self.retries = 0, 0, 0
        self._publish_route()
        res.success = True
        res.message = (f"{len(points)} 点を {self.teach_output} に書き、そのまま読み込んだ。"
                       "start で走る")
        self.get_logger().info(f"💾 {res.message}")
        return res

    # --- 見える化 ----------------------------------------------------------
    def _publish_route(self) -> None:
        """いまの巡回路（教示中なら教示中の点）を Path で流す。RViz で見るため。"""
        pts = (self._teach_with_yaw() if self.state == "TEACH"
               else [{"x": w.x, "y": w.y, "yaw_deg": math.degrees(w.yaw)}
                     for w in self.waypoints])
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for p in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p["x"])
            ps.pose.position.y = float(p["y"])
            _, _, qz, qw = yaw_to_quat(math.radians(float(p["yaw_deg"])))
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self._pub_route.publish(path)

    def _publish_status(self) -> None:
        wp = self.waypoints[self.index] if self.waypoints else None
        msg = String()
        msg.data = json.dumps({
            "state": self.state,
            "index": self.index,
            "waypoints": len(self.waypoints),
            "current": wp.name if wp else None,
            "loop_count": self.loop_count,
            "retries": self.retries,
            "last_result": self.last_result,
            "hold_reason": self.hold_reason,
            "bridge": self._bridge_state or "unknown",
            "teach_points": len(self._teach),
            "error": self.load_error or "",
        }, ensure_ascii=False)
        self._pub_status.publish(msg)


def main() -> None:
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
