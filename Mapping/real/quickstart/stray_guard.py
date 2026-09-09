#!/usr/bin/env python3
"""ゴールを投げた場所からの逸脱と、経過時間を見張って Nav2 をキャンセルする。

## なぜ要るのか

`check_navigation.py` にも `--max-stray` があるが、**あれは自分が投げたゴールしか守れない。**
RViz2 の「2D Goal Pose」からクリックすると保護が無くなる。しかも RViz のクリックには
**timeout が無い**ので、BT が 6 回リトライしながら Spin を繰り返すと数分動き続ける。

2026-09-09 に実機で踏んだ形（詳細は README-nav2 §7）:
**1.79m のゴールに対して累積 5.59m 動き 174° 回った。**ゴールの向きの指定が
機体と無関係な固定値だったため、位置は達成しても yaw 許容を満たせず、
その場旋回 → 進捗判定が並進しか数えないので失敗 → 復帰動作の Spin 90°。
**転倒もエラーも出ないので「なぜか着かない」としか見えない。**

このノードは**投げ方に関係なく**効く。常駐させておく。

## 何を見るか

- `/goal_pose` を受けた瞬間の `map -> base_link` を「開始点」として覚える
- 開始点からの距離が **ゴールまでの直線 + `--margin`** を超えたらキャンセル
- `--max-abs` を超えたら（ゴールに関係なく）キャンセル
- ゴールを受けてから `--max-seconds` 経ったらキャンセル
- `/navigate_to_pose/_action/status` でゴールの終了を見て待機に戻る

⚠️ **キャンセルは全ゴールに効く**（`cancel_nav.py` と同じ全ゼロ要求）。
複数人が同じ Nav2 にゴールを投げる運用では使わないこと。

使い方（Nav2 と同じ DDS 設定で。実機は --no-sim-time が要る）:
    python3 stray_guard.py --margin 0.5 --max-abs 3.0 --max-seconds 60 --no-sim-time
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

CANCEL_SERVICE = "/navigate_to_pose/_action/cancel_goal"
STATUS_TOPIC = "/navigate_to_pose/_action/status"
# ゴールが「まだ動いている」とみなす状態。これ以外になったら待機に戻る
ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING}


class StrayGuard(Node):
    def __init__(self, args) -> None:
        super().__init__("stray_guard", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, not args.no_sim_time)])
        self.a = args
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.cli = self.create_client(CancelGoal, CANCEL_SERVICE)
        self.watch: dict | None = None       # 見張り中の情報。None なら待機
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(GoalStatusArray, STATUS_TOPIC, self._on_status, 10)
        self.create_timer(1.0 / max(args.rate, 0.1), self._tick)
        self.get_logger().info(
            "[guard] 待機。余裕 {:.2f}m / 絶対上限 {:.2f}m / 時間上限 {:.0f}s".format(
                args.margin, args.max_abs, args.max_seconds))

    # ── 位置 ─────────────────────────────────────────────────────────
    def _xy(self) -> tuple[float, float] | None:
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return (tr.transform.translation.x, tr.transform.translation.y)

    # ── ゴールを受けた ───────────────────────────────────────────────
    def _on_goal(self, msg: PoseStamped) -> None:
        start = self._xy()
        if start is None:
            self.get_logger().error(
                "[guard] map -> base_link が引けない。**見張れないので投げ直すこと**")
            return
        gx, gy = msg.pose.position.x, msg.pose.position.y
        dist = math.hypot(gx - start[0], gy - start[1])
        leash = min(dist + self.a.margin, self.a.max_abs)
        self.watch = {"start": start, "goal": (gx, gy), "dist": dist,
                      "leash": leash, "t0": self.get_clock().now(), "fired": False}
        self.get_logger().info(
            "[guard] 見張り開始 開始点({:+.2f}, {:+.2f}) → ゴール({:+.2f}, {:+.2f}) "
            "直線 {:.2f}m / 上限 {:.2f}m / {:.0f}s".format(
                *start, gx, gy, dist, leash, self.a.max_seconds))

    # ── ゴールが終わった ─────────────────────────────────────────────
    def _on_status(self, msg: GoalStatusArray) -> None:
        if self.watch is None:
            return
        if any(s.status in ACTIVE for s in msg.status_list):
            return
        self.get_logger().info("[guard] ゴールが終了した。待機に戻る")
        self.watch = None

    # ── 監視 ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        w = self.watch
        if w is None or w["fired"]:
            return
        now = self._xy()
        if now is None:
            return
        d = math.hypot(now[0] - w["start"][0], now[1] - w["start"][1])
        elapsed = (self.get_clock().now() - w["t0"]).nanoseconds * 1e-9
        why = None
        if d > w["leash"]:
            why = "開始点から {:.2f}m（上限 {:.2f}m）".format(d, w["leash"])
        elif elapsed > self.a.max_seconds:
            why = "{:.0f}s 経過（上限 {:.0f}s）".format(elapsed, self.a.max_seconds)
        if why is None:
            return
        w["fired"] = True
        self.get_logger().error("[guard] **作動: " + why + "。キャンセルする**")
        self._cancel()

    def _cancel(self) -> None:
        if not self.cli.service_is_ready():
            # ここで待つと監視が止まるので待たない。次の tick で再試行される
            self.get_logger().error(
                "[guard] {} が居ない。**手で止めること**".format(CANCEL_SERVICE))
            if self.watch:
                self.watch["fired"] = False
            return
        fut = self.cli.call_async(CancelGoal.Request())   # 全ゼロ = 全ゴール
        fut.add_done_callback(self._cancelled)

    def _cancelled(self, fut) -> None:
        res = fut.result()
        if res is None:
            self.get_logger().error("[guard] キャンセルの応答が無い。**手で止めること**")
            return
        self.get_logger().error(
            "[guard] キャンセル完了 return_code={} / 対象 {} 件".format(
                res.return_code, len(res.goals_canceling)))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--margin", type=float, default=0.5,
                   help="ゴールまでの直線に足す余裕[m]。復帰動作の分だけ見ておく")
    p.add_argument("--max-abs", type=float, default=3.0,
                   help="開始点からの絶対上限[m]。遠いゴールでもここで必ず止める")
    p.add_argument("--max-seconds", type=float, default=60.0,
                   help="ゴールを受けてからの時間上限[s]。RViz のクリックには timeout が無い")
    p.add_argument("--rate", type=float, default=5.0, help="監視の周期[Hz]")
    p.add_argument("--no-sim-time", action="store_true",
                   help="実機で使うときに付ける（/clock が無いので必須）")
    args = p.parse_args(argv)

    rclpy.init()
    node = StrayGuard(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ⚠️ SIGTERM は ExternalShutdownException で来る（KeyboardInterrupt は SIGINT だけ）。
        # 拾わないと停止のたびに traceback が出て、本物のエラーと見分けが付かなくなる
        node.get_logger().info("[guard] 停止します")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
