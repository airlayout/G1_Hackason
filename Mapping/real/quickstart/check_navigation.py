#!/usr/bin/env python3
"""Nav2 に実際にゴールを投げて、**着くかどうか**を測る。

`check_planning.py` は経路が引けるか（`ComputePathToPose`）までしか見ない。
こちらは `NavigateToPose` を投げ、歩行ポリシーが動いて到達するかまでを見る。

## ⚠️ **経路が引けることは、着けることの証拠にならない**（2026-09-08 に踏んだ）

掃除済み地図 + Isaac Sim で `check_planning.py` は **8/8 通った**（機体セルのコストも 0）。
同じ状態で歩かせると **3 回とも着かない**。落ちたのは 2 か所:

- **測位が歩行中に発散する。** 4.5 m 歩く間に真値とのずれが 0.17 → 2.32 m。
  AMCL は「着いた」と思っている
- **障害物に乗り上げる。** 真値 z が 0.63 → 1.02 m（＝高さ 1.00 m の机の天面）。
  `runner.py` は転倒防止で後退を禁じているので、嵌ると抜けられない

**だから「着かない」を見たら、まず真値 z を見る**（`G1_SIM_LOG` が指すログの
`pos=` の 3 番目）。0.63 から外れていたら乗り上げで、地図もプランナも関係ない。

## ⚠️ 立っているだけで機体は動く

G1 は指令ゼロでも約 3.4 cm/s 後退する（ポリシーの残留速度）。
**ゴールは投げる直前の姿勢から選ぶ。**固定のゴールを使い回さない。

使い方:
    python3 check_navigation.py --waypoints waypoints.json --tries 3 --record /tmp/rec
"""
from __future__ import annotations

import argparse
import json
import os
import math
import re
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

# Isaac Sim の真値ログ。**/odom の z は 0 固定なので、乗り上げはここでしか分からない。**
# 実機で使うときは存在しないので、z の欄は空になる（それ以外は同じに動く）。
SIM_LOG = Path(os.environ.get(
    "G1_SIM_LOG", Path.home() / "G1_Hackason/IsaacSim_Env/logs/nav2_sim.log"))
ERROR_NAMES = {0: "NONE", 1: "UNKNOWN", 2: "FAILED_TO_LOAD_BEHAVIOR_TREE",
               3: "NO_VALID_PATH", 4: "TIMEOUT", 5: "TF_ERROR",
               6: "INVALID_PATH", 7: "GOAL_OCCUPIED"}


def sim_z() -> float | None:
    """Isaac Sim のログから真値 z を読む。**乗り上げの検出はこれでしかできない**
    （/odom の z は 0 固定）。5 秒ごとの粗いサンプルだが、乗り上げは遅い現象。"""
    try:
        m = re.findall(r"pos=\([-+0-9.]+, [-+0-9.]+, ([-+0-9.]+)\)",
                       SIM_LOG.read_text())
    except OSError:
        return None
    return float(m[-1]) if m else None


def sim_truth() -> tuple[float, float] | None:
    """Isaac Sim が 5 秒ごとに出す真値の最新を読む。"""
    try:
        m = re.findall(r"pos=\(([-+0-9.]+), ([-+0-9.]+),", SIM_LOG.read_text())
    except OSError:
        return None
    return (float(m[-1][0]), float(m[-1][1])) if m else None


class Navigator(Node):
    def __init__(self) -> None:
        super().__init__("run_navigation_test", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.feedback: list = []
        # --record 用。/odom は Isaac Sim の真値そのもの（runner.py の
        # _publish_ros が root_pos_w をそのまま流す）。ただし z は 0 固定なので
        # 乗り上げ（真値 z）はシムのログから別に読む。
        self.truth: tuple[float, float, float] | None = None
        self.plan_xy: list = []
        self.cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(NavPath, "/plan", self._on_plan, 5)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.track: list[dict] = []
        self.recording = False

    def _on_odom(self, m) -> None:
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.truth = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _on_plan(self, m) -> None:
        self.plan_xy = [[p.pose.position.x, p.pose.position.y] for p in m.poses]

    def _on_cmd(self, m) -> None:
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def sample(self, goal_xy) -> None:
        """今の状態を 1 点ぶん記録する。**測定には影響しない。**"""
        est = self.pose(retries=1)
        self.track.append({
            "t": time.time(),
            "sim_z": sim_z(),
            "truth": list(self.truth) if self.truth else None,
            "amcl": list(est) if est else None,
            "plan": list(self.plan_xy),
            "cmd": list(self.cmd),
            "goal": list(goal_xy) if goal_xy else None,
        })

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def pose(self, retries: int = 10) -> tuple[float, float, float] | None:
        """map -> base_link を取る。

        ⚠️ TF バッファを作り直した直後は、ノードの sim 時刻が TF の
        タイムスタンプより 1 秒ほど古く、"latest" のつもりの照会が
        「earliest data より前」で落ちる。バッファが埋まるまで繰り返す。
        """
        last = None
        for _ in range(retries):
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time())
            except Exception as exc:
                last = exc
                self.spin_for(1.0)
                continue
            t, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.x, t.y, yaw)
        self.get_logger().warn(f"map -> base_link が引けない: {last}")
        return None

    def navigate(self, gx: float, gy: float, timeout_s: float,
                 planner_id: str = "") -> dict:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0
        goal.planner_id = planner_id

        self.feedback = []
        send = self.client.send_goal_async(
            goal, feedback_callback=lambda fb: self.feedback.append(fb.feedback))
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"accepted": False}

        result_future = handle.get_result_async()
        end = time.monotonic() + timeout_s
        next_sample = 0.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.recording and time.monotonic() >= next_sample:
                self.sample((gx, gy))
                next_sample = time.monotonic() + 0.5
        if not result_future.done():
            handle.cancel_goal_async()
            self.spin_for(2.0)
            return {"accepted": True, "timeout": True}

        res = result_future.result()
        fb = self.feedback[-1] if self.feedback else None
        return {
            "accepted": True,
            "timeout": False,
            "status": res.status,
            "succeeded": res.status == GoalStatus.STATUS_SUCCEEDED,
            "error_code": getattr(res.result, "error_code", None),
            "recoveries": getattr(fb, "number_of_recoveries", None),
            "distance_remaining": getattr(fb, "distance_remaining", None),
        }


def save_record(record: str | None, waypoints: list, results: list,
                track: list, planner_id: str) -> None:
    """記録を書き出す。1 回ごとに呼んで上書きする（途中で止めても残る）。"""
    if not record:
        return
    d = Path(record)
    d.mkdir(parents=True, exist_ok=True)
    (d / "navigation.json").write_text(json.dumps({
        "waypoints": waypoints, "results": results, "track": track,
        "planner_id": planner_id,
    }, ensure_ascii=False))
    print(f"[record] -> {d}/navigation.json（{len(track)} サンプル / "
          f"{len(results)} 回ぶん）")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", required=True)
    ap.add_argument("--tries", type=int, default=3)
    ap.add_argument("--range", type=float, nargs=2, default=[3.0, 8.0])
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--record", metavar="DIR",
                    help="真値・AMCL・経路・指令を 0.5 秒ごとに記録する（動画用）")
    ap.add_argument("--planner-id", default="", metavar="PLUGIN_NAME",
                    help="planner_server の planner_plugins に登録された名前 "
                    "（例: Smac2D, ThetaStar）。空なら既定（GridBased/NavFn）")
    args = ap.parse_args()

    waypoints = json.loads(Path(args.waypoints).read_text())
    rclpy.init()
    node = Navigator()
    node.recording = args.record is not None
    node.spin_for(5.0)
    if not node.client.wait_for_server(timeout_sec=15.0):
        print("[FAIL] navigate_to_pose のサーバが居ない")
        return 1
    print(f"[planner_id] {args.planner_id or '(既定)'}")

    lo, hi = args.range
    results = []
    for k in range(args.tries):
        # ⚠️ 1 回ごとに記録を書く。最後にまとめて書くと、途中で止めたときに
        # **記録が丸ごと消える**（長距離は 1 回で最大 900 s なので、
        # 「見込みが無いから止める」判断ができなくなる。2026-09-08 に踏んだ）。
        # continue で抜ける経路が 4 つあるので try/finally で確実に通す。
        try:
            node.spin_for(1.0)
            pose = node.pose()
            truth = sim_truth()
            if pose is None:
                print(f"  {k + 1}: TF が引けない")
                results.append(False)
                continue
            rx, ry, _ = pose
            # 投げる直前の姿勢から --range の帯に入るゴールのうち最も近いものを選ぶ。
            # 着いた先から次も同じ帯で選ぶので、帯を長くすると部屋を往復する形になる。
            near = [(math.hypot(w["x"] - rx, w["y"] - ry), w) for w in waypoints]
            band = sorted((d, w) for d, w in near if lo <= d <= hi)
            if not band:
                print(f"  {k + 1}: {lo}-{hi} m にゴール候補が無い（現在地 "
                      f"{rx:+.2f}, {ry:+.2f}）")
                results.append(False)
                continue
            dist, w = band[0]
            t = f"真値({truth[0]:+.2f}, {truth[1]:+.2f})" if truth else "真値不明"
            print(f"  {k + 1}: 現在地 AMCL({rx:+.2f}, {ry:+.2f}) {t} → "
                  f"ゴール({w['x']:+.2f}, {w['y']:+.2f}) 直線 {dist:.2f} m "
                  f"(clearance {w['clearance']:.2f} m)")

            r = node.navigate(w["x"], w["y"], args.timeout, args.planner_id)
            if not r["accepted"]:
                print("       ゴールが受理されなかった")
                results.append(False)
                continue
            if r.get("timeout"):
                end_pose, end_truth = node.pose(), sim_truth()
                print(f"       時間切れ（{args.timeout:.0f} s）。"
                      f"残り {r.get('distance_remaining')}")
                if end_pose and end_truth:
                    print(f"       到達点 AMCL({end_pose[0]:+.2f}, {end_pose[1]:+.2f}) "
                          f"真値({end_truth[0]:+.2f}, {end_truth[1]:+.2f})")
                results.append(False)
                continue

            end_pose, end_truth = node.pose(), sim_truth()
            err = ERROR_NAMES.get(r["error_code"], str(r["error_code"]))
            print(f"       {'成功' if r['succeeded'] else '失敗'} "
                  f"error_code={r['error_code']}({err}) "
                  f"recoveries={r['recoveries']}")
            if end_pose and end_truth:
                gap = math.hypot(end_truth[0] - w["x"], end_truth[1] - w["y"])
                drift = math.hypot(end_truth[0] - end_pose[0], end_truth[1] - end_pose[1])
                print(f"       到達点 AMCL({end_pose[0]:+.2f}, {end_pose[1]:+.2f}) "
                      f"真値({end_truth[0]:+.2f}, {end_truth[1]:+.2f})  "
                      f"真値とゴールの差 {gap:.2f} m / AMCL と真値の差 {drift:.2f} m")
            results.append(bool(r["succeeded"]))
        finally:
            save_record(args.record, waypoints, results, node.track,
                       args.planner_id)

    print()
    ok = sum(results)
    print(f"== 到達 {ok}/{len(results)} ==")
    save_record(args.record, waypoints, results, node.track, args.planner_id)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
