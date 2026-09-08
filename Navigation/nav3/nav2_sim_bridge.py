#!/usr/bin/env python3
"""MuJoCo歩行シムをNav2のフルナビゲーションスタックに接続し、
「ゴールを与えたら実際にそこまで歩くか」を検証する。

## amcl_sim_verify.pyとの違い

`amcl_sim_verify.py`は「AMCLが正しく自己位置推定できるか」だけを、決め打ちの
動作台本で検証した。このスクリプトはその先を見る：`controller_server`が出す
`/cmd_vel`を実際にMuJoCoの歩行ポリシーへ渡し、`NavigateToPose`アクションで
ゴールを与えて、`planner_server`(経路計画) → `controller_server`(経路追従) →
歩行ポリシー、という一連が自律的に動くかを確かめる。

`amcl_sim_verify.Bridge`（/scan, /odom, /tf の発行）をそのまま継承し、
`/cmd_vel`購読だけを足す。地図・部屋は`amcl_sim_verify.py`と同じ
`sim/rooms.get_room("test_room")`を使う（地図とMuJoCoの世界が同じ元から
出ることが前提。モジュールdocstring参照）。

## 実行

  source /opt/ros/jazzy/setup.bash
  cd Navigation/nav3
  bash run_nav2_sim.sh   # map_server/amcl/controller_server/planner_server/
                          # behavior_server/bt_navigatorを一括起動し、
                          # このスクリプトを実行してゴールへ送り込む
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Callable

NAV_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from amcl_sim_verify import Bridge, CONTROL_TICK_S, grid_to_ros_map  # noqa: E402
from nav.protocol import Pose2D  # noqa: E402
from sim.rooms import get_room, room_from_point_cloud  # noqa: E402

DEFAULT_ROOM = "test_room"

# controller_serverからの/cmd_velがこれより長く途絶えたら停止する（安全側）。
# プランナが計画し直している間などは指令が来ないので、そのときは足踏みで待つ。
CMD_TIMEOUT_S = 0.5


class CmdVelBridge(Bridge):
    """親クラス(Bridge)にNav2からの/cmd_vel購読を足す。

    決め打ちの動作台本(amcl_sim_verify.pyのplan)の代わりに、
    controller_serverが出す速度指令をそのまま歩行ポリシーへ渡す。
    """

    def __init__(self, node, walker, lidar) -> None:
        super().__init__(node, walker, lidar)
        from geometry_msgs.msg import Twist

        self._last_cmd_time = time.monotonic()
        self._cmd_count = 0
        self._hold_pose: "Pose2D | None" = None
        node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

    def _on_cmd_vel(self, msg) -> None:
        self._walker.set_command(msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_time = time.monotonic()
        self._cmd_count += 1
        self._hold_pose = None  # 実指令が来ている間は保持しない

    def stop_if_stale(self) -> None:
        """cmd_velが途絶えていたら、その場に留まるよう補正指令を出す。

        ⚠️ 単純に`walker.stop()`(指令0)するだけでは実測で不十分だった:
        歩行ポリシーは速度指令追従用に学習されており「その場で位置を保つ」
        目的関数を持たないため、指令0でも歩幅の非対称などから
        ゆっくり特定方向へ**実際にドリフトする**。このドリフトだけで
        壁際まで押し出され、global costmapのinflationに引っかかって
        プランナが"Failed to create plan"を出し続けた実例がある
        （cmd_velが1本も来ていない=まだ経路すら計算されていない段階で
        1m近く動いていたことから判明）。
        シムなのでground truthが取れる特権を使い、静止指令に切り替わった
        瞬間の姿勢を`_hold_pose`として覚え、そこからのズレを打ち消す
        小さな補正速度を出す（実機のodom/AMCLでは使えない、シム専用の割り切り）。
        """
        if time.monotonic() - self._last_cmd_time <= CMD_TIMEOUT_S:
            return
        pose = self._walker.pose
        if self._hold_pose is None:
            self._hold_pose = pose
        dx_world = self._hold_pose.x - pose.x
        dy_world = self._hold_pose.y - pose.y
        cos_yaw, sin_yaw = math.cos(-pose.yaw), math.sin(-pose.yaw)
        vx = _clamp_hold(dx_world * cos_yaw - dy_world * sin_yaw)
        vy = _clamp_hold(dx_world * sin_yaw + dy_world * cos_yaw)
        self._walker.set_command(vx, vy, 0.0)


def _clamp_hold(error_m: float, gain: float = 1.0, max_speed: float = 0.15) -> float:
    return max(-max_speed, min(max_speed, error_m * gain))


def _open_viewer(walker):
    """MuJoCoのGUIビューアを開く。呼び出し元が`with`で使う想定。

    `sim/run_sim.py`の`--viewer`と同じ`mujoco.viewer.launch_passive`。
    Linuxでは無印pythonで動く(DISPLAYが要る。macOSのみmjpythonが必要)。
    """
    import mujoco.viewer

    print("[nav2-sim] MuJoCoビューアを開く(DISPLAYが必要)", flush=True)
    return mujoco.viewer.launch_passive(walker.model, walker.data)


def _make_frame_sync(viewer, walker, *, target: Pose2D | None) -> Callable[[], None]:
    """毎ステップ呼ぶと、ビューアに機体の軌跡と目標を描き直す。

    `sim/overlay.py`をそのまま使う(既存のwaypoint/target描画を流用。
    ここでは経由するwaypointが無いので`waypoints=[]`)。`--viewer`無しのときは
    `viewer`がNoneで、呼んでも何もしない関数を返す。
    """
    if viewer is None:
        return lambda: None

    from sim import overlay

    trail = overlay.Trail()

    def sync() -> None:
        if not viewer.is_running():
            return
        pose = walker.pose
        trail.add(pose.x, pose.y)
        viewer.user_scn.ngeom = 0
        overlay.draw(viewer.user_scn, waypoints=[], reached=-1, target=target, trail=trail)
        viewer.sync()

    return sync


def send_goal_and_run(node, bridge: CmdVelBridge, walker, goal_x: float, goal_y: float,
                       goal_yaw: float, timeout_s: float,
                       on_step: Callable[[], None] = lambda: None) -> int:
    """物理ループを止めずに、bt_navigatorのアクションサーバが立ち上がるのを待ってから送る。

    ⚠️ ここで`wait_for_server()`（ブロッキング）を使ってはいけない。
    `run_nav2_sim.sh`は「TFを流し続けるこのプロセス」を先に起動し、その後で
    controller_server/planner_server/behavior_server/bt_navigatorを起動する
    （逆順にすると、それらのlocal/global costmapがactivate時にTFを待って
    デッドロックする。実際に最初の実装でこれが起きた）。
    つまりこの関数が呼ばれた時点ではまだアクションサーバが無いのが通常であり、
    `server_is_ready()`（非ブロッキング）でポーリングしながら物理ループを回し続ける。
    """
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    import rclpy

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = goal_x
    goal.pose.pose.position.y = goal_y
    goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

    # ⚠️ `server_is_ready()`はDDSレベルのアクションサーバ発見（configure時点で
    # 存在する）を見ているだけで、bt_navigatorの内部状態がactiveになった
    # 瞬間とは一致しない。実測でその隙間（〜250ms）を突いて
    # 「Action server is inactive. Rejecting the goal.」を一度食らった。
    # なので拒否は致命的エラーにせず、少し待って送り直す。
    MAX_RETRIES = 10
    RETRY_DELAY_S = 1.0

    result = {"status": None, "sent": False, "next_send_at": None, "retries": 0}

    def on_goal_response(future) -> None:
        handle = future.result()
        if not handle.accepted:
            if result["retries"] >= MAX_RETRIES:
                print("[nav2-sim] ⚠️ ゴールが繰り返し拒否された(リトライ上限)")
                result["status"] = GoalStatus.STATUS_ABORTED
                return
            result["retries"] += 1
            result["sent"] = False
            result["next_send_at"] = time.monotonic() + RETRY_DELAY_S
            print(f"[nav2-sim] ゴールが拒否された(bt_navigatorがまだactiveでない可能性)。"
                  f"{RETRY_DELAY_S:.0f}s後に再送 (retry {result['retries']}/{MAX_RETRIES})")
            return
        print("[nav2-sim] ゴールが受理された。走行開始")
        result_future = handle.get_result_async()
        result_future.add_done_callback(on_result)

    def on_result(future) -> None:
        result["status"] = future.result().status

    start = time.monotonic()
    print(f"[nav2-sim] 出発姿勢: {walker.pose}")
    print("[nav2-sim] navigate_to_pose アクションサーバ待機中(物理は回したまま)...")
    waited_log = False
    while time.monotonic() - start < timeout_s:
        bridge.stop_if_stale()
        walker.step(CONTROL_TICK_S)
        bridge.publish_step()
        on_step()
        rclpy.spin_once(node, timeout_sec=0.0)

        if not result["sent"] and (result["next_send_at"] is None
                                    or time.monotonic() >= result["next_send_at"]):
            if client.server_is_ready():
                print(f"[nav2-sim] アクションサーバ検出。"
                      f"ゴール送信: ({goal_x:+.2f}, {goal_y:+.2f}, yaw={goal_yaw:+.2f})")
                client.send_goal_async(goal).add_done_callback(on_goal_response)
                result["sent"] = True
            elif not waited_log and time.monotonic() - start > 5.0:
                print("[nav2-sim] ...まだアクションサーバが見えない（他ノードの起動待ち）")
                waited_log = True

        time.sleep(0.03)
        if result["status"] is not None:
            break

    pose = walker.pose
    dist = math.hypot(pose.x - goal_x, pose.y - goal_y)
    status_name = {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        None: "TIMEOUT（timeout_s内に終わらなかった）",
    }.get(result["status"], f"status={result['status']}")

    print(f"[nav2-sim] 終了: {status_name}")
    print(f"[nav2-sim] 最終姿勢: {pose}  ゴールまでの距離: {dist:.3f}m "
          f"（/cmd_vel受信回数: {bridge._cmd_count}）")
    if walker.has_fallen:
        print("[nav2-sim] ⚠️ 転倒した（height未満）")
    return 0 if result["status"] == GoalStatus.STATUS_SUCCEEDED else 1


def run_interactive(node, bridge: CmdVelBridge, walker, timeout_s: float,
                     on_step: Callable[[], None] = lambda: None) -> int:
    """自分ではゴールを送らず、RViz等の外部クライアントがゴールを送るのを待つ。

    `navigate_to_pose`のアクションサーバ自体はbt_navigatorが立てているので、
    ゴールはどのクライアントから送ってもよい（RViz2の「Nav2 Goal」ツールが
    典型例）。ここでは物理ループ(/scan, /odom, /tf, /cmd_vel)を回し続けつつ、
    `/navigate_to_pose/_action/status`を購読して状態変化を端末にも出す
    （RViz側でも見えるが、ヘッドレスや確認用に）。
    """
    from action_msgs.msg import GoalStatus, GoalStatusArray
    import rclpy

    seen_status = {}

    def on_status(msg: GoalStatusArray) -> None:
        status_name = {
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED", GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        for entry in msg.status_list:
            goal_id = bytes(entry.goal_info.goal_id.uuid).hex()[:8]
            if seen_status.get(goal_id) == entry.status:
                continue
            seen_status[goal_id] = entry.status
            print(f"[nav2-sim] ゴール{goal_id}: {status_name.get(entry.status, entry.status)}")

    node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", on_status, 10)

    print("[nav2-sim] インタラクティブモード: RViz2などから navigate_to_pose に"
          "ゴールを送ってください(例: 'Nav2 Goal'ツール)。Ctrl+Cで終了")
    start = time.monotonic()
    last_report = 0.0
    while time.monotonic() - start < timeout_s:
        bridge.stop_if_stale()
        walker.step(CONTROL_TICK_S)
        bridge.publish_step()
        on_step()
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.03)
        now = time.monotonic()
        if now - last_report > 5.0:
            pose = walker.pose
            print(f"[nav2-sim] t={now - start:5.1f}s 現在姿勢: {pose}"
                  f"  (/cmd_vel受信回数: {bridge._cmd_count})")
            last_report = now
    print("[nav2-sim] タイムアウトで終了")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--room", default=DEFAULT_ROOM)
    parser.add_argument("--map", type=Path, default=None, metavar="PCD",
                        help="--roomの代わりに実地図(PCD)を使う。sim/run_sim.pyの--mapと同じ"
                             "room_from_point_cloud()で、通行不可の格子をMuJoCoの箱に起こす")
    parser.add_argument("--map-only", action="store_true", help="地図(.pgm/.yaml)だけ作って終わる")
    parser.add_argument("--goal-x", type=float, default=4.5)
    parser.add_argument("--goal-y", type=float, default=-2.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=None,
                        help="待つ実時間[s]。既定は自動送信150s/インタラクティブ600s")
    parser.add_argument("--interactive", action="store_true",
                        help="自分ではゴールを送らず、RViz2の'Nav2 Goal'ツール等が"
                             "送るのを待つ（--timeoutは待機の上限になる）")
    parser.add_argument("--viewer", action="store_true",
                        help="MuJoCoのビューアを出し、実際に歩く様子を見る"
                             "（sim/run_sim.pyの--viewerと同じ。DISPLAYが要る）")
    args = parser.parse_args()
    if args.timeout is None:
        args.timeout = 600.0 if args.interactive else 150.0
    return args


def main() -> int:
    args = parse_args()
    room = room_from_point_cloud(args.map) if args.map else get_room(args.room)
    out = Path(__file__).resolve().parent / "amcl_test_map"
    grid_to_ros_map(room, out)
    if args.map_only:
        return 0

    from sim.g1_walker import G1Walker, Mid360, build_model
    model = build_model(room)
    walker = G1Walker(model)
    lidar = Mid360(walker.model)

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("nav2_sim_bridge")
    bridge = CmdVelBridge(node, walker, lidar)

    target = None if args.interactive else Pose2D(args.goal_x, args.goal_y, args.goal_yaw)
    with (_open_viewer(walker) if args.viewer else nullcontext(None)) as viewer:
        on_step = _make_frame_sync(viewer, walker, target=target)

        # bt_navigator/planner_server/amclが起動しMap/TFが揃うのを少し待つ
        for _ in range(30):
            walker.step(CONTROL_TICK_S)
            bridge.publish_step()
            on_step()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.03)

        # ⚠️ AMCLは静止したままだと粒子をほぼ更新しない(update_min_d/aの既定値は
        # 「動いたら」更新)。立ったままゴールを送ると、初期姿勢のまま収束前の
        # 誤差(amcl_sim_verify.pyの検証で見た0.15〜0.65m)を引きずり、その誤差で
        # 壁際に立っていると誤認識されてglobal costmapのstart cellが
        # inscribed（プランナが"Failed to create plan"を出す）ことが実際にあった。
        #
        # その場での首振り回頭だけでは、spawn地点(-4.5,-2.0)付近が柱・仕切りの
        # どちらからも遠い特徴の乏しい壁際で、スキャンマッチングが曖昧になり
        # 収束しにくかった（実測）。amcl_sim_verify.pyの検証で実際に収束が
        # 進んだ「前進+旋回」を混ぜた動きを真似て、少し歩かせてから収束させる。
        print("[nav2-sim] AMCL収束待ち: 前進+旋回で少し歩かせる...")
        for vx, vz, dur in [(0.3, 0.0, 3.0), (0.0, 0.4, 2.0), (0.3, 0.0, 3.0), (0.0, -0.4, 2.0)]:
            walker.set_command(vx, 0.0, vz)
            t_end = walker.sim_time + dur
            while walker.sim_time < t_end:
                walker.step(CONTROL_TICK_S)
                bridge.publish_step()
                on_step()
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.03)
        walker.stop()
        for _ in range(10):
            walker.step(CONTROL_TICK_S)
            bridge.publish_step()
            on_step()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)
        if bridge.amcl_estimate is not None:
            ex, ey = bridge.amcl_estimate
            gt = walker.pose
            err = math.hypot(gt.x - ex, gt.y - ey)
            print(f"[nav2-sim] 首振り後のAMCL誤差: {err:.3f}m "
                  f"(真値=({gt.x:+.2f},{gt.y:+.2f}) AMCL=({ex:+.2f},{ey:+.2f}))")

        if args.interactive:
            code = run_interactive(node, bridge, walker, args.timeout, on_step=on_step)
        else:
            code = send_goal_and_run(node, bridge, walker, args.goal_x, args.goal_y,
                                      args.goal_yaw, args.timeout, on_step=on_step)

    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
