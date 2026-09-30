"""アダプタを実機なしで試すための、偽の ROS 側。**コンテナの中で動かす。**

⚠️ **物理も Nav2 も無い。** 本物と同じ**トピック名・型・QoS・JSON の形**で喋るだけ。
ここが通ってもアダプタが実機で通る保証にはならないが、「型が違う」「latched を
合わせていないので何も届かない」といった配線の誤りはここで全部出る。

本物に合わせてあるもの:
  - `/map` と `/g1/patrol/route` は **latched(transient_local)**
  - `/g1/bridge_status` は DiagnosticArray、名前 `g1_cmd_router` の `message` が状態
  - `/g1/patrol/status` は String の中に JSON（`state` / `index` / `waypoints` …）
  - 巡回の状態名は **IDLE / RUNNING / HOLD / DONE / TEACH**（PAUSED ではない）
  - `patrol/start` は bridge が NAVIGATING でないと断る
  - **巡回中の目的地は `/goal_pose` に流さない**（本物はアクションで送るため）
"""
from __future__ import annotations

import json
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)

RESOLUTION = 0.05
ORIGIN = (-5.2, -19.6)
WIDTH, HEIGHT = 40, 60          # 2.0 x 3.0 m の小さな地図
# ⚠️ **巡回路は地図の中に置くこと。** 地図は原点 (-5.2, -19.6) から 2.0 x 3.0 m の
# 範囲しか無いので、(0,0) のような座標を置くと地図の外になる（最初これで UI に
# 何も描かれなくなり、表示範囲の計算の不具合が見つかった）
ROUTE = [(-4.9, -19.3), (-3.5, -19.3), (-3.5, -16.9), (-4.9, -16.9)]


class FakeRos(Node):
    def __init__(self) -> None:
        super().__init__("fake_ros")
        self._bridge = "STANDBY"
        self._patrol = "IDLE"
        self._index = 0
        self._t = 0.0

        self._pub_map = self.create_publisher(OccupancyGrid, "/map", LATCHED)
        self._pub_route = self.create_publisher(PathMsg, "/g1/patrol/route", LATCHED)
        self._pub_plan = self.create_publisher(PathMsg, "/plan", 10)
        self._pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/g1/bridge_status", 10)
        self._pub_status = self.create_publisher(String, "/g1/patrol/status", 10)
        self._tf = TransformBroadcaster(self)

        self.create_service(SetBool, "/g1/enable_navigation", self._srv_enable)
        self.create_service(Trigger, "/g1/stop", self._srv_stop)
        self.create_service(Trigger, "/g1/clear_fault", self._srv_clear)
        self.create_service(Trigger, "/g1/patrol/start", self._srv_patrol_start)
        self.create_service(Trigger, "/g1/patrol/pause", self._srv_patrol_pause)
        self.create_service(Trigger, "/g1/patrol/stop", self._srv_patrol_stop)
        self.create_service(Trigger, "/g1/patrol/skip", self._srv_patrol_skip)

        self._publish_map()
        self._publish_route()
        self.create_timer(0.1, self._tick)          # TF は速めに
        self.create_timer(1.0, self._publish_slow)  # 状態は 1Hz（本物と同じ）
        self.get_logger().info("fake_ros を起動した")

    # ---- 出すもの -----------------------------------------------------------

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_map(self) -> None:
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.header.stamp = self._stamp()
        m.info.resolution = RESOLUTION
        m.info.width, m.info.height = WIDTH, HEIGHT
        m.info.origin.position.x, m.info.origin.position.y = ORIGIN
        m.info.origin.orientation.w = 1.0
        # 外周を壁にして中を自由空間に。上下の区別がつくよう下端に印を入れる
        cells = [0] * (WIDTH * HEIGHT)
        for x in range(WIDTH):
            cells[x] = 100                              # 下端（row 0 = 原点側）
            cells[(HEIGHT - 1) * WIDTH + x] = 100
        for y in range(HEIGHT):
            cells[y * WIDTH] = 100
            cells[y * WIDTH + WIDTH - 1] = 100
        # ⚠️ 下端寄りにだけ大きな塊を置く。**PNG の上下が反転していたら一目で分かる**
        # ようにするため（小さな目印だと枠線に埋もれて差が出ず、判定が弱くなる）
        for y in range(2, 10):
            for x in range(5, 35):
                cells[y * WIDTH + x] = 100
        for i in range(10 * WIDTH, 12 * WIDTH):         # 未知の帯
            cells[i] = -1
        m.data = cells
        self._pub_map.publish(m)

    def _path(self, pts: list[tuple[float, float]]) -> PathMsg:
        msg = PathMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = self._stamp()
        for x, y in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def _publish_route(self) -> None:
        self._pub_route.publish(self._path(ROUTE))

    def _tick(self) -> None:
        """TF `map->base_link` を出す。巡回中だけ動かす。"""
        if self._bridge == "NAVIGATING" and self._patrol == "RUNNING":
            self._t += 0.1
        x = -4.2 + 0.5 * math.cos(self._t * 0.3)
        y = -18.1 + 0.5 * math.sin(self._t * 0.3)
        yaw = self._t * 0.3 + math.pi / 2

        tf = TransformStamped()
        tf.header.stamp = self._stamp()
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self._tf.sendTransform(tf)

    def _publish_slow(self) -> None:
        d = DiagnosticArray()
        d.header.stamp = self._stamp()
        st = DiagnosticStatus()
        st.name = "g1_cmd_router"           # ⚠️ アダプタはこの名前で探す
        st.message = self._bridge
        d.status.append(st)
        self._pub_diag.publish(d)

        s = String()
        s.data = json.dumps({
            "state": self._patrol, "index": self._index, "waypoints": len(ROUTE),
            "current": f"wp{self._index + 1}", "loop_count": 0, "retries": 0,
            "last_result": "", "hold_reason": "", "bridge": self._bridge,
            "teach_points": 0, "error": "",
        }, ensure_ascii=False)
        self._pub_status.publish(s)

        self._publish_route()               # 本物と同じく 1Hz で出し直す
        if self._patrol == "RUNNING":
            gx, gy = ROUTE[self._index]
            # ⚠️ **巡回中に /goal_pose は出さない。** 本物の patrol_node は
            # `navigate_to_pose` アクションで Goal を送るので、このトピックには
            # 流れない。ここで出してしまうと**アダプタの取りこぼしが隠れる**
            # （実際に一度それで見落とし、本物の Nav2 に当てて気づいた）。
            self._pub_plan.publish(self._path([(-4.2, -18.1), (gx, gy)]))

    # ---- サービス -----------------------------------------------------------

    def _srv_enable(self, req, res):
        if req.data:
            self._bridge = "NAVIGATING"
            res.success, res.message = True, "走行を許可した"
        else:
            self._bridge = "READY"
            res.success, res.message = True, "走行許可を取り消した"
        return res

    def _srv_stop(self, req, res):
        del req
        if self._patrol == "RUNNING":
            self._patrol = "HOLD"
        res.success, res.message = True, "停止した"
        return res

    def _srv_clear(self, req, res):
        del req
        res.success, res.message = (self._bridge == "FAULT"), "FAULT ではない"
        return res

    def _srv_patrol_start(self, req, res):
        del req
        # ⚠️ 本物と同じ断り方（patrol_ctl.sh の表）
        if self._bridge != "NAVIGATING":
            res.success = False
            res.message = (f"走行できる状態ではない（bridge={self._bridge}）。"
                           "bridge=READY なら enable_navigation がまだ")
            return res
        self._patrol = "RUNNING"
        res.success, res.message = True, "巡回を始めた"
        return res

    def _srv_patrol_pause(self, req, res):
        del req
        if self._patrol != "RUNNING":
            res.success, res.message = True, f"巡回していない（{self._patrol}）"
            return res
        self._patrol = "HOLD"
        res.success, res.message = True, f"一時停止した（次は {self._index + 1} 点目）"
        return res

    def _srv_patrol_stop(self, req, res):
        del req
        self._patrol, self._index = "IDLE", 0
        res.success, res.message = True, "巡回を止めた。次の start は1点目から"
        return res

    def _srv_patrol_skip(self, req, res):
        del req
        self._index = (self._index + 1) % len(ROUTE)
        res.success, res.message = True, f"飛ばした（次は wp{self._index + 1}）"
        return res


def main() -> None:
    rclpy.init()
    node = FakeRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
