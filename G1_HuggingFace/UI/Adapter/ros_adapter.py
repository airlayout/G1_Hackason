"""ROS 側と UI をつなぐアダプタ。**Docker（ROS 2 Humble）の中で動かす。**

    ブラウザ ──> UI サーバ(venv・ROS 非依存) ──HTTP──> ここ(rclpy) ──> ROS

⚠️ **ROS の語彙が出てくるのはこのファイルだけ。** UI 側は `nav/base.py` の `NavState`
を JSON にしたものしか見ない。契約がずれないよう、**その `nav/base.py` を import して
使う**（コピーしない）。

⚠️ **購読しかしない。`navigate_to_pose` に Goal を送る口は作っていない。**
UI 経由で機体が歩き出す経路を存在させない、という設計上の決定（2026-09-20）。

読むもの:
    TF `map->base_link`      現在地（⚠️ `/odom` は odom フレームなので地図に描けない）
    /map                     地図（OccupancyGrid・latched）
    /plan                    Nav2 が立てた経路
    /g1/patrol/route         巡回路
    /goal_pose               人が RViz から送った目的地
    ⚠️ **巡回中の目的地は `/goal_pose` には流れない。** `patrol_node` は
       `navigate_to_pose` **アクション**で Goal を送るため。走行中の目的地は
       `/plan` の終点から取る（2026-09-20、本物の Nav2 に当てて判明）。
    /g1/bridge_status        走行状態（DiagnosticArray の "g1_cmd_router" の message）
    /g1/patrol/status        巡回状態（String に JSON が入っている）

呼ぶもの:
    /g1/enable_navigation (SetBool)   /g1/stop /g1/clear_fault (Trigger)
    /g1/patrol/{start,pause,stop,skip} (Trigger)
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

# ⚠️ 契約は UI 側と同じものを使う。コピーするとどちらかが古くなる
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Main"))
from nav.base import COMMANDS, CommandResult, NavState, Pose  # noqa: E402

# 操作名 -> ROS サービス
_TRIGGERS: dict[str, str] = {
    "stop": "/g1/stop",
    "clear_fault": "/g1/clear_fault",
    "patrol_start": "/g1/patrol/start",
    "patrol_pause": "/g1/patrol/pause",
    "patrol_stop": "/g1/patrol/stop",
    "patrol_skip": "/g1/patrol/skip",
}
_SETBOOL: dict[str, tuple[str, bool]] = {
    "enable_navigation": ("/g1/enable_navigation", True),
    "disable_navigation": ("/g1/enable_navigation", False),
}

# ⚠️ map と巡回路は latched（transient_local）で出ている。合わせないと**何も届かない**
_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)


def _yaw_deg(q) -> float:
    """四元数から yaw（度）を出す。"""
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class RosAdapter(Node):
    """ROS を読み、UI の契約に詰め替える。"""

    def __init__(self, stale_s: float = 3.0, plan_stale_s: float = 5.0) -> None:
        super().__init__("g1_ui_adapter")
        self._stale = float(stale_s)
        # Nav2 の BT は既定で 1Hz で経路を立て直すので、5 秒あれば十分に余裕がある
        self._plan_stale = float(plan_stale_s)
        self._lock = threading.Lock()
        # ⚠️ サービス呼び出しを購読コールバックと同じ群に入れると詰まるので分ける
        self._group = ReentrantCallbackGroup()

        self._plan: list[tuple[float, float]] = []
        # ⚠️ **経路には賞味期限を付ける。** Nav2 は Goal を畳んでも空の /plan を
        # 出し直さないので、最後の経路が残り続ける。そのまま描くと**止まっているのに
        # まだ向かっているように見える**（2026-09-20、一時停止して12秒後も
        # 94点の経路が出たままだった）。
        self._plan_at = 0.0
        self._route: list[tuple[float, float]] = []
        self._manual_goal: Pose | None = None   # RViz から人が送ったもの
        self._manual_goal_at = 0.0
        self._bridge_state = ""
        self._bridge_at = 0.0
        self._patrol: dict = {}
        self._patrol_at = 0.0
        self._map_png: bytes | None = None
        self._map_info: dict | None = None
        self._last_message = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(OccupancyGrid, "/map", self._on_map, _LATCHED)
        self.create_subscription(PathMsg, "/plan", self._on_plan, 10)
        self.create_subscription(PathMsg, "/g1/patrol/route", self._on_route, _LATCHED)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(DiagnosticArray, "/g1/bridge_status",
                                 self._on_bridge, 10)
        self.create_subscription(String, "/g1/patrol/status", self._on_patrol, 10)

        # ⚠️ `self._clients` にしないこと。**rclpy の Node が内部で使っている名前**で、
        # 上書きすると create_client() が AttributeError で落ちる（実際に踏んだ）
        self._srv_clients = {
            name: self.create_client(Trigger, srv, callback_group=self._group)
            for name, srv in _TRIGGERS.items()
        }
        for name, (srv, _) in _SETBOOL.items():
            self._srv_clients[name] = self.create_client(SetBool, srv,
                                                         callback_group=self._group)

        self.get_logger().info("g1_ui_adapter を起動した（購読のみ。Goal は送らない）")

    # ---- 購読 ---------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        q = msg.info.origin.orientation
        if abs(_yaw_deg(q)) > 0.5:
            # ⚠️ UI 側の座標変換は軸に平行な地図を前提にしている
            self.get_logger().warn(
                f"地図の原点が回転している（yaw={_yaw_deg(q):.1f}°）。UI の描画がずれる")

        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        canvas = np.full((h, w, 3), 0x44, dtype=np.uint8)            # 未知(-1 を含む)
        canvas[(grid >= 0) & (grid < 20)] = (0xE8, 0xE8, 0xE4)       # 自由
        canvas[grid > 65] = (0x1A, 0x1A, 0x1A)                       # 占有
        # ⚠️ OccupancyGrid の 0 行目は**下端**（原点側）。PNG は 0 行目が上端なので反転する
        canvas = np.flipud(canvas)

        ok, buf = cv2.imencode(".png", canvas)
        if not ok:
            self.get_logger().error("地図の PNG 変換に失敗した")
            return
        with self._lock:
            self._map_png = buf.tobytes()
            self._map_info = {
                # ⚠️ `resolution` は float32 なので、そのまま float にすると
                # 0.05 が 0.05000000074505806 になる。計算上は無視できる差だが、
                # yaml から読むモック側(mapimage.py)は 0.05 を出すので、
                # **同じ地図で2経路が違う値を出す**ことになる。6桁で丸めて揃える
                # （現実的な地図の分解能は 0.01〜1.0 m なので桁は足りる）
                "resolution": round(float(msg.info.resolution), 6),
                "origin": [float(msg.info.origin.position.x),
                           float(msg.info.origin.position.y)],
                "width": int(w),
                "height": int(h),
            }
        self.get_logger().info(
            f"地図を受け取った: {w}x{h}px "
            f"({w * msg.info.resolution:.1f} x {h * msg.info.resolution:.1f} m)")

    @staticmethod
    def _points(msg: PathMsg) -> list[tuple[float, float]]:
        return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_plan(self, msg: PathMsg) -> None:
        with self._lock:
            self._plan = self._points(msg)
            self._plan_at = time.monotonic()

    def _on_route(self, msg: PathMsg) -> None:
        with self._lock:
            self._route = self._points(msg)

    def _on_goal(self, msg: PoseStamped) -> None:
        """人が RViz の「2D Goal Pose」から送った目的地。

        ⚠️ **巡回中はここに来ない**（patrol_node はアクションで送る）。
        走行中の目的地は `_current_goal()` が `/plan` の終点から取る。
        """
        with self._lock:
            self._manual_goal = Pose(msg.pose.position.x, msg.pose.position.y,
                                     _yaw_deg(msg.pose.orientation))
            self._manual_goal_at = time.monotonic()

    def _plan_is_fresh(self, now: float) -> bool:
        """経路が「いまのもの」か。**ロックを取った状態で呼ぶこと。**"""
        return self._plan_at > 0.0 and (now - self._plan_at) <= self._plan_stale

    def _current_goal(self, now: float) -> Pose | None:
        """いま向かっている先。**ロックを取った状態で呼ぶこと。**

        `/plan` の終点を使う。Nav2 が実際に向かっている先であり、巡回でも手動 Goal
        でも同じように取れるため（巡回中の Goal は `/goal_pose` には流れない）。

        人が Goal を送ってから Nav2 が経路を立てるまでの短い隙間だけ、`/goal_pose`
        で受けたものを出す。**古い Goal は出さない** — 終わった目的地が残ると
        「まだそこへ向かっている」と読めてしまう。
        """
        if self._plan_is_fresh(now) and len(self._plan) >= 2:
            x, y = self._plan[-1]
            return Pose(x, y, 0.0)
        if (self._manual_goal is not None
                and (now - self._manual_goal_at) <= self._plan_stale):
            return self._manual_goal
        return None

    def _on_bridge(self, msg: DiagnosticArray) -> None:
        for st in msg.status:
            if st.name == "g1_cmd_router":
                with self._lock:
                    self._bridge_state = st.message
                    self._bridge_at = time.monotonic()

    def _on_patrol(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._patrol = d
            self._patrol_at = time.monotonic()

    # ---- 現在地 -------------------------------------------------------------

    def _pose(self) -> tuple[Pose | None, str]:
        """TF から `map->base_link` を引く。

        ⚠️ **古い TF をそのまま出さない。** 止まっている自己位置が生きているように
        見えるのがいちばん危ない。古ければ None を返して UI に「取れていない」と
        描かせる。
        """
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as exc:                       # tf2 の例外は種類が多い
            return None, f"TF map->base_link が引けない: {type(exc).__name__}"
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - stamp
        if age > self._stale:
            return None, f"TF が古い（{age:.1f} 秒前）"
        t = tf.transform.translation
        return Pose(t.x, t.y, _yaw_deg(tf.transform.rotation)), ""

    # ---- UI へ渡す形 --------------------------------------------------------

    def snapshot(self) -> NavState:
        pose, pose_why = self._pose()
        now = time.monotonic()
        with self._lock:
            bridge_fresh = self._bridge_at > 0 and (now - self._bridge_at) <= self._stale
            patrol_fresh = self._patrol_at > 0 and (now - self._patrol_at) <= self._stale
            patrol = dict(self._patrol)
            state = NavState(
                connected=bridge_fresh,
                pose=pose,
                goal=self._current_goal(now),
                plan=list(self._plan) if self._plan_is_fresh(now) else [],
                route=list(self._route),
                bridge_state=self._bridge_state if bridge_fresh else "DISCONNECTED",
                patrol_state=str(patrol.get("state", "IDLE")) if patrol_fresh else "IDLE",
                patrol_index=int(patrol.get("index", 0)),
                patrol_total=int(patrol.get("waypoints", 0)),
                message=self._last_message,
            )
        # 状況の説明は「困っていること」を優先して1行にする
        notes = []
        if not bridge_fresh:
            notes.append("/g1/bridge_status が来ていない（ROS 側が起動していない可能性）")
        if pose_why:
            notes.append(pose_why)
        if patrol_fresh and patrol.get("hold_reason"):
            notes.append(f"巡回が止まっている理由: {patrol['hold_reason']}")
        if notes:
            state.message = " / ".join(notes)
        return state

    # ---- 操作 ---------------------------------------------------------------

    def call(self, name: str, timeout_s: float = 5.0) -> CommandResult:
        client = self._srv_clients.get(name)
        if client is None:
            return CommandResult(False, f"知らない操作です: {name}")
        if not client.wait_for_service(timeout_sec=1.0):
            srv = _TRIGGERS.get(name) or _SETBOOL[name][0]
            return CommandResult(False, f"サービスがいません: {srv}")

        if name in _SETBOOL:
            req = SetBool.Request()
            req.data = _SETBOOL[name][1]
        else:
            req = Trigger.Request()

        future = client.call_async(req)
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return CommandResult(False, f"応答がありません（{timeout_s:.0f} 秒待った）")
            time.sleep(0.02)
        res = future.result()
        if res is None:
            return CommandResult(False, "呼び出しが失敗しました")
        # ⚠️ 断り文をそのまま UI へ通す。言い換えると原因が分からなくなる
        with self._lock:
            self._last_message = res.message
        return CommandResult(bool(res.success), res.message)

    def map_info(self) -> dict | None:
        with self._lock:
            return dict(self._map_info) if self._map_info else None

    def map_png(self) -> bytes | None:
        with self._lock:
            return self._map_png


class Handler(BaseHTTPRequestHandler):
    """UI サーバ（`nav/http_source.py`）が期待する口をそのまま実装する。"""

    server_version = "G1RosAdapter/0.1"
    adapter: RosAdapter

    def log_message(self, fmt: str, *args) -> None:
        """既定のアクセスログは 5Hz のポーリングで埋まるので黙らせる。"""

    def _send(self, body: bytes, ctype: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj: object, status: int = 200) -> None:
        self._send(json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                   "application/json; charset=utf-8", status)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/api/state":
            self._json(self.adapter.snapshot().as_dict())
        elif path == "/api/map.json":
            info = self.adapter.map_info()
            self._json(info if info else {"error": "/map がまだ来ていません"},
                       200 if info else HTTPStatus.NOT_FOUND)
        elif path == "/api/map.png":
            png = self.adapter.map_png()
            if png:
                self._send(png, "image/png")
            else:
                self._json({"error": "/map がまだ来ていません"}, HTTPStatus.NOT_FOUND)
        elif path == "/healthz":
            self._json({"ok": True})
        else:
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        prefix = "/api/command/"
        path = self.path.split("?", 1)[0]
        if not path.startswith(prefix):
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        name = path[len(prefix):]
        if name not in COMMANDS:
            self._json({"ok": False, "message": f"知らない操作です: {name}"},
                       HTTPStatus.BAD_REQUEST)
            return
        result = self.adapter.call(name)
        print(f"[操作] {name} -> {'受理' if result.ok else '拒否'}: {result.message}",
              flush=True)
        self._json(result.as_dict())


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="G1 警備 UI の ROS アダプタ")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--stale-s", type=float, default=3.0,
                        help="これより古い TF・状態は「取れていない」として扱う")
    parser.add_argument("--plan-stale-s", type=float, default=5.0,
                        help="これより古い経路・目的地は「もう向かっていない」として消す")
    args = parser.parse_args()

    rclpy.init()
    adapter = RosAdapter(stale_s=args.stale_s, plan_stale_s=args.plan_stale_s)
    executor = MultiThreadedExecutor()
    executor.add_node(adapter)
    spin = threading.Thread(target=executor.spin, name="rclpy", daemon=True)
    spin.start()

    handler = type("BoundHandler", (Handler,), {"adapter": adapter})
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    httpd.daemon_threads = True
    print(f"[アダプタ] http://{args.host}:{args.port} で待ち受け中。Ctrl-C で止める",
          flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[アダプタ] 停止します", flush=True)
    finally:
        httpd.server_close()
        executor.shutdown()
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
