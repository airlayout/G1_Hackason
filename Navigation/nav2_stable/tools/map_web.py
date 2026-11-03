#!/usr/bin/env python3
"""地図上の現在地をブラウザで見るための小さな Web サーバ。**PC2 で動かす。**

    # PC2 で（start_nav.sh と同じ pixi 環境で）
    python3 map_web.py --port 8090
    # 操作PC のブラウザで
    http://192.168.0.82:8090/

ふだんは操作PC から `tools/wifi_ops.sh mapview start` で上げる（PC2 で常駐する）。

## なぜ要るのか（2026-10-15）

無線運用では DDS を無線に通さない（WIRELESS.md の方針）ので、操作PC の RViz は使えない。
位置は `wifi_ops.sh pose` で数値では見られるが、地図のどこにいるかが分からない。
そこで **ROS の購読は PC2 の中で済ませ、無線には HTTP だけを流す**。

## 表示するもの

地図(/map) / 現在地と向き(TF map→base_link) / 軌跡 / 巡回路(/g1/patrol/route) /
計画経路(/plan) / 状態(/g1/bridge_status・/g1/patrol/status) /
自己位置が地図と合っているか(/g1/localizer_status の health。map_localizer.py が動いているとき)

**見るだけ。** 操作（Goal・巡回の開始）は持たせない。誰でも開ける URL に操作を置かないため。

⚠️ 位置は機体が推定した値。自己位置がずれていても正常に見えるので、目視と併用すること。
"""
from __future__ import annotations

import argparse
import array
import base64
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """クォータニオンから yaw [rad] を求める"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MapWebNode(Node):
    """ROS を購読して、HTTP で返す状態を組み立てる"""

    def __init__(self, map_frame: str, base_frame: str, trail_len: int) -> None:
        super().__init__("g1_map_web")
        self._lock = threading.Lock()
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._map_json: bytes | None = None
        self._map_version = 0
        self._pose: dict[str, float] | None = None
        self._pose_wall = 0.0
        self._trail: deque[tuple[float, float]] = deque(maxlen=trail_len)
        self._route: list[dict[str, Any]] = []
        self._plan: list[tuple[float, float]] = []
        self._bridge: dict[str, str] = {}
        self._patrol: dict[str, Any] = {}
        self._localizer: dict[str, str] = {}
        self._localizer_wall = 0.0

        # /map と巡回路は latched(transient_local)。ただし後から張った購読に届かないことがある
        # (HANDOVER 落とし穴 15)ので、巡回路は周期配信もされている前提で普通に受ける
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Path, "/g1/patrol/route", self._on_route, latched)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(DiagnosticArray, "/g1/bridge_status", self._on_bridge, 10)
        self.create_subscription(String, "/g1/patrol/status", self._on_patrol, 10)
        self.create_subscription(DiagnosticArray, "/g1/localizer_status", self._on_localizer, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._update_pose)

    # --- 購読 -----------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        # int8(-1/0..100) を 0..255 の 1 バイトに詰めて base64 で送る（ブラウザ側で色に直す）
        raw = array.array("b", msg.data).tobytes()
        body = {
            "width": info.width,
            "height": info.height,
            "resolution": info.resolution,
            "origin_x": info.origin.position.x,
            "origin_y": info.origin.position.y,
            "origin_yaw": yaw_from_quaternion(info.origin.orientation.x, info.origin.orientation.y,
                                              info.origin.orientation.z, info.origin.orientation.w),
            "data": base64.b64encode(raw).decode("ascii"),
        }
        with self._lock:
            self._map_version += 1
            body["version"] = self._map_version
            self._map_json = json.dumps(body).encode("utf-8")
        print(f"[map_web] 地図を受信した: {info.width}x{info.height} 解像度 {info.resolution:.3f}m")

    def _on_route(self, msg: Path) -> None:
        pts = []
        for i, ps in enumerate(msg.poses):
            p = ps.pose
            pts.append({"name": f"p{i + 1}", "x": p.position.x, "y": p.position.y})
        with self._lock:
            self._route = pts

    def _on_plan(self, msg: Path) -> None:
        # 経路は点が多いので間引く（表示には十分）
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        step = max(1, len(pts) // 300)
        with self._lock:
            self._plan = pts[::step] + pts[-1:]

    def _on_bridge(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        st = msg.status[0]
        d = {"state": st.message}
        for kv in st.values:
            d[kv.key] = kv.value
        with self._lock:
            self._bridge = d

    def _on_localizer(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        d = {kv.key: kv.value for kv in msg.status[0].values}
        with self._lock:
            self._localizer = d
            self._localizer_wall = time.time()

    def _on_patrol(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._patrol = d

    def _update_pose(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except Exception:  # noqa: BLE001 — TF がまだ無い・途切れたときは前の値を残し、古さで知らせる
            return
        tr = t.transform.translation
        q = t.transform.rotation
        pose = {"x": tr.x, "y": tr.y, "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
                "stamp": t.header.stamp.sec + t.header.stamp.nanosec * 1e-9}
        with self._lock:
            # 同じ TF を何度も引いたときに「新しい」と見なさない
            if self._pose is None or pose["stamp"] != self._pose["stamp"]:
                self._pose_wall = time.time()
            self._pose = pose
            last = self._trail[-1] if self._trail else None
            if last is None or math.hypot(pose["x"] - last[0], pose["y"] - last[1]) > 0.05:
                self._trail.append((pose["x"], pose["y"]))

    # --- HTTP から呼ぶ ---------------------------------------------------------
    def map_json(self) -> bytes | None:
        with self._lock:
            return self._map_json

    def state_json(self) -> bytes:
        with self._lock:
            age = (time.time() - self._pose_wall) if self._pose is not None else None
            body = {
                "map_version": self._map_version,
                "pose": self._pose,
                "pose_age_s": age,
                "trail": list(self._trail),
                "route": self._route,
                "plan": self._plan,
                "bridge": self._bridge,
                "patrol": self._patrol,
                "localizer": self._localizer,
                "localizer_age_s": (time.time() - self._localizer_wall) if self._localizer else None,
                "server_time": time.time(),
            }
        return json.dumps(body).encode("utf-8")


PAGE = """<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>G1 現在地</title>
<style>
  :root { --bg:#111; --fg:#eee; --muted:#999; --ok:#3c3; --warn:#fc3; --ng:#f44; }
  body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui, sans-serif; }
  #bar { padding:6px 10px; display:flex; flex-wrap:wrap; gap:6px 16px; align-items:center; }
  #bar b { font-size:16px; }
  .ok { color:var(--ok); } .warn { color:var(--warn); } .ng { color:var(--ng); font-weight:bold; }
  #wrap { position:relative; }
  canvas { display:block; width:100vw; height:calc(100vh - 64px); background:#222; touch-action:none; }
  #hint { color:var(--muted); font-size:12px; padding:0 10px; }
</style></head><body>
<div id="bar">
  <span>状態 <b id="st">-</b></span>
  <span>巡回 <b id="pt">-</b></span>
  <span>位置 <b id="pos">-</b></span>
  <span>自己位置 <b id="loc">-</b></span>
  <span id="age"></span>
  <span id="hb"></span>
</div>
<div id="hint">黄=機体（矢印が前） 水色=軌跡 緑=計画経路 橙=巡回路（★=次の目標）。ドラッグで移動、ホイールで拡大。⚠️ 位置は機体の推定値。目視と併用すること</div>
<canvas id="c"></canvas>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
let map = null, mapImg = null, mapVer = -1, st = null;
let scale = 20, ox = 0, oy = 0, follow = true, dragging = null, fitted = false;

async function loadMap() {
  const r = await fetch('map.json'); if (!r.ok) return;
  map = await r.json(); mapVer = map.version;
  const raw = atob(map.data), w = map.width, h = map.height;
  const off = document.createElement('canvas'); off.width = w; off.height = h;
  const oc = off.getContext('2d'), img = oc.createImageData(w, h);
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    let v = raw.charCodeAt(j * w + i); if (v > 127) v -= 256;   // int8 に戻す
    const c = v < 0 ? 90 : (v >= 65 ? 0 : 255 - Math.round(v * 2.0));
    const k = ((h - 1 - j) * w + i) * 4;                         // 画像は上が +y
    img.data[k] = img.data[k + 1] = img.data[k + 2] = c; img.data[k + 3] = 255;
  }
  oc.putImageData(img, 0, 0); mapImg = off;
  // 初回は地図全体が画面に収まる縮尺にする（以後はホイールで変えた縮尺を保つ）
  if (!fitted) {
    const cw = cv.clientWidth * devicePixelRatio, ch = cv.clientHeight * devicePixelRatio;
    scale = 0.9 * Math.min(cw / (w * map.resolution), ch / (h * map.resolution)); fitted = true;
    if (!st || !st.pose) { ox = map.origin_x + w * map.resolution / 2; oy = map.origin_y + h * map.resolution / 2; }
  }
}
function w2s(x, y) { return [cv.width / 2 + (x - ox) * scale, cv.height / 2 - (y - oy) * scale]; }

function draw() {
  cv.width = cv.clientWidth * devicePixelRatio; cv.height = cv.clientHeight * devicePixelRatio;
  ctx.clearRect(0, 0, cv.width, cv.height);
  if (mapImg && map) {
    const [sx, sy] = w2s(map.origin_x, map.origin_y + map.height * map.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImg, sx, sy, map.width * map.resolution * scale, map.height * map.resolution * scale);
  } else {
    ctx.fillStyle = '#aaa'; ctx.font = '20px sans-serif'; ctx.fillText('地図を待っている…（Nav2 が動いているか）', 20, 40);
  }
  if (!st) return;
  const line = (pts, color, width) => {
    if (pts.length < 2) return; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    pts.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
  };
  line(st.trail, 'rgba(0,200,255,0.8)', 3);
  line(st.plan, 'rgba(0,220,0,0.9)', 2);
  const cur = st.patrol && st.patrol.current;
  st.route.forEach(p => {
    const [x, y] = w2s(p.x, p.y), next = p.name === cur;
    ctx.fillStyle = next ? '#ff8800' : 'rgba(255,136,0,0.6)';
    ctx.beginPath(); ctx.arc(x, y, next ? 9 : 6, 0, 7); ctx.fill();
    // 床（白）の上でも読めるよう黒で縁取る
    const label = (next ? '★' : '') + p.name;
    ctx.font = (next ? 'bold 16px' : '14px') + ' sans-serif'; ctx.lineWidth = 4; ctx.strokeStyle = '#000';
    ctx.strokeText(label, x + 10, y - 8); ctx.fillStyle = '#fff'; ctx.fillText(label, x + 10, y - 8);
  });
  if (st.pose) {
    const [x, y] = w2s(st.pose.x, st.pose.y), a = -st.pose.yaw, L = Math.max(18, 0.6 * scale);
    const stale = st.pose_age_s === null || st.pose_age_s > 2.0;
    ctx.save(); ctx.translate(x, y); ctx.rotate(a);
    ctx.fillStyle = stale ? '#f44' : '#ffe000'; ctx.strokeStyle = '#000'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(L, 0); ctx.lineTo(-L * 0.6, L * 0.55); ctx.lineTo(-L * 0.3, 0); ctx.lineTo(-L * 0.6, -L * 0.55); ctx.closePath();
    ctx.fill(); ctx.stroke(); ctx.restore();
  }
}

async function poll() {
  try {
    const r = await fetch('state.json', {cache: 'no-store'}); st = await r.json();
    if (st.map_version !== mapVer && st.map_version > 0) await loadMap();
    const b = st.bridge || {}, p = st.patrol || {}, s = b.state || '?';
    const e = document.getElementById('st'); e.textContent = s + (b.fault_reason ? ' (' + b.fault_reason + ')' : '');
    e.className = s === 'NAVIGATING' || s === 'READY' ? 'ok' : (s === 'FAULT' || s === 'E_STOP' ? 'ng' : 'warn');
    document.getElementById('pt').textContent = p.state ? `${p.state} 次=${p.current} 周回=${p.loop_count}` : '-';
    if (st.pose) {
      document.getElementById('pos').textContent =
        `x=${st.pose.x.toFixed(2)} y=${st.pose.y.toFixed(2)} 向き=${(st.pose.yaw * 180 / Math.PI).toFixed(0)}°`;
      if (follow) { ox = st.pose.x; oy = st.pose.y; }
    }
    const lc = document.getElementById('loc'), L = st.localizer || {};
    if (!L.health || st.localizer_age_s > 5) { lc.textContent = 'map_localizer なし'; lc.className = 'warn'; }
    else {
      const avg = L.inlier_avg && L.inlier_avg !== '-' ? ` ${(parseFloat(L.inlier_avg) * 100).toFixed(0)}%` : '';
      lc.textContent = {ok: '合っている', lost: '⚠️ 地図と合っていない（relocalize）', unknown: '判定中'}[L.health] + avg;
      lc.className = L.health === 'ok' ? 'ok' : (L.health === 'lost' ? 'ng' : 'warn');
    }
    const ag = document.getElementById('age');
    if (st.pose_age_s === null) { ag.textContent = '位置がまだ無い（TF map→base_link）'; ag.className = 'ng'; }
    else if (st.pose_age_s > 2.0) { ag.textContent = `⚠️ 位置が ${st.pose_age_s.toFixed(0)} 秒古い`; ag.className = 'ng'; }
    else { ag.textContent = ''; }
    const hb = document.getElementById('hb');
    hb.textContent = b.operator_heartbeat ? 'heartbeat ' + b.operator_heartbeat : '';
    hb.className = b.operator_heartbeat === 'alive' ? 'ok' : 'warn';
  } catch (err) {
    const ag = document.getElementById('age'); ag.textContent = '⚠️ PC2 に届かない（無線・map_web を確認）'; ag.className = 'ng';
  }
  draw();
}
cv.addEventListener('wheel', ev => { ev.preventDefault(); scale *= ev.deltaY < 0 ? 1.2 : 1 / 1.2; draw(); }, {passive: false});
cv.addEventListener('pointerdown', ev => { dragging = [ev.clientX, ev.clientY]; follow = false; });
cv.addEventListener('pointermove', ev => {
  if (!dragging) return; const d = devicePixelRatio;
  ox -= (ev.clientX - dragging[0]) * d / scale; oy += (ev.clientY - dragging[1]) * d / scale; dragging = [ev.clientX, ev.clientY]; draw();
});
window.addEventListener('pointerup', () => { dragging = null; });
cv.addEventListener('dblclick', () => { follow = true; });   // ダブルクリックで機体追従に戻す
setInterval(poll, 500); poll();
</script></body></html>
"""


def make_handler(node: MapWebNode) -> type[BaseHTTPRequestHandler]:
    """node を参照する HTTP ハンドラを作る"""

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 — http.server の規約
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            elif path == "/state.json":
                self._send(200, "application/json", node.state_json())
            elif path == "/map.json":
                body = node.map_json()
                if body is None:
                    self._send(404, "application/json", b'{"error":"no map yet"}')
                else:
                    self._send(200, "application/json", body)
            else:
                self._send(404, "text/plain; charset=utf-8", "not found".encode("utf-8"))

        def log_message(self, fmt: str, *args: Any) -> None:  # 0.5 秒ごとのアクセスでログを埋めない
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="地図上の現在地をブラウザで見る（PC2 で動かす）")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--bind", default="0.0.0.0", help="待ち受けるアドレス（既定: 全インターフェース）")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--trail", type=int, default=2000, help="軌跡として残す点の数（5cm 間隔）")
    args = parser.parse_args()

    rclpy.init()
    node = MapWebNode(args.map_frame, args.base_frame, args.trail)
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[map_web] http://{args.bind}:{args.port}/ で待ち受ける（見るだけ。操作は無い）")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    server.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
