#!/usr/bin/env python3
"""Nav2 の状態を **ブラウザから IP 直叩き** で見るための小さな HTTP サーバ。

    http://<Mac の IP>:8080/

なぜ要るか: 自前ホスト版 Foxglove Studio の docker イメージは配布終了、
コンテナ `rviz` の noVNC は「コンテナ → PC2 が不通」で使えない（機体が無線のみ）。
一方 **Mac から PC2 の foxglove_bridge (ws://10.42.0.76:8765) へは直接届く**。
そこで Mac 上でこのサーバを動かし、橋から取った状態を PNG に描いて返す。
ブラウザは PNG を 1 秒ごとに取り直すだけなので、混在コンテンツも WebSocket も要らない。

読み取れるようにしてあるもの（見たい 2 点）:
  1. 機体の足元が膨張層に埋まっていないか
     → 右の拡大図に inflation_radius の円と「最寄りの占有セルまでの距離」を線で描き、
       見出しに INSIDE / outside と数値を出す
  2. 地図上のロボット位置が実際の立ち位置と合っているか
     → 事前地図に footprint と /scan を重ねる。合っていれば scan が黒い壁に乗る

⚠️ **止まった絵を「生きている」と読ませないこと。** /tf が途絶えても最後の姿勢は
残るので、2 秒以上古い入力は灰色にし、見出しに STALE と出す。

⚠️ このスクリプトは **読むだけ**。subscribe しか送らないので、
   /cmd_vel を含め機体へは一切書き込まない。

起動:
    Navigation/.venv/bin/python nav_live_view.py --host 10.42.0.76

座標系: 扱う地図は **OccupancyGrid**（origin が左下・row-major で下から上）なので
`imshow(origin="lower")` がそのまま使える。PGM を読む `measure_overlay.read_map` は
上下反転せずに返すので、予備地図に使うときだけ `[::-1]` で向きを合わせる。
"""
from __future__ import annotations

import argparse
import json
import struct
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from foxglove_ws import FoxgloveClient, MSG_DATA, OP_BINARY, OP_TEXT
from measure_overlay import read_map
from nav_live_draw import INFLATION_RADIUS_DEFAULT, error_png, render
from nav_live_msgs import TOPICS, State


# ── 橋に繋いで受け続けるスレッド ────────────────────────────────────────
def bridge_loop(state: State, host: str, port: int, topics: list[str],
                stop: threading.Event) -> None:
    """橋に繋いで受け続ける。橋へ送るのは subscribe だけで、機体へは何も書かない。

    ⚠️ **advertise を受け続けること。** 接続直後に一度だけ集める作りだと、
    (a) 描画スレッドに CPU を取られて settle の時計だけ進むと 0 件のまま黙り、
    (b) Nav2 を起動し直して後からトピックが生えても、二度と購読しない。
    2026-09-15 に両方を踏んだので、advertise は届くたびに処理する。
    """
    while not stop.is_set():
        client = None
        try:
            with state.lock:
                state.conn = f"接続中… ws://{host}:{port}"
            client = FoxgloveClient(host, port, timeout=10.0)
            client.sock.settimeout(5.0)
            subs: dict[str, int] = {}          # topic -> 購読 id
            by_id: dict[int, str] = {}
            chan: dict[int, str] = {}          # channelId -> topic
            seen: set[str] = set()
            next_id = 0
            with state.lock:
                state.conn = f"接続 OK ws://{host}:{port}（advertise 待ち）"
                state.err = ""
            while not stop.is_set():
                try:
                    opcode, payload = client.recv_frame()
                except TimeoutError:
                    continue
                except OSError as exc:                 # socket.timeout も OSError 派生
                    if "timed out" in str(exc):
                        continue
                    raise
                if opcode == OP_TEXT:
                    msg = json.loads(payload)          # 壊れていれば例外 → 繋ぎ直す
                    op = msg.get("op")
                    if op == "advertise":
                        want = []
                        for ch in msg["channels"]:
                            topic = ch["topic"]
                            seen.add(topic)
                            chan[ch["id"]] = topic
                            if topic in topics and topic not in subs:
                                subs[topic] = next_id
                                by_id[next_id] = topic
                                want.append({"id": next_id, "channelId": ch["id"]})
                                next_id += 1
                        if want:
                            client.send_json({"op": "subscribe", "subscriptions": want})
                            print("[view] 購読: %s" % ", ".join(
                                sorted(by_id[w["id"]] for w in want)), flush=True)
                    elif op == "unadvertise":
                        for cid in msg.get("channelIds", []):
                            topic = chan.pop(cid, None)
                            seen.discard(topic)
                            if topic in subs:          # 生え直したら購読し直せるように
                                by_id.pop(subs.pop(topic), None)
                                print(f"[view] 消えた: {topic}", flush=True)
                    else:
                        continue
                    with state.lock:
                        state.advertised = sorted(subs)
                        state.missing = [t for t in topics if t not in subs]
                        state.conn = (f"接続 OK ws://{host}:{port}"
                                      f"（{len(subs)}/{len(topics)} トピック"
                                      f" / 橋には {len(seen)} 件）")
                    continue
                if opcode != OP_BINARY or not payload or payload[0] != MSG_DATA:
                    continue
                topic = by_id.get(struct.unpack_from("<I", payload, 1)[0])
                if topic is None:
                    continue
                try:
                    state.put(topic, payload[13:])     # 1 + 4(subId) + 8(timestamp)
                except Exception:
                    with state.lock:
                        state.err = f"{topic} の復号に失敗: {traceback.format_exc(limit=2)}"
        except Exception as exc:
            with state.lock:
                state.conn = f"切断（2 秒後に再接続）: {exc}"
            print(f"[view] {state.conn}", flush=True)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        stop.wait(2.0)


# ── HTTP ────────────────────────────────────────────────────────────────
class Frame:
    """描画スレッドが作った PNG を HTTP スレッドへ渡す箱。"""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.png = b""
        self.info: dict = {"conn": "起動中"}

    def set(self, png: bytes, info: dict) -> None:
        with self.lock:
            self.png, self.info = png, info

    def get(self) -> tuple[bytes, dict]:
        with self.lock:
            return self.png, dict(self.info)


def render_loop(state: State, frame: Frame, opts, stop: threading.Event) -> None:
    while not stop.is_set():
        t0 = time.time()
        try:
            png, info = render(state.snapshot(), opts["prior"], opts["prior_name"],
                               opts["inflation"], opts["zoom"], opts["full"], opts["half"])
        except Exception:
            tb = traceback.format_exc()
            png, info = error_png(tb), {"conn": state.snapshot()["conn"], "error": tb}
        info["render_ms"] = round((time.time() - t0) * 1000)
        frame.set(png, info)
        stop.wait(max(0.05, opts["interval"] - (time.time() - t0)))


PAGE = """<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>G1 Nav2 live view</title>
<style>
  :root { --bg:#11151c; --fg:#e8ecf2; --dim:#9aa4b2; --ok:#35d07f; --ng:#ff6b6b; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font-family:-apple-system, "Hiragino Sans", "Noto Sans JP", sans-serif; }
  header { display:flex; align-items:baseline; gap:1.2rem; flex-wrap:wrap;
           padding:.6rem 1rem; border-bottom:1px solid #263041; }
  h1 { font-size:1rem; margin:0; letter-spacing:.02em; }
  #conn { font-size:.82rem; color:var(--dim); font-variant-numeric:tabular-nums; }
  #shot { display:block; width:100%; height:auto; background:#f4f5f7; }
  footer { padding:.6rem 1rem 1.4rem; font-size:.78rem; color:var(--dim); line-height:1.75; }
  b.ok { color:var(--ok); } b.ng { color:var(--ng); }
  code { background:#1b2230; padding:.05rem .35rem; border-radius:3px; }
</style>
<header>
  <h1>G1 Nav2 live view</h1>
  <span id="conn">接続を待っています…</span>
</header>
<img id="shot" alt="Nav2 の状態">
<footer>
  左＝全体（事前地図＋global costmap＋<code>/plan</code>）、右＝足元の拡大（local costmap＋footprint＋<code>/scan</code>）。<br>
  <b>膨張層</b>: 青→緑→黄がコスト 1〜98、橙が 99（内接円＝触れたら当たる）、赤が 100（致命）。
  点線の円が <code>inflation_radius</code>、赤い線が最寄りの占有セルまでの距離。<br>
  <b>位置が合っているか</b>: 右図のピンクの <code>/scan</code> が黒い壁に乗っていれば合っている。
  乗っていなければ、そのずれ量がそのまま測位のずれ。<br>
  <b>STALE</b> と出ている入力は止まっている。**絵は残るが中身は古い**ので、そのまま信じないこと。<br>
  1 秒ごとに自動更新。数値は <code>/status.json</code>。読むだけで、機体へは何も送らない。
</footer>
<script>
const img = document.getElementById('shot'), conn = document.getElementById('conn');
let busy = false;
function tick() {
  if (!busy) {
    busy = true;
    const next = new Image();
    next.onload = () => { img.src = next.src; busy = false; };
    next.onerror = () => { busy = false; };
    next.src = '/frame.png?t=' + Date.now();
  }
  fetch('/status.json').then(r => r.json()).then(s => {
    const latched = ['/map', '/tf_static'];
    const stale = Object.entries(s.topics || {})
      .filter(([k, v]) => !latched.includes(k) && (v === null || v > 2.0));
    conn.innerHTML = (s.conn || '') + (s.error ? ' <b class="ng">描画エラー</b>' : '') +
      (stale.length ? ' / <b class="ng">止まっている入力 ' + stale.map(e => e[0]).join(' ') + '</b>'
                    : ' <b class="ok">全て新鮮</b>') + (s.t ? ' / ' + s.t : '');
  }).catch(() => {});
}
tick(); setInterval(tick, 1000);
</script>
"""


def make_handler(frame: Frame):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, body: bytes, ctype: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: dict) -> None:
            self._send(json.dumps(obj, ensure_ascii=False, default=str).encode("utf-8"),
                       "application/json; charset=utf-8")

        def do_GET(self) -> None:                      # noqa: N802
            path = self.path.split("?")[0]
            try:
                if path == "/":
                    self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/frame.png":
                    png, _ = frame.get()
                    self._send(png or error_png("まだ 1 枚も描けていない（起動直後か橋に未接続）"),
                               "image/png")
                elif path == "/status.json":
                    _, info = frame.get()
                    self._json({k: v for k, v in info.items()
                                if k not in ("tf_tree", "frames", "grids")})
                elif path == "/debug.json":            # TF 木・frame_id・地図の諸元
                    _, info = frame.get()
                    self._json(info)
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, fmt, *args):             # 1 秒ごとに叩かれるので黙らせる
            pass

    return Handler


def load_prior(yaml_path: Path) -> dict:
    """PGM の事前地図を OccupancyGrid と同じ向き（下から上）に直して返す。"""
    occ, res, ox, oy = read_map(yaml_path)
    grid = np.where(occ[::-1], 100, 0).astype(np.int8)   # read_map は上下反転せずに返す
    return {"stamp": 0.0, "frame": "map", "res": res, "w": grid.shape[1], "h": grid.shape[0],
            "ox": ox, "oy": oy, "oyaw": 0.0, "grid": grid}


def local_ips() -> list[str]:
    """ブラウザに入れる URL を出すためだけの一覧。取れなければ空でよい。"""
    import re as _re
    import subprocess as _sp
    try:
        txt = _sp.run(["ifconfig"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    out = []
    for ip in _re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", txt):
        if not ip.startswith("127.") and ip not in out:
            out.append(ip)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Nav2 の状態をブラウザから IP 直叩きで見る")
    ap.add_argument("--host", default="10.42.0.76", help="foxglove_bridge のホスト（PC2）")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0", help="既定は全 NIC。IP 直叩きで見るため")
    ap.add_argument("--map", type=Path, default=None,
                    help="/map が来ないときに背景へ使う PGM の yaml")
    ap.add_argument("--inflation", type=float, default=INFLATION_RADIUS_DEFAULT)
    ap.add_argument("--zoom", type=float, default=3.0, help="右の拡大図の半幅 [m]")
    ap.add_argument("--half", type=float, default=10.0, help="左の全体図の半幅 [m]")
    ap.add_argument("--full-map", action="store_true", help="左を地図の全域にする")
    ap.add_argument("--interval", type=float, default=1.0, help="描画の周期 [s]")
    ap.add_argument("--skip", default="", help="購読しないトピック（カンマ区切り）。"
                    "機体の AP は実効 3 MB/s しかないので、測位の実験中に無線を空けたい"
                    "ときは --skip /global_costmap/costmap（実測 0.23 MB/s）で落とす")
    a = ap.parse_args()

    prior, prior_name = None, ""
    if a.map is not None:
        prior = load_prior(a.map)
        prior_name = a.map.name
        print(f"[view] 予備の事前地図 {a.map}（{prior['w']}x{prior['h']} / res {prior['res']}）",
              flush=True)

    skip = [t.strip() for t in a.skip.split(",") if t.strip()]
    topics = [t for t in TOPICS if t not in skip]
    state, frame, stop = State(), Frame(), threading.Event()
    state.skipped = skip
    if skip:
        print(f"[view] 購読しない: {skip}", flush=True)
    opts = {"prior": prior, "prior_name": prior_name, "inflation": a.inflation,
            "zoom": a.zoom, "half": a.half, "full": a.full_map, "interval": a.interval}
    threading.Thread(target=bridge_loop, args=(state, a.host, a.port, topics, stop),
                     daemon=True).start()
    threading.Thread(target=render_loop, args=(state, frame, opts, stop), daemon=True).start()

    httpd = ThreadingHTTPServer((a.bind, a.http_port), make_handler(frame))
    httpd.daemon_threads = True
    print(f"[view] 橋 ws://{a.host}:{a.port} を読み、HTTP で出す", flush=True)
    for ip in local_ips():
        print(f"[view]   http://{ip}:{a.http_port}/", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[view] 終了します")
    finally:
        stop.set()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
