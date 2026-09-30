"""警備システム UI のサーバ。

    ブラウザ ──> この1プロセス ──┬── 映像: ZMQ(G1本体) -> YOLO      … ROS と無関係
                                 └── 航法: NavSource            … モック or ROS アダプタ

⚠️ **このプロセスは ROS に依存しない。** 操作PC の素の venv で動く。ROS に触るのは
`nav/http_source.py` の向こう側（Docker 内のアダプタ）だけ。

⚠️ **依存を増やしていない。** Python 標準の `http.server` で足りる範囲に収めてある
（MJPEG は multipart、状態はポーリング）。FastAPI/uvicorn を足すと操作PC の venv に
別トラックと競合しうる依存が増えるので、必要になるまで入れない。

起動:
    ../../venv/bin/python server.py            # 既定は config.yaml
    ../../venv/bin/python server.py --nav http # ROS アダプタに繋ぐ
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

HERE = Path(__file__).resolve().parent
_PERCEPTION = HERE.parents[2] / "Perception"
if str(_PERCEPTION) not in sys.path:
    sys.path.insert(0, str(_PERCEPTION))

from common.camera.base import FrameSource                 # noqa: E402
from common.camera.video_file import VideoFileSource       # noqa: E402
from common.camera.webcam import WebcamSource              # noqa: E402
from common.camera.zmq_camera import ZmqFrameSource        # noqa: E402
from common.detector.yolo_detector import YoloDetector     # noqa: E402

from nav.base import COMMANDS, NavSource                   # noqa: E402
from vision import DetectionEvent, VisionWorker            # noqa: E402

STATIC = HERE / "static"
_MJPEG_BOUNDARY = "g1frame"


class EventLog:
    """検知イベントを画面用にメモリへ、記録用に JSONL へ残す。"""

    def __init__(self, directory: Path, keep: int = 200) -> None:
        self._dir = directory
        self._dir.mkdir(parents=True, exist_ok=True)
        self._recent: deque[dict[str, object]] = deque(maxlen=keep)
        self._lock = threading.Lock()
        print(f"[ログ] 検知イベントの記録先: {self._dir}")

    def add(self, event: DetectionEvent) -> None:
        row = event.as_dict()
        with self._lock:
            self._recent.append(row)
            path = self._dir / f"events-{time.strftime('%Y%m%d')}.jsonl"
            try:
                with path.open("a", encoding="utf-8") as fp:
                    fp.write(json.dumps(row, ensure_ascii=False) + "\n")
            except OSError as exc:
                # ⚠️ ログが書けないだけで UI を止めない
                print(f"[ログ] 書き込めませんでした: {exc}")

    def recent(self, limit: int = 50) -> list[dict[str, object]]:
        with self._lock:
            return list(self._recent)[-limit:][::-1]


def build_frame_source(cfg: dict) -> FrameSource:
    """設定から映像ソースを作る。UI は中身が何かを知らないまま使う。"""
    kind = str(cfg.get("source", "video"))
    if kind == "video":
        c = cfg.get("video", {})
        path = (HERE / str(c.get("path", ""))).resolve()
        if not path.exists():
            raise SystemExit(f"[設定] 動画が見つかりません: {path}")
        print(f"[映像] 動画ファイル: {path.name}")
        loop = bool(c.get("loop", True))
        if bool(c.get("realtime", True)):
            # ⚠️ 既定で実時間再生にする。素の VideoFileSource は時間の概念が無く、
            # 24fps の動画を 10fps で読むと 2.4 倍遅くなる（検知ログの継続時間も
            # 同じ倍率で水増しされる）。詳細は video_realtime.py
            from video_realtime import RealtimeVideoSource
            return RealtimeVideoSource(str(path), loop=loop)
        print("[映像] ⚠️ 実時間再生を切っている。検知ログの継続時間は実時間と合わない")
        return VideoFileSource(str(path), loop=loop)
    if kind == "zmq":
        c = cfg.get("zmq", {})
        print(f"[映像] ZMQ: {c.get('server_address')}:{c.get('port')} / {c.get('camera_name')}")
        return ZmqFrameSource(
            server_address=str(c["server_address"]),
            port=int(c.get("port", 5555)),
            camera_name=str(c.get("camera_name", "head_camera")),
            timeout_ms=int(c.get("timeout_ms", 5000)),
        )
    if kind == "webcam":
        c = cfg.get("webcam", {})
        print(f"[映像] Webカメラ: device={c.get('device_index', 0)}")
        return WebcamSource(device_index=int(c.get("device_index", 0)))
    raise SystemExit(f"[設定] 知らない映像ソースです: {kind}")


def build_nav_source(cfg: dict) -> NavSource:
    """設定から航法情報の取得元を作る。"""
    kind = str(cfg.get("source", "mock"))
    if kind == "mock":
        from nav.mock import MockNavSource
        c = cfg.get("mock", {})
        return MockNavSource(
            map_yaml=(HERE / str(c["map_yaml"])).resolve(),
            speed_mps=float(c.get("speed_mps", 0.35)),
        )
    if kind == "http":
        from nav.http_source import HttpNavSource
        c = cfg.get("http", {})
        return HttpNavSource(str(c.get("url", "http://127.0.0.1:8081")),
                             timeout_s=float(c.get("timeout_s", 0.5)))
    raise SystemExit(f"[設定] 知らない航法ソースです: {kind}")


class Handler(BaseHTTPRequestHandler):
    server_version = "G1SecurityUI/0.1"

    # 差し込まれる（make_handler 参照）
    vision: VisionWorker
    navsrc: NavSource
    events: EventLog

    def log_message(self, fmt: str, *args) -> None:
        """既定のアクセスログは MJPEG で埋まるので黙らせる。"""

    # ---- 返す道具 -----------------------------------------------------------

    def _send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, obj: object, status: int = 200) -> None:
        self._send_bytes(json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                         "application/json; charset=utf-8", status)

    def _send_static(self, name: str) -> None:
        path = (STATIC / name).resolve()
        if not str(path).startswith(str(STATIC)) or not path.is_file():
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        types = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8", ".png": "image/png"}
        self._send_bytes(path.read_bytes(), types.get(path.suffix, "application/octet-stream"))

    # ---- ルーティング -------------------------------------------------------

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ("/", "/index.html"):
            self._send_static("index.html")
        elif path.startswith("/static/"):
            self._send_static(path[len("/static/"):])
        elif path == "/video.mjpg":
            # ?boxes=0 で検出枠を焼いていない方を流す。既定は枠あり
            q = parse_qs(parsed.query)
            self._stream_video(boxes=q.get("boxes", ["1"])[0] not in ("0", "false"))
        elif path == "/api/state":
            self._send_json(self._state())
        elif path == "/api/events":
            self._send_json({"events": self.events.recent()})
        elif path == "/api/map.json":
            info = self.navsrc.map_info()
            self._send_json(info if info else {"error": "地図がありません"},
                            200 if info else HTTPStatus.NOT_FOUND)
        elif path == "/api/map.png":
            png = self.navsrc.map_png()
            if png:
                self._send_bytes(png, "image/png")
            else:
                self._send_json({"error": "地図がありません"}, HTTPStatus.NOT_FOUND)
        else:
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        prefix = "/api/command/"
        if not path.startswith(prefix):
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        name = path[len(prefix):]
        if name not in COMMANDS:
            # ⚠️ 知らない操作は素通しせず断る。UI の綴り間違いが黙って消えないように
            self._send_json({"ok": False, "message": f"知らない操作です: {name}"},
                            HTTPStatus.BAD_REQUEST)
            return
        result = self.navsrc.send_command(name)
        print(f"[操作] {name} -> {'受理' if result.ok else '拒否'}: {result.message}")
        self._send_json(result.as_dict())

    # ---- 中身 ---------------------------------------------------------------

    def _state(self) -> dict[str, object]:
        state = self.navsrc.get_state().as_dict()
        state["vision"] = self.vision.status
        state["mock"] = self.navsrc.is_mock
        state["ts"] = time.time()
        return state

    def _stream_video(self, boxes: bool = True) -> None:
        """MJPEG（multipart/x-mixed-replace）で最新フレームを流し続ける。

        ブラウザ側は `<img src="/video.mjpg">` だけで表示できる。WebSocket より単純で、
        この用途（片方向・最新だけ欲しい）には十分。

        ⚠️ `boxes` は**この接続だけ**に効く。YOLO は切り替えに関係なく回り続け、
        検知ログも残る（表示を消しても記録は消さない、が警備システムとしての前提）。
        """
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = -1
        try:
            while True:
                jpeg, seq = self.vision.wait_for_frame(seq, boxes=boxes)
                if jpeg is None:
                    time.sleep(0.2)
                    continue
                self.wfile.write(f"--{_MJPEG_BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass   # ブラウザを閉じただけ。正常
        except Exception as exc:
            print(f"[映像] 配信で例外: {exc}")


def make_handler(vision: VisionWorker, navsrc: NavSource, events: EventLog) -> type[Handler]:
    return type("BoundHandler", (Handler,),
                {"vision": vision, "navsrc": navsrc, "events": events})


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 警備システム UI のサーバ")
    parser.add_argument("--config", default=str(HERE / "config.yaml"))
    parser.add_argument("--nav", choices=("mock", "http"), help="設定より優先する航法ソース")
    parser.add_argument("--video", help="設定より優先する動画ファイル")
    parser.add_argument("--port", type=int, help="設定より優先する待ち受けポート")
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    if args.nav:
        cfg["nav"]["source"] = args.nav
    if args.video:
        cfg["video"]["source"] = "video"
        cfg["video"]["video"]["path"] = args.video

    vcfg = cfg["video"]
    dcfg = cfg["detector"]
    detector = YoloDetector(
        model_name=str(dcfg.get("model", "yolo26n.pt")),
        classes=list(dcfg.get("classes", ["person"])),
        confidence_threshold=float(dcfg.get("confidence_threshold", 0.5)),
        device=str(dcfg.get("device", "auto")),
    )
    events = EventLog((HERE / str(cfg["log"]["dir"])).resolve())
    vision = VisionWorker(
        source=build_frame_source(vcfg),
        detector=detector,
        target_fps=float(vcfg.get("target_fps", 10)),
        jpeg_quality=int(vcfg.get("jpeg_quality", 80)),
        present_frames=int(dcfg.get("present_frames", 2)),
        absent_grace_s=float(dcfg.get("absent_grace_s", 1.5)),
        on_event=events.add,
    )
    navsrc = build_nav_source(cfg["nav"])
    vision.start()

    host = str(cfg["server"]["host"])
    port = int(args.port or cfg["server"]["port"])
    try:
        httpd = ThreadingHTTPServer((host, port), make_handler(vision, navsrc, events))
    except OSError as exc:
        vision.stop()
        raise SystemExit(f"[UI] ポート {port} を使えません({exc})。"
                         f"--port で別の番号を指定してください") from exc
    httpd.daemon_threads = True
    print(f"[UI] http://{host}:{port} で待ち受け中。Ctrl-C で止める")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[UI] 停止します")
    finally:
        # ⚠️ ここで例外を握りつぶさないこと（Navigation 側で痛い目を見ている）
        vision.stop()
        navsrc.close()
        httpd.server_close()


if __name__ == "__main__":
    main()
