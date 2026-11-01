"""外部GPU PC上のローカルUI。静的画面・観測受信・VLA実行だけを提供する。"""

from __future__ import annotations

import argparse
import base64
import os
from pathlib import Path
import signal

import yaml

from .config import DEFAULT_CONFIG, ROOT, load_config
from .http import ApiClient, serve
from .policy import GrootPolicy, ObservationAdapter
from .session import ArmSession


class DemoClient:
    def __init__(self, bridge):
        self.bridge = bridge

    def request(self, path: str, body=None):
        return self.bridge.handle("GET" if body is None else "POST", path, body)[1]


class Application:
    def __init__(self, session: ArmSession, *, demo: bool = False):
        self.session, self.demo = session, demo

    def handle(self, method: str, path: str, body: dict | None):
        if method == "GET" and path == "/api/status":
            return "application/json", {**self.session.status(), "demo": self.demo}
        if method == "GET" and path == "/api/camera.jpg":
            return "image/jpeg", base64.b64decode(self.session.latest()["jpeg"])
        if method == "POST" and path == "/api/task":
            self.session.start(body["instruction"])
            return "application/json", self.session.status()
        if method == "POST" and path == "/api/stop":
            self.session.stop()
            return "application/json", self.session.status()
        static = {
            "/": ("index.html", "text/html"),
            "/app.js": ("app.js", "text/javascript"),
            "/style.css": ("style.css", "text/css"),
        }
        if method == "GET" and path in static:
            filename, content_type = static[path]
            return content_type, (Path(__file__).parent / "static" / filename).read_bytes()
        raise FileNotFoundError(path)


def main() -> None:
    parser = argparse.ArgumentParser(description="G1アームのVLAコンソール")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--robot", default="http://127.0.0.1:8766")
    parser.add_argument("--model-path", help="REAL_G1形式の追加学習済みcheckpointも指定可能")
    parser.add_argument("--device", help="例: cuda:0（16GBのGPUを指定）")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--demo", action="store_true", help="機体・VLAなしで通信とUIを確認する")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.device:
        config["device"] = args.device
    names = yaml.safe_load((ROOT / "configs/robot/g1_29dof.yaml").read_text())["joint_names"]
    adapter = ObservationAdapter(config, names)
    bridge = None
    if args.demo:
        from .demo import DemoBridge, HoldPolicy

        bridge = DemoBridge(config)
        client, policy = DemoClient(bridge), HoldPolicy()
    else:
        client = ApiClient(args.robot, token=os.environ.get("G1_ARM_TOKEN", ""))
        policy = GrootPolicy(config, args.model_path)
    session = ArmSession(client, policy, adapter, config)
    server = serve(Application(session, demo=args.demo), "127.0.0.1", args.port)

    def stop(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"[UI] http://127.0.0.1:{args.port}" + (" / デモ・VLA未使用" if args.demo else ""), flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        server.server_close()
        if bridge:
            bridge.close()


if __name__ == "__main__":
    main()
