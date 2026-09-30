#!/usr/bin/env python3
"""G1 開発コンソール（Mac 側）。標準ライブラリのみ。

  python3 server.py            # 実機: ssh g1 越しに Jetson 上のヘルパーを使う
  python3 server.py --mock     # 実機なしの模擬（UI 確認用）
ブラウザで http://127.0.0.1:8765 を開く。
"""
import argparse
import json
import queue
import shlex
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from modes import ALLOWED_IDS, BUTTONS, FSM_LABELS

HERE = Path(__file__).resolve().parent
REPLY_TIMEOUT_S = 8.0
READY_TIMEOUT_S = 30.0  # DDS 初期化待ち


class SshHelper:
    """ssh 越しの常駐ヘルパー。直列化し、異常時は kill して次回再起動する。"""

    def __init__(self, host: str):
        self._host = host
        self._lock = threading.Lock()
        self._proc = None
        self._lines = queue.Queue()

    def _start(self):
        src = (HERE / "remote_helper.py").read_text(encoding="utf-8")
        src = src.replace("__ALLOWED__", repr(sorted(ALLOWED_IDS)))
        cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", self._host,
               "cd ~/unitree_sdk2_python && python3 -u -c " + shlex.quote(src)]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, text=True)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self._proc, self._lines), daemon=True).start()
        ready = self._lines.get(timeout=READY_TIMEOUT_S)
        if ready is None or not json.loads(ready).get("ready"):
            raise RuntimeError("helper not ready")

    @staticmethod
    def _pump(proc, out):
        for line in proc.stdout:
            out.put(line)
        out.put(None)  # EOF

    def _kill(self):
        if self._proc is not None:
            self._proc.kill()
            self._proc = None

    def call(self, req: dict) -> dict:
        with self._lock:
            try:
                if self._proc is None or self._proc.poll() is not None:
                    self._start()
                self._proc.stdin.write(json.dumps(req) + "\n")
                self._proc.stdin.flush()
                line = self._lines.get(timeout=REPLY_TIMEOUT_S)
                if line is None:
                    raise RuntimeError("helper exited")
                return json.loads(line)
            except Exception as exc:
                self._kill()
                return {"ok": False, "offline": True, "error": "%s: %s" % (type(exc).__name__, exc)}


class MockHelper:
    """実機なしで UI を確認するための模擬。"""

    def __init__(self):
        self._fsm = 1

    def call(self, req: dict) -> dict:
        if req["op"] == "set":
            self._fsm = int(req["id"])
            return {"ok": True, "set_code": 0}
        return {"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 0, "fsm_id": self._fsm}


def describe(raw: dict) -> dict:
    """ヘルパーの生の応答を、画面表示用の状態に変換する。"""
    if not raw.get("ok"):
        return {"state": "offline", "label": "接続できません（G1 の電源/ネットワークを確認）",
                "detail": raw.get("error", "")}
    if raw.get("service") == "":
        return {"state": "debug", "label": "デバッグモード（標準モーションサービス解放中）", "detail": ""}
    fsm_id = raw.get("fsm_id")
    if fsm_id is None:
        return {"state": "unknown", "label": "状態を取得できません", "detail": "fsm_code=%s" % raw.get("fsm_code")}
    return {"state": "ok", "fsm_id": fsm_id, "label": FSM_LABELS.get(fsm_id, "不明"),
            "detail": "service=%s" % raw.get("service")}


def make_handler(helper):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, ctype: str, data: bytes):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _json(self, code: int, body: dict):
            self._send(code, "application/json; charset=utf-8",
                       json.dumps(body, ensure_ascii=False).encode("utf-8"))

        def do_GET(self):
            if self.path == "/api/status":
                self._json(200, describe(helper.call({"op": "status"})))
            elif self.path == "/api/buttons":
                self._json(200, {"buttons": BUTTONS})
            elif self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", (HERE / "index.html").read_bytes())
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/api/mode":
                return self._json(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                fsm_id = int(json.loads(self.rfile.read(length))["id"])
            except (ValueError, KeyError, TypeError):
                return self._json(400, {"error": "bad request"})
            if fsm_id not in ALLOWED_IDS:
                return self._json(400, {"error": "許可されていない FSM ID"})
            raw = helper.call({"op": "set", "id": fsm_id})
            self._json(200 if raw.get("ok") else 502, raw)

        def log_message(self, fmt, *args):
            print("[console] " + fmt % args)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--host", default="g1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    helper = MockHelper() if args.mock else SshHelper(args.host)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(helper))
    print("[console] http://127.0.0.1:%d (%s)" % (args.port, "mock" if args.mock else "ssh " + args.host))
    server.serve_forever()


if __name__ == "__main__":
    main()
