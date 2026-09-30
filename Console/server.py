#!/usr/bin/env python3
"""G1 開発コンソール（Mac 側）。標準ライブラリのみ。

  python3 server.py            # 実機: ssh g1 越しに Jetson 上のヘルパーを使う
  python3 server.py --mock     # 実機なしの模擬（UI 確認用。画面から状態を切り替えられる）
ブラウザで http://127.0.0.1:18790 を開く。

構成: 背景スレッド（Monitor）が約 1 秒ごとに G1 を確認して最新値を保持し、
画面は保持値を読むだけ（タブを増やしても ssh／DDS の負荷は増えない）。
"""
import argparse
import json
import queue
import shlex
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from modes import ALLOWED_IDS, BUTTONS, FEATURES, FSM_LABELS, PLANNED_ACTIONS, PLANNED_MODES

HERE = Path(__file__).resolve().parent
REPLY_TIMEOUT_S = 8.0
READY_TIMEOUT_S = 30.0  # DDS 初期化待ち
POLL_INTERVAL_S = 1.0
POLL_INTERVAL_DOWN_S = 3.0  # Jetson に届かないときは ssh を叩きすぎない
SSH_CONNECT_FAILED = 255  # ssh が接続自体に失敗したときの終了コード


class SshHelper:
    """ssh 越しの常駐ヘルパー。直列化し、異常時は kill して次回再起動する。"""

    backend = "ssh"

    def __init__(self, host: str):
        self._host = host
        self.backend = "ssh " + host
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

    def _exit_code(self):
        if self._proc is None:
            return None
        try:
            return self._proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            return None

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
                exit_code = self._exit_code()
                self._kill()
                return {"ok": False, "offline": True, "ssh_exit": exit_code,
                        "error": "%s: %s" % (type(exc).__name__, exc)}


class MockHelper:
    """実機なしで UI を確認するための模擬。シナリオは画面から切り替える。"""

    backend = "mock"
    SCENARIOS = {
        "ok": "正常（接続中）",
        "g1_off": "G1 未応答（Jetson には届く）",
        "jetson_off": "Jetson に届かない",
        "debug": "デバッグモード",
    }
    TRANSITION_S = 2.0  # 実機の遷移に時間がかかる様子を模擬

    def __init__(self):
        self.scenario = "ok"
        self._fsm = 1
        self._target = None
        self._due = 0.0

    def _current_fsm(self):
        if self._target is not None and time.time() >= self._due:
            self._fsm, self._target = self._target, None
        return self._fsm

    def call(self, req: dict) -> dict:
        if self.scenario == "jetson_off":
            return {"ok": False, "offline": True, "ssh_exit": SSH_CONNECT_FAILED, "error": "模擬"}
        if self.scenario == "g1_off":
            return {"ok": True, "checkmode_code": 3102, "service": None, "fsm_code": 3102, "fsm_id": None}
        if self.scenario == "debug":
            return {"ok": True, "checkmode_code": 0, "service": "", "fsm_code": 3102, "fsm_id": None}
        if req["op"] == "set":
            self._target, self._due = int(req["id"]), time.time() + self.TRANSITION_S
            return {"ok": True, "set_code": 0}
        return {"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 0,
                "fsm_id": self._current_fsm()}


def _jetson_detail(raw: dict) -> str:
    if raw.get("ssh_exit") == SSH_CONNECT_FAILED:
        return "ssh で接続できません（exit 255）"
    return "ヘルパーが応答しません（%s）" % raw.get("error", "不明")


def describe(raw: dict) -> dict:
    """ヘルパーの生の応答を、3 段の接続状態（jetson / g1）と現在のモードに変換する。"""
    if not raw.get("ok"):
        return {"jetson": {"state": "down", "detail": _jetson_detail(raw)},
                "g1": {"state": "unknown", "detail": "Jetson に届かないため未確認"},
                "mode": {"state": "none", "label": "—"}}
    jetson = {"state": "ok", "detail": "ssh 接続中"}
    checkmode, fsm_code = raw.get("checkmode_code"), raw.get("fsm_code")
    if checkmode != 0 and fsm_code != 0:
        return {"jetson": jetson,
                "g1": {"state": "down",
                       "detail": "モーションサービスが応答しません（CheckMode=%s, GetFsmId=%s）" % (checkmode, fsm_code)},
                "mode": {"state": "none", "label": "—"}}
    g1 = {"state": "ok", "detail": "モーションサービス応答あり"}
    if raw.get("service") == "":
        return {"jetson": jetson, "g1": g1,
                "mode": {"state": "debug", "label": "デバッグモード", "service": ""}}
    fsm_id = raw.get("fsm_id")
    if fsm_id is None:
        return {"jetson": jetson, "g1": g1,
                "mode": {"state": "unknown", "label": "状態を取得できません", "detail": "fsm_code=%s" % fsm_code}}
    return {"jetson": jetson, "g1": g1,
            "mode": {"state": "ok", "fsm_id": fsm_id, "label": FSM_LABELS.get(fsm_id, "不明"),
                     "service": raw.get("service")}}


class Monitor:
    """G1 を定期的に確認し、最新の観測を保持する。観測は不変の dict を丸ごと差し替える。"""

    def __init__(self, helper, interval: float = POLL_INTERVAL_S):
        self._helper = helper
        self._interval = interval
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snap = {
            "sampled_at": None, "latency_ms": None, "last_g1_ok_at": None, "last_mode": None,
            "jetson": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "g1": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "mode": {"state": "none", "label": "—"},
        }

    def poll_once(self) -> dict:
        started = time.time()
        observed = describe(self._helper.call({"op": "status"}))
        finished = time.time()
        with self._lock:
            prev = self._snap
            if prev["sampled_at"] is not None and prev["sampled_at"] > finished:
                return prev
            g1_ok = observed["g1"]["state"] == "ok"
            has_mode = observed["mode"]["state"] in ("ok", "debug")
            self._snap = {
                **observed,
                "sampled_at": finished,
                "latency_ms": round((finished - started) * 1000),
                "last_g1_ok_at": finished if g1_ok else prev["last_g1_ok_at"],
                "last_mode": {**observed["mode"], "at": finished} if has_mode else prev["last_mode"],
            }
            return self._snap

    def snapshot(self) -> dict:
        with self._lock:
            snap = self._snap
        age = None if snap["sampled_at"] is None else max(0.0, time.time() - snap["sampled_at"])
        return {**snap, "age_s": age, "poll_interval_s": self._interval,
                "server": {"backend": self._helper.backend, "mock": hasattr(self._helper, "scenario")}}

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stop.is_set():
            snap = self.poll_once()
            self._stop.wait(self._interval if snap["jetson"]["state"] == "ok" else POLL_INTERVAL_DOWN_S)


def make_handler(helper, monitor):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, ctype: str, data: bytes):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _json(self, code: int, body: dict):
            self._send(code, "application/json; charset=utf-8",
                       json.dumps(body, ensure_ascii=False).encode("utf-8"))

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            return json.loads(self.rfile.read(length))

        def do_GET(self):
            if self.path == "/api/status":
                self._json(200, monitor.snapshot())
            elif self.path == "/api/buttons":
                self._json(200, {"buttons": BUTTONS})
            elif self.path == "/api/features":
                self._json(200, {"features": FEATURES, "planned_modes": PLANNED_MODES,
                                 "planned_actions": PLANNED_ACTIONS})
            elif self.path == "/api/scenarios" and hasattr(helper, "scenario"):
                self._json(200, {"scenarios": helper.SCENARIOS, "current": helper.scenario})
            elif self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", (HERE / "index.html").read_bytes())
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if self.path == "/api/mode":
                return self._post_mode()
            if self.path == "/api/scenario" and hasattr(helper, "scenario"):
                return self._post_scenario()
            self._json(404, {"error": "not found"})

        def _post_mode(self):
            try:
                fsm_id = int(self._read_json()["id"])
            except (ValueError, KeyError, TypeError):
                return self._json(400, {"error": "bad request"})
            if fsm_id not in ALLOWED_IDS:
                return self._json(400, {"error": "許可されていない FSM ID"})
            raw = helper.call({"op": "set", "id": fsm_id})
            if raw.get("ok") and raw.get("set_code") != 0:
                return self._json(502, {"error": "G1 が要求を受理しませんでした（code=%s）" % raw.get("set_code")})
            self._json(200 if raw.get("ok") else 502, raw)

        def _post_scenario(self):
            try:
                name = self._read_json()["scenario"]
            except (ValueError, KeyError, TypeError):
                return self._json(400, {"error": "bad request"})
            if name not in helper.SCENARIOS:
                return self._json(400, {"error": "unknown scenario"})
            helper.scenario = name
            monitor.poll_once()
            self._json(200, {"scenario": name})

        def log_message(self, fmt, *args):
            print("[console] " + fmt % args)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--host", default="g1")
    parser.add_argument("--port", type=int, default=18790)
    args = parser.parse_args()
    helper = MockHelper() if args.mock else SshHelper(args.host)
    monitor = Monitor(helper)
    monitor.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(helper, monitor))
    print("[console] http://127.0.0.1:%d (%s)" % (args.port, helper.backend))
    server.serve_forever()


if __name__ == "__main__":
    main()
