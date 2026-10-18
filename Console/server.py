#!/usr/bin/env python3
"""G1 開発コンソール（操作用 PC 側。OS は問わない）。標準ライブラリのみ。

  python3 server.py            # 実機: 設定タブで入れた Jetson へ ssh し、そこのヘルパーを使う（--host でも指定可）
  python3 server.py --mock     # 実機なしの模擬（UI 確認用。画面から状態を切り替えられる）
ブラウザで http://127.0.0.1:18790 を開く。

構成: 背景スレッド（Monitor）が約 1 秒ごとに G1 を確認して最新値を保持し、
画面は保持値を読むだけ（タブを増やしても ssh／DDS の負荷は増えない）。
"""
import argparse
import json
import os
import queue
import shlex
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from g1console.auth import Auth, TOKEN_PATH, iface_ipv4, is_loopback, load_or_create_token, parse_bind
from g1console.camera_proc import LocalCameraCtl, MockCameraCtl, SshCameraCtl, resolve_host
from g1console.lidar_proc import LocalLidarCtl, MockLidarCtl, SshLidarCtl, forward_query
from g1console.control import control_summary
from g1console.dds_catalog import SERVICES, TOPICS
from g1console.replay import Replay
from g1console.api_spec import openapi, render_yaml
from g1console.settings import DEFAULT_PATH, LABELS, Settings
from g1console.motion import (HelperSender, MockSender, MotionController, MotionError, UnavailableSender, start_ticker)
from g1console.heartbeat import OperatorHeartbeat, parse_target
from g1console.locations import LocationStore
from g1console.nav import NavProxy, NavWatcher, parse_adapter_url
from g1console.navapi import NavApi
from g1console.topic_backend import HelperTopicBackend
from g1console.topics import MockTopicBackend, TopicService, default_allowlist_path, handle as topics_handle, load_allowlist
from g1console.tasks import TaskError, TaskRunner, load_definitions
from g1console.modes import (ACTIONS, ALLOWED_IDS, BUTTONS, FEATURES, FSM_LABELS, JOINT_NAMES, PLANNED_ACTIONS, PLANNED_MODES, TABS,
                   validate_audio)

HERE = Path(__file__).resolve().parent
HUB_STOP_DELAY_S = 0.3  # /api/hub/stop の応答を返してから止めるまでの待ち
REPLY_TIMEOUT_S = 8.0
READY_TIMEOUT_S = 30.0  # DDS 初期化待ち
POLL_INTERVAL_S = 1.0
POLL_INTERVAL_DOWN_S = 3.0  # Jetson に届かないときは ssh を叩きすぎない
SSH_CONNECT_FAILED = 255  # ssh が接続自体に失敗したときの終了コード
LOOPBACK_BIND = "127.0.0.1"
LOCAL_HOST = "127.0.0.1"  # --local（Jetson 上で動かす）のときの接続先。カメラ・lidar の配信も同じ機械にある
CAMERAS = {"std": "標準カメラ", "d435i": "D435i（RGB）", "depth": "D435i（深度）"}  # camera_stream.py の --camera 名と揃える
CAMERA_TIMEOUT_S = 5.0
CAMERA_CHUNK = 16384
LIDAR_TIMEOUT_S = 5.0
LIDAR_MAX_BYTES = 4 * 1024 * 1024  # 想定は約 80 KB。上流が暴れても読み切らない
LIDAR_HEADERS = ("X-Lidar-Seq", "X-Lidar-Points", "X-Lidar-Age-Ms")


class SshHelper:
    """ssh 越しの常駐ヘルパー。直列化し、異常時は kill して次回再起動する。"""

    backend = "ssh"

    def __init__(self, host=None, key: str = ""):
        self._lock = threading.Lock()
        self._proc = None
        self._lines = queue.Queue()
        self.reconfigure(host, key)

    def reconfigure(self, host, key: str = ""):
        """接続先を差し替える（実行中のヘルパーは捨てて、次の呼び出しで新しい先へつなぐ）。host は検証済みの値。"""
        self._host, self._key = host, key
        self.backend = "ssh " + host if host else "接続先未設定"
        self._kill()

    def _start(self):
        src = (HERE / "jetson" / "remote_helper.py").read_text(encoding="utf-8")
        src = src.replace("__ALLOWED__", repr(sorted(ALLOWED_IDS)))
        cmd, cwd = self._command(src)
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, text=True, cwd=cwd)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self._proc, self._lines), daemon=True).start()
        ready = self._lines.get(timeout=READY_TIMEOUT_S)
        if ready is None or not json.loads(ready).get("ready"):
            raise RuntimeError("helper not ready")

    def _command(self, src: str):
        """ヘルパーを起動するコマンドと cwd。ssh 版は Jetson 側で cd してから python3 に src を渡す。"""
        key = ["-i", self._key] if self._key else []
        cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", *key, "--", self._host,
               "cd ~/unitree_sdk2_python && python3 -u -c " + shlex.quote(src)]
        return cmd, None

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
        if not self._host:
            return {"ok": False, "offline": True, "unconfigured": True,
                    "error": "Jetson の接続先が未設定です（設定タブで入力）"}
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


class LocalHelper(SshHelper):
    """Jetson 上でこのコンソールを動かすとき用。ssh を使わず、同じ remote_helper.py をローカルの子プロセスで動かす。"""

    backend = "local"

    def __init__(self, sdk_dir=None):
        self._sdk_dir = Path(sdk_dir or os.environ.get("UNITREE_SDK_DIR") or "~/unitree_sdk2_python").expanduser()
        super().__init__(LOCAL_HOST)

    def reconfigure(self, host=None, key: str = ""):
        """接続先の概念が無い（常に自分自身）。設定タブの変更では何も差し替えない。"""
        self._host, self._key = LOCAL_HOST, ""
        self._kill()

    def _command(self, src: str):
        return [sys.executable, "-u", "-c", src], str(self._sdk_dir)


class MockHelper:
    """実機なしで UI を確認するための模擬。シナリオは画面から切り替える。"""

    backend = "mock"
    SCENARIOS = {
        "ok": "正常（接続中）",
        "g1_off": "G1 未応答（Jetson には届く）",
        "jetson_off": "Jetson に届かない",
        "debug": "デバッグモード",
        "replay": "実機ログの再生（デバッグモード, 循環）",
    }
    TRANSITION_S = 2.0  # 実機の遷移に時間がかかる様子を模擬

    def __init__(self):
        self.scenario = "ok"
        self.volume = 85
        self._fsm = 1
        self._target = None
        self._due = 0.0
        self._replay = None

    def _replay_telemetry(self):
        if self._replay is None:
            self._replay = Replay()
        return self._replay.telemetry()

    def _current_fsm(self):
        if self._target is not None and time.time() >= self._due:
            self._fsm, self._target = self._target, None
        return self._fsm

    @staticmethod
    def _telemetry():
        joints = [{"q": 0.01 * i, "dq": 0.0, "tau": 0.1, "temperature": [30 + i % 7, 29 + i % 5], "lost": 0, "mode": 1}
                  for i in range(29)]
        return {"battery": {"soc": 67, "current": -1970, "voltage": [48811, 48818, 0],
                            "temperature": [31, 27, 26, 32, 0, 0, 0, 0, 0, 0, 0, 0], "age_s": 0.1},
                "imu": {"rpy": [0.01, 0.02, 0.5], "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.1, 9.8],
                        "temperature": 40, "age_s": 0.1},
                "joints": {"age_s": 0.1, "mode_machine": 5, "items": joints},
                "odom": {"position": [0.0, 0.0, 0.7], "velocity": [0.0, 0.0, 0.0], "yaw_speed": 0.0,
                         "mode": 0, "gait_type": 0, "body_height": 0.7, "error_code": 0, "age_s": 0.1},
                "lowcmd": {"age_s": 0.01, "mode_machine": 5, "items": [
                    {"q": 0.01 * i, "dq": 0.0, "tau": 0.0, "kp": 300.0, "kd": 3.0, "mode": 1} for i in range(29)]},
                "imu2": {"rpy": [0.01, 0.02, 0.5], "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.1, 9.8], "age_s": 0.1},
                "mainboard": {"fan_state": [0] * 6, "state": [32, 0, 0, 0, 0, 0], "temperature": [49, 0, 0, 0, 0, 0],
                              "value": [45.6, 45.3, 1.6, 0.0, 0.0, 0.0], "age_s": 0.05},
                "estop": None, "remote": None,
                "strings": {"rt/rtc/state": {"value": {"connection_state": "not_connected"}, "age_s": 0.5},
                            "rt/arm/action/state": {"value": {"holding": False, "id": 0, "name": ""}, "age_s": 0.1}},
                "ages": {"rt/lowstate": 0.0, "rt/lowcmd": 0.0, "rt/lf/bmsstate": 0.0,
                         "rt/odommodestate": 0.0, "rt/secondary_imu": 0.0, "rt/servicestate": None},
                "system": {"load": [1.2, 1.0, 0.8], "mem_total_mb": 15388, "mem_avail_mb": 12700,
                           "uptime_s": 4000.0, "cpus": 8, "disk_free_gb": 1800.0,
                           "temps": {"CPU-therm": 58.9, "GPU-therm": 54.9, "tj-therm": 58.8}}}

    def _audio(self, req: dict) -> dict:
        if self.scenario == "g1_off":
            return {"ok": True, "code": 3102, "set_code": 3102, "read_code": 3102, "volume": None, "volume_after": None}
        op = req["op"]
        if op == "audio_get":
            return {"ok": True, "code": 0, "volume": self.volume}
        if op == "audio_volume":
            self.volume = req["volume"]
            return {"ok": True, "set_code": 0, "read_code": 0, "volume_after": self.volume}
        return {"ok": True, "code": 0}

    def call(self, req: dict) -> dict:
        if self.scenario == "jetson_off":
            return {"ok": False, "offline": True, "ssh_exit": SSH_CONNECT_FAILED, "error": "模擬"}
        if req["op"].startswith("audio_"):
            return self._audio(req)
        if self.scenario == "g1_off":
            return {"ok": True, "checkmode_code": 3102, "service": None, "fsm_code": 3102, "fsm_id": None,
                    "telemetry": None}
        if self.scenario in ("debug", "replay"):
            tele = self._replay_telemetry() if self.scenario == "replay" else self._telemetry()
            return {"ok": True, "checkmode_code": 0, "service": "", "fsm_code": 3102, "fsm_id": None,
                    "telemetry": tele}
        if req["op"] == "set":
            self._target, self._due = int(req["id"]), time.time() + self.TRANSITION_S
            return {"ok": True, "set_code": 0}
        return {"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 0,
                "fsm_id": self._current_fsm(), "telemetry": self._telemetry()}


def _jetson_detail(raw: dict) -> str:
    if raw.get("unconfigured"):
        return "接続先が未設定です（設定タブで Jetson の IP を入力）"
    if raw.get("ssh_exit") == SSH_CONNECT_FAILED:
        return "ssh で接続できません（exit 255）"
    return "ヘルパーが応答しません（%s）" % raw.get("error", "不明")


def describe(raw: dict) -> dict:
    """ヘルパーの生の応答を、3 段の接続状態・現在のモード・テレメトリに変換する。"""
    telemetry = raw.get("telemetry") if raw.get("ok") else None
    links = _describe_links(raw)
    return {**links, "telemetry": telemetry, "control": control_summary(links["mode"], telemetry)}


def _describe_links(raw: dict) -> dict:
    if not raw.get("ok"):
        return {"jetson": {"state": "down", "detail": _jetson_detail(raw)},
                "g1": {"state": "unknown", "detail": "Jetson に届かないため未確認"},
                "mode": {"state": "none", "label": "—"}}
    jetson = {"state": "ok", "detail": "ヘルパー応答あり"}
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
        self._wake = threading.Event()
        self._paused = False
        self._snap = {
            "sampled_at": None, "latency_ms": None, "last_g1_ok_at": None, "last_mode": None,
            "jetson": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "g1": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "mode": {"state": "none", "label": "—"}, "telemetry": None,
            "control": control_summary({"state": "none"}, None),
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
        return {**snap, "age_s": age, "poll_interval_s": self._interval, "paused": self._paused,
                "server": {"backend": self._helper.backend, "mock": hasattr(self._helper, "scenario")}}

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    @property
    def paused(self) -> bool:
        return self._paused

    def set_paused(self, paused: bool):
        """自動確認（と自動再接続）の停止／再開。停止中は背景では ssh も DDS も叩かない。"""
        self._paused = bool(paused)
        self._wake.set()

    def _loop(self):
        while not self._stop.is_set():
            wait = POLL_INTERVAL_DOWN_S
            if not self._paused:
                snap = self.poll_once()
                wait = self._interval if snap["jetson"]["state"] == "ok" else POLL_INTERVAL_DOWN_S
            self._wake.wait(wait)
            self._wake.clear()


STATE_KEYS = ("battery", "imu", "imu2", "odom", "mainboard", "system", "remote", "estop", "strings", "ages")
STATE_NOTES = {
    "units_assumed": ["battery.voltage: mV", "battery.current: mA（負=放電）", "battery.temperature: ℃"],
    "unverified": ["mainboard.value の意味", "joints の名前対応", "IMU の単位（rpy=rad, gyro=rad/s, accel=m/s²）は SDK 準拠"],
}


def view(snap: dict, name: str) -> dict:
    """monitor.snapshot() から、タブ単位の JSON を作る（画面と同じ内容を AI が 1 回の取得で読める）。"""
    tele = snap.get("telemetry") or {}
    head = {"sampled_at": snap["sampled_at"], "age_s": snap["age_s"], "jetson": snap["jetson"],
            "g1": snap["g1"], "mode": snap["mode"], "control": snap["control"]}
    if name == "state":
        return {**head, **{k: tele.get(k) for k in STATE_KEYS}, "notes": STATE_NOTES}
    if name == "joints":
        items = (tele.get("joints") or {}).get("items") or []
        cmds = (tele.get("lowcmd") or {}).get("items") or []
        rows = [{"index": i, "name": JOINT_NAMES[i] if i < len(JOINT_NAMES) else "#%d" % i, **m,
                 "cmd": cmds[i] if i < len(cmds) else None} for i, m in enumerate(items)]
        return {**head, "mode_machine": (tele.get("joints") or {}).get("mode_machine"),
                "age_s_joints": (tele.get("joints") or {}).get("age_s"), "items": rows,
                "notes": {"unverified": ["関節名の対応"]}}
    return {**head, "tabs": list(TABS), "state": view(snap, "state"), "joints": view(snap, "joints")}


LOGIN_PAGE = """<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>G1 コンソール ログイン</title></head>
<body style="font-family:sans-serif;max-width:420px;margin:12vh auto;padding:0 16px">
<h1 style="font-size:20px">G1 開発コンソール</h1>
<p>トークンを入力してください（Jetson の <code>~/.g1console/token</code>、または起動ログの URL）。</p>
<form method="get" action="/login"><input name="token" type="password" autocomplete="off" autofocus
 style="width:100%;font-size:18px;padding:10px;box-sizing:border-box">
<button type="submit" style="margin-top:12px;font-size:18px;padding:10px 20px">ログイン</button></form>
</body></html>
"""


def fsm_source(monitor):
    """運動の可否判定に渡す (FSM ID, 確認からの秒数)。取れていなければ (None, None)。"""
    def read():
        snap = monitor.snapshot()
        mode = snap["mode"]
        return (mode.get("fsm_id") if mode["state"] == "ok" else None), snap["age_s"]
    return read


def make_handler(helper, monitor, camera_base=None, settings=None, camera_ctl=None, lidar_ctl=None, lidar_base=None,
                 auth=None, motion=None, tasks=None, nav=None, topics=None, hub_stop=None):
    def camera_url():
        return camera_base or (settings.camera_base(resolve_host) if settings else None)

    def lidar_url():
        return lidar_base or (settings.lidar_base(resolve_host) if settings else None)

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

        def _gate(self, post: bool) -> bool:
            """認証が有効なとき、通してよい要求か。通さない場合はここで応答を返して False。"""
            if auth is None:
                return True
            host = self.headers.get("Host", "")
            if not auth.check_host(host):
                self._json(403, {"error": "このホスト名では開けません"})
                return False
            if post and not auth.check_origin(self.headers.get("Origin"), host):
                self._json(403, {"error": "Origin が一致しません"})
                return False
            if self.path.split("?", 1)[0] == "/login":
                self._login()
                return False
            if auth.authorized(self.headers):
                return True
            if self.path in ("/", "/index.html"):
                self._send(401, "text/html; charset=utf-8", LOGIN_PAGE.encode("utf-8"))
            else:
                self._json(401, {"error": "認証が必要です（/login にトークンを入れてください）"})
            return False

        def _login(self):
            """GET /login?token=… → Cookie を発行して / へ。トークン無しはログイン画面。"""
            token = parse_qs(urlsplit(self.path).query).get("token", [""])[0]
            client = self.client_address[0]
            if not auth.required:  # 認証なし運用: ログインは不要。古いブックマークの /login は / へ
                self.send_response(302)
                self.send_header("Location", "/")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            if not token:
                return self._send(200, "text/html; charset=utf-8", LOGIN_PAGE.encode("utf-8"))
            if auth.verify(token, client):
                self.send_response(302)
                self.send_header("Set-Cookie", auth.cookie_header())
                self.send_header("Location", "/")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            code = 429 if auth.throttled(client) else 401
            self._send(code, "text/html; charset=utf-8", LOGIN_PAGE.encode("utf-8"))

        def do_GET(self):
            if self._gate(post=False):
                self._route_get()

        def do_POST(self):
            if self._gate(post=True):
                self._route_post()

        def _route_get(self):
            hit = nav.get(self.path) if nav else None
            if hit:
                return self._send(*hit)
            hit = topics_handle(topics, "GET", self.path) if topics else None
            if hit:
                return self._json(*hit)
            if self.path == "/api":
                self._json(200, openapi())
            elif self.path == "/openapi.yaml":
                self._send(200, "application/yaml; charset=utf-8", render_yaml().encode("utf-8"))
            elif self.path == "/api/status":
                self._json(200, monitor.snapshot())
            elif self.path in ("/api/state", "/api/joints", "/api/snapshot"):
                self._json(200, view(monitor.snapshot(), self.path[len("/api/"):]))
            elif self.path == "/api/audio/volume":
                self._audio_reply(helper.call({"op": "audio_get"}), "code")
            elif self.path == "/api/buttons":
                self._json(200, {"buttons": BUTTONS})
            elif self.path == "/api/dds":
                self._json(200, {"topics": [dict(zip(("topic", "type", "status", "note"), t)) for t in TOPICS],
                                 "services": [dict(zip(("name", "status", "note"), v)) for v in SERVICES]})
            elif self.path == "/api/features":
                self._json(200, {"features": FEATURES, "planned_modes": PLANNED_MODES,
                                 "planned_actions": PLANNED_ACTIONS, "actions": ACTIONS})
            elif self.path == "/api/scenarios" and hasattr(helper, "scenario"):
                self._json(200, {"scenarios": helper.SCENARIOS, "current": helper.scenario})
            elif self.path == "/api/tasks" and tasks:
                self._json(200, tasks.list())
            elif self.path.split("?", 1)[0] == "/api/tasks/logs" and tasks:
                self._get_task_logs()
            elif self.path == "/api/motion/status" and motion:
                self._json(200, motion.status())
            elif self.path == "/api/cameras":
                self._json(200, {"enabled": bool(camera_url()), "cameras": CAMERAS})
            elif self.path == "/api/camera/status" and camera_ctl:
                self._camera_ctl("status")
            elif self.path == "/api/lidar/status" and lidar_ctl:
                self._lidar_ctl("status")
            elif self.path.split("?", 1)[0] == "/lidar" and lidar_ctl:
                self._proxy_lidar()
            elif self.path == "/api/settings" and settings:
                self._json(200, {"settings": settings.get(), "labels": LABELS})
            elif self.path.startswith("/camera/"):
                self._proxy_camera(self.path[len("/camera/"):])
            elif self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", (HERE / "web" / "index.html").read_bytes())
            else:
                self._json(404, {"error": "not found"})

        def _proxy_camera(self, name: str):
            """Jetson の MJPEG をそのまま中継する（ブラウザは同一オリジンで <img> に入れられる）。"""
            base = camera_url()
            if not base or name not in CAMERAS:
                return self._json(404, {"error": "カメラ未設定（設定タブで Jetson の IP を入力）"})
            try:
                upstream = urllib.request.urlopen("%s/%s" % (base, name), timeout=CAMERA_TIMEOUT_S)
            except OSError as exc:
                return self._json(502, {"error": "カメラに届きません: %s" % exc})
            with upstream:
                self.send_response(200)
                self.send_header("Content-Type", upstream.headers.get("Content-Type", "image/jpeg"))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while chunk := upstream.read(CAMERA_CHUNK):
                        self.wfile.write(chunk)
                except (OSError, ValueError):
                    pass  # どちらかが切れただけ

        def _proxy_lidar(self):
            """Jetson の最新点群 1 フレームを中継する（ブラウザは同一オリジンで fetch できる）。"""
            if hasattr(lidar_ctl, "frame"):  # 模擬
                frame = lidar_ctl.frame()
                if frame is None:
                    return self._json(502, {"error": "lidar_stream が起動していません"})
                body, points = frame
                return self._lidar_reply(body, {"X-Lidar-Seq": int(time.time() * 10), "X-Lidar-Points": points, "X-Lidar-Age-Ms": 0})
            base = lidar_url()
            if not base:
                return self._json(404, {"error": "lidar 未設定（設定タブで Jetson の IP を入力）"})
            try:
                with urllib.request.urlopen("%s/lidar%s" % (base, forward_query(self.path)), timeout=LIDAR_TIMEOUT_S) as up:
                    body = up.read(LIDAR_MAX_BYTES + 1)
                    heads = {k: up.headers.get(k) for k in LIDAR_HEADERS if up.headers.get(k)}
            except OSError as exc:
                return self._json(502, {"error": "lidar に届きません（起動しているか確認）: %s" % exc})
            if len(body) > LIDAR_MAX_BYTES:
                return self._json(502, {"error": "lidar の応答が大きすぎます"})
            self._lidar_reply(body, heads)

        def _lidar_reply(self, body: bytes, heads: dict):
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in heads.items():
                self.send_header(k, str(v))
            self.end_headers()
            self.wfile.write(body)

        def _lidar_ctl(self, action: str):
            try:
                self._json(200, lidar_ctl.call(action))
            except (RuntimeError, subprocess.TimeoutExpired, OSError) as exc:
                self._json(502, {"ok": False, "error": "lidar 操作に失敗しました: %s" % exc})

        def _route_post(self):
            if self.path == "/api/mode":
                return self._post_mode()
            if self.path.startswith("/api/audio/"):
                return self._post_audio(self.path[len("/api/audio/"):])
            if self.path == "/api/scenario" and hasattr(helper, "scenario"):
                return self._post_scenario()
            if self.path in ("/api/camera/start", "/api/camera/stop") and camera_ctl:
                return self._camera_ctl(self.path.rsplit("/", 1)[1])
            if self.path in ("/api/lidar/start", "/api/lidar/stop") and lidar_ctl:
                return self._lidar_ctl(self.path.rsplit("/", 1)[1])
            if nav and self.path.startswith(("/api/nav/", "/api/locations/")):
                return self._post_nav()
            if topics and self.path.startswith("/api/topics/"):
                return self._post_topics()
            if self.path in ("/api/tasks/start", "/api/tasks/stop") and tasks:
                return self._post_task(self.path.rsplit("/", 1)[1])
            if self.path.startswith("/api/motion/") and motion:
                return self._post_motion(self.path[len("/api/motion/"):])
            if self.path == "/api/monitor":
                return self._post_monitor()
            if self.path == "/api/hub/stop" and hub_stop:
                return self._post_hub_stop()
            if self.path == "/api/settings" and settings:
                return self._post_settings()
            self._json(404, {"error": "not found"})

        def _camera_ctl(self, action: str):
            try:
                self._json(200, camera_ctl.call(action))
            except (RuntimeError, subprocess.TimeoutExpired, OSError) as exc:
                self._json(502, {"ok": False, "error": "カメラ操作に失敗しました: %s" % exc})

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

        def _audio_reply(self, raw: dict, *code_keys: str):
            """受理コードがすべて 0 なら 200、それ以外は 502（画面には生の応答も返す）。"""
            failed = not raw.get("ok") or any(raw.get(k) != 0 for k in code_keys)
            if not failed:
                return self._json(200, raw)
            reason = raw.get("error") or ("G1 が要求を受理しませんでした（%s）" % ", ".join(
                "%s=%s" % (k, raw.get(k)) for k in code_keys) if raw.get("ok") else "ヘルパーが応答しません")
            self._json(502, {**raw, "error": reason})

        def _post_audio(self, kind: str):
            try:
                req = validate_audio(kind, self._read_json())
            except (ValueError, TypeError) as exc:
                return self._json(400, {"error": str(exc)})
            raw = helper.call(req)
            self._audio_reply(raw, *(("set_code", "read_code") if kind == "volume" else ("code",)))

        def _get_task_logs(self):
            query = parse_qs(urlsplit(self.path).query)
            try:
                since = int(query.get("since", ["0"])[0])
                if since < 0:
                    raise ValueError
            except ValueError:
                return self._json(400, {"error": "since は 0 以上の整数"})
            try:
                self._json(200, tasks.logs(query.get("name", [""])[0], since))
            except TaskError as exc:
                self._json(exc.status, {"error": exc.message})

        def _post_nav(self):
            try:
                body = self._read_json()
            except (ValueError, TypeError):
                return self._json(400, {"error": "bad request"})
            hit = nav.post(self.path, body)
            if hit is None:
                return self._json(404, {"error": "not found"})
            self._send(*hit)

        def _post_topics(self):
            try:
                body = self._read_json()
            except (ValueError, TypeError):
                return self._json(400, {"error": "bad request"})
            hit = topics_handle(topics, "POST", self.path, body)
            if hit is None:
                return self._json(404, {"error": "not found"})
            self._json(*hit)

        def _post_task(self, kind: str):
            """タスクの起動・停止。受け取るのはタスク名だけ（argv やパスは定義ファイルにしかない）。"""
            try:
                name = self._read_json().get("name")
            except (ValueError, TypeError, AttributeError):
                return self._json(400, {"error": "bad request"})
            if not isinstance(name, str):
                return self._json(400, {"error": "name（文字列）が必要です"})
            try:
                self._json(200, tasks.start(name) if kind == "start" else tasks.stop(name))
            except TaskError as exc:
                self._json(exc.status, {"error": exc.message})

        def _post_motion(self, kind: str):
            """運動操作。本文が空・不正なら 400（estop だけは本文なしでも止める）。"""
            try:
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length else b""
                body = json.loads(raw) if raw.strip() else {}
                if not isinstance(body, dict):
                    raise ValueError("本文は JSON オブジェクト")
            except (ValueError, TypeError):
                if kind == "estop":
                    body = {}
                else:
                    return self._json(400, {"error": "bad request"})
            try:
                if kind == "estop":
                    return self._json(200, motion.estop())
                if kind == "claim":
                    return self._json(200, motion.claim(body.get("takeover") is True))
                if kind == "release":
                    return self._json(200, motion.release(body.get("session")))
                if kind == "ctl":
                    return self._json(200, motion.ctl(body.get("session"), body.get("seq"),
                                                      body.get("vx"), body.get("vy"), body.get("wz")))
                if kind == "action":
                    return self._json(200, motion.action(body.get("session"), body.get("name")))
            except MotionError as exc:
                return self._json(exc.status, {"error": exc.message})
            except (ValueError, TypeError) as exc:
                return self._json(400, {"error": str(exc)})
            self._json(404, {"error": "not found"})

        def _post_monitor(self):
            """{"paused": bool} で自動確認を止める／再開。{"check": true} は 1 回だけ確認する。"""
            try:
                body = self._read_json()
                if "paused" in body:
                    monitor.set_paused(bool(body["paused"]))
                if body.get("check"):
                    monitor.poll_once()
            except (ValueError, TypeError):
                return self._json(400, {"error": "bad request"})
            self._json(200, {"paused": monitor.paused})

        def _post_hub_stop(self):
            """{"confirm": true} でハブ自体を止める。タスク実行中・操作権の保持中は止めない（子や運動指令を巻き込まない）。"""
            try:
                body = self._read_json()
                confirmed = body.get("confirm") is True
            except (ValueError, TypeError, AttributeError):
                return self._json(400, {"error": "bad request"})
            if not confirmed:
                return self._json(400, {"error": "confirm: true が必要です"})
            busy = []
            if tasks and any(t["state"] != "idle" for t in tasks.list()["tasks"]):
                busy.append("タスクが実行中です。先に停止してください")
            if motion and motion.status()["operator"]:
                busy.append("運動の操作権が取られています。先に解放してください")
            if busy:
                return self._json(409, {"error": "止められません: " + "。".join(busy)})
            self._json(200, {"ok": True, "message": "ハブを停止します。再起動は Jetson から行ってください"})
            threading.Timer(HUB_STOP_DELAY_S, hub_stop).start()  # 応答を返し終えてから止める

        def _post_settings(self):
            try:
                saved = settings.update(self._read_json())
            except (ValueError, TypeError) as exc:
                return self._json(400, {"error": str(exc)})
            except OSError as exc:
                return self._json(500, {"error": "設定を保存できません: %s" % exc})
            if hasattr(helper, "reconfigure"):
                helper.reconfigure(*settings.ssh_target())
                if not monitor.paused:
                    monitor.poll_once()
            self._json(200, {"settings": saved, "labels": LABELS})

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


def resolve_binds(binds, iface=None, iface_fn=iface_ipv4) -> list:
    """待ち受けアドレスの一覧。ループバックは常に先頭。重複は除く。不正なら ValueError。"""
    out = [LOOPBACK_BIND]
    extra = [parse_bind(b) for b in binds] + ([iface_fn(iface)] if iface else [])
    for addr in extra:
        if addr not in out:
            out.append(addr)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--local", action="store_true", help="Jetson 上で動かす（ssh を使わず自機でヘルパー・カメラ・lidar を扱う）")
    parser.add_argument("--host", help="Jetson の ssh 接続先（省略時は設定タブの値。~/.ssh/config の別名も可）")
    parser.add_argument("--port", type=int, default=18790)
    parser.add_argument("--camera-url", help="Jetson の camera_stream.py の URL 例: http://<Jetson の IP>:8081")
    parser.add_argument("--bind", action="append", default=[], metavar="IP",
                        help="待ち受ける IPv4（複数可）。127.0.0.1 以外を指定するとトークン認証が必須になる。0.0.0.0 は不可")
    parser.add_argument("--bind-iface", metavar="NAME", help="このインタフェース（例 wlan0）の IPv4 でも待ち受ける")
    parser.add_argument("--allow-host", action="append", default=[], metavar="NAME",
                        help="Host ヘッダに許す名前（mDNS 名など。待ち受け IP は自動で許す）")
    parser.add_argument("--require-token", action="store_true", help="ループバックだけでもトークン認証を要求する")
    parser.add_argument("--no-auth", action="store_true",
                        help="ループバック以外に開いてもトークン認証を求めない。信頼できる閉じたネットワーク専用"
                             "（Host・Origin の検査は残る）")
    parser.add_argument("--tasks-dir", default=str(HERE / "tasks"), help="タスク定義（*.json）のディレクトリ（既定は Console/tasks）")
    parser.add_argument("--nav-adapter", metavar="URL", help="Nav2 の ROS アダプタ（読み取り専用）の URL。http://127.0.0.1:<port> のみ")
    parser.add_argument("--heartbeat-to", metavar="HOST:PORT", help="Nav2 の操作者 heartbeat の送信先。画面を見ている間だけ送る")
    parser.add_argument("--topics-file", default=default_allowlist_path(HERE), help="トピック送信の許可リスト（JSON）。無ければ何も送れない")
    parser.add_argument("--locations-file", default=str(HERE / "data" / "locations.json"), help="ロケーションの保存先")
    args = parser.parse_args()
    if args.mock and args.local:
        parser.error("--mock と --local は同時に指定できません")
    initial = {"jetson_host": LOCAL_HOST, "jetson_user": ""} if args.local else (
        {"jetson_host": args.host, "jetson_user": ""} if args.host else {})
    settings = Settings(None if args.mock else DEFAULT_PATH, initial)
    lidar_port = lambda: settings.get()["lidar_port"]
    camera_port = lambda: settings.get()["camera_port"]
    if args.mock:
        helper, camera_ctl, lidar_ctl = MockHelper(), MockCameraCtl(), MockLidarCtl()
    elif args.local:
        helper, camera_ctl, lidar_ctl = LocalHelper(), LocalCameraCtl(camera_port), LocalLidarCtl(lidar_port)
    else:
        helper = SshHelper(*settings.ssh_target())
        camera_ctl, lidar_ctl = SshCameraCtl(settings.ssh_target, camera_port), SshLidarCtl(settings.ssh_target, lidar_port)
    monitor = Monitor(helper)
    monitor.start()
    sender = MockSender() if args.mock else (HelperSender() if args.local else UnavailableSender())
    motion = MotionController(sender, fsm_source(monitor))
    start_ticker(motion)
    camera_base = args.camera_url.rstrip("/") if args.camera_url else None
    try:
        binds = resolve_binds(args.bind, args.bind_iface)
    except ValueError as exc:
        parser.error(str(exc))
    auth = None
    if args.no_auth and args.require_token:
        parser.error("--no-auth と --require-token は同時に指定できません")
    if args.no_auth and any(not is_loopback(b) for b in binds):
        auth = Auth("", binds + args.allow_host, required=False)  # Host・Origin の検査だけ残す
        print("[console] 警告: --no-auth。同じネットワークの端末なら誰でも操作できる（トークン認証なし）")
    elif args.require_token or any(not is_loopback(b) for b in binds):
        auth = Auth(load_or_create_token(), binds + args.allow_host)
    try:
        locations = LocationStore(args.locations_file)
        tasks = TaskRunner(load_definitions(args.tasks_dir, HERE), HERE, locations=locations)
        proxy = NavProxy(parse_adapter_url(args.nav_adapter)) if args.nav_adapter else None
        heartbeat = OperatorHeartbeat(*parse_target(args.heartbeat_to)) if args.heartbeat_to else None
        allowlist = load_allowlist(args.topics_file)
    except (ValueError, OSError) as exc:
        parser.error("設定が不正です: %s" % exc)
    topics = TopicService(MockTopicBackend() if args.mock else HelperTopicBackend(allowlist), allowlist) if (
        args.mock or args.local) else None  # ssh 経由のときはトピックを扱わない（ヘルパーを Jetson 上で持てない）
    watcher = NavWatcher(proxy, motion) if proxy else None
    for service in (watcher, heartbeat):
        if service:
            service.start()
    servers = []
    handler = make_handler(helper, monitor, camera_base, settings, camera_ctl, lidar_ctl, auth=auth, motion=motion,
                           tasks=tasks, nav=NavApi(locations, watcher, proxy, heartbeat), topics=topics,
                           hub_stop=lambda: [srv.shutdown() for srv in servers])  # 終了コード 0 = systemd の on-failure では再起動しない
    servers.extend(ThreadingHTTPServer((b, args.port), handler) for b in binds)
    for b in binds:
        print("[console] http://%s:%d (%s)%s" % (b, args.port, helper.backend, "" if auth is None else (" 認証あり" if auth.required else " 認証なし")))
    if auth is not None and auth.required:
        for b in binds:
            if not is_loopback(b):  # スマホはこの URL を一度開けば Cookie が付く
                print("[console] ログイン用 URL: http://%s:%d/login?token=%s（トークン: %s）" % (
                    b, args.port, load_or_create_token(), TOKEN_PATH))
    for extra in servers[1:]:
        threading.Thread(target=extra.serve_forever, daemon=True).start()
    def on_sigterm(signum, frame):  # systemd の停止（SIGTERM）でも下の後始末を通す
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        servers[0].serve_forever()
    finally:  # 終了時は必ずゼロ速度にして、運動ヘルパーとタスクも止める
        motion.shutdown()
        sender.close()
        tasks.shutdown()
        if topics:
            topics.close()
        for service in (watcher, heartbeat):
            if service:
                service.stop()


if __name__ == "__main__":
    main()
