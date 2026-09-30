#!/usr/bin/env python3
"""G1 開発コンソール（Mac 側）。標準ライブラリのみ。

  python3 server.py            # 実機: 設定タブで入れた Jetson へ ssh し、そこのヘルパーを使う（--host でも指定可）
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
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from dds_catalog import SERVICES, TOPICS
from replay import Replay
from api_spec import openapi, render_yaml
from settings import DEFAULT_PATH, LABELS, Settings
from modes import (ALLOWED_IDS, BUTTONS, FEATURES, FSM_LABELS, JOINT_NAMES, PLANNED_ACTIONS, PLANNED_MODES, TABS,
                   validate_audio)

HERE = Path(__file__).resolve().parent
REPLY_TIMEOUT_S = 8.0
READY_TIMEOUT_S = 30.0  # DDS 初期化待ち
POLL_INTERVAL_S = 1.0
POLL_INTERVAL_DOWN_S = 3.0  # Jetson に届かないときは ssh を叩きすぎない
SSH_CONNECT_FAILED = 255  # ssh が接続自体に失敗したときの終了コード
CAMERAS = {"std": "標準カメラ", "d435i": "D435i（RGB）"}  # camera_stream.py の --camera 名と揃える
CAMERA_TIMEOUT_S = 5.0
CAMERA_CHUNK = 16384


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
        key = ["-i", self._key] if self._key else []
        cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", *key, "--", self._host,
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
        joints = [{"q": 0.01 * i, "dq": 0.0, "tau": 0.1, "temperature": [30 + i % 7, 29 + i % 5], "lost": 0}
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
    return {**_describe_links(raw), "telemetry": telemetry}


def _describe_links(raw: dict) -> dict:
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
        self._wake = threading.Event()
        self._paused = False
        self._snap = {
            "sampled_at": None, "latency_ms": None, "last_g1_ok_at": None, "last_mode": None,
            "jetson": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "g1": {"state": "unknown", "detail": "起動直後（初回の確認中）"},
            "mode": {"state": "none", "label": "—"}, "telemetry": None,
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
            "g1": snap["g1"], "mode": snap["mode"]}
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


def make_handler(helper, monitor, camera_base=None, settings=None):
    def camera_url():
        return camera_base or (settings.camera_base() if settings else None)

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
                                 "planned_actions": PLANNED_ACTIONS})
            elif self.path == "/api/scenarios" and hasattr(helper, "scenario"):
                self._json(200, {"scenarios": helper.SCENARIOS, "current": helper.scenario})
            elif self.path == "/api/cameras":
                self._json(200, {"enabled": bool(camera_url()), "cameras": CAMERAS})
            elif self.path == "/api/settings" and settings:
                self._json(200, {"settings": settings.get(), "labels": LABELS})
            elif self.path.startswith("/camera/"):
                self._proxy_camera(self.path[len("/camera/"):])
            elif self.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", (HERE / "index.html").read_bytes())
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

        def do_POST(self):
            if self.path == "/api/mode":
                return self._post_mode()
            if self.path.startswith("/api/audio/"):
                return self._post_audio(self.path[len("/api/audio/"):])
            if self.path == "/api/scenario" and hasattr(helper, "scenario"):
                return self._post_scenario()
            if self.path == "/api/monitor":
                return self._post_monitor()
            if self.path == "/api/settings" and settings:
                return self._post_settings()
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--host", help="Jetson の ssh 接続先（省略時は設定タブの値。~/.ssh/config の別名も可）")
    parser.add_argument("--port", type=int, default=18790)
    parser.add_argument("--camera-url", help="Jetson の camera_stream.py の URL 例: http://<Jetson の IP>:8081")
    args = parser.parse_args()
    initial = {"jetson_host": args.host, "jetson_user": ""} if args.host else {}
    settings = Settings(None if args.mock else DEFAULT_PATH, initial)
    helper = MockHelper() if args.mock else SshHelper(*settings.ssh_target())
    monitor = Monitor(helper)
    monitor.start()
    camera_base = args.camera_url.rstrip("/") if args.camera_url else None
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(helper, monitor, camera_base, settings))
    print("[console] http://127.0.0.1:%d (%s)" % (args.port, helper.backend))
    server.serve_forever()


if __name__ == "__main__":
    main()
