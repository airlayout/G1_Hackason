"""Nav2 の状態表示（読み取り専用）と、Nav2 稼働中の手動操作の排他。

ハブは ROS を持たない。同じ機体で動く ROS アダプタ（loopback の HTTP）から
状態・地図を読むだけで、**Nav2 へ目的地や操作を送る口は作らない**（enable_navigation・
Goal・巡回操作・G1_ARM は人が現地で扱う）。アダプタの応答は外部データとして検証する。
"""
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlsplit

LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "::1")
BRIDGE_STATES = ("DISCONNECTED", "STANDBY", "READY", "NAVIGATING", "FAULT", "E_STOP")
FETCH_TIMEOUT_S = 1.5
STATE_MAX_BYTES = 512 * 1024
INFO_MAX_BYTES = 4 * 1024
PNG_MAX_BYTES = 8 * 1024 * 1024
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
MAX_POINTS = 2000
MESSAGE_MAX_CHARS = 300
COORD_LIMIT = 10000.0
MAP_SIDE_MAX = 20000
PATROL_STATE_RE = re.compile(r"^[A-Z_]{1,16}$")

REASON_RUNNING = "Nav2 が稼働中のため、手動操作は無効です（指令元を 1 つにするため）"
REASON_UNKNOWN = "Nav2 の状態を取得できません（アダプタが応答しない）。安全のため手動操作は無効のままです"


class NavError(Exception):
    pass


def parse_adapter_url(url: str) -> str:
    """アダプタの URL を検証して正規化する。loopback の http だけを許す（外へ繋がせない）。"""
    parts = urlsplit(url)
    if (parts.scheme != "http" or parts.hostname not in LOOPBACK_HOSTS or parts.port is None
            or parts.path not in ("", "/") or parts.query or parts.username is not None):
        raise ValueError("アダプタの URL は http://127.0.0.1:<port> の形だけ使えます: %r" % url)
    host = "[%s]" % parts.hostname if ":" in parts.hostname else parts.hostname
    return "http://%s:%d" % (host, parts.port)


def _num(value, limit=COORD_LIMIT) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("数値でない")
    value = float(value)
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError("範囲外")
    return value


def _pose(raw):
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("pose が辞書でない")
    return {"x": _num(raw.get("x")), "y": _num(raw.get("y")), "yaw_deg": _num(raw.get("yaw_deg"), 360.0)}


def _points(raw) -> list:
    if not isinstance(raw, list) or len(raw) > MAX_POINTS:
        raise ValueError("点列が不正")
    out = []
    for p in raw:
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            raise ValueError("点が不正")
        out.append([_num(p[0]), _num(p[1])])
    return out


def _int(value, hi) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= hi:
        raise ValueError("整数が範囲外")
    return value


def sanitize_state(raw) -> dict:
    """アダプタの /api/state を、画面に出してよい形に絞る。不正なら ValueError。"""
    if not isinstance(raw, dict):
        raise ValueError("応答が辞書でない")
    patrol = raw.get("patrol") if isinstance(raw.get("patrol"), dict) else {}
    state = patrol.get("state", "IDLE")
    if not isinstance(state, str) or not PATROL_STATE_RE.match(state):
        raise ValueError("巡回状態が不正")
    bridge = raw.get("bridge_state")
    message = raw.get("message", "")
    if not isinstance(message, str):
        raise ValueError("message が文字列でない")
    return {
        "connected": raw.get("connected") is True,
        "pose": _pose(raw.get("pose")),
        "goal": _pose(raw.get("goal")),
        "plan": _points(raw.get("plan", [])),
        "route": _points(raw.get("route", [])),
        "bridge_state": bridge if bridge in BRIDGE_STATES else "UNKNOWN",
        "patrol": {"state": state, "index": _int(patrol.get("index", 0), 100000),
                   "total": _int(patrol.get("total", 0), 100000)},
        "message": message[:MESSAGE_MAX_CHARS],
    }


def sanitize_map_info(raw) -> dict:
    if not isinstance(raw, dict):
        raise ValueError("応答が辞書でない")
    resolution = _num(raw.get("resolution"), 100.0)
    origin = raw.get("origin")
    if resolution <= 0 or not isinstance(origin, list) or len(origin) != 2:
        raise ValueError("地図情報が不正")
    return {"resolution": resolution, "origin": [_num(origin[0]), _num(origin[1])],
            "width": _int(raw.get("width"), MAP_SIDE_MAX), "height": _int(raw.get("height"), MAP_SIDE_MAX)}


class NavProxy:
    """アダプタから読むだけの窓口。POST は一切しない。"""

    def __init__(self, base: str, opener=None):
        self.base = base
        # 環境変数のプロキシを経由させない（loopback へ直接）
        self._open = opener or urllib.request.build_opener(urllib.request.ProxyHandler({})).open

    def _get(self, path: str, limit: int) -> bytes:
        try:
            with self._open(self.base + path, timeout=FETCH_TIMEOUT_S) as resp:
                data = resp.read(limit + 1)
        except urllib.error.HTTPError as exc:
            raise NavError("アダプタが %d を返した（地図・状態がまだ無い可能性）" % exc.code)
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise NavError("アダプタに繋がらない（%s）" % type(exc).__name__)
        if len(data) > limit:
            raise NavError("アダプタの応答が大きすぎる")
        return data

    def _json(self, path: str, limit: int, sanitize) -> dict:
        try:
            return sanitize(json.loads(self._get(path, limit)))
        except (ValueError, UnicodeDecodeError) as exc:
            raise NavError("アダプタの応答が不正（%s）" % exc)

    def state(self) -> dict:
        return self._json("/api/state", STATE_MAX_BYTES, sanitize_state)

    def map_info(self) -> dict:
        return self._json("/api/map.json", INFO_MAX_BYTES, sanitize_map_info)

    def map_png(self) -> bytes:
        data = self._get("/api/map.png", PNG_MAX_BYTES)
        if not data.startswith(PNG_SIGNATURE):
            raise NavError("アダプタの地図が PNG でない")
        return data


class NavWatcher:
    """アダプタを 1 Hz で読み、最新値を配る。Nav2 稼働を見たら手動操作を無効にする。

    排他は fail-safe: 一度でも Nav2 の稼働を見たら、アダプタが黙っても解除しない。
    解除は「アダプタが応答し、かつ Nav2 が繋がっていない」と確認できたときだけ。
    """

    def __init__(self, proxy, motion, interval_s: float = 1.0, clock=time.monotonic):
        self._proxy, self._motion, self._interval, self._clock = proxy, motion, interval_s, clock
        self._lock = threading.Lock()
        self._latest = {"available": False, "reason": "まだ取得していない"}
        self._at = None
        self._seen_running = False
        self._applied = None
        self._stop = threading.Event()
        self._thread = None

    def poll_once(self) -> None:
        try:
            state = self._proxy.state()
            latest = dict(state, available=True)
            reason = REASON_RUNNING if state["connected"] else None
            self._seen_running = state["connected"]
        except NavError as exc:
            latest = {"available": False, "reason": str(exc)}
            reason = REASON_UNKNOWN if self._seen_running else None
        with self._lock:
            self._latest, self._at = latest, self._clock()
        if reason != self._applied:
            self._applied = reason
            self._motion.set_disable_reason(reason)

    def latest(self) -> dict:
        with self._lock:
            out = dict(self._latest)
            age = None if self._at is None else round(self._clock() - self._at, 1)
        out["age_s"] = age
        out["exclusive_reason"] = self._applied
        return out

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="nav-watcher", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.poll_once()
            except Exception as exc:  # 監視の失敗でスレッドを死なせない（排他が外れたままになるのを防ぐ）
                print("[nav] 監視で例外: %r" % (exc,))
            self._stop.wait(self._interval)
