"""トピック通信の方針（受信は自由、送信は許可リストだけ）と、それを強制するサービス層。

- 送信できるのは `--topics-file` の許可リストに載った String_ トピックだけ。載っていなければ拒否（既定は空）。
- 拒否リスト（DENY_PATTERNS）は許可リストより常に強い。許可リストに拒否対象を書くと起動が止まる。
- 送信にはトピックごとの最小間隔と最大バイト数がある。
- 受信は、ヘルパーが型を知っているトピックだけ。型が未対応なら購読を断る（読めないものを読めたふりをしない）。
バックエンド（Jetson の DDS ヘルパー）は別ファイル。ここは副作用を持たない判定と、受け渡しだけ。
"""
import fnmatch
import json
import os
import re
import threading
import time
from pathlib import Path

# 送信を常に拒否するトピック（fnmatch、小文字で比較。* は / も含めて一致する）。運動・関節・腕・ハンド・リモコン・非常停止・RPC 全般。
DENY_PATTERNS = (
    "rt/lowcmd", "rt/lf/lowcmd", "rt/arm_sdk", "rt/api/*", "rt/*/cmd",
    "rt/wirelesscontroller", "rt/lf/emergency_stop", "rt/lf/*state", "rt/lowstate", "rt/odommodestate",
)
SUPPORTED_TYPES = ("String_", "Error_", "WirelessController_", "BmsState_", "IMUState_", "SportModeState_")
PUBLISH_TYPES = ("String_",)  # 送れる型は当面これだけ

TOPIC_RE = re.compile(r"^rt/[A-Za-z0-9_\-]+(/[A-Za-z0-9_\-]+){0,5}$")
MAX_SUBSCRIPTIONS = 8
MAX_ALLOWED_TOPICS = 50
DEFAULT_MAX_BYTES = 512
HARD_MAX_BYTES = 4096
DEFAULT_MIN_INTERVAL_S = 1.0
MIN_INTERVAL_FLOOR_S = 0.1


class TopicError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status, self.message = status, message


def validate_topic(name):
    """トピック名を検証して返す。不正なら TopicError(400)。"""
    if not isinstance(name, str) or len(name) > 120 or not TOPIC_RE.match(name):
        raise TopicError(400, "topic は rt/ で始まる英数字・_・-・/ の名前（120 文字まで）")
    return name


def is_denied(topic):
    low = topic.lower()
    return any(fnmatch.fnmatchcase(low, pat) for pat in DENY_PATTERNS)


def load_allowlist(path):
    """許可リスト（JSON）を読む。無いファイルは空。壊れていれば ValueError（起動を止める）。"""
    p = Path(path)
    if not p.exists():
        return {}
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
        rows = doc["publish"]
        if doc.get("version") != 1 or not isinstance(rows, list) or len(rows) > MAX_ALLOWED_TOPICS:
            raise ValueError("version=1 の {publish: [...]}（%d 件まで）が必要" % MAX_ALLOWED_TOPICS)
        out = {}
        for row in rows:
            topic = validate_topic(row.get("topic"))
            if is_denied(topic):
                raise ValueError("%s は常に拒否するトピックです" % topic)
            if row.get("type", "String_") not in PUBLISH_TYPES:
                raise ValueError("%s: 送れる型は %s だけ" % (topic, "/".join(PUBLISH_TYPES)))
            max_bytes = row.get("max_bytes", DEFAULT_MAX_BYTES)
            interval = row.get("min_interval_s", DEFAULT_MIN_INTERVAL_S)
            if (isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or not 1 <= max_bytes <= HARD_MAX_BYTES
                    or isinstance(interval, bool) or not isinstance(interval, (int, float))
                    or interval < MIN_INTERVAL_FLOOR_S):
                raise ValueError("%s: max_bytes は 1〜%d、min_interval_s は %.1f 以上" % (
                    topic, HARD_MAX_BYTES, MIN_INTERVAL_FLOOR_S))
            out[topic] = {"topic": topic, "type": "String_", "max_bytes": max_bytes, "min_interval_s": float(interval),
                          "note": str(row.get("note", ""))[:200]}
        return out
    except (KeyError, TypeError, AttributeError, OSError) as exc:
        raise ValueError("%s を読めません: %r" % (path, exc))
    except TopicError as exc:
        raise ValueError(exc.message)


class MockTopicBackend:
    """--mock 用。購読すると偽の String_ を返し、送信は記録するだけ（どこにも出ない）。"""

    name = "mock"

    def __init__(self):
        self.published = []
        self._subs = {}

    def subscribe(self, topic, type_name):
        self._subs[topic] = {"topic": topic, "type": type_name, "n": 0}

    def unsubscribe(self, topic):
        self._subs.pop(topic, None)

    def poll(self):
        out = []
        for sub in self._subs.values():
            sub["n"] += 1
            out.append({"topic": sub["topic"], "type": sub["type"], "age_s": 0.0, "count": sub["n"],
                        "message": {"data": "mock %d" % sub["n"]}})
        return out

    def publish(self, topic, data):
        self.published.append((topic, data))

    def close(self):
        pass


class TopicService:
    def __init__(self, backend, allowlist=None, clock=time.monotonic):
        self._backend, self._allow, self._clock = backend, dict(allowlist or {}), clock
        self._lock = threading.Lock()
        self._subs = {}
        self._last_pub = {}

    def info(self):
        with self._lock:
            subs = sorted(self._subs)
        return {"enabled": True, "backend": self._backend.name, "deny": list(DENY_PATTERNS),
                "supported_types": list(SUPPORTED_TYPES), "max_subscriptions": MAX_SUBSCRIPTIONS,
                "publish": sorted(self._allow.values(), key=lambda r: r["topic"]), "subscribed": subs}

    def subscribe(self, topic, type_name):
        validate_topic(topic)
        if type_name not in SUPPORTED_TYPES:
            raise TopicError(422, "型 %r は読めません（対応: %s）。名前だけの購読はできません" % (type_name, " / ".join(SUPPORTED_TYPES)))
        with self._lock:
            if topic not in self._subs and len(self._subs) >= MAX_SUBSCRIPTIONS:
                raise TopicError(409, "購読は %d 件までです" % MAX_SUBSCRIPTIONS)
            self._backend.subscribe(topic, type_name)
            self._subs[topic] = type_name
        return {"topic": topic, "type": type_name}

    def unsubscribe(self, topic):
        validate_topic(topic)
        with self._lock:
            if topic not in self._subs:
                raise TopicError(404, "購読していません")
            self._backend.unsubscribe(topic)
            del self._subs[topic]
        return {"unsubscribed": topic}

    def received(self):
        with self._lock:
            if not self._subs:
                return {"items": []}
            return {"items": self._backend.poll()}

    def publish(self, topic, data):
        validate_topic(topic)
        if is_denied(topic):
            raise TopicError(403, "%s は常に送信を拒否するトピックです" % topic)
        rule = self._allow.get(topic)
        if rule is None:
            raise TopicError(403, "%s は送信の許可リストにありません" % topic)
        if not isinstance(data, str) or not data:
            raise TopicError(400, "data は空でない文字列")
        size = len(data.encode("utf-8"))
        if size > rule["max_bytes"]:
            raise TopicError(413, "data は %d バイトまで（%d バイト）" % (rule["max_bytes"], size))
        with self._lock:
            now = self._clock()
            wait = self._last_pub.get(topic, -1e9) + rule["min_interval_s"] - now
            if wait > 0:
                raise TopicError(429, "送信が速すぎます。あと %.1f 秒待ってください" % wait)
            self._backend.publish(topic, data)  # 失敗したときは間隔を消費しない
            self._last_pub[topic] = now
        return {"published": topic, "bytes": size}

    def close(self):
        self._backend.close()


def default_allowlist_path(console_dir):
    return os.path.join(str(console_dir), "data", "topics.json")


def handle(service, method, path, body=None):
    """HTTP の口。該当しない経路は None。戻りは (status, JSON にできる dict)。"""
    try:
        if method == "GET" and path == "/api/topics":
            return 200, service.info()
        if method == "GET" and path == "/api/topics/received":
            return 200, service.received()
        if method != "POST" or not path.startswith("/api/topics/"):
            return None
        kind = path[len("/api/topics/"):]
        if kind not in ("subscribe", "unsubscribe", "publish"):
            return None
        if not isinstance(body, dict):
            raise TopicError(400, "本文は JSON オブジェクト")
        if kind == "subscribe":
            return 200, service.subscribe(body.get("topic"), body.get("type"))
        if kind == "unsubscribe":
            return 200, service.unsubscribe(body.get("topic"))
        return 200, service.publish(body.get("topic"), body.get("data"))
    except TopicError as exc:
        return exc.status, {"error": exc.message}
