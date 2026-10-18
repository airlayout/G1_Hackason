"""Nav2 表示・ロケーション・操作者 presence の HTTP 口（server.py から呼ぶ薄い層）。

ここに Nav2 へ送る口は無い。地図・状態は読み取り専用、書くのはロケーションの JSON だけ。
"""
import json

from g1console.locations import LocationError
from g1console.nav import NavError

JSON = "application/json; charset=utf-8"
NOT_CONFIGURED = "--nav-adapter が未設定です（Nav2 の状態は表示できません）"


def _body(code, obj):
    return code, JSON, json.dumps(obj, ensure_ascii=False).encode("utf-8")


class NavApi:
    def __init__(self, locations, watcher=None, proxy=None, heartbeat=None):
        self._locations, self._watcher, self._proxy, self._heartbeat = locations, watcher, proxy, heartbeat

    def get(self, path):
        """GET。該当しない経路は None。戻りは (code, content_type, bytes)。"""
        if path == "/api/nav/state":
            state = self._watcher.latest() if self._watcher else {"available": False, "reason": NOT_CONFIGURED}
            state["heartbeat"] = self._heartbeat.status() if self._heartbeat else {"enabled": False}
            return _body(200, state)
        if path in ("/api/nav/map.json", "/api/nav/map.png"):
            if not self._proxy:
                return _body(404, {"error": NOT_CONFIGURED})
            try:
                if path.endswith(".png"):
                    return 200, "image/png", self._proxy.map_png()
                return _body(200, self._proxy.map_info())
            except NavError as exc:
                return _body(502, {"error": str(exc)})
        if path == "/api/locations":
            return _body(200, self._locations.list())
        return None

    def post(self, path, body):
        """POST。該当しない経路は None。戻りは (code, content_type, bytes)。"""
        if path == "/api/nav/presence":  # 人が画面を見ている間だけ呼ばれる（生存信号の送出条件）
            if self._heartbeat:
                self._heartbeat.touch()
            return _body(200, {"ok": True, "heartbeat": self._heartbeat is not None})
        if path not in ("/api/locations/save", "/api/locations/delete"):
            return None
        try:
            if path.endswith("/save"):
                return _body(200, self._locations.upsert(body))
            return _body(200, self._locations.delete(body.get("name") if isinstance(body, dict) else None))
        except LocationError as exc:
            return _body(exc.status, {"error": exc.message})
        except OSError as exc:
            print("[locations] 保存に失敗: %r" % (exc,))
            return _body(500, {"error": "地点を保存できませんでした"})
