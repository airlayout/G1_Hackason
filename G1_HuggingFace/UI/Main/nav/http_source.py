"""Docker 内で動く ROS アダプタに HTTP で問い合わせる NavSource。

    ブラウザ ──> この UI サーバ(venv) ──HTTP──> ros_adapter(Docker/rclpy) ──> ROS

⚠️ **アダプタ本体はまだ無い。** これはその相手を待つ側の実装。契約（`nav/base.py` の
`NavState` / `CommandResult`）を先に固定しておくことで、アダプタと UI を別々に作れる。

アダプタ側が満たすべき口:
    GET  /api/state            -> NavState.as_dict() と同じ JSON
    GET  /api/map.json         -> {"resolution":..., "origin":[x,y], "width":..., "height":...}
    GET  /api/map.png          -> 地図の PNG
    POST /api/command/<name>   -> CommandResult.as_dict() と同じ JSON

⚠️ **タイムアウトは短くすること。** アダプタが固まったときに UI まで巻き込まれると、
画面が「最後に見えた状態」のまま止まり、止まっていることに気づけない。
取れなければ connected=False を返して、UI 側に「切れている」と描かせる。
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request

from .base import CommandResult, NavSource, NavState, Pose


class HttpNavSource(NavSource):
    """ROS アダプタの HTTP 口を叩く。"""

    def __init__(self, base_url: str, timeout_s: float = 0.5) -> None:
        self._base = base_url.rstrip("/")
        self._timeout = float(timeout_s)
        self._map_info: dict[str, object] | None = None
        self._map_png: bytes | None = None
        self._warned = False
        print(f"[航法] ROS アダプタに接続する: {self._base}")

    def _get(self, path: str) -> bytes | None:
        try:
            with urllib.request.urlopen(f"{self._base}{path}", timeout=self._timeout) as res:
                return res.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            if not self._warned:
                print(f"[航法] アダプタに繋がりません({path}): {exc}")
                self._warned = True
            return None

    def get_state(self) -> NavState:
        raw = self._get("/api/state")
        if raw is None:
            return NavState(connected=False, message="ROS アダプタに繋がっていません")
        self._warned = False
        try:
            d = json.loads(raw)
        except json.JSONDecodeError as exc:
            return NavState(connected=False, message=f"アダプタの応答を解釈できません: {exc}")

        def pose_of(key: str) -> Pose | None:
            p = d.get(key)
            if not p:
                return None
            return Pose(float(p["x"]), float(p["y"]), float(p.get("yaw_deg", 0.0)))

        patrol = d.get("patrol") or {}
        return NavState(
            connected=bool(d.get("connected", True)),
            pose=pose_of("pose"),
            goal=pose_of("goal"),
            plan=[(float(a), float(b)) for a, b in d.get("plan", [])],
            route=[(float(a), float(b)) for a, b in d.get("route", [])],
            bridge_state=str(d.get("bridge_state", "DISCONNECTED")),
            patrol_state=str(patrol.get("state", "IDLE")),
            patrol_index=int(patrol.get("index", 0)),
            patrol_total=int(patrol.get("total", 0)),
            message=str(d.get("message", "")),
        )

    def send_command(self, name: str) -> CommandResult:
        req = urllib.request.Request(f"{self._base}/api/command/{name}", method="POST", data=b"")
        try:
            # 操作は人が押した時だけなので、状態取得より長く待ってよい
            with urllib.request.urlopen(req, timeout=max(2.0, self._timeout)) as res:
                d = json.loads(res.read())
            return CommandResult(bool(d.get("ok", False)), str(d.get("message", "")))
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return CommandResult(False, f"アダプタに届きませんでした: {exc}")

    def map_info(self) -> dict[str, object] | None:
        if self._map_info is None:
            raw = self._get("/api/map.json")
            if raw is not None:
                try:
                    self._map_info = json.loads(raw)
                except json.JSONDecodeError:
                    return None
        return self._map_info

    def map_png(self) -> bytes | None:
        if self._map_png is None:
            self._map_png = self._get("/api/map.png")
        return self._map_png
