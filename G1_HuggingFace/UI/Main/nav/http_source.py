"""Docker 内で動く ROS アダプタに HTTP で問い合わせる NavSource。

    ブラウザ ──> この UI サーバ(venv) ──HTTP──> ros_adapter(Docker/rclpy) ──> ROS

アダプタは2つの置き方がある。どちらも同じ口を持つので、この側は URL を変えるだけ。
    有線 … 操作PC の Docker 内（`Adapter/run.sh`）。既定 http://127.0.0.1:8081
    無線 … PC2 の pixi 環境（`run.sh --wifi`）。DDS を無線に通さないため、ROS の購読は
           PC2 の中で済ませて HTTP だけを Wi-Fi に流す（Navigation の WIRELESS.md と同じ方針）

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
import time
import urllib.error
import urllib.request
from pathlib import Path

from .base import CommandResult, NavSource, NavState, Pose

# Nav2 の地図と表示中の地図を照合し直す間隔。地図はめったに変わらないので粗くてよい
_MAP_CHECK_S = 10.0


class HttpNavSource(NavSource):
    """ROS アダプタの HTTP 口を叩く。"""

    def __init__(self, base_url: str, timeout_s: float = 0.5, read_only: bool = False,
                 map_yaml: Path | None = None) -> None:
        self._base = base_url.rstrip("/")
        self._timeout = float(timeout_s)
        # ⚠️ **UI 側でも操作を止める。** アダプタ側の read_only と二重にしておく
        #    （片方の設定を忘れても機体に操作が届かないように）
        self._read_only = bool(read_only)
        self._map_png: bytes | None = None
        self._warned = False
        # 地図ファイルを指定されたら、Nav2 の /map ではなくそれを描く（照合して食い違いは警告）
        self._file_map = None
        if map_yaml is not None:
            from mapimage import load_map
            self._file_map = load_map(map_yaml)
            self._map_name = map_yaml.name
        self._nav2_map: dict[str, object] | None = None
        self._nav2_map_at = 0.0
        print(f"[航法] ROS アダプタに接続する: {self._base}"
              f"{'（見るだけ）' if self._read_only else ''}")

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
            read_only=self._read_only or bool(d.get("read_only", False)),
        )

    def send_command(self, name: str) -> CommandResult:
        if self._read_only:
            return CommandResult(False, "見るだけで接続しています（無線）。操作はできません")
        req = urllib.request.Request(f"{self._base}/api/command/{name}", method="POST", data=b"")
        try:
            # 操作は人が押した時だけなので、状態取得より長く待ってよい
            with urllib.request.urlopen(req, timeout=max(2.0, self._timeout)) as res:
                d = json.loads(res.read())
            return CommandResult(bool(d.get("ok", False)), str(d.get("message", "")))
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return CommandResult(False, f"アダプタに届きませんでした: {exc}")

    def _nav2_map_info(self) -> dict[str, object] | None:
        """アダプタが配る地図のメタ情報。数秒ごとに取り直す。

        ⚠️ **最初の1回で固定しない。** Nav2 を別の地図で上げ直すと中身が変わるのに、
        古い地図を描き続けることになる。指紋が変わったら PNG も取り直す。
        """
        now = time.monotonic()
        if self._nav2_map is None or now - self._nav2_map_at > _MAP_CHECK_S:
            self._nav2_map_at = now
            raw = self._get("/api/map.json")
            if raw is not None:
                try:
                    got = json.loads(raw)
                except json.JSONDecodeError:
                    got = None
                if got is not None:
                    if self._nav2_map is not None and got.get("digest") != self._nav2_map.get("digest"):
                        print("[航法] Nav2 の地図が変わった。取り直す")
                        self._map_png = None
                    self._nav2_map = got
        return self._nav2_map

    def map_info(self) -> dict[str, object] | None:
        if self._file_map is not None:
            return self._file_map.as_dict()
        return self._nav2_map_info()

    def map_png(self) -> bytes | None:
        if self._file_map is not None:
            return self._file_map.png
        if self._map_png is None:
            self._map_png = self._get("/api/map.png")
        return self._map_png

    def map_warning(self) -> str:
        """地図ファイルを描いているときだけ、Nav2 の地図と照合する。"""
        if self._file_map is None:
            return ""
        return describe_mismatch(self._file_map.as_dict(), self._nav2_map_info(), self._map_name)


def describe_mismatch(shown: dict[str, object], nav2: dict[str, object] | None,
                      name: str) -> str:
    """表示中の地図と Nav2 の地図の食い違いを一言にする。同じなら空。"""
    if nav2 is None:
        return f"Nav2 の地図がまだ届いていないため、表示中の {name} と同じか確かめられていません"
    if (shown["width"], shown["height"]) != (nav2.get("width"), nav2.get("height")) \
            or abs(float(shown["resolution"]) - float(nav2.get("resolution", 0))) > 1e-6 \
            or any(abs(float(a) - float(b)) > 1e-3
                   for a, b in zip(shown["origin"], nav2.get("origin", [0, 0]))):
        return (f"表示中の {name} は Nav2 の地図と大きさ・原点が違います。"
                "現在地は Nav2 の地図の座標なので、この地図の上ではずれた場所に描かれます")
    if not nav2.get("digest"):
        return f"Nav2 の地図の中身を照合できません（アダプタが古い）。大きさ・原点は {name} と一致"
    if shown.get("digest") != nav2.get("digest"):
        return (f"表示中の {name} は Nav2 の地図と大きさ・原点は同じですが中身が違います"
                "（取り直した地図など）。現在地の座標は合いますが、壁の位置が違って見えます")
    return ""
