"""マップ上の名前つき座標（ロケーション）。タスクの argv から `{loc:NAME.x}` で参照する。

座標系は Nav2 の map フレーム（単位 m と度）。保存は固定パスの JSON を原子的に置き換える。
読み込みで壊れていたら起動を止める（黙って空にして上書きしない）。
"""
import json
import math
import os
import re
import threading

NAME_RE = re.compile(r"^[\w\-]{1,40}$")
MAP_ID_RE = re.compile(r"^[\w.\-]{0,64}$")
FIELDS = ("x", "y", "yaw_deg")
MAX_LOCATIONS = 200
COORD_LIMIT = 10000.0
VERSION = 1


class LocationError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status, self.message = status, message


def _num(body: dict, key: str, limit: float) -> float:
    value = body.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > limit:
        raise LocationError(400, "%s は有限の数値（絶対値 %g 以下）" % (key, limit))
    return float(value)


def validate(body) -> dict:
    if not isinstance(body, dict):
        raise LocationError(400, "本文は JSON オブジェクト")
    name = body.get("name")
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise LocationError(400, "name は英数字・日本語・_ - の 1〜40 文字")
    map_id = body.get("map_id", "")
    if not isinstance(map_id, str) or not MAP_ID_RE.match(map_id):
        raise LocationError(400, "map_id は英数字と . _ - の 64 文字以内")
    return {"name": name, "x": round(_num(body, "x", COORD_LIMIT), 3), "y": round(_num(body, "y", COORD_LIMIT), 3),
            "yaw_deg": round(_num(body, "yaw_deg", 180.0), 1), "map_id": map_id}


class LocationStore:
    def __init__(self, path):
        self._path = str(path)
        self._lock = threading.Lock()
        self._items = self._load()

    def _load(self) -> dict:
        try:
            with open(self._path, encoding="utf-8") as fh:
                raw = json.load(fh)
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as exc:
            raise ValueError("%s を読めません: %s" % (self._path, exc))
        if not isinstance(raw, dict) or raw.get("version") != VERSION or not isinstance(raw.get("locations"), list):
            raise ValueError("%s の形式が不正です（version %d の locations 配列）" % (self._path, VERSION))
        items = {}
        for entry in raw["locations"]:
            try:
                item = validate(entry)
            except LocationError as exc:
                raise ValueError("%s: %s" % (self._path, exc.message))
            if item["name"] in items:
                raise ValueError("%s: name が重複しています: %s" % (self._path, item["name"]))
            items[item["name"]] = item
        if len(items) > MAX_LOCATIONS:
            raise ValueError("%s: 地点が多すぎます（最大 %d）" % (self._path, MAX_LOCATIONS))
        return items

    def _save(self, items: dict) -> None:
        directory = os.path.dirname(self._path) or "."
        os.makedirs(directory, exist_ok=True)
        tmp = self._path + ".tmp"
        payload = {"version": VERSION, "locations": [items[k] for k in sorted(items)]}
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self._path)

    def list(self) -> dict:
        with self._lock:
            return {"locations": [self._items[k] for k in sorted(self._items)]}

    def upsert(self, body) -> dict:
        item = validate(body)
        with self._lock:
            items = dict(self._items)
            if item["name"] not in items and len(items) >= MAX_LOCATIONS:
                raise LocationError(409, "地点が上限（%d）に達しています" % MAX_LOCATIONS)
            items[item["name"]] = item
            self._save(items)  # 保存できたときだけ反映する
            self._items = items
        return item

    def delete(self, name) -> dict:
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise LocationError(400, "name が不正です")
        with self._lock:
            if name not in self._items:
                raise LocationError(404, "地点がありません: %s" % name)
            items = {k: v for k, v in self._items.items() if k != name}
            self._save(items)
            self._items = items
        return {"deleted": name}

    def resolve(self, name: str, field: str) -> str:
        """タスクの `{loc:NAME.field}` の置換値。未登録は 409。"""
        with self._lock:
            item = self._items.get(name)
        if item is None:
            raise LocationError(409, "地点 %r が登録されていません" % name)
        return ("%.1f" if field == "yaw_deg" else "%.3f") % item[field]
