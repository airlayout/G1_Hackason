"""環境ごとに変わる接続先（IP アドレス等）の設定。Console/settings.json に保存する（git 対象外）。

値は ssh のコマンドラインや URL に入るので、保存前に厳しく検証する（先頭 `-` 等でオプション注入させない）。
"""
import ipaddress
import json
import os
import re
import threading
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parent / "settings.json"
DEFAULTS = {"dev_pc": "", "jetson_host": "", "jetson_user": "unitree", "jetson_key": "", "g1_ip": "", "camera_port": 8081}
LABELS = {"dev_pc": "開発用 PC", "jetson_host": "Jetson PC", "jetson_user": "Jetson の ssh ユーザー",
          "jetson_key": "ssh 秘密鍵", "g1_ip": "G1 本体", "camera_port": "カメラ配信ポート"}

_HOSTNAME = re.compile(r"^(?=.{1,253}$)[A-Za-z0-9]([A-Za-z0-9._-]*[A-Za-z0-9])?$")
_NUMERIC = re.compile(r"^[0-9.]+$")
_USER = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")
MAX_KEY_PATH = 512


def _host(name: str, value) -> str:
    v = str(value or "").strip()
    if not v:
        return ""
    try:
        return str(ipaddress.ip_address(v))
    except ValueError:
        pass
    if _NUMERIC.match(v) or not _HOSTNAME.match(v):
        raise ValueError("%s は IP アドレス（例 192.168.123.164）かホスト名で入力してください" % LABELS[name])
    return v


def validate(values: dict) -> dict:
    """検証済みの設定（全項目）を返す。不正なら ValueError（画面にそのまま出せる文言）。"""
    out = {**DEFAULTS, **{k: values[k] for k in DEFAULTS if k in values}}
    for name in ("dev_pc", "jetson_host", "g1_ip"):
        out[name] = _host(name, out[name])
    user = str(out["jetson_user"] or "").strip()
    if user and not _USER.match(user):
        raise ValueError("%s は英小文字・数字・_ - だけで入力してください" % LABELS["jetson_user"])
    out["jetson_user"] = user
    key = str(out["jetson_key"] or "").strip()
    if key.startswith("-") or "\0" in key or len(key) > MAX_KEY_PATH:
        raise ValueError("%s のパスが不正です" % LABELS["jetson_key"])
    out["jetson_key"] = key
    try:
        port = int(out["camera_port"])
    except (TypeError, ValueError):
        raise ValueError("%s は数字で入力してください" % LABELS["camera_port"]) from None
    if not 1 <= port <= 65535:
        raise ValueError("%s は 1〜65535 で入力してください" % LABELS["camera_port"])
    out["camera_port"] = port
    return out


class Settings:
    """設定の保持と保存。path=None ならメモリだけ（テスト用）。優先順位: 保存済みファイル > initial > 既定値。"""

    def __init__(self, path=DEFAULT_PATH, initial=None):
        self._path = Path(path) if path else None
        self._lock = threading.Lock()
        saved = {}
        if self._path and self._path.exists():
            try:
                saved = json.loads(self._path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                saved = {}  # 壊れていたら既定値から（次の保存で直る）
        try:
            self._values = validate({**(initial or {}), **saved})
        except ValueError:
            self._values = validate(initial or {})

    def get(self) -> dict:
        with self._lock:
            return dict(self._values)

    def update(self, values: dict) -> dict:
        with self._lock:
            new = validate({**self._values, **values})
            if self._path:
                tmp = self._path.with_suffix(".tmp")
                tmp.write_text(json.dumps(new, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                os.replace(tmp, self._path)
            self._values = new
            return dict(new)

    def ssh_target(self):
        """(ssh の接続先 `user@host`, 鍵パス) を返す。未設定なら (None, "")。"""
        v = self.get()
        if not v["jetson_host"]:
            return None, ""
        return (v["jetson_user"] + "@" if v["jetson_user"] else "") + v["jetson_host"], v["jetson_key"]

    def camera_base(self):
        v = self.get()
        return "http://%s:%d" % (v["jetson_host"] if ":" not in v["jetson_host"] else "[%s]" % v["jetson_host"],
                                 v["camera_port"]) if v["jetson_host"] else None
