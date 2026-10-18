"""コンソールを Jetson からループバック以外へ開くときの認証。標準ライブラリのみ。

- トークンは ~/.g1console/token（0600）。初回に生成する。比較は定数時間。
- `/login?token=…` で Cookie（HttpOnly・SameSite=Strict）を発行し、以後は Cookie か `Authorization: Bearer`。
- `Host` ヘッダは待ち受けアドレス・ループバック・明示した名前だけ許す（DNS rebinding 対策）。
- POST は `Origin` があれば `Host` と同じものだけ許す。
- 平文 HTTP なので、同じ LAN で通信を覗かれればトークンは漏れる。運動操作の安全はこのトークンだけに頼らない（README 参照）。
"""
import hmac
import ipaddress
import os
import secrets
import subprocess
import threading
import time
from http.cookies import SimpleCookie
from pathlib import Path

TOKEN_PATH = Path.home() / ".g1console" / "token"
COOKIE_NAME = "g1c_token"
LOOPBACK_NAMES = ("localhost", "127.0.0.1", "::1")
FAIL_LIMIT = 5  # この回数の失敗で
FAIL_WINDOW_S = 60.0  # この秒数のあいだ
LOCKOUT_S = 30.0  # この秒数ロックする
IP_CMD_TIMEOUT_S = 3.0


def load_or_create_token(path=TOKEN_PATH) -> str:
    """トークンを読む。無ければ生成して 0600 で保存する。"""
    path = Path(path)
    try:
        token = path.read_text(encoding="utf-8").strip()
        if token:
            return token
    except OSError:
        pass
    token = secrets.token_urlsafe(24)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(token + "\n")
    os.chmod(str(path), 0o600)
    return token


def iface_ipv4(name: str, run=subprocess.run) -> str:
    """ネットワークインタフェースの IPv4 アドレスを返す（`ip -4 -o addr show dev NAME`）。取れなければ ValueError。"""
    if not name or name.startswith("-") or not all(c.isalnum() or c in "._-" for c in name):
        raise ValueError("インタフェース名が不正です: %r" % name)
    try:
        done = run(["ip", "-4", "-o", "addr", "show", "dev", name], capture_output=True, text=True,
                   timeout=IP_CMD_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError("%s の IP アドレスを取得できません: %s" % (name, exc)) from None
    for field in done.stdout.split():
        if "/" in field:
            try:
                return str(ipaddress.IPv4Interface(field).ip)
            except ValueError:
                continue
    raise ValueError("%s に IPv4 アドレスがありません（接続していますか）" % name)


def parse_bind(value: str) -> str:
    """--bind の値を検証する。IPv4 のみ。0.0.0.0（全インタフェース）は eth0 のロボット内部 LAN にも開くので拒否。"""
    try:
        addr = ipaddress.IPv4Address(value)
    except ValueError:
        raise ValueError("--bind は IPv4 アドレスで指定してください: %r" % value) from None
    if addr.is_unspecified:
        raise ValueError("0.0.0.0 には開けません（ロボット内部 LAN にも開いてしまう）。--bind-iface か個別の IP を指定してください")
    return str(addr)


def is_loopback(addr: str) -> bool:
    try:
        return ipaddress.ip_address(addr).is_loopback
    except ValueError:
        return False


def host_only(header: str) -> str:
    """`Host` ヘッダからポートを除いたホスト部（小文字）。"""
    h = (header or "").strip().lower()
    if h.startswith("["):  # [::1]:18790
        return h[1:].split("]", 1)[0]
    return h.rsplit(":", 1)[0] if h.count(":") == 1 else h


class Auth:
    def __init__(self, token: str, allowed_hosts=(), required: bool = True):
        if required and not token:
            raise ValueError("token is empty")
        self.required = required  # False = トークンを求めない（Host・Origin の検査だけ残す）
        self._token = token or ""
        self._hosts = set(LOOPBACK_NAMES) | {h.lower() for h in allowed_hosts}
        self._lock = threading.Lock()
        self._fails = {}  # client ip -> [失敗時刻…]
        self._locked_until = {}

    def check_host(self, header: str) -> bool:
        return host_only(header) in self._hosts

    def check_origin(self, origin, host_header: str) -> bool:
        """Origin が無ければ通す（curl 等）。あれば Host と同じものだけ。"""
        if not origin:
            return True
        netloc = origin.split("://", 1)[-1].rstrip("/")
        return netloc.lower() == (host_header or "").strip().lower()

    def _same(self, candidate: str) -> bool:
        return hmac.compare_digest(candidate.encode("utf-8"), self._token.encode("utf-8"))

    def authorized(self, headers) -> bool:
        """Cookie か Bearer に正しいトークンがあるか。ここでは失敗を数えない（画面の通常の読み込みで数えない）。"""
        if not self.required:
            return True
        bearer = headers.get("Authorization", "")
        if bearer.startswith("Bearer ") and self._same(bearer[len("Bearer "):].strip()):
            return True
        cookie = SimpleCookie()
        try:
            cookie.load(headers.get("Cookie", ""))
        except Exception:  # noqa: BLE001  壊れた Cookie は無いものとして扱う
            return False
        morsel = cookie.get(COOKIE_NAME)
        return bool(morsel) and self._same(morsel.value)

    def throttled(self, client: str, now=None) -> bool:
        now = time.monotonic() if now is None else now
        with self._lock:
            return self._locked_until.get(client, 0.0) > now

    def verify(self, candidate: str, client: str, now=None) -> bool:
        """ログインのトークン照合。失敗が続くとそのクライアントを一時ロックする。ロック中は正解でも拒否。"""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._locked_until.get(client, 0.0) > now:
                return False
            if self._same(candidate or ""):
                self._fails.pop(client, None)
                return True
            recent = [t for t in self._fails.get(client, []) if now - t < FAIL_WINDOW_S] + [now]
            self._fails[client] = recent
            if len(recent) >= FAIL_LIMIT:
                self._locked_until[client] = now + LOCKOUT_S
                self._fails.pop(client, None)
            return False

    def cookie_header(self) -> str:
        return "%s=%s; Path=/; HttpOnly; SameSite=Strict" % (COOKIE_NAME, self._token)
