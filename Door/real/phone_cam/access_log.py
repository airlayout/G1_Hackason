"""受け口に来た接続の跡。電話から「繋がらない」時に、要求が Mac まで来ているか・どこで断ったかを Mac の画面で見分ける。

  - 何も出ない            → 電話の要求が Mac に来ていない（別の Wi-Fi・IP の打ち間違い・受け口が止まっている）
  - 「握手が通らない」     → 来ているが証明書で断られた（iPhone が証明書を信頼していない等。直し方を添える）
  - 「鍵が合わない」       → ページは開いたが鍵が古い（Mac の画面の QR から開き直す）
  - 「撮影のページを渡した」→ 開けている。この先はページの表示（カメラの許可など）

毎秒の状態の問い合わせは出さない（同じ相手・同じ種類は REPEAT_S に 1 回）。
"""
from __future__ import annotations

import http.client
import ssl
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, Optional, Tuple

REPEAT_S = 30.0

# OpenSSL の誤りの名前 → 直し方（上から順に当てる）
TLS_HINTS: Tuple[Tuple[Tuple[str, ...], str], ...] = (
    (("UNKNOWN_CA", "CERTIFICATE_UNKNOWN", "BAD_CERTIFICATE"),
     "iPhone が証明書を信頼していない → iPhone の Safari で {plain} を開いて証明書を入れ、"
     "「設定 → 一般 → 情報 → 証明書信頼設定」で g1_phone_cam をオン"),
    (("HTTP_REQUEST",), "https の口に http:// で来た → Mac の画面の QR から開く（https://）"),
    (("WRONG_VERSION_NUMBER", "UNSUPPORTED_PROTOCOL", "VERSION_TOO_LOW", "NO_SHARED_CIPHER"),
     "TLS の版が合わない（古い iOS？）"),
    (("EOF", "RESET", "BROKEN PIPE", "TIMED OUT"),
     "相手が握手の途中で切った（証明書の警告で Safari が止めた時もこれになる → {plain} で証明書を入れる）"),
)


def tls_hint(err: BaseException, plain_url: str) -> str:
    """握手の誤りから、撮る人に見せる直し方を 1 行で"""
    name = f"{type(err).__name__} {getattr(err, 'reason', '') or ''} {err}".upper()
    for keys, hint in TLS_HINTS:
        if any(k in name for k in keys):
            return hint.format(plain=plain_url)
    return "（直し方は分からない。この行を Claude に見せる）"


class AccessLog:
    """同じ相手・同じ種類の出来事は REPEAT_S に 1 回だけ出す"""

    def __init__(self, plain_url: str = "", out: Callable[[str], None] = print, clock=time.monotonic,
                 quiet_ips: Iterable[str] = ()) -> None:
        self.plain_url = plain_url
        self.out = out
        self.clock = clock
        self.quiet_ips = frozenset(quiet_ips)  # Mac 自身（自己点検）は出さない。電話の跡と紛れる
        self.lock = threading.Lock()
        self.last: Dict[Tuple[str, str], float] = {}
        self.counts: Dict[Tuple[str, str], int] = {}

    def note(self, ip: str, kind: str, msg: str) -> bool:
        if ip in self.quiet_ips:
            return False
        key = (ip, kind)
        now = self.clock()
        with self.lock:
            self.counts[key] = self.counts.get(key, 0) + 1
            t = self.last.get(key)
            if t is not None and now - t < REPEAT_S:
                return False
            n = self.counts[key]
            self.counts[key] = 0
            self.last[key] = now
        more = f"（前に出してから {n} 回目）" if n > 1 else ""
        self.out(f"[phone] {msg}{more}")
        return True

    def tls_failed(self, ip: str, err: BaseException) -> bool:
        why = getattr(err, "reason", None) or type(err).__name__
        return self.note(ip, "tls", f"⚠️ {ip} の https の握手が通らない（{why}）: {tls_hint(err, self.plain_url)}")


def self_check(ip: str, port: int, http_port: int, token: str, ca: Optional[Path], tls: bool = True,
               timeout: float = 3.0) -> Tuple[bool, str]:
    """Mac 自身から LAN の IP で 2 つの口を試す（受け口の誤り・証明書の名前の食い違いを電話より先に見つける）"""
    problems = []
    try:
        if tls:
            ctx = ssl.create_default_context(cafile=str(ca)) if ca is not None else ssl.create_default_context()
            conn: http.client.HTTPConnection = http.client.HTTPSConnection(ip, port, timeout=timeout, context=ctx)
        else:
            conn = http.client.HTTPConnection(ip, port, timeout=timeout)
        conn.request("GET", "/api/time", headers={"X-Key": token})
        r = conn.getresponse()
        r.read()
        conn.close()
        if r.status != 200:
            problems.append(f"{'https' if tls else 'http'}://{ip}:{port} が {r.status}")
    except (OSError, ssl.SSLError, http.client.HTTPException) as e:
        problems.append(f"{'https' if tls else 'http'}://{ip}:{port} に届かない（{type(e).__name__}: {e}）")
    try:
        conn = http.client.HTTPConnection(ip, http_port, timeout=timeout)
        conn.request("GET", "/ca.crt")
        r = conn.getresponse()
        r.read()
        conn.close()
        if r.status != 200:
            problems.append(f"http://{ip}:{http_port}/ca.crt が {r.status}")
    except (OSError, http.client.HTTPException) as e:
        problems.append(f"http://{ip}:{http_port} に届かない（{type(e).__name__}: {e}）")
    if problems:
        return False, "・".join(problems)
    return True, f"Mac 自身から https://{ip}:{port} と http://{ip}:{http_port} に届いた（証明書の名前も合う）"
