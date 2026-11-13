"""オンボード音声処理の期限付き生存報告と、Nav2 の既存 heartbeat wire 形式。"""

from __future__ import annotations

import math
import secrets
import struct
import threading
import time
from collections.abc import Callable

ONBOARD_HEARTBEAT_PORT = 47312


class HealthLease:
    def __init__(self, timeout_s: float = 1.0, clock: Callable[[], float] = time.monotonic) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("生存報告の期限は正の有限値にしてください")
        self.timeout_s, self.clock = timeout_s, clock
        self.updated_at = float("-inf")
        self.healthy = False
        self.owner = ""
        self.lock = threading.Lock()

    def update(self, session: str, healthy: bool, sent_at: float) -> bool:
        now = self.clock()
        if not isinstance(session, str) or not session or type(healthy) is not bool:
            return False
        if not isinstance(sent_at, (int, float)) or not math.isfinite(sent_at) or not 0 <= now - sent_at < self.timeout_s:
            return False
        with self.lock:
            if self.owner and self.owner != session and now - self.updated_at < self.timeout_s:
                return False
            self.owner, self.healthy, self.updated_at = session, healthy, now
        return True

    def alive(self) -> bool:
        with self.lock:
            return self.healthy and self.clock() - self.updated_at < self.timeout_s


class HeartbeatWire:
    def __init__(self) -> None:
        self.session = secrets.randbits(64)
        self.sequence = 0

    def next(self) -> bytes:
        self.sequence += 1
        # Navigation/.../heartbeat.hpp と同じ 29 byte。別ポート・loopback で明示的に使う。
        return struct.pack("<IBQQQ", 0x47314842, 1, self.session, self.sequence, time.monotonic_ns())
