"""操作者の生存信号（Nav2 の操作 PC heartbeat）を、人が見ている間だけ送る。

形式は Navigation/nav2_stable/g1_sdk_bridge_cpp の HeartbeatPacket（29 バイト・リトルエンディアン）:
magic(4)=0x47314842, version(1)=1, session_id(8), seq(8), send_ns(8)。
受信側（g1_cmd_router）は途絶で FAULT にして機体を止める。

⚠️ ハブが生きているだけでは送らない。ブラウザが「見ている」と通知（touch）した直後だけ送る。
そうしないと、スマホが切れてもハブが信号を出し続け、途絶検知の意味がなくなる。
"""
import secrets
import socket
import struct
import threading
import time

MAGIC = 0x47314842
VERSION = 1
WIRE = struct.Struct("<IBQQQ")
PRESENCE_MAX_AGE_S = 0.6
PERIOD_S = 0.2


def parse_target(spec: str):
    host, sep, port = spec.rpartition(":")
    if not sep or not host or not port.isdigit() or not 1 <= int(port) <= 65535:
        raise ValueError("--heartbeat-to は HOST:PORT の形（例 127.0.0.1:47311）: %r" % spec)
    return host, int(port)


class OperatorHeartbeat:
    def __init__(self, host: str, port: int, clock=time.monotonic, sock=None, session_id=None):
        self._addr = (socket.gethostbyname(host), port)
        self._clock = clock
        self._sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._session = secrets.randbits(64) if session_id is None else session_id
        self._lock = threading.Lock()
        self._last_touch = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread = None

    def touch(self) -> None:
        with self._lock:
            self._last_touch = self._clock()

    def present(self) -> bool:
        with self._lock:
            return self._last_touch is not None and self._clock() - self._last_touch <= PRESENCE_MAX_AGE_S

    def tick(self) -> bool:
        """見ている間だけ 1 パケット送る。送ったら True。"""
        if not self.present():
            return False
        self._seq += 1
        packet = WIRE.pack(MAGIC, VERSION, self._session, self._seq, time.monotonic_ns())
        try:
            self._sock.sendto(packet, self._addr)
        except OSError as exc:
            print("[heartbeat] 送信失敗: %r" % (exc,))
            return False
        return True

    def status(self) -> dict:
        return {"enabled": True, "target": "%s:%d" % self._addr, "present": self.present(), "sent": self._seq}

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="heartbeat", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._sock.close()

    def _loop(self) -> None:
        while not self._stop.wait(PERIOD_S):
            self.tick()
