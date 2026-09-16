"""Foxglove Bridge に繋ぐ最小の WebSocket クライアント（標準ライブラリのみ）。

`check_foxglove_stream.py`（流量を測る）と `check_foxglove_layout.py`
（レイアウトの突合）の共通部分。依存を足さないのは、コンテナ・PC2・Mac の
どれでも同じ 1 ファイルで動かせるようにするため。

プロトコルは Foxglove WebSocket v1:
    サーバ → JSON の serverInfo / advertise、バイナリの MessageData
    クライアント → JSON の subscribe
"""
import base64, json, os, socket, struct, time

SUBPROTOCOL = "foxglove.websocket.v1"
OP_TEXT, OP_BINARY = 1, 2
MSG_DATA = 0x01                       # MessageData のバイナリ先頭 1 バイト


class FoxgloveClient:
    def __init__(self, host="127.0.0.1", port=8765, timeout=15.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self._handshake(host, port)
        self.server_info = None
        self.channels = {}            # topic -> channel dict

    # ── WebSocket の下回り ────────────────────────────────────────────
    def _handshake(self, host, port):
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            "GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
            "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: %s\r\n\r\n"
            % (host, port, key, SUBPROTOCOL)
        ).encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(1)
            if not chunk:
                raise EOFError("ハンドシェイク中に切断された")
            buf += chunk
        if b"101" not in buf.split(b"\r\n")[0]:
            raise RuntimeError("ハンドシェイク失敗: %r" % buf[:120])

    def _recv_exact(self, n):
        out = b""
        while len(out) < n:
            chunk = self.sock.recv(n - len(out))
            if not chunk:
                raise EOFError
            out += chunk
        return out

    def recv_frame(self):
        """(opcode, payload) を返す。サーバ → クライアントはマスクされない。"""
        b0, b1 = self._recv_exact(2)
        opcode = b0 & 0x0F
        length = b1 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._recv_exact(8))[0]
        if not b1 & 0x80:
            return opcode, self._recv_exact(length)
        mask = self._recv_exact(4)
        payload = bytearray(self._recv_exact(length))
        for i in range(length):
            payload[i] ^= mask[i % 4]
        return opcode, bytes(payload)

    def send_json(self, obj):
        payload = json.dumps(obj).encode()
        mask = os.urandom(4)
        header = bytearray([0x80 | OP_TEXT])
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        masked = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        self.sock.sendall(bytes(header) + mask + masked)

    def send_binary(self, payload: bytes):
        """クライアント→サーバのバイナリ枠（ClientMessageData 用）。

        ⚠️ クライアントからの枠は **必ずマスクする**（RFC 6455）。send_json と同じ作り。
        """
        mask = os.urandom(4)
        header = bytearray([0x80 | OP_BINARY])
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        masked = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        self.sock.sendall(bytes(header) + mask + masked)

    # ── Foxglove の手順 ──────────────────────────────────────────────
    def collect_channels(self, settle=3.0):
        """advertise は分割して届くので、静まるまで集める。"""
        self.sock.settimeout(1.0)
        deadline = time.time() + settle
        while time.time() < deadline:
            try:
                opcode, payload = self.recv_frame()
            except (socket.timeout, TimeoutError):
                continue
            if opcode != OP_TEXT:
                continue
            msg = json.loads(payload)
            if msg.get("op") == "serverInfo":
                self.server_info = msg
            elif msg.get("op") == "advertise":
                for ch in msg["channels"]:
                    self.channels[ch["topic"]] = ch
        return self.channels

    def subscribe(self, topics):
        """topic のリストを購読する。{購読 id: topic} を返す。"""
        subs, by_id = [], {}
        for i, topic in enumerate(topics):
            ch = self.channels.get(topic)
            if ch is None:
                continue
            subs.append({"id": i, "channelId": ch["id"]})
            by_id[i] = topic
        if subs:
            self.send_json({"op": "subscribe", "subscriptions": subs})
        return by_id

    def receive(self, seconds, by_id):
        """seconds 秒受け、{topic: (件数, バイト数)} を返す。"""
        stats = {topic: [0, 0] for topic in by_id.values()}
        self.sock.settimeout(seconds + 5)
        start = time.time()
        while time.time() - start < seconds:
            opcode, payload = self.recv_frame()
            if opcode != OP_BINARY or not payload or payload[0] != MSG_DATA:
                continue
            sub_id = struct.unpack_from("<I", payload, 1)[0]
            topic = by_id.get(sub_id)
            if topic is None:
                continue
            stats[topic][0] += 1
            stats[topic][1] += len(payload)
        return {t: tuple(v) for t, v in stats.items()}, time.time() - start

    def close(self):
        self.sock.close()
