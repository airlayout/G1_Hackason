"""Foxglove Bridge に最小の WebSocket クライアントで繋ぎ、実際に流れているか確かめる。

ブラウザを開く前の切り分け用。ブラウザで見えないとき、原因が
「橋が落ちている」「トンネルが切れている」「トピックが出ていない」のどれかを
依存パッケージ無し（標準ライブラリのみ）で判定できる。

    bash tunnel_foxglove.sh
    python3 check_foxglove_stream.py /unitree/slam_mapping/points 5
    python3 check_foxglove_stream.py /utlidar/cloud_livox_mid360 5

2026-09-03 の実測: 生LiDAR 10.13Hz / 4.5MB/s、地図 9.97Hz / 0.4MB/s。
encoding=cdr（バイナリのまま）で届く。
"""
import base64, json, os, socket, struct, sys, time

HOST, PORT = "127.0.0.1", 8765
WANT = sys.argv[1] if len(sys.argv) > 1 else "/utlidar/cloud_livox_mid360"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0


def handshake(s):
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((
        "GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n"
        "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n" % (HOST, PORT, key)
    ).encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(1)
    assert b"101" in buf.split(b"\r\n")[0], buf[:120]


def recv_exact(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError
        b += c
    return b


def recv_frame(s):
    """(opcode, payload) を返す。サーバ→クライアントはマスクされない。"""
    b0, b1 = recv_exact(s, 2)
    op = b0 & 0x0F
    ln = b1 & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", recv_exact(s, 2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", recv_exact(s, 8))[0]
    if b1 & 0x80:
        m = recv_exact(s, 4)
        p = bytearray(recv_exact(s, ln))
        for i in range(ln):
            p[i] ^= m[i % 4]
        return op, bytes(p)
    return op, recv_exact(s, ln)


def send_text(s, obj):
    p = json.dumps(obj).encode()
    m = os.urandom(4)
    hdr = bytearray([0x81])
    n = len(p)
    if n < 126:
        hdr.append(0x80 | n)
    elif n < 65536:
        hdr.append(0x80 | 126); hdr += struct.pack(">H", n)
    else:
        hdr.append(0x80 | 127); hdr += struct.pack(">Q", n)
    masked = bytes(c ^ m[i % 4] for i, c in enumerate(p))
    s.sendall(bytes(hdr) + m + masked)


s = socket.create_connection((HOST, PORT), timeout=15)
handshake(s)
print("  ハンドシェイク OK")

chan = None
deadline = time.time() + 12
while chan is None and time.time() < deadline:
    op, pl = recv_frame(s)
    if op != 1:
        continue
    msg = json.loads(pl)
    if msg.get("op") == "serverInfo":
        print("  serverInfo: name=%s capabilities=%s" % (msg.get("name"), msg.get("capabilities")))
    elif msg.get("op") == "advertise":
        for c in msg["channels"]:
            if c["topic"] == WANT:
                chan = c
                print("  advertise: %s  schema=%s  encoding=%s  id=%s"
                      % (c["topic"], c["schemaName"], c["encoding"], c["id"]))
                break

if chan is None:
    print("  **%s が advertise されなかった**" % WANT); sys.exit(1)

send_text(s, {"op": "subscribe", "subscriptions": [{"id": 1, "channelId": chan["id"]}]})
print("  subscribe 送信 -> %.1f 秒受信する" % DURATION)

n = tot = 0
t0 = time.time()
s.settimeout(DURATION + 5)
while time.time() - t0 < DURATION:
    op, pl = recv_frame(s)
    if op == 2 and pl and pl[0] == 1:      # binary / MessageData
        n += 1
        tot += len(pl)
el = time.time() - t0
print("  受信: %d メッセージ / %.1f MB / %.2f 秒  => %.2f Hz, %.1f MB/s"
      % (n, tot / 1e6, el, n / el, tot / 1e6 / el))
s.close()
