"""LiDAR 本体（Livox MID360）の待機・計測の切り替え。lidar_proc.py の「点群配信プロセス」とは別物。

Jetson（--local）から、LiDAR（既定 192.168.123.120:56100/UDP）へ Livox 制御プロトコルを直接送る。
work_tgt_mode（キー 0x001A）を IDLE(2) にするとレーザーを止め、SAMPLING(1) で計測に戻す。電源を入れ直すと SAMPLING に戻る。
PC1 経由ではなく LiDAR へ直接なので、G1 の DDS・モーターには触れない。
出典: Livox MID-360 プロトコル資料（戻りコード 0x03=設定値が範囲外。設定できる値は 1 SAMPLING / 2 IDLE / 9 READY）。
"""
import socket
import struct
import threading
import time
import zlib

LIDAR_ADDR = ("192.168.123.120", 56100)
KEY_WORK_MODE = 0x001A
CMD_SET, CMD_GET = 0x0100, 0x0101
SAMPLING, IDLE = 1, 2
MODE_NAMES = {1: "計測中（SAMPLING）", 2: "待機中（IDLE・レーザー停止）", 9: "準備中（READY）"}
ACTIONS = {"idle": IDLE, "sampling": SAMPLING}
HEADER_LEN = 24
REPLY_WAIT_S = 0.8
SETTLE_S = 1.0
RETRIES = 3
SEQ_MAX = 0xFFFFFFFF


def crc16_ccitt_false(data: bytes) -> int:
    """実機の LiDAR パケットで照合済み（init 0xFFFF, poly 0x1021, MSB first, xorout 0）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(cmd_id: int, data: bytes, seq: int) -> bytes:
    head = struct.pack("<BBHIHBB6s", 0xAA, 0, HEADER_LEN + len(data), seq, cmd_id, 0, 0, b"\0" * 6)
    return head + struct.pack("<HI", crc16_ccitt_false(head), zlib.crc32(data) & 0xFFFFFFFF) + data


def parse_reply(pkt: bytes, cmd_id: int):
    """自分の cmd_id への応答なら、データ部（bytes）。そうでなければ None。"""
    if len(pkt) < HEADER_LEN or pkt[0] != 0xAA or struct.unpack_from("<H", pkt, 8)[0] != cmd_id:
        return None
    return pkt[HEADER_LEN:]


class LivoxSensorCtl:
    backend = "livox"

    def __init__(self, addr=LIDAR_ADDR):
        self.addr = addr
        self.settle_s = SETTLE_S
        self._seq = 0
        self._lock = threading.Lock()  # 画面の連打で応答を取り違えないよう 1 回ずつ送る

    def _ask(self, cmd_id: int, data: bytes):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(0.2)
            sock.connect(self.addr)
            for _ in range(RETRIES):
                self._seq = (self._seq + 1) & SEQ_MAX
                sock.send(build_packet(cmd_id, data, self._seq))
                end = time.time() + REPLY_WAIT_S
                while time.time() < end:
                    try:
                        reply = parse_reply(sock.recv(2048), cmd_id)
                    except socket.timeout:
                        continue
                    if reply is not None:
                        return reply
        finally:
            sock.close()
        raise RuntimeError("LiDAR %s:%d から応答がありません" % self.addr)

    def get_mode(self) -> int:
        reply = self._ask(CMD_GET, struct.pack("<HHH", 1, 0, KEY_WORK_MODE))
        if not reply:
            raise RuntimeError("LiDAR の応答が空です")
        return reply[-1]

    def set_mode(self, mode: int) -> None:
        reply = self._ask(CMD_SET, struct.pack("<HHHHB", 1, 0, KEY_WORK_MODE, 1, mode))
        if not reply or reply[0] != 0:
            raise RuntimeError("LiDAR が設定を受け付けませんでした（ret_code=%s）" % (reply[0] if reply else "なし"))

    def call(self, action: str) -> dict:
        """status / idle / sampling。設定のあとは読み戻して、実際のモードを返す。"""
        if action != "status" and action not in ACTIONS:
            raise ValueError("unknown action")
        with self._lock:
            if action != "status":
                self.set_mode(ACTIONS[action])
                time.sleep(self.settle_s)  # 切り替わりを待ってから読み戻す
            mode = self.get_mode()
        return reply_body(mode, action)


def reply_body(mode: int, action: str) -> dict:
    return {"ok": True, "mode": mode, "label": MODE_NAMES.get(mode, "不明（%s）" % mode),
            "laser_off": mode == IDLE, "action": action}


class MockSensorCtl:
    """実機なしの模擬。"""

    backend = "mock"

    def __init__(self):
        self.mode = SAMPLING

    def call(self, action: str) -> dict:
        if action != "status" and action not in ACTIONS:
            raise ValueError("unknown action")
        if action != "status":
            self.mode = ACTIONS[action]
        return reply_body(self.mode, action)
