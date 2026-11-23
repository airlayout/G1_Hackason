import json
import socket
import struct
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from g1console.lidar_sensor import (CMD_GET, CMD_SET, IDLE, KEY_WORK_MODE, SAMPLING, LivoxSensorCtl,  # noqa: E402
                                    MockSensorCtl, build_packet, crc16_ccitt_false, parse_reply)
from server import MockHelper, Monitor, make_handler  # noqa: E402

# 実機の LiDAR が流していた探索応答 2 個（ヘッダ 18 バイトと CRC16 の組）
REAL_PKTS = ["aa0030006e08000000000101000000000000cd917f554d9e002341524d43503445303033333639380000c0a87b7824db",
             "aa0030006f08000000000101000000000000acea7f554d9e002341524d43503445303033333639380000c0a87b7824db"]


class CrcTest(unittest.TestCase):
    def test_crc_matches_real_lidar_packets(self):
        for h in REAL_PKTS:
            p = bytes.fromhex(h)
            self.assertEqual(crc16_ccitt_false(p[:18]), struct.unpack_from("<H", p, 18)[0])

    def test_build_packet_header_and_crc32(self):
        pkt = build_packet(CMD_GET, b"\x01\x00\x00\x00\x1a\x00", 7)
        self.assertEqual((pkt[0], struct.unpack_from("<H", pkt, 2)[0], struct.unpack_from("<H", pkt, 8)[0]), (0xAA, len(pkt), CMD_GET))
        self.assertEqual(struct.unpack_from("<H", pkt, 18)[0], crc16_ccitt_false(pkt[:18]))

    def test_parse_reply_ignores_other_commands(self):
        self.assertEqual(parse_reply(build_packet(CMD_GET, b"abc", 1), CMD_GET), b"abc")
        self.assertIsNone(parse_reply(build_packet(CMD_SET, b"abc", 1), CMD_GET))
        self.assertIsNone(parse_reply(b"short", CMD_GET))


class FakeLidar(threading.Thread):
    """LiDAR の模擬。GET は今のモードを、SET は ret_code=0 を返す（reject=True なら 3）。"""

    def __init__(self, reject=False):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.addr = self.sock.getsockname()
        self.mode, self.reject, self.stop = SAMPLING, reject, False

    def run(self):
        self.sock.settimeout(0.1)
        while not self.stop:
            try:
                pkt, peer = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            cmd, body = struct.unpack_from("<H", pkt, 8)[0], pkt[24:]
            if cmd == CMD_SET:
                if self.reject:
                    reply = bytes([3]) + struct.pack("<H", KEY_WORK_MODE)
                else:
                    self.mode, reply = body[-1], bytes([0]) + struct.pack("<H", 0)
            else:
                reply = bytes([0]) + struct.pack("<HHHHB", 1, 0, KEY_WORK_MODE, 1, self.mode)
            self.sock.sendto(build_packet(cmd, reply, 1), peer)


class LivoxCtlTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeLidar()
        self.fake.start()
        self.ctl = LivoxSensorCtl(self.fake.addr)

    def tearDown(self):
        self.fake.stop = True

    def test_idle_then_sampling_reads_back(self):
        self.ctl.settle_s = 0  # 読み戻しの待ちは省く
        self.assertEqual(self.ctl.call("status")["mode"], SAMPLING)
        r = self.ctl.call("idle")
        self.assertEqual((r["mode"], r["laser_off"], self.fake.mode), (IDLE, True, IDLE))
        self.assertEqual(self.ctl.call("sampling")["mode"], SAMPLING)

    def test_rejected_set_raises_with_ret_code(self):
        self.fake.reject = True
        with self.assertRaisesRegex(RuntimeError, "ret_code=3"):
            self.ctl.call("idle")

    def test_unknown_action_is_refused_before_sending(self):
        with self.assertRaises(ValueError):
            self.ctl.call("sleep")  # 3(Sleep) は MID360 では範囲外。許可リストに無い


class ApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        helper = MockHelper()
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), lidar_sensor=MockSensorCtl()))
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def req(self, path, method="GET"):
        r = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), method=method, data=b"" if method == "POST" else None)
        with urllib.request.urlopen(r, timeout=5) as resp:
            return json.loads(resp.read())

    def test_idle_and_sampling_round_trip(self):
        self.assertEqual(self.req("/api/lidar/sensor")["mode"], SAMPLING)
        self.assertTrue(self.req("/api/lidar/sensor/idle", "POST")["laser_off"])
        self.assertEqual(self.req("/api/lidar/sensor")["mode"], IDLE)
        self.assertEqual(self.req("/api/lidar/sensor/sampling", "POST")["mode"], SAMPLING)

    def test_sleep_path_is_not_routed(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.req("/api/lidar/sensor/sleep", "POST")
        self.assertEqual(cm.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
