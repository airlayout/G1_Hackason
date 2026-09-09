import unittest

import _pathfix  # noqa: F401
from protocol import MAGIC, BridgeStatus, CmdPacket, StatePacket, make_cmd


class TestCmdPacket(unittest.TestCase):
    def test_roundtrip(self):
        pkt = make_cmd(seq=42, vx=0.2, vy=0.0, omega=-0.1)
        decoded = CmdPacket.decode(pkt.encode())
        self.assertEqual(decoded, pkt)

    def test_decode_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            CmdPacket.decode(b"\x00" * 4)

    def test_decode_rejects_bad_magic(self):
        pkt = make_cmd(seq=1, vx=0.0, vy=0.0, omega=0.0)
        raw = bytearray(pkt.encode())
        raw[0:4] = (MAGIC ^ 0xFFFFFFFF).to_bytes(4, "little")
        with self.assertRaises(ValueError):
            CmdPacket.decode(bytes(raw))

    def test_age_seconds_is_nonnegative_and_grows(self):
        pkt = make_cmd(seq=1, vx=0.0, vy=0.0, omega=0.0)
        now = pkt.timestamp_ns
        self.assertEqual(pkt.age_seconds(now), 0.0)
        self.assertAlmostEqual(pkt.age_seconds(now + 100_000_000), 0.1, places=6)

    def test_age_seconds_clamped_to_zero_if_future(self):
        # 時刻の巻き戻り(単調時計なので理論上起きないはずだが)に対しても負にならないことを保証する
        pkt = make_cmd(seq=1, vx=0.0, vy=0.0, omega=0.0)
        self.assertEqual(pkt.age_seconds(pkt.timestamp_ns - 1_000_000), 0.0)


class TestStatePacket(unittest.TestCase):
    def test_roundtrip(self):
        pkt = StatePacket(
            seq=7,
            timestamp_ns=123456789,
            status=BridgeStatus.NAVIGATING,
            x=1.5,
            y=-2.5,
            yaw=0.78,
            vx=0.1,
            vy=0.0,
            omega=0.05,
            sdk_error_count=2,
        )
        decoded = StatePacket.decode(pkt.encode())
        self.assertEqual(decoded, pkt)
        self.assertIsInstance(decoded.status, BridgeStatus)

    def test_decode_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            StatePacket.decode(b"\x00" * 10)


if __name__ == "__main__":
    unittest.main()
