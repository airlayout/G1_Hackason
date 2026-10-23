import unittest

from estop.io import ECHO_TIMEOUT_S, SEND_PERIOD_S, SerialEchoSource


class FakeSerial:
    """TXD→接点→RXD のループを模す。closed=False なら返りが無い。"""

    def __init__(self) -> None:
        self.closed = True
        self.broken = False
        self.stuck: bytes = b""   # 接点に関係なく RXD に入ってくる値（固着の模擬）
        self._buf = b""

    @property
    def in_waiting(self) -> int:
        if self.broken:
            raise OSError("device disconnected")
        return len(self._buf)

    def read(self, n: int) -> bytes:
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def write(self, data: bytes) -> None:
        if self.broken:
            raise OSError("device disconnected")
        self._buf += data if self.closed else self.stuck


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class SerialEchoSourceTest(unittest.TestCase):
    def setUp(self):
        self.ser, self.clk = FakeSerial(), Clock()
        self.src = SerialEchoSource("fake", ser=self.ser, clock=self.clk)

    def run_for(self, seconds: float) -> bool:
        ok = False
        end = self.clk.t + seconds
        while self.clk.t < end:
            self.clk.t += 0.01
            ok = self.src.is_ok()
        return ok

    def test_not_ok_before_first_echo(self):
        self.assertFalse(self.src.is_ok())

    def test_ok_when_contact_closed(self):
        self.assertTrue(self.run_for(1.0))

    def test_stop_when_contact_opens(self):
        self.run_for(1.0)
        self.ser.closed = False
        self.assertFalse(self.run_for(ECHO_TIMEOUT_S + SEND_PERIOD_S + 0.1))

    def test_recovers_when_contact_closes_again(self):
        self.ser.closed = False
        self.assertFalse(self.run_for(1.0))
        self.ser.closed = True
        self.assertTrue(self.run_for(0.5))

    def test_stuck_rx_is_not_ok(self):
        self.ser.closed = False
        self.ser.stuck = b"\x01\x02\x03\x7f"  # 毎回違う値を返さない固着は不一致になる
        self.assertFalse(self.run_for(2.0))

    def test_usb_unplug_is_stop(self):
        self.run_for(1.0)
        self.ser.broken = True
        self.assertFalse(self.src.is_ok())


if __name__ == "__main__":
    unittest.main()
