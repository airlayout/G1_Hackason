import queue
import unittest

from estop.io import ButtonSource, RobotPort
from estop.monitor import tick
from estop.safe_stop import Observation, SafeStop, State

STILL = Observation(2.0, 0.0, 0.02)


class FakeButton(ButtonSource):
    def __init__(self) -> None:
        self.ok = True

    def is_ok(self) -> bool:
        return self.ok


class FakeRobot(RobotPort):
    def __init__(self) -> None:
        self.obs = STILL
        self.applied: list[str] = []

    def observe(self) -> Observation:
        return self.obs

    def apply(self, cmd) -> None:
        self.applied.append(cmd.locomotion)


class MonitorTest(unittest.TestCase):
    def setUp(self):
        self.b, self.r, self.s = FakeButton(), FakeRobot(), SafeStop()
        self.q: queue.Queue[str] = queue.Queue()
        self.logs: list[str] = []

    def tick(self, now):
        tick(now, self.b, self.r, self.s, self.q, self.logs.append)

    def test_armed_sends_nothing_to_robot(self):
        self.tick(0.0)
        self.assertEqual(self.r.applied, [])

    def test_button_open_sends_stop_move_every_tick(self):
        self.b.ok = False
        self.tick(0.0)
        self.tick(0.01)
        self.assertEqual(self.r.applied, ["stop_move", "stop_move"])
        self.assertTrue(any("stopping" in m for m in self.logs))

    def test_operator_damp_and_reset(self):
        self.b.ok = False
        self.tick(0.0)
        self.q.put("damp")
        self.tick(0.1)
        self.assertEqual(self.r.applied[-1], "damp")
        self.b.ok = True
        self.tick(0.2)
        self.q.put("reset")
        self.tick(3.0)
        self.assertEqual(self.s.state, State.ARMED)

    def test_reset_ignored_while_button_open(self):
        self.b.ok = False
        self.tick(0.0)
        self.q.put("reset")
        self.tick(5.0)
        self.assertNotEqual(self.s.state, State.ARMED)


if __name__ == "__main__":
    unittest.main()
