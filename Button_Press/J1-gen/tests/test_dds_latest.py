"""lowstate の最新の 1 通だけを読むときに取りに行く受け口（common/dds.py の LatestLowState）を確かめる。

DDS には接続しない（DataReader の代わりに、take() が決めた順に返すだけの偽物を入れる）。
cyclonedds の InvalidSample を使うので、unitree_sdk2py（cyclonedds を含む）が無い環境ではスキップする。
"""

from __future__ import annotations

import time
import unittest
from types import SimpleNamespace
from typing import Any

from _env import needs
from common.dds import SIM_MARKER, LatestLowState
from common.robot_model import NUM_MOTORS


def fake_lowstate(q: float, sim: bool = False) -> SimpleNamespace:
    """parse_lowstate が読む項目だけを持つ LowState_ の代わり。"""
    motor = [SimpleNamespace(q=q, dq=0.0, mode=1) for _ in range(NUM_MOTORS)]
    return SimpleNamespace(motor_state=motor, imu_state=SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0]),
                           mode_machine=5, reserve=[SIM_MARKER if sim else 0, 0])


class FakeReader:
    """take(N=1) を呼ばれるたびに、batches の先頭を 1 つずつ返す（尽きたら空 = 新しい 1 通が無い）。"""

    def __init__(self, batches: list[list[Any]]) -> None:
        self.batches = batches
        self.calls = 0

    def take(self, N: int = 1) -> list[Any]:  # noqa: N803（cyclonedds の引数名に合わせる）
        self.calls += 1
        return self.batches.pop(0) if self.batches else []


@needs("sdk")
class TestLatestLowState(unittest.TestCase):
    def make(self, batches: list[list[Any]]) -> LatestLowState:
        ls = LatestLowState()
        ls._reader = FakeReader(batches)
        return ls

    def test_none_before_first_message(self) -> None:
        self.assertIsNone(self.make([]).latest())

    def test_takes_new_message_on_each_read(self) -> None:
        ls = self.make([[fake_lowstate(0.1)], [fake_lowstate(0.2)]])
        self.assertAlmostEqual(ls.latest().q[0], 0.1)
        self.assertAlmostEqual(ls.latest().q[0], 0.2)
        self.assertEqual(ls.count, 2)

    def test_keeps_old_state_and_stamp_when_lowstate_stops(self) -> None:
        # 途切れたら同じ値・同じ stamp のまま（commander の state_timeout_s で「途切れている」と分かる）
        ls = self.make([[fake_lowstate(0.3)]])
        first = ls.latest()
        time.sleep(0.01)
        again = ls.latest()
        self.assertIs(again, first)
        self.assertGreaterEqual(time.monotonic() - again.stamp, 0.01)
        self.assertEqual(ls.count, 1)

    def test_skips_invalid_sample(self) -> None:
        from cyclonedds.internal import InvalidSample

        invalid = InvalidSample.__new__(InvalidSample)
        ls = self.make([[fake_lowstate(0.4)], [invalid]])
        self.assertAlmostEqual(ls.latest().q[0], 0.4)
        self.assertAlmostEqual(ls.latest().q[0], 0.4)
        self.assertEqual(ls.count, 1)

    def test_sim_marker_is_parsed(self) -> None:
        self.assertTrue(self.make([[fake_lowstate(0.0, sim=True)]]).latest().sim_marker)


if __name__ == "__main__":
    unittest.main()
