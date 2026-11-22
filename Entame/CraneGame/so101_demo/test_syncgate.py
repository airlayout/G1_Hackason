import unittest

from home import HOME_JOINTS, input_of, neutral_map
from limits import default_limits
from server import Desk, MockArm
from syncgate import KEEP_SAMPLES, TOLERANCE, SyncGate

HOME = {"shoulder_pan": 0.0, "shoulder_lift": -80.0, "elbow_flex": 70.0, "wrist_flex": 10.0, "wrist_roll": 0.0}


class InputOfTests(unittest.TestCase):
    def test_is_the_inverse_of_neutral_map(self):
        for home in (None, -20.0, -100.0, 100.0):
            for value in (-1, -0.5, 0, 0.3, 1):
                position = neutral_map(value, -60, 40, home)
                back = input_of(position, -60, 40, home)
                if home in (-100.0, 100.0) and (value < 0 if home == -100.0 else value > 0):
                    continue                                     # 基準が端なら、その側の入力は同じ位置に潰れる
                self.assertAlmostEqual(back, value, places=6)

    def test_position_outside_the_range_is_clamped(self):
        self.assertEqual(input_of(100, -60, 40, -20), 1.0)
        self.assertEqual(input_of(-100, -60, 40, -20), -1.0)

    def test_no_room_on_a_side_is_zero(self):
        self.assertEqual(input_of(-100, -100, 100, -100), 0.0)


class SyncGateTests(unittest.TestCase):
    def setUp(self):
        self.limits = default_limits()
        self.current = dict(HOME)

    def update(self, gate, **hand):
        return gate.update({j: v for j, v in hand.items()}, self.current, self.limits, HOME)

    def test_disabled_gate_is_always_engaged(self):
        gate = SyncGate(False)
        self.assertTrue(self.update(gate, shoulder_lift=1.0))

    def test_waits_until_matched_for_enough_samples(self):
        gate = SyncGate(True)
        for _ in range(KEEP_SAMPLES - 1):
            self.assertFalse(self.update(gate, shoulder_lift=0.0, elbow_flex=0.1))
        self.assertTrue(self.update(gate, shoulder_lift=0.0, elbow_flex=0.1))

    def test_a_mismatch_restarts_the_count(self):
        gate = SyncGate(True)
        for _ in range(KEEP_SAMPLES - 1):
            self.update(gate, shoulder_lift=0.0)
        self.assertFalse(self.update(gate, shoulder_lift=0.9))     # 一度でも外れたら数え直し
        self.assertEqual(gate.off(), ["shoulder_lift"])
        for _ in range(KEEP_SAMPLES - 1):
            self.assertFalse(self.update(gate, shoulder_lift=0.0))
        self.assertTrue(self.update(gate, shoulder_lift=0.0))

    def test_engaged_stays_engaged_but_gaps_keep_updating(self):
        gate = SyncGate(True)
        for _ in range(KEEP_SAMPLES):
            self.update(gate, shoulder_lift=0.0)
        self.assertTrue(self.update(gate, shoulder_lift=1.0))
        self.assertAlmostEqual(gate.state()["gaps"]["shoulder_lift"]["hand"], 1.0)

    def test_tolerance_is_inclusive(self):
        gate = SyncGate(True)
        for _ in range(KEEP_SAMPLES):
            self.update(gate, shoulder_lift=TOLERANCE)
        self.assertTrue(gate.engaged)

    def test_ignores_joints_that_are_not_arm_joints(self):
        gate = SyncGate(True)
        self.assertEqual(set(HOME_JOINTS), set(HOME))
        self.update(gate, gripper=1.0)
        self.assertEqual(gate.gaps, {})


class DeskSyncTests(unittest.TestCase):
    def setUp(self):
        self.time = [10.0]
        limits = default_limits()
        for j in HOME_JOINTS:
            limits[j].update(speed=150.0, step=15.0)
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], limits=limits, home=dict(HOME), sync=True)
        self.desk.arm.pose.update(HOME)
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]

    def sample(self, **inputs):
        self.time[0] += 0.065
        return self.desk.command(dict(op="sample", token=self.token, height=0.5, opening=0.5, **inputs))

    def test_arm_does_not_move_while_the_hand_is_off(self):
        before = self.desk.arm.read()
        for _ in range(20):
            state = self.sample(lift=1.0, reach=-1.0, pan=0.8, flex=0.0, roll=0.0)
        self.assertEqual(self.desk.arm.read(), before)
        self.assertTrue(state["sync"]["waiting"])
        self.assertIn("同期待ち", state["note"])
        self.assertAlmostEqual(state["sync"]["gaps"]["shoulder_lift"]["hand"], 1.0)

    def test_moves_once_the_hand_matches(self):
        for _ in range(KEEP_SAMPLES):
            state = self.sample(lift=0.0, reach=0.0, pan=0.0, flex=0.0, roll=0.0)
        self.assertFalse(state["sync"]["waiting"])
        self.assertIn("追従中", state["note"])
        before = self.desk.arm.read()
        self.sample(lift=0.2, reach=0.0, pan=0.0, flex=0.0, roll=0.0)
        self.assertGreater(self.desk.arm.read()["shoulder_lift"], before["shoulder_lift"])

    def test_every_follow_waits_again(self):
        for _ in range(KEEP_SAMPLES):
            self.sample(lift=0.0)
        self.desk.command(dict(op="stop"))
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        self.assertTrue(self.sample(lift=0.0)["sync"]["waiting"])

    def test_gripper_also_waits(self):
        before = self.desk.arm.read()
        self.sample(lift=1.0)
        self.assertEqual(self.desk.arm.read()["gripper"], before["gripper"])

    def test_sync_is_none_when_not_following(self):
        self.desk.command(dict(op="stop"))
        self.assertIsNone(self.desk.state()["sync"])


if __name__ == "__main__":
    unittest.main()
