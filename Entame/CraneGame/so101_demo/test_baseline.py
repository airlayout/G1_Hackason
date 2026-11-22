import unittest

from baseline import KEEP_SAMPLES, MAX_SAMPLES, Baseline
from home import HOME_JOINTS, input_of, neutral_map, relative
from limits import default_limits
from server import Desk, MockArm

POSE = {"shoulder_pan": 10.0, "shoulder_lift": -40.0, "elbow_flex": 30.0, "wrist_flex": -20.0, "wrist_roll": 5.0}


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


class RelativeTests(unittest.TestCase):
    def test_base_is_zero_and_ends_are_kept(self):
        for base in (-0.6, 0.0, 0.5):
            self.assertAlmostEqual(relative(base, base), 0.0)
            self.assertAlmostEqual(relative(1.0, base), 1.0)
            self.assertAlmostEqual(relative(-1.0, base), -1.0)

    def test_is_proportional_on_each_side(self):
        self.assertAlmostEqual(relative(0.75, 0.5), 0.5)
        self.assertAlmostEqual(relative(0.0, 0.5), -1 / 3)

    def test_base_at_an_end_does_not_blow_up(self):
        self.assertAlmostEqual(relative(1.0, 1.0), 0.0)
        self.assertAlmostEqual(relative(0.9, 1.0), -0.05)
        self.assertAlmostEqual(relative(1.0, 0.95), 0.25)     # 余地が 0.05 しかなくても、0.2 で割る（敏感になりすぎない）


class BaselineTests(unittest.TestCase):
    def setUp(self):
        self.limits = default_limits()

    def update(self, baseline, current=POSE, **hand):
        return baseline.update(hand, current, self.limits)

    def test_disabled_is_always_ready_and_passes_input_through(self):
        baseline = Baseline(False)
        self.assertTrue(self.update(baseline, shoulder_lift=0.4))
        self.assertEqual(baseline.hand_input("shoulder_lift", 0.4), 0.4)
        self.assertEqual(baseline.gaps, {})

    def test_waits_for_a_steady_hand(self):
        baseline = Baseline(True)
        for _ in range(KEEP_SAMPLES - 1):
            self.assertFalse(self.update(baseline, shoulder_lift=0.6))
        self.assertTrue(self.update(baseline, shoulder_lift=0.6))
        self.assertAlmostEqual(baseline.hand["shoulder_lift"], 0.6)
        self.assertEqual(baseline.anchor, POSE)

    def test_a_shaking_hand_postpones_the_baseline(self):
        baseline = Baseline(True)
        for i in range(KEEP_SAMPLES * 3):
            self.assertFalse(self.update(baseline, shoulder_lift=1.0 if i % 2 else -1.0))
        self.assertFalse(baseline.ready)

    def test_a_shaking_hand_gets_a_baseline_at_the_limit(self):
        baseline = Baseline(True)
        for i in range(MAX_SAMPLES - 1):
            self.update(baseline, shoulder_lift=1.0 if i % 2 else -1.0)
        self.assertTrue(self.update(baseline, shoulder_lift=1.0))
        self.assertIn("shoulder_lift", baseline.hand)

    def test_anchor_is_the_arm_pose_when_the_baseline_was_taken(self):
        baseline = Baseline(True)
        for _ in range(KEEP_SAMPLES):
            self.update(baseline, shoulder_lift=0.0)
        self.update(baseline, current={**POSE, "shoulder_lift": 50.0}, shoulder_lift=0.0)
        self.assertEqual(baseline.anchor["shoulder_lift"], POSE["shoulder_lift"])

    def test_gaps_are_hand_offset_against_arm_offset(self):
        baseline = Baseline(True)
        for _ in range(KEEP_SAMPLES):
            self.update(baseline, shoulder_lift=0.2)
        self.update(baseline, shoulder_lift=0.6)
        gap = baseline.state()["gaps"]["shoulder_lift"]
        self.assertAlmostEqual(gap["hand"], relative(0.6, 0.2))
        self.assertAlmostEqual(gap["robot"], 0.0)

    def test_joint_without_a_baseline_is_not_driven(self):
        baseline = Baseline(True)
        for _ in range(KEEP_SAMPLES):
            self.update(baseline, shoulder_lift=0.0)
        self.assertIsNone(baseline.hand_input("elbow_flex", 0.5))

    def test_ignores_joints_that_are_not_arm_joints(self):
        baseline = Baseline(True)
        self.assertEqual(set(HOME_JOINTS), set(POSE))
        for _ in range(KEEP_SAMPLES):
            self.update(baseline, gripper=1.0)
        self.assertEqual(baseline.hand, {})


class DeskCase(unittest.TestCase):
    def setUp(self):
        self.time = [10.0]
        limits = default_limits()
        for j in HOME_JOINTS:
            limits[j].update(speed=150.0, step=15.0)
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], limits=limits, sync=True)
        self.desk.arm.pose.update(POSE)
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]

    def sample(self, **inputs):
        self.time[0] += 0.065
        return self.desk.command(dict(op="sample", token=self.token, **{"height": 0.5, "opening": 0.5, **inputs}))

    def hold(self, count=KEEP_SAMPLES, **inputs):
        for _ in range(count):
            state = self.sample(**inputs)
        return state


class DeskBaselineTests(DeskCase):
    def test_arm_stays_while_the_baseline_is_taken(self):
        before = self.desk.arm.read()
        state = self.sample(lift=0.6, reach=-0.3, pan=0.8, flex=0.0, roll=0.4)
        self.assertEqual(self.desk.arm.read(), before)
        self.assertTrue(state["sync"]["waiting"])
        self.assertIn("基準", state["note"])

    def test_any_hand_pose_starts_from_the_current_arm_pose(self):
        hand = dict(lift=0.6, reach=-0.3, pan=0.8, flex=0.7, roll=0.4)
        state = self.hold(KEEP_SAMPLES + 20, **hand)
        self.assertFalse(state["sync"]["waiting"])
        self.assertIn("追従中", state["note"])
        for j, expected in POSE.items():
            self.assertAlmostEqual(self.desk.arm.read()[j], expected, places=3)

    def test_hand_offset_moves_the_arm_and_both_ends_are_reachable(self):
        self.hold(lift=0.6, reach=0.0)
        for _ in range(100):
            self.sample(lift=1.0, reach=-1.0)
        self.assertAlmostEqual(self.desk.arm.read()["shoulder_lift"], 100.0, places=3)
        self.assertAlmostEqual(self.desk.arm.read()["elbow_flex"], -100.0, places=3)
        for _ in range(100):
            self.sample(lift=-1.0, reach=1.0)
        self.assertAlmostEqual(self.desk.arm.read()["shoulder_lift"], -100.0, places=3)
        self.assertAlmostEqual(self.desk.arm.read()["elbow_flex"], 100.0, places=3)

    def test_small_hand_move_gives_a_small_arm_move(self):
        self.hold(lift=0.0)
        for _ in range(100):
            self.sample(lift=0.5)
        self.assertAlmostEqual(self.desk.arm.read()["shoulder_lift"], neutral_map(0.5, -100, 100, POSE["shoulder_lift"]),
                               places=3)

    def test_every_follow_takes_a_new_baseline(self):
        self.hold(lift=0.0)
        self.desk.command(dict(op="stop"))
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        self.assertTrue(self.sample(lift=0.9)["sync"]["waiting"])

    def test_gripper_also_waits(self):
        before = self.desk.arm.read()
        self.sample(lift=1.0)
        self.assertEqual(self.desk.arm.read()["gripper"], before["gripper"])

    def test_sync_is_none_when_not_following(self):
        self.desk.command(dict(op="stop"))
        self.assertIsNone(self.desk.state()["sync"])


class DeskFollowSelectionTests(DeskCase):
    ALL = dict(lift=0.0, reach=0.0, pan=0.0, flex=0.0, roll=0.0)

    def test_only_checked_joints_move(self):
        self.hold(follow=["shoulder_lift"], **self.ALL)
        before = self.desk.arm.read()
        for _ in range(60):
            self.sample(follow=["shoulder_lift"], lift=1.0, reach=-1.0, pan=1.0, flex=1.0, roll=1.0)
        after = self.desk.arm.read()
        self.assertAlmostEqual(after["shoulder_lift"], 100.0, places=3)
        for j in ("shoulder_pan", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
            self.assertEqual(after[j], before[j], j)

    def test_unchecked_gripper_stays(self):
        self.hold(follow=["shoulder_lift"], **self.ALL)
        before = self.desk.arm.read()["gripper"]
        self.sample(follow=["shoulder_lift"], opening=1.0, **self.ALL)
        self.assertEqual(self.desk.arm.read()["gripper"], before)

    def test_no_follow_key_means_all(self):
        self.hold(**self.ALL)
        for _ in range(60):
            self.sample(lift=1.0, reach=-1.0, pan=0.0, flex=0.0, roll=0.0)
        self.assertAlmostEqual(self.desk.arm.read()["shoulder_lift"], 100.0, places=3)
        self.assertAlmostEqual(self.desk.arm.read()["elbow_flex"], -100.0, places=3)

    def test_joint_turned_on_midway_starts_from_where_it_is(self):
        self.hold(follow=["shoulder_lift"], **self.ALL)
        for _ in range(20):                                   # 肘はOFFのまま、手だけ大きく動かす
            self.sample(follow=["shoulder_lift"], **{**self.ALL, "reach": -1.0})
        before = self.desk.arm.read()["elbow_flex"]
        self.sample(follow=["shoulder_lift", "elbow_flex"], **{**self.ALL, "reach": -1.0})
        self.assertAlmostEqual(self.desk.arm.read()["elbow_flex"], before, places=3)   # 飛ばない
        for _ in range(60):
            self.sample(follow=["shoulder_lift", "elbow_flex"], **{**self.ALL, "reach": 1.0})
        self.assertGreater(self.desk.arm.read()["elbow_flex"], before)

    def test_command_keeps_moving_ahead_of_a_stuck_joint(self):
        """摩擦で動かない関節でも、指令は実測位置の +1 サンプルぶんではなく、積み上がって差が開く。"""
        self.hold(follow=["shoulder_lift"], **self.ALL)
        written = []
        self.desk.arm.write = lambda pose: written.append(dict(pose))     # 書いても腕は動かない（pose は不変）
        self.desk.limits["shoulder_lift"].update(speed=11.0, step=5.0)
        for _ in range(40):
            self.sample(follow=["shoulder_lift"], **{**self.ALL, "lift": 1.0})
        actual = self.desk.arm.read()["shoulder_lift"]
        lead = written[-1]["shoulder_lift"] - actual
        self.assertGreater(lead, 5.0)                                     # 1 サンプルぶん(約 1.4)より大きい
        self.assertLessEqual(lead, 5.0 * 2 + 1e-6)                        # step(5%) = 10 raw を超えない

    def test_command_restarts_from_the_actual_position_after_a_joint_is_turned_off(self):
        self.hold(follow=["shoulder_lift"], **self.ALL)
        for _ in range(5):
            self.sample(follow=["shoulder_lift"], **{**self.ALL, "lift": 1.0})
        self.sample(follow=[], **self.ALL)
        self.assertNotIn("shoulder_lift", self.desk.cmd)

    def test_invalid_follow_is_rejected(self):
        for bad in ("shoulder_lift", ["nope"], [1]):
            with self.assertRaises(ValueError):
                self.sample(follow=bad, **self.ALL)


if __name__ == "__main__":
    unittest.main()
