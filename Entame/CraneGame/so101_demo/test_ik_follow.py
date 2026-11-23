import json
import unittest
from pathlib import Path

import reach_map
from ik_follow import CONTROLLED_JOINTS, IkFollow, hand_motion, near_home
from server import FALLBACK_CALIBRATION, Desk, MockArm

HOME = json.loads((Path(__file__).resolve().parent / "home.json").read_text())
UNITS = reach_map.joint_units(FALLBACK_CALIBRATION)


class IkFollowTests(unittest.TestCase):
    def test_starts_at_home_pose_then_eases_to_the_ik_target_without_jumping(self):
        ik = IkFollow(UNITS, HOME)
        want = ik.step(0.0, 0.0, 0.07)
        for joint in CONTROLLED_JOINTS:
            self.assertAlmostEqual(want[joint], HOME[joint], delta=1.0, msg=joint)
        previous = want
        for _ in range(60):
            want = ik.step(0.0, 0.0, 0.07)
            for joint in CONTROLLED_JOINTS:
                self.assertLess(abs(want[joint] - previous[joint]), 3.0, joint)    # 1 サンプル 0.07 秒での変化が小さい
            previous = want
        target = reach_map.target(0.0, 0.0, ik.anchor, UNITS, ik.safe)[1]
        for joint in CONTROLLED_JOINTS:
            self.assertAlmostEqual(want[joint], target[joint], delta=1.0, msg=joint)

    def test_full_reach_straightens_the_elbow_and_stays_in_the_envelope(self):
        ik = IkFollow(UNITS, HOME)
        for _ in range(60):
            want = ik.step(1.0, 0.0, 0.07)
        self.assertLess(want["elbow_flex"], -50)
        self.assertTrue(reach_map.inside_envelope({**HOME, **want}, UNITS))

    def test_near_home(self):
        self.assertTrue(near_home(HOME, HOME))
        self.assertFalse(near_home({**HOME, "elbow_flex": HOME["elbow_flex"] + 40}, HOME))

    def test_hand_motion_is_relative_to_the_base_and_height_flips_the_shoulder_input(self):
        base = {"elbow_flex": 0.2, "shoulder_lift": -0.3}
        self.assertEqual(hand_motion(base, base), (0.0, 0.0))
        reach, height = hand_motion(base, {"elbow_flex": 1.0, "shoulder_lift": -1.0})
        self.assertEqual(reach, 1.0)
        self.assertGreater(height, 0)


class DeskIkTests(unittest.TestCase):
    def start(self, ik_follow, pose=None, follow=None):
        self.time = [10.0]
        arm = MockArm()
        arm.pose.update(pose or HOME)
        self.desk = Desk(arm, clock=lambda: self.time[0], home=dict(HOME), sync=True, ik_follow=ik_follow)
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        self.follow = follow

    def sample(self, **inputs):
        self.time[0] += 0.065
        extra = {} if self.follow is None else dict(follow=self.follow)
        data = dict(height=0.5, opening=0.5, pan=0.0, lift=0.0, reach=0.0, flex=0.0, roll=0.0, **extra)
        data.update(inputs)
        self.desk.command(dict(op="sample", token=self.token, **data))
        return self.desk.arm.read()

    def settle_and_reach(self):
        for _ in range(8):                  # 基準を取る（手が止まっている）
            self.sample()
        for _ in range(80):
            pose = self.sample(reach=1.0)
        return pose

    def test_off_by_default_keeps_the_joint_wise_follow(self):
        self.start(False)
        self.settle_and_reach()
        self.assertIsNone(self.desk.ik)

    def test_on_reaches_with_a_straighter_elbow_than_home(self):
        self.start(True)
        pose = self.settle_and_reach()
        self.assertIsNotNone(self.desk.ik)
        self.assertLess(pose["elbow_flex"], HOME["elbow_flex"] - 20)

    def test_far_from_home_does_not_use_ik_and_says_why(self):
        self.start(True, pose={**HOME, "elbow_flex": HOME["elbow_flex"] + 40})
        self.settle_and_reach()
        self.assertIsNone(self.desk.ik)
        self.assertIn("IK 追従を使えません", self.desk.note)

    def test_needs_all_three_bending_joints_followed(self):
        self.start(True, follow=["gripper", "shoulder_lift", "elbow_flex"])
        self.settle_and_reach()
        self.assertIsNone(self.desk.ik)
        self.assertIn("IK 追従を使えません", self.desk.note)

    def test_unregistered_home_does_not_use_ik(self):
        self.start(True)
        self.desk.home = None
        self.settle_and_reach()
        self.assertIsNone(self.desk.ik)


if __name__ == "__main__":
    unittest.main()
