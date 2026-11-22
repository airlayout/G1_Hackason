import unittest

from home import HOME_JOINTS
from limits import default_limits, raw
from server import Desk, MockArm
from skeleton import (FEATURES, MAX_RESIDUAL, MOVE_TIMEOUT_S, POSE_NAMES, POSE_OFFSETS, Calibration, PoseMover,
                      calibration_poses, fit_joint, parse_features, wrap)

BASE = {"shoulder_pan": 0.0, "shoulder_lift": -10.0, "elbow_flex": 10.0, "wrist_flex": 0.0, "wrist_roll": 0.0}


def human_for(robot):
    """ロボットの姿勢に対応する、人の角度（テスト用の、ずれた比例関係）。"""
    return {FEATURES[j]: 0.9*robot[j] + 10.0 for j in HOME_JOINTS}


class WrapTests(unittest.TestCase):
    def test_wraps_into_half_turn(self):
        self.assertAlmostEqual(wrap(190), -170)
        self.assertAlmostEqual(wrap(-190), 170)
        self.assertAlmostEqual(wrap(30), 30)


class ParseFeaturesTests(unittest.TestCase):
    def test_accepts_partial_and_empty(self):
        self.assertEqual(parse_features({"pan": 10, "lift": 20.5}), {"pan": 10.0, "lift": 20.5})
        self.assertEqual(parse_features({}), {})

    def test_rejects_bad_input(self):
        for bad in (None, [1], {"nope": 1}, {"pan": "1"}, {"pan": float("nan")}, {"pan": 400}, {"pan": True}):
            with self.assertRaises(ValueError, msg=repr(bad)):
                parse_features(bad)


class CalibrationPosesTests(unittest.TestCase):
    def test_three_poses_built_from_the_base_and_inside_limits(self):
        limits = default_limits()
        poses = calibration_poses(BASE, limits)
        self.assertEqual(len(poses), len(POSE_NAMES))
        self.assertEqual(poses[0], BASE)
        self.assertAlmostEqual(poses[1]["shoulder_lift"], BASE["shoulder_lift"] + POSE_OFFSETS[1]["shoulder_lift"])

    def test_every_joint_takes_at_least_two_values(self):
        poses = calibration_poses(BASE, default_limits())
        for joint in HOME_JOINTS:
            self.assertGreaterEqual(len({round(p[joint], 3) for p in poses}), 2, joint)

    def test_clamped_to_the_limits(self):
        limits = default_limits()
        limits["shoulder_lift"].update(min=45.0, max=55.0)
        for pose in calibration_poses(BASE, limits):
            self.assertTrue(raw("shoulder_lift", 45.0) <= pose["shoulder_lift"] <= raw("shoulder_lift", 55.0))


class FitJointTests(unittest.TestCase):
    def test_exact_line_is_recovered(self):
        fit = fit_joint([(10.0, -20.0), (40.0, 10.0), (70.0, 40.0)])
        self.assertTrue(fit["ok"])
        self.assertAlmostEqual(fit["a"], 1.0)
        self.assertAlmostEqual(fit["resid"], 0.0)

    def test_negative_slope_is_allowed(self):
        self.assertAlmostEqual(fit_joint([(0.0, 30.0), (30.0, 0.0), (60.0, -30.0)])["a"], -1.0)

    def test_needs_two_points(self):
        self.assertFalse(fit_joint([(1.0, 2.0)])["ok"])
        self.assertFalse(fit_joint([])["ok"])

    def test_small_human_span_is_rejected(self):
        fit = fit_joint([(10.0, -20.0), (14.0, 10.0), (12.0, 40.0)])
        self.assertFalse(fit["ok"])
        self.assertIn("幅", fit["reason"])

    def test_robot_not_moving_is_rejected(self):
        self.assertFalse(fit_joint([(0.0, 5.0), (30.0, 5.0), (60.0, 5.0)])["ok"])

    def test_points_far_from_a_line_are_rejected(self):
        fit = fit_joint([(0.0, 0.0), (30.0, 60.0), (60.0, 0.0)])
        self.assertGreater(fit["resid"], MAX_RESIDUAL)
        self.assertFalse(fit["ok"])

    def test_angles_across_the_wrap_are_one_line(self):
        fit = fit_joint([(170.0, 0.0), (-160.0, 30.0), (-130.0, 60.0)])      # 170 → 200 → 230（度）
        self.assertTrue(fit["ok"])
        self.assertAlmostEqual(fit["a"], 1.0)


class CalibrationTests(unittest.TestCase):
    def test_ready_after_enough_captures_and_maps_back(self):
        calibration = Calibration()
        self.assertFalse(calibration.ready)
        poses = calibration_poses(BASE, default_limits())
        for i, pose in enumerate(poses):
            calibration.add(i, pose, human_for(pose))
        self.assertTrue(calibration.ready)
        for pose in poses:
            for joint in HOME_JOINTS:
                if calibration.fits[joint]["ok"]:
                    self.assertAlmostEqual(calibration.target(joint, human_for(pose)[FEATURES[joint]]), pose[joint], places=6)

    def test_joint_that_is_not_usable_has_no_target(self):
        calibration = Calibration()
        self.assertIsNone(calibration.target("shoulder_lift", 10.0))

    def test_missing_angle_leaves_that_joint_unusable(self):
        calibration = Calibration()
        for i, pose in enumerate(calibration_poses(BASE, default_limits())):
            human = human_for(pose)
            del human["roll"]
            calibration.add(i, pose, human)
        self.assertFalse(calibration.fits["wrist_roll"]["ok"])
        self.assertTrue(calibration.fits["shoulder_lift"]["ok"])

    def test_clear(self):
        calibration = Calibration()
        calibration.add(0, BASE, human_for(BASE))
        calibration.clear()
        self.assertEqual(calibration.captures, {})


class PoseMoverTests(unittest.TestCase):
    @staticmethod
    def direct(joint, actual, target):
        return target

    def test_arrival_sets_at_and_stops(self):
        mover = PoseMover()
        mover.go(1, {"shoulder_lift": 10.0}, 0.0)
        self.assertEqual(mover.step({"shoulder_lift": 0.0}, 0.1, self.direct), {"shoulder_lift": 10.0})
        self.assertIsNone(mover.step({"shoulder_lift": 9.5}, 0.2, self.direct))
        self.assertEqual(mover.at, 1)
        self.assertFalse(mover.active)

    def test_timeout_stops_without_arriving(self):
        mover = PoseMover()
        mover.go(2, {"shoulder_lift": 10.0}, 0.0)
        self.assertIsNone(mover.step({"shoulder_lift": 0.0}, MOVE_TIMEOUT_S + 1, self.direct))
        self.assertIsNone(mover.at)
        self.assertFalse(mover.active)

    def test_idle_does_nothing(self):
        self.assertIsNone(PoseMover().step({}, 0.0, self.direct))


class DeskSkeletonTests(unittest.TestCase):
    def setUp(self):
        self.time = [10.0]
        limits = default_limits()
        for j in HOME_JOINTS:
            limits[j].update(speed=150.0, step=15.0)
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], limits=limits, home=dict(BASE))
        self.desk.arm.pose.update(BASE)

    def run_command(self, **data):
        self.time[0] += 0.05
        return self.desk.command(data)

    def settle(self):
        for _ in range(400):
            self.time[0] += 0.03
            self.desk.tick()
            if not self.desk.mover.active:
                return
        self.fail("ロボットが着かない")

    def capture_all(self):
        for i in range(len(POSE_NAMES)):
            self.run_command(op="skeleton_pose", index=i)
            self.settle()
            self.run_command(op="skeleton_capture", index=i, features=human_for(self.desk.arm.read()))

    def follow(self, **extra):
        self.capture_all()
        return self.run_command(op="skeleton_start", **extra)["token"]

    def test_robot_moves_to_each_pose_and_reports_arrival(self):
        state = self.run_command(op="skeleton_pose", index=1)
        self.assertTrue(state["skeleton"]["moving"])
        self.settle()
        pose = self.desk.arm.read()
        for joint, value in self.desk.poses_sk[1].items():
            self.assertAlmostEqual(pose[joint], value, delta=2.0)
        self.assertEqual(self.desk.state()["skeleton"]["at"], 1)
        self.assertIn(POSE_NAMES[1], self.desk.note)

    def test_first_pose_is_the_home_pose(self):
        self.run_command(op="skeleton_pose", index=0)
        self.assertEqual(self.desk.poses_sk[0], BASE)

    def test_capture_needs_the_robot_to_have_arrived(self):
        features = human_for(BASE)
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_capture", index=0, features=features)
        self.run_command(op="skeleton_pose", index=0)
        with self.assertRaises(ValueError):            # 動いている間
            self.run_command(op="skeleton_capture", index=0, features=features)
        self.settle()
        with self.assertRaises(ValueError):            # 別の姿勢
            self.run_command(op="skeleton_capture", index=1, features=features)
        with self.assertRaises(ValueError):            # 骨格なし
            self.run_command(op="skeleton_capture", index=0, features={})
        state = self.run_command(op="skeleton_capture", index=0, features=features)
        self.assertEqual(state["skeleton"]["captured"], [True, False, False])

    def test_slider_move_forgets_the_arrival(self):
        self.run_command(op="skeleton_pose", index=0)
        self.settle()
        self.run_command(op="move", joint="shoulder_lift", value=0)
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_capture", index=0, features=human_for(BASE))

    def test_calibration_is_ready_after_three_poses(self):
        self.capture_all()
        state = self.desk.state()["skeleton"]
        self.assertTrue(state["ready"])
        self.assertEqual(state["captured"], [True, True, True])
        self.assertTrue(all(f["ok"] for f in state["fits"].values()), state["fits"])

    def test_reset_clears_the_calibration(self):
        self.capture_all()
        state = self.run_command(op="skeleton_reset")["skeleton"]
        self.assertEqual(state["captured"], [False, False, False])
        self.assertFalse(state["ready"])
        self.assertIsNone(state["at"])

    def test_cannot_start_without_calibration(self):
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_start")

    def test_follow_moves_the_arm_to_the_mapped_pose(self):
        token = self.follow()
        target = self.desk.poses_sk[1]
        for _ in range(80):
            self.run_command(op="skeleton_sample", token=token, features=human_for(target))
        pose = self.desk.arm.read()
        for joint, value in target.items():
            self.assertAlmostEqual(pose[joint], value, delta=1.0, msg=joint)
        self.assertTrue(self.desk.state()["skeleton"]["following"])

    def test_unchecked_joint_and_missing_angle_hold_their_pose(self):
        token = self.follow()
        before = self.desk.arm.read()
        target = self.desk.poses_sk[1]
        features = human_for(target)
        del features["elbow"]
        for _ in range(80):
            self.run_command(op="skeleton_sample", token=token, features=features,
                             follow=["shoulder_lift", "elbow_flex", "wrist_flex"])
        after = self.desk.arm.read()
        self.assertAlmostEqual(after["shoulder_lift"], target["shoulder_lift"], delta=1.0)
        self.assertAlmostEqual(after["wrist_flex"], target["wrist_flex"], delta=1.0)
        for joint in ("elbow_flex", "shoulder_pan", "wrist_roll"):
            self.assertEqual(after[joint], before[joint], joint)

    def test_empty_features_keep_following_alive_and_hold(self):
        token = self.follow()
        before = self.desk.arm.read()
        self.run_command(op="skeleton_sample", token=token, features={})
        self.assertEqual(self.desk.arm.read(), before)
        self.assertIsNotNone(self.desk.token)

    def test_gripper_is_never_driven(self):
        token = self.follow()
        before = self.desk.arm.read()["gripper"]
        self.run_command(op="skeleton_sample", token=token, features=human_for(self.desk.poses_sk[2]), follow=["gripper"])
        self.assertEqual(self.desk.arm.read()["gripper"], before)

    def test_targets_are_clamped_to_the_joint_limits(self):
        token = self.follow()
        self.desk.limits["shoulder_lift"].update(min=45.0, max=55.0)
        for _ in range(80):
            self.run_command(op="skeleton_sample", token=token, features=human_for(self.desk.poses_sk[1]))
        lift = self.desk.arm.read()["shoulder_lift"]
        self.assertTrue(raw("shoulder_lift", 45.0) - 1e-6 <= lift <= raw("shoulder_lift", 55.0) + 1e-6)

    def test_sample_needs_the_token_and_skeleton_mode(self):
        token = self.follow()
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_sample", token="x", features={})
        with self.assertRaises(ValueError):            # 旧方式のサンプルは、骨格版の追従中は受け付けない
            self.run_command(op="sample", token=token, height=0.5, opening=0.5)
        self.run_command(op="stop")
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_sample", token=token, features={})

    def test_old_follow_does_not_accept_skeleton_samples(self):
        token = self.run_command(op="start", gripper_only=True)["token"]
        with self.assertRaises(ValueError):
            self.run_command(op="skeleton_sample", token=token, features={})

    def test_settings_are_locked_while_following(self):
        self.follow()
        for op in ("skeleton_reset", "skeleton_pose", "skeleton_capture", "skeleton_start"):
            with self.assertRaises(ValueError, msg=op):
                self.run_command(op=op, index=0, features={})

    def test_pose_and_start_are_refused_when_the_motors_are_not_on(self):
        self.capture_all()
        self.run_command(op="drive", mode="off")
        for op in ("skeleton_pose", "skeleton_start"):
            with self.assertRaises(ValueError, msg=op):
                self.run_command(op=op, index=0)

    def test_stop_halts_the_move(self):
        self.run_command(op="skeleton_pose", index=1)
        self.run_command(op="stop")
        self.assertFalse(self.desk.mover.active)

    def test_bad_pose_index_is_rejected(self):
        for bad in (-1, 3, "0", 1.0, None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                self.run_command(op="skeleton_pose", index=bad)


if __name__ == "__main__":
    unittest.main()
