"""FK / IK、衝突の確認、押し込みの軌道（タスク2）。"""

from __future__ import annotations

import unittest

import numpy as np

from _env import needs
from common.collision import CollisionChecker
from common.config import load_config
from common.kinematics import ArmKinematics
from common.press_planner import PressPlanner, UnreachableError
from common.robot_model import load_model

REACHABLE = [((0.40, -0.20, 0.05), (1, 0, 0)), ((0.45, -0.15, 0.15), (1, 0, 0))]


@needs('mujoco', 'pin')
class TestKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_config("robot.yaml")
        cls.press = load_config("press.yaml")
        cls.arm = load_config("arm.yaml")
        cls.kin = {s: ArmKinematics(cls.robot, s, cls.press["ik"]) for s in ("left", "right")}

    def test_urdf_and_xml_fk_agree(self) -> None:
        """IK 用 URDF（Pinocchio）とシミュレーション用 XML（MuJoCo）で、手先の位置と向きが一致する。"""
        import mujoco

        m = load_model(self.robot)
        d = mujoco.MjData(m)
        pel = m.body("pelvis").id
        rng = np.random.default_rng(0)
        for side, kin in self.kin.items():
            ee = self.robot["end_effector"][side]
            b = m.body(ee["link"]).id
            for _ in range(100):
                q = rng.uniform(-0.8, 0.8, 29)
                p1, r1 = kin.fk(q)
                d.qpos[:] = q
                mujoco.mj_kinematics(m, d)
                rp = d.xmat[pel].reshape(3, 3)
                rb = d.xmat[b].reshape(3, 3)
                p2 = rp.T @ (d.xpos[b] + rb @ np.asarray(ee["offset"]) - d.xpos[pel])
                np.testing.assert_allclose(p1, p2, atol=1e-5)
                np.testing.assert_allclose(r1, rp.T @ rb, atol=1e-5)

    def test_ik_reaches_fk_of_random_pose(self) -> None:
        """ある姿勢の手先を目標にすると、別の初期値からでも IK で届く。"""
        kin = self.kin["right"]
        rng = np.random.default_rng(1)
        ok = 0
        for _ in range(20):
            q_goal = np.zeros(29)
            q_goal[kin.arm_idx] = rng.uniform(kin.lower[kin.arm_idx] * 0.4, kin.upper[kin.arm_idx] * 0.4)
            p, r = kin.fk(q_goal)
            res = kin.ik(p, np.zeros(29), r[:, 0])
            if res.success:
                ok += 1
                self.assertLess(res.pos_err, 0.0005)
                np.testing.assert_allclose(kin.fk_pos(res.q), p, atol=0.0005)
        self.assertGreaterEqual(ok, 16)

    def test_ik_rejects_far_target(self) -> None:
        res = self.kin["right"].ik(np.array([1.5, -0.2, 0.0]), np.zeros(29), None)
        self.assertFalse(res.success)
        self.assertTrue(res.reason)

    def test_ik_keeps_waist_and_other_arm(self) -> None:
        kin = self.kin["right"]
        q0 = np.zeros(29)
        q0[12:15] = [0.05, -0.02, 0.03]
        q0[15:22] = 0.1
        res = kin.ik(np.array([0.40, -0.20, 0.05]), q0, np.array([1.0, 0, 0]))
        self.assertTrue(res.success, res.reason)
        others = [i for i in range(29) if i not in kin.arm_idx]
        np.testing.assert_array_equal(res.q[others], q0[others])


@needs('mujoco')
class TestCollision(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cc = CollisionChecker(load_config("robot.yaml"), "right", 0.01)

    def q(self, **deg: float) -> np.ndarray:
        idx = {"sp": 22, "sr": 23, "sy": 24, "el": 25}
        q = np.zeros(29)
        for k, v in deg.items():
            q[idx[k]] = np.radians(v)
        return q

    def test_zero_pose_is_free(self) -> None:
        self.assertEqual(self.cc.contacts(self.q()), [])

    def test_arm_into_torso_detected(self) -> None:
        self.assertTrue(self.cc.contacts(self.q(sr=10)))
        hits = self.cc.contacts(self.q(sy=80))
        self.assertTrue(any("torso_link" in (h.body1, h.body2) for h in hits))

    def test_hand_box_added(self) -> None:
        """公式モデルのハンドには衝突形状が無いので、箱を足している。"""
        center, half = self.cc.hand_box
        self.assertGreater(center[0], 0.05)  # 手首より前
        self.assertTrue(np.all(half > 0.01))


@needs('mujoco', 'pin')
class TestPressPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_config("robot.yaml")
        cls.press = load_config("press.yaml")
        cls.arm = load_config("arm.yaml")
        cls.pl = PressPlanner.from_config(cls.robot, cls.press, cls.arm, "right")

    def test_plan_reachable(self) -> None:
        for tgt, d in REACHABLE:
            plan = self.pl.plan(np.zeros(29), np.array(tgt), np.array(d))
            q = np.zeros(29)
            q[self.pl.kin.arm_idx] = plan.press_in[-1]
            np.testing.assert_allclose(self.pl.kin.fk_pos(q), plan.end_point, atol=0.001)
            steps = np.abs(np.diff(np.array([plan.q_approach] + plan.press_in), axis=0)).max()
            self.assertLessEqual(steps, self.pl.max_joint_step + 1e-9)
            # 押し込みの直線は、押す方向に沿っている
            q[self.pl.kin.arm_idx] = plan.q_approach
            np.testing.assert_allclose(self.pl.kin.fk_pos(q), plan.approach_point, atol=0.001)
            self.assertEqual(len(plan.press_out), len(plan.press_in))

    def test_depth_is_capped(self) -> None:
        plan = self.pl.plan(np.zeros(29), np.array([0.40, -0.20, 0.05]), np.array([1.0, 0, 0]), depth=0.5)
        depth = float(np.dot(plan.end_point - plan.target_point, plan.push_dir))
        self.assertAlmostEqual(depth, self.press["press"]["max_press_depth_m"])

    def test_rejects_outside_workspace(self) -> None:
        with self.assertRaisesRegex(UnreachableError, "作業空間"):
            self.pl.plan(np.zeros(29), np.array([0.9, -0.2, 0.0]), np.array([1.0, 0, 0]))

    def test_rejects_collision(self) -> None:
        with self.assertRaisesRegex(UnreachableError, "ぶつかる"):
            self.pl.plan(np.zeros(29), np.array([0.30, 0.10, 0.0]), np.array([1.0, 0, 0]))

    def test_rejects_bad_input(self) -> None:
        with self.assertRaises(UnreachableError):
            self.pl.plan(np.zeros(29), np.array([np.nan, 0, 0]), np.array([1.0, 0, 0]))
        with self.assertRaises(UnreachableError):
            self.pl.plan(np.zeros(29), np.array([0.4, -0.2, 0.05]), np.zeros(3))


if __name__ == "__main__":
    unittest.main()
