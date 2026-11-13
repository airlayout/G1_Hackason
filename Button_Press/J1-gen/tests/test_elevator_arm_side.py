"""エレベーターの呼びボタンを左腕でも押す（configs/elevator.yaml の press.arm、common/elevator_press.py）。

設定の腕ごとの値（IK の初期姿勢、指の傾き、経由点、勧める立ち位置）は右腕の値で書き、左腕では左右を反転して使う。
G1 は左右対称なので、ボタンの位置と立った姿勢を左右反転すると、左腕の計画は右腕の計画の鏡像になるはず（2026-11-04）。
"""

from __future__ import annotations

import copy
import unittest

import numpy as np

from _env import needs
from common.config import load_config
from common.elevator_buttons import Button3D, CallButtons
from common.robot_model import LEFT_ARM_IDX, RIGHT_ARM_IDX, mirror_arm_q

# ラボの車いす用の ▲▼ の高さ（pelvis から。sim/elevator_stand_map.py と同じ）
Z_UP, Z_DOWN, DIAM = 0.21, 0.14, 0.035
# 10/21 の実機の立った姿勢（sim/reach_map.py の REAL_STAND_1021）の右腕と腰。左腕は右腕の鏡像にして、左右対称の姿勢にする
STAND_RIGHT_ARM = np.array([0.286950, -0.125894, 0.024520, 0.982035, -0.096293, 0.058938, -0.020649])
FLIP_Y = np.array([1.0, -1.0, 1.0])


def symmetric_stand() -> np.ndarray:
    q = np.zeros(29)
    q[list(RIGHT_ARM_IDX)] = STAND_RIGHT_ARM
    q[list(LEFT_ARM_IDX)] = mirror_arm_q(STAND_RIGHT_ARM)
    return q


def make_call(x: float, y: float, z_up: float = Z_UP, z_down: float = Z_DOWN) -> CallButtons:
    up = Button3D(np.array([x, y, z_up]), DIAM, "up", (0, 0, 1, 1), 100, "up")
    down = Button3D(np.array([x, y, z_down]), DIAM, "down", (0, 0, 1, 1), 100, "down")
    return CallButtons(up, down, np.array([1.0, 0.0, 0.0]))


class TestMirrorArmQ(unittest.TestCase):
    def test_mirror_twice_is_identity(self) -> None:
        q = np.arange(7, dtype=float) / 10
        np.testing.assert_allclose(mirror_arm_q(mirror_arm_q(q)), q)
        np.testing.assert_allclose(mirror_arm_q(q)[[0, 3, 5]], q[[0, 3, 5]])  # pitch と肘は同じ

    def test_wrong_size(self) -> None:
        with self.assertRaises(ValueError):
            mirror_arm_q(np.zeros(6))


@needs('mujoco', 'pin')
class TestMirrorFk(unittest.TestCase):
    def test_mirrored_left_arm_reaches_the_mirrored_point(self) -> None:
        """右腕の角度の roll と yaw の符号を変えて左腕に入れると、指先は y を反転した位置に来る。"""
        from common.kinematics import ArmKinematics

        robot, ik = load_config("robot.yaml"), load_config("press.yaml")["ik"]
        right, left = ArmKinematics(robot, "right", ik), ArmKinematics(robot, "left", ik)
        rng = np.random.default_rng(0)
        for _ in range(30):
            q = np.zeros(29)
            q_arm = rng.uniform(-0.8, 0.8, 7)
            q[list(RIGHT_ARM_IDX)] = q_arm
            q[list(LEFT_ARM_IDX)] = mirror_arm_q(q_arm)
            np.testing.assert_allclose(left.fk_pos(q), right.fk_pos(q) * FLIP_Y, atol=1e-4)  # 公式モデルの左右の差は 0.01 mm ほど


@needs('mujoco', 'pin')
class TestElevatorLeftArm(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfgs = tuple(load_config(n) for n in ("robot.yaml", "press.yaml", "arm.yaml", "elevator.yaml"))

    def controller(self, side: str | None, button_set: str):
        from common.elevator_press import ElevatorPressController

        robot, press, arm, elev = self.cfgs
        e = copy.deepcopy(elev)
        e["geometry"]["button_set"] = button_set
        if side is None:
            e["press"].pop("arm", None)
        else:
            e["press"]["arm"] = side
        return ElevatorPressController.from_config(robot, press, arm, e)

    def plan(self, side: str, button_set: str, target: str, call: CallButtons, q: np.ndarray):
        ctl = self.controller(side, button_set)
        ctl.reset(target, 0.02, np.full(17, 60.0))
        ctl._plan(q, call)
        return ctl

    def test_default_is_right(self) -> None:
        """press.arm が無ければ右腕（今までどおり）。"""
        self.assertEqual(self.controller(None, "general").side, "right")
        self.assertEqual(self.controller("left", "general").kin.arm_idx.tolist(), list(LEFT_ARM_IDX))

    def test_unknown_arm_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.controller("both", "general")

    def test_left_plan_is_the_mirror_of_the_right_plan(self) -> None:
        """ボタンを左右反転した位置に置くと、左腕の計画（手前の姿勢・経由の姿勢・指の向き）は右腕の鏡像になる。
        wheelchair（指をまっすぐ・指の並びを縛る）、general（指を斜めに傾ける。正面の ▲ は経由点を通る）の両方。"""
        q0 = symmetric_stand()
        cases = [("wheelchair", 0.48, -0.14), ("general", 0.34, 0.0), ("general", 0.34, -0.04)]
        vias = 0
        for button_set, x, y in cases:
            for target in ("up", "down"):
                with self.subTest(button_set=button_set, x=x, y=y, target=target):
                    r = self.plan("right", button_set, target, make_call(x, y), q0).plan_
                    left = self.plan("left", button_set, target, make_call(x, -y), q0)
                    l = left.plan_
                    np.testing.assert_allclose(l.approach_point, r.approach_point * FLIP_Y, atol=1e-9)
                    np.testing.assert_allclose(l.finger_axis, r.finger_axis * FLIP_Y, atol=1e-6)
                    np.testing.assert_allclose(l.q_approach, mirror_arm_q(r.q_approach), atol=1e-3)
                    self.assertEqual(l.q_via is None, r.q_via is None)
                    if r.q_via is not None:
                        vias += 1
                        np.testing.assert_allclose(l.q_via, mirror_arm_q(r.q_via), atol=1e-3)
                    # 動くのは左腕だけ（右腕は立った姿勢のまま）
                    self.assertEqual(left.q_ik[list(RIGHT_ARM_IDX)].tolist(), q0[list(RIGHT_ARM_IDX)].tolist())
        self.assertGreater(vias, 0, "経由点を通る場合も確かめる")

    def test_left_via_points_are_on_the_left(self) -> None:
        """経由点は、押す腕の側（左腕なら左、y が大きい側）にずらす。"""
        ctl = self.controller("left", "general")
        touch, n = np.array([0.34, 0.0, 0.45]), np.array([1.0, 0.0, 0.0])
        for p in ctl._via_points(touch, n):
            self.assertGreater(p[1], touch[1])
        for p in self.controller("right", "general")._via_points(touch, n):
            self.assertLess(p[1], touch[1])

    def test_stand_advice_is_mirrored(self) -> None:
        """勧める立ち位置（wheelchair はボタンが体の右 14 cm）は、左腕では体の左 14 cm。"""
        left = self.controller("left", "wheelchair")
        right = self.controller("right", "wheelchair")
        self.assertEqual(left._stand_advice(make_call(0.48, 0.14)), "")
        self.assertEqual(right._stand_advice(make_call(0.48, -0.14)), "")
        advice = left._stand_advice(make_call(0.48, -0.14))
        self.assertIn("右へ 26 cm 以上寄る", advice)  # ボタンを体の左に持ってくるには、ロボットが右へ動く
        self.assertIn("体の中心から左 14±2 cm", advice)

    def test_left_arm_plans_from_the_real_stand(self) -> None:
        """実機の立った姿勢（10/21。左右は少し非対称）からも、左腕で車いす用の ▲▼ の計画が立つ（勧める立ち位置）。"""
        q = symmetric_stand()
        q[list(LEFT_ARM_IDX)] = [0.291049, 0.129657, 0.015711, 0.981556, 0.087161, 0.076807, 0.015232]
        for target in ("up", "down"):
            with self.subTest(target=target):
                ctl = self.plan("left", "wheelchair", target, make_call(0.48, 0.14), q)
                self.assertGreater(ctl.plan_.approach_point[1], 0.0)


@needs('mujoco', 'pin')
class TestElevatorAutoArm(unittest.TestCase):
    """press.arm: auto。ボタンを見つけたあと、計画が立つ腕を選ぶ（体の左寄りなら左腕から試す）。2026-11-13。
    開始の姿勢は 10/21 の実機の立った姿勢（地図 sim/elevator_stand_map.py と同じ）。"""

    @classmethod
    def setUpClass(cls) -> None:
        import sys

        from common.config import FEATURE_DIR

        sys.path.insert(0, str(FEATURE_DIR / "sim"))
        from reach_map import REAL_STAND_1021

        cls.q0 = REAL_STAND_1021.copy()
        cls.cfgs = tuple(load_config(n) for n in ("robot.yaml", "press.yaml", "arm.yaml", "elevator.yaml"))

    def controller(self, side: str | None, button_set: str):
        from common.elevator_press import ElevatorPressController

        robot, press, arm, elev = self.cfgs
        e = copy.deepcopy(elev)
        e["geometry"]["button_set"] = button_set
        if side is None:
            e["press"].pop("arm", None)
        else:
            e["press"]["arm"] = side
        return ElevatorPressController.from_config(robot, press, arm, e)

    def test_fixed_arm_keeps_one_planner(self) -> None:
        """right / left / 無しは今までどおり（計画器は 1 つ。reset しても腕は変わらない）。"""
        for side, want in (("right", "right"), ("left", "left"), (None, "right")):
            with self.subTest(side=side):
                ctl = self.controller(side, "wheelchair")
                self.assertFalse(ctl.auto)
                self.assertEqual(list(ctl.planners), [want])
                ctl.reset("down", 0.02, np.full(17, 60.0))
                self.assertEqual(ctl.side, want)

    def test_auto_has_both_planners_and_defaults_to_right(self) -> None:
        ctl = self.controller("auto", "wheelchair")
        self.assertTrue(ctl.auto)
        self.assertEqual(sorted(ctl.planners), ["left", "right"])
        self.assertEqual(ctl.side, "right")  # 腕が決まる前（look より前）は右腕
        self.assertEqual(ctl.kin.arm_idx.tolist(), list(RIGHT_ARM_IDX))

    def test_unknown_arm_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.controller("both", "wheelchair")

    def test_picks_the_arm_on_the_button_side(self) -> None:
        """車いす用の組で、ボタンが体の右 14 cm（柱まで 40 cm）なら右腕、左 14 cm なら左腕。"""
        for y, want in ((-0.14, "right"), (0.14, "left")):
            for target in ("down", "up"):
                with self.subTest(y=y, target=target):
                    ctl = self.controller("auto", "wheelchair")
                    self.assertIsNone(ctl.can_plan(self.q0, make_call(0.40, y), target))
                    self.assertEqual(ctl.side, want)
                    self.assertEqual(ctl.kin.arm_idx.tolist(), list(RIGHT_ARM_IDX if want == "right" else LEFT_ARM_IDX))
                    self.assertEqual(ctl.sgn, 1.0 if want == "right" else -1.0)
                    self.assertEqual(ctl.force.kin.side, want)  # 押す力の推定も選んだ腕のもの
                    self.assertIn("押す腕を選んだ: " + want, ctl.log[-1].message)

    def test_switches_to_the_other_arm_when_the_first_fails(self) -> None:
        """一般用で柱まで 44 cm・▲・体の右 12 cm は、先に試す右腕では立たず、左腕で立つ（地図 gen_auto の配置）。逆も同じ。"""
        for y, first, want in ((-0.12, "right", "left"), (0.12, "left", "right")):
            with self.subTest(y=y):
                call = make_call(0.44, y)
                fixed = self.controller(first, "general")
                self.assertIsNotNone(fixed.can_plan(self.q0, call, "up"))  # 先に試す腕だけでは立たない
                ctl = self.controller("auto", "general")
                self.assertIsNone(ctl.can_plan(self.q0, call, "up"))
                self.assertEqual(ctl.side, want)

    def test_both_fail_reports_both_arms(self) -> None:
        """どちらも立たない配置（車いす用、柱まで 52 cm・体の右 28 cm の ▼）では、理由に両方の腕が入り、
        腕は先に試した腕に戻る（立つ位置の助言をその腕で出す）。"""
        ctl = self.controller("auto", "wheelchair")
        why = ctl.can_plan(self.q0, make_call(0.52, -0.28), "down")
        self.assertIsNotNone(why)
        self.assertIn("right 腕:", why)
        self.assertIn("left 腕:", why)
        self.assertEqual(ctl.side, "right")
        self.assertEqual(ctl.sgn, 1.0)
        why = ctl.can_plan(self.q0, make_call(0.52, 0.28), "down")
        self.assertIsNotNone(why)
        self.assertEqual(ctl.side, "left")  # 左寄りなら先に左腕
        self.assertEqual(ctl.sgn, -1.0)

    def test_reset_chooses_again(self) -> None:
        """左腕を選んだあと reset すると右腕に戻り、右寄りのボタンでは右腕を選び直す。"""
        ctl = self.controller("auto", "wheelchair")
        ctl.reset("down", 0.02, np.full(17, 60.0))
        ctl._plan_arm(self.q0, make_call(0.40, 0.14))
        self.assertEqual(ctl.side, "left")
        ctl.reset("down", 0.02, np.full(17, 60.0))
        self.assertEqual(ctl.side, "right")
        self.assertIsNone(ctl._hand_vertices)  # 手のメッシュは選んだ腕のものだけを読む
        ctl._plan_arm(self.q0, make_call(0.40, -0.14))
        self.assertEqual(ctl.side, "right")
        self.assertIsNone(ctl.can_plan(self.q0, make_call(0.40, 0.14), "up"))  # can_plan も reset し直して選び直す
        self.assertEqual(ctl.side, "left")

    def test_hand_mesh_follows_the_arm(self) -> None:
        """手のメッシュは腕ごとに読み、腕を切り替えても取り違えない。"""
        ctl = self.controller("auto", "wheelchair")
        K = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 180.0], [0.0, 0.0, 1.0]])
        verts = {}
        for side in ("right", "left"):
            ctl._use_side(side)
            ctl._hand_circles(self.q0, K)
            verts[side] = ctl._hand_vertices
        self.assertFalse(np.array_equal(verts["right"], verts["left"]))
        ctl._use_side("right")
        self.assertIs(ctl._hand_vertices, verts["right"])


@needs("pin")
class TestStandMapWorker(unittest.TestCase):
    """立つ位置の地図（sim/elevator_stand_map.py）の準備で、押す腕が設定に渡る。2026-11-10 に、引数の腕（"right"）を
    arm.yaml の中身で上書きしてしまい、地図が起動しなかった。"""

    def test_init_worker_uses_the_arm(self) -> None:
        import sys

        from common.config import FEATURE_DIR

        sys.path.insert(0, str(FEATURE_DIR / "sim"))
        import elevator_stand_map as m

        for side in ("right", "left"):
            with self.subTest(side=side):
                m.init_worker("wheelchair", side)
                self.assertEqual(m._G["ctl"].side, side)


@needs("pin")
class TestBentWristSeed(unittest.TestCase):
    """車いす用は、今の初期姿勢で計画が立たないとき、手首を曲げた初期姿勢（press.by_button_set.wheelchair の
    ik_seed_right_arm_more）でも試す。ボタンが体の中央寄り（柱まで 40 cm・左右 0）でも、指の並びを縛ったまま立つ（2026-11-10）。"""

    def test_center_button_plans_with_bent_wrist(self) -> None:
        import sys

        from common.config import FEATURE_DIR

        sys.path.insert(0, str(FEATURE_DIR / "sim"))
        import elevator_stand_map as m
        from reach_map import REAL_STAND_1021

        m.init_worker("wheelchair", "right")
        ctl = m._G["ctl"]
        for target in ("down", "up"):
            with self.subTest(target=target):
                ctl.reset(target, 0.02, np.full(17, 60.0))
                ctl._plan(REAL_STAND_1021, m.make_call(0.40, 0.0, 0.0))
                self.assertIn("初期姿勢 2 番目", ctl.log[-1].message)
                self.assertIsNotNone(ctl.plan_.row_normal)  # 指の並びは縛ったまま（11/1 の小指で押すのに戻らない）
                self.assertLess(ctl.plan_.q_approach[6], -0.9)  # 手首 yaw を大きく曲げている
        # 今までどおり押せる位置（勧める立ち位置）は、今の初期姿勢のまま
        ctl.reset("down", 0.02, np.full(17, 60.0))
        ctl._plan(REAL_STAND_1021, m.make_call(0.48, -0.14, 0.0))
        self.assertNotIn("初期姿勢 2 番目", ctl.log[-1].message)


if __name__ == "__main__":
    unittest.main()
