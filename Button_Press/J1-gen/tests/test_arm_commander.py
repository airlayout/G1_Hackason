"""ArmCommander の単体テスト（タスク1）。実機も MuJoCo も使わず、偽のバックエンドで確かめる。"""

from __future__ import annotations

import time
import unittest

import numpy as np

from _env import needs
from common.arm import (
    ArmCommander,
    NoMotionError,
    StateTimeoutError,
    StopRequested,
    UnsafeTargetError,
    WaistDeviationError,
    WorkspaceBox,
)
from common.arm.gravity import GravityModel
from common.arm.backend import ArmBackend
from common.arm.types import JointCommand, JointState
from common.config import load_config
from common.robot_model import NUM_MOTORS, RIGHT_ARM_IDX, WAIST_IDX

LOWER = np.full(NUM_MOTORS, -2.0)
UPPER = np.full(NUM_MOTORS, 2.0)
ARM = np.array(RIGHT_ARM_IDX)


class FakeBackend(ArmBackend):
    """指令を記録するだけの偽物。follow=True なら実測値が指令に追従する（weight 込み）。"""

    def __init__(self, uses_weight: bool = True, follow: bool = True, dry_run: bool = False,
                 mode_machine: int = 5, motor_mode: int = 1, needs_support_check: bool = False) -> None:
        self.name = "fake"
        self.needs_support_check = needs_support_check
        self.uses_weight = uses_weight
        self.dry_run = dry_run
        self.follow = follow
        self.q = np.linspace(-0.3, 0.3, NUM_MOTORS)
        self.q0 = self.q.copy()
        self.mode_machine = mode_machine
        self.motor_mode = motor_mode
        self.sent: list[JointCommand] = []
        self.stale = False

    def open(self) -> None:
        pass

    def read_state(self) -> JointState:
        stamp = time.monotonic() - (10.0 if self.stale else 0.0)
        return JointState(self.q.copy(), np.zeros(NUM_MOTORS), np.full(NUM_MOTORS, self.motor_mode),
                          self.mode_machine, stamp)

    def send(self, cmd: JointCommand) -> None:
        self.sent.append(cmd.copy())
        if self.follow and not self.dry_run:
            w = cmd.weight if self.uses_weight else 1.0
            active = cmd.kp > 0
            self.q[active] = w * cmd.q[active] + (1 - w) * self.q0[active]

    def tick(self, dt: float) -> None:
        pass

    def close(self) -> None:
        pass


def make(backend: FakeBackend, **kw: object) -> ArmCommander:
    cfg = load_config("arm.yaml")
    cfg["arm"] = "right"
    return ArmCommander(backend, cfg, LOWER, UPPER, **kw)  # type: ignore[arg-type]


class TestStart(unittest.TestCase):
    def test_weight_ramps_up_while_holding_measured_pose(self) -> None:
        be = FakeBackend()
        arm = make(be)
        arm.start()
        weights = [c.weight for c in be.sent]
        n = len(weights)
        self.assertEqual(n, int(np.ceil(2.0 * 50)))  # weight_ramp_s × control_hz
        self.assertTrue(np.all(np.diff(weights) > 0))
        self.assertLessEqual(max(np.diff(weights)), 1.0 / n + 1e-12)
        self.assertAlmostEqual(weights[-1], 1.0)
        for c in be.sent:  # 上げている間、腕の目標は開始時の実測値のまま
            np.testing.assert_allclose(c.q[ARM], be.q0[ARM])
        arm.safe_stop()

    def test_refuse_when_motor_disabled(self) -> None:
        be = FakeBackend(motor_mode=0)
        with self.assertRaisesRegex(RuntimeError, "ゼロトルク"):
            make(be).start()
        self.assertEqual(be.sent, [])

    def test_refuse_wrong_mode_machine(self) -> None:
        be = FakeBackend(mode_machine=2)
        with self.assertRaisesRegex(RuntimeError, "mode_machine"):
            make(be).start()
        self.assertEqual(be.sent, [])

    def test_lowcmd_commands_all_joints(self) -> None:
        be = FakeBackend(uses_weight=False)
        arm = make(be)
        self.assertEqual(arm.joints, list(range(NUM_MOTORS)))
        arm.start()
        self.assertTrue(np.all(be.sent[-1].kp > 0))
        np.testing.assert_allclose(be.sent[-1].q, be.q0)
        arm.safe_stop()

    def test_arm_sdk_does_not_command_legs(self) -> None:
        arm = make(FakeBackend())
        self.assertTrue(all(i >= 12 for i in arm.joints))


class TestMove(unittest.TestCase):
    def test_move_reaches_target_within_rate_limit(self) -> None:
        be = FakeBackend()
        with make(be) as arm:
            target = be.q0[ARM] + np.radians([-20, 0, 0, 15, 0, 0, 0])
            n0 = len(be.sent)
            arm.move_to(target)
        moves = np.array([c.q[ARM] for c in be.sent[n0:]])
        steps = np.abs(np.diff(moves, axis=0)).max()
        self.assertLessEqual(steps, arm.max_step + 1e-12)
        self.assertTrue(np.any(np.all(np.abs(moves - target) < 1e-9, axis=1)))
        self.assertAlmostEqual(be.sent[-1].weight, 0.0)  # 抜けたら weight は 0

    def test_out_of_limit_target_rejected_without_sending(self) -> None:
        be = FakeBackend()
        with self.assertRaises(UnsafeTargetError):
            with make(be) as arm:
                n0 = len(be.sent)
                bad = be.q0[ARM].copy()
                bad[0] = 1.99  # リミット 2.0 から余裕 0.05 の内側に入っていない
                try:
                    arm.move_to(bad)
                finally:
                    moved = [c for c in be.sent[n0:] if not np.allclose(c.q[ARM], be.q0[ARM])]
                    self.assertEqual(moved, [])

    def test_workspace_rejects(self) -> None:
        be = FakeBackend()
        box = WorkspaceBox(np.array([-1.0, -1, -1]), np.array([1.0, 1, 1]))
        # 偽の FK: 肩ピッチの角度をそのまま x 座標にする
        fk = lambda q: np.array([q[ARM[0]] * 10, 0.0, 0.0])  # noqa: E731
        with self.assertRaises(UnsafeTargetError):
            with make(be, fk=fk, workspace=box) as arm:
                t = be.q0[ARM].copy()
                t[0] = 0.5
                arm.move_to(t)

    def test_no_motion_detected_and_stopped_safely(self) -> None:
        be = FakeBackend(follow=False)
        with self.assertRaises(NoMotionError):
            with make(be) as arm:
                arm.move_to(be.q0[ARM] + np.radians([10, 0, 0, 0, 0, 0, 0]))
        self.assertAlmostEqual(be.sent[-1].weight, 0.0)

    def test_one_joint_pushed_back_is_not_no_motion(self) -> None:
        """押し当てで 1 つの関節だけ押し戻されても、腕全体として動いていれば「動いていない」にしない。"""
        be = FakeBackend()
        with make(be) as arm:
            q0 = be.q[ARM].copy()
            target = q0 + np.radians([10, 10, 10, 10, 0, 3, 0])
            arm.move_to(target)
            be.q[ARM[5]] = q0[5] - 0.01  # 手首ピッチだけ逆向きに押し戻された
            arm.check_motion(q0, q0, target)

    def test_dry_run_skips_motion_check(self) -> None:
        be = FakeBackend(dry_run=True)
        with make(be) as arm:
            arm.move_to(be.q0[ARM] + np.radians([10, 0, 0, 0, 0, 0, 0]))

    def test_exception_in_block_still_stops_safely(self) -> None:
        be = FakeBackend()
        with self.assertRaises(ZeroDivisionError):
            with make(be):
                1 / 0  # noqa: B018
        self.assertAlmostEqual(be.sent[-1].weight, 0.0)

    def test_stop_requested_is_swallowed(self) -> None:
        be = FakeBackend()
        with make(be) as arm:
            raise StopRequested("SIGINT")
        self.assertEqual(arm.stop_reason, "SIGINT")
        self.assertAlmostEqual(be.sent[-1].weight, 0.0)

    def test_stop_holds_current_pose(self) -> None:
        """安全終了では、古い目標ではなく今の実測姿勢を保つ。"""
        be = FakeBackend()
        with make(be) as arm:
            arm.move_to(be.q0[ARM] + np.radians([10, 0, 0, 0, 0, 0, 0]))
            be.q[ARM[0]] += 0.01  # 外から押されてずれた
            q_now = be.q[ARM].copy()
        np.testing.assert_allclose(be.sent[-1].q[ARM], q_now)

    def test_state_timeout(self) -> None:
        be = FakeBackend()
        with self.assertRaises(StateTimeoutError):
            with make(be) as arm:
                be.stale = True
                arm.move_to(be.q0[ARM] + 0.1)

    def test_confirm_q_aborts(self) -> None:
        be = FakeBackend()
        answers = iter(["", "q"])
        with make(be, confirm=True, input_fn=lambda _: next(answers)) as arm:
            arm.move_to(be.q0[ARM] + 0.1)
        self.assertIn("中止", arm.stop_reason or "")
        # move_to は送られていない（開始時の姿勢のまま）
        for c in be.sent:
            np.testing.assert_allclose(c.q[ARM], be.q0[ARM])

    def test_follow_rejects_big_step(self) -> None:
        be = FakeBackend()
        with self.assertRaises(UnsafeTargetError):
            with make(be) as arm:
                arm.follow([be.q0[ARM] + 0.5])



class TestWaistSupportGravity(unittest.TestCase):
    """タスク1の追加分: 腰のずれの監視、プランBの支持の確認、重力補償。"""

    def test_waist_deviation_stops(self) -> None:
        be = FakeBackend()
        with self.assertRaises(WaistDeviationError):
            with make(be) as arm:
                be.q[WAIST_IDX[2]] += 0.06  # 上限 0.05 rad を超えて腰ピッチがずれた
                arm.move_to(be.q0[ARM] + 0.05)
        self.assertAlmostEqual(be.sent[-1].weight, 0.0)

    def test_small_waist_deviation_is_ok(self) -> None:
        be = FakeBackend()
        with make(be) as arm:
            be.q[WAIST_IDX[2]] += 0.03
            arm.move_to(be.q0[ARM] + 0.05)

    def test_support_check_asked_for_plan_b(self) -> None:
        asked: list[str] = []

        def answer(prompt: str) -> str:
            asked.append(prompt)
            return "q"

        be = FakeBackend(uses_weight=False, needs_support_check=True)
        arm = make(be, input_fn=answer)
        # 開始処理（with に入る前）での中止なので、StopRequested がそのまま外に出る
        with self.assertRaises(StopRequested):
            with arm:
                pass
        self.assertEqual(len(asked), 1)
        self.assertIn("座った状態", asked[0])
        self.assertEqual(be.sent, [])  # 何も送っていない
        self.assertIn("支持", arm.stop_reason or "")

    def test_support_check_not_asked_for_plan_a(self) -> None:
        def never(prompt: str) -> str:
            raise AssertionError("聞かれないはず")

        with make(FakeBackend(), input_fn=never):
            pass

    @needs('mujoco')
    def test_gravity_tau_scaled_and_clipped(self) -> None:
        gm = GravityModel(load_config("robot.yaml"))
        taus = {}
        for scale in (0.0, 0.5, 1.0):
            cfg = load_config("arm.yaml")
            cfg["arm"] = "right"
            cfg["gravity_compensation"]["scale"] = scale
            be = FakeBackend()
            with ArmCommander(be, cfg, LOWER, UPPER, gravity=gm):
                pass
            taus[scale] = be.sent[0].tau.copy()
        self.assertTrue(np.all(taus[0.0] == 0.0))
        arms = list(range(15, 29))
        np.testing.assert_allclose(taus[0.5][arms], 0.5 * taus[1.0][arms], atol=1e-9)
        self.assertTrue(np.all(taus[1.0][:15] == 0.0))  # 脚・腰には送らない
        self.assertTrue(np.all(np.abs(taus[1.0]) <= 7.0))
        self.assertGreater(np.abs(taus[1.0][arms]).max(), 0.1)

    @needs('mujoco')
    def test_waist_gravity_only_in_plan_b(self) -> None:
        gm = GravityModel(load_config("robot.yaml"))
        for uses_weight in (True, False):
            cfg = load_config("arm.yaml")
            cfg["arm"] = "right"
            cfg["gravity_compensation"]["waist_scale"] = 1.0
            be = FakeBackend(uses_weight=uses_weight)
            with ArmCommander(be, cfg, LOWER, UPPER, gravity=gm):
                pass
            waist_tau = be.sent[0].tau[list(WAIST_IDX)]
            if uses_weight:
                self.assertTrue(np.all(waist_tau == 0.0))  # arm_sdk では送らない
            else:
                self.assertGreater(np.abs(waist_tau).max(), 1.0)
                self.assertTrue(np.all(np.abs(waist_tau) <= 15.0))

    def test_gravity_scale_validated(self) -> None:
        cfg = load_config("arm.yaml")
        cfg["gravity_compensation"]["scale"] = 1.5
        with self.assertRaises(ValueError):
            ArmCommander(FakeBackend(), cfg, LOWER, UPPER)
        cfg["gravity_compensation"]["scale"] = 0.5
        with self.assertRaises(ValueError):  # GravityModel が無い
            ArmCommander(FakeBackend(), cfg, LOWER, UPPER)


if __name__ == "__main__":
    unittest.main()
