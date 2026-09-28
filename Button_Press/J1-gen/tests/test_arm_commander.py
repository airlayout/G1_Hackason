"""ArmCommander の単体テスト（タスク1）。実機も MuJoCo も使わず、偽のバックエンドで確かめる。"""

from __future__ import annotations

import time
import unittest

import numpy as np

from common.arm import (
    ArmCommander,
    NoMotionError,
    StateTimeoutError,
    StopRequested,
    UnsafeTargetError,
    WorkspaceBox,
)
from common.arm.backend import ArmBackend
from common.arm.types import JointCommand, JointState
from common.config import load_config
from common.robot_model import NUM_MOTORS, RIGHT_ARM_IDX

LOWER = np.full(NUM_MOTORS, -2.0)
UPPER = np.full(NUM_MOTORS, 2.0)
ARM = np.array(RIGHT_ARM_IDX)


class FakeBackend(ArmBackend):
    """指令を記録するだけの偽物。follow=True なら実測値が指令に追従する（weight 込み）。"""

    def __init__(self, uses_weight: bool = True, follow: bool = True, dry_run: bool = False,
                 mode_machine: int = 5, motor_mode: int = 1) -> None:
        self.name = "fake"
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


if __name__ == "__main__":
    unittest.main()
