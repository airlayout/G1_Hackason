"""腰 yaw を回して確かめる道具（common/waist_turn.py と real/waist_turn_check.py）の単体テスト。

実機にも DDS にもつながない。送る口は偽のバックエンドに差し替える（tests/test_arm_commander.py と同じ作り）。
"""

from __future__ import annotations

import io
import json
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

from common.arm.backend import ArmBackend
from common.arm.types import JointCommand, JointState
from common.body_motion import read_lowstate_csv
from common.config import FEATURE_DIR
from common.robot_model import NUM_MOTORS
from common.waist_turn import (
    ABS_MAX_DEG,
    GuardLimits,
    WaistGuard,
    WaistTurnPlan,
    WaistTurnRecorder,
    smooth_return_path,
    summary_text,
)

sys.path.insert(0, str(FEATURE_DIR / "real"))
import waist_turn_check as tool  # noqa: E402

YAW = 12
DT = 0.02
Q0 = np.linspace(-0.3, 0.3, NUM_MOTORS)
QUAT0 = np.array([1.0, 0.0, 0.0, 0.0])


def quat_rp(roll_deg: float, pitch_deg: float) -> np.ndarray:
    r, p = np.radians(roll_deg) / 2, np.radians(pitch_deg) / 2
    qr = np.array([np.cos(r), np.sin(r), 0, 0])
    qp = np.array([np.cos(p), 0, np.sin(p), 0])
    w1, x1, y1, z1 = qr
    w2, x2, y2, z2 = qp
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


class FakeBackend(ArmBackend):
    """送った指令を記録する偽物。実測は指令に追従する（weight 込み）。on_send で外から乱す。"""

    def __init__(self, dry_run: bool = False) -> None:
        self.name = "fake"
        self.uses_weight = True
        self.dry_run = dry_run
        self.q = Q0.copy()
        self.q0 = Q0.copy()
        self.quat = QUAT0.copy()
        self.sent: list[JointCommand] = []
        self.stale = False
        self.on_send = None  # (送った回数, backend) → None。実測を乱す

    def open(self) -> None:
        pass

    def read_state(self) -> JointState:
        stamp = time.monotonic() - (10.0 if self.stale else 0.0)
        return JointState(self.q.copy(), np.zeros(NUM_MOTORS), np.ones(NUM_MOTORS, dtype=int), 5, stamp,
                          imu_quat=self.quat.copy(), tau_est=np.zeros(NUM_MOTORS))

    def send(self, cmd: JointCommand) -> None:
        self.sent.append(cmd.copy())
        if not self.dry_run:
            active = cmd.kp > 0
            self.q[active] = cmd.weight * cmd.q[active] + (1 - cmd.weight) * self.q0[active]
        if self.on_send is not None:
            self.on_send(len(self.sent), self)

    def tick(self, dt: float) -> None:
        pass

    def close(self) -> None:
        pass


LIMITS = (np.full(NUM_MOTORS, -3.0), np.full(NUM_MOTORS, 3.0))


def parse(*argv: str):  # type: ignore[no-untyped-def]
    return tool.build_parser().parse_args(list(argv))


def run_tool(argv: list[str], backend: FakeBackend | None = None) -> tuple[int, str]:
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = tool.run(parse(*argv), backend=backend, input_fn=lambda _: "", lower_upper=LIMITS)
    return code, buf.getvalue()


class TestPlan(unittest.TestCase):
    def test_start_angles_and_return(self) -> None:
        plan = WaistTurnPlan(0.1, [5, -5], max_deg=5, speed_deg_s=10, hold_s=3)
        self.assertAlmostEqual(plan.at(0.0), 0.1)
        for k, deg in enumerate([5, -5]):
            hold = [s for s in plan.segments if s.kind == "hold" and s.index == k][0]
            self.assertAlmostEqual(plan.at((hold.t0 + hold.t1) / 2), 0.1 + np.radians(deg), places=9)
        self.assertAlmostEqual(plan.at(plan.duration), 0.1, places=9)
        self.assertAlmostEqual(plan.at(plan.duration + 10.0), 0.1, places=9)  # 終わったあとも始めの角度

    def test_step_never_exceeds_speed_limit(self) -> None:
        plan = WaistTurnPlan(0.0, [5, -5], speed_deg_s=10)
        ts = np.arange(0.0, plan.duration, DT)
        steps = np.abs(np.diff([plan.at(t) for t in ts]))
        peak = np.radians(1.5 * 10) * DT
        self.assertLessEqual(float(steps.max()), peak * 1.001)
        self.assertGreater(float(steps.max()), peak * 0.5)  # 動いてはいる

    def test_other_joints_unchanged(self) -> None:
        plan = WaistTurnPlan(float(Q0[YAW]), [5, -5])
        for t in (0.0, 2.0, 5.0, 9.0):
            q = plan.command29(Q0, t)
            idx = [i for i in range(NUM_MOTORS) if i != YAW]
            np.testing.assert_array_equal(q[idx], Q0[idx])

    def test_limits_raise(self) -> None:
        with self.assertRaises(ValueError):
            WaistTurnPlan(0.0, [10], max_deg=5)
        with self.assertRaises(ValueError):
            WaistTurnPlan(0.0, [5, -6], max_deg=5)
        with self.assertRaises(ValueError):
            WaistTurnPlan(0.0, [5], max_deg=ABS_MAX_DEG + 1)
        with self.assertRaises(ValueError):
            WaistTurnPlan(0.0, [5], speed_deg_s=14.0)  # 最大が 21°/s
        with self.assertRaises(ValueError):
            WaistTurnPlan(0.0, [])
        WaistTurnPlan(0.0, [20, -20], max_deg=20)  # 明示すれば通る


class TestGuard(unittest.TestCase):
    def guard(self) -> WaistGuard:
        return WaistGuard(Q0, QUAT0, GuardLimits())

    def test_quiet_is_ok(self) -> None:
        self.assertIsNone(self.guard().check(Q0, QUAT0, float(Q0[YAW]), 0.01))

    def test_each_condition_stops(self) -> None:
        g = self.guard()
        q = Q0.copy()
        q[YAW] += np.radians(4)
        self.assertEqual(g.check(q, QUAT0, float(Q0[YAW]), 0.01).kind, "track")
        self.assertEqual(g.check(Q0, quat_rp(4, 0), float(Q0[YAW]), 0.01).kind, "tilt")
        self.assertEqual(g.check(Q0, quat_rp(0, -4), float(Q0[YAW]), 0.01).kind, "tilt")
        for idx in (14, 22):  # 腰 pitch、右腕
            q = Q0.copy()
            q[idx] += np.radians(4)
            r = g.check(q, QUAT0, float(Q0[YAW]), 0.01)
            self.assertEqual(r.kind, "other")
            self.assertFalse(r.hold_in_place)
        r = g.check(Q0, QUAT0, float(Q0[YAW]), 0.5)
        self.assertEqual(r.kind, "stale")
        self.assertTrue(r.hold_in_place)

    def test_return_scale_is_looser(self) -> None:
        g = self.guard()
        q = Q0.copy()
        q[YAW] += np.radians(4)
        self.assertIsNotNone(g.check(q, QUAT0, float(Q0[YAW]), 0.0))
        self.assertIsNone(g.check(q, QUAT0, float(Q0[YAW]), 0.0, scale=2.0))

    def test_smooth_return_has_no_jump(self) -> None:
        path = smooth_return_path(np.radians(5), 0.0, DT)
        steps = np.abs(np.diff(np.concatenate([[np.radians(5)], path])))
        self.assertLessEqual(float(steps.max()), np.radians(5.0 * 1.5) * DT * 1.001)
        self.assertEqual(float(path[-1]), 0.0)


class TestSummary(unittest.TestCase):
    def test_numbers_from_fake_measurements(self) -> None:
        """実測 = 指令より 0.5° 遅れ + 小さい揺れ。指令との差の最大が 0.5° 前後、保つ間の平均が目標 − 0.5° 前後になる。"""
        plan = WaistTurnPlan(0.0, [5, -5], hold_s=3)
        rec = WaistTurnRecorder()
        rng = np.random.default_rng(0)
        for t in np.arange(0.0, plan.duration, DT):
            seg = plan.segment_at(t)
            cmd = plan.at(t)
            target = np.radians(5 if seg.index == 0 else -5)
            lag = 0.5 * np.sign(cmd) if seg.kind == "hold" else 0.0
            noise = rng.normal(0.0, 0.05)
            q = Q0.copy()
            q[YAW] = cmd - np.radians(lag + noise)
            rec.add(float(t), seg.kind, seg.index, cmd, q, quat_rp(0.4, 0.0) if seg.kind == "hold" else QUAT0)
        s = rec.summarize(0.0, [5, -5])
        a0, a1 = s["angles"]
        self.assertAlmostEqual(a0["hold_mean_err_deg"], -0.5, delta=0.05)
        self.assertAlmostEqual(a1["hold_mean_err_deg"], +0.5, delta=0.05)
        self.assertAlmostEqual(a0["hold_std_deg"], 0.05, delta=0.02)
        self.assertAlmostEqual(a0["max_track_err_deg"], 0.5, delta=0.3)
        self.assertAlmostEqual(a0["max_tilt_change_deg"], 0.4, delta=0.02)
        self.assertAlmostEqual(s["return_diff_deg"], 0.0, delta=0.1)
        text = summary_text(s)
        self.assertIn("+5.0", text)
        self.assertIn("-5.0", text)
        json.dumps(s)


class TestTool(unittest.TestCase):
    def test_dry_run_sends_nothing(self) -> None:
        be = FakeBackend(dry_run=True)
        code, out = run_tool(["--skip-prechecks"], be)
        self.assertEqual(code, 0, out)
        self.assertEqual(be.sent, [])
        self.assertIn("kp 60 / kd 1.5", out)  # 既定の PD を表示する
        self.assertIn("dry-run", out)

    def test_no_robot_prints_plan_only(self) -> None:
        code, out = run_tool(["--no-robot"])
        self.assertEqual(code, 0, out)
        self.assertIn("hold0", out)

    def test_angle_above_max_deg_is_refused(self) -> None:
        be = FakeBackend(dry_run=True)
        code, out = run_tool(["--angles", "10"], be)
        self.assertEqual(code, 2, out)
        self.assertIn("max_deg", out)
        self.assertEqual(be.sent, [])
        code, out = run_tool(["--angles", "10", "--max-deg", "10", "--skip-prechecks"], FakeBackend(dry_run=True))
        self.assertEqual(code, 0, out)
        code, out = run_tool(["--angles", "25", "--max-deg", "25"], be)
        self.assertEqual(code, 2, out)

    def test_waist_gains_are_shown_and_bounded(self) -> None:
        code, out = run_tool(["--no-robot", "--waist-kp", "300", "--waist-kd", "5"])
        self.assertEqual(code, 0)
        self.assertIn("kp 300 / kd 5（指定）", out)
        code, _ = run_tool(["--no-robot", "--waist-kp", "400"])
        self.assertEqual(code, 2)

    def test_execute_completes_and_returns(self) -> None:
        be = FakeBackend()
        with tempfile.TemporaryDirectory() as d:
            code, out = run_tool(["--execute", "--no-confirm", "--skip-prechecks", "--hold-s", "1", "--out", d], be)
            self.assertEqual(code, 0, out)
            res = json.loads((Path(d) / "summary.json").read_text())
            self.assertTrue(res["completed"])
            self.assertEqual(res["waist_pd"], {"kp": 60.0, "kd": 1.5})
            # 記録の CSV は、既存の body_log と同じ読み方で読める
            t, q, imu, labels = read_lowstate_csv(Path(d) / "body_log.csv")
            self.assertIn("hold0", labels)
            self.assertAlmostEqual(float(np.degrees(q[:, YAW].max() - q[0, YAW])), 5.0, delta=0.01)
        # 送った指令: 腰 yaw 以外の関節の指令は変わらず、腰 PD は 60/1.5、最後は weight 0
        cmds = [c for c in be.sent]
        for c in cmds:
            idx = [i for i in range(13, 29)]
            np.testing.assert_allclose(c.q[idx], Q0[idx])
            self.assertEqual((c.kp[YAW], c.kd[YAW]), (60.0, 1.5))
        self.assertEqual(cmds[-1].weight, 0.0)
        steps = np.abs(np.diff([c.q[YAW] for c in cmds]))
        self.assertLessEqual(float(steps.max()), np.radians(15) * DT * 1.001)
        self.assertAlmostEqual(cmds[-1].q[YAW], Q0[YAW], places=6)  # 始めの角度に戻って終わる

    def trip_run(self, disturb) -> tuple[int, dict, FakeBackend]:  # type: ignore[no-untyped-def]
        be = FakeBackend()
        be.on_send = disturb
        with tempfile.TemporaryDirectory() as d:
            code, out = run_tool(["--execute", "--no-confirm", "--skip-prechecks", "--hold-s", "1", "--out", d], be)
            res = json.loads((Path(d) / "summary.json").read_text())
        return code, res, be

    def test_tilt_stops_and_returns_smoothly(self) -> None:
        def disturb(n: int, be: FakeBackend) -> None:
            if n > 220:  # 動かし始めて少し経ったあと、骨盤が傾く
                be.quat = quat_rp(4.0, 0.0)

        code, res, be = self.trip_run(disturb)
        self.assertEqual(code, tool.EXIT_GUARD)
        self.assertEqual(res["stop_reason"]["kind"], "tilt")
        self.assertFalse(res["completed"])
        yaws = np.array([c.q[YAW] for c in be.sent])
        self.assertLessEqual(float(np.abs(np.diff(yaws)).max()), np.radians(15) * DT * 1.001)  # 急に飛ばない
        self.assertAlmostEqual(float(yaws[-1]), float(Q0[YAW]), places=6)
        self.assertEqual(be.sent[-1].weight, 0.0)

    def test_other_joint_stops_and_returns(self) -> None:
        def disturb(n: int, be: FakeBackend) -> None:
            if n > 220:
                be.q[22] = Q0[22] + np.radians(4)  # 右腕が押された

        code, res, be = self.trip_run(disturb)
        self.assertEqual(code, tool.EXIT_GUARD)
        self.assertEqual(res["stop_reason"]["kind"], "other")
        self.assertTrue(res["returned_to_start"] in (True, False))

    def test_track_error_stops(self) -> None:
        def disturb(n: int, be: FakeBackend) -> None:
            if n > 220:
                be.q[YAW] = be.q[YAW] + np.radians(4)  # 腰が指令より先へ行く（外から押された）

        code, res, be = self.trip_run(disturb)
        self.assertEqual(code, tool.EXIT_GUARD)
        self.assertEqual(res["stop_reason"]["kind"], "track")

    def test_stale_holds_in_place_and_lowers_weight(self) -> None:
        def disturb(n: int, be: FakeBackend) -> None:
            if n == 220:
                be.stale = True

        code, res, be = self.trip_run(disturb)
        self.assertEqual(res["stop_reason"]["kind"], "stale")
        self.assertFalse(res["returned_to_start"])
        yaws = np.array([c.q[YAW] for c in be.sent])
        # 戻らず、その場の指令のまま weight だけが下がる（指令が飛ばない）
        self.assertLessEqual(float(np.abs(np.diff(yaws)).max()), np.radians(15) * DT * 1.001)
        self.assertEqual(be.sent[-1].weight, 0.0)

    def test_help(self) -> None:
        p = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "waist_turn_check.py"), "--help"],
                           capture_output=True, text=True, timeout=120)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("--execute", p.stdout)
        self.assertIn("--max-deg", p.stdout)


if __name__ == "__main__":
    unittest.main()
