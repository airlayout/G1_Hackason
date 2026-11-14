"""1 回分だけ歩く（common/loco.py）。実機・SDK は使わず、偽物の足（FakeLoco）と時計で確かめる。"""

from __future__ import annotations

import copy
import subprocess
import sys
import unittest
from dataclasses import dataclass, field

import numpy as np

from _env import needs
from common.config import FEATURE_DIR, load_config
from common.loco import FakeLoco, LegStillness, LocoError, LocoMover, Pulse

CFG = load_config("approach.yaml")["loco"]
# 前へ 1 秒あたりに動く量と最小の時間（実機で測って変わるので、数字を書かずに設定から取る）
RATE_FWD = float(CFG["axes"]["forward"]["rate"])
MIN_S_FWD = float(CFG["axes"]["forward"]["min_duration_s"])


def mover(fake: FakeLoco | None = None, cfg: dict | None = None, go: bool = True,
          stillness: object | None = None) -> tuple[LocoMover, FakeLoco, list[str]]:
    fake = fake or FakeLoco()
    logs: list[str] = []
    m = LocoMover(fake, stillness or fake, cfg or CFG, go=go, sleep=fake.sleep, now=fake.now, log=logs.append)
    return m, fake, logs


class TestPlan(unittest.TestCase):
    def test_forward(self) -> None:
        m, _, _ = mover()
        p = m.plan("forward", 0.20)
        self.assertEqual((p.vx, p.vy, p.wz), (0.30, 0.0, 0.0))
        self.assertAlmostEqual(p.duration_s, 0.20 / RATE_FWD)

    def test_large_is_capped_and_small_is_skipped(self) -> None:
        m, _, _ = mover()
        self.assertEqual(m.plan("forward", 0.80).duration_s, 1.0)  # 1 回では足りない。呼ぶ側が測り直して続ける
        step = RATE_FWD * MIN_S_FWD  # 最小の 1 回で動く量（0.21 × 0.5 = 約 10.5 cm）
        self.assertAlmostEqual(m.plan("forward", step * 0.8).duration_s, MIN_S_FWD)  # 最小より小さくても半分以上なら最小の時間
        self.assertIsNone(m.plan("forward", step * 0.45))  # 最小の半分より小さいずれは直さない
        self.assertAlmostEqual(m.min_step("forward"), step)

    def test_sides_and_turns(self) -> None:
        m, _, _ = mover()
        self.assertEqual(m.plan("right", 0.15).vy, -0.30)
        self.assertEqual(m.plan("turn_left", np.radians(15)).wz, 0.30)
        self.assertEqual(m.plan("turn_right", np.radians(15)).wz, -0.30)

    def test_bad_axis(self) -> None:
        m, _, _ = mover()
        with self.assertRaises(ValueError):
            m.plan("up", 0.1)
        with self.assertRaises(LocoError):
            m.plan("backward", 0.1)  # 既定では使わない


class TestCheck(unittest.TestCase):
    def test_rejects(self) -> None:
        m, fake, _ = mover()
        for p in (Pulse(0.3, 0.3, 0.0, 0.5),   # 2 つの軸
                  Pulse(0.1, 0.0, 0.0, 0.5),   # 遅すぎて歩き出さない
                  Pulse(0.5, 0.0, 0.0, 0.5),   # 速すぎる
                  Pulse(0.3, 0.0, 0.0, 1.5),   # 時間が長すぎる（落ちたときに止まるまでが長い）
                  Pulse(0.0, 0.0, 0.0, 0.5)):  # 何も動かさない
            with self.subTest(p=p), self.assertRaises(ValueError):
                m.pulse(p)
        with self.assertRaises(LocoError):
            m.pulse(Pulse(-0.3, 0.0, 0.0, 0.5))  # 後ろ
        self.assertEqual(fake.calls, [])  # どれも送っていない

    def test_limits(self) -> None:
        cfg = copy.deepcopy(CFG)
        cfg["limits"]["max_pulses"] = 2
        m, fake, _ = mover(cfg=cfg)
        m.pulse(m.plan("left", 0.15))
        m.pulse(m.plan("left", 0.15))
        with self.assertRaises(LocoError):
            m.pulse(m.plan("left", 0.15))
        self.assertEqual(len(fake.calls), 2)

    def test_forward_total(self) -> None:
        m, fake, _ = mover()
        n = int(float(CFG["limits"]["max_forward_total_m"]) // RATE_FWD)  # 1 回 RATE_FWD m（1 秒）で、上限 1.5 m
        for _ in range(n):
            m.pulse(m.plan("forward", 0.5))
        with self.assertRaises(LocoError):
            m.pulse(m.plan("forward", 0.5))
        self.assertEqual(len(fake.calls), n)


    def test_side_total(self) -> None:
        """横へ動いた見込みの合計（左右で打ち消す）が上限を超える 1 回は送らない（頭カメラは横の物が見えない。11/14）。"""
        m, fake, _ = mover()
        lim = float(CFG["limits"]["max_side_total_m"])
        step = float(CFG["axes"]["left"]["rate"]) * float(CFG["axes"]["left"]["max_duration_s"])
        n = int(lim // step)
        for _ in range(n):
            m.pulse(m.plan("left", 0.5))
        self.assertAlmostEqual(m.side_total_m, n * step)
        with self.assertRaises(LocoError):
            m.pulse(m.plan("left", 0.5))
        m.pulse(m.plan("right", 0.5))  # 戻る向きなら送れる
        self.assertEqual(len(fake.calls), n + 1)


class TestPulse(unittest.TestCase):
    def test_dry_run_sends_nothing(self) -> None:
        logs: list[str] = []
        m = LocoMover(None, None, CFG, go=False, log=logs.append)
        r = m.pulse(m.plan("forward", 0.2))
        self.assertFalse(r.sent)
        self.assertTrue(any("送らない" in s for s in logs))

    def test_waits_until_legs_stop(self) -> None:
        m, fake, _ = mover(FakeLoco(rate_ratio=0.75, tail_s=1.9))
        r = m.pulse(m.plan("forward", 0.20))
        self.assertTrue(r.sent and r.stopped)
        dur = 0.20 / RATE_FWD
        # 指令の時間（約 0.95 秒）＋歩き終わり（1.9 秒）が過ぎるまで待った
        self.assertGreaterEqual(r.settle_s, dur + 1.9 - 1e-6)
        self.assertLess(r.settle_s, dur + 1.9 + 0.1)
        self.assertAlmostEqual(fake.pose[0], 0.75 * 0.30 * dur, places=6)
        self.assertEqual(fake.calls[0][3], dur)  # duration を明示して送った

    def test_not_stopping_sends_stop(self) -> None:
        class Never:
            def is_still(self) -> bool:
                return False

        stops: list[int] = []
        fake = FakeLoco()
        fake.stop = lambda: stops.append(1)  # type: ignore[method-assign]
        m, _, _ = mover(fake, stillness=Never())
        r = m.pulse(m.plan("turn_left", 0.2))
        self.assertFalse(r.stopped)
        self.assertEqual(stops, [1])
        self.assertIn("StopMove", r.note)

    def test_wrong_fsm(self) -> None:
        fake = FakeLoco()
        fake.fsm = 200  # 歩ける状態でない
        m, _, _ = mover(fake)
        with self.assertRaises(LocoError):
            m.pulse(m.plan("forward", 0.2))
        self.assertEqual(fake.calls, [])

    def test_interrupt_stops(self) -> None:
        """歩いている途中に Ctrl+C（例外）で抜けても、StopMove を送る。"""
        stops: list[int] = []
        fake = FakeLoco()
        fake.stop = lambda: stops.append(1)  # type: ignore[method-assign]

        def sleep(dt: float) -> None:
            raise KeyboardInterrupt

        m = LocoMover(fake, fake, CFG, go=True, sleep=sleep, now=fake.now, log=lambda s: None)
        with self.assertRaises(KeyboardInterrupt), m:
            m.pulse(m.plan("forward", 0.2))
        self.assertEqual(stops, [1])


class TestFakeLoco(unittest.TestCase):
    def test_too_slow_does_not_move(self) -> None:
        fake = FakeLoco(min_speed={"vx": 0.25, "vy": 0.25, "wz": 0.25})
        fake.set_velocity(0.2, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(fake.pose, 0.0)

    def test_heading_error_drifts_sideways(self) -> None:
        fake = FakeLoco(rate_ratio=0.77, heading_error_deg=-19.0)
        fake.set_velocity(0.3, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(np.hypot(*fake.pose[:2]), 0.231, places=3)
        self.assertLess(fake.pose[1], -0.07)  # 右へ約 7.5 cm

    def test_turn_then_forward_in_floor_frame(self) -> None:
        fake = FakeLoco(rate_ratio=1.0)
        fake.set_velocity(0.0, 0.0, 0.5, np.pi)  # 左に 90°（π/2 rad）
        fake.set_velocity(0.3, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(fake.pose, [0.0, 0.3, np.pi / 2], atol=1e-9)


@dataclass
class _State:
    dq: np.ndarray
    stamp: float


@dataclass
class _Reader:
    state: _State | None = None
    clock: list[float] = field(default_factory=lambda: [0.0])

    def latest(self) -> _State | None:
        return self.state


class TestLegStillness(unittest.TestCase):
    def test_hold(self) -> None:
        clock = [0.0]
        reader = _Reader()
        still = LegStillness(reader, CFG["still"], now=lambda: clock[0])
        self.assertFalse(still.is_still())  # lowstate が無い
        dq = np.zeros(29)
        dq[3] = 0.5
        reader.state = _State(dq.copy(), 0.0)
        self.assertFalse(still.is_still())  # 膝が動いている
        dq[3] = 0.02
        dq[20] = 3.0  # 腕は見ない
        for t in (0.1, 0.3, 0.55, 0.65):
            clock[0] = t
            reader.state = _State(dq.copy(), t)
            res = still.is_still()
        self.assertTrue(res)  # 0.1 秒から 0.55 秒以上続いた
        clock[0] = 1.5
        self.assertFalse(still.is_still())  # lowstate が古い

    def _run(self, still: LegStillness, reader: "_Reader", clock: list[float], seq: list[tuple[float, float]]) -> float | None:
        """(時刻, 膝の角速度) を順に流し、止まったとみなした最初の時刻を返す（無ければ None）。"""
        for t, v in seq:
            dq = np.zeros(29)
            dq[3] = v
            clock[0] = t
            reader.state = _State(dq, t)
            if still.is_still():
                return t
        return None

    def test_short_spike_is_ignored(self) -> None:
        """止まって立っていても 1 回だけ跳ねる値（11/14 の実機で 0.2〜0.46 rad/s）では数え直さない。"""
        clock = [0.0]
        reader = _Reader()
        still = LegStillness(reader, CFG["still"], now=lambda: clock[0])
        # 1/16 秒おき（時刻の足し算で丸めの誤差が出ないように）。0.125 秒・0.375 秒に 1 回ずつ 0.3 rad/s が跳ねる
        seq = [(i / 16, 0.3 if i in (2, 6) else 0.03) for i in range(24)]
        self.assertEqual(self._run(still, reader, clock, seq), 0.5)  # 0.0 秒から 0.5 秒続いた
        # 跳ねを無視しない判定（spike_s 0）では、0.4375 秒から数え直して 0.9375 秒になる
        clock = [0.0]
        still = LegStillness(reader, dict(CFG["still"], spike_s=0.0), now=lambda: clock[0])
        self.assertEqual(self._run(still, reader, clock, seq), 0.9375)

    def test_long_or_large_is_motion(self) -> None:
        """続けて超える（歩き終わり）か、跳ねにしては大きい値（歩いている）なら、短くても数え直す。"""
        clock = [0.0]
        reader = _Reader()
        still = LegStillness(reader, CFG["still"], now=lambda: clock[0])
        # 0.1875〜0.3125 秒の 3 回続けて 0.3 rad/s（0.125 秒続いた）→ 数え直し、0.375 秒から 0.5 秒で 0.875 秒
        seq = [(i / 16, 0.3 if i in (3, 4, 5) else 0.03) for i in range(24)]
        self.assertEqual(self._run(still, reader, clock, seq), 0.875)
        clock = [0.0]
        still = LegStillness(reader, CFG["still"], now=lambda: clock[0])
        # 0.1875 秒に 1 回だけ 1.5 rad/s → 数え直し、0.25 秒から 0.5 秒で 0.75 秒
        seq = [(i / 16, 1.5 if i == 3 else 0.03) for i in range(24)]
        self.assertEqual(self._run(still, reader, clock, seq), 0.75)


# 同じプロセスで別の口の DDS を初期化すると、ほかのテストとぶつかるので、別のプロセスで動かす
_RPC_SCRIPT = r"""
import json, sys, time
from common.dds import init_dds
init_dds(0, "lo")
from unitree_sdk2py.g1.loco import g1_loco_api as api
from unitree_sdk2py.rpc.server import Server
got = []
class S(Server):  # Yada の sim/mujoco/g1_sim_server.py の make_loco_server と同じ形
    def __init__(self):
        super().__init__(api.LOCO_SERVICE_NAME)
    def Init(self):
        self._SetApiVersion(api.LOCO_API_VERSION)
        for n in dir(api):
            if n.startswith("ROBOT_API_ID_LOCO_"):
                i = getattr(api, n)
                self._RegistHandler(i, self.h(i), False)
    def h(self, i):
        def f(p):
            got.append([i, json.loads(p or "{}")])
            return (0, json.dumps({"data": 501})) if i == api.ROBOT_API_ID_LOCO_GET_FSM_ID else (0, "")
        return f
s = S(); s.Init(); s.Start(False); time.sleep(0.5)
from common.loco import SdkLocoBackend
b = SdkLocoBackend()
fsm = b.fsm_id()
b.set_velocity(0.3, 0.0, 0.0, 0.6)
b.stop()
time.sleep(0.3)
print("RESULT " + json.dumps({"fsm": fsm, "got": got}))
"""


@needs("sdk")
class TestSdkBackendLoopback(unittest.TestCase):
    def test_sends_explicit_duration(self) -> None:
        """実機用の SdkLocoBackend が、SetVelocity に duration を明示して送る（ループバックの sport サーバで受ける）。"""
        import json

        r = subprocess.run([sys.executable, "-c", _RPC_SCRIPT], cwd=FEATURE_DIR, capture_output=True, text=True,
                           timeout=60)
        line = [x for x in r.stdout.splitlines() if x.startswith("RESULT ")]
        self.assertTrue(line, r.stdout + r.stderr)
        res = json.loads(line[0][7:])
        self.assertEqual(res["fsm"], 501)
        vel = [p for i, p in res["got"] if i == 7105]
        self.assertEqual(vel[0], {"velocity": [0.3, 0.0, 0.0], "duration": 0.6})
        self.assertEqual(vel[1]["velocity"], [0.0, 0.0, 0.0])  # StopMove


if __name__ == "__main__":
    unittest.main()
