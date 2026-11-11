"""立つ位置まで寄る（common/approach.py）。

どちらへ動くか（decide）は観測を手で作って確かめる。寄る流れ（Approacher）は、偽物の足（common/loco.py の FakeLoco）の
床の上の位置から、柱に対する距離・向き・ボタンの左右を計算する偽物の観測で通す（画像は使わない。評価環境もまだ使わない）。
押せるかの判定（PlanFeasibility）は、本物の計画（Pinocchio と公式モデル）で確かめる。
"""

from __future__ import annotations

import copy
import unittest

import numpy as np

from _env import needs
from common.approach import Approacher, Measurement, decide
from common.config import load_config
from common.loco import FakeLoco, LocoMover

CFG = load_config("approach.yaml")
APP = CFG["approach"]
# 車いす用の勧める立つ位置（elevator.yaml の press.by_button_set.wheelchair.stand と同じ形）
STAND_R = {"min_dist_m": 0.30, "max_dist_m": 0.50, "target_dist_m": 0.40, "center_side_m": -0.14, "max_side_m": 0.02}
STAND_L = dict(STAND_R, center_side_m=0.14)


def dry_mover() -> LocoMover:
    return LocoMover(None, None, CFG["loco"], go=False, log=lambda s: None)


def face(d: float, yaw_deg: float = 0.0) -> Measurement:
    return Measurement("face", d, np.radians(yaw_deg), None)


def buttons(d: float, side: float, yaw_deg: float = 0.0) -> Measurement:
    return Measurement("buttons", d, np.radians(yaw_deg), side)


def dec(m: Measurement, feasible: str | None = None, stands: dict | None = None, cfg: dict | None = None):
    return decide(m, feasible, {"right": "（理由）"} if feasible is None else {}, stands or {"right": STAND_R},
                  cfg or APP, dry_mover())


class TestDecide(unittest.TestCase):
    def test_too_close(self) -> None:
        self.assertEqual(dec(face(0.25)).action, "abort")

    def test_ready_when_plan_ok(self) -> None:
        d = dec(buttons(0.42, -0.10), feasible="right")
        self.assertEqual((d.action, d.arm), ("ready", "right"))

    def test_turn_first(self) -> None:
        d = dec(face(1.0, yaw_deg=12))
        self.assertEqual(d.pulse.axis, "turn_left")
        self.assertEqual(dec(face(1.0, yaw_deg=-12)).pulse.axis, "turn_right")
        self.assertEqual(dec(face(1.0, yaw_deg=4)).pulse.axis, "forward")  # 6° 以内は回らない

    def test_far_face_walks_forward(self) -> None:
        d = dec(face(1.0))
        self.assertEqual(d.pulse.axis, "forward")
        self.assertAlmostEqual(d.pulse.amount, 0.50)  # ▲▼ が見え始める 50 cm まで（1 回では足りず、時間は上限の 1 秒）
        self.assertEqual(d.pulse.duration_s, 1.0)

    def test_face_near_target_steps_until_buttons_visible(self) -> None:
        d = dec(face(0.45))  # 目安まで 5 cm（1 回の最小 11.5 cm の半分より小さい）が、▲▼ が見えない
        self.assertEqual(d.action, "move")
        self.assertIn("見える", d.reason)
        self.assertEqual(dec(face(0.38)).action, "abort")  # 最小の 1 回でも 30 cm より近づく

    def test_face_closer_than_target_does_not_walk(self) -> None:
        stand = dict(STAND_R, target_dist_m=0.48)
        d = dec(face(0.44), stands={"right": stand})  # 目安より近い。▲▼ が見えるまで最小の 1 回だけ進む
        self.assertEqual(d.pulse.duration_s, 0.5)  # 最小の 1 回

    def test_buttons_far_walks_forward(self) -> None:
        d = dec(buttons(0.62, -0.14))
        self.assertEqual(d.pulse.axis, "forward")

    def test_sidestep(self) -> None:
        d = dec(buttons(0.42, -0.02))  # ボタンが目安（右 14 cm）より 12 cm 左 → 体を左へ
        self.assertEqual(d.pulse.axis, "left")
        self.assertAlmostEqual(d.pulse.amount, 0.12)
        self.assertEqual(dec(buttons(0.42, -0.27)).pulse.axis, "right")

    def test_sidestep_blocked_near_wall(self) -> None:
        d = dec(buttons(0.31, 0.10))  # 横へ 24 cm 動くと前にも出て、27 cm より近づくかもしれない
        self.assertEqual(d.action, "abort")
        self.assertIn("横に動けない", d.reason)

    def test_nothing_more_to_do(self) -> None:
        d = dec(buttons(0.42, -0.12))  # 目安に近く、1 回の最小より小さいずれしか残っていないのに計画が立たない
        self.assertEqual(d.action, "abort")
        self.assertIn("計画が立たない", d.reason)

    def test_too_close_for_plan(self) -> None:
        d = dec(buttons(0.28, -0.14))
        self.assertEqual(d.action, "abort")
        self.assertIn("下げる", d.reason)

    def test_picks_closer_arm(self) -> None:
        stands = {"right": STAND_R, "left": STAND_L}
        self.assertEqual(dec(buttons(0.42, 0.20), stands=stands).pulse.axis, "left")  # 左腕の目安（+14）へ
        self.assertEqual(dec(buttons(0.42, -0.20), stands=stands).pulse.axis, "right")  # 右腕の目安（−14）へ


# ---- 寄る流れを通す ---------------------------------------------------------------------


class World:
    """床の上に柱（押す向き +x、▲▼ の中点が床の (bx, by)、柱の面は by の右 20 cm〜左 25 cm）。ロボットの位置は FakeLoco.pose。

    edges=False なら柱の端は分からない（壁と同じ面の柱・画像で切れている）。
    """

    def __init__(self, fake: FakeLoco, bx: float, by: float, visible_m: float = 0.50, noise: float = 0.003,
                 seed: int = 0, edges: bool = True) -> None:
        self.fake, self.bx, self.by, self.visible_m = fake, bx, by, visible_m
        self.edges = edges
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.closest = np.inf
        self.last_reason = ""
        self.n_measure = 0

    def reset(self) -> None:
        pass

    def measure(self) -> Measurement | None:
        self.n_measure += 1
        x, y, yaw = self.fake.pose
        self.closest = min(getattr(self, "closest", np.inf), float(self.bx - x))
        c, s = np.cos(yaw), np.sin(yaw)
        rel = np.array([self.bx - x, self.by - y])
        r = np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1]])  # 胴体の座標
        d = float(self.bx - x) + self.noise * self.rng.standard_normal()
        side = float(r[1]) + self.noise * self.rng.standard_normal()
        if d < self.visible_m:
            return Measurement("buttons", d, -yaw, side)
        if not self.edges:
            return Measurement("face", d, -yaw, None)
        # 端は面に沿って測る（面の向きは床の +y が左）。ばらつきは 1 cm
        e = lambda off: float(self.by + off - y) + 0.01 * self.rng.standard_normal()  # noqa: E731
        return Measurement("face", d, -yaw, None, left_edge_m=e(0.25), right_edge_m=e(-0.20))


class FakeFeasibility:
    """勧める範囲の中・左右 ±6 cm・向き 8° 以内なら押せるとみなす（押し方を選べば押せる、の代わり）。"""

    def __init__(self, stands: dict[str, dict]) -> None:
        self.stands = stands
        self.arms = list(stands)
        self.calls = 0

    def check(self, m: Measurement) -> tuple[str | None, dict[str, str]]:
        self.calls += 1
        reasons = {}
        for arm, st in self.stands.items():
            if (st["min_dist_m"] <= m.distance_m <= st["max_dist_m"] and abs(m.side_m - st["center_side_m"]) <= 0.06
                    and abs(np.degrees(m.yaw_rad)) < 8):
                return arm, reasons
            reasons[arm] = "範囲の外"
        return None, reasons

    def stand(self, arm: str) -> dict:
        return self.stands[arm]


def run(fake: FakeLoco, world: World, stands: dict | None = None, cfg: dict | None = None, go: bool = True):
    mover = LocoMover(fake, fake, CFG["loco"], go=go, sleep=fake.sleep, now=fake.now, log=lambda s: None)
    feas = FakeFeasibility(stands or {"right": STAND_R})
    return Approacher(world, feas, mover, cfg or APP, log=lambda s: None).run(), feas


class TestApproacher(unittest.TestCase):
    def test_reaches_from_1m_with_drift(self) -> None:
        """1 m 手前・ボタンが体の正面から 5 cm 左。前に進むと右へ 19° ずれる足でも、測り直して押せる位置に着く。"""
        fake = FakeLoco(rate_ratio=0.77, heading_error_deg=-19.0, noise=0.1, seed=1)
        res, _ = run(fake, World(fake, bx=1.0, by=0.05))
        self.assertEqual(res.outcome, "ready", res.reason)
        self.assertEqual(res.arm, "right")
        last = res.steps[-1]
        self.assertTrue(0.30 <= last["distance_m"] <= 0.50)
        self.assertLessEqual(abs(last["side_m"] + 0.14), 0.06)
        self.assertLessEqual(len(res.steps), 10)

    def test_turned_start(self) -> None:
        fake = FakeLoco(rate_ratio=0.75, seed=2)
        fake.pose[2] = np.radians(-15)  # 柱から右へ 15° 向いている
        res, _ = run(fake, World(fake, bx=0.9, by=-0.10))
        self.assertEqual(res.outcome, "ready", res.reason)
        self.assertEqual(res.steps[0]["pulse"]["axis"], "turn_left")

    def test_many_seeds(self) -> None:
        """置く位置・足のばらつきを変えても、ほとんど押せる位置に着く（着かないときは中止で、ぶつかりはしない）。"""
        rng = np.random.default_rng(0)
        ready = 0
        for seed in range(30):
            fake = FakeLoco(rate_ratio=rng.uniform(0.6, 0.9), heading_error_deg=rng.uniform(-25, 5), noise=0.15,
                            seed=seed)
            fake.pose[2] = np.radians(rng.uniform(-12, 12))
            world = World(fake, bx=rng.uniform(0.7, 1.4), by=rng.uniform(-0.25, 0.20), seed=seed)
            res, _ = run(fake, world)
            ready += res.outcome == "ready"
            closest = min(world.closest, world.bx - fake.pose[0])
            self.assertGreaterEqual(closest, APP["min_safe_dist_m"] - 0.02, f"seed {seed}: {res.reason}")  # 近づきすぎない
        self.assertGreaterEqual(ready, 27)

    def test_many_seeds_without_edges(self) -> None:
        """柱の端が分からないと、遠くで溜まった横のずれを近くで直しきれないことがある。そのときも中止で、ぶつかりはしない。"""
        rng = np.random.default_rng(1)
        ready = 0
        for seed in range(20):
            fake = FakeLoco(rate_ratio=rng.uniform(0.6, 0.9), heading_error_deg=rng.uniform(-25, 5), noise=0.15,
                            seed=seed)
            world = World(fake, bx=rng.uniform(0.7, 1.2), by=rng.uniform(-0.25, 0.20), seed=seed, edges=False)
            res, _ = run(fake, world)
            ready += res.outcome == "ready"
            self.assertGreaterEqual(min(world.closest, world.bx - fake.pose[0]), APP["min_safe_dist_m"] - 0.02)
        self.assertGreaterEqual(ready, 10)

    def test_edges_keep_side_while_far(self) -> None:
        """前に進むと右へずれる足でも、柱の端を見て、▲▼ が見える前に横へ戻す。"""
        fake = FakeLoco(rate_ratio=0.85, heading_error_deg=-22.0, seed=3)
        res, _ = run(fake, World(fake, bx=1.3, by=0.0))
        self.assertEqual(res.outcome, "ready", res.reason)
        self.assertTrue(any(s["source"] == "face" and s["pulse"] and s["pulse"]["axis"] == "left" for s in res.steps))

    def test_dry_run_stops_after_first_decision(self) -> None:
        fake = FakeLoco()
        res, _ = run(fake, World(fake, bx=1.0, by=0.0), go=False)
        self.assertEqual(res.outcome, "dry_run")
        self.assertEqual(fake.calls, [])
        np.testing.assert_allclose(fake.pose, 0.0)

    def test_already_ready(self) -> None:
        fake = FakeLoco()
        res, _ = run(fake, World(fake, bx=0.42, by=-0.13))
        self.assertEqual(res.outcome, "ready")
        self.assertEqual(fake.calls, [])

    def test_too_close_start(self) -> None:
        fake = FakeLoco()
        res, _ = run(fake, World(fake, bx=0.24, by=-0.14))
        self.assertEqual(res.outcome, "aborted")
        self.assertEqual(fake.calls, [])

    def test_nothing_seen(self) -> None:
        class Blind(World):
            def measure(self) -> Measurement | None:
                self.last_reason = "柱の面が見つからない"
                return None

        fake = FakeLoco()
        res, _ = run(fake, Blind(fake, 1.0, 0.0))
        self.assertEqual(res.outcome, "aborted")
        self.assertIn("見つからない", res.reason)

    def test_legs_do_not_stop(self) -> None:
        class Never(FakeLoco):
            def is_still(self) -> bool:
                return False

        fake = Never()
        res, _ = run(fake, World(fake, bx=1.0, by=0.0))
        self.assertEqual(res.outcome, "aborted")
        self.assertEqual(len(fake.calls), 1)  # 止まらなかったら、それ以上は動かない

    def test_step_limit(self) -> None:
        """足が全く動かない（下限より遅いとみなす）と、回数の上限で止まる。"""
        fake = FakeLoco(min_speed={"vx": 1.0, "vy": 1.0, "wz": 1.0})
        cfg = dict(APP, max_steps=4)
        res, _ = run(fake, World(fake, bx=1.0, by=0.0), cfg=cfg)
        self.assertEqual(res.outcome, "aborted")
        self.assertEqual(len(fake.calls), 4)


@needs("pin", "models")
class TestPlanFeasibility(unittest.TestCase):
    def test_wheelchair_plans(self) -> None:
        """車いす用の ▲▼ が柱まで 40 cm・体の右 14 cm なら右腕で、左 14 cm なら左腕で計画が立ち、70 cm では立たない。"""
        from common.approach import PlanFeasibility
        from common.elevator_buttons import Button3D, CallButtons

        el = copy.deepcopy(load_config("elevator.yaml"))
        el["geometry"]["button_set"] = "wheelchair"
        feas = PlanFeasibility.from_config(load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"),
                                           el, ["right", "left"], "up")

        def call(x: float, y: float) -> Measurement:
            b = lambda z, a: Button3D(np.array([x, y, z]), 0.035, a, (300, 200, 320, 220), 100, a)  # noqa: E731
            return Measurement.from_call(CallButtons(b(0.21, "up"), b(0.14, "down"), np.array([1.0, 0, 0])), np.zeros(29))

        self.assertEqual(feas.check(call(0.40, -0.14))[0], "right")
        self.assertEqual(feas.check(call(0.40, 0.14))[0], "left")
        arm, reasons = feas.check(call(0.70, -0.14))
        self.assertIsNone(arm)
        self.assertEqual(set(reasons), {"right", "left"})
        self.assertAlmostEqual(feas.stand("left")["center_side_m"], -feas.stand("right")["center_side_m"])


if __name__ == "__main__":
    unittest.main()
