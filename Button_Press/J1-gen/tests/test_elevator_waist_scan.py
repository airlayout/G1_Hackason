"""腰を回して探す段階（configs/elevator.yaml の press.waist_scan、common/elevator_press.py の turn / untwist、
common/front_clearance.py）。2026-11-13。

finder を偽物（腰の角度を見て、見つかる・見つからないを返す）に差し替え、体は指令どおりに動くとして 50 Hz で回す。
画像は偽（暗い画像。保持・戻りの間は白い画像にして、点灯したことにする）。
"""

from __future__ import annotations

import copy
import unittest
from unittest import mock

import numpy as np

from _env import needs
from common.config import FEATURE_DIR, load_config
from common.elevator_buttons import Button3D, ButtonNotFound, CallButtons

DT = 0.02
K = np.array([[300.0, 0.0, 32.0], [0.0, 300.0, 32.0], [0.0, 0.0, 1.0]])
BOX = (20, 20, 40, 40)
UPPER = np.full(17, 60.0)


def make_call(x: float, y: float, z_up: float = 0.21, z_down: float = 0.14) -> CallButtons:
    up = Button3D(np.array([x, y, z_up]), 0.035, "up", BOX, 100, "up")
    down = Button3D(np.array([x, y, z_down]), 0.035, "down", BOX, 100, "down")
    return CallButtons(up, down, np.array([1.0, 0.0, 0.0]))


class FakeDetector:
    def detect(self, rgb: np.ndarray) -> list:
        return []


class FakeFinder:
    """腰 yaw が found_deg のどれかに近いときだけ call を返す（それ以外は ButtonNotFound）。"""

    def __init__(self, call: CallButtons, found_deg: list[float]) -> None:
        self.call, self.found = call, [np.radians(d) for d in found_deg]
        self.detector = FakeDetector()
        self.seen_waist: list[float] = []  # find に渡された腰 yaw [rad]

    def find(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, q_waist: np.ndarray) -> CallButtons:
        yaw = float(q_waist[0])
        self.seen_waist.append(yaw)
        if any(abs(yaw - f) < np.radians(1.0) for f in self.found):
            return self.call
        raise ButtonNotFound("偽: 画像に入らない")


@needs("mujoco", "pin")
class WaistScanBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import sys

        sys.path.insert(0, str(FEATURE_DIR / "sim"))
        from reach_map import REAL_STAND_1021

        cls.q0 = REAL_STAND_1021.copy()
        cls.cfgs = tuple(load_config(n) for n in ("robot.yaml", "press.yaml", "arm.yaml", "elevator.yaml"))

    def controller(self, enabled: bool = True, arm: str = "right", scan: dict | None = None, button_set: str = "wheelchair"):
        from common.elevator_press import ElevatorPressController

        robot, press, armc, elev = self.cfgs
        e = copy.deepcopy(elev)
        e["geometry"]["button_set"] = button_set
        e["press"]["arm"] = arm
        e["press"]["waist_scan"]["enabled"] = enabled
        e["press"]["waist_scan"].update(scan or {})
        return ElevatorPressController.from_config(robot, press, armc, e)

    def run_loop(self, ctl, finder, stop: str | None = None, max_steps: int = 8000, target: str = "down"):
        """done か、段階が stop になるまで回す。記録 (段階, 腰の指令 [rad], 返した腰の目標 [rad]) の並びと、最後の done を返す。"""
        ctl.finder = finder
        ctl.reset(target, DT, UPPER)
        dark, bright = np.full((64, 64, 3), 50, np.uint8), np.full((64, 64, 3), 255, np.uint8)
        depth = np.zeros((64, 64))
        q, t, rec, done = self.q0.copy(), 0.0, [], False
        for _ in range(max_steps):
            t += DT
            rgb = bright if ctl.phase in ("hold", "retract", "check", "home") else dark
            out, done = ctl.step(rgb, depth, K, q, t, t)
            q = out.copy()  # 体は指令どおりに動く
            rec.append((ctl.phase, float(ctl.q_hold[12]), float(out[12])))
            if done or (stop is not None and ctl.phase == stop):
                break
        return rec, done

    def phases(self, rec) -> list[str]:
        out: list[str] = []
        for p, _, _ in rec:
            if not out or out[-1] != p:
                out.append(p)
        return out

    def near_front(self, ctl, dist: float = 0.5, n: int = 1000):
        return mock.patch.object(ctl, "_front_distance", return_value=(dist, n))


class TestDefaultOff(WaistScanBase):
    def test_yaml_default_is_off(self) -> None:
        self.assertFalse(self.cfgs[3]["press"]["waist_scan"]["enabled"])

    def test_look_timeout_gives_up_as_before(self) -> None:
        """既定（オフ）では、look の時間切れで今までどおり中止する。腰は動かさない。"""
        ctl = self.controller(enabled=False)
        rec, done = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[10.0]))
        self.assertTrue(done)
        self.assertEqual(self.phases(rec), ["look", "done"])
        self.assertTrue(ctl.outcome.startswith("中止: 時間内に一般用の ▲▼ を見つけられない"), ctl.outcome)
        self.assertNotIn("腰", ctl.outcome)
        self.assertEqual({round(w, 9) for _, w, _ in rec}, {round(float(self.q0[12]), 9)})


class TestScan(WaistScanBase):
    def test_found_at_plus_10_presses_with_the_waist_twisted_and_returns(self) -> None:
        """腰 0° では見つからず +10° で見つかる → turn → look → 計画 → 押す。reach 以降も腰の指令は +10°。
        done の前に、腰が始めの角度へ戻る（done は戻ってから）。"""
        ctl = self.controller()
        finder = FakeFinder(make_call(0.40, -0.14), found_deg=[10.0])
        with self.near_front(ctl):
            rec, done = self.run_loop(ctl, finder)
        self.assertTrue(done, ctl.outcome)
        self.assertEqual(ctl.outcome, "点灯した")
        ph = self.phases(rec)
        self.assertEqual(ph[:5], ["look", "turn", "look", "reach", "settle"], ph)
        self.assertEqual(ph[-3:], ["home", "untwist", "done"], ph)
        yaw0, y10 = float(self.q0[12]), np.radians(10.0)
        for p, w, _ in rec:
            if p in ("reach", "settle", "press", "hold", "retract", "check", "home"):
                self.assertAlmostEqual(w, y10, places=9, msg=p)  # 腕の段階で腰の指令が戻らない
        self.assertAlmostEqual(rec[-1][1], yaw0, places=9)
        first_done = next(w for p, w, _ in rec if p == "done")
        self.assertAlmostEqual(first_done, yaw0, places=9)  # done の段階に入るときには、もう腰の指令は戻っている
        # 見つけたときに finder へ渡された腰 yaw は、回した先（実際の q）
        self.assertAlmostEqual(finder.seen_waist[-1], y10, places=3)
        self.assertEqual(ctl.scan_tried, [10.0])
        msgs = " ".join(l.message for l in ctl.log)
        self.assertIn("腰を +10° へ回して探す", msgs)
        self.assertIn("見つけた", msgs)
        self.assertIn("腰を始めの角度へ戻した", msgs)

    def test_found_at_minus_10_tries_plus_10_first(self) -> None:
        """−10° で見つかるなら、+10° → −10° の順に回り、−10° のまま計画へ進む。"""
        ctl = self.controller()
        finder = FakeFinder(make_call(0.40, -0.14), found_deg=[-10.0])
        with self.near_front(ctl):
            rec, _ = self.run_loop(ctl, finder, stop="reach")
        self.assertEqual(ctl.scan_tried, [10.0, -10.0])
        self.assertEqual(self.phases(rec), ["look", "turn", "look", "turn", "look", "reach"])
        self.assertAlmostEqual(rec[-1][1], np.radians(-10.0), places=9)
        self.assertAlmostEqual(finder.seen_waist[-1], np.radians(-10.0), places=3)

    def test_nothing_found_returns_and_gives_up_with_the_tried_angles(self) -> None:
        ctl = self.controller()
        with self.near_front(ctl):
            rec, done = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[]))
        self.assertTrue(done)
        self.assertEqual(self.phases(rec)[-2:], ["untwist", "done"])
        self.assertTrue(ctl.outcome.startswith("中止:"), ctl.outcome)
        self.assertIn("+10°、-10°、+20°、-20°", ctl.outcome)
        self.assertAlmostEqual(rec[-1][1], float(self.q0[12]), places=9)  # 腰は始めの角度へ戻っている
        self.assertEqual(self.phases(rec).count("turn"), 4)

    def test_turn_speed_is_limited(self) -> None:
        """連続する周期の腰の指令の差は、speed_deg_s から決まる上限（smoothstep の最大は平均の 1.5 倍）を超えない。
        指令がいちばん動くのは turn と untwist（腰の指令の差が出るのはこの 2 つだけ）。"""
        speed = 10.0
        ctl = self.controller(scan={"speed_deg_s": speed})
        with self.near_front(ctl):
            rec, _ = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[]))
        cmd = np.array([w for _, w, _ in rec])
        step = np.abs(np.diff(cmd))
        limit = 1.5 * np.radians(speed) * DT + 1e-9
        self.assertLessEqual(float(step.max()), limit)
        self.assertGreater(float(step.max()), 0.5 * limit)  # 実際に回している
        # 回し終わるまでの時間 = 角度差 / 速さ（10° なら 1 秒）
        self.assertAlmostEqual(abs(cmd[-1] - self.q0[12]), 0.0, places=9)

    def test_unreachable_after_turn_untwists_and_gives_up(self) -> None:
        """回した先で見つけたが計画が立たない → 腰を始めの角度へ戻してから中止。"""
        ctl = self.controller()
        with self.near_front(ctl):
            rec, done = self.run_loop(ctl, FakeFinder(make_call(0.52, -0.28), found_deg=[10.0]))
        self.assertTrue(done)
        self.assertEqual(self.phases(rec), ["look", "turn", "look", "untwist", "done"])
        self.assertIn("計画できない", ctl.outcome)
        self.assertAlmostEqual(rec[-1][1], float(self.q0[12]), places=9)

    def test_works_with_auto_arm(self) -> None:
        """press.arm: auto と一緒でも動く（腕を選んで、腰をひねったまま reach まで進む）。"""
        ctl = self.controller(arm="auto")
        with self.near_front(ctl):
            rec, _ = self.run_loop(ctl, FakeFinder(make_call(0.40, 0.14), found_deg=[10.0]), stop="settle")
        self.assertEqual(self.phases(rec), ["look", "turn", "look", "reach", "settle"])
        self.assertEqual(ctl.side, "left")
        self.assertAlmostEqual(rec[-1][1], np.radians(10.0), places=9)

    def test_reset_forgets_the_previous_angle(self) -> None:
        ctl = self.controller()
        with self.near_front(ctl):
            self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[]), stop="turn")
        self.assertEqual(ctl.scan_tried, [10.0])
        ctl.reset("up", DT, UPPER)
        self.assertEqual((ctl.scan_tried, ctl.scan_i, ctl.waist0, ctl._turn, ctl._look_until), ([], 0, None, None, None))

    def test_max_deg_is_enforced(self) -> None:
        with self.assertRaises(ValueError):
            self.controller(scan={"yaws_deg": [10, 25], "max_deg": 20})
        self.controller(scan={"yaws_deg": [10, 25], "max_deg": 30})  # 上限を上げれば作れる


class HintFinder(FakeFinder):
    """FakeFinder に last_buttons を持たせたもの。見つからないときも、見えているボタン seen（y のリスト）を残す。"""

    def __init__(self, call: CallButtons, found_deg: list[float], seen_y: list[float] | None = None) -> None:
        super().__init__(call, found_deg)
        self.seen_y = seen_y or []
        self.last_buttons: list = []

    def find(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, q_waist: np.ndarray) -> CallButtons:
        self.last_buttons = [make_call(0.40, y).up for y in self.seen_y]  # find のたびに新しい list（ButtonFinder と同じ）
        return super().find(rgb, depth, K, q_waist)


class FakeFaceFinder:
    """柱の面の端だけを返す偽物（left / right は None で切れている）。FaceNotFound を出す指定もできる。"""

    def __init__(self, left: float | None, right: float | None, notfound: bool = False) -> None:
        self.left, self.right, self.notfound = left, right, notfound
        self.calls = 0

    def find(self, depth, K, q_waist, imu_quat=None):  # noqa: ANN001
        from types import SimpleNamespace

        from common.column_face import FaceNotFound

        self.calls += 1
        if self.notfound:
            raise FaceNotFound("偽: 面なし")
        return SimpleNamespace(left_edge_m=self.left, right_edge_m=self.right)


class TestOrderHint(WaistScanBase):
    def order(self, finder, face=None, scan: dict | None = None) -> tuple[list[float], "ElevatorPressController"]:
        """最初に回し始める段階（turn）まで回し、決まった順と controller を返す。"""
        ctl = self.controller(scan=scan)
        ctl.face_finder = face
        with self.near_front(ctl):
            self.run_loop(ctl, finder, stop="turn")
        return ctl.scan_order, ctl

    def test_no_hint_keeps_configured_order(self) -> None:
        for finder in (FakeFinder(make_call(0.40, -0.14), []), HintFinder(make_call(0.40, -0.14), [])):
            order, ctl = self.order(finder)
            self.assertEqual(order, [10.0, -10.0, 20.0, -20.0])
            self.assertEqual(ctl.scan_tried, [10.0])

    def test_visible_button_on_right_starts_from_minus(self) -> None:
        order, ctl = self.order(HintFinder(make_call(0.40, -0.14), [], seen_y=[-0.20, -0.18]))
        self.assertEqual(order, [-10.0, -20.0, 10.0, 20.0])
        self.assertEqual(ctl.scan_tried, [-10.0])
        self.assertTrue(any("回す順" in m.message and "ボタン" in m.message and "右" in m.message for m in ctl.log))

    def test_visible_button_on_left_starts_from_plus(self) -> None:
        order, _ = self.order(HintFinder(make_call(0.40, 0.14), [], seen_y=[0.20]))
        self.assertEqual(order, [10.0, 20.0, -10.0, -20.0])

    def test_button_near_front_keeps_configured_order(self) -> None:
        order, ctl = self.order(HintFinder(make_call(0.40, 0.0), [], seen_y=[0.04, -0.03]), face=FakeFaceFinder(0.2, None))
        self.assertEqual(order, [10.0, -10.0, 20.0, -20.0])
        self.assertEqual(ctl.face_finder.calls, 0)  # ボタンが見えているので柱の面は見ない

    def test_stale_last_buttons_are_ignored(self) -> None:
        """find が last_buttons を更新しなかった（同じ list のまま）なら、古い値として使わない。"""
        f = HintFinder(make_call(0.40, -0.14), [])
        f.last_buttons = [make_call(0.40, -0.30).up]
        f.find = lambda rgb, depth, K, q: (_ for _ in ()).throw(ButtonNotFound("偽: 更新前に失敗"))  # type: ignore[method-assign]
        order, _ = self.order(f)
        self.assertEqual(order, [10.0, -10.0, 20.0, -20.0])

    def test_face_edge_only_left_visible_goes_right(self) -> None:
        """左の端だけ見える（右は切れている）→ 柱は右へ続く → 右から。"""
        order, ctl = self.order(FakeFinder(make_call(0.40, -0.14), []), face=FakeFaceFinder(0.02, None))
        self.assertEqual(order, [-10.0, -20.0, 10.0, 20.0])
        self.assertTrue(any("仮定" in m.message for m in ctl.log))

    def test_face_edge_only_right_visible_goes_left(self) -> None:
        order, _ = self.order(FakeFinder(make_call(0.40, 0.14), []), face=FakeFaceFinder(None, -0.02))
        self.assertEqual(order, [10.0, 20.0, -10.0, -20.0])

    def test_face_both_edges_uses_center_sign(self) -> None:
        self.assertEqual(self.order(FakeFinder(make_call(0.4, 0), []), face=FakeFaceFinder(-0.05, -0.35))[0],
                         [-10.0, -20.0, 10.0, 20.0])
        self.assertEqual(self.order(FakeFinder(make_call(0.4, 0), []), face=FakeFaceFinder(0.35, 0.05))[0],
                         [10.0, 20.0, -10.0, -20.0])
        self.assertEqual(self.order(FakeFinder(make_call(0.4, 0), []), face=FakeFaceFinder(0.25, -0.20))[0],
                         [10.0, -10.0, 20.0, -20.0])  # 中央 +2.5 cm は正面に近い

    def test_face_unusable_keeps_configured_order(self) -> None:
        for face in (FakeFaceFinder(None, None), FakeFaceFinder(None, None, notfound=True), None):
            order, _ = self.order(FakeFinder(make_call(0.40, -0.14), []), face=face)
            self.assertEqual(order, [10.0, -10.0, 20.0, -20.0])

    def test_wrong_guess_still_tries_the_other_side(self) -> None:
        """手がかりは右だが、実際は +10° で見つかる → −10, −20 を試したあと +10 で見つけて押す。"""
        ctl = self.controller()
        ctl.face_finder = FakeFaceFinder(0.02, None)
        finder = FakeFinder(make_call(0.40, -0.14), found_deg=[10.0])
        with self.near_front(ctl):
            rec, _ = self.run_loop(ctl, finder, stop="reach")
        self.assertEqual(ctl.scan_tried, [-10.0, -20.0, 10.0])
        self.assertAlmostEqual(rec[-1][1], np.radians(10.0), places=9)

    def test_right_button_is_found_sooner_with_the_hint(self) -> None:
        def steps(hint: bool) -> int:
            ctl = self.controller(scan={"order_hint": {"enabled": hint}})
            ctl.face_finder = FakeFaceFinder(0.02, None)
            with self.near_front(ctl):
                rec, _ = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[-10.0]), stop="reach")
            return len(rec)

        with_hint, without = steps(True), steps(False)
        self.assertLess(with_hint, without)
        print(f"[test] -10° で見つかるまで: ヒントあり {with_hint * DT:.1f} 秒 / なし {without * DT:.1f} 秒")

    def test_disabled_keeps_configured_order_and_does_not_look(self) -> None:
        face = FakeFaceFinder(0.02, None)
        order, ctl = self.order(HintFinder(make_call(0.4, -0.14), [], seen_y=[-0.2]), face=face,
                                scan={"order_hint": {"enabled": False}})
        self.assertEqual(order, [10.0, -10.0, 20.0, -20.0])
        self.assertEqual(face.calls, 0)

    def test_order_is_decided_again_after_reset(self) -> None:
        ctl = self.controller()
        ctl.face_finder = FakeFaceFinder(0.02, None)
        with self.near_front(ctl):
            self.run_loop(ctl, FakeFinder(make_call(0.4, -0.14), []), stop="turn")
            self.assertEqual(ctl.scan_order[0], -10.0)
            ctl.face_finder = FakeFaceFinder(None, -0.02)
            self.run_loop(ctl, FakeFinder(make_call(0.4, 0.14), []), stop="turn")
        self.assertEqual(ctl.scan_order[0], 10.0)

    def test_scan_off_does_nothing(self) -> None:
        ctl = self.controller(enabled=False)
        ctl.face_finder = FakeFaceFinder(0.02, None)
        rec, _ = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), []))
        self.assertEqual(ctl.face_finder.calls, 0)
        self.assertEqual(self.phases(rec), ["look", "done"])

    def test_from_config_builds_face_finder_only_when_enabled(self) -> None:
        self.assertIsNone(self.controller(enabled=False).face_finder)
        self.assertIsNotNone(self.controller().face_finder)
        self.assertIsNone(self.controller(scan={"order_hint": {"use_face": False}}).face_finder)


class TestFaceEdgesAtNearRange(unittest.TestCase):
    """FaceFinder の端が、腰をひねる前の近さ（柱まで 28〜40 cm）で出るかの確認（合成の深度。tests/test_column_face.py の render）。
    結果: 距離は出るが、端が出るのは端が画像の中（pelvis の正面から左右 約 12 cm 以内）にあるときだけ。
    面が画像より広いと両端とも None（手がかりにならない）。"""

    def test_near_edges_visible_only_when_inside_the_image(self) -> None:
        import test_column_face as T

        for dist in (0.28, 0.34, 0.40):
            # 体の右にある柱（面は pelvis の正面から右へ 45 cm、左の端が正面付近）: 左の端だけ見える
            f = T.find(T.render(dist, edges=(-0.45, 0.02)))
            self.assertIsNone(f.right_edge_m, dist)
            self.assertIsNotNone(f.left_edge_m, dist)
            # 左へ 45 cm: 右の端だけ
            f = T.find(T.render(dist, edges=(-0.02, 0.45)))
            self.assertIsNone(f.left_edge_m, dist)
            self.assertIsNotNone(f.right_edge_m, dist)
            # ラボの柱の幅（45 cm）が画像いっぱいに写る真正面: 両端とも切れて手がかりにならない
            f = T.find(T.render(dist, edges=(-0.20, 0.25)))
            self.assertIsNone(f.left_edge_m, dist)
            self.assertIsNone(f.right_edge_m, dist)


class TestFrontCheck(WaistScanBase):
    def test_close_object_prevents_the_twist(self) -> None:
        ctl = self.controller()
        with self.near_front(ctl, dist=0.20):
            rec, done = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[10.0]))
        self.assertTrue(done)
        self.assertEqual(self.phases(rec), ["look", "done"])  # 回していない
        self.assertIn("柱に近すぎるので腰をひねって探さない", ctl.outcome)
        self.assertIn("20 cm", ctl.outcome)
        self.assertIn("あと 4 cm 下がる", ctl.outcome)  # 0.24 - 0.20
        self.assertEqual(ctl.scan_tried, [])
        self.assertEqual({round(w, 9) for _, w, _ in rec}, {round(float(self.q0[12]), 9)})

    def test_no_depth_prevents_the_twist(self) -> None:
        ctl = self.controller()
        with mock.patch.object(ctl, "_front_distance", return_value=(None, 12)):
            rec, done = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[10.0]))
        self.assertTrue(done)
        self.assertEqual(self.phases(rec), ["look", "done"])
        self.assertIn("確かめられない", ctl.outcome)
        self.assertIn("12 個", ctl.outcome)

    def test_exactly_clear_distance_allows_the_twist(self) -> None:
        ctl = self.controller()
        with self.near_front(ctl, dist=float(ctl.cfg["press"]["waist_scan"]["clear_x_m"])):
            rec, _ = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[10.0]), stop="reach")
        self.assertIn("turn", self.phases(rec))

    def test_without_camera_transform_it_cannot_check(self) -> None:
        """finder に頭カメラの変換が無ければ、確かめられないのでひねらない（安全側）。"""
        ctl = self.controller()
        rec, done = self.run_loop(ctl, FakeFinder(make_call(0.40, -0.14), found_deg=[10.0]))
        self.assertTrue(done)
        self.assertIn("確かめられない", ctl.outcome)


@needs("mujoco", "pin")
class TestClearXAndDepth(unittest.TestCase):
    """clear_x_m の根拠（モデルから出した上体の前端 + 5 cm）と、深度からの距離。"""

    @classmethod
    def setUpClass(cls) -> None:
        import sys

        sys.path.insert(0, str(FEATURE_DIR / "sim"))
        from reach_map import REAL_STAND_1021

        cls.q0 = REAL_STAND_1021.copy()
        cls.robot = load_config("robot.yaml")
        cls.scan = load_config("elevator.yaml")["press"]["waist_scan"]

    def test_clear_x_is_at_least_the_computed_front_plus_margin(self) -> None:
        from common.front_clearance import MARGIN_M, max_front_x, required_clear_x

        need = required_clear_x(self.robot, self.q0, float(self.scan["max_deg"]))
        self.assertGreaterEqual(float(self.scan["clear_x_m"]), need)
        self.assertLess(float(self.scan["clear_x_m"]), need + 0.02)  # 大きすぎない（切り上げ程度）
        self.assertAlmostEqual(need, max_front_x(self.robot, self.q0, [-20, -15, -10, -5, 0, 5, 10, 15, 20]) + MARGIN_M)
        # 腰をひねると前端は前へ出る（始めの姿勢より）
        self.assertGreater(max_front_x(self.robot, self.q0, [20]), max_front_x(self.robot, self.q0, [0]))

    def test_distance_from_a_plane_in_front(self) -> None:
        """pelvis の x = 0.30 m の平面の深度から、距離 0.30 m が出る。外れ値の点が混ざっても最小にならない。"""
        from common.camera_geometry import HeadCameraTransform
        from common.front_clearance import front_distance

        tf = HeadCameraTransform(self.robot)
        rot, pos = tf.pelvis_from_optical(np.zeros(3))
        h, w, fx = 360, 640, 454.0
        v, u = np.mgrid[0:h, 0:w]
        ray = np.stack([(u - w / 2) / fx, (v - h / 2) / fx, np.ones_like(u, dtype=float)], axis=-1) @ rot.T
        s = (0.30 - pos[0]) / ray[..., 0]
        depth = np.where((ray[..., 0] > 1e-3) & (s > 0.2), s, 0.0)
        Kc = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1.0]])
        args = ((0.0, 0.40), 0.35, 3.0, 300)
        dist, n = front_distance(depth, Kc, rot, pos, *args)
        self.assertIsNotNone(dist, n)
        self.assertAlmostEqual(dist, 0.30, places=2)
        depth[100:103, 300:303] = 0.25  # 飛んだ点
        dist2, _ = front_distance(depth, Kc, rot, pos, *args)
        self.assertAlmostEqual(dist2, 0.30, places=2)
        # 点が足りなければ確かめられない
        self.assertIsNone(front_distance(np.zeros_like(depth), Kc, rot, pos, *args)[0])


if __name__ == "__main__":
    unittest.main()
