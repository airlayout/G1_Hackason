"""エレベーターの呼びボタンを押す制御（50 Hz で 1 歩ずつ進める）。Yada の評価環境のエージェントの中身。

    ctl = ElevatorPressController.from_config(robot_cfg, press_cfg, arm_cfg, elevator_cfg)
    ctl.reset(target="up", control_dt=0.02, upper_kp=kp17)   # kp17: motor 12〜28 の PD の強さ
    q29, done = ctl.step(rgb, depth, K, q, t, image_t, imu_quat)   # 上半身の目標の角度（29 関節の並び）

J1-gen の部品をそのまま使う:
- 見つける: common/elevator_buttons.py（一般用の ▲▼、柱の面の向き、3 フレームでばらつきを確かめる）
- 計画: common/press_planner.py（IK、腕と体の衝突、作業空間の箱、押し込みの直線）
- 柱（壁）との衝突: 柱の面の位置から壁の箱を作り、開始 → 手前の姿勢の移動で腕が壁に当たらないかを確かめる。
  当たるなら、壁から離れた右側の経由点を通る（fungi-55 さんの push_button の経由点と同じ考え方）。
  押し込みの直線は壁（ボタン）に触れるのが目的なので、壁とは調べない（体との衝突は調べる）
- 重力: common/arm/gravity.py（IMU の向きを使う）。腕は弱い PD（kp 60）で重力を補償しないので、
  垂れる量（重力のトルク ÷ kp）だけ目標をずらす

段階:
  look → reach（関節空間で手前の姿勢へ。経由点があれば通る）→ settle（手前で止まり、指先のずれを補正）
  → press（押す向きに直線で押し込む。指先が進まなくなったらそこで止める）→ hold → retract（手前へ戻る）→ check（点灯したかを画像で見る。
  点灯していなければ 1 回だけ深く押し直す）→ home（来た経路を逆にたどって、始めの姿勢へ戻す）→ done

終わる前に腕を戻す理由: 評価環境は点灯した瞬間に終わるが、実機・模擬 G1 では、エージェントが終わると腕の
weight が 0 に戻り、腕が柱の数 cm 手前から元の姿勢へ動く。そのとき腕が柱に当たらないよう、エージェントの中で
戻しておく（fungi-55 さんの push_button では、5 cm 手前から関節空間でまっすぐ戻すと、指先が弧を描いて壁に
40 N で当たった。2026-10-08）。
戻る経路は、来た経路（始め →（経由の姿勢 →）手前の姿勢）の逆。来るときに、腕が体にも柱・壁にも当たらないことを
確かめてある。押す向きと逆にまっすぐ離れる方法は、肘がたたまれて右の手首が胴体に入った（柱まで 31 cm で、
面から 10 cm 以上離れると当たる。2026-10-14）。

指先のずれの補正（Yada の見本のエージェントと同じ考え方）: 実際の関節の角度から指先の位置を計算（FK）し、
目標とのずれを少しずつ足し込む。押す向きの成分は、押している間は足し込まない（押し込む力を出すため）。
ボタンに触れる直前で補正を止める（触れたあとは摩擦で横に動けず、補正だけが膨らむため）。
"""

from __future__ import annotations

import copy
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from .arm.gravity import GravityModel
from .collision import CollisionChecker
from .elevator_buttons import ButtonFinder, ButtonNotFound, ButtonTracker, CallButtons
from .press_planner import PressPlan, PressPlanner, UnreachableError
from .robot_model import WAIST_IDX

UPPER_IDX = list(range(12, 29))


def finger_axis_for(normal: np.ndarray, left_deg: float, up_deg: float) -> np.ndarray:
    """押す向き normal から、指の向きを決める（左・上に傾ける。右下から斜めに指を伸ばす）。"""
    n = np.asarray(normal, dtype=float)
    left = np.cross([0.0, 0.0, 1.0], n)
    left /= np.linalg.norm(left)
    up = np.cross(n, left)
    a, b = np.radians(left_deg), np.radians(up_deg)
    f = np.cos(b) * (np.cos(a) * n + np.sin(a) * left) + np.sin(b) * up
    return f / np.linalg.norm(f)


def wall_obstacle(call: CallButtons, cfg: dict[str, Any]) -> dict[str, Any]:
    """柱（とその奥の壁）を、柱の面の奥に置いた大きな箱として表す（pelvis 座標。箱は面の向きに回す）。

    最初は箱を pelvis の軸にそろえ、面が斜めの分だけ前面を手前に出していた。realistic では面の向きが
    3° ほど傾いて推定され、前面が 4 cm 手前に来て、届く経路まで「壁に当たる」になった（2026-10-07、
    練習用の 20 試行のうち 13 試行が動く前に中止）。そこで、面の向きに回した箱にした。
    """
    n = np.asarray(call.normal, dtype=float)
    c = 0.5 * (call.up.center + call.down.center)
    depth = float(cfg["thickness_m"])
    center = c + (float(cfg["margin_m"]) + depth / 2) * n
    return {"name": "wall", "center": center.tolist(), "x_axis": n.tolist(),
            "half_size": [depth / 2, float(cfg["half_width_m"]), float(cfg["half_height_m"])]}


@dataclass
class StepLog:
    phase: str
    message: str


class ElevatorPressController:
    def __init__(self, robot_cfg: dict[str, Any], planner: PressPlanner, finder: ButtonFinder,
                 gravity: GravityModel, cfg: dict[str, Any]) -> None:
        self.robot_cfg = robot_cfg
        self.planner = planner
        self.kin = planner.kin
        self.finder = finder
        self.gravity = gravity
        self.cfg = cfg
        self.tracker = ButtonTracker(cfg["tracking"])
        self.log: list[StepLog] = []
        # 押す前（見つけたとき）と点灯の確認のときの画像を、外へ渡す口（agent.py が記録に使う）。
        # on_snapshot(種類 "before" / "check", 画像 RGB, 内容の辞書)。無ければ何もしない
        self.on_snapshot: Callable[[str, np.ndarray, dict[str, Any]], None] | None = None

    @classmethod
    def from_config(cls, robot_cfg: dict[str, Any], press_cfg: dict[str, Any], arm_cfg: dict[str, Any],
                    elevator_cfg: dict[str, Any]) -> "ElevatorPressController":
        """press_cfg（configs/press.yaml）の press と ik を、elevator_cfg の press と ik で上書きして使う。"""
        pc = copy.deepcopy(press_cfg)
        pc["press"].update(elevator_cfg["press"].get("planner", {}))
        pc["ik"].update(elevator_cfg.get("ik", {}))
        planner = PressPlanner.from_config(robot_cfg, pc, arm_cfg, "right")
        finder = ButtonFinder.from_config(robot_cfg, elevator_cfg)
        return cls(robot_cfg, planner, finder, GravityModel(robot_cfg), elevator_cfg)

    # ---- 外から呼ぶ ------------------------------------------------------------------

    def reset(self, target: str, control_dt: float, upper_kp: np.ndarray) -> None:
        if target not in ("up", "down"):
            raise ValueError(f"押すボタンは up / down: {target}")
        self.target = target
        self.dt = float(control_dt)
        self.kp = np.asarray(upper_kp, dtype=float)  # 上半身 17 関節（motor 12〜28）の PD の強さ
        if self.kp.shape != (len(UPPER_IDX),):
            raise ValueError(f"upper_kp は 17 個: {self.kp.shape}")
        self.phase, self.t_phase = "look", 0.0
        self.lag0: float | None = None  # 押し込みを始めた周期の指令の遅れ（_enter("press") で測り直す）
        self.tracker.reset()
        self.last_image_t = -np.inf
        self.q_hold: np.ndarray | None = None
        self.q_ik: np.ndarray | None = None
        self.call: CallButtons | None = None
        self.plan_: PressPlan | None = None
        self.legs: list[tuple[np.ndarray, np.ndarray, float]] = []
        self.corr = np.zeros(3)
        self.retries = 0
        self.wall: CollisionChecker | None = None
        self.home_legs: list[tuple[np.ndarray, np.ndarray, float]] = []
        self.baseline_brightness: float | None = None
        self.baseline_level: float | None = None
        self.outcome = ""
        self.log = []

    def step(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, q: np.ndarray, t: float,
             image_t: float, imu_quat: np.ndarray | None = None) -> tuple[np.ndarray, bool]:
        """1 周期分。上半身の目標の角度（29 関節の並び。下半身は今の角度のまま）と、終わったかを返す。"""
        q = np.asarray(q, dtype=float)
        if self.q_hold is None:
            self.q_hold = q.copy()
        q_des = self.q_hold.copy()
        arm = self.kin.arm_idx
        el = t - self.t_phase
        c = self.cfg["press"]
        done = False
        fresh = self._is_fresh(t, image_t)

        if self.phase == "look":
            if t >= float(c["look_start_s"]) and fresh:
                self._look(rgb, depth, K, q, t)
            if self.phase == "look" and t > float(c["look_start_s"]) + float(c["look_timeout_s"]):
                st = c["stand"]
                done = self._give_up(t, "時間内に一般用の ▲▼ を見つけられない"
                                     + (f"（最後の理由: {self._last_reason}）" if getattr(self, "_last_reason", "") else "")
                                     + f"。立つ位置を確かめる: 柱まで {float(st['min_dist_m']) * 100:.0f}〜"
                                     f"{float(st['max_dist_m']) * 100:.0f} cm、左右 {float(st['max_side_m']) * 100:.0f} cm 以内"
                                     "（遠いと高い ▲ が頭カメラの上に切れる）")
        elif self.phase == "reach":
            q_des[arm] = self._reach(el, t)
        elif self.phase == "settle":
            q_des[arm] = self._servo(q, self.plan_.approach_point, integrate_push=True)
            if el >= float(c["settle_s"]):
                self._enter("press", t)
        elif self.phase == "press":
            T = float(c["press_s"])
            s = smoothstep(el / T)
            p = (1 - s) * self.plan_.approach_point + s * self.plan_.end_point
            lag = self._push_lag(q)
            if self.lag0 is None:
                self.lag0 = lag  # 押す前からある遅れ（実機の重力補償 0.5 で 4.5 mm）。増え方で止める
            if lag - self.lag0 > self._stop_lag():
                p = self._stop_press(p, lag, t)
                q_des[arm] = self._servo(q, p, integrate_push=False)
            else:
                q_des[arm] = self._servo(q, p, integrate_push=False)
                if s >= 1.0:
                    self._enter("hold", t)
        elif self.phase == "hold":
            q_des[arm] = self._servo(q, self.plan_.end_point, integrate_push=False)
            if el >= float(c["hold_s"]):
                self._enter("retract", t)
        elif self.phase == "retract":
            s = smoothstep(el / float(c["retract_s"]))
            p = (1 - s) * self.plan_.end_point + s * self.plan_.approach_point
            q_des[arm] = self._servo(q, p, integrate_push=False)
            if s >= 1.0:
                self._enter("check", t)
        elif self.phase == "check":
            q_des[arm] = self._servo(q, self.plan_.approach_point, integrate_push=False)
            if el >= float(c["check_wait_s"]) and fresh:
                self._check(rgb, t)
        elif self.phase == "home":
            q_des[arm] = self._play(self.home_legs, el)
            if el >= sum(T for _, _, T in self.home_legs):
                self.q_ik[arm] = self.home_legs[-1][1]
                self._note("腕を始めの姿勢へ戻した")
                self._enter("done", t)
        elif self.phase == "done":
            q_des[arm] = self.q_ik[arm] if self.q_ik is not None else q_des[arm]
            done = True

        return self._gravity_offset(q_des, imu_quat), done

    def _is_fresh(self, t: float, image_t: float) -> bool:
        """新しい画像か。撮った時刻（image_t）が進んでいれば新しい。

        撮った時刻が渡されない（0 のまま）ときは、image_interval_s ごとに新しいとみなす。評価環境の実機用の口
        （Yada の contest/robots/real_g1.py。模擬 G1 と実機）は image_t を入れず、常に 0 になる。
        image_t だけで判断すると最初の 1 枚しか使えず、3 フレームそろわずに中止した（2026-10-07、模擬 G1 の smoke）。
        """
        if image_t > 0.0:
            if image_t > self.last_image_t:
                self.last_image_t = image_t
                return True
            return False
        if t - self.last_image_t >= float(self.cfg["press"]["image_interval_s"]) - 1e-6:
            self.last_image_t = t
            return True
        return False

    # ---- 段階 ------------------------------------------------------------------

    def _note(self, msg: str) -> None:
        self.log.append(StepLog(self.phase, msg))
        print(f"[elevator] {self.phase}: {msg}")

    def _enter(self, phase: str, t: float) -> None:
        self.phase, self.t_phase = phase, t
        if phase == "press":
            self.lag0 = None  # 押し込みを始めた周期の遅れを、次の step で測り直す

    def _give_up(self, t: float, why: str) -> bool:
        self.outcome = f"中止: {why}"
        self._note(self.outcome)
        self._enter("done", t)
        return True

    def _look(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, q: np.ndarray, t: float) -> None:
        try:
            call = self.finder.find(rgb, depth, K, q[list(WAIST_IDX)])
        except ButtonNotFound as e:
            self._last_reason = str(e)
            self.tracker.reset()
            return
        merged = self.tracker.add(call)
        if merged is None:
            return
        self.call = merged
        self._note(f"見つけた: {merged.summary()}")
        b = merged.button(self.target)
        self.baseline_brightness = self._brightness(rgb, b.bbox)
        self.baseline_level = self._lit_level(rgb, b.bbox)
        self._snapshot("before", rgb, b.bbox, t, level=self.baseline_level, retry=0)
        advice = self._stand_advice(merged)
        try:
            self._plan(q, merged)
        except UnreachableError as e:
            self._give_up(t, (f"{advice}。" if advice else "") + f"計画できない: {e}")
            return
        if advice:
            self._note(f"⚠️ {advice}（計画は立ったので押す）")
        self._enter("reach", t)

    def _stand_advice(self, call: CallButtons) -> str:
        """立っている位置が勧める範囲（press.stand）の外なら、どちらへ何 cm 動けばよいかの文。範囲の中なら空。

        距離は pelvis から柱の面（▲▼ の中点を通り、押す向きに垂直な面）まで、押す向きに測る。左右は ▲▼ の中点の
        pelvis の y（左が +）。範囲は、計画だけの網羅と評価環境での押し込みで確かめたもの（2026-10-15）。
        範囲の外でも押せることは多い（推定の位置は realistic で押す向きに最大 19 mm ずれる）ので、この文で止めはしない。
        計画できなかったときの理由に添え、計画できたときは注意として出す。
        """
        st = self.cfg["press"]["stand"]
        mid = (call.up.center + call.down.center) / 2.0
        dist, side = float(mid @ call.normal), float(mid[1])
        lo, hi, w = float(st["min_dist_m"]), float(st["max_dist_m"]), float(st["max_side_m"])
        def cm(m: float) -> int:  # 動く量は切り上げ（「あと 0 cm」と出さない）
            return max(1, int(np.ceil(m * 100 - 1e-6)))

        msgs = []
        if dist < lo:
            msgs.append(f"柱まで {dist * 100:.1f} cm と近すぎる。あと {cm(lo - dist)} cm 以上下がる")
        elif dist > hi:
            msgs.append(f"柱まで {dist * 100:.1f} cm と遠すぎる。あと {cm(dist - hi)} cm 以上近づく")
        if abs(side) > w:
            way = "左" if side > 0 else "右"
            msgs.append(f"ボタンが体の中心から{way}に {abs(side) * 100:.1f} cm。{way}へ {cm(abs(side) - w)} cm 以上寄る")
        if not msgs:
            return ""
        return ("立つ位置が勧める範囲の外: " + "、".join(msgs)
                + f"（勧める範囲: 柱まで {lo * 100:.0f}〜{hi * 100:.0f} cm、左右 {w * 100:.0f} cm 以内）")

    def _plan(self, q: np.ndarray, call: CallButtons, depth: float | None = None) -> None:
        c = self.cfg["press"]
        n = call.normal
        face = call.button(self.target).center
        # 中指の先（の球の中心）の目標: 球の表面がボタンの面に触れる位置
        touch = face - float(c["tip_radius_m"]) * n
        seed = q.copy()
        seed[self.kin.arm_idx] = np.asarray(c["ik_seed_right_arm"], dtype=float)
        approach = touch - float(self.planner.cfg["approach_distance_m"]) * n
        # 姿勢を選ぶため、まず位置だけで解いてから、指の向きも合わせる（Yada の見本と同じ 2 段）
        r0 = self.kin.ik(approach, seed)
        if r0.success:
            seed = r0.q
        wall = CollisionChecker(self.robot_cfg, "right", float(c["wall_clearance_m"]),
                                [wall_obstacle(call, self.cfg["wall"])])
        self.wall = wall
        errors: list[str] = []
        # 指の傾きは候補を順に試す（柱が近く面が斜めのとき、最初の傾きでは手首が胴体に近づきすぎた。
        # 2026-10-07、realistic の seed 8）
        for left_deg, up_deg in c["finger_tilts_deg"]:
            finger = finger_axis_for(n, float(left_deg), float(up_deg))
            for via in [None] + list(self._via_points(touch, n)):
                q_via = None
                what = f"指 {left_deg:g}°/{up_deg:g}°、" + (
                    "経由点なし" if via is None else f"経由点 {np.round(via, 3).tolist()}")
                if via is not None:
                    # 経由点でも指を手前の姿勢と同じ向きにする（位置だけで解くと、手首を大きく曲げた解になり、
                    # 関節リミットや胴体に当たった。2026-10-07、評価環境の seed 1 の ▼）
                    rv = self.kin.ik(via, seed)
                    rv = self.kin.ik(via, rv.q if rv.success else seed, finger)
                    if not rv.success:
                        errors.append(f"{what}: 経由点に届かない（{rv.reason}）")
                        continue
                    q_via = rv.q[self.kin.arm_idx]
                try:
                    plan = self.planner.plan(q, touch, n, q_seed=seed, depth=depth, q_via_arm=q_via,
                                             finger_axis=finger)
                    self._check_wall(wall, q, plan)
                except UnreachableError as e:
                    errors.append(f"{what}: {e}")
                    continue
                self.plan_ = plan
                self._build_legs(q, plan)
                self._note(f"計画（{what}）: {plan.summary()}")
                return
        raise UnreachableError(" / ".join(errors))

    def _via_points(self, touch: np.ndarray, n: np.ndarray):
        """壁から離れた、体の右側の経由点（手前の姿勢へ、壁をこすらずに入るため）。"""
        v = self.cfg["press"]["via"]
        right = np.cross(n, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        for back, side in v["offsets_m"]:
            p = touch - float(back) * n + float(side) * right
            p[2] = max(p[2], float(v["min_height_m"]))
            yield p

    def _check_wall(self, wall: CollisionChecker, q: np.ndarray, plan: PressPlan) -> None:
        """開始 →（経由 →）手前の姿勢の移動と手前の姿勢で、腕が柱・壁に当たらないか。"""
        arm = self.kin.arm_idx
        pts = [q[arm]] + ([plan.q_via] if plan.q_via is not None else []) + [plan.q_approach]
        n = int(self.cfg["wall"]["path_samples"])
        for leg, (a, b) in enumerate(zip(pts[:-1], pts[1:]), start=1):
            for k, s in enumerate(np.linspace(0.0, 1.0, n + 1)[1:], start=1):
                qq = q.copy()
                qq[arm] = a + smoothstep(float(s)) * (b - a)
                hits = wall.contacts(qq)
                if hits:
                    raise UnreachableError(f"手前の姿勢へ動く途中（区間 {leg}/{len(pts) - 1} の {k}/{n}）で腕が柱・壁に"
                                           "当たる: " + "; ".join(str(h) for h in hits[:2]))

    def _build_legs(self, q: np.ndarray, plan: PressPlan) -> None:
        arm = self.kin.arm_idx
        speed = float(self.cfg["press"]["move_speed_rad_s"])
        pts = [q[arm].copy()] + ([plan.q_via] if plan.q_via is not None else []) + [plan.q_approach]
        self.legs = []
        for a, b in zip(pts[:-1], pts[1:]):
            T = max(float(self.cfg["press"]["min_move_s"]), 1.5 * float(np.abs(b - a).max()) / speed)
            self.legs.append((a, b, T))
        self.q_ik = q.copy()
        self.q_ik[arm] = plan.q_approach

    def _reach(self, el: float, t: float) -> np.ndarray:
        if el >= sum(T for _, _, T in self.legs):
            self._enter("settle", t)
        return self._play(self.legs, el)

    @staticmethod
    def _play(legs: list[tuple[np.ndarray, np.ndarray, float]], el: float) -> np.ndarray:
        """関節空間の区間を順にたどる（各区間は smoothstep）。最後を過ぎたら最後の姿勢。"""
        for a, b, T in legs:
            if el < T:
                return a + smoothstep(el / T) * (b - a)
            el -= T
        return legs[-1][1]

    def _path_hits(self, q: np.ndarray, pts: list[np.ndarray]) -> bool:
        if self.wall is None:
            return False
        arm = self.kin.arm_idx
        n = int(self.cfg["wall"]["path_samples"])
        for a, b in zip(pts[:-1], pts[1:]):
            for s in np.linspace(0.0, 1.0, n + 1)[1:]:
                qq = q.copy()
                qq[arm] = a + smoothstep(float(s)) * (b - a)
                if self.wall.contacts(qq):
                    return True
        return False

    def _servo(self, q: np.ndarray, p_goal: np.ndarray, integrate_push: bool) -> np.ndarray:
        """指先を p_goal に合わせる右腕 7 関節。実際の指先とのずれを積分して目標に足す。"""
        c = self.cfg["press"]["servo"]
        n = self.plan_.push_dir
        tip = self.kin.fk_pos(q)
        if float((self.plan_.target_point - tip) @ n) > float(c["freeze_m"]):
            e = p_goal - tip
            if not integrate_push:
                e = e - (e @ n) * n
            self.corr = np.clip(self.corr + float(c["gain"]) * self.dt * e, -float(c["max_m"]), float(c["max_m"]))
        axis = self.plan_.finger_axis
        r = self.kin.ik(p_goal + self.corr, self.q_ik, axis, max_iter=50)
        if np.isfinite(r.pos_err) and r.pos_err < float(c["accept_err_m"]):
            self.q_ik = r.q
        return self.q_ik[self.kin.arm_idx]

    def _push_lag(self, q: np.ndarray) -> float:
        """指令した指先（前の周期の目標の角度の FK）が、実際の指先より押す向きにどれだけ先にあるか [m]。"""
        n = self.plan_.push_dir
        return float((self.kin.fk_pos(self.q_ik) - self.kin.fk_pos(q)) @ n)

    def _stop_lag(self) -> float:
        """押し込みを止める遅れの増え方 [m]（押し込みを始めたときの遅れからの増分）。押し直しのときは大きく（強く押す）。"""
        cs = self.cfg["press"]["contact_stop"]
        return float(cs["retry_lag_m"] if self.retries > 0 else cs["lag_m"])

    def _stop_press(self, p: np.ndarray, lag: float, t: float) -> np.ndarray:
        """指先が進まなくなった（ボタンが底に着いた・何かに当たった）。押し込み終わりを、実際の指先の
        keep_m 先に置き直して保持へ移る。返すのは新しい押し込み終わり。

        目標を奥に置いたまま押し続けると、遅れ（＝押す力）が増え続け、指先がボタンの面を横に滑って柱の穴の縁に
        当たった（2026-10-15、realistic の seed 0 で 21.6 N。底に着いてから遅れが 26 mm まで増え、横に約 9 mm 滑った）。
        """
        keep = float(self.cfg["press"]["contact_stop"]["keep_m"])
        n = self.plan_.push_dir
        end = p - (lag - keep) * n
        self.plan_ = PressPlan(**{**self.plan_.__dict__, "end_point": end})
        self._note(f"指先が進まなくなった（遅れ {lag * 1000:.1f} mm）。押し込みを止めて保持する")
        self._enter("hold", t)
        return end

    def _brightness(self, rgb: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = bbox
        patch = np.asarray(rgb[y1:y2, x1:x2], dtype=float)
        return float(patch.mean()) if patch.size else float("nan")

    def _lit_level(self, rgb: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
        """ボタンの明るい部分の明るさ: 枠に内接する楕円を lit_check.radius 倍に広げた範囲の、明るい側から
        lit_check.percentile % の値（0〜255）。

        ラボのエレベーター（2026-10-15、スマホの写真 5 枚）は、点灯すると縁の細い輪だけが光り、面と矢印は変わらない。
        枠全体の平均では輪の光が薄まり、上がり方が +21〜+31（以前のしきい値 +25 でぎりぎり）だった。明るい側の値なら
        +51〜+95。評価環境（面全体が白くなる）では +39〜+55（点灯していないときは 1 以下）。縁の輪の平均は、
        評価環境で黒い盤が混ざって +18〜+34 に薄まったので使わない。
        """
        lc = self.cfg["press"]["lit_check"]
        x1, y1, x2, y2 = bbox
        cx, cy, rx, ry = (x1 + x2) / 2, (y1 + y2) / 2, max((x2 - x1) / 2, 1), max((y2 - y1) / 2, 1)
        ro = float(lc["radius"])
        h, w = rgb.shape[:2]
        a1, b1 = max(int(np.floor(cx - ro * rx)), 0), max(int(np.floor(cy - ro * ry)), 0)
        a2, b2 = min(int(np.ceil(cx + ro * rx)), w), min(int(np.ceil(cy + ro * ry)), h)
        if a2 <= a1 or b2 <= b1:
            return float("nan")
        v, u = np.mgrid[b1:b2, a1:a2]
        inside = np.hypot((u + 0.5 - cx) / rx, (v + 0.5 - cy) / ry) <= ro
        gray = np.asarray(rgb[b1:b2, a1:a2], dtype=float).mean(axis=2)
        return float(np.percentile(gray[inside], float(lc["percentile"]))) if inside.any() else float("nan")

    def _current_bbox(self, rgb: np.ndarray, bbox: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        """見つけたときの枠に最も近い、今の画像のボタンの枠。無ければ、見つけたときの枠のまま。

        腕を動かすと体が揺れて頭カメラの画像がずれ、見つけたときの枠がボタンから外れた（2026-10-14、realistic の
        seed 10 で約 25 px ずれ、枠がボタンの半分しか覆わず、点灯していたのに明るさ +1 → 押し直し・見落とし）。
        近さは中心の距離（lit_track.max_shift_px まで）、大きさは幅・高さの比（lit_track.max_size_ratio まで）で見る。
        """
        lt = self.cfg["press"]["lit_track"]
        x1, y1, x2, y2 = bbox
        cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, max(x2 - x1, 1), max(y2 - y1, 1)
        r = float(lt["max_size_ratio"])
        best, best_d = None, float(lt["max_shift_px"])
        for cand in self.finder.detector.detect(rgb):
            a1, b1, a2, b2 = cand.bbox
            if not (1 / r <= (a2 - a1) / w <= r and 1 / r <= (b2 - b1) / h <= r):
                continue
            d = float(np.hypot((a1 + a2) / 2 - cx, (b1 + b2) / 2 - cy))
            if d <= best_d:
                best, best_d = cand.bbox, d
        return best if best is not None else bbox

    def _snapshot(self, kind: str, rgb: np.ndarray, bbox: tuple[int, int, int, int], t: float, **info: Any) -> None:
        """on_snapshot があれば、そのときの画像と、使った枠・明るさなどを渡す（実機で、手が枠に被さって点灯の確認が
        狂っていないかを、あとで画像で見るため）。記録の失敗で押す動きを止めない。"""
        if self.on_snapshot is None:
            return
        try:
            self.on_snapshot(kind, rgb, {"t": float(t), "target": self.target, "bbox": [int(v) for v in bbox],
                                         "baseline_level": self.baseline_level, **info})
        except Exception as e:  # noqa: BLE001
            print(f"[elevator] ⚠️ 画像の記録に失敗（続ける）: {e}")

    def _is_lit(self, level: float, rise: float) -> bool:
        """点灯したか: 押す前からの上がり方が min_rise 以上、または、明るさが abs_level 以上で上がり方が abs_min_rise 以上。

        後ろは、押す前から金属の面が反射で白い（194〜215）ボタン用。上がり方が +10〜15 にしかならず、前だけだと点灯しても
        「点灯していない」になる（2026-10-19、スマホの写真 57 枚: 点灯 224〜230、消灯でも反射で 215 まで）。
        押す前から abs_level 以上のままで上がらない（rise が abs_min_rise 未満）なら、点灯とは見ない。
        """
        lc = self.cfg["press"]["lit_check"]
        if rise >= float(lc["min_rise"]):
            return True
        return bool(level >= float(lc["abs_level"]) and rise >= float(lc["abs_min_rise"]))

    def _check(self, rgb: np.ndarray, t: float) -> bool:
        """押したボタンが点灯したか（明るい部分の明るさが、見つけたときより lit_check.min_rise 以上上がったか。
        反射で白いボタンは _is_lit の第 2 の条件）を見る。
        点灯していなければ深く押し直す。枠全体の明るさの上がり方も、比べるために表示する。"""
        c = self.cfg["press"]
        b = self.call.button(self.target)
        box = self._current_bbox(rgb, b.bbox)
        level = self._lit_level(rgb, box)
        rise = level - (self.baseline_level if self.baseline_level is not None else level)
        whole = self._brightness(rgb, box) - (self.baseline_brightness if self.baseline_brightness is not None else 0.0)
        shown = f"明るい部分 {rise:+.0f}、枠全体の平均 {whole:+.0f}"
        self._snapshot("check", rgb, box, t, level=level, retry=self.retries, rise=rise, whole=whole,
                       lit=self._is_lit(level, rise), found_box=b.bbox, moved=box != b.bbox)
        if self._is_lit(level, rise):
            self.outcome = "点灯した"
            self._note(f"点灯した（{shown}）")
            self._start_home(t)
            return True
        if self.retries >= int(c["max_retries"]):
            self.outcome = f"点灯を確かめられない（{shown}）"
            self._note(self.outcome)
            self._start_home(t)
            return True
        self.retries += 1
        depth = min(self.plan_.depth + float(c["retry_extra_depth_m"]), float(self.planner.cfg["max_press_depth_m"]))
        self._note(f"点灯していない（{shown}）。{depth * 1000:.0f} mm で押し直す")
        p = self.plan_
        self.plan_ = PressPlan(**{**p.__dict__, "end_point": p.target_point + depth * p.push_dir, "depth": depth})
        self._enter("press", t)
        return False

    def _start_home(self, t: float) -> None:
        """押し終わった（点灯した・確かめられない）。来た経路を逆にたどって、始めの姿勢へ戻す。

        今の右腕（指先のずれの補正で、計画した手前の姿勢から少しずれている）から、まず計画した手前の姿勢へ戻り、
        そのあと来た区間を逆にたどる。今の姿勢から手前の姿勢へ戻る区間も、柱・壁と体に当たらないことを確かめる。
        当たるなら、今の姿勢のまま終わる。
        """
        arm = self.kin.arm_idx
        speed = float(self.cfg["press"]["move_speed_rad_s"])
        now = self.q_ik[arm].copy()
        first = (now, self.legs[-1][1], max(float(self.cfg["press"]["min_move_s"]),
                                           1.5 * float(np.abs(self.legs[-1][1] - now).max()) / speed))
        q = self.q_hold.copy()
        q[arm] = now
        if self._path_hits(q, [now, self.legs[-1][1]]):
            self._note("⚠️ 手前の姿勢へ戻ると腕が当たるので、今の姿勢のまま終わる")
            self._enter("done", t)
            return
        self.home_legs = [first] + [(b, a, T) for a, b, T in reversed(self.legs)]
        self._note(f"来た経路を逆にたどって、始めの姿勢へ戻す（{sum(T for _, _, T in self.home_legs):.1f} 秒）")
        self._enter("home", t)

    def _gravity_offset(self, q_des: np.ndarray, imu_quat: np.ndarray | None) -> np.ndarray:
        """PD が重力で垂れる量（重力のトルク ÷ kp）だけ、上半身の目標をずらす。"""
        tau = self.gravity.torques(q_des, imu_quat)
        out = q_des.copy()
        kp = np.where(self.kp > 0, self.kp, np.inf)
        out[UPPER_IDX] = q_des[UPPER_IDX] + tau[UPPER_IDX] / kp
        return out


def smoothstep(s: float) -> float:
    s = min(max(float(s), 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)
