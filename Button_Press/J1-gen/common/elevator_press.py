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
  → press（押す向きに直線で押し込む）→ hold → retract（手前へ戻る）→ check（点灯したかを画像で見る。
  点灯していなければ 1 回だけ深く押し直す）→ done

指先のずれの補正（Yada の見本のエージェントと同じ考え方）: 実際の関節の角度から指先の位置を計算（FK）し、
目標とのずれを少しずつ足し込む。押す向きの成分は、押している間は足し込まない（押し込む力を出すため）。
ボタンに触れる直前で補正を止める（触れたあとは摩擦で横に動けず、補正だけが膨らむため）。
"""

from __future__ import annotations

import copy
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
        self.tracker.reset()
        self.last_image_t = -np.inf
        self.q_hold: np.ndarray | None = None
        self.q_ik: np.ndarray | None = None
        self.call: CallButtons | None = None
        self.plan_: PressPlan | None = None
        self.legs: list[tuple[np.ndarray, np.ndarray, float]] = []
        self.corr = np.zeros(3)
        self.retries = 0
        self.baseline_brightness: float | None = None
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
                done = self._give_up(t, "時間内に一般用の ▲▼ を見つけられない"
                                     + (f"（最後の理由: {self._last_reason}）" if getattr(self, "_last_reason", "") else ""))
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
                done = self._check(rgb, t)
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
        try:
            self._plan(q, merged)
        except UnreachableError as e:
            self._give_up(t, f"計画できない: {e}")
            return
        self._enter("reach", t)

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
        for a, b, T in self.legs:
            if el < T:
                return a + smoothstep(el / T) * (b - a)
            el -= T
        self._enter("settle", t)
        return self.legs[-1][1]

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

    def _brightness(self, rgb: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = bbox
        patch = np.asarray(rgb[y1:y2, x1:x2], dtype=float)
        return float(patch.mean()) if patch.size else float("nan")

    def _check(self, rgb: np.ndarray, t: float) -> bool:
        """押したボタンが点灯したか（同じ枠の明るさが上がったか）を見る。点灯していなければ深く押し直す。"""
        c = self.cfg["press"]
        b = self.call.button(self.target)
        now = self._brightness(rgb, b.bbox)
        rise = now - (self.baseline_brightness if self.baseline_brightness is not None else now)
        if rise >= float(c["lit_brightness_rise"]):
            self.outcome = "点灯した"
            self._note(f"点灯した（明るさ +{rise:.0f}）")
            self._enter("done", t)
            return True
        if self.retries >= int(c["max_retries"]):
            self.outcome = f"点灯を確かめられない（明るさ {rise:+.0f}）"
            self._note(self.outcome)
            self._enter("done", t)
            return True
        self.retries += 1
        depth = min(self.plan_.depth + float(c["retry_extra_depth_m"]), float(self.planner.cfg["max_press_depth_m"]))
        self._note(f"点灯していない（明るさ {rise:+.0f}）。{depth * 1000:.0f} mm で押し直す")
        p = self.plan_
        self.plan_ = PressPlan(**{**p.__dict__, "end_point": p.target_point + depth * p.push_dir, "depth": depth})
        self._enter("press", t)
        return False

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
