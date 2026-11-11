"""人がリモコンで柱の手前（約 0.6〜1.5 m）に置いたあと、頭カメラだけで押せる位置まで寄る（観測 → 1 回動く → 止まる → 観測）。

    perception = HeadPerception(get_frame, button_finder, face_finder, cfg)
    feasibility = PlanFeasibility.from_config(robot_cfg, press_cfg, arm_cfg, elevator_cfg, arms=["right"], target="up")
    result = Approacher(perception, feasibility, mover, cfg).run()
    result.outcome, result.arm          # "ready"（押せる。auto_press をこの腕で）/ "aborted" / "dry_run"

1 回の流れ:
1. 観測する（HeadPerception）: 新しいフレームで、まず ▲▼（common/elevator_buttons.py）を探す。3 フレームそろえば
   ボタンの位置から距離・向き・左右を出す。見えなければ柱の面（common/column_face.py）から距離・向きだけを出す
   （遠いとボタンは頭カメラの上に外れる）
2. ボタンが見えていて、押し方の候補のどれかで計画が立てば止まる（ready）。決まった範囲ではなく計画で決めるので、
   押し方を選べば押せる位置なら止まれる（PlanFeasibility。押すときと同じ候補を試す。腕は動かさない）
3. 立たなければ、どちらへ動くかを決める（decide）。順番は 向き → 左右（▲▼ が見えたら）→ 前。
   - 面しか見えないうちは face_stop_dist_m（▲▼ が見え始める距離）までしか進まない。▲▼ が見えてから左右を直すので、
     その余地を残す（純正の歩行は進む向きが 18〜20° ずれるので、横に動くと前にも出る）
   - 面しか見えないうちも、柱の端が見えていれば、最初に見えた端の位置（人が置いた左右の位置）を覚えておき、
     前に進んでずれたら横に戻す（1 m 進むと、向きのずれで横に 30 cm 以上ずれることがある）
   - 1 回で動ける最小（common/loco.py。前は約 11 cm）より十分小さいずれは直さない
   - どの 1 回分も、多めに動き向きもずれたときの前進（step_uncertainty・max_heading_error_deg）を見積もり、
     それでも min_safe_dist_m より近づくなら送らない。前進は勧める範囲（press.stand）の近い側に forward_margin_m を
     残して止める。後ろへは動かない（既定。近すぎたら人がリモコンで下げる）
4. 1 回分歩いて、脚が止まるのを待ち、1 に戻る（Approacher）。回数の上限で止まる

腕は動かさない（下ろしたまま）。押すのは今までどおり real/auto_press.py（ready の腕で）。
"""

from __future__ import annotations

import copy
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from .column_face import ColumnFace, FaceFinder, FaceNotFound, FaceTracker
from .elevator_buttons import ButtonFinder, ButtonNotFound, ButtonTracker, CallButtons
from .loco import LocoError, LocoMover, Pulse
from .robot_model import WAIST_IDX


@dataclass
class Frame:
    """1 枚分の観測。rgb は RGB、depth は m、q は 29 関節（motor 番号順）、imu_quat は (w, x, y, z) か None。"""

    rgb: np.ndarray
    depth: np.ndarray
    K: np.ndarray
    q: np.ndarray
    imu_quat: np.ndarray | None = None


@dataclass
class Measurement:
    """柱に対する今の立ち位置（pelvis 基準）。"""

    source: str               # buttons（▲▼ が見えた）/ face（柱の面だけ）
    distance_m: float         # pelvis から柱の面まで（押す向きに測る）
    yaw_rad: float            # 面に正対するのに回る角度（+ は左回り）
    side_m: float | None      # ▲▼ の中点の左右（pelvis の y、+ が左）。面だけのときは None
    q: np.ndarray | None = None
    call: CallButtons | None = None
    face: ColumnFace | None = None
    # 柱の面の左右の端（面に沿って pelvis の足から測る。+ が左。common/column_face.py）。分からなければ None
    left_edge_m: float | None = None
    right_edge_m: float | None = None

    @classmethod
    def from_call(cls, call: CallButtons, q: np.ndarray | None) -> "Measurement":
        mid = (call.up.center + call.down.center) / 2.0
        n = call.normal  # 押す向き（柱に向かう）
        return cls("buttons", float(mid @ n), float(np.arctan2(n[1], n[0])), float(mid[1]), q, call=call)

    @classmethod
    def from_face(cls, face: ColumnFace, q: np.ndarray | None) -> "Measurement":
        return cls("face", face.distance_m, face.yaw_rad, None, q, face=face, left_edge_m=face.left_edge_m,
                   right_edge_m=face.right_edge_m)

    def summary(self) -> str:
        side = "" if self.side_m is None else f"、ボタンは体の{'左' if self.side_m > 0 else '右'} {abs(self.side_m) * 100:.1f} cm"
        what = "▲▼ から" if self.source == "buttons" else "柱の面から（▲▼ は見えない）"
        return f"{what}: 柱まで {self.distance_m * 100:.1f} cm、向き {np.degrees(self.yaw_rad):+.1f}°{side}"


@dataclass
class Decision:
    action: str                 # ready / move / abort
    reason: str
    pulse: Pulse | None = None
    arm: str | None = None


@dataclass
class ApproachResult:
    outcome: str                # ready / aborted / dry_run
    reason: str
    arm: str | None = None
    steps: list[dict[str, Any]] = field(default_factory=list)


class Perception(Protocol):
    last_reason: str

    def measure(self) -> Measurement | None: ...

    def reset(self) -> None: ...


class Feasibility(Protocol):
    arms: list[str]

    def check(self, m: Measurement) -> tuple[str | None, dict[str, str]]: ...

    def stand(self, arm: str) -> dict[str, Any]: ...


# ---- 観測 -----------------------------------------------------------------------


class HeadPerception:
    """頭カメラのフレームから、▲▼ か柱の面で立ち位置を求める。get_frame() は新しいフレームを 1 枚返す（届かなければ None）。"""

    def __init__(self, get_frame: Callable[[], Frame | None], buttons: ButtonFinder | None, face: FaceFinder,
                 elevator_cfg: dict[str, Any], face_cfg: dict[str, Any], cfg: dict[str, Any]) -> None:
        self.get_frame = get_frame
        self.buttons = buttons
        self.face = face
        self.button_tracker = ButtonTracker(elevator_cfg["tracking"])
        self.face_tracker = FaceTracker(face_cfg["tracking"])
        self.max_frames = int(cfg["max_frames"])
        self.skip_after_move = int(cfg["skip_frames_after_move"])
        self._skip = 0
        self.last_reason = ""

    def reset(self) -> None:
        """動いたあとに呼ぶ（そろえていたフレームを捨て、動いている間に撮られたかもしれない数枚を読み飛ばす）。"""
        self.button_tracker.reset()
        self.face_tracker.reset()
        self._skip = self.skip_after_move

    def measure(self) -> Measurement | None:
        face_m: Measurement | None = None
        reasons: list[str] = []
        for _ in range(self.max_frames + self._skip):
            fr = self.get_frame()
            if fr is None:
                self.last_reason = "頭カメラのフレームが届かない（RGB-D サーバ・接続を確かめる）"
                return None
            if self._skip > 0:
                self._skip -= 1
                continue
            qw = np.asarray(fr.q, dtype=float)[list(WAIST_IDX)]
            if self.buttons is not None:
                try:
                    merged = self.button_tracker.add(self.buttons.find(fr.rgb, fr.depth, fr.K, qw))
                    if merged is not None:
                        return Measurement.from_call(merged, np.asarray(fr.q, dtype=float).copy())
                except ButtonNotFound as e:
                    self.button_tracker.reset()
                    reasons.append(f"▲▼: {e}")
            try:
                fm = self.face_tracker.add(self.face.find(fr.depth, fr.K, qw, fr.imu_quat))
                if fm is not None:
                    face_m = Measurement.from_face(fm, np.asarray(fr.q, dtype=float).copy())
            except FaceNotFound as e:
                self.face_tracker.reset()
                reasons.append(f"柱の面: {e}")
        if face_m is not None:
            return face_m
        self.last_reason = (f"{self.max_frames} フレームで ▲▼ も柱の面もそろわない（最後の理由: "
                            + ("、".join(reasons[-2:]) if reasons else "ばらつきが大きい") + "）")
        return None


# ---- 押せるか -------------------------------------------------------------------


class PlanFeasibility:
    """押し方の候補のどれかで計画が立つかを、腕ごとに確かめる（common/elevator_press.py の can_plan）。"""

    def __init__(self, controllers: dict[str, Any], target: str) -> None:
        self.controllers = controllers
        self.arms = list(controllers)
        self.target = target

    @classmethod
    def from_config(cls, robot_cfg: dict[str, Any], press_cfg: dict[str, Any], arm_cfg: dict[str, Any],
                    elevator_cfg: dict[str, Any], arms: list[str], target: str) -> "PlanFeasibility":
        from .elevator_press import ElevatorPressController

        ctls = {}
        for arm in arms:
            cfg = copy.deepcopy(elevator_cfg)
            cfg["press"]["arm"] = arm
            ctls[arm] = ElevatorPressController.from_config(robot_cfg, press_cfg, arm_cfg, cfg)
        return cls(ctls, target)

    def check(self, m: Measurement) -> tuple[str | None, dict[str, str]]:
        """計画が立った最初の腕と、立たなかった腕ごとの理由。"""
        if m.call is None or m.q is None:
            return None, {a: "▲▼ が見えていない" for a in self.arms}
        reasons = {}
        for arm, ctl in self.controllers.items():
            why = ctl.can_plan(m.q, m.call, self.target)
            if why is None:
                return arm, reasons
            reasons[arm] = why
        return None, reasons

    def stand(self, arm: str) -> dict[str, Any]:
        """勧める立つ位置。center_side_m は押す腕に合わせた符号（左腕なら右腕の値の反対）。"""
        st = self.controllers[arm].stand_cfg()
        st["center_side_m"] = (1.0 if arm == "right" else -1.0) * float(st.get("center_side_m", 0.0))
        return st


# ---- どちらへ動くか -------------------------------------------------------------------


def edge_shift(m: Measurement, ref: dict[str, float]) -> float | None:
    """最初に覚えた柱の端（ref: left / right）から、今の端がどれだけ左へずれたか [m]（両方分かれば平均）。分からなければ None。

    体が右へずれると、端は左（+）へずれて見える。なので + なら左へ動けば戻る。
    """
    d = [now - ref[k] for k, now in (("left", m.left_edge_m), ("right", m.right_edge_m)) if now is not None and k in ref]
    return float(np.mean(d)) if d else None


def decide(m: Measurement, feasible_arm: str | None, reasons: dict[str, str], stands: dict[str, dict[str, Any]],
           cfg: dict[str, Any], mover: LocoMover, edge_ref: dict[str, float] | None = None) -> Decision:
    """観測から、止まる（ready）・1 回動く（move）・やめる（abort）を決める。

    stands は腕ごとの勧める立つ位置。edge_ref は最初に見えた柱の端（面しか見えないうちに左右を保つのに使う）。
    """
    d = m.distance_m
    if d < float(cfg["min_safe_dist_m"]):
        return Decision("abort", f"柱まで {d * 100:.1f} cm と近すぎる（{float(cfg['min_safe_dist_m']) * 100:.0f} cm 未満）。"
                                 "人がリモコンで下げる")
    if m.source == "buttons" and feasible_arm is not None:
        return Decision("ready", f"{feasible_arm} 腕で押す計画が立った", arm=feasible_arm)

    safe_d = float(cfg["min_safe_dist_m"])
    spill = np.sin(np.radians(float(cfg["max_heading_error_deg"])))
    unc = 1.0 + float(cfg["step_uncertainty"])

    def worst_forward(p: Pulse) -> float:
        """この 1 回で柱へ近づく量の見積もりの最大（多めに動き、向きもずれたとき）。回るのは 0 とみなす。"""
        if p.axis.startswith("turn"):
            return 0.0
        moved = float(mover.cfg["axes"][p.axis]["rate"]) * p.duration_s * unc
        return moved if p.axis == "forward" else moved * spill

    def is_safe(p: Pulse) -> bool:
        return d - worst_forward(p) >= safe_d - 1e-9

    # 向き
    yaw = m.yaw_rad
    if abs(np.degrees(yaw)) > float(cfg["yaw_tol_deg"]):
        p = mover.plan("turn_left" if yaw > 0 else "turn_right", abs(yaw))
        if p is not None:
            return Decision("move", f"柱に正対していない（{np.degrees(yaw):+.1f}°）。回る", p)

    # 腕を選ぶ（▲▼ が見えていれば、左右の目安に近い腕。見えなければ最初の腕）
    if m.side_m is not None:
        arm = min(stands, key=lambda a: abs(m.side_m - float(stands[a]["center_side_m"])))
    else:
        arm = next(iter(stands))
    st = stands[arm]
    lo, hi = float(st["min_dist_m"]), float(st["max_dist_m"])
    target = cfg.get("target_dist_m")
    target = float(target if target is not None else st.get("target_dist_m", (lo + hi) / 2.0))
    err = d - target
    step = mover.min_step("forward")
    margin = float(cfg["forward_margin_m"])

    def forward(amount: float, why: str) -> Decision | None:
        """前へ amount 進む 1 回分。最小の 1 回でも勧める範囲の近い側（＋余裕）より近づく・安全でないなら None。"""
        amount = min(amount, d - lo - margin)
        if amount <= 0:
            return None
        p = mover.plan("forward", amount)
        if p is None or d - max(amount, step) < lo - 1e-9 or not is_safe(p):
            return None
        return Decision("move", why, p)

    if m.source == "face":
        shift = edge_shift(m, edge_ref or {})
        if shift is not None and abs(shift) > float(cfg["side_tol_m"]):
            p = mover.plan("left" if shift > 0 else "right", abs(shift))
            if p is not None and is_safe(p):
                return Decision("move", f"柱の端が最初より {shift * 100:+.1f} cm ずれた（体が{'右' if shift > 0 else '左'}へずれた）。"
                                        f"{'左' if shift > 0 else '右'}へ戻す", p)
        stop = max(target, float(cfg["face_stop_dist_m"]))
        dec = forward(d - stop, f"柱まで {d * 100:.1f} cm。▲▼ が見え始める {stop * 100:.0f} cm まで前へ")
        if dec is not None:
            return dec
        dec = forward(step, f"柱まで {d * 100:.1f} cm だが ▲▼ が見えない。見えるところまで前へ")
        if dec is not None:
            return dec
        return Decision("abort", f"柱まで {d * 100:.1f} cm まで来たが ▲▼ が見えない（勧める範囲 {lo * 100:.0f}〜{hi * 100:.0f} cm）。"
                                 "頭カメラの向き・ボタンの高さ・照明を確かめる")

    # ▲▼ が見えているが計画が立たない。左右を先に直す（横に動くと前にも少し出るので、前は最後）
    side_err = float(m.side_m) - float(st["center_side_m"])
    side_blocked = ""
    if abs(side_err) > float(cfg["side_tol_m"]):
        p = mover.plan("left" if side_err > 0 else "right", abs(side_err))
        if p is not None:
            way = "左" if side_err > 0 else "右"
            if is_safe(p):
                return Decision("move", f"ボタンが目安（{arm} 腕、体の中心から {float(st['center_side_m']) * 100:+.0f} cm）から"
                                        f" {side_err * 100:+.1f} cm。{way}へ", p)
            side_blocked = f"{way}へ {abs(side_err) * 100:.0f} cm 寄りたいが、柱に近すぎて横に動けない。"
    if err > 0:
        dec = forward(err, f"柱まで {d * 100:.1f} cm（目安 {target * 100:.0f} cm、勧める範囲 {lo * 100:.0f}〜{hi * 100:.0f} cm）。前へ")
        if dec is not None:
            return dec
    if side_blocked:
        return Decision("abort", side_blocked + "人がリモコンで下げてからやり直す")
    if d < lo:
        return Decision("abort", f"柱まで {d * 100:.1f} cm と近すぎて計画が立たない（勧める範囲 {lo * 100:.0f}〜{hi * 100:.0f} cm）。"
                                 "人がリモコンで下げる")
    why = " / ".join(f"{a}: {r[:200]}" for a, r in reasons.items())
    return Decision("abort", f"これ以上動いても直せない（1 回の最小より小さいずれ）が、計画が立たない。{why}")


# ---- 寄る -----------------------------------------------------------------------


class Approacher:
    def __init__(self, perception: Perception, feasibility: Feasibility, mover: LocoMover, cfg: dict[str, Any],
                 log: Callable[[str], None] = print) -> None:
        self.perception = perception
        self.feasibility = feasibility
        self.mover = mover
        self.cfg = cfg
        self.log = log
        self.stands = {a: feasibility.stand(a) for a in feasibility.arms}
        # 最初に見えた柱の端（人が置いた左右の位置）。端ごとに、初めて分かったときの値を覚える
        self.edge_ref: dict[str, float] = {}

    def run(self) -> ApproachResult:
        steps: list[dict[str, Any]] = []
        with self.mover:
            for k in range(int(self.cfg["max_steps"])):
                m = self.perception.measure()
                if m is None:
                    return self._end("aborted", f"見つからない: {self.perception.last_reason}", steps)
                self.log(f"[approach] {k + 1} 回目の観測 — {m.summary()}")
                for name, v in (("left", m.left_edge_m), ("right", m.right_edge_m)):
                    if v is not None and name not in self.edge_ref and m.source == "face":
                        self.edge_ref[name] = v
                arm, reasons = self.feasibility.check(m) if m.source == "buttons" else (None, {})
                dec = decide(m, arm, reasons, self.stands, self.cfg, self.mover, self.edge_ref)
                row: dict[str, Any] = {"step": k + 1, "source": m.source, "distance_m": m.distance_m,
                                       "yaw_deg": float(np.degrees(m.yaw_rad)), "side_m": m.side_m,
                                       "left_edge_m": m.left_edge_m, "right_edge_m": m.right_edge_m,
                                       "action": dec.action, "reason": dec.reason,
                                       "pulse": None if dec.pulse is None else vars(dec.pulse)}
                steps.append(row)
                self.log(f"[approach] → {dec.reason}")
                if dec.action == "ready":
                    return self._end("ready", dec.reason, steps, dec.arm)
                if dec.action == "abort":
                    return self._end("aborted", dec.reason, steps)
                assert dec.pulse is not None
                try:
                    res = self.mover.pulse(dec.pulse)
                except LocoError as e:
                    return self._end("aborted", str(e), steps)
                row["settle_s"], row["stopped"] = res.settle_s, res.stopped
                if not self.mover.go:
                    return self._end("dry_run", f"送っていない（go=False）。次の 1 回: {dec.pulse.summary()}", steps)
                if not res.stopped:
                    return self._end("aborted", res.note, steps)
                self.perception.reset()
        return self._end("aborted", f"{int(self.cfg['max_steps'])} 回観測しても押せる位置にならない。人が確かめる", steps)

    def _end(self, outcome: str, reason: str, steps: list[dict[str, Any]], arm: str | None = None) -> ApproachResult:
        mark = {"ready": "✅", "aborted": "❌", "dry_run": "（送らない）"}[outcome]
        self.log(f"[approach] {mark} {reason}")
        return ApproachResult(outcome, reason, arm, steps)
