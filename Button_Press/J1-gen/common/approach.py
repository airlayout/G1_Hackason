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
   - ▲▼ が組でそろわなくても、ボタンが 1 個でも見えれば、その左右（▲▼ は同じ柱に縦に並ぶので左右は同じ）を手がかりにする
     （評価環境で、▲ が画像の上に切れて「1 個」のまま前へ進み、横に 34 cm ずれてボタンが画像の外に出た）
   - 前方の Web カメラ（WebcamAssist、おまけ）があれば、遠くでもボタンの左右が分かる（Web カメラの方向の線と柱の面の交点）。
     左右の手がかりがあれば、端の代わりに、ボタンを左右の目安へ寄せる。無い・映像が来ない日は端だけで動く
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
from dataclasses import dataclass, field, replace
from typing import Any, Protocol

import numpy as np

from .column_face import ColumnFace, FaceFinder, FaceNotFound, FaceTracker, motion_between
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
    # 面しか見えないときのボタンの左右の手がかり（pelvis の y）と、その出どころ（"head_single" = 頭カメラで 1 個だけ見えた
    # ボタン / "webcam" = 前方の Web カメラ）。Web カメラの推定の中身（WebcamAssist.estimate）も残す。無ければ None
    side_hint_m: float | None = None
    side_hint_source: str | None = None
    webcam: dict[str, Any] | None = None

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
        if self.side_m is None and self.side_hint_m is not None:
            src = {"webcam": "Web カメラ", "head_single": "1 個だけ見えたボタン"}.get(self.side_hint_source or "", "手がかり")
            side = f"、{src}ではボタンは体の{'左' if self.side_hint_m > 0 else '右'} {abs(self.side_hint_m) * 100:.1f} cm"
        what = "▲▼ から" if self.source == "buttons" else "柱の面から（▲▼ は見えない）"
        return f"{what}: 柱まで {self.distance_m * 100:.1f} cm、向き {np.degrees(self.yaw_rad):+.1f}°{side}"


@dataclass
class Decision:
    action: str                 # ready / move / abort
    reason: str
    pulse: Pulse | None = None
    arm: str | None = None
    # 横へ動くとき、何を目安にしたか: "edges"（最初に覚えた柱の端の位置へ戻す）/ "target"（ボタン・手がかりの目安へ寄る）
    side_basis: str | None = None


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
                 elevator_cfg: dict[str, Any], face_cfg: dict[str, Any], cfg: dict[str, Any],
                 webcam: "WebcamAssist | None" = None) -> None:
        """webcam: 前方の Web カメラ（おまけ）。面しか見えないときに、ボタンの左右を推定する。None なら使わない。"""
        self.get_frame = get_frame
        self.webcam = webcam
        self.buttons = buttons
        self.face = face
        self.button_tracker = ButtonTracker(elevator_cfg["tracking"])
        self.face_tracker = FaceTracker(face_cfg["tracking"])
        self.max_frames = int(cfg["max_frames"])
        self.column_tol = float(elevator_cfg["geometry"]["column_tolerance_m"])
        self.single_depth_tol = float(cfg["single_depth_tol_m"])
        self.single_min_frames = int(cfg["single_min_frames"])
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
        why = {"▲▼": "", "柱の面": ""}
        singles: list[list[np.ndarray]] = []  # フレームごとの、組にならなかったボタンの位置（pelvis 座標）
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
                    why["▲▼"] = str(e)
                    # 組にならなくても、3 次元の位置が出たボタンを覚えておく（最後に、柱の面の上にあり、2 フレーム以上で
                    # 同じ左右に見えたものだけを手がかりにする。乱しありの評価環境では、ボタンでない物を 1 個だけ誤って
                    # 見つけ、左右が ±60 cm 飛んで行ったり来たりした）
                    singles.append([np.asarray(b.center, dtype=float) for b in getattr(self.buttons, "last_buttons", [])])
            try:
                fm = self.face_tracker.add(self.face.find(fr.depth, fr.K, qw, fr.imu_quat))
                if fm is not None:
                    face_m = Measurement.from_face(fm, np.asarray(fr.q, dtype=float).copy())
                elif self.face_tracker.last_reject:
                    why["柱の面"] = self.face_tracker.last_reject
            except FaceNotFound as e:
                self.face_tracker.reset()
                why["柱の面"] = str(e)
        if face_m is not None:
            single_y = self._single_side(singles, face_m.face)
            if single_y is not None:
                face_m.side_hint_m, face_m.side_hint_source = single_y, "head_single"
            if self.webcam is not None and face_m.face is not None:
                est = self.webcam.estimate(face_m.face)
                face_m.webcam = est if est is not None else {"reason": self.webcam.last_reason}
                if est is not None and est.get("use") is not None and face_m.side_hint_m is None:
                    face_m.side_hint_m, face_m.side_hint_source = float(est["use"][1]), "webcam"
            return face_m
        self.last_reason = (f"{self.max_frames} フレームで ▲▼ も柱の面もそろわない（最後の理由: "
                            + "、".join(f"{k}: {v}" for k, v in why.items() if v) + "）")
        return None


    def _single_side(self, singles: list[list[np.ndarray]], face: ColumnFace | None) -> float | None:
        """組にならなかったボタンのうち、柱の面の上（奥行きが面と single_depth_tol_m 以内）にあり、single_min_frames 以上の
        フレームで、しかも見えたフレームの半分以上で同じ左右（column_tolerance 以内）に見えたものの左右（中央値）。無ければ None。"""
        if face is None or not singles:
            return None
        ys = []
        for found in singles:
            ok = [float(c[1]) for c in found if abs(float(-face.normal @ c) - face.distance_m) <= self.single_depth_tol]
            if ok and np.ptp(ok) <= self.column_tol:
                ys.append(float(np.median(ok)))
        if len(ys) < self.single_min_frames:
            return None
        med = float(np.median(ys))
        agree = [y for y in ys if abs(y - med) <= self.column_tol]
        # 見えたフレームの半分以上で同じ左右（飛び回る誤検出は、同じ値が何度か出ても半分には届かない）
        return float(np.median(agree)) if len(agree) >= max(self.single_min_frames, 0.5 * len(ys)) else None


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
           cfg: dict[str, Any], mover: LocoMover, edge_ref: dict[str, float] | None = None,
           edge_prev: dict[str, float] | None = None) -> Decision:
    """観測から、止まる（ready）・1 回動く（move）・やめる（abort）を決める。

    stands は腕ごとの勧める立つ位置。edge_ref は最初に見えた柱の端（面しか見えないうちに左右を保つのに使う）、
    edge_prev は前の観測の柱の端（端が 1 回分ではありえないほど飛んだかを見る）。
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
        # 左右の手がかり（1 個だけ見えたボタン・Web カメラ）があればそれで、無ければ柱の端で、横のずれを直す
        if m.side_hint_m is not None:
            hint_err = float(m.side_hint_m) - float(st["center_side_m"])
            if abs(hint_err) > float(cfg["side_tol_m"]):
                p = mover.plan("left" if hint_err > 0 else "right", abs(hint_err))
                if p is not None and is_safe(p):
                    way = "左" if hint_err > 0 else "右"
                    src = {"webcam": "Web カメラ", "head_single": "1 個だけ見えたボタン"}.get(m.side_hint_source or "", "手がかり")
                    return Decision("move", f"{src}でボタンが目安（{arm} 腕、体の中心から {float(st['center_side_m']) * 100:+.0f} cm）"
                                            f"から {hint_err * 100:+.1f} cm。{way}へ", p, side_basis="target")
        else:
            shift = edge_shift(m, edge_ref or {})
            jump = edge_shift(m, edge_prev or {})
            if jump is not None and abs(jump) > float(cfg["max_edge_shift_m"]):
                # 前の観測から、1 回分で横にずれうる量より大きく飛んだ。柱と壁の段が近い（評価環境は 1 cm）と、手前の段が柱に
                # なったり壁になったりして端が飛ぶ（+33 cm と出た）。別の段の端を見ているとみなし、端では直さない
                # （飛びは前の観測からの変化で見る。最初の位置からの合計で見ると、少しずつ溜まったずれまで捨てた）
                shift = None
            if shift is not None and abs(shift) > float(cfg["edge_tol_m"]):
                p = mover.plan("left" if shift > 0 else "right", abs(shift))
                if p is not None and is_safe(p):
                    return Decision("move", f"柱の端が最初より {shift * 100:+.1f} cm ずれた（体が{'右' if shift > 0 else '左'}へずれた）。"
                                            f"{'左' if shift > 0 else '右'}へ戻す", p, side_basis="edges")
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
                                        f" {side_err * 100:+.1f} cm。{way}へ", p, side_basis="target")
            side_blocked = f"{way}へ {abs(side_err) * 100:.0f} cm 寄りたいが、柱に近すぎて横に動けない。"
    if err > 0:
        dec = forward(err, f"柱まで {d * 100:.1f} cm（目安 {target * 100:.0f} cm、勧める範囲 {lo * 100:.0f}〜{hi * 100:.0f} cm）。前へ")
        if dec is not None:
            return dec
    if d > hi:
        # 目安まで 1 回の最小の半分より近いが、勧める範囲より遠く計画も立たない（評価環境、柱まで 52 cm・目安 48 cm で止まった）。
        # 最小の 1 回分でも前へ（範囲の近い側・安全はいつもどおり確かめる）
        dec = forward(step, f"柱まで {d * 100:.1f} cm と勧める範囲（〜{hi * 100:.0f} cm）より遠く、計画が立たない。最小の 1 回分だけ前へ")
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


class EdgeCheck:
    """観測ごとの柱の端を確かめ、当てにならない端を捨てる（11/14 の実機で、誤った左端で逆へ動き、横の台にぶつかった）。

    1. 端ごとに、最後に通った値と比べる。その後に送った横の 1 回分（見込み）を差し引いても、max_edge_shift_m × その後に動いた
       回数より動いていれば、その端だけ捨てる（前の観測に無かった端も、最後に見えた値と比べられる。見えない間に本当にずれた分は、
       回数に応じて許す）
    2. 左右の端が両方あれば、その幅（左 − 右）を最初に両方見えた幅と比べ、edge_width_tol_m より違えば両方捨てる
       （どちらが誤りか分からないので）。11/14 は幅 95 cm の板で、誤った左端と右端の幅が 34 cm だった
    """

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.max_jump = float(cfg["max_edge_shift_m"])
        tol = cfg.get("edge_width_tol_m")
        self.width_tol = None if tol is None else float(tol)
        self.last: dict[str, float] = {}     # 端ごとの、最後に通った値
        self.expect: dict[str, float] = {}   # 最後に通ってから、横の 1 回分で端が動くはずの量
        self.n_since: dict[str, int] = {}    # 最後に通ってから動いた回数
        self.width: float | None = None

    def moved(self, side_m: float = 0.0) -> None:
        """1 回動いた（前・回る・横のどれでも）。side_m は横へ動いた見込み（+ が左）。体が左へ動くと、端は右（−）へ動いて見える。"""
        for k in ("left", "right"):
            self.expect[k] = self.expect.get(k, 0.0) - side_m
            self.n_since[k] = self.n_since.get(k, 0) + 1

    def check(self, m: Measurement) -> tuple[Measurement, list[str]]:
        """当てにならない端を None にした Measurement と、捨てた理由を返す。"""
        vals = {"left": m.left_edge_m, "right": m.right_edge_m}
        notes: list[str] = []
        name = {"left": "左端", "right": "右端"}
        for k, v in vals.items():
            if v is None or k not in self.last:
                continue
            jump = v - (self.last[k] + self.expect.get(k, 0.0))
            if abs(jump) > self.max_jump * max(1, self.n_since.get(k, 0)):
                notes.append(f"{name[k]} {v * 100:+.1f} cm は、最後に見えた {self.last[k] * 100:+.1f} cm（横の動きの見込み "
                             f"{self.expect.get(k, 0.0) * 100:+.1f} cm）から {jump * 100:+.1f} cm 飛んだ。使わない")
                vals[k] = None
        if vals["left"] is not None and vals["right"] is not None and self.width_tol is not None:
            w = vals["left"] - vals["right"]
            if self.width is None:
                self.width = w
            elif abs(w - self.width) > self.width_tol:
                notes.append(f"左右の端の幅 {w * 100:.1f} cm が、最初の {self.width * 100:.1f} cm と違う。端は使わない")
                vals = {"left": None, "right": None}
        for k, v in vals.items():
            if v is not None:
                self.last[k] = v
                self.expect[k] = 0.0
                self.n_since[k] = 0
        return replace(m, left_edge_m=vals["left"], right_edge_m=vals["right"]), notes


class Approacher:
    def __init__(self, perception: Perception, feasibility: Feasibility, mover: LocoMover, cfg: dict[str, Any],
                 log: Callable[[str], None] = print) -> None:
        self.perception = perception
        self.feasibility = feasibility
        self.mover = mover
        self.cfg = cfg
        self.log = log
        self.stands = {a: feasibility.stand(a) for a in feasibility.arms}
        # 最初に見えた柱の端（人が置いた左右の位置）。端ごとに、正対してから初めて分かったときの値を覚える
        self.edge_ref: dict[str, float] = {}
        # 端の確かめ（EdgeCheck）。飛びは端ごとに最後に通った値と比べるので、decide の edge_prev（前の観測）は使わない
        self.edges = EdgeCheck(cfg)
        # 途中で止めても（Ctrl+C）、それまでの観測を real/approach.py が保存できるように持っておく
        self.steps: list[dict[str, Any]] = []

    def run(self) -> ApproachResult:
        steps = self.steps
        with self.mover:
            for k in range(int(self.cfg["max_steps"])):
                m = self.perception.measure()
                if m is None:
                    return self._end("aborted", f"見つからない: {self.perception.last_reason}", steps)
                self.log(f"[approach] {k + 1} 回目の観測 — {m.summary()}")
                raw_edges = (m.left_edge_m, m.right_edge_m)
                edge_notes: list[str] = []
                if m.source == "face":
                    m, edge_notes = self.edges.check(m)
                    for note in edge_notes:
                        self.log(f"[approach] ⚠️ {note}")
                # 最初の端の位置は、正対してから（向きのずれが yaw_tol_deg 以内で）覚える。斜めのまま覚えると、回っただけで
                # 端の値が動き（11/14: 26.7° → 5.8° と回る間に右端が −48 → −36 cm）、見かけのずれを直しに横へ動いてしまう
                facing = abs(np.degrees(m.yaw_rad)) <= float(self.cfg["yaw_tol_deg"])
                for name, v in (("left", m.left_edge_m), ("right", m.right_edge_m)):
                    if v is not None and name not in self.edge_ref and m.source == "face" and facing:
                        self.edge_ref[name] = v
                arm, reasons = self.feasibility.check(m) if m.source == "buttons" else (None, {})
                dec = decide(m, arm, reasons, self.stands, self.cfg, self.mover, self.edge_ref, None)
                row: dict[str, Any] = {"step": k + 1, "source": m.source, "distance_m": m.distance_m,
                                       "yaw_deg": float(np.degrees(m.yaw_rad)), "side_m": m.side_m,
                                       "left_edge_m": m.left_edge_m, "right_edge_m": m.right_edge_m,
                                       "raw_left_edge_m": raw_edges[0], "raw_right_edge_m": raw_edges[1],
                                       "edge_notes": edge_notes, "edge_ref": dict(self.edge_ref),
                                       "side_hint_m": m.side_hint_m, "side_hint_source": m.side_hint_source,
                                       "webcam": m.webcam,
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
                side = self.mover.side_step(dec.pulse)
                self.edges.moved(side)
                if side != 0.0 and dec.side_basis == "target":
                    # ボタン・手がかりの目安へ寄った: 目指す左右が変わったので、柱の端の基準も同じだけ動かす
                    # （動かさないと、次に面しか見えないとき「最初の位置からずれた」と戻してしまい、行ったり来たりする）
                    self.edge_ref = {k: v - side for k, v in self.edge_ref.items()}
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


# ---- 前方の Web カメラ（おまけ） ----------------------------------------------------------


class WebcamAssist:
    """前方の Web カメラで、面しか見えない遠さからボタンの左右を推定する（common/webcam_buttons.py の 3 つの方法）。

    get_frame() は (RGB の画像, 受け取ってからの秒数) か None を返す。決めるのに使うのは webcam.yaml の assist.use
    （plane = 頭カメラの柱の面との交点 / size = ▲▼ の間隔から / off = 記録するだけ）。ほかの方法の値も記録に残す
    （大きさ・視差の当たり具合を、実機で比べるため）。交点と間隔からの距離が大きく食い違えば使わない。
    """

    def __init__(self, get_frame: Callable[[], tuple[np.ndarray, float] | None], detector: Any, cam: Any,
                 cfg: dict[str, Any]) -> None:
        self.get_frame = get_frame
        self.detector = detector
        self.cam = cam
        self.cfg = cfg
        self.last_reason = ""
        self._prev: tuple[np.ndarray, ColumnFace] | None = None

    def estimate(self, face: ColumnFace | None) -> dict[str, Any] | None:
        from .webcam_buttons import find_pair, from_plane, from_size, triangulate

        a = self.cfg["assist"]
        got = self.get_frame()
        if got is None:
            self.last_reason = "Web カメラの映像が届かない"
            return None
        rgb, age = got
        if age > float(a["max_age_s"]):
            self.last_reason = f"Web カメラの映像が古い（{age:.1f} 秒前）"
            return None
        pair = find_pair(self.detector.detect(rgb), self.cfg)
        if pair is None:
            self.last_reason = "Web カメラの画像に ▲▼ の組が見つからない"
            self._prev = None
            return None
        est: dict[str, Any] = {"uv": pair.mid_px.tolist(), "spacing_px": pair.spacing_px}
        plane = from_plane(self.cam, pair.mid_px, face.normal, face.point) if face is not None else None
        sizes = from_size(self.cam, pair, self.cfg)
        if plane is not None:
            est["plane"] = plane.tolist()
        for k, v in sizes.items():
            est[k] = v.tolist()
        if self._prev is not None and face is not None:
            uv0, f0 = self._prev
            mv = motion_between(f0, face)
            if mv["left_cm"] is not None:
                tri = triangulate(self.cam, uv0, pair.mid_px,
                                  (mv["forward_cm"] / 100, mv["left_cm"] / 100, np.radians(mv["turn_deg"])))
                if tri is not None:
                    est["parallax"], est["parallax_gap_m"] = tri[0].tolist(), tri[1]
        self._prev = (pair.mid_px.copy(), face) if face is not None else None
        if plane is not None and "by_spacing" in sizes:
            r_plane = float(np.linalg.norm(plane - self.cam.xyz))
            r_size = float(np.linalg.norm(sizes["by_spacing"] - self.cam.xyz))
            if abs(r_size / r_plane - 1.0) > float(a["max_range_disagreement"]):
                est["use"] = None
                self.last_reason = (f"Web カメラの推定が食い違う（柱の面との交点まで {r_plane * 100:.0f} cm、"
                                    f"▲▼ の間隔からは {r_size * 100:.0f} cm）。使わない")
                est["reason"] = self.last_reason
                return est
        use = {"plane": plane, "size": sizes.get("by_spacing"), "off": None}[str(a["use"])]
        est["use"] = None if use is None else np.asarray(use).tolist()
        return est
