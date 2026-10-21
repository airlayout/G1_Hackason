"""押し込みの実行（PressPlan を ArmCommander で送る）。sim / real 共通。タスク2・6 で使う。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

from .arm import ArmCommander, RetreatStep
from .post_press import NoCheck, PostPressCheck, PostPressContext, PostPressResult
from .press_planner import PressPlan, PressPlanner

# 段階ごとに呼ぶ関数（段階の名前、その時点の計画）。記録やシミュレーションでの測定に使う
StageCallback = Callable[[str, PressPlan], None]


def _replan_at_approach(arm: ArmCommander, planner: PressPlanner, plan: PressPlan, depth: float | None,
                        on_stage: StageCallback | None, stage: str) -> PressPlan:
    """手前の姿勢で lowstate から実際の腰の角度を読み直し、その角度で押し込みを計算し直す。"""
    q_meas = arm.measured_q()
    new = planner.replan(plan, q_meas, arm.commanded_arm_q, depth=depth)
    dw = np.degrees(new.waist_q - plan.waist_q)
    dq = float(np.max(np.abs(new.q_approach - plan.q_approach)))
    dw_s = ", ".join(f"{v:+.2f}" for v in dw)
    print(f"[press] 実測の腰の角度で計算し直した（腰の変化 [{dw_s}]°、"
          f"手前の姿勢の変化 最大 {np.degrees(dq):.2f}°、深さ {new.depth * 1000:.1f} mm）")
    if dq > 1e-6:
        arm.move_to(new.q_approach, label="手前の姿勢を補正")
    if on_stage:
        on_stage(stage, new)
    return new


class ContactStop:
    """押し込みの途中で、指先が進まなくなったかを見る（ArmCommander.follow の stop_when に渡す）。

    指令した指先（指令の角度の FK）が実際の指先より押す向きに先にある量を「遅れ」とする。**押し込みを始めた周期の遅れ
    （baseline_m）を基準にして、そこからの増え方が lag_max を超えたら**止める。押す前から遅れがある実機では、遅れの
    絶対値で見ると、触れる前に止めてしまう（2026-10-21、重力補償 0.5 で押す前から 4.5 mm 遅れていて、約 1 mm 進んだだけで
    lag_m 5 mm を超え、ボトルの約 5 cm 手前で止まった）。
    MuJoCo の固いボトルでは、止めないと遅れが 25 mm まで増え、その反動で腰が 2.9° ねじれて安全確認（腰のずれ）で中止した
    （2026-10-15、机 0.79 m・ボトルが右 25 cm）。押し直し（attempt 2 以降）は retry_lag_m まで押す。
    記録（tip_start / tip_end / baseline_m / extra_m）は、押し込みの結果の表示に使う。
    """

    def __init__(self, planner: PressPlanner, plan: PressPlan, attempt: int) -> None:
        cs = planner.cfg.get("contact_stop") or {}
        self.kin, self.n = planner.kin, plan.push_dir
        self.lag_max = float(cs["retry_lag_m"] if attempt > 1 else cs["lag_m"]) if cs else None
        self.baseline_m: float | None = None  # 押し込みを始めた周期の遅れ
        self.extra_m = 0.0                    # 最後に見た、基準からの遅れの増え方
        self.tip_start: float | None = None   # 押し込みを始めたときの実際の指先（押す向きの座標）
        self.tip_end: float | None = None     # 最後に見た実際の指先
        self.stopped = False

    def __call__(self, q_cmd: np.ndarray, q_meas: np.ndarray) -> bool:
        tip = float(self.kin.fk_pos(q_meas) @ self.n)
        lag = float((self.kin.fk_pos(q_cmd) - self.kin.fk_pos(q_meas)) @ self.n)
        if self.baseline_m is None:
            self.baseline_m, self.tip_start = lag, tip
        self.tip_end = tip
        self.extra_m = lag - self.baseline_m
        if self.lag_max is not None and self.extra_m > self.lag_max:
            print(f"[press] 指先が進まなくなった（遅れ {lag * 1000:.1f} mm、押す前から {self.baseline_m * 1000:.1f} mm、"
                  f"増えた分 {self.extra_m * 1000:.1f} mm）。押し込みを止める")
            self.stopped = True
            return True
        return False

    def report(self, planned_m: float) -> dict[str, float | bool | None]:
        """押し込みの結果（指先が実際にどれだけ進んだか）。planned_m は計画の押し込みの長さ。"""
        adv = None if self.tip_start is None or self.tip_end is None else self.tip_end - self.tip_start
        return {"stopped": self.stopped, "planned_m": planned_m, "tip_advance_m": adv,
                "baseline_lag_m": self.baseline_m, "extra_lag_m": self.extra_m}


def _report_press(stop: ContactStop, planner: PressPlanner, plan: PressPlan, q_meas: np.ndarray, attempt: int,
                  sent: int, on_press: Callable[[dict[str, Any]], None] | None) -> None:
    """押し込みの結果を表示し、on_press に渡す。「成功」だけでは、触れたかが分からない（ボトルには押したあとの確認が無い）。"""
    n = plan.push_dir
    planned = float((plan.end_point - plan.approach_point) @ n)
    stop.tip_end = float(planner.kin.fk_pos(q_meas) @ n)  # 押し込みを送り終えて落ち着いたあとの指先
    info = stop.report(planned)
    info.update(attempt=attempt, sent=sent, total=len(plan.press_in))
    adv = info["tip_advance_m"]
    adv_s = "不明" if adv is None else f"{adv * 1000:.1f} mm"
    print(f"[press] 押し込み（{attempt} 回目）の結果: 指先が実際に進んだ {adv_s}（計画 {planned * 1000:.1f} mm、"
          f"{sent}/{len(plan.press_in)} 点、{'途中で止めた' if info['stopped'] else '最後まで押した'}）")
    if on_press:
        on_press(info)


def _hold_index(planner: PressPlanner, plan: PressPlan, sent: int, q_meas: np.ndarray,
                keep: float | None = None) -> int:
    """押し込みの直線の点のうち、実際の指先より押す向きに keep（既定は contact_stop.keep_m）先までにある最も奥の点の
    番号（sent より前）。"""
    keep = float(planner.cfg["contact_stop"]["keep_m"]) if keep is None else keep
    n, arm = plan.push_dir, planner.kin.arm_idx
    tip = float(planner.kin.fk_pos(q_meas) @ n)
    q = q_meas.copy()
    j = 0
    for k in range(sent):
        q[arm] = plan.press_in[k]
        if float(planner.kin.fk_pos(q) @ n) - tip <= keep:
            j = k
    return j


class PressRetreat:
    """中止したときに腕を戻す経路（来た経路を逆にたどる）。execute_press が進み具合（stage）と計画を書き込み、
    ArmCommander が中止したときに呼ぶ。

    stage:
      to_via      開始姿勢 → 経由の姿勢へ移動中          → 開始姿勢
      to_approach 経由の姿勢 → 手前の姿勢へ移動中・補正中  → 経由の姿勢 → 開始姿勢
      press       押し込み・保持・戻り・確認・押し直し      → 押す力が抜ける点で止める → 押し込みの直線をさかのぼって
                                                           手前 → 経由 → 開始
      to_via_back 手前の姿勢 → 経由の姿勢へ戻る途中        → 経由の姿勢 → 開始姿勢
      to_start    経由の姿勢 → 開始姿勢へ戻る途中          → 開始姿勢
    press 以外では、今の指令のまま止める（ArmCommander の既定）。
    """

    def __init__(self, plan: PressPlan, planner: PressPlanner, return_to: np.ndarray | None) -> None:
        self.plan = plan
        self.planner = planner
        self.return_to = None if return_to is None else np.asarray(return_to, dtype=float).copy()
        self.stage = "to_via"

    def __call__(self, q_meas: np.ndarray, q_cmd_arm: np.ndarray) -> list[RetreatStep]:
        p = self.plan
        steps: list[RetreatStep] = []
        if self.stage == "press":
            # 押し当てていると、指令が実際の指先より奥にあって押す力が残る。指令を押し込みの直線の点を逆にたどって、
            # 実際の指先の位置に当たる点（k）まで戻してから止める（押す向きと逆にまっすぐ。実測の姿勢をそのまま指令に
            # すると、重力で下がっている分だけさらに下がる: MuJoCo で手首が机の縁に 4.4 N。関節ごとに速さの上限で
            # 寄せると、指先が直線から外れる）。そこから直線を手前の姿勢までさかのぼる
            dist = [float(np.max(np.abs(w - q_cmd_arm))) for w in p.press_in]
            j = int(np.argmin(dist))  # 今の指令の点
            k = min(_hold_index(self.planner, p, len(p.press_in), q_meas, keep=0.0), j)
            if dist[j] > 1e-6:  # 指令が直線の上に無い（押し直しの前の補正の途中など。手前の姿勢の近く）
                steps.append(RetreatStep("押し込みの直線へ寄せる", target=p.press_in[k].copy()))
            elif j > k:
                steps.append(RetreatStep("押す力が抜ける点まで下がる",
                                         line=[w.copy() for w in reversed(p.press_in[k:j])], hold=True))
            steps.append(RetreatStep("押す向きと逆に下がって手前の姿勢へ",
                                     line=[w.copy() for w in reversed(p.press_in[:k])] + [p.q_approach.copy()]))
        if self.stage in ("press", "to_approach", "to_via_back") and p.q_via is not None:
            steps.append(RetreatStep("経由の姿勢へ", target=p.q_via.copy()))
        if self.return_to is not None:
            # 開始姿勢は腕がいた場所なので、作業空間の箱の外でもよい
            steps.append(RetreatStep("開始姿勢へ", target=self.return_to.copy(), check_workspace=False))
        return steps


def execute_press(
    arm: ArmCommander,
    plan: PressPlan,
    hold_s: float,
    planner: PressPlanner,
    return_to: np.ndarray | None = None,
    on_stage: StageCallback | None = None,
    post_check: PostPressCheck | None = None,
    max_retries: int = 0,
    deeper_limit_m: float | None = None,
    grab_frame: Callable[[], Any] | None = None,
    replan: bool = True,
    contact_stop: bool = True,
    on_press: Callable[[dict[str, Any]], None] | None = None,
) -> tuple[PressPlan, list[PostPressResult]]:
    """（経由の姿勢 →）手前の姿勢へ移動 → 実測の腰の角度で計算し直す → 直線で押し込む → 止まる → 直線で戻る →
    押したあとの確認 →（必要なら深くして押し直す）→（与えれば）return_to へ戻る。

    planner は必須（実測の腰の角度で計算し直す手順を、必ず通すため）。replan=False は MuJoCo の比較実験
    （sim/press_sim.py）専用。全体の流れ（common/pipeline.py）では渡さない（常に計算し直す）。
    押し直しは max_retries 回まで。深さは deeper_limit_m（無ければ press.max_press_depth_m）で頭打ち。
    contact_stop（既定で有効、設定は press.contact_stop）: 押し込みの途中で指先が進まなくなったら、それ以上奥へ
    押さず、実際の指先の少し先を押したまま保持する（ContactStop。押す前からある遅れは基準にして、増え方で見る）。
    on_press: 押し込みのたびに、結果（止めたか、指先が実際にどれだけ進んだか）を dict で受け取る関数。
    計算し直した結果が届かない・ぶつかるなら UnreachableError で止まる（ArmCommander が安全に終了する）。
    実際に使った最後の計画と、確認の結果の一覧を返す。
    """
    check = post_check or NoCheck()
    limit = deeper_limit_m if deeper_limit_m is not None else float(planner.cfg["max_press_depth_m"])
    print(f"[press] {plan.summary()}")
    # 途中で中止したら、来た経路を逆にたどって戻る（ArmCommander の safe_stop が呼ぶ）
    retreat = PressRetreat(plan, planner, return_to)
    arm.set_retreat(retreat)
    if plan.q_via is not None:
        arm.move_to(plan.q_via, label="経由の姿勢へ移動")
    retreat.stage = "to_approach"
    arm.move_to(plan.q_approach, label="手前の姿勢へ移動")
    if on_stage:
        on_stage("approach", plan)
    if replan:
        plan = _replan_at_approach(arm, planner, plan, None, on_stage, "approach_replanned")
        retreat.plan = plan
    retreat.stage = "press"
    results: list[PostPressResult] = []
    attempt = 1
    while True:
        stop = ContactStop(planner, plan, attempt) if contact_stop else None
        sent = arm.follow(plan.press_in, label=f"押し込み（{attempt} 回目、深さ {plan.depth * 1000:.1f} mm）",
                          stop_when=stop)
        if stop is not None:
            _report_press(stop, planner, plan, arm.measured_q(), attempt, sent, on_press)
        q_hold, press_out = plan.press_in[-1], plan.press_out
        if sent < len(plan.press_in):
            j = _hold_index(planner, plan, sent, arm.measured_q())
            # 止めた点から、保持する点まで直線をさかのぼる（1 周期の関節の動きの上限を守るため）
            back = [w.copy() for w in reversed(plan.press_in[j:sent - 1])]
            if back:
                arm.follow(back, label="押し込みを少し戻す")
            q_hold = plan.press_in[j]
            press_out = [w.copy() for w in reversed(plan.press_in[:j])] + [plan.q_approach.copy()]
        if on_stage:
            on_stage("end", plan)
        arm.follow([q_hold] * max(1, int(hold_s / arm.dt)), label="押し込んだまま保持")
        arm.follow(press_out, label="戻り")
        if on_stage:
            on_stage("retreated", plan)
        res = check(PostPressContext(attempt=attempt, depth_m=plan.depth, plan=plan,
                                     grab_frame=grab_frame or (lambda: None)))
        results.append(res)
        print(f"[press] 押したあとの確認（{attempt} 回目）: {'OK' if res.ok else 'NG'} {res.message}")
        if res.ok or res.deeper_m <= 0.0:
            break
        if attempt > max_retries:
            print(f"[press] 押し直しの回数の上限（{max_retries} 回）に達したので終わる")
            break
        new_depth = min(plan.depth + res.deeper_m, limit)
        if new_depth <= plan.depth + 1e-9:
            print(f"[press] 深さが上限（{limit * 1000:.1f} mm）なので押し直さない")
            break
        attempt += 1
        plan = _replan_at_approach(arm, planner, plan, new_depth, on_stage, f"retry_{attempt}_replanned")
        retreat.plan = plan
    if plan.q_via is not None:
        retreat.stage = "to_via_back"
        arm.move_to(plan.q_via, label="経由の姿勢へ戻る")
    retreat.stage = "to_start"
    if return_to is not None:
        # 開始姿勢は腕がいた場所なので、作業空間の箱の外でもよい
        arm.move_to(return_to, label="開始姿勢へ戻る", check_workspace=False)
    arm.set_retreat(None)  # 戻り終えた（このあとの中止では、その場で weight を下げる）
    return plan, results
