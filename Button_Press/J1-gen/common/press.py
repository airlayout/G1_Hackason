"""押し込みの実行（PressPlan を ArmCommander で送る）。sim / real 共通。タスク2・6 で使う。"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from .arm import ArmCommander
from .press_planner import PressPlan, PressPlanner

# 段階ごとに呼ぶ関数（段階の名前、その時点の計画）。シミュレーションで手先の位置を測るのに使う
StageCallback = Callable[[str, PressPlan], None]


def execute_press(
    arm: ArmCommander,
    plan: PressPlan,
    hold_s: float,
    return_to: np.ndarray | None = None,
    planner: PressPlanner | None = None,
    on_stage: StageCallback | None = None,
) -> PressPlan:
    """手前の姿勢へ移動 →（planner があれば）実測の腰の角度で計算し直す → 直線で押し込む → 止まる →
    直線で戻る →（与えれば）return_to へ戻る。実際に使った計画を返す。

    計算し直した結果が届かない・ぶつかるなら UnreachableError で止まる（ArmCommander が安全に終了する）。
    各段階で ArmCommander が安全チェックと「動いたか」の確認を行う。確認モードなら段階ごとに Enter を待つ。
    """
    print(f"[press] {plan.summary()}")
    arm.move_to(plan.q_approach, label="手前の姿勢へ移動")
    if on_stage:
        on_stage("approach", plan)
    if planner is not None:
        q_meas = arm.measured_q()
        new = planner.replan(plan, q_meas, arm.commanded_arm_q)
        dw = np.degrees(new.waist_q - plan.waist_q)
        dq = float(np.max(np.abs(new.q_approach - plan.q_approach)))
        print(f"[press] 実測の腰の角度で計算し直した（腰の変化 {np.round(dw, 2)}°、"
              f"手前の姿勢の変化 最大 {np.degrees(dq):.2f}°）")
        plan = new
        if dq > 1e-6:
            arm.move_to(plan.q_approach, label="手前の姿勢を補正")
        if on_stage:
            on_stage("approach_replanned", plan)
    arm.follow(plan.press_in, label="押し込み")
    if on_stage:
        on_stage("end", plan)
    arm.follow([plan.press_in[-1]] * max(1, int(hold_s / arm.dt)), label="押し込んだまま保持")
    arm.follow(plan.press_out, label="戻り")
    if return_to is not None:
        arm.move_to(return_to, label="開始姿勢へ戻る")
    return plan
