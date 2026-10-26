"""段 5（2026-10-23 の再発防止）: HOMIE だけで立つ段（STAND）の見張り。numpy だけ。door_run_real.py から使う。

10-23 の実機: STAND に入って 1.5 秒で足首から揺れが始まり、4 秒で股の yaw が 18 rad/s、6 秒で 20.8 rad/s に達して自動で脱力した。
原因は、右足に体重が乗っていなかったこと（綱が体重を受けていたか、右足が床に着ききっていなかった）。
  STAND の最初の 1 秒の足首（pitch）の力: 10-16（144 秒立てた）L +11.0・R +9.8 N·m ／ 10-23（揺れた）L +7.0・R +1.9 N·m
  ⚠️ A を押す前の HOLD の力では見分けられない（10-16 +3.4・+3.6、10-23 +1.6・+1.3）。STAND に入ってから見る

見張りは 2 つ。どちらも脱力ではなく「立ち直し」（今の姿勢から 3 秒で決まった姿勢へ戻して固く保つ＝RAMP → HOLD）に戻す:
  load_problem         STAND の LOAD_WINDOW_S（0.5〜1.5 s）の足首の力の中央値が、どちらも LOAD_MIN_NM 以上で、
                       軽い方 ÷ 重い方が LOAD_MIN_RATIO 以上でなければ、片足が軽い
  oscillation_problem  STAND の間、足の 12 関節の速さが OSC_MAX_DQ を越える・傾きが OSC_MAX_TILT_DEG を越えたら、揺れが育っている
                       （10-16 に立てた STAND は最大 0.8 rad/s・5.7°。10-23 は 3.0 s で 6.8 rad/s・12.2°、3.5 s で 13 rad/s）
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

ANKLE_PITCH_IDX = (4, 10)  # SDK の関節の番号: 左・右の足首 pitch
LEG_IDX = slice(0, 12)
LOAD_WINDOW_S = (0.5, 1.5)  # STAND に入ってから（最初の 0.5 s は既定の姿勢から HOMIE の目標へ移している）
LOAD_MIN_NM = 5.0
LOAD_MIN_RATIO = 0.5
OSC_MAX_DQ = 6.0  # [rad/s]
OSC_MAX_TILT_DEG = 10.0


def load_problem(ankle_l: Sequence[float], ankle_r: Sequence[float]) -> str:
    """STAND の 0.5〜1.5 s の足首 pitch の力 [N·m] の並び → 片足が軽ければ理由（よければ ""）"""
    if len(ankle_l) == 0 or len(ankle_r) == 0:
        return "足首の力の記録が無い"
    left, right = float(np.median(ankle_l)), float(np.median(ankle_r))
    lo, hi = min(left, right), max(left, right)
    if lo < LOAD_MIN_NM or (hi > 0 and lo / hi < LOAD_MIN_RATIO):
        light = "左" if left < right else "右"
        return (f"{light}足に体重が乗っていない（足首の力 左 {left:+.1f}・右 {right:+.1f} N·m、"
                f"どちらも {LOAD_MIN_NM:.0f} 以上・軽い方が重い方の {LOAD_MIN_RATIO:.0%} 以上が要る）")
    return ""


def oscillation_problem(dq: np.ndarray, tilt_deg: float) -> str:
    """STAND の 1 周期: 足の関節の速さ・傾きから、揺れが育っていれば理由（よければ ""）"""
    legs = np.abs(np.asarray(dq)[LEG_IDX])
    j = int(np.argmax(legs))
    if legs[j] > OSC_MAX_DQ:
        return f"足の関節 {j} の速さ {legs[j]:.1f} rad/s（STAND の上限 {OSC_MAX_DQ:.0f}）"
    if tilt_deg > OSC_MAX_TILT_DEG:
        return f"傾き {tilt_deg:.1f}°（STAND の上限 {OSC_MAX_TILT_DEG:.0f}°）"
    return ""


class StandLoadCheck:
    """STAND の間に足首の力をためて、LOAD_WINDOW_S が終わった周期に 1 回だけ判定する"""

    def __init__(self) -> None:
        self.left: list = []
        self.right: list = []
        self.verdict: Optional[str] = None  # None: まだ・"": 通った・それ以外: 理由

    def update(self, t_in_stand: float, tau: np.ndarray) -> Optional[str]:
        """判定がこの周期に決まったら理由（通ったら ""）を返す。まだ・もう決まっているなら None"""
        if self.verdict is not None:
            return None
        if LOAD_WINDOW_S[0] <= t_in_stand <= LOAD_WINDOW_S[1]:
            self.left.append(float(tau[ANKLE_PITCH_IDX[0]]))
            self.right.append(float(tau[ANKLE_PITCH_IDX[1]]))
            return None
        if t_in_stand > LOAD_WINDOW_S[1]:
            self.verdict = load_problem(self.left, self.right)
            return self.verdict
        return None

    @property
    def passed(self) -> bool:
        return self.verdict == ""
