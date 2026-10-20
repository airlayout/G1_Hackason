"""行動と、体の感覚の観測（学習環境と、評価環境の Agent で同じ計算にする部分）。

学習環境（tasks/press_env.py、torch）と、評価に出す Agent（agent/agent.py、numpy）の両方がここを使う。
2 つで食い違うと、学習した方策が評価で動かないので、計算はここにだけ書く（tests/test_proprio.py で確かめる）。

行動: 右腕 7 関節の目標の角度の差分。a ∈ [−1, 1] に ACTION_SCALE を掛けて、前の目標に足す。
ACTION_SCALE は、評価のランナーの安全のための処理（1 周期の最大移動量 = 1.0 rad/s × 0.02 s）と同じにしてあるので、
方策の出す目標は評価で丸められない。目標は関節の可動範囲から limit_margin 内側に丸める（ランナーと同じ）。

体の感覚（PROPRIO_DIM 個）:
  右腕の角度 − 初期の角度 (7)、右腕の速度 × 0.1 (7)、
  （目標 − 角度）× 5 (7)  ← 押すとここが大きくなる（力センサが無くても、押す力の手がかりになる。実機でも取れる）
  目標 − 初期の角度 (7)、前の行動 (7)
"""

from __future__ import annotations

from typing import Any

# 右腕（motor 22〜28）の関節の名前。contest/interface.py の JOINT_NAMES[22:29] と同じ
RIGHT_ARM_JOINTS: tuple[str, ...] = (
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
NUM_ACTIONS = len(RIGHT_ARM_JOINTS)
ACTION_SCALE = 0.02  # [rad / 周期]（= 1.0 rad/s × 0.02 s）
DQ_SCALE = 0.1
ERR_SCALE = 5.0
PROPRIO_DIM = 5 * NUM_ACTIONS


def step_command(q_cmd: Any, action: Any, lo: Any, hi: Any, clip: Any) -> Any:
    """前の目標 q_cmd に行動を足し、可動範囲（lo〜hi、余裕を引いたもの）に丸めた新しい目標。

    clip は numpy.clip か torch.clamp（どちらでも同じ引数の順）。
    """
    return clip(q_cmd + ACTION_SCALE * clip(action, -1.0, 1.0), lo, hi)


def proprio(q: Any, dq: Any, q_cmd: Any, q0: Any, last_action: Any, cat: Any) -> Any:
    """体の感覚の観測。最後の次元に並べる。cat は numpy.concatenate か torch.cat（axis / dim は −1）。"""
    return cat([q - q0, DQ_SCALE * dq, ERR_SCALE * (q_cmd - q), q_cmd - q0, last_action], -1)


def policy_obs(prop: Any, face: Any, tip: Any, cat: Any) -> Any:
    """方策（教師）の観測: 体の感覚 + ボタンの面の位置 + 指先の位置 + その差 × 10（腰の座標 [m]）。"""
    return cat([prop, face, tip, 10.0 * (tip - face)], -1)


POLICY_OBS_DIM = PROPRIO_DIM + 9
