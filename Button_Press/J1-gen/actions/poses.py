"""腕の定型動作の定義（右腕 7 関節の角度の列）。

関節の順: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw [deg]。
動作は「開始姿勢 → 各段 → 開始姿勢へ戻る」の順に送る（戻る段は runner が自動で足す）。

⚠️ 下の角度は FK（common/realday.py の FingertipFK、pelvis 座標）で求めた計画値で、**実機ではまだ動かしていない**。
   最初は sim で見て（real/run_action.py --path sim）、実機は dry-run → 確認モードで 1 段ずつ進める。
   指先の位置は pelvis 座標（x 前・y 左・z 上）。肩の位置は (0, -0.10, 0.29)、頭カメラは z 約 0.46。
   関節の符号の決まり: shoulder_pitch が負で腕が前へ上がる、elbow が負で肘が曲がり手が上がる（ゼロ姿勢は肘を約 90° 曲げて前に出した姿勢）。
   左腕で使うときは arm の設定が left のとき ARM_MIRROR_JOINTS の符号を反転する（common/robot_model.mirror_arm_q）。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Step:
    """1 段: q_deg の姿勢へ動き、hold_s 秒保つ。duration は動く秒数（None なら上限速度から自動）。"""

    label: str
    q_deg: tuple[float, float, float, float, float, float, float]
    hold_s: float = 0.0
    duration: float | None = None


@dataclass(frozen=True)
class Action:
    name: str
    description: str
    steps: tuple[Step, ...]
    # 指先の計画値（pelvis 座標 [m]）。tests/test_actions.py が FK で確かめる。最後の段の値
    planned_tip_m: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))
    caution: str = ""


# 敬礼: 手を顔の右（頭の右前、耳の少し前）まで上げて止まる。肘は体の前に出す形（肘を横へ張る敬礼ではない）。
# 肘を横へ張る形は、手首が 80° 以上曲がる解になり、まだ検証していないので使っていない。
SALUTE = Action(
    name="salute",
    description="右手を顔の右まで上げて 2 秒止まり、戻る（敬礼の簡易版）",
    steps=(
        Step("手を顔の右へ上げる", (-41.3, -4.3, -23.2, -53.6, 0.0, 0.0, 0.0), hold_s=2.0),
    ),
    planned_tip_m=(0.12, -0.17, 0.52),
    caution="手と頭の間は約 19 cm（前腕と頭の中心の距離）。顔に当たる姿勢ではないが、目視で確かめる。",
)

# 指さし（既定）: 腕を前下へ伸ばして指す。水平まで上げないので、体の前傾が小さい。
POINT = Action(
    name="point",
    description="右腕を前下へ伸ばして 3 秒止まり、戻る（指さし確認）",
    steps=(
        Step("腕を前下へ伸ばす", (-52.7, -36.1, 33.0, 34.7, 0.0, 0.0, 0.0), hold_s=3.0),
    ),
    planned_tip_m=(0.50, -0.12, 0.22),
    caution="腕を前へ出すので体が前へ傾くことがある（docs/knowledge/patterns/robot_arm_control.md）。body_motion の傾きを見る。",
)

# 指さし（水平）: 肩の高さで、まっすぐ前へ。体への負担が大きいので、point で確かめてから使う。
POINT_LEVEL = Action(
    name="point_level",
    description="右腕を肩の高さで前へまっすぐ伸ばして 3 秒止まり、戻る（指さし確認・水平）",
    steps=(
        Step("腕を前へ水平に伸ばす", (-92.7, -16.2, -30.5, 99.7, 0.0, 0.0, 0.0), hold_s=3.0),
    ),
    planned_tip_m=(0.55, -0.10, 0.29),
    caution="腕をいちばん前へ出す姿勢。point で体の傾きを確かめてから使う。",
)

ACTIONS: dict[str, Action] = {a.name: a for a in (SALUTE, POINT, POINT_LEVEL)}
