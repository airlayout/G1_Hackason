"""腕の定型動作の定義（右腕 7 関節の角度の列）。

関節の順: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw [deg]。
動作は「開始姿勢 → 各段 → 開始姿勢へ戻る」の順に送る（戻る段は runner が自動で足す）。

⚠️ 下の角度は FK（common/realday.py の FingertipFK、pelvis 座標）で求めた計画値。実機で動かしたかは各動作のコメントに書く。
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


# 敬礼（既定）: 肘を横へ張り、前腕から手先までまっすぐにして、指先を額の右に当てる。正面からは手の側面（小指側の縁）が見える。
# 2026-11-14 に MuJoCo（公式モデル・胴体固定）で、指先 (0.10, -0.06, 0.50)・肘 (0.08, -0.30, 0.25)・掌の向きの前後成分 0
# を目標に、手首 pitch / yaw を 0 に固定して最適化した。手の側面を前へ向ける分は手首 roll（前腕のひねり）だけでまかなう。
# メッシュの最短距離: 手と頭 9.7 cm、腕と胴 10.0 cm（動く途中も頭へは 9.3 cm より近づかない）。
SALUTE = Action(
    name="salute",
    description="右肘を横へ張り、まっすぐな手を額の右に当てて 2 秒止まり、戻る（敬礼）",
    steps=(
        Step("肘を横へ張って手を額へ", (-30.9, -63.5, -27.3, -53.9, 47.1, 0.0, 0.0), hold_s=2.0),
    ),
    planned_tip_m=(0.10, -0.058, 0.502),
    caution="肘を横へ張るので、右横に人や物が無いこと。手と頭の間は約 10 cm（MuJoCo）。実機では追従誤差 4〜5° があるので目で見る。",
)

# 敬礼（手首を曲げる）: salute と同じく肘を横へ張り、手の側面を前へ向けるが、手首を曲げて手先を額へ向ける（2026-11-14 の案 F）。
# salute は手首をまっすぐにした版（案 G）。目標は同じ（指先 (0.10, -0.06, 0.50)・肘 (0.08, -0.30, 0.25)・掌の向きの前後成分 0）。
# メッシュの最短距離: 手と頭 9.5 cm、腕と胴 9.0 cm。
SALUTE_WRIST_BENT = Action(
    name="salute_wrist_bent",
    description="右肘を横へ張り、手首を曲げて手を額の右に当てて 2 秒止まり、戻る（敬礼・手首を曲げる）",
    steps=(
        Step("肘を横へ張って手首を曲げ、手を額へ", (-32.8, -66.0, -4.4, -48.9, 27.7, -20.3, -20.8), hold_s=2.0),
    ),
    planned_tip_m=(0.10, -0.06, 0.50),
    caution="肘を横へ張るので、右横に人や物が無いこと。手と頭の間は約 10 cm（MuJoCo）。",
)

# 敬礼（肘を高く）: salute より肘を肩の高さまで上げる。手首は少し曲がる（pitch -28.9°）。
# 目標: 指先 (0.10, -0.06, 0.53)・肘 (0.08, -0.30, 0.30)。メッシュの最短距離: 腕と頭 9.9 cm、腕と胴 6.2 cm。
SALUTE_ELBOW_HIGH = Action(
    name="salute_elbow_high",
    description="右肘を肩の高さまで横へ張り、手を額の右に当てて 2 秒止まり、戻る（敬礼・肘を高く）",
    steps=(
        Step("肘を高く張って手を額へ", (-50.9, -75.6, -23.5, -26.6, 8.8, -28.9, 10.8), hold_s=2.0),
    ),
    planned_tip_m=(0.10, -0.06, 0.53),
    caution="肩を横へ大きく上げる（肩 roll -76°）ので、salute より肩にかかる重さが大きい。右横に人や物が無いこと。",
)

# 敬礼（簡易版）: 肘は体の前に出し、手を顔の右に立てる。2026-11-13 に実機で完走（腰の上限 5°、腰 pitch の最大 +2.9°）。
SALUTE_SIMPLE = Action(
    name="salute_simple",
    description="右手を顔の右まで上げて 2 秒止まり、戻る（敬礼の簡易版）",
    steps=(
        Step("手を顔の右へ上げる", (-41.3, -4.3, -23.2, -53.6, 0.0, 0.0, 0.0), hold_s=2.0),
    ),
    planned_tip_m=(0.12, -0.17, 0.52),
    caution="手と頭の間は約 12 cm（MuJoCo のメッシュの最短距離）。",
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

ACTIONS: dict[str, Action] = {a.name: a for a in (SALUTE, SALUTE_WRIST_BENT, SALUTE_ELBOW_HIGH, SALUTE_SIMPLE, POINT, POINT_LEVEL)}
