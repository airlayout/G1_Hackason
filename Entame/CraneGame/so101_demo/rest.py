"""畳んだ姿勢（ロボットの休め姿勢）。人が腕を下ろして肘を曲げた姿勢（H2）の教示を、そのまま使う。

SO-101 を畳むと、上腕と前腕が重なって台座の上に収まる。モーターを OFF にしても保てる、安定した姿勢になる。
人の腕の下ろした姿勢（H2: 上腕を下ろし肘を直角に）を、この姿勢に対応づけておくと、追従の前後の行き先に使える。

- 値は新しく持たず、教示（teach.json の H2）を読む。教示し直せば、畳んだ姿勢も変わる。
- 教示は可動域の外の値も持つ（モーター OFF で手が動かせる範囲が広いため）。ここでは limits の中に丸めて返す。
- H2 が未教示なら None（行き先がない）。
"""
from limits import clamp_pose
from teach import HUMAN_IDS

REST_ID = "H2"
REST_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")   # グリッパーは含めない


def rest_pose(teach, limits):
    """畳んだ姿勢（5関節。内部の値。limits の中に丸める）。H2 が未教示なら None。"""
    if REST_ID not in HUMAN_IDS:
        raise ValueError(f"{REST_ID} は人の姿勢にありません")
    entry = teach.get(REST_ID)
    if not entry:
        return None
    return clamp_pose({j: entry["robot"][j] for j in REST_JOINTS}, limits)
