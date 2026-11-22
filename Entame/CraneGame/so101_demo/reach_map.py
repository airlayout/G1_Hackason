"""手の入力（前へ伸ばす・高さ）から、腕の先端の目標を決め、肩・肘・手首の関節値にする（ik_arm.py の逆運動学を使う）。

関節ごとに手の入力を結ぶと、肩が手の高さだけで端まで引かれて「後ろ上へ振り上がる」異常な姿勢が出る
（2026-11-22 録画ログ。利用者の指摘: この動きは想定しておらず、使われることはない）。ここでは、
- 肩・肘・手首は、必ず「使ってよい範囲」（教示した使う範囲）に入る。入らない目標は、入る所まで手前に引く。
- 低いとき（机の近く）は道具を真下へ、高く上げるほど水平へ向ける（グリッパーが宙を向かない）。
値はすべて、実機で確かめていない設計値（根拠は各定数のコメント）。机の高さ・お菓子の位置は未測定。
"""
import math

import ik_arm

# 机の高さ（台座の軸の高さを 0 としたモデルの z）。初期姿勢は「グリッパーが机に接した姿勢」（home.py）で、その先端が z=-0.013。
# 実機で定規では測っていない（CRANE_PLAN.md §2）。
Z_DESK = -0.013
Z_MIN = Z_DESK + 0.005         # 先端が机にめり込まない余裕
DESK_TOUCH = 0.005             # 机に接した姿勢（初期姿勢）の高さの誤差の許容（m）
HEIGHT_MAX = 0.20             # グリッパー先端の机からの高さの上限（m）。教示 H4（腕を前へ水平に伸ばす）は約 0.17 m。未測定の設計値
PITCH_RANGE = (-115.0, 0.0)    # 道具の向き（度）の許す範囲。初期姿勢（机に接する姿勢）がモデル上 -111° なのでそれを含める。上向き（> 0）は使わない
Z_MAX = Z_DESK + HEIGHT_MAX
X_MIN = 0.18                   # 先端の前後の動く範囲（台座の軸から）。これより手前は、道具を下に向けると関節が使ってよい範囲を出る
X_MAX = 0.44                   # 手の入力 +1（伸ばし切り）の先端 x。机の高さで、腕がまっすぐ伸びて届く最遠（道具は -35° 付近。下の掃引で確認）
PITCH_LOW, PITCH_HIGH = -80.0, -15.0      # 道具の向き（度）。机の近く＝真下、高い所＝ほぼ水平（教示 H4 は -15°）
Z_PITCH_LOW, Z_PITCH_HIGH = 0.0, 0.20     # 向きを変える高さの範囲
PITCH_RELAX = 45.0                        # 遠くて届かないとき、道具を上向きへ傾けてよい最大（度）。-80° → -35°。腕をまっすぐ伸ばし切ると、道具は斜め前へ向く

# 使ってよい関節値（-100〜100）。肩は後ろ（負）へは -35 より先に行かせない。前（正）と肘の下限は、腕をまっすぐ伸ばせるところまで。
# 肘: 腕がまっすぐになる値は約 -75（ik_arm の形から。肩の値によらない）。下限が -5 だと L 字（約 74° 曲がった形）までしか伸びず、
# 机の高さで先端が x=0.30〜0.35 m までしか届かなかった（利用者の指摘 2026-11-22: 腕を伸ばしても肘が浅く、遠くへ届かない）。
# 肩: 腕をまっすぐ前へ向けるのに +60〜+80 が要る（道具の向き -80〜-30°で計算）。校正の範囲（±100）の内側。
# 実機でこの姿勢（肘をほぼ伸ばし切る・肩を前へ倒す）が机・台座にぶつからないかは未確認。
USABLE = {"shoulder_lift": (-35.0, 80.0), "elbow_flex": (-80.0, 100.0), "wrist_flex": (-30.0, 100.0)}
MARGIN = 4.0     # 目標姿勢は USABLE の端からこれだけ内側に取る（サーボの不感帯 replay.DEAD 分の押し込みが USABLE の外に出ないように）


def pitch_for(z):
    """高さ z の道具の向き（度）。"""
    f = max(0.0, min(1.0, (z - Z_PITCH_LOW) / (Z_PITCH_HIGH - Z_PITCH_LOW)))
    return PITCH_LOW + (PITCH_HIGH - PITCH_LOW) * f


def joint_units(calibration):
    """較正（lerobot の so101.json）から、各関節の 1 単位（関節値 1）あたりの角度 rad。"""
    out = {}
    for joint in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        c = calibration[joint]
        out[joint] = (c["range_max"] - c["range_min"]) / 2 * 2 * math.pi / 4096 / 100
    return out


def solve(x, z, units):
    """先端 (x, z) の関節値（辞書）と、使ってよい範囲に入るか。道具の向きは pitch_for(z) を基本に、
    遠くて届かないときだけ PITCH_RELAX（度）まで上向きへ緩める。入らないとき、値は範囲に丸めてある。"""
    for relax in range(0, int(PITCH_RELAX) + 1, 5):
        lift, elbow, wrist, reached = ik_arm.inverse(x, z, pitch_for(z) + relax)
        values = {"shoulder_lift": lift / units["shoulder_lift"], "elbow_flex": elbow / units["elbow_flex"],
                  "wrist_flex": wrist / units["wrist_flex"]}
        inside = reached and all(lo + MARGIN <= values[j] <= hi - MARGIN for j, (lo, hi) in USABLE.items())
        if inside:
            break
    return {j: min(USABLE[j][1], max(USABLE[j][0], v)) for j, v in values.items()}, inside


def tip_of(pose, units):
    """関節値の姿勢から、先端 (x, z) と道具の向き（度）。"""
    return ik_arm.forward(*(pose[j] * units[j] for j in ("shoulder_lift", "elbow_flex", "wrist_flex")))


def height_above_desk(pose, units):
    """関節値の姿勢での、グリッパー先端の机からの高さ（m）。"""
    return tip_of(pose, units)[0][1] - Z_DESK


def inside_envelope(pose, units):
    """使ってよい姿勢か。先端の机からの高さが HEIGHT_MAX 以下、道具が PITCH_RANGE の向き、肩が使ってよい下限以上。
    手の入力の結び方（関節ごと・IK）に関わらず、この外の姿勢は出させない（利用者の指示 2026-11-22）。"""
    (_, z), pitch = tip_of(pose, units)
    return (z - Z_DESK <= HEIGHT_MAX and z >= Z_DESK - DESK_TOUCH
            and PITCH_RANGE[0] <= pitch <= PITCH_RANGE[1]
            and all(lo <= pose[j] <= hi for j, (lo, hi) in USABLE.items()))


def clamp_to_envelope(want, safe, units, steps=32):
    """目標の姿勢 want が包絡の外なら、安全な姿勢 safe から want へ向かう関節空間の直線上で、包絡内で最も遠い点を返す。"""
    if inside_envelope(want, units):
        return dict(want)
    best = dict(safe)
    for i in range(1, steps + 1):
        f = i / steps
        mix = {j: safe[j] + (want[j] - safe[j]) * f for j in want}
        if not inside_envelope(mix, units):
            break
        best = mix
    return best


def pull_in(x, z, units, steps=24):
    """目標 (x, z) が使ってよい範囲の外なら、範囲内の安全な場所（初期の高さ・前後）から目標へ向かう線上で、行ける所まで進めた点を返す。"""
    if solve(x, z, units)[1]:
        return x, z
    sx, sz = 0.20, Z_MIN
    best = (sx, sz)
    for i in range(1, steps + 1):
        f = i / steps
        p = (sx + (x - sx) * f, sz + (z - sz) * f)
        if solve(*p, units)[1]:
            best = p
        else:
            break
    return best


def neutral(value, low, high, base):
    """入力 -1〜1（0 が基準 base、+1 が high、-1 が low）を、low〜high の中へ写す。home.neutral_map と同じ形。"""
    base = min(high, max(low, base))
    return base + (high - base) * value if value >= 0 else base + (base - low) * value


def target(reach, height, anchor, units, safe=None):
    """基準からの手の動き reach（前へ伸ばす＋）・height（上げる＋、各 -1〜1）から、先端の目標 (x, z) と関節値を返す。
    reach=+1（手を伸ばし切った）は X_MAX＝机の高さで腕がまっすぐ伸びきる所。
    anchor は動き出す前の先端 (x, z)。safe（初期姿勢の関節値）を渡すと、結果を包絡（clamp_to_envelope）の内側に収める。"""
    x = neutral(reach, X_MIN, X_MAX, anchor[0])
    z = neutral(height, Z_MIN, Z_MAX, anchor[1])
    x, z = pull_in(x, z, units)
    pose, _ = solve(x, z, units)
    if safe is not None:
        pose = clamp_to_envelope(pose, safe, units)
        (x, z), _ = tip_of(pose, units)
    return (x, z), pose
