"""教示した姿勢から、人の角度 → ロボットの関節値を求める（補間）。

教示（teach.py）は、人の姿勢 H ごとに「ロボットをこの形にしたい」という意図した姿勢を残す。人の関節とロボットの関節は
1対1に対応しない（肘を手首で曲げる、など）ので、関節ごとの直線にはしない。2つに分けて求める。

- 台座（shoulder_pan）: 人の pan だけから、教示点の間を折れ線でつなぐ。人の pan が同じ点（pan 0 が多い）は平均した折れ線にし、
  各教示点との差（残り）を、lift・elbow が近い点ほど強く足す。端の外は、端の値で水平。
- ほかの関節（肩・肘・手首）: lift・elbow の2つの角度の空間で、距離の逆数で重みづけして混ぜる（Shepard 補間）。
  pan は距離にほぼ使わない（lift・elbow が同じで pan だけ違う点を見分けるためだけに、ごく小さく足す）。
- 教示点の角度では、保存した姿勢をそのまま返す。
- 肩・肘・手首の結果は、教示した姿勢の値の範囲（凸包）を出ない。教示していない極端な値は作らない。
- 人の pan が測れないとき（上腕がほぼ垂直。pan=None）は、台座を、教示した台座の値の混ぜ合わせにする（pan の折れ線は使わない）。
- 保存値は可動域の外でもそのまま持つ。limits を渡したときだけ、結果を関節ごとの移動制限の中に丸める。
- グリッパーは混ぜない（指の開きで動かすため）。
"""
import math

from limits import JOINTS, clamp_pose
from teach import HUMAN_POSES

FEATURES = ("pan", "lift", "elbow")
ARM_JOINTS = JOINTS[:-1]
PAN_JOINT = "shoulder_pan"
OTHER_JOINTS = tuple(j for j in ARM_JOINTS if j != PAN_JOINT)
SCALE = 30.0       # lift・elbow の差をこの値（度）で割って距離にする。2つの角度を同じ重さで扱う
POWER = 2.0        # 距離の何乗の逆数で重みづけするか。大きいほど、近い教示点を強く採る
PAN_TIE = 1000.0   # pan の差の重み（度）。大きいほど弱い。lift・elbow が同じ点を見分けるためだけ
EXACT = 1e-9       # この距離未満なら、教示点そのものとみなす
HUMAN = {p["id"]: p for p in HUMAN_POSES}


def parse_angles(angles):
    """人の角度を検証する。lift・elbow は必須の有限な数、pan は無くても None でもよい（測れない）。"""
    if not isinstance(angles, dict):
        raise ValueError("角度は辞書で指定してください")
    out = {}
    for key in FEATURES:
        value = angles.get(key)
        if value is None and key == "pan":
            out[key] = None
        elif type(value) in (int, float) and math.isfinite(value):
            out[key] = float(value)
        else:
            raise ValueError(f"{key} は有限な数で指定してください")
    return out


def distance2(query, point):
    """lift・elbow の距離の2乗に、pan が両方にあるときだけ、ごく小さく pan の差を足す。"""
    d2 = sum(((query[k] - point[k]) / SCALE) ** 2 for k in ("lift", "elbow"))
    if query["pan"] is not None and point["pan"] is not None:
        d2 += ((query["pan"] - point["pan"]) / PAN_TIE) ** 2
    return d2


def blend(values, weights):
    """重みつき平均。"""
    return sum(w * v for w, v in zip(weights, values)) / sum(weights)


def pan_curve(pairs):
    """(人の pan, ロボットの台座) の組から、折れ線の関数を作る。同じ pan は平均し、端の外は水平。"""
    groups = {}
    for pan, value in pairs:
        groups.setdefault(pan, []).append(value)
    knots = sorted((pan, sum(v) / len(v)) for pan, v in groups.items())

    def curve(pan):
        if pan <= knots[0][0]:
            return knots[0][1]
        for (a, ya), (b, yb) in zip(knots, knots[1:]):
            if pan <= b:
                return ya + (yb - ya) * (pan - a) / (b - a)
        return knots[-1][1]
    return curve


def solve_pan(query, points, weights):
    """台座の値。人の pan があれば、pan の折れ線に、lift・elbow が近い点の残りを足す。無ければ、台座の値の混ぜ合わせ。"""
    with_pan = [(h, r, w) for (h, r), w in zip(points, weights) if h["pan"] is not None]
    if query["pan"] is None or not with_pan:
        return blend([r[PAN_JOINT] for _, r in points], weights)
    curve = pan_curve([(h["pan"], r[PAN_JOINT]) for h, r, _ in with_pan])
    residuals = [r[PAN_JOINT] - curve(h["pan"]) for h, r, _ in with_pan]
    return curve(query["pan"]) + blend(residuals, [w for _, _, w in with_pan])


def solve(teach, angles, limits=None, power=POWER):
    """人の角度 angles（pan・lift・elbow、度）から、ロボットの5関節の値（内部の値）を返す。teach は teach.json の内容。"""
    query = parse_angles(angles)
    points = [(HUMAN[k], e["robot"]) for k, e in teach.items() if k in HUMAN]
    if not points:
        raise ValueError("教示が1つもありません")
    weights = []
    for human, robot in points:
        d2 = distance2(query, human)
        if d2 < EXACT:
            pose = {j: robot[j] for j in ARM_JOINTS}
            return clamp_pose(pose, limits) if limits else pose
        weights.append(d2 ** (-power / 2))
    pose = {j: blend([robot[j] for _, robot in points], weights) for j in OTHER_JOINTS}
    pose[PAN_JOINT] = solve_pan(query, points, weights)
    pose = {j: pose[j] for j in ARM_JOINTS}
    return clamp_pose(pose, limits) if limits else pose


def leave_one_out(teach, power=POWER):
    """各教示点を1つ外して補間し、外した点の値とのずれ（関節ごとの差の最大）を返す。補間の当たり具合の目安（H の id → ずれ）。"""
    out = {}
    for key, e in teach.items():
        rest = {k: v for k, v in teach.items() if k != key}
        if not rest:
            continue
        guess = solve(rest, {f: HUMAN[key][f] for f in FEATURES}, power=power)
        out[key] = max(abs(guess[j] - e["robot"][j]) for j in ARM_JOINTS)
    return out
