"""骨格版（肩〜手の骨格推定で追従する）の、較正とロボット側の動き。画面は skeleton.html。

流れ: ロボットが決まった3姿勢を順に取る → 人が同じ姿勢をとって撮影 → 各関節で「人の角度 → ロボットの値」の直線を引く
→ 追従。人の角度は画面（skeleton_math.js）が骨格から出す。角度の定義と、ロボットの関節との対応:
  pan   上腕を真上から見た向き        → shoulder_pan
  lift  上腕の、真下からの角度        → shoulder_lift
  elbow 肘の角度（伸ばすと 180）      → elbow_flex
  flex  前腕に対する手の曲がり         → wrist_flex
  roll  前腕まわりの、手のひらの回転   → wrist_roll
3姿勢の値（POSE_OFFSETS）は暫定で、実機での向き・安全は未確認。ロボットは初期姿勢（未登録なら今の姿勢）を基準に、
そこから足した姿勢へ、速度制限つきで動く。
"""
import math

from home import HOME_JOINTS
from limits import clamp_pose

FEATURES = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
            "wrist_flex": "flex", "wrist_roll": "roll"}      # 関節 → 人の角度の名前
POSE_NAMES = ("姿勢A（基準）", "姿勢B", "姿勢C")
# 基準の姿勢（初期姿勢）に足す量（内部の値 -100〜100）。どの関節も2通り以上の値を取るようにしてある
POSE_OFFSETS = (
    dict(shoulder_pan=0, shoulder_lift=0, elbow_flex=0, wrist_flex=0, wrist_roll=0),
    dict(shoulder_pan=30, shoulder_lift=-35, elbow_flex=35, wrist_flex=25, wrist_roll=0),
    dict(shoulder_pan=-30, shoulder_lift=35, elbow_flex=-35, wrist_flex=-25, wrist_roll=40),
)
ANGLE_LIMIT = 360.0         # 角度として受け付ける範囲（度）
MIN_SPAN_DEG = 15.0         # 3姿勢での人の角度の幅がこれ未満の関節は、追従できない（差が小さく、ノイズに負ける）
MAX_RESIDUAL = 20.0         # 直線から外れた量（ロボットの内部の値）がこれを超える関節は、追従できない
ARRIVE_TOL = 2.0            # 目標の姿勢へ着いたとみなす差（内部の値）
MOVE_TIMEOUT_S = 15.0       # これだけ待って着かなければ、動くのをやめる


def wrap(degrees):
    """角度の差を -180〜180 に収める。"""
    return (degrees + 180.0) % 360.0 - 180.0


def parse_features(data):
    """画面から送られた人の角度を検証する。送られた項目だけ返す（骨格が隠れた関節は無い）。不正なら fail fast。"""
    if not isinstance(data, dict):
        raise ValueError("人の角度が必要です")
    result = {}
    for key, value in data.items():
        if key not in FEATURES.values():
            raise ValueError(f"不明な角度: {key}")
        if type(value) not in (float, int) or not math.isfinite(value) or not -ANGLE_LIMIT <= value <= ANGLE_LIMIT:
            raise ValueError(f"{key} の角度が範囲外です")
        result[key] = float(value)
    return result


def calibration_poses(base, limits):
    """基準の姿勢から、3姿勢（関節 5 つ）を作る。可動範囲の中に収める。"""
    return [clamp_pose({j: max(-100.0, min(100.0, base[j] + offset[j])) for j in HOME_JOINTS}, limits)
            for offset in POSE_OFFSETS]


def fit_joint(points):
    """(人の角度, ロボットの値) の点から、直線 値 = a * wrap(角度 - ref) + b を引く。使えないときは理由つき。"""
    if len(points) < 2:
        return dict(ok=False, n=len(points), reason="撮影が2枚以上必要です（この関節の角度が写っていない）")
    ref = points[0][0]
    xs = [wrap(angle - ref) for angle, _ in points]
    ys = [value for _, value in points]
    span = max(xs) - min(xs)
    mx, my = sum(xs)/len(xs), sum(ys)/len(ys)
    sxx = sum((x - mx)**2 for x in xs)
    if span < MIN_SPAN_DEG or sxx == 0:
        return dict(ok=False, n=len(points), span=span, reason=f"人の角度の幅が {span:.0f}° しかありません（{MIN_SPAN_DEG:g}° 以上必要）")
    a = sum((x - mx)*(y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a*mx
    resid = max(abs(y - (a*x + b)) for x, y in zip(xs, ys))
    if resid > MAX_RESIDUAL:
        return dict(ok=False, n=len(points), span=span, resid=resid,
                    reason=f"直線から {resid:.0f} ずれています（{MAX_RESIDUAL:g} 以内が必要）。姿勢を撮り直してください")
    if a == 0:
        return dict(ok=False, n=len(points), span=span, reason="ロボットの姿勢がこの関節で変わっていません")
    return dict(ok=True, n=len(points), span=span, resid=resid, a=a, b=b, ref=ref, reason="")


class Calibration:
    """撮影した各姿勢の、ロボットの実際の姿勢と、人の角度。3姿勢のうち撮れた分から、関節ごとの直線を作る。"""

    def __init__(self):
        self.captures = {}          # 姿勢の番号 → dict(robot=姿勢, human=角度)
        self.fits = {}

    def clear(self):
        self.captures, self.fits = {}, {}

    def add(self, index, robot, human):
        self.captures[index] = dict(robot=dict(robot), human=dict(human))
        self.fits = self.fit()

    def fit(self):
        fits = {}
        for joint, key in FEATURES.items():
            points = [(c["human"][key], c["robot"][joint]) for _, c in sorted(self.captures.items()) if key in c["human"]]
            fits[joint] = fit_joint(points)
        return fits

    @property
    def ready(self):
        return any(f["ok"] for f in self.fits.values())

    def target(self, joint, angle):
        """人の角度から、ロボットの関節の値（丸める前）。使えない関節は None。"""
        fit = self.fits.get(joint)
        if not fit or not fit["ok"]:
            return None
        return fit["a"] * wrap(angle - fit["ref"]) + fit["b"]


class PoseMover:
    """較正の姿勢へ、ロボットを動かす。サーバーが一定間隔で step を呼び、指令を少しずつ進める（急に動かさない）。"""

    def __init__(self):
        self.target, self.index, self.started = None, None, 0.0
        self.at = None              # 着いた姿勢の番号

    @property
    def active(self):
        return self.target is not None

    def go(self, index, target, now):
        self.target, self.index, self.started, self.at = dict(target), index, now, None

    def stop(self):
        self.target = None

    def step(self, current, now, goal_fn):
        """次の指令（関節 → 値）。着いた・時間切れ・動いていないときは None。goal_fn(joint, 実際, 目標) が指令を決める。"""
        if self.target is None:
            return None
        if all(abs(self.target[j] - current[j]) < ARRIVE_TOL for j in self.target):
            self.at, self.target = self.index, None
            return None
        if now - self.started > MOVE_TIMEOUT_S:
            self.target = None
            return None
        return {j: goal_fn(j, current[j], t) for j, t in self.target.items()}
