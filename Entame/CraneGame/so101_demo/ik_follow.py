"""IK 追従（実機サーバー用の、既定オフの選択肢）。シミュレーションで確かめた追従（crane_sim.run）と同じ計算を、
関節ごとの写し方の代わりに使う。mujoco / rerun には依存しない。

手の「前へ伸ばす」「上げる」の 2 つの入力から、先端の目標 (x, z) を決め、平面の逆運動学で肩・肘・手首の曲げを同時に決める。
先端は包絡（机から一定の高さ以下・道具の向き・肩の下限。reach_map.inside_envelope）の外へ出さない。
**実機では未確認**（2026-11-22 夜時点。シミュレーションだけで確かめた）。入力は One Euro、関節の指令は臨界減衰で滑らかにする。
"""
import reach_map
from home import relative
from smoothing import CriticallyDamped, OneEuro

INPUT_MIN_CUTOFF = 1.0        # 手の入力の One Euro フィルタ: 静止時のカットオフ（Hz）。小さいほど静止時に強く平らにする
INPUT_BETA = 2.0              # 同: 速い動きでカットオフを上げる係数。大きいほど速い動きの遅れが小さい
COMMAND_TIME_CONSTANT = 0.08  # 関節指令の臨界減衰フィルタの時定数（秒）。サーボの指令の階段・向き反転を平らにする
INPUT_JOINTS = ("elbow_flex", "shoulder_lift")   # 手の入力が来る関節（reach ← 肘の入力、height ← 肩の入力の符号反転）
CONTROLLED_JOINTS = tuple(reach_map.USABLE)      # IK が決める関節（肩・肘・手首の曲げ）
START_TOLERANCE = 15.0        # 追従開始時、この 3 関節が初期姿勢からこれ以上離れていたら IK 追従は使わない（関節値）。
                              # 離れていると、最初の 1 歩で初期姿勢へ向かって大きく動いてしまうため
# 入力 0（手が基準のまま）でも、IK の目標は初期姿勢と違う（2026-11-22 計算: 初期姿勢は先端が机の高さ・道具の向き -111°、
# IK は机の高さの向き -80° を選ぶので、肩 -39・肘 +46・手首 -34 の差）。追従を始めた瞬間に腕が跳ばないよう、
# 最初の RAMP_S 秒をかけて、初期姿勢から IK の目標へ寄せる。
RAMP_S = 3.0


def near_home(current, home, tolerance=START_TOLERANCE):
    """3 関節がどれも初期姿勢の近く（tolerance 以内）か。"""
    return all(abs(float(current[j]) - float(home[j])) <= tolerance for j in CONTROLLED_JOINTS)


def hand_motion(hand_base, free):
    """手の入力 free（関節 → -1〜1）を、基準 hand_base（追従開始時の手の入力）を 0 にした (reach, height) にする。
    シミュレーション（crane_sim.recorded_inputs）と同じく、height は肩の入力の符号を反転したもの。"""
    reach = relative(free["elbow_flex"], hand_base["elbow_flex"])
    height = relative(-free["shoulder_lift"], -hand_base["shoulder_lift"])
    return reach, height


class IkFollow:
    """追従 1 回ぶんの状態（フィルタ）。home は初期姿勢（動き出す前。入力 0 の先端はここ）。"""

    def __init__(self, units, home):
        self.units = units
        self.anchor = reach_map.tip_of(home, units)[0]
        self.safe = {j: float(home[j]) for j in reach_map.USABLE}
        self.inputs = [OneEuro(INPUT_MIN_CUTOFF, INPUT_BETA) for _ in range(2)]
        self.commands = {j: CriticallyDamped(COMMAND_TIME_CONSTANT) for j in CONTROLLED_JOINTS}
        self.started = False
        self.elapsed = 0.0
        self.last = None             # 直近の (滑らかにした reach, height)（ログ用）

    def step(self, reach, height, dt):
        """手の動き（-1〜1）と経過時間 dt から、3 関節の目標（関節値 -100〜100）を返す。
        最初の RAMP_S 秒は、初期姿勢から IK の目標へ、なだらかに寄せる（下の RAMP_S の説明）。"""
        step_dt = dt if self.started else 0.0
        self.started = True
        self.elapsed += step_dt
        reach, height = (f(v, step_dt) for f, v in zip(self.inputs, (reach, height)))
        self.last = (reach, height)
        _, want = reach_map.target(reach, height, self.anchor, self.units, self.safe)
        w = min(1.0, self.elapsed / RAMP_S)
        w = w * w * (3 - 2 * w)
        return {j: self.commands[j](self.safe[j] + w * (want[j] - self.safe[j]), step_dt) for j in CONTROLLED_JOINTS}
