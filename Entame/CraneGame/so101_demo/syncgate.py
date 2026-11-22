"""追従の開始時の「同期待ち」。手の姿勢と腕の姿勢が合うまで、腕には指令を出さない（暴れ・飛び出しの防止）。

手の入力（-1〜1）を、腕の今の姿勢を同じ尺度（-1〜1）にしたものと比べる。すべての関節のずれが許容内に
連続して KEEP_SAMPLES 回収まったら、追従を始める（腕が動き出す）。始まった後も、ずれ（追従の遅れ）は画面に出す。
"""
from home import HOME_JOINTS, input_of
from limits import raw

TOLERANCE = 0.25      # 手と腕のずれの許容（-1〜1 の尺度。1 は「初期姿勢から上限（下限）まで」）
KEEP_SAMPLES = 5      # 許容内が、この回数つづいたら開始（約 0.3 秒）


class SyncGate:
    def __init__(self, enabled):
        self.enabled = enabled
        self.reset()

    def reset(self):
        self.engaged = not self.enabled
        self.kept = 0
        self.gaps = {}

    def update(self, free, current, limits, home):
        """手の入力 free（関節 → -1〜1）と腕の姿勢 current から、ずれを測り、追従してよいかを返す。"""
        self.gaps = {}
        for joint, hand in free.items():
            if joint not in HOME_JOINTS:
                continue
            low, high = raw(joint, limits[joint]["min"]), raw(joint, limits[joint]["max"])
            self.gaps[joint] = dict(hand=hand, robot=input_of(current[joint], low, high, home and home[joint]))
        if not self.engaged:
            self.kept = self.kept + 1 if self.matched() else 0
            self.engaged = self.kept >= KEEP_SAMPLES
        return self.engaged

    def matched(self):
        return all(abs(g["hand"] - g["robot"]) <= TOLERANCE for g in self.gaps.values())

    def off(self):
        """許容を外れている関節の名前（画面の案内用）。"""
        return [j for j, g in self.gaps.items() if abs(g["hand"] - g["robot"]) > TOLERANCE]

    def state(self):
        return dict(waiting=not self.engaged, tolerance=TOLERANCE, gaps=self.gaps)
