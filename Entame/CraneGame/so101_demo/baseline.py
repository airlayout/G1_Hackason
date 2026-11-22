"""追従の開始時に取る「基準」。人が手と腕の姿勢を合わせるのではなく、システムが合わせる。

追従を始めた直後、手が少し止まった瞬間（KEEP_SAMPLES 回ぶん、どの関節も SPREAD 以内）の手の入力を「手の基準」、
そのときの腕の姿勢を「腕の基準」にする。以後、手が手の基準から動いた分（relative）だけ、腕が腕の基準から動く。
手の構えは何でもよく、手の基準の側に余地が少なくても、反対側へ動かせば端まで届く（両端は保つ）。
基準を取るまでの間は、腕に指令を出さない。手が止まらなくても MAX_SAMPLES 回で、そのときの平均で取る。
"""
from home import HOME_JOINTS, input_of, relative
from limits import raw

KEEP_SAMPLES = 5      # この回数ぶん、手が止まっていたら基準を取る（約 0.3 秒）
MAX_SAMPLES = 40      # 手が止まらなくても、この回数で基準を取る（約 2.6 秒）
SPREAD = 0.3          # 「止まっている」とみなす、入力（-1〜1）の揺れの幅
TOLERANCE = 0.25      # 画面で、手と腕が「合っている」とみなす差（追いつくまでの遅れの目安）


class Baseline:
    def __init__(self, enabled):
        self.enabled = enabled
        self.reset()

    def reset(self):
        self.ready = not self.enabled
        self.recent, self.total = [], 0
        self.hand, self.anchor, self.gaps = {}, {}, {}

    def update(self, free, current, limits):
        """手の入力 free（関節 → -1〜1）と腕の姿勢 current から、基準を取る／ずれを測る。基準ができていれば True。"""
        if not self.ready:
            self.take(free, current)
        self.gaps = {}
        if self.ready and self.enabled:
            for joint, value in free.items():
                if joint in self.hand:
                    low, high = raw(joint, limits[joint]["min"]), raw(joint, limits[joint]["max"])
                    self.gaps[joint] = dict(hand=relative(value, self.hand[joint]),
                                            robot=input_of(current[joint], low, high, self.anchor[joint]))
        return self.ready

    def take(self, free, current):
        self.recent = (self.recent + [dict(free)])[-KEEP_SAMPLES:]
        self.total += 1
        joints = [j for j in HOME_JOINTS if all(j in sample for sample in self.recent)]
        steady = all(max(s[j] for s in self.recent) - min(s[j] for s in self.recent) <= SPREAD for j in joints)
        if len(self.recent) >= KEEP_SAMPLES and (steady or self.total >= MAX_SAMPLES):
            self.hand = {j: sum(s[j] for s in self.recent) / len(self.recent) for j in joints}
            self.anchor = {j: float(current[j]) for j in HOME_JOINTS}
            self.ready = True

    def hand_input(self, joint, value):
        """手の入力を、腕へ写す入力（-1〜1。0 が腕の基準）にする。基準が無い関節は None（動かさない）。"""
        if not self.enabled:
            return value
        return relative(value, self.hand[joint]) if joint in self.hand else None

    def rebase(self, joint, value, current):
        """追従の途中でONにした関節の基準を、その時点の手の入力と腕の姿勢で取り直す（基準を使うときだけ）。"""
        if self.enabled and self.ready:
            self.hand[joint] = value
            self.anchor[joint] = float(current[joint])

    def reference(self, joint, home):
        """入力 0 に当たる腕の位置（内部の値）。基準を使わないときは初期姿勢（未登録なら None＝範囲の真ん中）。"""
        return self.anchor[joint] if self.enabled else home and home[joint]

    def state(self):
        return dict(waiting=not self.ready, tolerance=TOLERANCE, gaps=self.gaps)
