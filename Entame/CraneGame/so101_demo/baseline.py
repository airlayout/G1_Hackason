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

# 腕が可動域の端（たたんだ姿勢）から始まるとき、手の動かせる向きと腕の動かせる向きが逆だと、腕は動けない
# （2026-11-22 実機: たたんだ肩 -99 で始め、手を上げると入力 -1 が下端のさらに先へ向かい、肩が全く動かなかった）。
# その関節は、手の向きを反転して、余地のある側へ動かす。反転したときの動きの大きさは、教えた「腕を上げた」姿勢
# （肩 -29〜-2 ＝ 範囲の約半分）に合わせて FLIP_TRAVEL（範囲の余地に対する割合）で抑える。
# 肘も同じ（2026-11-22 実機ログ: 追従は必ずたたんだ姿勢＝肘 85〜98 から始まる。手を「近づける」と肘は上限の外へ向かい、
# 動けなかった。手の基準が中央〜手前寄り（0 以下）なら反転し、近づけると肘が伸びる向きにする。手の基準が遠い側（+）なら
# 反転しない＝遠ざけると肘が伸びる）。肘は、たたんだ（約 95）から教えた伸ばした姿勢（約 -1〜25）まで動かすので、範囲いっぱい使う。
FLIP_JOINTS = ("shoulder_lift", "elbow_flex")
FLIP_HAND = {"shoulder_lift": 0.3, "elbow_flex": 0.0}   # 手の基準がこれより端に寄っていれば、手の動かせる向きは反対側（-1〜1 の尺度）
FLIP_EDGE = 0.15      # 腕の基準が、手の動かせる向きの端からこの割合（範囲に対して）より近ければ、腕はその向きに動けない
# 手を画面の端（一番下・一番上）で構えて始めると、手は片方向にしか動かせず、腕もその側にしか動かない
# （2026-11-22 実機ログ: 手を一番下で構えて肩 -56 で始めた 2 回とも、肩の目標は起動時の -56 を超えず、机側＝値の大きい側へ一度も動かなかった。
# 机側の教示は -29〜20）。腕に、動かせなくなる側の余地が LOST_ROOM 以上あれば、手が真ん中に来るまで基準を取らない。
# 手が真ん中に来ないまま FORCE_SAMPLES（約 8 秒）経ったら、そのまま取る。
CENTER_JOINT = "shoulder_lift"
CENTER_HAND = 0.6
LOST_ROOM = 0.3
FORCE_SAMPLES = MAX_SAMPLES * 3
FLIP_TRAVEL = {"shoulder_lift": 0.5, "elbow_flex": 1.0}


class Baseline:
    def __init__(self, enabled):
        self.enabled = enabled
        self.reset()

    def reset(self):
        self.ready = not self.enabled
        self.recent, self.total = [], 0
        self.hand, self.anchor, self.gaps, self.flip = {}, {}, {}, {}
        self.limits = None
        self.edge = False        # 手が端にあり、真ん中へ来るのを待っている

    def update(self, free, current, limits):
        """手の入力 free（関節 → -1〜1）と腕の姿勢 current から、基準を取る／ずれを測る。基準ができていれば True。"""
        self.limits = limits
        if not self.ready:
            self.take(free, current)
        self.gaps = {}
        if self.ready and self.enabled:
            for joint, value in free.items():
                if joint in self.hand:
                    low, high = raw(joint, limits[joint]["min"]), raw(joint, limits[joint]["max"])
                    self.gaps[joint] = dict(hand=self.hand_input(joint, value),
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
            self.flip = {j: self.flipped(j) for j in FLIP_JOINTS if j in self.hand}
            if self.off_center() and self.total < FORCE_SAMPLES:
                self.hand, self.anchor, self.flip, self.edge = {}, {}, {}, True
                return
            self.edge = False
            self.ready = True

    def off_center(self):
        """手が端にあり、その結果、腕が動かせなくなる側に余地が大きいとき True（反転する関節は、余地のある側へ動かすので対象外）。"""
        j = CENTER_JOINT
        if self.limits is None or j not in self.hand or self.flip.get(j) or abs(self.hand[j]) <= CENTER_HAND:
            return False
        low, high = raw(j, self.limits[j]["min"]), raw(j, self.limits[j]["max"])
        if high <= low:
            return False
        position = (self.anchor[j]-low) / (high-low)
        lost = 1.0-position if self.hand[j] > 0 else position     # 手が下端（＋）なら、腕は低い値側にしか動かない＝高い値側（机側）を失う
        return lost >= LOST_ROOM

    def flipped(self, joint):
        """手の動かせる向きの側に、腕の余地がほとんど無く、反対側に余地があれば True（向きを反転する）。"""
        if self.limits is None or abs(self.hand[joint]) <= FLIP_HAND[joint]:
            return False
        low, high = raw(joint, self.limits[joint]["min"]), raw(joint, self.limits[joint]["max"])
        if high <= low:
            return False
        near_low = (self.anchor[joint]-low) / (high-low)          # 腕の基準が下端からどれだけ離れているか（0〜1）
        room_hand_side = near_low if self.hand[joint] > 0 else 1.0-near_low     # 手の基準が +側なら、手は -側（下）へ動く
        return room_hand_side < FLIP_EDGE and 1.0-room_hand_side > 0.5

    def hand_input(self, joint, value):
        """手の入力を、腕へ写す入力（-1〜1。0 が腕の基準）にする。基準が無い関節は None（動かさない）。"""
        if not self.enabled:
            return value
        if joint not in self.hand:
            return None
        rel = relative(value, self.hand[joint])
        return -rel*FLIP_TRAVEL[joint] if self.flip.get(joint) else rel

    def rebase(self, joint, value, current):
        """追従の途中でONにした関節の基準を、その時点の手の入力と腕の姿勢で取り直す（基準を使うときだけ）。"""
        if self.enabled and self.ready:
            self.hand[joint] = value
            self.anchor[joint] = float(current[joint])
            if joint in FLIP_JOINTS:
                self.flip[joint] = self.flipped(joint)

    def reference(self, joint, home):
        """入力 0 に当たる腕の位置（内部の値）。基準を使わないときは初期姿勢（未登録なら None＝範囲の真ん中）。"""
        return self.anchor[joint] if self.enabled else home and home[joint]

    def state(self):
        return dict(waiting=not self.ready, edge=self.edge, tolerance=TOLERANCE, gaps=self.gaps, flip=sorted(j for j, v in self.flip.items() if v))
