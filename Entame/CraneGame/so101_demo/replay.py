"""追従ログの再生シミュレーション。人が動かなくても、撮り溜めたログの手の入力で追従の計算を何度でも確かめる。

ログ（debug/follow-*.jsonl）の各サンプルの手の入力を、実際の Desk.command へそのまま流し込み、
実機の応答（不感帯＋一次遅れ）を真似た擬似アーム SimArm を動かす。
  .venv/bin/python replay.py            # 全ログで、シミュレータが実機の動きを再現できているか確かめる
"""
import json
import math
from pathlib import Path

from server import Desk, MockArm, clamp_pose

# 実機ログ（2026-11-22、8 本）の指令と実測から当てはめた、関節ごとの不感帯（指令との差がこれ未満だと動かない。raw。
# タプルは (上げる向き, 下げる向き)）と、一次遅れの時定数（秒）。実機との差は全関節で RMS 1.5 raw 以内
DEAD = dict(shoulder_pan=0.3, shoulder_lift=(3.5, 2.0), elbow_flex=(2.0, 4.0), wrist_flex=(6.0, 4.0),
            wrist_roll=(1.0, 2.0), gripper=1.0)
TAU = 0.1


class SimArm(MockArm):
    """write は目標を置くだけ。advance(dt) で時間を進めると、不感帯を超えた分だけ一次遅れで動く。"""

    def __init__(self, pose, dead=None, tau=TAU):
        super().__init__()
        self.pose.update(pose)
        self.target = dict(self.pose)
        self.dead = dict(DEAD, **(dead or {}))
        self.tau = tau

    def write(self, pose):
        self.target.update(clamp_pose(pose, self.limits))

    def advance(self, dt):
        gain = 1 - math.exp(-dt / self.tau)
        for joint, target in self.target.items():
            error = target - self.pose[joint]
            dead = self.dead[joint]
            if isinstance(dead, tuple):                  # (上げる向き, 下げる向き)。重力のかかる関節は向きで違う
                dead = dead[0] if error > 0 else dead[1]
            if abs(error) >= dead:
                self.pose[joint] += error * gain


class Capture:
    """Desk のログの代わり。サンプルごとの want / goal / rel を受け取る。"""

    def __init__(self):
        self.samples = []

    def begin(self, **info):
        pass

    def write(self, ev, **fields):
        if ev == "sample":
            self.samples.append(fields)

    def end(self, reason):
        pass

    def close(self):
        pass


def load(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    samples = [r for r in rows if r["ev"] == "sample" and r.get("follow") is not None]
    return rows[0], samples


def replay(start, samples, limits=None, dead=None, tau=TAU, follow=None):
    """ログの入力を流し込み、サンプルごとの結果を返す。各要素: ts / real（実機の姿勢）/ sim（再生した姿勢）/ want / engaged。"""
    clock = [start["ts"]]
    arm = SimArm(start["pose"], dead, tau)
    capture = Capture()
    desk = Desk(arm, clock=lambda: clock[0], limits=limits or start["limits"], home=start.get("home"),
                log=capture, sync=True)
    token = desk.command(dict(op="start", gripper_only=True))["token"]
    out, previous = [], start["ts"]
    for sample in samples:
        arm.advance(sample["ts"] - previous)
        previous, clock[0] = sample["ts"], sample["ts"]
        before = arm.read()
        inputs = {k: v for k, v in sample["inp"].items()}
        desk.command(dict(op="sample", token=token, follow=sorted(follow or sample["follow"]), **inputs))
        logged = capture.samples[-1]
        out.append(dict(ts=sample["ts"], real=sample.get("cur"), sim=before, want=logged["want"],
                        engaged=logged["sync"]["engaged"]))
    return out


def rms_difference(results, joint):
    """シミュレータの姿勢と実機の姿勢の差（RMS）。追従が始まってからだけ比べる。"""
    pairs = [(r["sim"][joint], r["real"][joint]) for r in results if r["engaged"]]
    return math.sqrt(sum((a - b) ** 2 for a, b in pairs) / len(pairs)) if pairs else None


def tracking(results, joint, window=1.0):
    """目標(want)への追従の質。mean=平均の遅れ（raw）、stall=「大きく離れているのに、1 秒間ほぼ動かない」割合。"""
    rows = [r for r in results if r["engaged"] and joint in r["want"]]
    if not rows:
        return None
    lag = [abs(r["want"][joint] - r["sim"][joint]) for r in rows]
    stalled = 0
    for i, r in enumerate(rows):
        past = next((p for p in rows[:i] if r["ts"] - p["ts"] <= window), None)
        if past and lag[i] > 10 and abs(r["sim"][joint] - past["sim"][joint]) < 1.0:
            stalled += 1
    return dict(mean=sum(lag) / len(lag), stall=stalled / len(rows), n=len(rows))
