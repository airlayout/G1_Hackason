"""接近中の関節指令を保持し、同じ経路を逆向きに再生する。"""

import math

import numpy as np

from trajectory import validate_arm_q


class ApproachPath:
    def __init__(self, start):
        self.samples = [(0.0, validate_arm_q(np.asarray(start).tolist()))]
        self.frozen = False

    def append(self, target, dt):
        if self.frozen:
            raise ValueError("押下開始後は接近経路を更新できません")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("接近経路の時間間隔が不正です")
        self.samples.append((self.samples[-1][0]+dt,
                             validate_arm_q(np.asarray(target).tolist())))

    def freeze(self):
        if len(self.samples) < 2:
            raise ValueError("接近経路が記録されていません")
        self.frozen = True

    def reversed_targets(self, hz=50.0, speed_limit=0.8):
        if not self.frozen or len(self.samples) < 2:
            raise ValueError("逆再生できる接近経路がありません")
        if not np.all(np.isfinite([hz, speed_limit])) or min(hz, speed_limit) <= 0:
            raise ValueError("逆再生の周期または速度上限が不正です")
        times = np.array([sample[0] for sample in self.samples])
        poses = np.array([validate_arm_q(list(sample[1])) for sample in self.samples])
        intervals = np.diff(times)
        if not np.all(np.isfinite(times)) or np.any(intervals <= 0):
            raise ValueError("接近経路の時刻が不正です")
        scale = max(1.0, float(np.max(np.abs(np.diff(poses, axis=0))/intervals[:, None]))/speed_limit)
        result = [tuple(poses[-1])]
        for i in range(len(poses)-1, 0, -1):
            # ceilで時間を延ばし、記録時より速い再生と速度超過を防ぐ。
            steps = max(1, math.ceil(intervals[i-1]*scale*hz-1e-9))
            for step in range(1, steps+1):
                ratio = step/steps
                result.append(tuple((1-ratio)*poses[i]+ratio*poses[i-1]))
        return result


def validate_return_path(kin, targets, face, normal):
    """実測姿勢からの接続と、隣接する全区間を検査し、運動学状態を復元する。"""
    original = kin.data.qpos.copy()
    try:
        for goal in targets:
            kin.validate_path(np.asarray(goal), face, normal)
            qpos = kin.data.qpos.copy()
            qpos[kin.qadr] = goal
            kin.update(qpos)
    finally:
        kin.update(original)
