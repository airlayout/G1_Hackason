"""11-10: シムの評価の記録で、腕（指先・手・肘）が開口の脇の壁にどれだけ近づくかを順運動学で数える（11-09 午前の実機の 10 章と同じ物差し）。

シムの扉（door.py）: 扉の面 x=0（板の面は −0.02）、開口の縁は y=±幅/2、脇の壁（left_frame/right_frame・当たりの形あり）の面は x=−0.07。
「壁の前 3 cm 以内」= その点が開口の縁より外（|y| > 幅/2）で、x > −0.07 − 0.03。めり込みも含む。
腕の位置は学習の USD と同じ URDF（g1_arm_fk.py の ArmFK）で関節の角度から出し、記録の pelvis の位置と向き（四元数）で扉の座標へ置く。
数えるのは段 1（構える）から完了・打ち切りまでの最初の 1 回。把手は全部左（y>0）なので、レバーの側の壁は左。

使い方: python ana_arm_wall_sim_1110.py <記録の置き場> ...（count_success.py と同じ states_*.npz・static.npz）
"""
from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_arm_fk as fk  # noqa: E402

WALL_X = -0.07  # 脇の壁の面（door.py: 中心 −0.02・半分の厚さ 0.05）
NEAR = 0.03  # 「3 cm 以内」
POINTS = ("L指先", "L手", "L肘", "R指先", "R肘")


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def analyse(d: Path, arm: fk.ArmFK) -> None:
    s = np.load(d / "static.npz", allow_pickle=True)
    names = [str(n) for n in s["dof_names"]]
    order = [names.index(n) for n in fk.JOINT_ORDER]  # 記録の並び → SDK の並び
    fs = sorted(glob.glob(str(d / "states_*.npz")))
    keys = ("root_pos", "root_quat_xyzw", "dof_pos", "stage", "episode_length", "door_dof_pos")
    Z = {k: np.concatenate([np.load(f)[k] for f in fs]) for k in keys}
    T, N = Z["stage"].shape
    hw = s["door_width"][:N] / 2.0
    frac = {p: [] for p in POINTS}
    deep = {p: [] for p in POINTS}
    ok = []
    for e in range(N):
        ep = Z["episode_length"][:, e]
        end = np.nonzero(np.diff(ep) < 0)[0]
        L = int(end[0] + 1) if len(end) else T
        ok.append(np.degrees(Z["door_dof_pos"][:L, e, 0]).max() >= 3.0)
        near = {p: 0 for p in POINTS}
        dmax = {p: -1.0 for p in POINTS}
        for i in range(0, L, 2):  # 0.04 s おき
            pts = arm.points(Z["dof_pos"][i, e][order])
            R = quat_xyzw_to_matrix(Z["root_quat_xyzw"][i, e])
            for p in POINTS:
                w = Z["root_pos"][i, e] + R @ pts[p]
                outside = abs(w[1]) > hw[e]  # 開口の縁より外＝壁の前
                if outside and w[0] > WALL_X - NEAR:
                    near[p] += 1
                    dmax[p] = max(dmax[p], w[0] - WALL_X)  # 正ならめり込み
        n = len(range(0, L, 2))
        for p in POINTS:
            frac[p].append(near[p] / n)
            deep[p].append(dmax[p])
    ok = np.array(ok)
    print(f"{d.name}: 回 {N}・扉 3° の成功 {ok.sum()}/{N}")
    for p in POINTS:
        f = np.array(frac[p])
        dd = np.array(deep[p])
        hit = f > 0
        print(f"  {p}: 壁の前 3 cm 以内にいた回 {hit.sum():3d}/{N}（成功 {hit[ok].sum()}・失敗 {hit[~ok].sum()}）"
              f"・その回の時間の割合 中央 {np.median(f[hit]) if hit.any() else 0:.2f}"
              f"・めり込んだ回 {(dd > 0).sum()}")


def main() -> None:
    arm = fk.ArmFK(fk.default_urdf())
    for a in sys.argv[1:]:
        analyse(Path(a), arm)


if __name__ == "__main__":
    main()
