"""11-10（案 1・レバーを下げたかの判定）: 順運動学の左手の位置だけで「レバーを下げた」を当てられるかを、シムの評価の記録で測る。

判定（実機でも測れる量だけ）: 左の手のひら（rubber_hand・順運動学）が、閉じたレバーの当てる点（回の始めの値＝実機は止まって深度で測る）の
水平 R 以内にあり、かつ当てる点の高さより DROP 以上低い。正解はシムの真のレバーの角度 30°（今の脚を放す線・掛け金が縁まで引っ込む）。
数えるもの: 正解が立つ回のうち判定も立つ割合（取りこぼし）・判定が立った時のレバーの角度（早すぎ）・正解より何秒早い／遅いか。
手とレバーの力（記録の hand_lever_force）も、判定が立った時の大きさを見る（実機ではトルクから出す量の見当）。

使い方: python ana_lever_detect_1110.py <記録の置き場> [R=0.08] [DROP=0.04] [FORCE=0（N・手とレバーの力の下限）] [HOLD=1（歩・続けて立つ数）]
"""
from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_arm_fk as fk  # noqa: E402
from ana_arm_wall_sim_1110 import quat_xyzw_to_matrix  # noqa: E402

TRUE_DEG = 30.0
FPS = 50


def main() -> None:
    d = Path(sys.argv[1])
    R = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08
    DROP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.04
    FORCE = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    HOLD = int(sys.argv[5]) if len(sys.argv) > 5 else 1
    arm = fk.ArmFK(fk.default_urdf())
    s = np.load(d / "static.npz", allow_pickle=True)
    names = [str(n) for n in s["dof_names"]]
    order = [names.index(n) for n in fk.JOINT_ORDER]
    fs = sorted(glob.glob(str(d / "states_*.npz")))
    keys = ("root_pos", "root_quat_xyzw", "dof_pos", "episode_length", "door_dof_pos", "grasp_target", "hand_lever_force")
    Z = {k: np.concatenate([np.load(f)[k] for f in fs]) for k in keys}
    T, N = Z["episode_length"].shape
    hit_true, hit_det, lead, lever_at_det, force_at_det, false_early = 0, 0, [], [], [], 0
    for e in range(N):
        ep = Z["episode_length"][:, e]
        end = np.nonzero(np.diff(ep) < 0)[0]
        L = int(end[0] + 1) if len(end) else T
        lever = np.degrees(Z["door_dof_pos"][:L, e, 1])
        ref = Z["grasp_target"][0, e]  # 閉じたレバーの当てる点（回の始め）
        det = np.zeros(L, dtype=bool)
        for i in range(L):
            p = Z["root_pos"][i, e] + quat_xyzw_to_matrix(Z["root_quat_xyzw"][i, e]) @ arm.points(Z["dof_pos"][i, e][order])["L手"]
            det[i] = (np.hypot(p[0] - ref[0], p[1] - ref[1]) <= R and p[2] <= ref[2] - DROP
                      and np.linalg.norm(Z["hand_lever_force"][i, e]) >= FORCE)
        if HOLD > 1:  # HOLD 歩つづけて立ったら（その最後の歩で立てる）
            run = np.convolve(det.astype(int), np.ones(HOLD, dtype=int), mode="full")[:L]
            det = run >= HOLD
        t_true = np.argmax(lever >= TRUE_DEG) if (lever >= TRUE_DEG).any() else None
        t_det = np.argmax(det) if det.any() else None
        hit_true += t_true is not None
        if t_det is not None:
            hit_det += 1
            lever_at_det.append(lever[t_det])
            force_at_det.append(float(np.linalg.norm(Z["hand_lever_force"][t_det, e])))
            if lever[t_det] < 10.0:
                false_early += 1
        if t_true is not None and t_det is not None:
            lead.append((t_true - t_det) / FPS)
    lead, lad = np.array(lead), np.array(lever_at_det)
    q = lambda a: f"中央 {np.median(a):.2f} [10% {np.percentile(a, 10):.2f}・90% {np.percentile(a, 90):.2f}]" if len(a) else "—"
    print(f"{d.name}  R={R} DROP={DROP} FORCE={FORCE} HOLD={HOLD}: 回 {N}・正解（レバー {TRUE_DEG:.0f}°）が立った回 {hit_true}・判定が立った回 {hit_det}")
    print(f"  両方立った回 {len(lead)}・判定が正解より早い秒 {q(lead)}（負は遅い）")
    print(f"  判定が立った時のレバーの角度 {q(lad)}・10° 未満で立った（早すぎ）{false_early} 回")
    print(f"  判定が立った時の手とレバーの力 [N] {q(np.array(force_at_det))}")


if __name__ == "__main__":
    main()
