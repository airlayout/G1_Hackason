"""腕だけの空振り（2026-10-23）の解析: door_run_real.py --legs hold の記録から、手のひらが見立てのレバーへどこまで行ったかを出す。

手のひらの位置は関節の角度の順運動学（g1_arm_fk.py。シミュレータの palm と同じ定義＝手首 yaw の先 (0.0415, 0.003, 0) m）。
pelvis の床の上の姿勢は記録の pose（見立ての扉の座標・--legs hold では動かない）、傾きは IMU、床からの高さは --pelvis-height。
見立てのレバー: 軸は扉の座標で y = 幅/2 − バックセット（把手が左）、z = レバーの高さ、x = 板の面から --lever-standoff 手前。
握る棒は軸から蝶番の側へ --lever-length。手のひらと棒（線分）の距離を出す。

使い方（Mac）: Navigation/.venv/bin/python Door/real/hand_reach_dry.py <door_real_*.npz> [--png 図]
"""
from __future__ import annotations

import argparse
import ast
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_arm_fk as F  # noqa: E402

PHASES = ("WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "FREEZE", "DAMP")


def tilt_matrix_from_wxyz(q: np.ndarray) -> np.ndarray:
    """IMU の四元数（w, x, y, z）から yaw を除いた傾き（roll・pitch）の回転"""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--pelvis-height", type=float, default=0.75, help="[m] 床から pelvis（HOLD の姿勢。シミュレータは 0.74）")
    ap.add_argument("--lever-standoff", type=float, default=0.06, help="[m] 板の面から握る棒まで")
    ap.add_argument("--lever-length", type=float, default=0.151)
    ap.add_argument("--thickness", type=float, default=0.04)
    ap.add_argument("--png", type=Path)
    a = ap.parse_args()
    z = np.load(a.log)
    meta = ast.literal_eval(str(z["meta"])) if "meta" in z.files else {}
    args = meta.get("args", {})
    w, h, lh, bs = (float(v) for v in str(args.get("door", "0.95,2.18,1.00,0.05")).split(","))
    lever_left = not args.get("lever_right", False)
    side = "left" if lever_left else "right"
    sign = 1.0 if lever_left else -1.0
    axis = np.array([-a.thickness / 2 - a.lever_standoff, sign * (w / 2 - bs), lh])
    tip = axis + np.array([0.0, -sign * a.lever_length, 0.0])  # 握る棒は軸から蝶番の側へ
    chain = F.load_chain()
    ph = z["phase"].astype(int)
    door = np.nonzero(ph >= PHASES.index("DOOR"))[0]
    door = door[ph[door] <= PHASES.index("POST")]
    if not len(door):
        raise SystemExit("[reach] DOOR の周期が無い")
    t0 = z["t"][door[0]]
    rows = []
    for k in door:
        pose = z["pose"][k]
        if not np.isfinite(pose).all():
            continue
        Rt = tilt_matrix_from_wxyz(z["quat"][k])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        base = np.array([pose[0], pose[1], a.pelvis_height])
        palms = {sd: base + Rz @ Rt @ F.palm_in_pelvis(chain, z["q"][k], sd) for sd in ("left", "right")}
        rows.append((z["t"][k] - t0, PHASES[ph[k]], palms["left"], palms["right"], seg_dist(palms[side], axis, tip)))
    t = np.array([r[0] for r in rows])
    P = {"left": np.array([r[2] for r in rows]), "right": np.array([r[3] for r in rows])}
    d = np.array([r[4] for r in rows])
    v = np.r_[0.0, np.linalg.norm(np.diff(P[side], axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
    k_min = int(np.argmin(d))
    print(f"== {a.log.name}: 把手は{'左' if lever_left else '右'}・使う手 {side}・見立ての握る棒 軸 {axis.round(3).tolist()}〜先 {tip.round(3).tolist()}")
    print(f"   棒にいちばん近づいた: {d[k_min] * 100:.1f} cm（{t[k_min]:.2f} s・{rows[k_min][1]}・手のひら {P[side][k_min].round(3).tolist()}）")
    print(f"   手のひらの速さの最大 {v.max():.2f} m/s・高さの範囲 {P[side][:, 2].min():.2f}〜{P[side][:, 2].max():.2f} m")
    for tt in np.arange(0.0, t[-1] + 1e-9, 0.5):
        k = int(np.argmin(np.abs(t - tt)))
        print(f"   {t[k]:4.1f}s {rows[k][1]:4s} 手のひら（{side}）x {P[side][k,0]:+.3f} y {P[side][k,1]:+.3f} z {P[side][k,2]:.3f}・棒まで {d[k]*100:5.1f} cm")
    if a.png:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["font.family"] = "Hiragino Sans"
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        for sd, col in (("left", "#2a78d6"), ("right", "#eb6834")):
            ax[0].plot(P[sd][:, 1], P[sd][:, 0], color=col, label=f"{sd} の手のひら")
        ax[0].plot([axis[1], tip[1]], [axis[0], tip[0]], color="#111620", lw=4, label="見立ての握る棒")
        ax[0].axhline(-a.thickness / 2, color="#8b94a3", lw=1, ls="--")
        ax[0].set_xlabel("y [m]（扉を向いて左が正）")
        ax[0].set_ylabel("x [m]（扉の板の面が −厚み/2）")
        ax[0].invert_xaxis()
        ax[0].set_aspect("equal")
        ax[0].legend(frameon=False, fontsize=9)
        ax[1].plot(t, d * 100, color="#46505f")
        ax[1].set_xlabel("扉の方策を始めてからの秒")
        ax[1].set_ylabel("使う手の手のひらから棒まで [cm]")
        for x in ax:
            x.grid(color="#dce1e9")
        fig.tight_layout()
        fig.savefig(a.png, dpi=130)
        print(f"   図: {a.png}")


if __name__ == "__main__":
    main()
