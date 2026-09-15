#!/usr/bin/env python3
"""**GLIM の地図座標系を、既存の map 系（floor0）へ乗せる剛体変換を求める。**

## なぜ要るのか

GLIM の事前地図は `GlobalMapping::save()` が吐く**サブマップの因子グラフ**である
（`graph.txt` ＋ `%06d/data.txt` ＋ 点群のバイナリ）。`Localization::load()` は
`SubMap::load()` でそれを読む。**既存の PCD を入れる口は無い。**
`load_ply()` は localization.hpp:69 に**宣言だけあって実体が無い**（2026-09-15 に確認）。

⇒ 事前地図は同じ記録から GLIM で作り直すしかない。そして GLIM の原点は
**記録の最初の IMU 姿勢（重力整列）**なので、既存の floor0 系とは一致しない。
だから**変換を測って `run_*.sh` で吸収する**。地図そのものは触らない。

## どうやるか

同じ記録なので、**両者の軌跡は同じ時刻に同じ物理点を指している**。
時刻で突き合わせて Umeyama（スケール無しの剛体）で解く。

  MOLA  mola_floor0/traj.txt   TUM: stamp x y z qx qy qz qw  ＝ T_map_livox
  GLIM  glim/map/traj_lidar.txt 同じ形式                      ＝ T_glimmap_livox

  求めるのは T_map_glimmap（＝ 静的 TF map -> glim_map）。

残差 RMSE も出す。これが大きいなら、どちらかの軌跡が壊れている。

  Navigation/.venv/bin/python align_glim_to_map.py \
      <glim_dump_dir> <mola_traj.txt> [--out <yaml>]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# ⚠️ **GLIM と MOLA はスキャンに打つ時刻の規約が違う。**実測で GLIM の方が
#    中央値 0.0499 s 遅い（＝ちょうど半周期。MOLA はスキャン先頭、GLIM は
#    per-point time を足した後の時刻を使う）。最近傍で突き合わせると、歩行中の
#    0.05 s ぶんの変位がそのまま残差に乗る。⇒ **MOLA 側を補間して合わせる。**
MATCH_TOL_S = 0.12          # 補間の穴（欠測）を捨てる閾値。スキャン周期 0.1 s より少し広く


def read_tum(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.split()
        if len(v) < 8:
            continue
        rows.append([float(x) for x in v[:8]])
    if not rows:
        raise ValueError(f"TUM の行が無い: {path}")
    return np.array(sorted(rows))


def match(a: np.ndarray, b: np.ndarray, tol: float) -> tuple[np.ndarray, np.ndarray]:
    """a の時刻で b を**補間**して返す。b の刻みが tol より粗い区間は捨てる。"""
    lo, hi = b[0, 0], b[-1, 0]
    a = a[(a[:, 0] >= lo) & (a[:, 0] <= hi)]
    if len(a) == 0:
        return a, a
    j = np.clip(np.searchsorted(b[:, 0], a[:, 0]), 1, len(b) - 1)
    t0, t1 = b[j - 1, 0], b[j, 0]
    keep = (t1 - t0) <= tol
    a, j, t0, t1 = a[keep], j[keep], t0[keep], t1[keep]
    if len(a) == 0:
        return a, a
    w = ((a[:, 0] - t0) / np.maximum(t1 - t0, 1e-12))[:, None]
    pos = b[j - 1, 1:4] * (1 - w) + b[j, 1:4] * w
    rots = np.empty((len(a), 4))
    for k in range(len(a)):
        key = Rotation.from_quat(np.vstack([b[j[k] - 1, 4:8], b[j[k], 4:8]]))
        rots[k] = Slerp([0.0, 1.0], key)(float(w[k, 0])).as_quat()
    return a, np.column_stack([a[:, 0], pos, rots])


def umeyama_rigid(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """dst ≈ R @ src + t を解く（スケールは 1 に固定）。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d) / len(src)
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = Vt.T @ S @ U.T
    return R, mu_d - R @ mu_s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("glim_dump", type=Path, help="GLIM の dump ディレクトリ（traj_lidar.txt がある）")
    ap.add_argument("mola_traj", type=Path, help="mola_floor0/traj.txt")
    ap.add_argument("--tol", type=float, default=MATCH_TOL_S)
    ap.add_argument("--out", type=Path, default=None, help="結果を書く .txt")
    a = ap.parse_args()

    g_path = a.glim_dump / "traj_lidar.txt"
    if not g_path.exists():
        print(f"⛔ {g_path} が無い（建図が保存まで届いていない）", file=sys.stderr)
        return 2

    g, m = read_tum(g_path), read_tum(a.mola_traj)
    gm, mm_ = match(g, m, a.tol)
    if len(gm) < 50:
        print(f"⛔ 対応が {len(gm)} 点しかない（GLIM {len(g)} / MOLA {len(m)}）", file=sys.stderr)
        return 3

    R, t = umeyama_rigid(gm[:, 1:4], mm_[:, 1:4])
    res = (gm[:, 1:4] @ R.T + t) - mm_[:, 1:4]
    d = np.linalg.norm(res, axis=1)

    # 回転の食い違いも測る（位置だけ合っていて向きが違う、を見逃さない）
    Rg = Rotation.from_quat(gm[:, 4:8]).as_matrix()
    Rm = Rotation.from_quat(mm_[:, 4:8]).as_matrix()
    ang = np.degrees(Rotation.from_matrix(np.einsum("ij,njk->nik", R, Rg).transpose(0, 2, 1)
                                          @ Rm).magnitude())

    rot = Rotation.from_matrix(R)
    q = rot.as_quat()
    rpy = rot.as_euler("xyz", degrees=True)

    print(f"=== GLIM 地図 -> 既存 map 系（floor0）の剛体変換 ===")
    print(f"  GLIM 軌跡 {len(g)} 点 / MOLA 軌跡 {len(m)} 点 / 対応 {len(gm)} 点（MOLA を補間）")
    print(f"  位置残差  RMSE {np.sqrt((d ** 2).mean()):.4f} m / 中央値 {np.median(d):.4f} m / 最大 {d.max():.4f} m")
    print(f"  回転残差  中央値 {np.median(ang):.3f} deg / 最大 {ang.max():.3f} deg")
    print()
    print(f"  T_map_glimmap  xyz  {t[0]:.6f} {t[1]:.6f} {t[2]:.6f}")
    print(f"                 quat {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}  (x y z w)")
    print(f"                 rpy  {rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}  deg (ROS Rz*Ry*Rx)")
    print()
    print("  静的 TF（run スクリプトが出す）:")
    print(f"    static_transform_publisher --x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} \\")
    print(f"        --roll {np.radians(rpy[0]):.9f} --pitch {np.radians(rpy[1]):.9f} "
          f"--yaw {np.radians(rpy[2]):.9f} \\")
    print(f"        --frame-id map --child-frame-id glim_map")

    if a.out:
        a.out.write_text(
            "# GLIM 地図 -> 既存 map 系（floor0）。align_glim_to_map.py が出した\n"
            f"# 対応 {len(gm)} 点 / 位置 RMSE {np.sqrt((d ** 2).mean()):.4f} m "
            f"/ 回転中央値 {np.median(ang):.3f} deg\n"
            f"G1_GLIM_MAP_XYZ=\"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}\"\n"
            f"G1_GLIM_MAP_RPY_DEG=\"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}\"\n"
        )
        print(f"\n  書いた: {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
