"""推定の軌跡を正解と比べる（合成の部屋の試し用）。

    python tests/eval_traj.py <推定の traj.txt> <正解 room_gt.txt>

推定の時刻は OMEN に届いた壁の時刻、正解は映像の中の秒なので、時刻の起点のずれを探して組にする。
出す物:
  scale_sim3   縮尺を自由にして合わせたときの倍率（1.0 なら実寸が合っている）
  ate_se3_m    縮尺を 1 に固定して合わせたときの位置の誤差（RMSE・メートル）
  path_ratio   推定の道のりと正解の道のりの比
"""

from __future__ import annotations

import json
import sys

import numpy as np


def load(path: str) -> np.ndarray:
    return np.atleast_2d(np.loadtxt(path))


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool) -> tuple[float, np.ndarray, np.ndarray]:
    """dst ≈ s R src + t を最小二乗で"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / xs.var(0).sum()) if with_scale else 1.0
    return s, R, mu_d - s * R @ mu_s


def associate(est: np.ndarray, gt: np.ndarray, offset: float, max_dt: float = 0.02):
    te = est[:, 0] - est[0, 0] + offset
    idx = np.searchsorted(gt[:, 0], te).clip(1, len(gt) - 1)
    near = np.where(np.abs(gt[idx - 1, 0] - te) < np.abs(gt[idx, 0] - te), idx - 1, idx)
    ok = np.abs(gt[near, 0] - te) < max_dt
    return est[ok, 1:4], gt[near[ok], 1:4]


def main() -> None:
    est, gt = load(sys.argv[1]), load(sys.argv[2])
    best = None
    for off in np.arange(0.0, 6.0, 1 / 60):
        e, g = associate(est, gt, off)
        if len(e) < max(10, len(est) // 2):
            continue
        s, R, t = umeyama(e, g, with_scale=False)
        err = float(np.sqrt(((e @ R.T + t - g) ** 2).sum(1).mean()))
        if best is None or err < best[0]:
            best = (err, off)
    if best is None:
        raise SystemExit("組にできる時刻が見つからない")
    e, g = associate(est, gt, best[1])
    s_sim3, _, _ = umeyama(e, g, with_scale=True)
    path = lambda x: float(np.linalg.norm(np.diff(x, axis=0), axis=1).sum())  # noqa: E731
    print(json.dumps({
        "n_matched": int(len(e)),
        "time_offset_s": round(best[1], 3),
        "scale_sim3": round(s_sim3, 4),
        "ate_se3_m": round(best[0], 4),
        "path_est_m": round(path(e), 3),
        "path_gt_m": round(path(g), 3),
        "path_ratio": round(path(e) / max(path(g), 1e-9), 4),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
