"""段 0（2026-10-16）: 頭のライダーの記録から、扉の開いた角度を 1 回の測定（10 Hz）ごとに出す。機体は止まっている前提。

入力は crop_lidar_bag.py（PC2）が前方だけを切り出した npz（xyz は mm の int16、ライダーの座標のまま）。

1. 上下逆さまの取付を戻し（x 軸まわり 180°）、床に平面を当てて床を z = 0・水平にする（全部の回を重ねて 1 回だけ）
2. 前方の縦の平面（壁と閉じた扉）を当てる。壁に沿う向き u（機体の左が正）と、壁からの距離 v（機体の側が正）を取る
3. 扉は内開き（機体から遠ざかる）で、蝶番は機体から見て右（u = −幅/2）。蝶番のまわりで、壁の面から奥にある点を
   半径 0.2〜0.85 m・床から 0.2〜1.25 m で拾う（把手の側の枠 0.94 m を入れない）
4. 点の角度 φ = atan2(−v, u − u_蝶番) の山（2° 刻み）を取り、山の ±6° の点で 2 次元の直線（主成分）を当てて扉の角度とする
   点が MIN_POINTS に満たない回は「測れない」

使い方: Navigation/.venv/bin/python Door/real/door_lidar_angle.py <crop.npz> <出力先> [--name 名前]
出力: <名前>_angle.npz（時刻・角度・点の数・測れたか）・<名前>_angle.png・<名前>_angle.json（要約）
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

DOOR_W = 0.94  # [m] 扉の幅（10-06 の巻き尺）
R_MIN, R_MAX = 0.20, 0.85  # [m] 蝶番からの半径（把手の側の枠を入れない）
Z_MIN, Z_MAX = 0.20, 1.25  # [m] 床から（ライダーは 1.28 m より上を見ない）
MIN_POINTS = 30
BIN_DEG = 2.0
PEAK_HALF_DEG = 6.0


def load_scans(path: Path) -> tuple[list[np.ndarray], np.ndarray]:
    z = np.load(path)
    xyz = z["xyz_mm"].astype(np.float64) / 1000.0
    edges = np.concatenate([[0], np.cumsum(z["counts"])])
    scans = [xyz[edges[i]:edges[i + 1]] for i in range(len(z["counts"]))]
    return scans, z["stamp_ns"].astype(np.float64) / 1e9


def rot_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """単位ベクトル a を b に重ねる最小の回転"""
    v, c = np.cross(a, b), float(np.dot(a, b))
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def ransac_plane(p: np.ndarray, tol: float, iters: int = 400, seed: int = 0) -> tuple[np.ndarray, float]:
    """平面 n·x + d = 0（|n| = 1）を RANSAC で当て、内側の点で最小二乗に直す"""
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((p - a) @ n) < tol
        if inl.sum() > best_n:
            best, best_n = inl, inl.sum()
    q = p[best]
    centroid = q.mean(0)
    n = np.linalg.svd(q - centroid, full_matrices=False)[2][-1]
    return n, -float(n @ centroid)


def level(scans: list[np.ndarray]) -> tuple[np.ndarray, float]:
    """取付の上下逆さまを戻し、床を z = 0 の水平にする回転と高さ（全部の回を重ねる）"""
    flip = np.diag([1.0, -1.0, -1.0])
    allp = np.concatenate(scans[::5]) @ flip.T
    low = allp[allp[:, 2] < -0.6]
    n, d = ransac_plane(low[::max(1, len(low) // 60000)], 0.02)
    if n[2] < 0:
        n, d = -n, -d
    return rot_between(n, np.array([0.0, 0.0, 1.0])) @ flip, d


def fit_wall(p: np.ndarray) -> tuple[np.ndarray, float]:
    """前方の縦の平面。水平の法線 n（機体の方を向く）と n·x = c の c（機体から壁までは −c）"""
    m = (p[:, 0] > 0.4) & (p[:, 0] < 2.5) & (np.abs(p[:, 1]) < 2.0) & (p[:, 2] > 0.3) & (p[:, 2] < 1.25)
    q = p[m][::max(1, m.sum() // 60000)]
    n3, d = ransac_plane(q, 0.03)
    n = np.array([n3[0], n3[1], 0.0])
    n /= np.linalg.norm(n)
    if n[0] > 0:
        n = -n
    on = np.abs(q @ n3 + d) < 0.03
    return n, float(np.median(q[on] @ n))


def door_angle(u: np.ndarray, v: np.ndarray, u_hinge: float) -> tuple[float, int]:
    """蝶番のまわりの点から扉の角度 [deg] と使った点の数。測れなければ (nan, 点の数)"""
    du, dv = u - u_hinge, -v  # dv: 壁から奥（部屋の中）が正
    r = np.hypot(du, dv)
    phi = np.degrees(np.arctan2(dv, du))
    m = (r > R_MIN) & (r < R_MAX) & (phi > -8.0) & (phi < 115.0) & (dv > -0.03)
    if m.sum() < MIN_POINTS:
        return float("nan"), int(m.sum())
    hist, edges = np.histogram(phi[m], bins=np.arange(-8.0, 116.0, BIN_DEG))
    peak = edges[hist.argmax()] + BIN_DEG / 2
    k = m & (np.abs(phi - peak) < PEAK_HALF_DEG)
    if k.sum() < MIN_POINTS:
        return float("nan"), int(k.sum())
    pts = np.stack([du[k], dv[k]], 1)
    d = np.linalg.svd(pts - pts.mean(0), full_matrices=False)[2][0]
    if d[0] < 0:
        d = -d
    return math.degrees(math.atan2(d[1], d[0])), int(k.sum())


def analyze(path: Path, out: Path, name: str) -> dict:
    scans, t = load_scans(path)
    R, h = level(scans)
    lev = [s @ R.T + np.array([0.0, 0.0, h]) for s in scans]
    n, c = fit_wall(np.concatenate(lev[::3]))
    tvec = np.array([-n[1], n[0], 0.0])
    if tvec[1] < 0:
        tvec = -tvec
    u_hinge = -DOOR_W / 2  # 開口の真ん中に立っている前提（10-16 は巻き尺で「ほぼ真ん中」）
    ang, cnt = [], []
    for q in lev:
        band = (q[:, 2] > Z_MIN) & (q[:, 2] < Z_MAX)
        a, k = door_angle(q[band] @ tvec, q[band] @ n - c, u_hinge)
        ang.append(a)
        cnt.append(k)
    ang, cnt = np.array(ang), np.array(cnt)
    t = t - t[0]
    ok = np.isfinite(ang)
    summary = {
        "name": name, "scans": len(scans), "seconds": round(float(t[-1]), 1),
        "lidar_height_m": round(h, 3), "wall_distance_m": round(-c, 3),
        "facing_deg": round(math.degrees(math.atan2(-n[1], -n[0])), 2),
        "measured_ratio": round(float(ok.mean()), 3),
        "angle_max_deg": round(float(np.nanmax(ang)), 1) if ok.any() else None,
    }
    np.savez_compressed(out / f"{name}_angle.npz", t=t, angle_deg=ang, points=cnt, ok=ok)
    (out / f"{name}_angle.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    plot(t, ang, cnt, out / f"{name}_angle.png", name)
    return summary


def plot(t: np.ndarray, ang: np.ndarray, cnt: np.ndarray, path: Path, name: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.family"] = "Hiragino Sans"
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ok = np.isfinite(ang)
    axes[0].plot(t[ok], ang[ok], ".", ms=3, color="#1f6feb")
    axes[0].plot(t[~ok], np.zeros((~ok).sum()) - 5, "|", ms=8, color="#d1242f", label="測れない回")
    for y, lab in ((70, "通り抜けの角度 70°"), (75, "計器の模型の上限 75°"), (90, "止まる角度 約 90°")):
        axes[0].axhline(y, color="0.6", lw=0.8, ls="--")
        axes[0].text(t[-1], y, " " + lab, va="center", fontsize=8, color="0.4")
    axes[0].set_ylabel("扉の角度 [°]")
    axes[0].set_ylim(-10, 115)
    axes[0].legend(loc="upper left")
    axes[0].set_title(f"頭のライダーで測った扉の角度（{name}・1 回の測定ごと）")
    axes[1].plot(t, cnt, "-", lw=0.8, color="0.3")
    axes[1].axhline(MIN_POINTS, color="#d1242f", lw=0.8, ls="--")
    axes[1].set_ylabel("扉の点の数")
    axes[1].set_xlabel("記録の始めからの時間 [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("crop", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--name", default="")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    print(json.dumps(analyze(a.crop, a.out, a.name or a.crop.stem), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
