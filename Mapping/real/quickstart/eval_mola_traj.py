#!/usr/bin/env python3
"""MOLA-LO の軌跡（TUM 形式）を真値と突き合わせ、計画書 §4 の合否を出す。

    Navigation/.venv/bin/python eval_mola_traj.py \
        runs/<SESSION>/mola/traj.txt runs/<SESSION>/benchmark_s5/poses.txt

真値 = benchmark_s5/poses.txt（scan-to-map ICP・livox_frame -> その記録自身の地図）
MOLA = 初期姿勢が単位行列の自前座標系なので、**定数の SE(3)** で真値座標系に合わせてから比べる。
    位置: Umeyama（スケール無し）で 1 つの剛体変換を最小二乗当てはめ
    姿勢: A = mean_i( R_true[i] @ R_mola[i]^T )（回転平均）
合わせるのは 1 回だけ。区間ごとに取り直すと誤差を吸収してしまい判定にならない。

⚠️ 真値の限界: 真値はその記録自身の地図に対する ICP なので、
「地図が実際の部屋と合っているか」は検証していない（計画書 §6）。
ここで測れるのは「内蔵 SLAM の地図に対する整合」まで。
"""
import math
import sys
import warnings
from pathlib import Path

import numpy as np

# numpy 2.2.6 + Apple Accelerate BLAS は正常な入力でも matmul で
# divide-by-zero / overflow / invalid を投げる（乱数でも再現する空振り）。
# 非有限値の混入は下の assert で別途見ているので、ここでは黙らせる。
warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
from scipy.spatial.transform import Rotation

WALK_SPEED_MS = 0.05  # これ以上を歩行中とみなす（measure3.py と同じ）
PASS = {
    "att_median_deg": 2.0,
    "att_p95_deg": 5.0,
    "pos_median_m": 0.05,
    "rate_hz": 5.0,
    "point_5m_p95_m": 0.5,
}
RIVAL = {"att_median_deg": 1.90, "att_p95_deg": 4.33, "point_5m_p95_m": 0.377}


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = [l.split() for l in path.read_text().splitlines() if l.strip() and not l.startswith("#")]
    a = np.array([[float(c) for c in r[:8]] for r in rows])
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def umeyama_rigid(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """スケール無しの最小二乗剛体変換 dst ≈ R @ src + t."""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def geodesic_deg(D: np.ndarray) -> float:
    return math.degrees(np.arccos(np.clip((np.trace(D) - 1) / 2, -1, 1)))


def stats(label: str, e: np.ndarray) -> None:
    if len(e) < 10:
        print(f"  {label:<8} n={len(e)} — 少なすぎるので出さない")
        return
    print(
        f"  {label:<8} n={len(e):5d}  中央値 {np.median(e[:,0]):6.2f}°  p95 {np.percentile(e[:,0],95):6.2f}°  最大 {e[:,0].max():6.2f}°"
        f"   |roll| p95 {np.percentile(abs(e[:,1]),95):5.2f}°"
        f"  |pitch| p95 {np.percentile(abs(e[:,2]),95):5.2f}°"
        f"  |yaw| p95 {np.percentile(abs(e[:,3]),95):5.2f}°"
    )


def main() -> int:
    traj_p, poses_p = Path(sys.argv[1]), Path(sys.argv[2])
    tm, pm, qm = load_tum(traj_p)
    tt, pt, qt = load_tum(poses_p)
    for name, arr in (("MOLA", np.c_[pm, qm]), ("真値", np.c_[pt, qt])):
        assert np.isfinite(arr).all(), f"{name} に非有限値がある"

    print(f"MOLA 軌跡 : {traj_p}  {len(tm)} 姿勢  {tm[0]:.3f} 〜 {tm[-1]:.3f}（{tm[-1]-tm[0]:.1f} 秒）")
    print(f"真値      : {poses_p}  {len(tt)} 姿勢  {tt[0]:.3f} 〜 {tt[-1]:.3f}（{tt[-1]-tt[0]:.1f} 秒）")

    # レート（MOLA が何 Hz で姿勢を出したか）
    dt = np.diff(tm)
    rate = 1.0 / np.median(dt)
    print(f"\n=== レート ===\n  中央値 {rate:.2f} Hz（間隔 中央値 {np.median(dt)*1000:.1f} ms / 最大 {dt.max()*1000:.0f} ms）")

    # 真値の各時刻に最も近い MOLA 姿勢を取る（外挿しない）
    lo, hi = max(tm[0], tt[0]), min(tm[-1], tt[-1])
    keep = (tt >= lo) & (tt <= hi)
    if keep.sum() < 10:
        print("\n重なる区間が無い。MOLA が途中で止まっている可能性がある")
        return 1
    tt, pt, qt = tt[keep], pt[keep], qt[keep]
    idx = np.abs(tm[None, :] - tt[:, None]).argmin(axis=1)
    gap = np.abs(tm[idx] - tt)
    ok = gap < 0.06  # LiDAR 10 Hz の半周期
    print(f"  真値と対応が取れた点: {ok.sum()}/{len(tt)}（時刻差 中央値 {np.median(gap)*1000:.1f} ms）")
    tt, pt, qt, idx = tt[ok], pt[ok], qt[ok], idx[ok]
    pm_s, qm_s = pm[idx], qm[idx]

    R_true = Rotation.from_quat(qt).as_matrix()
    R_mola = Rotation.from_quat(qm_s).as_matrix()

    # 歩行判定は真値の速度から（MOLA の推定に依存させない）
    v = np.r_[0, np.linalg.norm(np.diff(pt[:, :2], axis=0), axis=1) / np.maximum(np.diff(tt), 1e-3)]
    walking = v > WALK_SPEED_MS
    print(f"  歩行率 {100*walking.mean():.0f} %（静止 {(~walking).sum()} / 歩行 {walking.sum()}）")

    # --- 定数 SE(3) で 1 回だけ合わせる ---
    Rp, tp = umeyama_rigid(pm_s, pt)
    A = Rotation.from_matrix(np.array([R_true[i] @ R_mola[i].T for i in range(len(tt))])).mean().as_matrix()
    print("\n=== 真値座標系への合わせ（全区間で 1 つ）===")
    print(f"  位置合わせ  R rpy[deg] = {np.round(Rotation.from_matrix(Rp).as_euler('xyz', degrees=True), 2)}  t = {np.round(tp,3)}")
    print(f"  姿勢合わせ  A rpy[deg] = {np.round(Rotation.from_matrix(A).as_euler('xyz', degrees=True), 2)}")

    # --- 姿勢誤差 ---
    err = []
    for i in range(len(tt)):
        D = R_true[i] @ (A @ R_mola[i]).T
        r, p_, y = Rotation.from_matrix(D).as_euler("xyz", degrees=True)
        err.append((geodesic_deg(D), r, p_, y))
    err = np.array(err)

    print("\n=== 姿勢誤差 ===")
    stats("全区間", err)
    stats("静止中", err[~walking])
    stats("歩行中", err[walking])

    # --- 位置誤差 ---
    pos_err = np.linalg.norm((Rp @ pm_s.T).T + tp - pt, axis=1)
    print("\n=== 位置誤差 ===")
    print(
        f"  中央値 {np.median(pos_err)*100:.1f} cm  p95 {np.percentile(pos_err,95)*100:.1f} cm  最大 {pos_err.max()*100:.1f} cm"
    )

    # --- 5 m 先の点がどれだけ動くか ---
    def chord(deg: float, r: float = 5.0) -> float:
        return 2 * r * math.sin(math.radians(deg) / 2)

    p95_walk = np.percentile(err[walking][:, 0], 95) if walking.sum() >= 10 else np.percentile(err[:, 0], 95)
    print("\n=== 5 m 先の点のずれ（2·r·sin(θ/2)）===")
    print(f"  歩行中 p95 {p95_walk:.2f}° -> {chord(p95_walk):.3f} m")

    # --- 合否 ---
    m_all = err[walking] if walking.sum() >= 10 else err
    got = {
        "att_median_deg": float(np.median(m_all[:, 0])),
        "att_p95_deg": float(np.percentile(m_all[:, 0], 95)),
        "pos_median_m": float(np.median(pos_err)),
        "rate_hz": float(rate),
        "point_5m_p95_m": float(chord(p95_walk)),
    }
    lower_is_better = {k: k != "rate_hz" for k in PASS}
    print("\n=== 合否（計画書 §4。基準は測る前に決めたもの）===")
    all_ok = True
    for k, thr in PASS.items():
        v_ = got[k]
        ok_ = v_ <= thr if lower_is_better[k] else v_ >= thr
        all_ok &= ok_
        rival = RIVAL.get(k)
        rmark = ""
        if rival is not None:
            rmark = f"   対抗馬(odom yaw+IMU) {rival:.2f} -> {'MOLA が勝ち' if v_ <= rival else 'MOLA が負け'}"
        print(f"  [{'PASS' if ok_ else 'FAIL'}] {k:<16} 実測 {v_:8.3f}  基準 {'<=' if lower_is_better[k] else '>='} {thr}{rmark}")
    print(f"\n総合: {'PASS' if all_ok else 'FAIL'}")
    return 0 if all_ok else 2


if __name__ == "__main__":
    sys.exit(main())
