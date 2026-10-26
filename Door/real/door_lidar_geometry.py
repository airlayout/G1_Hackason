"""頭のライダーで扉の寸法（幅・レバーの高さ・バックセット）を測る。機体は止まっていて、扉は閉じている前提（10-21）。

なぜ: 生徒は扉の寸法を観測で受け取り、要る精度は約 1 cm（実験 B: 誤差 σ 2 cm で −11 pt）。画像（V2）からは読めなかった（実験 A）。
巻き尺の代わりにライダーで測れれば、測っていない扉にも使える。

door_lidar_angle.py と同じ前処理（上下逆さまを戻し床で水平に）→ door_lidar_frame.reference_wall で壁に沿う向き u（左が正）と
壁からの距離 v を取る → 真ん中の点で扉の板の面を当て、板からの出っ張り w（機体の側が正）にする。
  幅        高さ 0.3〜0.95 m の点で、u ごと（5 mm）の w の中央値を真ん中から外へたどり、柱の段差の半分の高さを越える所を左右で探す（線形補間）。その差
  レバーの高さ 板から 4〜12 cm 手前・高さ 0.8〜1.25 m の点（レバーの握る棒）の z の中央値
  バックセット 把手の側の扉の縁 − 棒の縁の側の端（棒の点の u の 98 パーセンタイル）。
              ⚠️ 棒の端から縁の側へ斜めに伸びる点は、ライダーの光がレバーの端をかすめてできる「にじみ」なので、出っ張り 4 cm 以上だけを使う
              ⚠️ 軸（回る中心）は棒の端より首の太さぶん内側にある（10-16 の扉で約 1 cm）。NECK_M で足す

使い方: Navigation/.venv/bin/python Door/real/door_lidar_geometry.py <crop.npz> [--lever-side left|right] [--angle <角度の npz> --max-angle 2] [--window 100]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_angle as dla  # noqa: E402
import door_lidar_frame as dlf  # noqa: E402

warnings.filterwarnings("ignore", category=RuntimeWarning)  # mac の BLAS が matmul で出す見かけだけの警告

BIN_M = 0.005
EDGE_Z = (0.3, 0.95)  # [m] 縁を探す高さ（今日のように頭が下を向くと 0.95 m より上は写らない）
LEVER_W = (0.04, 0.12)  # [m] 板からの出っ張り（握る棒）
LEVER_Z = (0.80, 1.25)  # [m]
NECK_M = 0.01  # [m] 棒の端から軸までの首の太さ（10-16 の扉の形から。扉ごとに違いうる）
MIN_LEVER_POINTS = 200


def to_door_coords(scans: list[np.ndarray], ref_scans: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """点を (u, w, z) に。w は扉の板の面からの出っ張り。返り値の最後は板の面の傾き [°]"""
    R, h = dla.level(ref_scans)
    lev_ref = [s @ R.T + np.array([0.0, 0.0, h]) for s in ref_scans[:60]]
    n0, c0 = dlf.reference_wall(lev_ref)
    t0 = np.array([-n0[1], n0[0]])
    if t0[1] < 0:
        t0 = -t0
    q = np.concatenate(scans) @ R.T + np.array([0.0, 0.0, h])
    v, u, z = q[:, :2] @ n0 - c0, q[:, :2] @ t0, q[:, 2]
    m = (np.abs(u) < 0.3) & (z > 0.4) & (z < 1.3) & (np.abs(v) < 0.04)
    coef = np.linalg.lstsq(np.c_[u[m], np.ones(m.sum())], v[m], rcond=None)[0]
    return u, v - (coef[0] * u + coef[1]), z, math.degrees(math.atan(coef[0]))


def find_edge(u: np.ndarray, w: np.ndarray, z: np.ndarray, side: int) -> tuple[float, float]:
    """真ん中から side（+1 左・−1 右）へ、柱の段差の半分の高さを越える所 [m] と、段差の高さ [m]"""
    band = (z > EDGE_Z[0]) & (z < EDGE_Z[1]) & (w > -0.05) & (w < 0.10)
    ub, wb = u[band], w[band]
    edges = np.arange(0.0, 0.9, BIN_M) * side
    mids, meds = [], []
    for a in edges:
        lo, hi = (a, a + BIN_M) if side > 0 else (a - BIN_M, a)
        k = (ub >= lo) & (ub < hi)
        if k.sum() >= 20:
            mids.append((lo + hi) / 2)
            meds.append(float(np.median(wb[k])))
    mids, meds = np.array(mids), np.array(meds)
    first = np.nonzero(meds > 0.008)[0]  # 板から 8 mm 出たら柱の段差に入った
    if not len(first):
        return float("nan"), float("nan")
    i0 = first[0]
    plateau = float(np.median(meds[i0 + 2:i0 + 10])) if i0 + 10 <= len(meds) else float(meds[i0:].max())
    half = plateau / 2
    j = np.nonzero(meds[:i0 + 3] >= half)[0]
    j = j[0] if len(j) else i0
    if j == 0:
        return float(mids[0]), plateau
    # half を跨ぐ 2 点で線形補間
    m0, m1, u0_, u1_ = meds[j - 1], meds[j], mids[j - 1], mids[j]
    t = (half - m0) / (m1 - m0) if m1 != m0 else 0.5
    return float(u0_ + t * (u1_ - u0_)), plateau


def measure(u: np.ndarray, w: np.ndarray, z: np.ndarray, lever_side: int) -> dict:
    left, p_left = find_edge(u, w, z, +1)
    right, p_right = find_edge(u, w, z, -1)
    out = {"width": left - right, "edge_left": left, "edge_right": right, "jamb_left_m": p_left, "jamb_right_m": p_right,
           "z_max_seen": float(np.percentile(z[(u > right) & (u < left)], 99.5)) if np.isfinite(left - right) else float("nan")}
    latch = left if lever_side > 0 else right
    L = (w > LEVER_W[0]) & (w < LEVER_W[1]) & (z > LEVER_Z[0]) & (z < LEVER_Z[1]) & (u > right + 0.01) & (u < left - 0.01)
    out["lever_points"] = int(L.sum())
    if L.sum() < MIN_LEVER_POINTS:
        out.update(lever_h=float("nan"), backset=float("nan"), lever_end=float("nan"))
        return out
    # 棒は 1 本の水平な塊。高さの中央値の ±3 cm に絞ってから端を取る
    zc = float(np.median(z[L]))
    L &= np.abs(z - zc) < 0.03
    # 端の「にじみ」（光が棒の端をかすめて奥へ抜けた点）は棒より出っ張りが小さい。棒の出っ張りの中央値から 1 cm 以内だけで端を取る
    grip = float(np.median(w[L]))
    E = L & (w > grip - 0.01)
    end = float(np.percentile(u[E], 98)) if lever_side > 0 else float(np.percentile(u[E], 2))
    out.update(lever_h=float(np.median(z[L])), lever_end=end,
               lever_len_seen=float(np.percentile(u[L], 98) - np.percentile(u[L], 2)),
               backset=abs(latch - end) + NECK_M)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("crop", type=Path)
    ap.add_argument("--lever-side", choices=("left", "right"), default="left", help="機体から見た把手の側")
    ap.add_argument("--angle", type=Path, help="door_lidar_angle.py の *_angle.npz（扉が閉じた回だけ使う）")
    ap.add_argument("--max-angle", type=float, default=2.0)
    ap.add_argument("--window", type=int, default=100, help="揺れを見る区切り（回の数。10 Hz で 100 = 10 秒）")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    side = +1 if a.lever_side == "left" else -1
    scans, _ = dla.load_scans(a.crop)
    keep = np.ones(len(scans), bool)
    if a.angle:
        ang = np.load(a.angle)
        keep = (ang["ok"] > 0) & (np.abs(ang["angle_deg"]) < a.max_angle)
    idx = np.nonzero(keep)[0]
    sel = [scans[i] for i in idx]
    u, w, z, tilt = to_door_coords(sel, sel)
    allm = measure(u, w, z, side)
    wins = []
    for k in range(0, len(sel) - a.window + 1, a.window):
        uu, ww, zz, _ = to_door_coords(sel[k:k + a.window], sel)
        wins.append(measure(uu, ww, zz, side))
    print(f"== {a.crop.stem}: 使った回 {len(sel)}/{len(scans)}・板の面の傾き {tilt:+.2f}°・扉の間で見えた高さの上限 {allm['z_max_seen']:.2f} m")
    for key, label in (("width", "幅"), ("lever_h", "レバーの高さ"), ("backset", "バックセット")):
        vals = np.array([wd[key] for wd in wins])
        ok = np.isfinite(vals)
        sd = f"・{a.window // 10} 秒ごとの揺れ σ {np.std(vals[ok]) * 1000:.1f} mm（{ok.sum()}/{len(vals)} 区切り）" if ok.sum() > 1 else ""
        val = allm[key]
        print(f"  {label}: {val * 100:.1f} cm{sd}" if np.isfinite(val) else f"  {label}: 測れない（レバーの点 {allm['lever_points']}）")
    print(f"  （縁 左 {allm['edge_left']:+.3f}・右 {allm['edge_right']:+.3f} m、柱の段差 左 {allm['jamb_left_m'] * 100:.1f}・右 {allm['jamb_right_m'] * 100:.1f} cm）")
    if a.json:
        a.json.write_text(json.dumps({"all": allm, "windows": wins, "tilt_deg": tilt, "scans_used": len(sel)}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
