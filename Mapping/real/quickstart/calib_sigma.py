#!/usr/bin/env python3
"""段 2a: **MOLA の事前の σ を記録から導出する**（再生不要・純粋な計算）。

## なぜ σ なのか

MOLA は脚 odometry を `ICP_Input::prior`（`CPose3DPDFGaussianInf`）として
**ICP のコスト項に入れている**。配線は正しい。**重みだけが 2〜3 桁緩い。**

`StateEstimationSimple` の共分散モデル（`state-estimation-simple.yaml` が C++ 既定より勝つ）:

    cov = sigma_relative_pose_*^2 + (sigma_random_walk_acceleration_* x dt)^2

| パラメータ | 既定 | 環境変数 |
|---|---|---|
| `sigma_relative_pose_linear`  | 0.5 m       | `MOLA_NAVSTATE_SIGMA_POSITION` |
| `sigma_relative_pose_angular` | 0.1 rad     | `MOLA_NAVSTATE_SIGMA_ANG` |
| `sigma_random_walk_acceleration_linear`  | 1.0 m/s^2  | `MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC` |
| `sigma_random_walk_acceleration_angular` | **10.0 rad/s^2** | `MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC` |

dt = 0.1 s（10 Hz）でも sigma_rot = sqrt(0.1^2 + 1.0^2) = 1.005 rad = **57.6°**。
支配しているのは ANGACC の項。**「どちらを向いているか 57° 分からない」と宣言している**のと同じで、
これでは ICP が 180° ずれた解を選んでも事前が止められない。

## 何を真値にするか

| 量 | 真値 | 使う記録 |
|---|---|---|
| **回転** | ジャイロ積分（200 Hz） | spin 4 本 + still 2 本 |
| **並進** | **0**（静止しているので動いていない） | still 2 本 |

⚠️ **これは下限であって最終値ではない。** MOLA が要求する σ は「脚 odom の誤差」ではなく
**「事前姿勢全体の誤差」**で、状態推定器自身の外挿誤差も含む。**筋の通った出発点**として使う。

    Navigation/.venv/bin/python quickstart/calib_sigma.py \
        runs/spin_20260912T13* runs/still_20260912T13*
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_traj import (gyro_yaw, net_yaw, open_bag, read_imu, read_odom,  # noqa: E402
                       read_static_tf)

# MOLA の事前が効くのはスキャン間。LiDAR は 10 Hz なので dt = 0.1 s が動作点
DT_LIST_S = (0.05, 0.1, 0.2, 0.4, 0.8)
OPERATING_DT_S = 0.1
# 窓をずらす刻み。重なりを許して標本数を稼ぐ（独立ではないが std の推定には足りる）
STRIDE_S = 0.05


def resample(t_src: np.ndarray, v_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    return np.interp(t_dst, t_src, v_src)


def window_diffs(t: np.ndarray, a: np.ndarray, b: np.ndarray, dt: float) -> np.ndarray:
    """窓 dt の中の増分の差 (a - b) を集める。a も b も累積量。"""
    i0 = np.searchsorted(t, t[0] + np.arange(0, t[-1] - t[0] - dt, STRIDE_S))
    i1 = np.searchsorted(t, t[i0] + dt)
    ok = i1 < len(t)
    i0, i1 = i0[ok], i1[ok]
    return (a[i1] - a[i0]) - (b[i1] - b[i0])


def fit_sigma_model(dts: np.ndarray, sigmas: np.ndarray) -> tuple[float, float, float]:
    """sigma^2 = sigma_rel^2 + (sigma_rwa * dt)^2 を最小二乗で当てる。

    sigma^2 を dt^2 の 1 次式として解く。切片 -> sigma_rel、傾き -> sigma_rwa^2。
    """
    A = np.c_[np.ones_like(dts), dts**2]
    coef, *_ = np.linalg.lstsq(A, sigmas**2, rcond=None)
    rel = math.sqrt(max(coef[0], 0.0))
    rwa = math.sqrt(max(coef[1], 0.0))
    pred = np.sqrt(np.maximum(coef[0] + coef[1] * dts**2, 0.0))
    r2 = 1.0 - ((sigmas - pred) ** 2).sum() / max(((sigmas - sigmas.mean()) ** 2).sum(), 1e-18)
    return rel, rwa, float(r2)


def collect(run_dir: Path) -> dict | None:
    con, tid = open_bag(run_dir)
    imu, odom, st = read_imu(con, tid), read_odom(con, tid), read_static_tf(con, tid)
    if imu is None or odom is None or st is None:
        print("  {}: IMU か /dog_odom か /tf_static が無い。飛ばす".format(run_dir.name))
        return None
    R_bl_imu = Rotation.from_quat(st[1]).as_matrix()
    t_g, yaw_g = gyro_yaw(imu, R_bl_imu)

    # 共通の時間軸は脚 odom を 100 Hz に間引いたもの（1010 Hz は細かすぎる）
    t0 = max(t_g[0], odom[0, 0]); t1 = min(t_g[-1], odom[-1, 0])
    t = np.arange(t0, t1, 0.01)
    yaw_o = resample(odom[:, 0], net_yaw(odom[:, 4:8]), t)
    yaw_gi = resample(t_g, yaw_g, t)
    x = resample(odom[:, 0], odom[:, 1], t)
    y = resample(odom[:, 0], odom[:, 2], t)

    rate = np.abs(np.gradient(yaw_gi, t))
    is_still = float(np.median(rate)) < 0.02

    out = {"run": run_dir.name, "still": is_still, "ang": {}, "lin": {}}
    for dt in DT_LIST_S:
        d = window_diffs(t, yaw_o, yaw_gi, dt)
        if len(d) > 20:
            out["ang"][dt] = {"sigma_rad": float(d.std()), "n": int(len(d))}
        if is_still:
            # 静止なので真値は 0。脚 odom の増分そのものが誤差
            zero = np.zeros_like(x)
            dx = window_diffs(t, x, zero, dt)
            dy = window_diffs(t, y, zero, dt)
            if len(dx) > 20:
                out["lin"][dt] = {"sigma_m": float(np.hypot(dx, dy).std()), "n": int(len(dx))}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()

    runs = []
    for r in args.runs:
        got = collect(r)
        if got:
            runs.append(got)

    print("\n=== 記録ごとの sigma（窓 dt の中の「脚 odom - ジャイロ」の標準偏差）===")
    hdr = "  {:<28} {:>6}" + " {:>9}" * len(DT_LIST_S)
    print(hdr.format("記録", "種別", *["dt={}".format(d) for d in DT_LIST_S]))
    for r in runs:
        vals = ["{:.4f}".format(math.degrees(r["ang"][d]["sigma_rad"])) if d in r["ang"] else "-"
                for d in DT_LIST_S]
        print(hdr.format(r["run"][:28], "静止" if r["still"] else "旋回", *vals))
    print("  （単位は度）")

    # 角度: 旋回の記録だけを使う（静止では回転の誤差が出ない）
    spin = [r for r in runs if not r["still"]]
    still = [r for r in runs if r["still"]]

    res = {}
    for label, group, key, unit, scale in (
            ("角度", spin or runs, "ang", "rad", 1.0),
            ("並進", still, "lin", "m", 1.0)):
        pts = {}
        for r in group:
            for dt, v in r[key].items():
                pts.setdefault(dt, []).append(v["sigma_rad" if key == "ang" else "sigma_m"])
        if not pts:
            continue
        dts = np.array(sorted(pts))
        sig = np.array([np.mean(pts[d]) for d in dts])
        rel, rwa, r2 = fit_sigma_model(dts, sig)
        at_op = math.sqrt(rel**2 + (rwa * OPERATING_DT_S) ** 2)
        res[label] = {"sigma_rel": rel, "sigma_rwa": rwa, "r2": r2,
                      "at_dt_0.1": at_op, "unit": unit, "runs": [r["run"] for r in group]}
        print("\n=== {} の当てはめ（{} 本）===".format(label, len(group)))
        for d, s in zip(dts, sig):
            print("  dt={:<5} sigma = {:.5f} {}{}".format(
                d, s * scale, unit,
                "  ({:.3f}°)".format(math.degrees(s)) if key == "ang" else ""))
        print("  -> sigma_relative_pose = {:.5f} {}   sigma_random_walk_acc = {:.5f} {}/s^2   R^2 = {:.3f}"
              .format(rel, unit, rwa, unit, r2))
        print("     dt = 0.1 s での sigma = {:.5f} {}{}".format(
            at_op, unit, "  ({:.2f}°)".format(math.degrees(at_op)) if key == "ang" else ""))

    print("\n=== 既定との差 ===")
    if "角度" in res:
        d = res["角度"]
        cur = math.sqrt(0.1**2 + (10.0 * OPERATING_DT_S) ** 2)
        print("  角度: 既定 {:.3f} rad ({:.1f}°)  ->  導出 {:.5f} rad ({:.2f}°)   **{:.0f} 倍きつい**"
              .format(cur, math.degrees(cur), d["at_dt_0.1"], math.degrees(d["at_dt_0.1"]),
                      cur / d["at_dt_0.1"]))
        print("    MOLA_NAVSTATE_SIGMA_ANG={:.5f}  MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC={:.5f}"
              .format(d["sigma_rel"], d["sigma_rwa"]))
    if "並進" in res:
        d = res["並進"]
        cur = math.sqrt(0.5**2 + (1.0 * OPERATING_DT_S) ** 2)
        print("  並進: 既定 {:.3f} m  ->  導出 {:.5f} m   **{:.0f} 倍きつい**"
              .format(cur, d["at_dt_0.1"], cur / d["at_dt_0.1"]))
        print("    MOLA_NAVSTATE_SIGMA_POSITION={:.5f}  MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC={:.5f}"
              .format(d["sigma_rel"], d["sigma_rwa"]))

    print("\n  ⚠️ これは**下限**。MOLA が要る sigma は脚 odom の誤差ではなく事前姿勢全体の誤差で、")
    print("     状態推定器自身の外挿誤差も含む。段 2b の掃引の出発点として使う。")

    if args.json:
        args.json.write_text(json.dumps({"per_run": runs, "fit": res},
                                        ensure_ascii=False, indent=2))
        print("\n  書き出し: {}".format(args.json))


if __name__ == "__main__":
    main()
