"""L5 の解析（2026-10-23）: door_run_real.py --door-source lidar の記録（logs/door_real_<日時>.npz）を読み、機体の日の結果をまとめる。

出すもの:
  1. 状態ごとの長さ・傾き・制御の周期（最大・99%）・ライダーの電文の遅れ（受け取ってからの古さ）
  2. 扉の方策の間（DOOR）に生徒に渡した pelvis の姿勢（track: 毎回当て直した値）と、DOOR を始めた時の姿勢から
     脚の odometry だけで運んだ姿勢（still と同じ作り）の差。差が大きくなっていけば、HOMIE の歩き方で odometry がずれている
     （ライダーの方が正しいとは限らない。止まった POST で両方を比べ、ライダーの静止の揺れ 1 mm と並べて読む）
  3. 扉の角度（生徒に渡した値）の最大・測れた割合・最後の値
  4. 立ったまま止めた理由（記録の meta の reason）

使い方（Mac）: Navigation/.venv/bin/python Door/real/analyze_l5.py <door_real_*.npz> [--png 図の出力]
"""
from __future__ import annotations

import argparse
import ast
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_frame as dfr  # noqa: E402

PHASES = ("WAIT", "RAMP", "HOLD", "STAND", "DOOR", "POST", "FREEZE", "DAMP")


def odom_carried(pose: np.ndarray, odom: np.ndarray, k0: int) -> np.ndarray:
    """k0 の pelvis の姿勢から、odometry の動きだけで運んだ姿勢 [N, 3]（k0 より前は nan）"""
    out = np.full_like(pose, np.nan)
    start, ref = dfr.PlanarPose(*pose[k0]), dfr.PlanarPose(*odom[k0])
    for k in range(k0, len(pose)):
        if np.isfinite(odom[k]).all():
            p = dfr.compose_planar(start, ref, dfr.PlanarPose(*odom[k]))
            out[k] = (p.x, p.y, p.yaw)
    return out


def _has(z, key: str) -> bool:
    return key in (z.files if hasattr(z, "files") else z)


def summarize(z) -> dict:
    """z は np.load の結果か、同じ名前の配列の辞書"""
    t = z["t"] - z["t"][0]
    ph = z["phase"].astype(int)
    out = {"phases": {}}
    for i, name in enumerate(PHASES):
        m = ph == i
        if not m.any():
            continue
        g = z["quat"][m]
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (g[:, 1] ** 2 + g[:, 2] ** 2), -1, 1)))
        per = z["period"][m] * 1000
        out["phases"][name] = {"秒": float(t[m][-1] - t[m][0]), "傾き最大°": float(np.nanmax(tilt)),
                               "周期最大ms": float(np.nanmax(per)), "周期99%ms": float(np.nanpercentile(per, 99))}
    if _has(z, "lidar_rx_age"):
        age = z["lidar_rx_age"]
        ok = np.isfinite(age)
        out["電文の古さ"] = {"中央ms": float(np.median(age[ok]) * 1000), "最大ms": float(np.max(age[ok]) * 1000)} if ok.any() else None
    door = np.nonzero(ph == PHASES.index("DOOR"))[0]
    if len(door) and np.isfinite(z["pose"][door[0]]).all():
        pose, odom = z["pose"], z["odom"]
        carried = odom_carried(pose, odom, door[0])
        after = np.arange(door[0], len(pose))
        d = np.hypot(*(pose[after, :2] - carried[after, :2]).T)
        dyaw = np.degrees(np.abs(np.arctan2(np.sin(pose[after, 2] - carried[after, 2]), np.cos(pose[after, 2] - carried[after, 2]))))
        walked = np.nansum(np.hypot(*np.diff(pose[after, :2], axis=0).T))
        out["DOOR から"] = {"歩いた道のり m": float(walked), "当て直し−odometry 最大 cm": float(np.nanmax(d) * 100),
                           "最後 cm": float(d[np.isfinite(d)][-1] * 100) if np.isfinite(d).any() else None,
                           "向きの差の最大°": float(np.nanmax(dyaw))}
        out["_curve"] = (t[after], d, pose[after], carried[after])
    if _has(z, "foot_fz"):
        # 10-26: 左右の足の鉛直の力 [N]（door_foot_load.py の逆算）。A を押す直前の HOLD の 1 秒と、STAND の平均
        fz = z["foot_fz"]
        f = {}
        hold = np.nonzero(ph == PHASES.index("HOLD"))[0]
        if len(hold):
            last = hold[t[hold] >= t[hold[-1]] - 1.0]
            f["A の前 1 秒 左 N"], f["A の前 1 秒 右 N"] = (float(v) for v in np.nanmean(fz[last], axis=0))
        stand = ph == PHASES.index("STAND")
        if stand.any():
            f["STAND 左 N"], f["STAND 右 N"] = (float(v) for v in np.nanmean(fz[stand], axis=0))
        if f:
            out["足の力"] = f
    if _has(z, "door_angle_obs"):
        a = z["door_angle_obs"]
        m = ph >= PHASES.index("STAND")
        if m.any():
            ang = np.degrees(a[m, 0])
            out["扉の角度"] = {"最大°": float(np.max(ang)), "測れた割合": float(np.mean(a[m, 1] > 0.5)), "最後°": float(ang[-1])}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--png", type=Path)
    a = ap.parse_args()
    z = np.load(a.log, allow_pickle=False)
    meta = ast.literal_eval(str(z["meta"])) if "meta" in z.files else {}
    s = summarize(z)
    print(f"== {a.log.name}・止めた理由: {meta.get('reason', '（無し）')}")
    for name, v in s["phases"].items():
        print(f"  {name:6s} {v['秒']:6.1f} s・傾き最大 {v['傾き最大°']:4.1f}°・周期 最大 {v['周期最大ms']:5.1f} ms・99% {v['周期99%ms']:5.1f} ms")
    for key in ("足の力", "電文の古さ", "DOOR から", "扉の角度"):
        if s.get(key):
            print(f"  {key}: " + "・".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in s[key].items()))
    if a.png and "_curve" in s:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams["font.family"] = "Hiragino Sans"
        tt, d, pose, carried = s["_curve"]
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        ax[0].plot(pose[:, 1], pose[:, 0], color="#2a78d6", label="生徒に渡した（毎回当て直す）")
        ax[0].plot(carried[:, 1], carried[:, 0], color="#eb6834", label="odometry だけで運んだ")
        ax[0].set_xlabel("y [m]（扉を向いて左が正）")
        ax[0].set_ylabel("x [m]（扉の板が 0）")
        ax[0].invert_xaxis()
        ax[0].set_aspect("equal")
        ax[0].legend(frameon=False)
        ax[1].plot(tt - tt[0], d * 100, color="#46505f")
        ax[1].set_xlabel("扉の方策を始めてからの秒")
        ax[1].set_ylabel("2 つの差 [cm]")
        for x in ax:
            x.grid(color="#dce1e9")
        fig.tight_layout()
        fig.savefig(a.png, dpi=130)
        print(f"  図: {a.png}")


if __name__ == "__main__":
    main()
