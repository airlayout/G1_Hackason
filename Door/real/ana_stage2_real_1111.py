"""11-11 実機: 段 2（腕だけの方策 s2f）の回を記録（door_stock_*.npz）で解く。numpy だけ。

各回について:
  - 体（pelvis）の動き: FAST-LIO の口の扉の座標の姿勢（pose）・IMU の yaw・/dog_odom の yaw を段 2 の始めから
  - 手のひら（左・順運動学）の通り道を扉の座標で。本当の姿勢で（実）／段 2 の始めの姿勢のまま（狙い）／方策の目標角で（目標）
  - 巻き尺のレバー（付け根 y=幅/2−バックセット・蝶番の側へ 13 cm・板から 4.5 cm 離れた棒・上面 100.8 cm）に一番近づいた所
  - 腕の速さ・目標との差・足の力（踏み出し）

  ../../.venv/bin/python ana_stage2_real_1111.py logs_real/20261111/door_stock_20261111_075139.npz [...] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk

# 11-11 の巻き尺（ユーザー）: レバーの付け根から先 13 cm・棒と板の表の間 4.5 cm・棒の上面 100.8 cm
LEVER_LEN_M = 0.13
LEVER_GAP_M = 0.045
LEVER_TOP_Z = 1.008
BAR_R_M = 0.01  # 棒の半径（目測。中心は上面の 1 cm 下・板の表から 4.5＋1 cm）
BOARD_HALF_M = 0.02  # 扉の座標の原点は板の中心（表の 2 cm 奥・pose_from_tape と同じ）
FOOT_LIFT_N = 40.0  # 片足の力がこれ未満＝足を上げた
ARMS = slice(15, 29)
LEFT_ARM = slice(15, 22)
T_MARKS = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0)


@dataclass(frozen=True)
class Lever:
    x: float      # 棒の中心の前後（扉の座標・負が機体の側）
    y0: float     # 付け根
    y1: float     # 先（蝶番の側）
    z: float      # 棒の中心の高さ

    def nearest(self, p: np.ndarray) -> np.ndarray:
        """棒（線分）の上の p に一番近い点"""
        lo, hi = min(self.y0, self.y1), max(self.y0, self.y1)
        return np.array([self.x, float(np.clip(p[1], lo, hi)), self.z])


def lever_from_door(width: float, backset: float, lever_left: bool = True) -> Lever:
    lr = 1.0 if lever_left else -1.0
    pivot = lr * (width / 2 - backset)
    return Lever(x=-(BOARD_HALF_M + LEVER_GAP_M + BAR_R_M), y0=pivot, y1=pivot - lr * LEVER_LEN_M, z=LEVER_TOP_Z - BAR_R_M)


def yaw_of_quat_wxyz(q: np.ndarray) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def palm_path(chain: fk.ArmFK, q: np.ndarray, quat: np.ndarray, poses: np.ndarray, pelvis_z: float) -> np.ndarray:
    """各歩の左手のひら（扉の座標）。poses は [N,3]（x, y, yaw）"""
    out = np.full((len(q), 3), np.nan)
    for k in range(len(q)):
        if not np.isfinite(poses[k]).all():
            continue
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat[k])
        R = dfr.rot_z(poses[k, 2]) @ dfr.tilt_matrix(roll, pitch)
        out[k] = np.array([poses[k, 0], poses[k, 1], pelvis_z]) + R @ fk.palm_in_pelvis(chain, q[k], "left")
    return out


SIM_WALL_X = -0.07  # シムの脇の壁の面（ana_arm_wall_sim_1110.py と同じ）
WALL_NEAR_M = 0.03
WALL_POINTS = ("L指先", "L手", "L肘")


def wall_contacts(chain: fk.ArmFK, q: np.ndarray, quat: np.ndarray, poses: np.ndarray, pelvis_z: float,
                  half_width: float, t: np.ndarray) -> Dict[str, Dict[str, float]]:
    """左腕の点が開口の縁より外（y > 幅/2・把手の側の壁の前）にいる間の、一番壁へ寄った x と、シムの壁の面の 3 cm 以内にいた時間"""
    out = {}
    pts_all = []
    for k in range(len(q)):
        if not np.isfinite(poses[k]).all():
            pts_all.append(None)
            continue
        roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat[k])
        R = dfr.rot_z(poses[k, 2]) @ dfr.tilt_matrix(roll, pitch)
        base = np.array([poses[k, 0], poses[k, 1], pelvis_z])
        pts = chain.points(q[k])
        pts_all.append({n: base + R @ pts[n] for n in WALL_POINTS})
    for n in WALL_POINTS:
        xs, near_s, first = [], 0.0, None
        for k, pk in enumerate(pts_all):
            if pk is None or pk[n][1] <= half_width:
                continue
            xs.append(pk[n][0])
            if pk[n][0] > SIM_WALL_X - WALL_NEAR_M:
                near_s += 0.02
                first = t[k] if first is None else first
        out[n] = {"outside_s": round(0.02 * len(xs), 2), "max_x_outside": round(float(max(xs)), 3) if xs else None,
                  "near_sim_wall_s": round(near_s, 2), "first_near_t": None if first is None else round(float(first), 2)}
    return out


def closest(lever: Lever, path: np.ndarray, t: np.ndarray) -> Dict[str, float]:
    ok = np.isfinite(path).all(axis=1)
    if not ok.any():
        return {}
    d = np.full(len(path), np.inf)
    rel = np.full((len(path), 3), np.nan)
    for k in np.flatnonzero(ok):
        n = lever.nearest(path[k])
        rel[k] = path[k] - n
        d[k] = float(np.linalg.norm(rel[k]))
    k = int(np.argmin(d))
    # 手のひらが棒の y の範囲の上にあった間の、棒の中心からの前後・高さ
    lo, hi = min(lever.y0, lever.y1), max(lever.y0, lever.y1)
    over = ok & (path[:, 1] >= lo) & (path[:, 1] <= hi)
    res = {"t": float(t[k]), "dist_cm": 100 * d[k], "dx_cm": 100 * rel[k, 0], "dy_cm": 100 * rel[k, 1],
           "dz_cm": 100 * rel[k, 2], "palm": path[k].round(3).tolist(), "over_bar_s": float(over.sum() * 0.02)}
    if over.any():
        res["over_dx_cm_range"] = [float(100 * np.nanmin(rel[over, 0])), float(100 * np.nanmax(rel[over, 0]))]
        res["over_dz_cm_min"] = float(100 * np.nanmin(rel[over, 2]))
    return res


def analyze(path: Path, chain: fk.ArmFK) -> Dict:
    z = dict(np.load(path, allow_pickle=True))
    meta = eval(str(z["meta"]), {"__builtins__": {}}, {"inf": math.inf, "nan": math.nan, "None": None})  # noqa: S307 記録の dict の文字列
    args = meta["args"]
    width, _h, lever_h, backset = (float(v) for v in args["door"].split(","))
    pelvis_z = float(args.get("pelvis_z") or 0.75)
    lever = lever_from_door(width, backset, not args.get("lever_right", False))
    st, t = z["staged"], z["t"]
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 2] == 2)  # 段 2・腕は方策
    if not in2.any():
        return {"file": path.name, "reason": meta.get("reason"), "stage2": False}
    k0 = int(np.flatnonzero(in2)[0])
    ks = np.flatnonzero(np.isfinite(st[:, 0]) & (st[:, 0] >= 2))
    k1 = int(ks[-1])
    # 段 2 の後、POST に落ちてから 1.5 秒まで（止めた後の流れも見る）
    k_end = int(min(len(t) - 1, np.searchsorted(t, t[k1] + 1.5)))
    sl = slice(k0, k_end + 1)
    tt = t[sl] - t[k0]
    pose = z["pose"][sl].copy()
    lm = z["lidar_msg"][sl]
    # pose（方策に渡した値）は POST では書かれない → 口の電文の姿勢で埋める（同じ扉の座標）
    miss = ~np.isfinite(pose).all(axis=1)
    pose[miss] = lm[miss][:, 1:4]
    q, quat = z["q"][sl], z["quat"][sl]
    imu_yaw = np.unwrap([yaw_of_quat_wxyz(v) for v in quat])
    od = z["odom"][sl]
    p0 = pose[0]
    real = palm_path(chain, q, quat, pose, pelvis_z)
    still = palm_path(chain, q, quat, np.repeat(p0[None], len(q), 0), pelvis_z)
    qp = q.copy()
    qpol = z["q_policy"][sl]
    have_pol = np.isfinite(qpol[:, ARMS]).all(axis=1)
    qp[have_pol, ARMS] = qpol[have_pol, ARMS]
    want = palm_path(chain, qp, quat, np.repeat(p0[None], len(q), 0), pelvis_z)
    want[~have_pol] = np.nan
    dq_arm = np.abs(z["dq"][sl][:, LEFT_ARM]).max(axis=1)
    err = np.abs(qpol[:, ARMS] - q[:, ARMS]).max(axis=1)
    ff = z["foot_fz"][sl]
    lift = np.flatnonzero((ff < FOOT_LIFT_N).any(axis=1))
    in_pol = in2[sl]

    def at(series: np.ndarray, s: float):
        k = int(min(len(tt) - 1, np.searchsorted(tt, s)))
        return series[k]

    marks = []
    for s in T_MARKS:
        if s > tt[-1]:
            break
        pz = at(pose, s)
        marks.append({"t": s, "dx_cm": round(100 * (pz[0] - p0[0]), 1), "dy_cm": round(100 * (pz[1] - p0[1]), 1),
                      "dyaw_deg": round(math.degrees(wrap(pz[2] - p0[2])), 1),
                      "imu_dyaw_deg": round(math.degrees(at(imu_yaw, s) - imu_yaw[0]), 1),
                      "odom_dyaw_deg": round(math.degrees(wrap(at(od[:, 2], s) - od[0, 2])), 1) if np.isfinite(od[0]).all() else None,
                      "palm_real": [round(v, 3) for v in at(real, s)], "palm_still": [round(v, 3) for v in at(still, s)],
                      "dq_arm_max": round(float(at(dq_arm, s)), 2)})
    pol_end = float(tt[np.flatnonzero(in_pol)[-1]])
    return {
        "file": path.name, "reason": meta.get("reason"), "stage2": True,
        "door": args["door"], "pelvis_z": pelvis_z, "no_leg_release": args.get("no_leg_release"),
        "press_radius": args.get("press_radius"), "stage0_tape": args.get("stage0_tape"),
        "lever": {"x": lever.x, "y": [lever.y0, lever.y1], "z": lever.z},
        "pose0": [round(p0[0], 3), round(p0[1], 3), round(math.degrees(p0[2]), 1)],
        "policy_s": pol_end, "handover": bool(np.nanmax(st[sl][:, 4]) > 0) if np.isfinite(st[sl][:, 4]).any() else False,
        "first_foot_lift_s": float(tt[lift[0]]) if len(lift) else None,
        "dq_arm_max": float(np.nanmax(dq_arm[in_pol])), "track_err_max_rad": float(np.nanmax(err[in_pol])),
        "track_err_med_rad": float(np.nanmedian(err[in_pol])),
        "yaw_change_end_deg": round(math.degrees(wrap(pose[-1, 2] - p0[2])), 1),
        "imu_yaw_change_end_deg": round(math.degrees(imu_yaw[-1] - imu_yaw[0]), 1),
        "xy_change_end_cm": round(100 * float(np.hypot(*(pose[-1, :2] - p0[:2]))), 1),
        "wall_real": wall_contacts(chain, q[in_pol], quat[in_pol], pose[in_pol], pelvis_z, width / 2, tt[in_pol]),
        "wall_still": wall_contacts(chain, q[in_pol], quat[in_pol], np.repeat(p0[None], int(in_pol.sum()), 0), pelvis_z,
                                    width / 2, tt[in_pol]),
        "closest_real": closest(lever, real[in_pol], tt[in_pol]),
        "closest_still": closest(lever, still[in_pol], tt[in_pol]),
        "closest_want": closest(lever, want[in_pol], tt[in_pol]),
        "marks": marks,
    }


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    chain = fk.load_chain(a.urdf)
    rows = [analyze(Path(f), chain) for f in a.npz]
    for r in rows:
        print(json.dumps(r, ensure_ascii=False, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
