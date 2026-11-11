"""11-11: シムの段 2（s2f）の評価の記録で、実機の段 2（ana_stage2_real_1111.py）と同じ量を測る。numpy だけ（OMEN の CPU）。

記録: state_recorder の records/<名前>/states_*.npz（台の原点＝扉の座標・0.02 s ごと）と static.npz。各台の最初の回だけ。
測るもの（回の始め＝構えの位置で腕が学習の初めの姿勢から動き出す時を 0 秒）:
  - pelvis の向きと位置の変わり（立つ HOMIE・歩きの指令 0）
  - 左の手のひら（順運動学）の通り道と、レバーに初めて触れた時（hand_lever_force の大きさ > 1 N）の手のひら（レバーの部品の原点から）
  - 左腕の関節の速さの最大（差分）・手のひらが外へ出た一番（y の最大）・指先が開口の縁より外で壁の面（x=−0.07）へ一番寄った x
  - レバーの部品の位置（板の中心から前・付け根の y・高さ）とレバーの長さ（door_handle_width）

  python ana_stage2_sim_1111.py <records/start_1110_1703> [--urdf g1_29dof_rev_1_0.urdf] [--json out.json] [--wall-x -0.07]
  --wall-x: 壁の前の面の x（既定 −0.07＝シムの元の壁 5 cm。0104 の課題 door_en1154_g13_shaped_left_weight_realwall は −0.03）
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
from typing import Dict, List

import numpy as np

import g1_arm_fk as fk

CONTACT_N = 1.0
T_MARKS = (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0)
LEFT_ARM_NAMES = [n for n in fk.JOINT_ORDER if n.startswith("left_") and ("shoulder" in n or "elbow" in n or "wrist" in n)]
WALL_X = -0.07


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_xyzw(q: np.ndarray) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pct(v: List[float], ps=(10, 50, 90)) -> List[float]:
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], dtype=float)
    return np.percentile(v, ps).round(3).tolist() if len(v) else []


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("records")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--wall-x", type=float, default=WALL_X)
    a = ap.parse_args(argv)
    chain = fk.load_chain(a.urdf)
    st = np.load(os.path.join(a.records, "static.npz"), allow_pickle=True)
    names = [str(n) for n in st["dof_names"]]
    order = [names.index(n) for n in fk.JOINT_ORDER]
    larm = [names.index(n) for n in LEFT_ARM_NAMES]
    bodies = [str(n) for n in st["door_body_names"]]
    H, PANEL = bodies.index("door_handle"), bodies.index("door_panel")
    dt = float(st["dt"])
    keys = ("root_pos", "root_quat_xyzw", "dof_pos", "episode_length", "door_dof_pos", "door_body_pos", "hand_lever_force",
            "grasp_target", "upper_vel_max")
    files = sorted(glob.glob(os.path.join(a.records, "states_*.npz")))
    Z = {k: np.concatenate([np.load(f)[k] for f in files]) for k in keys}
    T, N = Z["episode_length"].shape
    rows = []
    for e in range(N):
        ep = Z["episode_length"][:, e]
        end = np.nonzero(np.diff(ep) < 0)[0]
        L = int(end[0] + 1) if len(end) else T
        t = np.arange(L) * dt
        f = np.linalg.norm(Z["hand_lever_force"][:L, e], axis=-1)
        kc = np.flatnonzero(f > CONTACT_N)
        kc = int(kc[0]) if len(kc) else None
        k_hi = kc if kc is not None else L - 1
        yaw = np.unwrap([yaw_xyzw(Z["root_quat_xyzw"][k, e]) for k in range(L)])
        rp = Z["root_pos"][:L, e]
        palms, tips = [], []
        for k in range(L):
            q = Z["dof_pos"][k, e][order]
            R = quat_xyzw_to_matrix(Z["root_quat_xyzw"][k, e])
            palms.append(rp[k] + R @ fk.palm_in_pelvis(chain, q, "left"))
            tips.append(rp[k] + R @ chain.points(q)["L指先"])
        palms, tips = np.array(palms), np.array(tips)
        lever0 = Z["door_body_pos"][0, e, H]
        panel0 = Z["door_body_pos"][0, e, PANEL]
        hw = float(st["door_width"][e]) / 2
        dq = np.abs(np.diff(Z["dof_pos"][:L, e][:, larm], axis=0)) / dt
        out_tip = tips[: k_hi + 1][tips[: k_hi + 1, 1] > hw]
        r = {"env": e, "opened3": bool(np.degrees(Z["door_dof_pos"][:L, e, 0]).max() >= 3.0),
             "lever_len": float(st["door_handle_width"][e]), "lever_h": float(st["door_handle_height"][e]), "half_width": hw,
             "lever_x": float(lever0[0] - panel0[0]), "lever_y_from_edge": float(hw - lever0[1]), "lever_z": float(lever0[2]),
             "pose0": [float(rp[0, 0]), float(rp[0, 1]), math.degrees(yaw[0])],
             "t_contact": None if kc is None else float(t[kc]),
             "palm_rel_lever_at_contact": None if kc is None else (palms[kc] - lever0).round(3).tolist(),
             "palm_x_at_contact": None if kc is None else float(palms[kc, 0]),
             "dyaw_at_contact_deg": None if kc is None else math.degrees(yaw[kc] - yaw[0]),
             "dyaw_max_before_contact_deg": float(np.degrees(np.abs(yaw[: k_hi + 1] - yaw[0])).max()),
             "dxy_max_before_contact_cm": float(100 * np.hypot(*(rp[: k_hi + 1, :2] - rp[0, :2]).T).max()),
             "dq_arm_max_before_contact": float(dq[:k_hi].max()) if k_hi > 0 else None,
             "palm_y_max_before_contact": float(palms[: k_hi + 1, 1].max()),
             "tip_outside_max_x": float(out_tip[:, 0].max()) if len(out_tip) else None,
             "tip_near_wall_s": float(dt * (out_tip[:, 0] > a.wall_x - 0.03).sum()) if len(out_tip) else 0.0,
             "marks": {}}
        for s in T_MARKS:
            k = int(round(s / dt))
            if k < L:
                r["marks"][s] = {"palm": palms[k].round(3).tolist(), "dyaw_deg": round(math.degrees(yaw[k] - yaw[0]), 2)}
        rows.append(r)
    con = [r for r in rows if r["t_contact"] is not None]
    summ = {
        "records": a.records, "wall_x": a.wall_x, "episodes": N, "contact": len(con), "opened3": sum(r["opened3"] for r in rows),
        "lever_len_pct": pct([r["lever_len"] for r in rows]), "lever_x_pct": pct([r["lever_x"] for r in rows]),
        "lever_y_from_edge_pct": pct([r["lever_y_from_edge"] for r in rows]), "lever_z_minus_h_pct": pct([r["lever_z"] - r["lever_h"] for r in rows]),
        "t_contact_pct": pct([r["t_contact"] for r in con]),
        "palm_rel_lever_at_contact_x_pct": pct([r["palm_rel_lever_at_contact"][0] for r in con]),
        "palm_rel_lever_at_contact_y_pct": pct([r["palm_rel_lever_at_contact"][1] for r in con]),
        "palm_rel_lever_at_contact_z_pct": pct([r["palm_rel_lever_at_contact"][2] for r in con]),
        "palm_x_at_contact_pct": pct([r["palm_x_at_contact"] for r in con]),
        "dyaw_max_before_contact_deg_pct": pct([r["dyaw_max_before_contact_deg"] for r in rows], (50, 90, 99)),
        "dxy_max_before_contact_cm_pct": pct([r["dxy_max_before_contact_cm"] for r in rows], (50, 90, 99)),
        "dq_arm_max_before_contact_pct": pct([r["dq_arm_max_before_contact"] for r in rows], (10, 50, 90)),
        "palm_y_max_minus_half_width_pct": pct([r["palm_y_max_before_contact"] - r["half_width"] for r in rows]),
        "tip_outside_max_x_pct": pct([r["tip_outside_max_x"] for r in rows if r["tip_outside_max_x"] is not None]),
        "tip_near_wall_eps": sum(r["tip_near_wall_s"] > 0 for r in rows),
        "marks_palm_median": {str(s): np.median([r["marks"][s]["palm"] for r in rows if s in r["marks"]], axis=0).round(3).tolist()
                              for s in T_MARKS},
        "marks_palm_minus_halfwidth_y_median": {str(s): float(np.median([r["marks"][s]["palm"][1] - r["half_width"]
                                                                          for r in rows if s in r["marks"]])) for s in T_MARKS},
        "marks_dyaw_p90": {str(s): float(np.percentile([abs(r["marks"][s]["dyaw_deg"]) for r in rows if s in r["marks"]], 90))
                           for s in T_MARKS},
        "pose0_pct": {"x": pct([r["pose0"][0] for r in rows]), "y_minus_half_width": pct([r["pose0"][1] - r["half_width"] for r in rows]),
                      "yaw": pct([r["pose0"][2] for r in rows])},
    }
    print(json.dumps(summ, ensure_ascii=False, indent=1))
    if a.json:
        with open(a.json, "w") as fo:
            json.dump({"summary": summ, "rows": rows}, fo, ensure_ascii=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
