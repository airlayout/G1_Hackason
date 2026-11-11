"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-decision.md 5 節 3・すき間の罰の X を実測から決める）: 実機の段 2 の腕の振り出し
（段 2 の 0〜1.4 s）で、手の部品（指先・手のひら・手首・前腕の中ほど・肘）と把手の側の枠・壁の、上から見た距離を出す。numpy だけ。

扉の座標は、その回の構えの区間の点群の扉の板の直線でそろえる（geom_latch_side_1111.py の json の shift_cm・yaw_err_deg）。
開口の縁（把手の側）は k1_frame_edges_1111.py で読んだ値（片側しか写らない回は蝶番の側＋開口の幅の平均）。
枠・壁の前の面（板の表 x=−0.02 からの出）:
  巻き尺: 縁から COVER_W は枠（出 3 cm＝x −0.05）、その外は壁（出 1 cm＝x −0.03）
  点群: 縁の外は全部 x −0.05（点群の黒い面は巻き尺より約 2 cm 手前に写る。安全側の見方）
接触の目印: 左手のひらの外力（k8_press_torque_1111.py と同じ推定・0.1 s の中央値）の大きさが、止まっている時の残り（4〜5 N）を
大きく越えた所。その時刻の模型の距離が「模型では離れて見えるのに実際は触れていた量」＝X の下限の手がかり。

  ../../.venv/bin/python gap_x_from_real_1111.py [--geom logs_real/20261111/geom_1111.json]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk
import k8_press_torque_1111 as k8
import render_staged_views_1111 as rv

L = Path("logs_real/20261111")
RUNS = (("070921", "1 回目"), ("072728", "2 回目"), ("073522", "3 回目"), ("075139", "4 回目"))
EDGE = {"1 回目": 0.501, "2 回目": 0.488, "3 回目": -0.538 + 0.960, "4 回目": 0.415}  # k1_frame_edges_1111.py
BOARD_X = -0.02
COVER_W = 0.05
FRAME_X, WALL_X_TAPE, WALL_X_CLOUD = -0.05, -0.03, -0.05
T_SWING = 1.4
PELVIS_Z = 0.73
PARTS = ("指先", "手", "手首", "前腕", "肘")
CONTACT_N = 12.0


def parts_world(chain: fk.ArmFK, q: np.ndarray, quat: np.ndarray, pose: np.ndarray) -> Dict[str, np.ndarray]:
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
    R = dfr.rot_z(pose[2]) @ dfr.tilt_matrix(roll, pitch)
    base = np.array([pose[0], pose[1], PELVIS_Z])
    p = chain.points(q)
    out = {k: base + R @ p["L" + k] for k in ("指先", "手", "手首", "肘")}
    out["前腕"] = (out["手首"] + out["肘"]) / 2
    return out


def align(P: np.ndarray, shift_cm: float, yaw_deg: float) -> np.ndarray:
    b = math.tan(math.radians(yaw_deg))
    a = BOARD_X - shift_cm / 100 + 0.08 * b  # shift は y=−0.08 での量（geom_latch_side_1111.py）
    out = P.copy()
    out[..., 0] = P[..., 0] - (a + b * P[..., 1]) + BOARD_X
    return out


def gap(P: np.ndarray, edge: float, tape: bool) -> np.ndarray:
    """縁の外（y > edge）の点の、枠・壁の前の面までの距離 [m]（正＝手前に離れている・負＝面より奥＝めり込み）。縁の内は inf"""
    y, x = P[..., 1], P[..., 0]
    if tape:
        face = np.where(y <= edge + COVER_W, FRAME_X, WALL_X_TAPE)
    else:
        face = np.full_like(x, WALL_X_CLOUD)
    return np.where(y > edge, face - x, np.inf)


def analyze(run: str, label: str, geom: dict, chain: fk.ArmFK, masses) -> Dict:
    z = rv.stock_arrays(str(L / f"door_stock_20261111_{run}.npz"))
    st, t = z["staged"], z["t"]
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 2] == 2)
    idx = np.flatnonzero(in2)
    tt = t[idx] - t[idx[0]]
    sw = idx[tt <= T_SWING]
    ts = tt[tt <= T_SWING]
    g = geom[label]["構え"]
    P = {k: [] for k in PARTS}
    fmag = []
    for k in sw:
        pose = z["pose_f"][k]
        pw = parts_world(chain, z["q"][k], z["quat"][k], pose)
        for n in PARTS:
            P[n].append(pw[n])
        gv, _ = k8.gravity_in_pelvis(z["quat"][k])
        fmag.append(float(np.linalg.norm(k8.hand_force(chain, masses, z["q"][k], z["tau"][k], gv))))
    fmag = k8.smooth(np.array(fmag), ts, 0.1)
    edge = EDGE[label]
    d_tape = {n: gap(align(np.array(P[n]), g["shift_cm"], g["yaw_err_deg"]), edge, True) for n in PARTS}
    d_cloud = {n: gap(align(np.array(P[n]), g["shift_cm"], g["yaw_err_deg"]), edge, False) for n in PARTS}
    dmin_tape = np.min(np.stack([d_tape[n] for n in PARTS]), axis=0)
    dmin_cloud = np.min(np.stack([d_cloud[n] for n in PARTS]), axis=0)
    roll_tau = np.abs(z["tau"][sw, chain.index["left_shoulder_roll_joint"]])
    con = np.flatnonzero(fmag > CONTACT_N)
    res = {"run": run, "label": label, "edge": edge, "align_shift_cm": g["shift_cm"],
           "min_gap_tape_cm": float(100 * np.min(dmin_tape)), "min_gap_cloud_cm": float(100 * np.min(dmin_cloud)),
           "t_min_gap": float(ts[int(np.argmin(dmin_tape))]),
           "part_min": {n: round(float(100 * np.min(d_tape[n])), 1) for n in PARTS if np.isfinite(d_tape[n]).any()},
           "contact": []}
    # 接触の山ごと（続いた区間ごと）に、その間の模型の距離の最小
    if len(con):
        runs, start = [], con[0]
        for a_, b_ in zip(con, con[1:]):
            if b_ != a_ + 1:
                runs.append((start, a_))
                start = b_
        runs.append((start, con[-1]))
        for s_, e_ in runs:
            seg = slice(s_, e_ + 1)
            res["contact"].append({"t": [round(float(ts[s_]), 2), round(float(ts[e_]), 2)], "f_max_n": round(float(fmag[seg].max()), 1),
                                   "roll_tau_max": round(float(roll_tau[seg].max()), 1),
                                   "gap_tape_cm": round(float(100 * np.min(dmin_tape[seg])), 1),
                                   "gap_cloud_cm": round(float(100 * np.min(dmin_cloud[seg])), 1)})
    return res


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--geom", default=str(L / "geom_1111.json"))
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    geom = json.loads(Path(a.geom).read_text())
    chain = fk.load_chain()
    masses = k8.link_masses(fk.default_urdf())
    rows = [analyze(r, lab, geom, chain, masses) for r, lab in RUNS]
    for r in rows:
        print(f"{r['label']}（{r['run']}）縁 {r['edge']:+.3f}・そろえ {r['align_shift_cm']:+.1f} cm: 振り出しの間の一番近い所 "
              f"巻き尺 {r['min_gap_tape_cm']:+.1f} cm（{r['t_min_gap']:.2f} s）・点群 {r['min_gap_cloud_cm']:+.1f} cm・部品ごと {r['part_min']}")
        for c in r["contact"]:
            print(f"    接触の目印 {c['t'][0]:.2f}〜{c['t'][1]:.2f} s: 手の外力の山 {c['f_max_n']} N・肩 roll {c['roll_tau_max']} N·m・"
                  f"その間の模型の距離 巻き尺 {c['gap_tape_cm']} cm／点群 {c['gap_cloud_cm']} cm")
    if a.json:
        Path(a.json).write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
