"""L5 の準備（2026-10-23）: その場で回った記録から、pelvis（/dog_odom の基準点）から見た頭のライダーの水平の位置を解く。

なぜ: ライダーは pelvis の約 0.46 m 上で、首の垂れ（日によって 24° 動く）の分だけ前後にずれる。シミュレータで、
止まって測る最初の値が 2 cm 片寄ると成功率が 90.6 → 81.2% に落ちた（L3 の start2）ので、機体の日に測り直す。

解き方: 止まっている区間 k ごとに、ライダーの扉の座標での姿勢 L_k（lidar_door_est の重ね直し）と /dog_odom の姿勢 O_k を取り、
  L_k = T ⊕ O_k ⊕ o    （T: odometry の座標 → 扉の座標、o: pelvis から見たライダー〔前・左〕）
の T（3 つ）と o（2 つ）を最小二乗で解く。向きの違う区間（90° ずつ回る）が要る。/dog_odom の並進の倍率の誤差（横 7〜10% 短い）は、
その場で回るだけなら動きが小さいので効きが小さい。
⚠️ /dog_odom の基準点を pelvis とみなしている（確かめていない）。L5 の準備で巻き尺の pelvis（両足の間の中心）と比べる。

使い方（Mac）: Navigation/.venv/bin/python Door/real/lidar_offset_from_turns.py <ライダーのセッション> <odom のセッション> <remote.log> [--ref <扉の正面で止まった記録>]
  記録は PC2 の rec_take.sh で録る（扉の前で、純正の歩行のまま 90° ずつ 4 回・各 5 秒止まる）
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_door_est as L  # noqa: E402
from dog_odom_bag import load_odom  # noqa: E402
from lidar_still_icp import EDGE_S  # noqa: E402
from lidar_track_vs_icp import load_scans, make_tracker  # noqa: E402
from take_quicklook import remote_events, still_segments  # noqa: E402


def solve_offset(lidar: np.ndarray, odom: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """lidar・odom: [K, 3]（x, y, yaw）。返り値は o（前・左）、T（x, y, yaw）、位置の残差の RMS [m]"""
    dth = np.arctan2(np.sin(lidar[:, 2] - odom[:, 2]), np.cos(lidar[:, 2] - odom[:, 2]))
    t_yaw = math.atan2(float(np.mean(np.sin(dth))), float(np.mean(np.cos(dth))))
    c, s = math.cos(t_yaw), math.sin(t_yaw)
    R = np.array([[c, -s], [s, c]])
    rows, rhs = [], []
    for (lx, ly, _), (ox, oy, oth) in zip(lidar, odom):
        b = np.array([lx, ly]) - R @ np.array([ox, oy])
        ca, sa = math.cos(t_yaw + oth), math.sin(t_yaw + oth)
        rows += [[1.0, 0.0, ca, -sa], [0.0, 1.0, sa, ca]]
        rhs += [b[0], b[1]]
    A, y = np.array(rows), np.array(rhs)
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = A @ sol - y
    return sol[2:], np.array([sol[0], sol[1], t_yaw]), float(np.sqrt(np.mean(res ** 2)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("li", type=Path)
    ap.add_argument("odom", type=Path)
    ap.add_argument("remote", type=Path)
    ap.add_argument("--ref", type=Path, help="記録の始めで扉が測れないとき、扉と地図を作る記録")
    a = ap.parse_args()
    tr_odom = load_odom(a.odom)
    t0 = float(tr_odom.recv_s[0])
    segs = still_segments(tr_odom, remote_events(a.remote, t0))
    t, scans = load_scans(a.li)
    tracker, how = make_tracker(scans, a.ref)
    est = [tracker.step(ti, s) for ti, s in zip(t, scans)]
    acc = np.array([e.accepted for e in est])
    pose = np.array([[e.lidar.x, e.lidar.y, e.lidar.yaw] for e in est])
    lid, odo = [], []
    ot = tr_odom.recv_s
    for sa, sb in segs:
        lo, hi = t0 + sa + EDGE_S, t0 + sb - EDGE_S
        m = (t >= lo) & (t <= hi) & acc
        mo = (ot >= lo) & (ot <= hi)
        if m.sum() < 5 or mo.sum() < 50:
            continue
        lid.append([np.median(pose[m, 0]), np.median(pose[m, 1]),
                    math.atan2(np.median(np.sin(pose[m, 2])), np.median(np.cos(pose[m, 2])))])
        oy = tr_odom.yaw[mo]
        odo.append([tr_odom.xyz[mo, 0].mean(), tr_odom.xyz[mo, 1].mean(), math.atan2(np.mean(np.sin(oy)), np.mean(np.cos(oy)))])
    lid, odo = np.array(lid), np.array(odo)
    spread = float(np.degrees(np.ptp(np.unwrap(lid[:, 2])))) if len(lid) else 0.0
    print(f"== {a.li.name}: 止まった区間 {len(lid)}・向きの幅 {spread:.0f}°・扉と地図 {how}")
    if len(lid) < 3 or spread < 60:
        raise SystemExit("[offset] 向きの違う止まった区間が足りない（90° ずつ 4 回回って、各 5 秒止まる）")
    o, T, rms = solve_offset(lid, odo)
    print(f"  pelvis（/dog_odom の基準点）から見たライダー: 前 {o[0]:+.3f} m・左 {o[1]:+.3f} m（位置の残差 RMS {rms * 100:.1f} cm）")
    # 1 つずつ抜いて解き直し、値の揺れを見る
    if len(lid) >= 4:
        loo = np.array([solve_offset(np.delete(lid, k, 0), np.delete(odo, k, 0))[0] for k in range(len(lid))])
        print(f"  1 区間ずつ抜いた解の幅: 前 {np.ptp(loo[:, 0]) * 100:.1f} cm・左 {np.ptp(loo[:, 1]) * 100:.1f} cm")
    print(f"  → run_door_lidar.sh の CENTER=\"{o[0]:.3f} {o[1]:.3f}\"")


if __name__ == "__main__":
    main()
