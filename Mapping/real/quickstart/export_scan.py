#!/usr/bin/env python3
"""記録から**1 スキャンを base_link 系で**書き出す。再定位の探索に食わせるため。

## なぜ要るのか

`mola_relocalization` は C++ ライブラリしか無い（Python バインディングは apt にも
入っていない。2026-09-11 に確認）。探索の試作は C++ で書くことになるが、
**rosbag2 を C++ で読むのは重い**。記録を読む道具は Python 側に既にある
（`measure_overlay.read_bag`）ので、**間に平文の XYZ を挟む**。

## 座標系（ここを間違えると静かに外れる）

再定位は「ロボットを格子の各点に置いてスキャンを地図に当てる」ので、
渡すスキャンは **base_link 系**でなければならない。生 LiDAR の点は `livox_frame`
に居るので `/tf_static` の `base_link -> livox_frame` を掛ける。
この機体は LiDAR が上下逆さま（roll 178°）なので、掛け忘れると**落ちずに数字だけ下がる**。

`map -> base_link` は**掛けない**（それが探索で求める当のもの）。
ただし記録には入っているので、**真値として JSON に添える**。

使い方:

    Navigation/.venv/bin/python quickstart/export_scan.py \\
        runs/stage_20260910T084053/bag --index 20 --out /tmp/scan20

    # 出るもの: /tmp/scan20.xyz（x y z の平文）と /tmp/scan20.json（真値の姿勢など）
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import RANGE_MAX, RANGE_MIN, read_bag  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--index", type=int, default=None,
                    help="何枚目のスキャンか（既定は /tf の範囲内の真ん中）")
    ap.add_argument("--out", type=Path, required=True, help="拡張子なしの書き出し先")
    ap.add_argument("--band", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                    help="base_link 系の z でこの帯だけ残す（壁の帯を試すとき）")
    a = ap.parse_args()

    tfs, scans, sensor_tf = read_bag(a.bag)
    if len(tfs) < 2 or not scans:
        raise SystemExit("map->base_link の /tf か生 LiDAR が足りない")
    tt = tfs[:, 0]

    usable = [i for i, (ts, _) in enumerate(scans) if tt[0] <= ts <= tt[-1]]
    if not usable:
        raise SystemExit("/tf の時間範囲に入るスキャンが 1 枚も無い")
    index = a.index if a.index is not None else usable[len(usable) // 2]
    if index not in usable:
        raise SystemExit("index {} は /tf の範囲外。使えるのは {}..{}".format(
            index, usable[0], usable[-1]))

    ts, pts = scans[index]

    # ── base_link 系へ（map へは動かさない）──────────────────────────────
    if sensor_tf is None:
        s_t, s_R = np.zeros(3), np.eye(3)
        note = "/tf_static に base_link->livox_frame が無いので恒等とみなした"
    else:
        s_t, s_q = sensor_tf
        s_R = Rotation.from_quat(s_q).as_matrix()
        rpy = Rotation.from_quat(s_q).as_euler("ZYX", degrees=True)[::-1]
        note = "base_link->livox_frame xyz {} / rpy {} deg".format(
            np.round(s_t, 4).tolist(), np.round(rpy, 2).tolist())

    p = pts[np.isfinite(pts).all(1)]
    d = np.linalg.norm(p, axis=1)
    p = p[(d > RANGE_MIN) & (d < RANGE_MAX)]
    p = p @ s_R.T + s_t
    if a.band is not None:
        p = p[(p[:, 2] >= a.band[0]) & (p[:, 2] <= a.band[1])]
        if len(p) == 0:
            raise SystemExit("帯 {} に点が 1 つも残らない".format(a.band))

    # ── 真値（探索の採点用。探索には渡さない）────────────────────────────
    truth_t = [float(np.interp(ts, tt, tfs[:, 1 + i])) for i in range(3)]
    q = np.array([np.interp(ts, tt, tfs[:, 4 + i]) for i in range(4)])
    q = q / np.linalg.norm(q)
    truth_yaw = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                           1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))

    a.out.parent.mkdir(parents=True, exist_ok=True)
    xyz = a.out.with_suffix(".xyz")
    np.savetxt(xyz, p, fmt="%.4f")
    meta = {
        "bag": str(a.bag), "scan_index": index, "stamp": float(ts),
        "points": int(len(p)), "band": a.band,
        "frame": "base_link", "sensor_tf": note,
        "truth_map_base_link": {"x": truth_t[0], "y": truth_t[1], "z": truth_t[2],
                                "yaw_rad": float(truth_yaw),
                                "yaw_deg": float(math.degrees(truth_yaw))},
    }
    a.out.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))

    print("{} 枚目（t={:.3f}）を base_link 系で {} 点 -> {}".format(index, ts, len(p), xyz))
    print("  {}".format(note))
    print("  真値 map->base_link: x {:.3f} / y {:.3f} / yaw {:.2f} deg".format(
        truth_t[0], truth_t[1], math.degrees(truth_yaw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
