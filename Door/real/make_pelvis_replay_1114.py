"""11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D）: 実機の記録から、シムで再生する
pelvis の動き（3a に入ってからの、扉の座標の前後・横・向きと、IMU の roll・pitch の差）を回ごとに書き出す。

  python make_pelvis_replay_1114.py "logs_real/robot_20261113/door_stock_*.npz" --out logs_real/robot_20261113/pelvis_replay_1114.npz

出力（npz）: labels (R,) 回の番号（作業ログ 11-13 の 1〜21）・t (M,) 3a に入ってからの秒（50 Hz）・
d (R, M, 5) [前後 m, 横 m, 向き rad, roll rad, pitch rad] の 3a の入りからの差（回の終わりより後は最後の値のまま）・
n (R,) 回ごとの有効な長さ・press_cm (R,) 2d の下がり・walk3a (R,) 3a で足踏みしたか。
姿勢（記録の pose・50 Hz で毎回更新）は 0.1 秒の移動平均でならす（FAST-LIO の細かい揺れが速さの先回りに入らないように）。
"""
from __future__ import annotations

import argparse
import glob
import math
from typing import Dict, List

import numpy as np

import ana_script_run as asr
import door_frame as dfr
import door_script as dscr

S2D, S3A = dscr.SUBS.index("2d"), dscr.SUBS.index("3a")
SMOOTH_N = 5          # 0.1 秒の移動平均
MAX_S = 12.0          # 書き出す長さの上限 [s]
DT = 0.02


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def smooth(x: np.ndarray, n: int = SMOOTH_N) -> np.ndarray:
    """周期の向きの移動平均（端は短い窓）"""
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = x[max(0, i - n // 2): i + n // 2 + 1].mean(axis=0)
    return out


def run_delta(pose: np.ndarray, quat: np.ndarray, sub: np.ndarray, vel_want: np.ndarray) -> Dict:
    """1 回ぶんの区間から、3a の入りからの差 [前後, 横, 向き, roll, pitch]（3a に入っていなければ None）"""
    i3 = np.where(sub == S3A)[0]
    if not i3.size:
        return {}
    k0 = int(i3[0])
    rp = np.array([dfr.roll_pitch_from_quat_wxyz(q) for q in quat])
    yaw = np.unwrap(pose[:, 2])
    raw = np.c_[pose[:, 0], pose[:, 1], yaw, rp]
    sm = smooth(raw)
    d = sm[k0:] - sm[k0]
    d[:, 2:] = _wrap(d[:, 2:])
    d = d[: int(MAX_S / DT)]
    walk3a = bool(np.any(vel_want[i3, 0] > 0))
    return {"d": d, "walk3a": walk3a}


def build(paths: List[str]) -> Dict[str, np.ndarray]:
    labels, ds, presses, walks = [], [], [], []
    label = 0
    for path in sorted(paths):
        z = np.load(path, allow_pickle=True)
        meta = asr.load_meta(z)
        arr = {k: z[k] for k in ("phase", "script", "pose", "quat", "vel_want")}
        for a0, a1 in asr.door_spans(arr["phase"], asr.door_index(meta)):
            label += 1
            sub = arr["script"][a0:a1, 0]
            r = run_delta(arr["pose"][a0:a1], arr["quat"][a0:a1], sub, arr["vel_want"][a0:a1])
            if not r:
                continue
            d2 = np.where(sub == S2D)[0]
            press = 100 * float(arr["script"][a0 + d2[0], 6] - arr["script"][a0 + d2[-1], 6]) if d2.size else math.nan
            labels.append(label)
            ds.append(r["d"])
            presses.append(press)
            walks.append(r["walk3a"])
    m = max(len(d) for d in ds)
    out = np.zeros((len(ds), m, 5))
    for k, d in enumerate(ds):
        out[k, : len(d)] = d
        out[k, len(d):] = d[-1]
    return {"labels": np.array(labels), "t": np.arange(m) * DT, "d": out, "n": np.array([len(d) for d in ds]),
            "press_cm": np.array(presses), "walk3a": np.array(walks)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--labels", default="", help="書き出す回（例 1,3,7）。空なら 3a に入った全部")
    a = ap.parse_args(argv)
    res = build(sorted(p for g in a.npz for p in glob.glob(g)))
    if a.labels:
        keep = np.isin(res["labels"], [int(v) for v in a.labels.split(",")])
        res = {k: (v[keep] if k != "t" else v) for k, v in res.items()}
    np.savez(a.out, **res)
    for k, lb in enumerate(res["labels"]):
        d = res["d"][k, : res["n"][k]]
        i1 = min(len(d) - 1, int(1.0 / DT))
        print(f"[replay] 回 {lb:2d}: {res['n'][k] * DT:4.1f} 秒・下がり {res['press_cm'][k]:4.1f} cm・足踏み {bool(res['walk3a'][k])}・"
              f"1 秒後 前 {100 * d[i1, 0]:+5.1f} 横 {100 * d[i1, 1]:+5.1f} cm 向き {math.degrees(d[i1, 2]):+5.1f}°・"
              f"最大の横 {100 * np.max(np.abs(d[:, 1])):4.1f} cm")
    print(f"[replay] 書いた: {a.out}（{len(res['labels'])} 回・{len(res['t'])} 点）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
