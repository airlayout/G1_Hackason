"""段 6 を記録で出す。/tf の代わりに MOLA-LO の traj.txt（TUM）で姿勢を与える。

measure_overlay.py と同じ判定式・同じ定数を使う（read_map / Cdr をそのまま借りる）。
3 GB の bag を全部メモリに載せないよう、スキャンは 1 枚ずつ流す。
"""
import argparse
import sqlite3
import sys
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_overlay as mo


def stream_scans(bag_dir: Path):
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    topic = "/utlidar/cloud_livox_mid360"
    n_bad = 0
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
        r = mo.Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        fid = r.st()
        r.u32(); r.u32()
        for _ in range(r.u32()):
            r.st(); r.u32(); r.u8(); r.u32()
        r.u8()
        ps = r.u32(); r.u32(); dl = r.u32()
        if fid != "livox_frame" or ps != mo.LIVOX_POINT_STEP:
            n_bad += 1
            continue
        raw = np.frombuffer(blob, dtype=np.uint8, count=dl, offset=r.a)
        xyz = raw.reshape(-1, mo.LIVOX_POINT_STEP)[:, :12].copy().view(np.float32).reshape(-1, 3)
        yield sec + nsec * 1e-9, xyz
    print(f"  （汚染メッセージを {n_bad} 件弾いた）", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("traj", type=Path, help="MOLA-LO の traj.txt（TUM: t x y z qx qy qz qw）")
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--tol-cells", type=int, default=1)
    ap.add_argument("--split", type=int, default=4, help="何区間に分けて内訳を出すか")
    a = ap.parse_args()

    occ, res, ox, oy = mo.read_map(a.map_yaml)
    h, w = occ.shape
    print(f"事前地図 {w}x{h} セル / res {res} m / origin ({ox}, {oy}) / 占有 {occ.sum()} セル")

    T = np.loadtxt(a.traj)
    tt = T[:, 0]
    slerp = Slerp(tt, Rotation.from_quat(T[:, 4:8]))
    print(f"軌跡 {len(T)} 姿勢 / {tt[-1]-tt[0]:.1f} 秒")

    k = 2 * a.tol_cells + 1
    occ_tol = binary_dilation(occ, np.ones((k, k), bool))

    edges = np.linspace(tt[0], tt[-1], a.split + 1)
    seg_tot = np.zeros(a.split, np.int64)
    seg_hit = np.zeros(a.split, np.int64)
    seg_tol = np.zeros(a.split, np.int64)

    tot = hit = hit_tol = used = 0
    for ts, p in stream_scans(a.bag):
        if not (tt[0] <= ts <= tt[-1]):
            continue
        used += 1
        p = p[np.isfinite(p).all(1)]
        d = np.linalg.norm(p, axis=1)
        p = p[(d > mo.RANGE_MIN) & (d < mo.RANGE_MAX)]
        if not len(p):
            continue
        t = np.array([np.interp(ts, tt, T[:, 1 + i]) for i in range(3)])
        q = (slerp(ts).as_matrix() @ p.T).T + t
        q = q[(q[:, 2] > mo.WALL_Z_MIN) & (q[:, 2] < mo.WALL_Z_MAX)]
        ix = np.floor((q[:, 0] - ox) / res).astype(int)
        iy = np.floor((q[:, 1] - oy) / res).astype(int)
        m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[m], iy[m]
        row = h - 1 - iy
        nh = int(occ[row, ix].sum())
        nt = int(occ_tol[row, ix].sum())
        tot += len(ix); hit += nh; hit_tol += nt
        s = min(int(np.searchsorted(edges, ts, "right")) - 1, a.split - 1)
        seg_tot[s] += len(ix); seg_hit[s] += nh; seg_tol[s] += nt
        if used % 1000 == 0:
            print(f"  ... {used} 枚 / ここまで {100*hit/max(tot,1):.1f} %", flush=True)

    if not tot:
        print("地図の範囲に落ちた点が無い", file=sys.stderr)
        return 1
    print(f"\n=== 段 6: 歩行中の重畳（記録 {a.bag.name}）===")
    print(f"  使ったスキャン {used} 枚 / 評価した点 {tot:,}")
    print(f"  占有セルに乗った割合                  {100*hit/tot:.1f} %")
    print(f"  ±{a.tol_cells} セル({a.tol_cells*res*100:.0f} cm)まで許した割合  {100*hit_tol/tot:.1f} %")
    print(f"\n  区間ごと（歩行が進むと崩れないかを見る）:")
    for i in range(a.split):
        if seg_tot[i]:
            print(f"    区間{i+1} ({edges[i]-tt[0]:5.0f}-{edges[i+1]-tt[0]:5.0f}s): "
                  f"占有 {100*seg_hit[i]/seg_tot[i]:5.1f} % / ±10cm {100*seg_tol[i]/seg_tot[i]:5.1f} % "
                  f"({seg_tot[i]:,} 点)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
