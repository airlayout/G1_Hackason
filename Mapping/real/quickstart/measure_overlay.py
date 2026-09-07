#!/usr/bin/env python3
"""ライブ点群が事前地図にどれだけ乗っているかを**数値で**出す。

段 6 の「重畳が合っている」は目で見るしかない、と計画書には書いてあるが、
数えれば数字になる。RViz2 のスクリーンショットは角度と拡大率で印象が変わるので、
こちらを合否の根拠にする。

  # 1. 実機（または再生）で /utlidar/cloud_livox_mid360 と /tf を録る
  ros2 bag record -o live_overlay_bag /utlidar/cloud_livox_mid360 /tf /tf_static

  # 2. 測る（Mac 側の venv で。コンテナには numpy/scipy が無い）
  Navigation/.venv/bin/python measure_overlay.py \
      runs/live_overlay_bag runs/<SESSION>/map/nav_map.yaml

やっていること: 各スキャンを、その時刻の `map -> base_link`（MOLA-LO が出す）で
map 系に変換し、壁の高さ帯だけ残して、事前地図の占有セルに当たった割合を数える。

⚠️ **静止中の値は甘い。** 静止中はもともとずれない（2026-09-07 実測で姿勢のばらつき
0.39°）。判定は**歩かせながら録った bag** で行う。
"""
import argparse
import re
import sqlite3
import struct
import sys
import warnings
from pathlib import Path

import numpy as np

# numpy 2.x + Apple Accelerate BLAS は正常な入力でも matmul で警告を投げる（乱数でも再現）
warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation, Slerp

LIVOX_POINT_STEP = 22        # x y z intensity ring time
WALL_Z_MIN, WALL_Z_MAX = -1.2, 0.6   # map 系。床と天井を除いて壁だけ見る
RANGE_MIN, RANGE_MAX = 0.5, 10.0     # 自機の近傍と遠方の外れを除く


class Cdr:
    """rosbag2 の CDR を必要な分だけ読む。"""

    def __init__(self, b: bytes) -> None:
        self.b, self.a = b, 4          # 先頭 4 byte は encapsulation

    def _al(self, n: int) -> None:
        p = (self.a - 4) % n
        if p:
            self.a += n - p

    def u8(self) -> int:
        v = self.b[self.a]; self.a += 1; return v

    def u32(self) -> int:
        self._al(4); v = struct.unpack_from("<I", self.b, self.a)[0]; self.a += 4; return v

    def i32(self) -> int:
        self._al(4); v = struct.unpack_from("<i", self.b, self.a)[0]; self.a += 4; return v

    def f64(self) -> float:
        self._al(8); v = struct.unpack_from("<d", self.b, self.a)[0]; self.a += 8; return v

    def st(self) -> str:
        n = self.u32(); v = self.b[self.a:self.a + n - 1].decode("utf-8", "replace"); self.a += n; return v


def read_map(yaml_path: Path) -> tuple[np.ndarray, float, float, float]:
    meta = dict(re.findall(r"(\w+):\s*(.+)", yaml_path.read_text()))
    res = float(meta["resolution"])
    ox, oy, _ = [float(v) for v in meta["origin"].strip("[]").split(",")]
    with open(yaml_path.parent / meta["image"].strip(), "rb") as f:
        assert f.readline().strip() == b"P5", "pgm ではない"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        f.readline()                                   # maxval
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # ROS の慣習: 画素値が小さいほど占有
    occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
    return occ, res, ox, oy


def read_bag(bag_dir: Path) -> tuple[np.ndarray, list]:
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}

    tfs = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            sec, nsec = r.i32(), r.u32()
            parent, child = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]
            q = [r.f64() for _ in range(4)]
            if parent == "map" and child == "base_link":
                tfs.append([sec + nsec * 1e-9, *tr, *q])

    scans = []
    for (blob,) in con.execute(
        "SELECT data FROM messages WHERE topic_id=?", (tid["/utlidar/cloud_livox_mid360"],)
    ):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        fid = r.st()
        r.u32(); r.u32()                               # height, width
        for _ in range(r.u32()):                       # fields
            r.st(); r.u32(); r.u8(); r.u32()
        r.u8()                                         # is_bigendian
        ps = r.u32(); r.u32(); dl = r.u32()            # point_step, row_step, data len
        # ⚠️ 記録には frame_id='map' / point_step=48 の異物が混ざる（SLAM 点群が
        #    生 LiDAR のトピックに書かれている。2026-09-07 に 5900 件中 229 件で確認）。
        #    frame_id と point_step の両方で弾く。
        if fid != "livox_frame" or ps != LIVOX_POINT_STEP:
            continue
        raw = np.frombuffer(blob, dtype=np.uint8, count=dl, offset=r.a)
        xyz = raw.reshape(-1, LIVOX_POINT_STEP)[:, :12].copy().view(np.float32).reshape(-1, 3)
        scans.append((sec + nsec * 1e-9, xyz))
    return np.array(sorted(tfs)), scans


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--tol-cells", type=int, default=1, help="何セルまで許すか（既定 1 = 10 cm）")
    a = ap.parse_args()

    occ, res, ox, oy = read_map(a.map_yaml)
    h, w = occ.shape
    print(f"事前地図 {w}x{h} セル / res {res} m / origin ({ox}, {oy}) / 占有 {occ.sum()} セル")

    tfs, scans = read_bag(a.bag)
    if len(tfs) < 2 or not scans:
        print("map->base_link の /tf か生 LiDAR が足りない", file=sys.stderr)
        return 1
    print(f"map->base_link の /tf {len(tfs)} 件 / 生 LiDAR {len(scans)} 枚")

    tt = tfs[:, 0]
    slerp = Slerp(tt, Rotation.from_quat(tfs[:, 4:8]))
    k = 2 * a.tol_cells + 1
    occ_tol = binary_dilation(occ, np.ones((k, k), bool))

    tot = hit = hit_tol = 0
    used = 0
    for ts, p in scans:
        if not (tt[0] <= ts <= tt[-1]):
            continue
        used += 1
        p = p[np.isfinite(p).all(1)]
        d = np.linalg.norm(p, axis=1)
        p = p[(d > RANGE_MIN) & (d < RANGE_MAX)]
        t = np.array([np.interp(ts, tt, tfs[:, 1 + i]) for i in range(3)])
        q = (slerp(ts).as_matrix() @ p.T).T + t
        q = q[(q[:, 2] > WALL_Z_MIN) & (q[:, 2] < WALL_Z_MAX)]
        ix = np.floor((q[:, 0] - ox) / res).astype(int)
        iy = np.floor((q[:, 1] - oy) / res).astype(int)
        m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[m], iy[m]
        row = h - 1 - iy                               # pgm は上下反転
        tot += len(ix)
        hit += int(occ[row, ix].sum())
        hit_tol += int(occ_tol[row, ix].sum())

    if not tot:
        print("地図の範囲に落ちた点が無い。初期姿勢か地図が合っていない", file=sys.stderr)
        return 1
    print(f"\n=== 重畳（ライブ点群 -> 事前地図の占有セル）===")
    print(f"  使ったスキャン {used} 枚 / 評価した点 {tot:,}")
    print(f"  占有セルに乗った割合                  {100*hit/tot:.1f} %")
    print(f"  ±{a.tol_cells} セル({a.tol_cells*res*100:.0f} cm)まで許した割合  {100*hit_tol/tot:.1f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
