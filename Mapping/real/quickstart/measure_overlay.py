#!/usr/bin/env python3
"""ライブ点群が事前地図にどれだけ乗っているかを**数値で**出す。

段 6 の「重畳が合っている」は目で見るしかない、と計画書には書いてあるが、
数えれば数字になる。RViz2 のスクリーンショットは角度と拡大率で印象が変わるので、
こちらを合否の根拠にする。

  # 1. 実機（または再生）で /utlidar/cloud_livox_mid360 と /tf を録る
  #    記録トピックは quickstart/record_topics.txt が定義する（直書きしない）
  TOPICS="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' quickstart/record_topics.txt)"
  ros2 bag record -o live_overlay_bag ${TOPICS}

  # 2. 測る（Mac 側の venv で。コンテナには numpy/scipy が無い）
  Navigation/.venv/bin/python measure_overlay.py \
      runs/live_overlay_bag runs/<SESSION>/map/nav_map_ref.yaml

⚠️ **基準は間引いていない nav_map_ref を渡す。**Nav2 が走る nav_map_clean は
机を落としてあるので、静止の対照でも 43.9% が天井になり合格線 85% が引けない
（同じ記録が間引いていない地図では 78.2%。2026-09-11 実測）。

`nav_map_ref` は `make_ref_map.py` が MOLA 自身の地図（`mola_floor0/map_full.mm`）から作る。
2026-09-12 まで基準は 09-06 の OctoMap 由来の旧 `nav_map`（`map/old/`）で、
**地図を作り直しても付いてこない別系統**だった（計画書 段 13）。

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
# 壁だけ見るための高さ帯。**LiDAR の高さからの相対**で持つ（map の z 原点に依存させない）。
#
# ⚠️ 以前は map 系の絶対値 -1.2 .. 0.6 で焼いてあった。これは `mola_aligned` の地図
# （map の z 原点がセンサ高さ = 床が z≈-1.23）専用の値で、`mola_floor0` の地図
# （床が z≈0 / センサが z≈+1.23）で使うと**帯が床より下に来て点がほぼ全部落ちる。**
# 2026-09-09 に実機で踏んだ: 245 枚のスキャンで評価点が 6,021 点しか残らず、
# 重畳が 44.5%（既知の静止値 89.2%）と出た。**地図も測位も正しかった。**
# センサ高さを基準にすれば、どちらの地図でも同じ「床の上・天井の下」を見る。
WALL_Z_BELOW_SENSOR, WALL_Z_ABOVE_SENSOR = -1.2, 0.6
RANGE_MIN, RANGE_MAX = 0.5, 10.0     # 自機の近傍と遠方の外れを除く（センサ系）


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


def read_static_sensor_tf(con, tid) -> tuple[np.ndarray, np.ndarray] | None:
    """/tf_static の base_link -> livox_frame。無ければ None（= base_link が LiDAR 系）。

    ⚠️ **これを掛け忘れると静かに間違う。** 生 LiDAR の点は `livox_frame` に居るので、
    `map -> base_link` だけで変換すると G1 では取付の roll 178°（LiDAR は上下逆さま）が
    抜けたまま地図に重ねることになる。落ちるのではなく重畳の数字だけが下がるので、
    「測位がずれた」と読み違える（2026-09-09 に実機で 1 度読み違えた）。
    """
    if "/tf_static" not in tid:
        return None
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf_static"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            r.i32(); r.u32()
            parent, child = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]
            q = [r.f64() for _ in range(4)]
            if parent == "base_link" and child == "livox_frame":
                return np.array(tr), np.array(q)
    return None


def read_bag(bag_dir: Path) -> tuple[np.ndarray, list, tuple | None]:
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}

    sensor_tf = read_static_sensor_tf(con, tid)
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
    return np.array(sorted(tfs)), scans, sensor_tf


def world_points(bag_dir: Path, wall_z: tuple[float, float] | None = None,
                 verbose: bool = True) -> tuple[np.ndarray, int, float, float]:
    """記録の各スキャンを map 系に起こし、**壁の高さ帯だけ**を (N, 2) で返す。

    重畳を数えるのも、ずらして最良を探すのも、入口はここ 1 つにする。
    **ここが 2 つあると、片方だけ静的変換を掛け忘れても数字が下がるだけで落ちない**
    （2026-09-09 に 1 度読み違えた型）。

    戻り値: (map 系の xy, 使ったスキャン数, 帯の下限, 帯の上限)。
    帯は既定では LiDAR 高さからの相対なのでスキャンごとに動く。返すのは最後の 1 枚の値。
    """
    tfs, scans, sensor_tf = read_bag(bag_dir)
    if len(tfs) < 2 or not scans:
        raise ValueError("map->base_link の /tf か生 LiDAR が足りない")
    if verbose:
        print(f"map->base_link の /tf {len(tfs)} 件 / 生 LiDAR {len(scans)} 枚")

    # base_link -> livox_frame。無い記録は「base_link が LiDAR 系」の構成なので恒等でよい
    if sensor_tf is None:
        s_t = np.zeros(3)
        s_R = np.eye(3)
        if verbose:
            print("base_link -> livox_frame: /tf_static に無いので恒等とみなす"
                  "（ignore_lidar_pose_from_tf:=true の構成）")
    else:
        s_t, s_q = sensor_tf
        s_R = Rotation.from_quat(s_q).as_matrix()
        if verbose:
            rpy = Rotation.from_quat(s_q).as_euler("ZYX", degrees=True)[::-1]
            print(f"base_link -> livox_frame: xyz {np.round(s_t, 4).tolist()} / "
                  f"rpy {np.round(rpy, 2).tolist()} deg")

    tt = tfs[:, 0]
    slerp = Slerp(tt, Rotation.from_quat(tfs[:, 4:8]))
    out = []
    used = 0
    z_lo = z_hi = float("nan")
    for ts, p in scans:
        if not (tt[0] <= ts <= tt[-1]):
            continue
        used += 1
        p = p[np.isfinite(p).all(1)]
        d = np.linalg.norm(p, axis=1)
        p = p[(d > RANGE_MIN) & (d < RANGE_MAX)]
        t = np.array([np.interp(ts, tt, tfs[:, 1 + i]) for i in range(3)])
        R = slerp(ts).as_matrix()
        # livox_frame -> base_link -> map。**静的変換を先に掛ける**
        q = (R @ (s_R @ p.T + s_t[:, None])).T + t
        if wall_z is not None:
            z_lo, z_hi = wall_z
        else:
            sensor_z = float((R @ s_t + t)[2])   # map 系での LiDAR の高さ
            z_lo = sensor_z + WALL_Z_BELOW_SENSOR
            z_hi = sensor_z + WALL_Z_ABOVE_SENSOR
        q = q[(q[:, 2] > z_lo) & (q[:, 2] < z_hi)]
        out.append(q[:, :2])
    if not out:
        raise ValueError("帯に残った点が無い")
    return np.vstack(out), used, z_lo, z_hi


def count_hits(xy: np.ndarray, occ: np.ndarray, res: float, ox: float, oy: float,
               dx: float = 0.0, dy: float = 0.0) -> tuple[int, int]:
    """(占有セルに乗った点数, 地図の範囲に落ちた点数)。(dx, dy) だけずらして数える。"""
    h, w = occ.shape
    ix = np.floor((xy[:, 0] + dx - ox) / res).astype(int)
    iy = np.floor((xy[:, 1] + dy - oy) / res).astype(int)
    m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    ix, iy = ix[m], iy[m]
    row = h - 1 - iy                                   # pgm は上下反転
    return int(occ[row, ix].sum()), int(m.sum())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("--tol-cells", type=int, default=1, help="何セルまで許すか（既定 1 = 10 cm）")
    ap.add_argument("--wall-z", nargs=2, type=float, metavar=("MIN", "MAX"), default=None,
                    help="壁とみなす高さ帯を **map 系の絶対値** で指定する。"
                         "既定は LiDAR 高さからの相対 "
                         f"({WALL_Z_BELOW_SENSOR:+.2f} .. {WALL_Z_ABOVE_SENSOR:+.2f} m) で、"
                         "地図の z 原点（aligned / floor0）に依らず同じ帯を見る")
    a = ap.parse_args()

    occ, res, ox, oy = read_map(a.map_yaml)
    h, w = occ.shape
    # ⚠️ **どの地図で測ったかを必ず残す。**走る地図（nav_map_clean）と基準地図
    # （間引いていない nav_map_ref）が別なので、数字だけ見ると取り違える
    print(f"基準地図 {a.map_yaml}")
    print(f"事前地図 {w}x{h} セル / res {res} m / origin ({ox}, {oy}) / 占有 {occ.sum()} セル")

    try:
        xy, used, z_lo, z_hi = world_points(a.bag, a.wall_z)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    k = 2 * a.tol_cells + 1
    occ_tol = binary_dilation(occ, np.ones((k, k), bool))
    hit, tot = count_hits(xy, occ, res, ox, oy)
    hit_tol, _ = count_hits(xy, occ_tol, res, ox, oy)

    if not tot:
        print("地図の範囲に落ちた点が無い。初期姿勢か地図が合っていない", file=sys.stderr)
        return 1
    print(f"\n=== 重畳（ライブ点群 -> 事前地図の占有セル）===")
    print(f"  壁とみなした高さ帯（map 系）          {z_lo:+.2f} .. {z_hi:+.2f} m")
    print(f"  使ったスキャン {used} 枚 / 評価した点 {tot:,}")
    print(f"  占有セルに乗った割合                  {100*hit/tot:.1f} %")
    print(f"  ±{a.tol_cells} セル({a.tol_cells*res*100:.0f} cm)まで許した割合  {100*hit_tol/tot:.1f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
