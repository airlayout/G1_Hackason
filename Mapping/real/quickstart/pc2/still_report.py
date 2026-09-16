#!/usr/bin/env python3
"""**静止記録 1 本から、測位候補を同じ物差しで採点する。**

## なぜ要るのか

候補ごとに違う測り方をしたら比較にならない。段 A（MOLA）・段 B（AMCL）・
段 C（FAST_LIO_LOCALIZATION）は出す物が全部違うが、
**`map` から `base_link` までの鎖を出す**という 1 点だけは共通なので、そこだけを見る。

⚠️ **鎖の形は候補ごとに違う。** 直接 `map -> base_link` を出すのは MOLA だけで、

    MOLA        map -> base_link
    AMCL        map -> odom -> base_link
    FAST_LIO    map -> odom -> camera_init -> body -> base_link   （静的 2 本を含む）

直接のものだけを見ると「この候補は測位を出していない」と誤報する（2026-09-15 に 2 度踏んだ）。
だから **frame のグラフを組んで BFS で辿る**。

## 出す量（計画書 2026-09-12_2 の M1/M3/M4 に対応）

| 量 | 定義 | 合否 |
|---|---|---|
| 滑り | 端から端の変位 [m]。**機体は静止しているので真値は 0** | < 0.05 m |
| 震え | 0.5 s 窓の見かけの速さの中央値の最大 [m/s] | < 0.10 m/s |
| 上限超 | 見かけの速さが 0.361 m/s（velocity_smoother の hypot）を超えた割合 | 0 % |
| 初期姿勢からのずれ | 与えた初期姿勢との距離 [m] | < 0.30 m |
| 重畳 | 事前地図の占有セルに乗った割合。**既定帯(許容0)と壁の帯(許容±1)の両方** | 壁の帯 > 80 % |

⚠️ 重畳は 1 つの数字で判断しない。既定帯は床の反射を「外れ」と数えるので、
同じ姿勢が既定 44.4% / 壁の帯 91.9% に割れる（2026-09-15 実測）。
⚠️ **この部屋のこの場所では、2D 重畳は姿勢を 2 m より細かく決められない**
（種のまわり ±4 m を掃くと 96% 台が 2.4 m の幅で並ぶ）。順位づけの審判には使えない。
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import sys
from collections import deque
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from measure_overlay import Cdr, read_bag, read_map  # noqa: E402

SPEED_LIMIT = 0.361          # velocity_smoother の hypot（計画書 M3）
WALL_LO, WALL_HI = 1.30, 1.82
DEF_LO, DEF_HI = 0.02, 1.82
# live の取付値。記録に base_link->livox_frame が無い候補のための既定
# ⚠️ 2026-09-16 に床基準へ定義し直した（旧 (0,0,1.228)/(177.93,3.32,0) は内蔵 odom 基準で
#    base_link が床から 5.90 度傾く）。詳細は dog_odom_to_tf.py の注記。
LIVOX_XYZ = (-0.112579, -0.057151, 1.209672)
LIVOX_RPY_DEG = (-179.401, -1.9437, 0.0321)


def read_edges(bag_dir: Path) -> dict[tuple[str, str], np.ndarray]:
    """`/tf` と `/tf_static` の全部の辺を (parent, child) -> (N, 8) で返す。"""
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    seen: dict[tuple[str, str], list] = {}
    for topic in ("/tf", "/tf_static"):
        if topic not in tid:
            continue
        for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
            r = Cdr(blob)
            for _ in range(r.u32()):
                sec, nsec = r.i32(), r.u32()
                parent, child = r.st(), r.st()
                tr = [r.f64() for _ in range(3)]
                q = [r.f64() for _ in range(4)]
                seen.setdefault((parent, child), []).append([sec + nsec * 1e-9, *tr, *q])
    return {k: np.array(sorted(v)) for k, v in seen.items() if v}


def find_path(edges, src: str, dst: str) -> list[tuple[tuple[str, str], bool]] | None:
    """src -> dst の辺の並びを返す。bool は「辺を逆向きに使うか」。"""
    adj: dict[str, list] = {}
    for (p, c) in edges:
        adj.setdefault(p, []).append((c, (p, c), False))
        adj.setdefault(c, []).append((p, (p, c), True))
    prev: dict[str, tuple] = {src: ()}
    q = deque([src])
    while q:
        f = q.popleft()
        if f == dst:
            break
        for nxt, key, inv in adj.get(f, []):
            if nxt not in prev:
                prev[nxt] = (f, key, inv)
                q.append(nxt)
    if dst not in prev:
        return None
    out, f = [], dst
    while prev[f]:
        f, key, inv = prev[f]
        out.append((key, inv))
    return list(reversed(out))


def chain(edges, path, times) -> np.ndarray:
    """path に沿って合成し、各時刻の map->base_link を (N, 8) で返す。"""
    out = []
    for t in times:
        R, tr = Rotation.identity(), np.zeros(3)
        for key, inv in path:
            a = edges[key]
            k = int(np.argmin(np.abs(a[:, 0] - t))) if len(a) > 1 else 0
            Re, te = Rotation.from_quat(a[k, 4:8]), a[k, 1:4]
            if inv:
                Re, te = Re.inv(), -Re.inv().apply(te)
            tr = R.apply(te) + tr
            R = R * Re
        out.append([t, *tr, *R.as_quat()])
    return np.array(out)


def map_to_base_link(bag_dir: Path) -> tuple[np.ndarray | None, str]:
    edges = read_edges(bag_dir)
    path = find_path(edges, "map", "base_link")
    if path is None:
        return None, "map から base_link へ辿れない"
    # 一番よく動く辺の時刻を使う
    dyn = max((edges[k] for k, _ in path), key=len)
    if len(dyn) < 2:
        return None, "動的な辺が無い"
    desc = " -> ".join(["map"] + [(k[0] if inv else k[1]) for k, inv in path])
    return chain(edges, path, dyn[:, 0]), desc


def overlay(points, pose, occ, res, ox, oy, lo, hi, tol_cells):
    x, y, yaw_deg = pose
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    p = points[(points[:, 2] >= lo) & (points[:, 2] <= hi)]
    if len(p) == 0:
        return None
    wx = x + c * p[:, 0] - s * p[:, 1]
    wy = y + s * p[:, 0] + c * p[:, 1]
    col = np.floor((wx - ox) / res).astype(int)
    iy = np.floor((wy - oy) / res).astype(int)
    h, w = occ.shape
    ins = (col >= 0) & (col < w) & (iy >= 0) & (iy < h)
    if not ins.any():
        return None
    # ⚠️ read_map は PGM を反転せずに返す。行は h-1-iy（2026-09-11 にここを落とした）
    row = h - 1 - iy
    g = occ if tol_cells == 0 else binary_dilation(occ, np.ones((2 * tol_cells + 1,) * 2, bool))
    return 100.0 * g[row[ins], col[ins]].sum() / int(ins.sum())


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--ref", type=Path, required=True)
    ap.add_argument("--init", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW"))
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    _, scans, sensor_tf = read_bag(a.bag)
    tfs, desc = map_to_base_link(a.bag)
    if tfs is None or not scans:
        print(f"⛔ {desc}（この候補は測位を出していない）")
        return 3

    t = tfs[:, 0] - tfs[0, 0]
    xy = tfs[:, 1:3]
    drift = float(np.hypot(*(xy[-1] - xy[0])))
    dyaw = yaw_of(tfs[-1, 4:8]) - yaw_of(tfs[0, 4:8])

    dt = np.diff(t)
    ok = dt > 1e-3
    tv = t[1:][ok]
    v = np.hypot(*np.diff(xy, axis=0).T)[ok] / dt[ok]
    over = 100.0 * float((v > SPEED_LIMIT).mean()) if len(v) else 0.0
    tremble = 0.0
    for i in range(len(v)):
        m = (tv >= tv[i]) & (tv < tv[i] + 0.5)
        if m.sum() >= 3:
            tremble = max(tremble, float(np.median(v[m])))

    occ, res, ox, oy = read_map(a.ref)[:4]
    ts, raw = scans[len(scans) // 2]
    k = int(np.argmin(np.abs(tfs[:, 0] - ts)))
    pose = (tfs[k, 1], tfs[k, 2], yaw_of(tfs[k, 4:8]))
    p = raw[np.isfinite(raw).all(1)]
    if sensor_tf is not None:
        s_t, s_q = sensor_tf
        p = p @ Rotation.from_quat(s_q).as_matrix().T + s_t
        stf = "記録の /tf_static"
    else:
        # ⚠️ 記録に base_link->livox_frame が無い候補（FAST_LIO は imu_link で組む）。
        #    恒等で済ませると逆さま付けを落として数字だけ下がるので、live の取付値を使う
        R = Rotation.from_euler("xyz", np.radians(LIVOX_RPY_DEG)).as_matrix()
        p = p @ R.T + np.array(LIVOX_XYZ)
        stf = "live の取付値（記録に base_link->livox_frame が無い）"
    ov_def = overlay(p, pose, occ, res, ox, oy, DEF_LO, DEF_HI, 0)
    ov_wall = overlay(p, pose, occ, res, ox, oy, WALL_LO, WALL_HI, 1)

    print(f"=== {a.label or a.bag.name} ===")
    print(f"  鎖 {desc}")
    print(f"  記録 {t[-1]:.1f} s / 姿勢 {len(tfs)} 件 / スキャン {len(scans)} 枚 / 取付 {stf}")
    print(f"  滑り（端から端）        {drift:7.3f} m   （合否 < 0.05）{'✅' if drift < 0.05 else '❌'}")
    print(f"  震え（0.5 s 窓の最大）  {tremble:7.3f} m/s （合否 < 0.10）{'✅' if tremble < 0.10 else '❌'}")
    print(f"  上限 0.361 m/s 超       {over:7.1f} %   （合否 0）{'✅' if over == 0 else '❌'}")
    print(f"  yaw 変化                {dyaw:+7.2f} deg")
    if a.init:
        d0 = math.hypot(pose[0] - a.init[0], pose[1] - a.init[1])
        print(f"  初期姿勢からのずれ      {d0:7.3f} m   （合否 < 0.30）{'✅' if d0 < 0.30 else '❌'}")
    print(f"  姿勢（中間スキャン時）  ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.2f} deg)")
    print(f"  重畳 既定帯 許容0       {ov_def:7.1f} %")
    print(f"  重畳 壁の帯 許容±1      {ov_wall:7.1f} %   （合否 > 80）"
          f"{'✅' if ov_wall and ov_wall > 80 else '❌'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
