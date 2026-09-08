#!/usr/bin/env python3
"""G1（測位器の補正の跳び）を**実機の記録**に当てて、誤報が出ないかを測る。

## なぜこれが要るか

完璧な測位（map -> odom を恒等）の記録で G1 が鳴らないのは当たり前で、
**誤報の検査になっていない**（構造上ぴったり恒等なので跳びが起きない）。
本当に要るのは「実際の測位器が正常に動いている記録で鳴らないか」。

この記録（20260906T135940_UiS_room_v3）は MOLA-LO が真値から
0.023〜0.030 m で追えている＝**測位が正しかった**記録なので、
ここで G1 が鳴れば誤報。

## 何と何を比べるか

実機で G1 が見るのは map -> odom の跳び。これは
    map -> odom = (map -> base) ∘ (odom -> base)^-1
なので、独立な 2 本が要る:

- `map -> base` … MOLA-LO の姿勢（mola_floor0/traj.txt、TUM）
- `odom -> base` … 内蔵 SLAM のオドメトリ（/unitree/slam_mapping/odom）

⚠️ 内蔵 SLAM の `frame_id` は "map" と書いてあるが**実体は odom**
（ロボット起動時原点の相対姿勢）。信じると座標系を間違える。

使い方:
    python3 eval_guard_real.py --run <runs>/20260906T135940_UiS_room_v3
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))

# eval_guards.py と同じ定数を使う（別の値でごまかさない）
G1_WINDOW_M = 2.0
G1_TRANS_M = 0.30
G1_ROT_DEG = 10.0

ODOM_TOPIC = "/unitree/slam_mapping/odom"


class Cdr:
    """rosbag2 の CDR を必要な分だけ読む（measure_overlay.py と同じ実装）。"""

    def __init__(self, b: bytes) -> None:
        self.b, self.a = b, 4

    def _al(self, n: int) -> None:
        p = (self.a - 4) % n
        if p:
            self.a += n - p

    def u32(self) -> int:
        import struct
        self._al(4); v = struct.unpack_from("<I", self.b, self.a)[0]
        self.a += 4; return v

    def i32(self) -> int:
        import struct
        self._al(4); v = struct.unpack_from("<i", self.b, self.a)[0]
        self.a += 4; return v

    def f64(self) -> float:
        import struct
        self._al(8); v = struct.unpack_from("<d", self.b, self.a)[0]
        self.a += 8; return v

    def st(self) -> str:
        n = self.u32()
        v = self.b[self.a:self.a + n - 1].decode("utf-8", "replace")
        self.a += n
        return v


def read_odom(bag_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """nav_msgs/Odometry を読む。戻りは (時刻, xyz, quat[xyzw], frame_id)。"""
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    if ODOM_TOPIC not in tid:
        raise SystemExit(f"[NG] {ODOM_TOPIC} が bag に無い: {sorted(tid)}")
    ts, xyz, quat = [], [], []
    frame = ""
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?",
                               (tid[ODOM_TOPIC],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        frame = r.st()          # ⚠️ "map" と書いてあるが実体は odom
        r.st()                  # child_frame_id
        px, py, pz = r.f64(), r.f64(), r.f64()
        qx, qy, qz, qw = r.f64(), r.f64(), r.f64(), r.f64()
        ts.append(sec + nsec * 1e-9)
        xyz.append((px, py, pz))
        quat.append((qx, qy, qz, qw))
    con.close()
    return np.array(ts), np.array(xyz), np.array(quat), frame


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a = np.loadtxt(path)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def se2(xyz: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """xy と yaw だけ取る。門番は平面の跳びを見る。"""
    yaw = Rotation.from_quat(quat).as_euler("xyz")[:, 2]
    return np.column_stack([xyz[:, 0], xyz[:, 1], yaw])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compose_inv(mb: np.ndarray, ob: np.ndarray) -> np.ndarray:
    """T_mo = T_mb ∘ T_ob^-1。行ごとに計算する。"""
    dth = wrap(mb[:, 2] - ob[:, 2])
    c, s = np.cos(dth), np.sin(dth)
    return np.column_stack([
        mb[:, 0] - (c * ob[:, 0] - s * ob[:, 1]),
        mb[:, 1] - (s * ob[:, 0] + c * ob[:, 1]),
        dth,
    ])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--traj", default="mola_floor0/traj.txt")
    args = ap.parse_args()
    run = Path(args.run)

    ot, oxyz, oq, oframe = read_odom(run / "raw/rosbag2")
    print(f"[odom] {ODOM_TOPIC}: {len(ot)} 件 / "
          f"{ot[-1] - ot[0]:.1f} s / frame_id={oframe!r}"
          f"（⚠️ 実体は odom）")
    mt, mxyz, mq = read_tum(run / args.traj)
    print(f"[mola] {args.traj}: {len(mt)} 件 / {mt[-1] - mt[0]:.1f} s")

    lo, hi = max(ot[0], mt[0]), min(ot[-1], mt[-1])
    print(f"[共通の時間帯] {hi - lo:.1f} s")
    if hi - lo < 10.0:
        raise SystemExit("[NG] 共通の時間帯が短すぎる")

    # MOLA の時刻に odom を合わせる（odom の方が細かいので内挿でなく最近傍）
    keep = (mt >= lo) & (mt <= hi)
    mt, mb = mt[keep], se2(mxyz[keep], mq[keep])
    idx = np.searchsorted(ot, mt).clip(1, len(ot) - 1)
    pick = np.where(np.abs(ot[idx] - mt) < np.abs(ot[idx - 1] - mt), idx, idx - 1)
    gap = np.abs(ot[pick] - mt)
    print(f"[突き合わせ] 時刻の差 中央 {np.median(gap) * 1000:.1f} ms / "
          f"最大 {gap.max() * 1000:.1f} ms")
    ob = se2(oxyz[pick], oq[pick])

    mo = compose_inv(mb, ob)
    # odom の走行距離（実機で積算できる量）
    step = np.hypot(np.diff(ob[:, 0]), np.diff(ob[:, 1]))
    dist = np.concatenate([[0.0], np.cumsum(step)])
    print(f"[走行] odom {dist[-1]:.2f} m / MOLA "
          f"{np.hypot(np.diff(mb[:,0]), np.diff(mb[:,1])).sum():.2f} m")

    # G1 を全区間に当てる。窓 G1_WINDOW_M ごとの map->odom の跳び
    j = np.searchsorted(dist, dist - G1_WINDOW_M, side="right") - 1
    valid = (j >= 0) & (dist - dist[j.clip(0)] >= G1_WINDOW_M)
    dt = np.hypot(mo[:, 0] - mo[j.clip(0), 0], mo[:, 1] - mo[j.clip(0), 1])
    dr = np.abs(np.degrees(wrap(mo[:, 2] - mo[j.clip(0), 2])))
    dt, dr = dt[valid], dr[valid]
    print(f"[G1 の観測量] 有効な窓 {valid.sum()} 個")
    print(f"   並進の跳び: 中央 {np.median(dt):.3f} m / 95% "
          f"{np.percentile(dt, 95):.3f} m / 最大 {dt.max():.3f} m "
          f"（しきい {G1_TRANS_M} m）")
    print(f"   回転の跳び: 中央 {np.median(dr):.2f} 度 / 95% "
          f"{np.percentile(dr, 95):.2f} 度 / 最大 {dr.max():.2f} 度 "
          f"（しきい {G1_ROT_DEG} 度）")
    n_trip = int(((dt > G1_TRANS_M) | (dr > G1_ROT_DEG)).sum())
    print(f"\n== 誤報 {n_trip} / {valid.sum()} 窓 "
          f"({'PASS: 鳴らなかった' if n_trip == 0 else 'FAIL: 鳴った'}) ==")
    if n_trip == 0:
        margin_t = G1_TRANS_M / max(dt.max(), 1e-9)
        margin_r = G1_ROT_DEG / max(dr.max(), 1e-9)
        print(f"   余裕: 並進 {margin_t:.1f} 倍 / 回転 {margin_r:.1f} 倍")
    return 0 if n_trip == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
