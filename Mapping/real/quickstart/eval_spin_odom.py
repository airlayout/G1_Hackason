#!/usr/bin/env python3
"""旋回ベンチの bag で、純正の脚 odometry (/dog_odom) と MOLA (map->base_link) を比べる。

問い: **その場旋回中に脚 odom は並進を捏造しないか。**
（MOLA は 2.5 m 捏造する。脚 odom が捏造しないなら、それが「並進していない」という事前情報になる）

    Navigation/.venv/bin/python quickstart/eval_spin_odom.py runs/spin_XXXX/bag [...]

⚠️ 2026-09-12 時点で /dog_odom 入りの旋回 bag はまだ無い（record_topics.txt に入れたのが
   旋回ベンチ 2 本のあと）。Odometry の CDR 読みは静止 8 秒の bag（7985 件）で検証済み。

出すもの（旋回区間 = MOLA の |ω_yaw| > 0.15 rad/s）:
  - 脚 odom の xy 逸脱の最大 / 終端（開始点からの距離）
  - 脚 odom の yaw 総回転（ジャイロ積分 ≈ 200° と比べる）
  - MOLA の同じ量（対照）
  - 脚 odom の xy が描く円の半径 ≈ robot_center と回転軸のずれ（レバーアーム）
"""
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import Cdr


def read_odom(con, tid, topic):
    if topic not in tid:
        return None
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32(); r.st(); r.st()
        p = [r.f64() for _ in range(3)]; q = [r.f64() for _ in range(4)]
        [r.f64() for _ in range(36)]
        v = [r.f64() for _ in range(3)]; w = [r.f64() for _ in range(3)]
        rows.append([sec + nsec * 1e-9] + p + q + v + w)
    return np.array(sorted(rows)) if rows else None


def read_tf(con, tid):
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            sec, nsec = r.i32(), r.u32(); par, ch = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]; q = [r.f64() for _ in range(4)]
            if par == "map" and ch == "base_link":
                rows.append([sec + nsec * 1e-9] + tr + q)
    return np.array(sorted(rows))


def summarize(name, a):
    """a: [t, x, y, z, qx, qy, qz, qw, ...]"""
    t = a[:, 0] - a[0, 0]
    d = np.hypot(a[:, 1] - a[0, 1], a[:, 2] - a[0, 2])
    yaw = np.unwrap(Rotation.from_quat(a[:, 4:8]).as_euler("xyz")[:, 2])
    rate = np.abs(np.gradient(yaw, a[:, 0]))
    turning = rate > 0.15
    # 旋回区間だけの xy 逸脱（旋回前の静止で基準を取り直す）
    if turning.any():
        i0 = int(np.argmax(turning))
        d_turn = np.hypot(a[turning, 1] - a[i0, 1], a[turning, 2] - a[i0, 2])
        # レバーアーム: 旋回中の xy を円でフィット（最小二乗）
        X, Y = a[turning, 1], a[turning, 2]
        A = np.c_[2 * X, 2 * Y, np.ones_like(X)]
        cx, cy, c = np.linalg.lstsq(A, X**2 + Y**2, rcond=None)[0]
        radius = math.sqrt(max(c + cx**2 + cy**2, 0.0))
    else:
        d_turn, radius = np.array([0.0]), float("nan")
    print("  {:<12} 件 {:>5}  xy逸脱 最大 {:.3f} m / 終端 {:.3f} m   旋回中の逸脱最大 {:.3f} m   "
          "yaw総回転 {:6.1f}°   旋回中xyの円半径(レバーアーム) {:.3f} m".format(
              name, len(a), d.max(), d[-1], d_turn.max(), math.degrees(abs(yaw[-1] - yaw[0])), radius))


def main():
    for arg in sys.argv[1:]:
        db = next(Path(arg).glob("*.db3"))
        con = sqlite3.connect("file:{}?mode=ro".format(db), uri=True)
        tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
        print("\n=== {} ===".format(Path(arg).parent.name))
        od = read_odom(con, tid, "/dog_odom")
        if od is None:
            print("  /dog_odom が記録に無い（record_topics.txt に入れる前の記録）")
        else:
            summarize("脚 odom", od)
        summarize("MOLA", read_tf(con, tid))


if __name__ == "__main__":
    main()
