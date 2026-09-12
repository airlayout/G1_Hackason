#!/usr/bin/env python3
"""旋回ベンチの bag で、純正の脚 odometry (/dog_odom) と MOLA (map->base_link) を比べる。

問い: **その場旋回中に脚 odom は並進を捏造しないか。**
（MOLA は 2.5 m 捏造する。脚 odom が捏造しないなら、それが「並進していない」という事前情報になる）

    Navigation/.venv/bin/python quickstart/eval_spin_odom.py runs/spin_XXXX/bag [...]

⚠️ 2026-09-12 時点で /dog_odom 入りの旋回 bag はまだ無い（record_topics.txt に入れたのが
   旋回ベンチ 2 本のあと）。Odometry の CDR 読みは静止 8 秒の bag（7985 件）で検証済み。

⚠️ **「開始点からの xy 逸脱」で捏造を測ってはいけない（2026-09-12 に踏んだ）。**
`robot_center`（骨盤）も `base_link` も**回転の中心ではない**ので、その場旋回でも
幾何的に半径 r の円を描き、逸脱は最大 2r まで出る。これは本物の運動で捏造ではない。

**捏造は「円の中心が動いたか」で測る。** その場で回るなら中心は止まっているはず。
往路と復路で別々に円を当て、中心の差を見る（`RETURN=1` で録った場合）。
円からの残差 RMS は「円運動としてどれだけ素直か」。

出すもの:
  - 往路 / 復路それぞれの円の中心・半径・残差 RMS
  - **中心の移動量**（往路の中心 → 復路の中心）← これが捏造の量
  - 終端の逸脱（出て戻ったあとに開始点へ帰れたか）
  - yaw の振幅（peak-to-peak。net ではない。RETURN=1 では net ≈ 0 になる）
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


def fit_circle(x, y):
    """最小二乗で円を当てる。(cx, cy, r, 残差RMS) を返す。"""
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    cx, cy, c = np.linalg.lstsq(A, x**2 + y**2, rcond=None)[0]
    r = math.sqrt(max(c + cx**2 + cy**2, 0.0))
    res = np.hypot(x - cx, y - cy) - r
    return cx, cy, r, float(np.sqrt((res**2).mean()))


def summarize(name, a):
    """a: [t, x, y, z, qx, qy, qz, qw, ...]"""
    t = a[:, 0] - a[0, 0]
    d = np.hypot(a[:, 1] - a[0, 1], a[:, 2] - a[0, 2])
    yaw = np.unwrap(Rotation.from_quat(a[:, 4:8]).as_euler("xyz")[:, 2])
    rate = np.abs(np.gradient(yaw, a[:, 0]))
    turning = rate > 0.15

    # 往路と復路は yaw が最も離れた点で分ける（RETURN=1 で戻ってくる作り）
    turn = int(np.argmax(np.abs(yaw - yaw[0])))
    legs = []
    for lbl, sl in (("往路", slice(0, turn + 1)), ("復路", slice(turn, len(a)))):
        m = turning[sl]
        if m.sum() < 20:
            continue
        X, Y = a[sl][m, 1], a[sl][m, 2]
        legs.append((lbl,) + fit_circle(X, Y))

    print("  【{}】 件 {}  yaw 振幅 {:.1f}°（net {:+.1f}°）  終端の逸脱 {:.3f} m  逸脱最大 {:.3f} m".format(
        name, len(a), math.degrees(np.ptp(yaw)), math.degrees(yaw[-1] - yaw[0]), d[-1], d.max()))
    for lbl, cx, cy, r, rms in legs:
        print("    {} 円の中心 ({:+.3f}, {:+.3f})  半径 {:.3f} m  残差RMS {:.3f} m".format(
            lbl, cx, cy, r, rms))
    if len(legs) == 2:
        shift = math.hypot(legs[0][1] - legs[1][1], legs[0][2] - legs[1][2])
        print("    ★ 中心の移動 {:.3f} m  ← その場旋回なら 0 のはず（捏造の量）".format(shift))


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
