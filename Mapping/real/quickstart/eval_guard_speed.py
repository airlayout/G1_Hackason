#!/usr/bin/env python3
"""門番 G0（**物理的妥当性**）を記録に当てて、誤報率と検知の遅れを測る。

## なぜこの門番なのか

2026-09-10 の実機 3 本は全部、逸脱ガード（距離 1.50 m）で中止した。
翌日に原因を追って分かったのは「**変位が幻だった**」こと:

- r2 は `navigation.json` の 5 サンプル全部が `vx=0.000 / vyaw=+0.500`＝
  **その場旋回しか指令していない**のに、推定姿勢は 1.98 m 動いた
- bag の全レートで見ると見かけの速さは最大 **3.71 m/s**（指令上限 0.30 m/s）。
  全区間の 80% が上限超、31% が 1.0 m/s 超
- 見かけの速さと dt は**無相関**（r = −0.087）＝ 姿勢の穴を跨いだ再着地ではなく
  連続的なすべり

つまり「推定姿勢が**機体に出せない速さで動いている**」ことは、真値が無くても
その場で言える。これが G0。既存の候補（`eval_guards.py` の G1〜G4）は
跳び・障害物侵入・停滞を見るもので、**この壊れ方は捕まえられない**
（G1 は実機の記録で 16.6% 誤報、G4 は 108 s かかって被害を止めるだけ）。

## 何を測るか

| | 素材 | 合格 |
|---|---|---|
| 検出 | 歩行 3 本（`stage_20260910T182639_r{1,2,3}`） | **3 本とも鳴る**。なるべく早く |
| 誤報 | 静止の対照（`stage_20260910T084053`、最大変位 0.049 m） | **鳴らない** |

## 判定の作り

各 `/tf`（map -> base_link）の連続 2 点から見かけの速さ |Δxy|/Δt を出し、
**窓 W 秒の中央値**が `上限 × 係数` を超えたら鳴る。中央値にするのは
1 サンプルの尖りで鳴らせないため（`pose-jump-vs-command-limit` の型）。

使い方:

    Navigation/.venv/bin/python quickstart/eval_guard_speed.py \\
        --walk runs/stage_20260910T182639_r1 runs/stage_20260910T182639_r2 \\
               runs/stage_20260910T182639_r3 \\
        --still runs/stage_20260910T084053 \\
        --nav2-yaml ../../Navigation/nav2/g1_nav2.yaml
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import Cdr  # noqa: E402

# 掃引する範囲。ここを動かして合否が変わるなら、その門番は使えない
FACTORS = (1.5, 2.0, 3.0, 5.0)
WINDOWS_S = (0.5, 1.0, 2.0)
FALLBACK_MAX_VEL = 0.30      # yaml を読めなかったときだけ使う。黙って 0 にしない
# 静止の対照の「窓中央値の最大」に対して、しきい値がこれだけ離れていないと採らない。
# ⚠️ ぎりぎりのしきい値は別の日の静止で鳴く。**余裕は合否の一部**であって後付けの注記ではない
MIN_MARGIN = 2.0


def read_tf_track(bag_dir: Path) -> np.ndarray:
    """bag の /tf から map -> base_link だけを取る（点群は読まない。速い）。

    `measure_overlay.read_bag` と同じ読み方だが、あちらは 200 万点の点群も
    読むので、姿勢だけ要るここでは使わない（CDR の読み手だけ借りる）。
    """
    db = next(bag_dir.glob("*.db3"))
    con = sqlite3.connect("file:{}?mode=ro".format(db), uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    if "/tf" not in tid:
        raise SystemExit("{} に /tf が無い（記録トピックを確かめる）".format(bag_dir))

    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            sec, nsec = r.i32(), r.u32()
            parent, child = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]
            q = [r.f64() for _ in range(4)]
            if parent == "map" and child == "base_link":
                yaw = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                                 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
                rows.append([sec + nsec * 1e-9, tr[0], tr[1], yaw])
    if not rows:
        raise SystemExit("{} に map->base_link が 1 件も無い".format(bag_dir))
    return np.array(sorted(rows))


def apparent_speed(track: np.ndarray) -> "tuple[np.ndarray, np.ndarray]":
    """連続 2 点の見かけの速さ [m/s] と、その区間の中央時刻を返す。"""
    dt = np.diff(track[:, 0])
    good = dt > 1e-6                      # 同時刻の重複は捨てる（0 割りを作らない）
    d = np.hypot(np.diff(track[:, 1]), np.diff(track[:, 2]))
    return d[good] / dt[good], (track[:-1, 0] + track[1:, 0])[good] / 2.0


def first_fire(times: np.ndarray, speed: np.ndarray,
               limit: float, window_s: float) -> "float | None":
    """窓の中央値が limit を超えた最初の時刻（track の先頭からの秒数）。"""
    for i in range(len(times)):
        lo = times[i] - window_s
        window = speed[(times >= lo) & (times <= times[i])]
        if len(window) >= 3 and float(np.median(window)) > limit:
            return float(times[i] - times[0])
    return None


def read_max_vel(yaml_path: "Path | None") -> float:
    """機体に届く並進の上限 [m/s] を Nav2 の yaml から読む。

    見るのは `velocity_smoother` の `max_velocity: [vx, vy, vyaw]`。
    **これが /cmd_vel に出る最後の関門**で、RPP の `desired_linear_vel` より硬い。
    速さは vx と vy の合成（横歩きも並進なので勝手に落とさない）。
    ⚠️ 読めなければ言ってから控えに落ちる（黙って 0 にしない）。
    """
    if yaml_path is None or not yaml_path.exists():
        print("⚠️ nav2 の yaml を読めないので上限 {} m/s を使う（{}）".format(
            FALLBACK_MAX_VEL, yaml_path))
        return FALLBACK_MAX_VEL
    for line in yaml_path.read_text().splitlines():
        head = line.split("#", 1)[0]
        if "max_velocity:" in head and "[" in head:
            nums = head.split("[", 1)[1].split("]", 1)[0].split(",")
            vx, vy = float(nums[0]), float(nums[1])
            print("上限は velocity_smoother の max_velocity [{}, {}] から "
                  "hypot = {:.3f} m/s".format(vx, vy, math.hypot(vx, vy)))
            return math.hypot(vx, vy)
    print("⚠️ {} に max_velocity が無いので上限 {} m/s を使う".format(
        yaml_path, FALLBACK_MAX_VEL))
    return FALLBACK_MAX_VEL


def worst_window_median(times: np.ndarray, speed: np.ndarray, window_s: float) -> float:
    """窓の中央値がいちばん大きくなる値。**静止でこれを見て余裕を決める。**"""
    worst = 0.0
    for i in range(len(times)):
        window = speed[(times >= times[i] - window_s) & (times <= times[i])]
        if len(window) >= 3:
            worst = max(worst, float(np.median(window)))
    return worst


def describe(name: str, track: np.ndarray, speed: np.ndarray, limit: float) -> None:
    span = track[-1, 0] - track[0, 0]
    moved = float(np.hypot(track[-1, 1] - track[0, 1], track[-1, 2] - track[0, 2]))
    print("  {:<34} /tf {:4d} 件 / {:5.1f} s / 端から端 {:5.2f} m / "
          "見かけ最大 {:5.2f} m/s / 上限超 {:4.1f}%".format(
              name, len(track), span, moved, float(speed.max()),
              100.0 * float((speed > limit).mean())))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--walk", nargs="+", type=Path, required=True,
                    help="鳴ってほしい記録（歩行）")
    ap.add_argument("--still", nargs="+", type=Path, required=True,
                    help="鳴ってほしくない記録（静止の対照）")
    ap.add_argument("--nav2-yaml", type=Path, default=None, help="max_vel_x の出どころ")
    a = ap.parse_args()

    limit = read_max_vel(a.nav2_yaml)
    print("指令の上限 max_vel_x = {} m/s\n".format(limit))

    data = {}
    print("素材:")
    for kind, dirs in (("歩行", a.walk), ("静止", a.still)):
        for d in dirs:
            track = read_tf_track(d / "bag")
            speed, times = apparent_speed(track)
            data[d.name] = (kind, track, speed, times)
            describe("[{}] {}".format(kind, d.name), track, speed, limit)

    # 窓ごとに「静止でいちばん大きくなる窓中央値」を先に出す（余裕の分母）
    still_worst = {}
    for window in WINDOWS_S:
        still_worst[window] = max(
            worst_window_median(t, s, window)
            for n, (k, _, s, t) in data.items() if k == "静止")

    print("\n静止の対照の窓中央値の最大（しきい値はこれの {:.0f} 倍以上でないと採らない）:".format(
        MIN_MARGIN))
    for window in WINDOWS_S:
        print("  窓 {:.1f} s -> {:.3f} m/s".format(window, still_worst[window]))

    print("\n掃引（窓 W 秒の中央値 > 上限 x 係数 で鳴る）:")
    print("  {:>4} {:>5} {:>7} {:>5} | {:<26} | {}".format(
        "係数", "窓s", "しきい", "余裕", "歩行が鳴るまで [s]", "静止"))
    best = None
    for factor in FACTORS:
        for window in WINDOWS_S:
            threshold = limit * factor
            fires, false_alarm = [], []
            for name, (kind, track, speed, times) in data.items():
                t = first_fire(times, speed, threshold, window)
                (fires if kind == "歩行" else false_alarm).append((name, t))
            got = [t for _, t in fires if t is not None]
            bad = [n for n, t in false_alarm if t is not None]
            margin = (threshold / still_worst[window]) if still_worst[window] > 0 else 99.0
            detail = " / ".join("{:.1f}".format(t) if t is not None else "鳴らず"
                                for _, t in fires)
            ok = len(got) == len(fires) and not bad and margin >= MIN_MARGIN
            if bad:
                verdict = "誤報 {}".format(",".join(bad))
            elif margin < MIN_MARGIN:
                verdict = "鳴らずだが余裕不足"
            else:
                verdict = "合格"
            print("  {:>4.1f} {:>5.1f} {:>7.2f} {:>4.1f}x | {:<26} | {}".format(
                factor, window, threshold, margin, detail, verdict))
            # 合格の条件: 歩行が全部鳴り、静止が鳴らず、**余裕が MIN_MARGIN 以上**。
            # その中で「いちばん遅い歩行が鳴るまでの時間」が最短のものを採る
            if ok:
                score = max(got)
                if best is None or score < best[0]:
                    best = (score, factor, window, margin)

    print()
    if best is None:
        print("**この門番は今の素材では成立しない**"
              "（全部鳴らせるしきい値では静止との余裕が {:.0f} 倍に届かない）".format(MIN_MARGIN))
        return 1
    score, factor, window, margin = best
    print("**採用: 上限 x {:.1f}（= {:.2f} m/s）/ 窓 {:.1f} s**".format(
        factor, limit * factor, window))
    print("  歩行 3 本とも遅くとも {:.1f} s で鳴る / 静止では鳴らない / "
          "静止に対する余裕 {:.1f} 倍".format(score, margin))
    return 0


if __name__ == "__main__":
    sys.exit(main())
