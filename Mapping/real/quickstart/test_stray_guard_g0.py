#!/usr/bin/env python3
"""常駐ガードの門番 G0 が、記録に対して**評価器と同じ判定**になることを確かめる。

## なぜこの試験が要るのか

`eval_guard_speed.py` は記録を丸ごと numpy で処理して「鳴るか」を出す。
`stray_guard.py` は **1 サンプルずつ来るものを窓に溜めて**判定する。
同じ式でも、窓の切り方・最低本数・重複の捨て方が 1 つずれれば結論は変わる。
**実機に載せる前に、記録の上で両者が一致することを見ておく。**

⚠️ **実機も再生も要らない。**記録の `/tf` を読んでノードに流し込むだけ。

## ⚠️ 2 段に分かれている理由（2026-09-13 に踏んだ）

記録を読む側（`measure_overlay` → `scipy`）と、ガード本体（`rclpy`）が
**同じ python に同居できない**。コンテナの scipy は numpy 1 系向けにビルドされており、
入っている numpy は 2.2.6 なので `numpy.core.multiarray failed to import` で落ちる。
そこで **軌跡の取り出しは Mac の venv、判定はコンテナ**に分けてある。

    # 1. Mac 側で軌跡を取り出す（runs/_g0_tracks/*.npy に置く）
    Navigation/.venv/bin/python Mapping/real/quickstart/test_stray_guard_g0.py --extract

    # 2. コンテナ側で判定する
    docker exec -u ubuntu rviz bash -c "source /opt/ros/humble/setup.bash && \
      python3 /work/G1_Hackason/Mapping/real/quickstart/test_stray_guard_g0.py"

## 合否

| 素材 | 期待 |
|---|---|
| 歩行 3 本（`stage_20260910T182639_r{1,2,3}`）| **3 本とも鳴る** |
| 静止 7 本（09-10 の 1 本 ＋ 09-12 の 6 本）| **1 本も鳴らない** |

⚠️ 静止を 1 日ぶんで済ませないこと。震えは日によって 3 割動く。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
CACHE = RUNS / "_g0_tracks"

WALK = ["stage_20260910T182639_r1", "stage_20260910T182639_r2", "stage_20260910T182639_r3"]
STILL = ["stage_20260910T084053", "still_20260912T085449_r1", "still_20260912T085449_r2",
         "still_20260912T085449_r3", "still_20260912T0930_after_walk",
         "still_20260912T0956_after_goal1", "still_20260912T1035_after_spin"]


def extract() -> int:
    """Mac 側。bag の /tf から map -> base_link を取り出して .npy に置く。"""
    sys.path.insert(0, str(HERE))
    from eval_guard_speed import read_tf_track          # scipy が要る側
    CACHE.mkdir(parents=True, exist_ok=True)
    for name in WALK + STILL:
        track = read_tf_track(RUNS / name / "bag")
        np.save(CACHE / f"{name}.npy", track)
        print(f"  {name:<34} {len(track):5d} 点 -> {CACHE.name}/{name}.npy")
    print(f"\n{len(WALK) + len(STILL)} 本を書き出した。次はコンテナ側で引数なしで実行する")
    return 0


def run_one(node, track: np.ndarray) -> "float | None":
    """記録を 1 本流し、G0 が鳴るまでの秒数を返す（鳴らなければ None）。"""
    node.last_stamp = node.last_xy = None
    node.speeds.clear()
    fired: list[float] = []
    node._cancel = lambda: fired.append(node._feed_t)
    t0 = float(track[0][0])
    start = (float(track[0][1]), float(track[0][2]))
    for row in track:                  # 列は [t, x, y, yaw]。yaw は G0 では使わない
        t, x, y = row[0], row[1], row[2]
        node._feed_t = float(t) - t0
        # TF の代わりに記録の 1 点を返す
        node._xy_stamped = lambda t=float(t), x=float(x), y=float(y): (t, (x, y))
        node.watch = {"start": start, "goal": (0.0, 0.0), "dist": 0.0, "leash": 1e9,
                      "t0": node.get_clock().now(), "fired": False}
        node._tick()
        if fired:
            return fired[0]
    return None


def judge() -> int:
    """コンテナ側。ガード本体に流し込んで合否を出す。"""
    sys.path.insert(0, str(HERE))
    import rclpy
    import stray_guard                                  # rclpy が要る側

    class Args:
        """argparse の代わり。既定値は stray_guard の定義から取る（焼き直さない）。"""
        margin = 0.5
        max_abs = 1e9          # 首輪では鳴らせない
        max_seconds = 1e9      # 時間でも鳴らせない
        max_tf_gap = 1e9       # 途絶でも鳴らせない ⇒ **鳴ったら G0 だけが理由**
        rate = 20.0
        g0_factor = stray_guard.G0_FACTOR
        g0_window = stray_guard.G0_WINDOW_S
        no_g0 = False
        no_sim_time = False
        nav2_yaml = str(HERE.parent.parent.parent / "Navigation" / "nav2" / "g1_nav2.yaml")

    missing = [n for n in WALK + STILL if not (CACHE / f"{n}.npy").exists()]
    if missing:
        print(f"⚠️ 軌跡が無い（{len(missing)} 本）。先に Mac 側で --extract を実行すること")
        print(f"   置き場: {CACHE}")
        return 2

    rclpy.init()
    node = stray_guard.StrayGuard(Args())
    print("\nしきい値 {:.3f} m/s（上限 {:.3f} x {:.1f}）/ 窓 {:.1f}s / 最低 {} 本\n".format(
        node.g0_threshold, node.limit, Args.g0_factor, Args.g0_window,
        stray_guard.G0_MIN_SAMPLES))

    bad = 0
    for names, want_fire in ((WALK, True), (STILL, False)):
        for name in names:
            t = run_one(node, np.load(CACHE / f"{name}.npy"))
            ok = (t is not None) == want_fire
            bad += 0 if ok else 1
            print("  [{}] {:<34} {:<18} {}".format(
                "PASS" if ok else "FAIL", name,
                "{:.1f}s で鳴った".format(t) if t is not None else "鳴らず",
                "" if ok else ("**鳴ってほしかった**" if want_fire else "**誤報**")))
    # ── /tf の途絶 ───────────────────────────────────────────────
    # ⚠️ 実機（--no-sim-time）でしか効かない判定なので、**論理だけ**を見る。
    # 記録の再生では /clock も止まるため gap が育たない（stray_guard の注記参照）。
    print()
    node.a.max_tf_gap = 3.0
    now_s = node.get_clock().now().nanoseconds * 1e-9
    w = {"start": (0.0, 0.0), "goal": (0.0, 0.0), "dist": 0.0, "leash": 1e9,
         "t0": node.get_clock().now(), "fired": False}
    node.last_stamp = now_s - 5.0
    stalled = node._tf_stall(w) is not None
    node.last_stamp = now_s - 1.0
    fresh = node._tf_stall(w) is None
    bad += 0 if (stalled and fresh) else 1
    print("  [{}] /tf が 5s 止まれば鳴り、1s なら鳴らない（上限 3.0s）".format(
        "PASS" if (stalled and fresh) else "FAIL"))

    print("\n== 合否 ==")
    print("  [{}] 歩行 {} 本が鳴り、静止 {} 本が鳴らず、/tf 途絶も判定できる（不一致 {} 件）".format(
        "PASS" if bad == 0 else "FAIL", len(WALK), len(STILL), bad))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if bad == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--extract", action="store_true",
                    help="**Mac 側で**実行し、bag から軌跡を .npy に取り出す")
    return extract() if ap.parse_args().extract else judge()


if __name__ == "__main__":
    sys.exit(main())
