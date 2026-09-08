#!/usr/bin/env python3
"""G3'（指令したのに並進も旋回もしない）のしきい値を掃く。

## G3 から G3' に変えた理由（実測）

並進だけを見る G3 は、**ゴール到着時のその場旋回**を「嵌った」と誤判定した。
成功した記録の最も動かない 10 s 窓は、yaw_rate 0.8 rad/s で
31〜48 度回りながら 0.12 m しか進んでいない（vx は 0）。
だから並進のしきい値を 0.15 m 以上にすると誤報し、0.10 m 以下でしか使えず、
そのぶん検知が遅れた（最速でも 101 s）。

旋回も進捗と数えれば、並進のしきい値を緩めても誤報しない。

## 実機で使える量だけ

cmd_vel（指令）と脚オドメトリの姿勢。**測位器も地図も使わない。**
だから測位が壊れていても地図が古くても動き、`rclpy` も要らないので
**脚を止められる側（SDK プロセス）に置ける**。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

CMD_MIN = 0.05
CMD_FRAC = 0.8
GRACE_S = 5.0

RUN_DIR = Path("Mapping/real/runs/20260906T135940_UiS_room_v3/measure_20260908")


def split_runs(track: list[dict]) -> list[tuple[int, int]]:
    b = [0]
    for i in range(1, len(track)):
        if track[i]["goal"] != track[i - 1]["goal"] or \
           track[i]["t"] - track[i - 1]["t"] > 5.0:
            b.append(i)
    b.append(len(track))
    return [(b[k], b[k + 1]) for k in range(len(b) - 1)]


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def fire_time(seg: list[dict], win_s: float, moved_m: float,
              turned_deg: float) -> float | None:
    """並進も旋回もしていない窓を最初に見つけた時刻。鳴らなければ None。"""
    t0 = seg[0]["t"]
    dist = [0.0]
    turn = [0.0]
    for i in range(1, len(seg)):
        p, q = seg[i - 1]["truth"], seg[i]["truth"]
        dist.append(dist[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
        turn.append(turn[-1] + abs(wrap(q[2] - p[2])))
    for i in range(len(seg)):
        if seg[i]["t"] - t0 < GRACE_S + win_s:
            continue
        j = i
        while j > 0 and seg[i]["t"] - seg[j]["t"] < win_s:
            j -= 1
        if seg[i]["t"] - seg[j]["t"] < win_s:
            continue
        win = seg[j:i + 1]
        commanded = sum(1 for s in win
                        if math.hypot(s["cmd"][0], s["cmd"][1]) > CMD_MIN
                        or abs(s["cmd"][2]) > CMD_MIN)
        if commanded / len(win) < CMD_FRAC:
            continue
        if dist[i] - dist[j] < moved_m and \
           math.degrees(turn[i] - turn[j]) < turned_deg:
            return seg[i]["t"] - t0
    return None


def main() -> None:
    fail = json.loads((RUN_DIR / "navigation.json").read_text())
    ok = json.loads((RUN_DIR / "perfect_loc/navigation.json").read_text())
    fail_runs = [fail["track"][a:b] for a, b in split_runs(fail["track"])]
    ok_runs = [ok["track"][a:b] for a, b in split_runs(ok["track"])]
    print(f"失敗 {fail['results']} / {len(fail_runs)} 区間、"
          f"成功 {ok['results']} / {len(ok_runs)} 区間（猶予 {GRACE_S:.0f} s 固定）\n")
    print("窓[s] 移動[m] 旋回[度] | 失敗区間で鳴った時刻 | 成功区間の誤報")
    print("-" * 70)
    best = []
    for win in (5.0, 8.0, 10.0, 15.0):
        for moved in (0.15, 0.25, 0.40):
            for turned in (10.0, 20.0, 30.0):
                f = [fire_time(s, win, moved, turned) for s in fail_runs]
                o = [fire_time(s, win, moved, turned) for s in ok_runs]
                fp = sum(1 for x in o if x is not None)
                hit = sum(1 for x in f if x is not None)
                mark = ""
                if fp == 0 and hit == len(fail_runs):
                    mark = "  <= 使える"
                    best.append((max(f), win, moved, turned))
                fs = " ".join("鳴らず" if x is None else f"{x:5.0f}s" for x in f)
                print(f"{win:4.0f} {moved:7.2f} {turned:7.0f}  | {fs:20s} | "
                      f"{fp}/{len(ok_runs)}{mark}")
    print()
    if best:
        best.sort()
        t, win, moved, turned = best[0]
        print(f"== 誤報 0 で全区間を捕まえるうち最速: 窓 {win:.0f} s / "
              f"移動 {moved:.2f} m / 旋回 {turned:.0f} 度（最も遅い検知 {t:.0f} s）==")
    else:
        print("== 誤報 0 で全区間を捕まえる組み合わせは無かった ==")


if __name__ == "__main__":
    main()
