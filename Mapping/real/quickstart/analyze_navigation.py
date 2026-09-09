#!/usr/bin/env python3
"""`check_navigation.py --record` の記録を読んで、走行の中身を数字にする。

到達 n/N だけでは「なぜ着かなかったのか」が読めない。この入口は次を出す。

- ゴールごとの直線距離・所要・**実効速度**（wall clock）
- 実際に歩いた距離（真値の積算）と、それが直線距離の何倍か
- **止まっていた時間**（指令は出ているのに動かない時間）
- **経路が通る所の最小余裕**（事前地図の距離変換）。Nav2 は `robot_radius`
  0.25 m の円で計画するので、G1 が物理的に通れない隙間へ経路を引くことがある
- 停滞した地点の余裕

⚠️ 実効速度は記録ごとに大きく違う。2026-09-08 の実測では、
**短距離（直線 3.8 m）が 0.076 m/s、長距離（直線 15.2 m）が 0.0249 m/s** で
**3 倍違った**。短距離の値で `--timeout` を決めると長距離が時間切れになる。

    .venv/bin/python Mapping/real/quickstart/analyze_navigation.py \\
        --rec <記録のディレクトリ>
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

R = Path("Mapping/real/runs/20260906T135940_UiS_room_v3")
ROBOT_RADIUS = 0.25          # IsaacSim_Env/config/nav2.yaml の値
STAND_Z = 0.63               # 正常に立っているときの pelvis の高さ [m]
STALL_M = 0.02               # 0.5 秒でこれ未満しか進まなければ「並進していない」
STALL_DEG = 1.0              # 0.5 秒でこれ未満しか回らなければ「旋回していない」
CMD_EPS = 0.02               # 指令が出ていると見なす下限


def load_clearance() -> tuple[np.ndarray, float, float, float]:
    """事前地図の各セルから最も近い障害物までの距離 [m]。"""
    meta: dict[str, str] = {}
    for line in (R / "map/nav_map_clean.yaml").read_text().splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    img = np.asarray(Image.open(R / "map/nav_map_clean.pgm"), dtype=np.float64)
    occ = ((255.0 - img) / 255.0) > float(meta["occupied_thresh"])
    return ndimage.distance_transform_edt(~occ) * res, res, ox, oy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True, help="navigation.json が在るディレクトリ")
    args = ap.parse_args()

    data = json.loads((Path(args.rec) / "navigation.json").read_text())
    track = [s for s in data["track"] if s["truth"] and s["amcl"]]
    if not track:
        raise SystemExit("[NG] 記録が空")

    clear, res, ox, oy = load_clearance()
    h = clear.shape[0]

    def clearance_at(x: float, y: float) -> float:
        r, c = h - 1 - int((y - oy) / res), int((x - ox) / res)
        if 0 <= r < clear.shape[0] and 0 <= c < clear.shape[1]:
            return float(clear[r, c])
        return 0.0

    # ゴールが変わったところで区切る
    segments: list[list[dict]] = []
    for s in track:
        g = tuple(s["goal"]) if s["goal"] else None
        if not segments or (segments[-1][0]["goal"] and
                            tuple(segments[-1][0]["goal"]) != g):
            segments.append([s])
        else:
            segments[-1].append(s)

    print(f"[rec] {args.rec}")
    print(f"[planner_id] {data.get('planner_id') or '(既定)'}")
    print(f"[rec] {len(track)} サンプル / "
          f"{track[-1]['t'] - track[0]['t']:.0f} s / {len(segments)} 区間\n")

    for k, seg in enumerate(segments, 1):
        g = seg[0]["goal"]
        t0, t1 = seg[0]["t"], seg[-1]["t"]
        start = seg[0]["truth"]
        end = seg[-1]["truth"]
        straight = math.hypot(g[0] - start[0], g[1] - start[1])
        gap = math.hypot(g[0] - end[0], g[1] - end[1])
        walked = sum(math.hypot(b["truth"][0] - a["truth"][0],
                                b["truth"][1] - a["truth"][1])
                     for a, b in zip(seg, seg[1:]))
        elapsed = t1 - t0
        # 指令は出ているのに動かない時間。
        # ⚠️ 並進だけを見ると**その場旋回**を誤って数える（ゴール到着時の
        # 向き合わせは vx 0 / yaw 0.8 rad/s で正常な動作である）。
        # 「進んでいない」と「進みも回りもしていない」を分けて出す。
        no_move, no_move_no_turn = 0.0, 0.0
        for a, b in zip(seg, seg[1:]):
            moved = math.hypot(b["truth"][0] - a["truth"][0],
                               b["truth"][1] - a["truth"][1])
            turned = abs(math.degrees(
                math.atan2(math.sin(b["truth"][2] - a["truth"][2]),
                           math.cos(b["truth"][2] - a["truth"][2]))))
            cmd = max(abs(a["cmd"][0]), abs(a["cmd"][2]))
            if cmd > CMD_EPS and moved < STALL_M:
                no_move += b["t"] - a["t"]
                if turned < STALL_DEG:
                    no_move_no_turn += b["t"] - a["t"]
        zs = [s["sim_z"] for s in seg if s["sim_z"] is not None]
        errs = [math.hypot(s["truth"][0] - s["amcl"][0],
                           s["truth"][1] - s["amcl"][1]) for s in seg]
        # 経路が通る所の最小余裕（この区間で見たすべての経路の中で）
        path_clear, tight = math.inf, None
        for s in seg:
            for px, py in s["plan"]:
                c = clearance_at(px, py)
                if c < path_clear:
                    path_clear, tight = c, (px, py)

        print(f"── 区間 {k}: ゴール ({g[0]:+.2f}, {g[1]:+.2f}) "
              f"余裕 {clearance_at(*g):.2f} m ─────")
        print(f"   出発 ({start[0]:+.2f}, {start[1]:+.2f}) 余裕 "
              f"{clearance_at(start[0], start[1]):.2f} m  →  "
              f"到達 ({end[0]:+.2f}, {end[1]:+.2f}) 余裕 "
              f"{clearance_at(end[0], end[1]):.2f} m")
        print(f"   直線 {straight:5.2f} m / 歩いた {walked:5.2f} m "
              f"（{walked / straight:.2f} 倍） / ゴールまで残り {gap:5.2f} m")
        print(f"   所要 {elapsed:5.1f} s  実効 {straight / elapsed:.4f} m/s"
              f"（直線ベース） {walked / elapsed:.4f} m/s（軌跡ベース）")
        print(f"   指令は出ているのに進まなかった時間 {no_move:5.1f} s"
              f"（{100 * no_move / elapsed:.0f} %）"
              f"  うち旋回もしていない {no_move_no_turn:5.1f} s"
              f"（{100 * no_move_no_turn / elapsed:.0f} %）")
        if path_clear < math.inf:
            mark = " ← robot_radius を割る所を通そうとしている" \
                if path_clear < ROBOT_RADIUS else ""
            print(f"   経路が通る所の最小余裕 {path_clear:.2f} m "
                  f"@ ({tight[0]:+.2f}, {tight[1]:+.2f}){mark}")
        if zs:
            print(f"   真値 z {min(zs):.2f}〜{max(zs):.2f} m"
                  + ("  ← 乗り上げ" if max(zs) > STAND_Z + 0.10 else ""))
        print(f"   測位のずれ 中央 {1000 * float(np.median(errs)):.1f} mm / "
              f"最大 {1000 * max(errs):.1f} mm")
        print()

    total_walked = sum(math.hypot(b["truth"][0] - a["truth"][0],
                                  b["truth"][1] - a["truth"][1])
                       for a, b in zip(track, track[1:]))
    print(f"[合計] 歩いた {total_walked:.2f} m / "
          f"{track[-1]['t'] - track[0]['t']:.0f} s")


if __name__ == "__main__":
    main()
