#!/usr/bin/env python3
"""任意の 2D 地図（.yaml）を、**走る地図と同じ採点表**で採点する。

## なぜ要るか

`ghost/make_nav_map_run.py` は「`nav_map_clean` に足す候補」専用に書かれていて、
採点関数（`measure`）だけが欲しい場面で丸ごと動かせない。
段 1〜4（`octomap_server` の種と `/projected_map`）では**地図の出どころが違う**ので、
採点だけを切り出して呼ぶ。**採点の定義は 1 箇所（`make_nav_map_run.measure`）のまま**。

採点の 6 列の意味は `ghost/make_nav_map_run.py` の docstring にある。要点:

| 列 | 何を見るか | 合否の線 |
|---|---|---|
| 占有 | **消しすぎの歯止め。**激減していないこと | 現行比 |
| クリア中央 | 軌跡が自分の道を塞いでいないか | > 0.30 m（`robot_radius`）|
| 壁の帯 | 測位の重畳（09-10 のライブ基準） | 90 % 以上 |
| 机の帯 | 机が 2D に出ているか | 現行以上 |
| 通れる wp | 経路が引けるか | 現行と同数 |

⚠️ **重畳は 1 つの数字で判断しない。**同じ正しい姿勢が既定帯で 44.4 %、
壁の帯で 91.9 % に割れる（2026-09-11 実測）。だから壁と机を別に出す。

⚠️ **範囲の違いに注意。**`/projected_map` は木の bbox しか持たない（レイの届いた所だけ）。
`nav_map_run` は SLAM 地図の全域なので、**遠方の廊下を含む**。`--bbox-of` を渡すと
その地図の範囲に切って比べられる（範囲の差と中身の差を分ける）。

## 使い方

    G1_Hackason/.venv/bin/python quickstart/score_nav_map.py \\
        runs/<id>/map/nav_map_run.yaml runs/<id>/map/seed_projected.yaml

    # 範囲を揃えて比べる
    ... --bbox-of runs/<id>/map/seed_projected.yaml
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "ghost"))

from check_map_clearance import load_map                              # noqa: E402
from ghost.make_nav_map_run import ROBOT_RADIUS, measure, row         # noqa: E402
from ghost.score import SESSION, Scorecard                            # noqa: E402

HEADER = "{:<24}{:>8}{:>10}{:>8}{:>8}{:>8}{:>10}".format(
    "地図", "占有", "クリア中央", "<0.30m", "壁の帯", "机の帯", "通れる wp")


def bbox_of(yaml_path: Path) -> "tuple[float, float, float, float]":
    """地図の占有セルが在りうる範囲[m]を (x0, x1, y0, y1) で返す。"""
    g = load_map(yaml_path)
    h, w = g.shape
    return (g.origin[0], g.origin[0] + w * g.resolution,
            g.origin[1], g.origin[1] + h * g.resolution)


def clip_traj(traj: np.ndarray, box: "tuple[float, float, float, float]") -> np.ndarray:
    x0, x1, y0, y1 = box
    keep = ((traj[:, 0] >= x0) & (traj[:, 0] <= x1)
            & (traj[:, 1] >= y0) & (traj[:, 1] <= y1))
    return traj[keep]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("maps", type=Path, nargs="+", help="採点する地図の .yaml（複数可）")
    p.add_argument("--bbox-of", type=Path, default=None,
                   help="この地図の範囲に軌跡と waypoint を切ってから採点する。"
                        "範囲が違う地図どうしを比べるときに使う")
    p.add_argument("--labels", nargs="*", default=None, help="表に出す名前（既定はファイル名）")
    a = p.parse_args()

    sc = Scorecard()
    traj = np.loadtxt(SESSION / "mola_floor0" / "traj.txt", usecols=(1, 2))
    waypoints = json.loads((SESSION / "measure_20260908" / "waypoints.json").read_text())

    if a.bbox_of:
        box = bbox_of(a.bbox_of)
        before_traj, before_wp = len(traj), len(waypoints)
        traj = clip_traj(traj, box)
        waypoints = [w for w in waypoints
                     if box[0] <= w["x"] <= box[1] and box[2] <= w["y"] <= box[3]]
        print("範囲を {} に合わせた: x {:.1f}〜{:.1f} / y {:.1f}〜{:.1f}".format(
            a.bbox_of.name, *box))
        print("  軌跡 {:,} → {:,} 点 / waypoint {} → {}\n".format(
            before_traj, len(traj), before_wp, len(waypoints)))
        if len(traj) == 0:
            raise SystemExit("範囲内に軌跡が 0 点。bbox を間違えている")

    labels = a.labels or [m.stem for m in a.maps]
    print(HEADER)
    scores = []
    for label, path in zip(labels, a.maps):
        m = measure(path, sc, traj, waypoints)
        row(label, m)
        scores.append((label, m))

    print("\n段 3 の合否（基準は 1 番目の地図）")
    base = scores[0][1]
    for label, m in scores[1:]:
        checks = [
            ("壁の帯が 90 % 以上", m["wall"] >= 90.0),
            ("クリアランス中央 > {:.2f} m".format(ROBOT_RADIUS), m["med"] > ROBOT_RADIUS),
            ("占有セルが基準の 50 % 以上（消しすぎの歯止め）",
             m["cells"] >= base["cells"] * 0.50),
            ("通れる wp が基準と同数", m["free"] >= base["free"]),
        ]
        print("  [{}]".format(label))
        for text, ok in checks:
            print("    {} {}".format("✅" if ok else "❌", text))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
