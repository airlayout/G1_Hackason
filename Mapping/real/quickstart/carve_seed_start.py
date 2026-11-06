#!/usr/bin/env python3
"""**育つ地図の種から、機体が立っている円柱を空きにする。**（up.sh の段 2.5・Mac で動かす）

    G1_Hackason/.venv/bin/python quickstart/carve_seed_start.py \\
        --seed seed.bt --out seed_start.bt --pose "1.633 -1.300 97.05"

## なぜ要るのか（2026-09-28 実機）

2D Goal Pose をクリックすると、機体が**その場で 90° 回って 5 秒待つ**を 2 回繰り返し、
ゴールを諦めた。planner のログは毎回

    Smac2D: failed to create plan, invalid use: Starting point in lethal space!

原因は 3 つの重なり:

1. 種（`nav_map_run` → `seed.bt`）に、**記録のあとで動かされた家具**が残っていた。
   09-06 と 09-11 の記録では同じ場所に物があった（09-11 は 582/1210 枚・27,608 点・
   高さ 0.3〜1.5 m）が、今日の LiDAR には写らない。機体の立ち位置とも重なる。
2. `octomap_server` は**レイが通ったボクセルしか空きにしない**。頭の LiDAR は
   機体から水平 0.90 m より内側の床を見られない（今日のスキャンの実測）。
   帯 0.23〜1.80 m のうち、機体から 0.7 m 以内の低い所には**レイが一度も通らない**。
   その場で回っても、見えない範囲は機体を中心とした円なので変わらない。
3. `robot_radius` 0.30 m の円に占有が 1 セルでもあれば、Smac2D は経路を作らない。

⇒ **機体の体がある場所に障害物は存在できない。**だから、この円柱を空きにするのは
測らなくても正しい（家具がどこへ動いても同じ）。

## 半径の決め方（既定 0.35 m）

- `filter_self_returns.py` が自分の体として落とす円柱と**同じ 0.35 m**
  （`run_nav2_live.sh` の `SELF_RADIUS`）。LiDAR の点もここから先しか入らない
- `robot_radius` 0.30 m ＋ 半セル。セルの中心で数えるので、0.35 m より外の占有は
  スタートの cost を 99（内接）未満に保つ
- ⚠️ **広げすぎない。** 0.35〜0.9 m も死角だが、そこは機体が動けばレイが届く。
  本物の机の縁を「空き」で上書きすると、死角の中では**確かめる手段が無い**

## やらないこと

- 元の `seed.bt` は書き換えない（出力は別ファイル。起動のたびに作り直す）
- 空きにするのは円柱の中だけ。円柱の外の占有は 1 つも触らない（検算で確かめる）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import octomap

DEFAULT_RADIUS = 0.35      # filter_self_returns.py の円柱と同じ
DEFAULT_Z = (0.0, 2.0)     # 帯 0.23〜1.80 m を覆う
MISS_REPEAT = 20           # 占有の上限（clamping）から空きへ確実に落とすための回数


def parse_pose(text: str) -> "tuple[float, float]":
    parts = text.split()
    if len(parts) != 3:
        raise SystemExit(f"[carve] --pose は 'X Y YAW_DEG' の 3 つ: {text!r}")
    return float(parts[0]), float(parts[1])


def cylinder_points(cx: float, cy: float, radius: float,
                    z_range: "tuple[float, float]", res: float) -> np.ndarray:
    """円柱の中のボクセル中心を、分解能の半分の刻みで並べる（取りこぼし防止）。"""
    step = res / 2.0
    xs = np.arange(cx - radius, cx + radius + step, step)
    ys = np.arange(cy - radius, cy + radius + step, step)
    zs = np.arange(z_range[0] + step / 2, z_range[1], step)
    gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
    pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    inside = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) <= radius
    return np.ascontiguousarray(pts[inside])


def occupied_leaves(tree: octomap.OcTree) -> np.ndarray:
    out = [(n.getX(), n.getY(), n.getZ()) for n in tree.begin_leafs() if tree.isNodeOccupied(n)]
    return np.asarray(out, dtype=float).reshape(-1, 3)


def still_occupied(tree: octomap.OcTree, q: np.ndarray) -> bool:
    node = tree.search(np.asarray(q, dtype=float))
    try:
        return bool(tree.isNodeOccupied(node))
    except Exception:                 # 見つからない（unknown）＝ 占有ではなくなった
        return False


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seed", type=Path, required=True, help="元の種（読むだけ）")
    p.add_argument("--out", type=Path, required=True, help="書き出す種")
    p.add_argument("--pose", required=True, help="base_link の初期姿勢 'X Y YAW_DEG'（map 系）")
    p.add_argument("--radius", type=float, default=DEFAULT_RADIUS)
    p.add_argument("--z", type=float, nargs=2, default=DEFAULT_Z, metavar=("MIN", "MAX"))
    p.add_argument("--resolution", type=float, default=0.1)
    a = p.parse_args()

    cx, cy = parse_pose(a.pose)
    tree = octomap.OcTree(a.resolution)
    if not tree.readBinary(str(a.seed).encode()):
        print(f"[carve] ⛔ 種を読めない: {a.seed}", file=sys.stderr)
        return 2

    before = occupied_leaves(tree)
    d_before = np.hypot(before[:, 0] - cx, before[:, 1] - cy)
    in_before = int((d_before <= a.radius).sum())
    near = d_before.min() if len(d_before) else float("inf")
    print(f"[carve] 立ち位置 ({cx:+.3f}, {cy:+.3f}) / 半径 {a.radius:.2f} m / "
          f"最寄りの占有 {near:.2f} m / 円柱の中の占有 {in_before}")

    pts = cylinder_points(cx, cy, a.radius, tuple(a.z), a.resolution)
    for _ in range(MISS_REPEAT):
        tree.updateNodes(pts, False)
    tree.updateInnerOccupancy()

    after = occupied_leaves(tree)
    d_after = np.hypot(after[:, 0] - cx, after[:, 1] - cy)
    in_after = int((d_after <= a.radius).sum())
    # ⚠️ **葉の数で比べない。**octomap は同じ値の 8 子を 1 枚の大きな葉にまとめる（prune）ので、
    # 中身が同じでも葉の数は変わる（初版はこれで「外が減った」と誤って落ちた）。
    # 書き換え前に占有だった位置を 1 点ずつ引き直し、円柱の外で空きに変わった物を数える
    outside = before[d_before > a.radius + a.resolution]
    lost = sum(1 for q in outside if not still_occupied(tree, q))

    a.out.parent.mkdir(parents=True, exist_ok=True)
    tree.writeBinary(str(a.out).encode())
    back = octomap.OcTree(a.resolution)
    readable = bool(back.readBinary(str(a.out).encode()))

    checks = [
        ("円柱の中の占有が 0", in_after == 0),
        (f"円柱の外の占有は減っていない（{len(outside)} 点を引き直した）", lost == 0),
        ("書いた種を読み戻せる", readable),
    ]
    for label, ok in checks:
        print(f"[carve]   {'✅' if ok else '❌'} {label}")
    if not all(ok for _, ok in checks):
        print("[carve] ⛔ 検算に落ちた。元の種を使うこと", file=sys.stderr)
        return 1
    near_after = d_after.min() if len(d_after) else float("inf")
    print(f"[carve] 空きにした占有 {in_before - in_after} ボクセル → "
          f"最寄りの占有 {near_after:.2f} m / 出力 {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
