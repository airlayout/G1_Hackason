#!/usr/bin/env python3
"""`dump_grids.py` が落とした 2D の層を 1 枚の PNG に並べる。

## なぜ要るのか

「RViz の 2D 地図がざらつく」のような話は、**どの層が描いたのか**が分からないと
直せない。2026-09-11 にこの形で比べて、ざらつきの正体が `/projected_map`
（octomap_server がライブで足し続ける投影）だと分かった。範囲の 43% が占有で、
床まで投影していた。Nav2 は一切参照していなかったので opt-in に落とした。

## 使い方（Mac 側の venv。コンテナの matplotlib は numpy 1.x ビルドで壊れている）

    Navigation/.venv/bin/python quickstart/render_layers.py \\
        runs/stage_.../layers --out runs/stage_.../layers.png

⚠️ 来なかった層は**描かずに「来なかった」と出す**（`dump_grids.py` の counts.json）。
   `/projected_map` は 2026-09-11 から既定 off なので、来ないのが正常。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_nav_map import DPI, as_image, extent  # noqa: E402
from check_map_clearance import load_map  # noqa: E402
from jp_font import japanese_font  # noqa: E402

# 並べる順と見出し。counts.json に無い層は飛ばす
TITLES = {
    "static": "Static map 2D\n(map_server / Nav2 の静的レイヤ)",
    "global_costmap": "Global Costmap\n(静的 + 障害物 + 膨張)",
    "local_costmap": "Local Costmap\n(ライブ。rolling)",
    "projected": "Projected map 2D\n(octomap_server。要 G1_OCTOMAP=1)",
}


def subtitle(info: dict) -> str:
    occ = info.get("occupied_65_98", 0) + info.get("lethal_99_100", 0)
    cells = max(info.get("cells", 1), 1)
    return ("{w}x{h} / res {r:.3f} m\n"
            "占有 {occ:,} ({pct:.1f}%) / 膨張 {inf:,} / 未知 {unk:,}".format(
                w=info["size"][0], h=info["size"][1], r=float(info["resolution"]),
                occ=occ, pct=100.0 * occ / cells,
                inf=info.get("inflated_26_64", 0), unk=info.get("unknown", 0)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump_dir", type=Path, help="dump_grids.py --out で指した場所")
    ap.add_argument("--out", type=Path, default=None, help="既定は <dump_dir>/../layers.png")
    ap.add_argument("--title", default="", help="図の上に出す一言（回の名前など）")
    a = ap.parse_args()

    # ⚠️ 見出しは日本語。フォントが無いまま描くと全部豆腐になる（黙って進めない）
    if japanese_font() is None:
        print("⚠️ 日本語フォントが見つからない。見出しが豆腐になる", file=sys.stderr)

    counts_path = a.dump_dir / "counts.json"
    if not counts_path.exists():
        raise SystemExit("counts.json が無い: {}（先に dump_grids.py を回す）".format(counts_path))
    report = json.loads(counts_path.read_text())

    got = [(n, i) for n, i in report.items() if i.get("received")]
    missing = [n for n, i in report.items() if not i.get("received")]
    if not got:
        raise SystemExit("受け取れた層が 1 つも無い（スタックが起きているか確かめる）")

    fig, axes = plt.subplots(1, len(got), figsize=(5.2 * len(got), 5.6))
    if len(got) == 1:
        axes = [axes]
    for ax, (name, info) in zip(axes, got):
        grid = load_map(a.dump_dir / name)
        ax.imshow(as_image(grid), origin="lower", extent=extent(grid),
                  cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        ax.set_title("{}\n{}".format(TITLES.get(name, name), subtitle(info)), fontsize=9)
        ax.set_xlabel("map x [m]", fontsize=8)
        ax.tick_params(labelsize=7)
    axes[0].set_ylabel("map y [m]", fontsize=8)

    head = a.title or "2D の層（同じ時刻の latched topic を落として描いた）"
    if missing:
        head += "   ／ 来なかった層: " + ", ".join(missing)
    fig.suptitle(head, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    out = a.out or (a.dump_dir.parent / "layers.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=DPI)
    print("書いた: {}".format(out))
    if missing:
        print("来なかった層: {}".format(", ".join(missing)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
