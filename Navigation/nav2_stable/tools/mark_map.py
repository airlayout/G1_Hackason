#!/usr/bin/env python3
"""地図をクリックして印を付ける。**押した位置の地図座標(m)がそのまま記録される。**

「地図では黒いのに実際は通路」「地図では空いているのに物がある」といった食い違いを、
現地の人が**座標を読まずに**指し示すための道具（2026-09-24）。

    python3 mark_map.py                      # 既定の地図を開く
    python3 mark_map.py ../maps/grids/room_b_map_Sorasta_20260923.yaml

## 操作

| | |
|---|---|
| **左クリック** | 印を付ける（赤丸＋番号） |
| **u** | 直前の印を消す |
| **s** | いま保存する（閉じても保存される） |
| 虫めがね/手のアイコン | 拡大・移動（matplotlib の標準ツールバー） |

印は `_local/map_view/marks.json`（地図座標）と `marks.png` に保存される。

📌 **色の意味**（プランナから見た形）:
白=走れる / 黒=障害物 / 薄橙=障害物に近すぎて入れない / 水色=切り離された島 / 灰=未観測
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402

# ⚠️ 既定の DejaVu Sans には日本語が入っておらず、**タイトルが豆腐になる**。
# Noto CJK があれば使う（無ければ英字のまま。落とさない）。
for _name in ("Noto Sans CJK JP", "Noto Sans CJK SC", "IPAGothic"):
    if any(_name == f.name for f in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _name
        break

DEFAULT_MAP = Path(__file__).resolve().parent.parent / "maps/grids/room_a_map_20260911.yaml"
OUT_DIR = Path.home() / "G1/_local/map_view"


def planner_view(grid: np.ndarray) -> np.ndarray:
    """白=走れる / 黒=障害物 / 薄橙=近すぎ / 水色=孤島 / 灰=未観測。"""
    occ, free = grid < 50, grid > 250
    infl = ndimage.binary_dilation(occ, ndimage.generate_binary_structure(2, 2), iterations=4)
    passable = free & ~infl
    lbl, n = ndimage.label(passable)
    sizes = ndimage.sum(passable, lbl, range(1, n + 1)) if n else np.array([0])
    main = lbl == (int(np.argmax(sizes)) + 1) if n else np.zeros_like(grid, bool)
    rgb = np.full((*grid.shape, 3), 200, np.uint8)
    rgb[free] = (255, 226, 160)
    rgb[passable & ~main] = (120, 190, 255)
    rgb[main] = (255, 255, 255)
    rgb[occ] = (0, 0, 0)
    return rgb


def main() -> int:
    ap = argparse.ArgumentParser(description="地図をクリックして印を付ける")
    ap.add_argument("map_yaml", nargs="?", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    grid = np.array(Image.open(args.map_yaml.parent / meta["image"]))
    h, w = grid.shape

    args.out_dir.mkdir(parents=True, exist_ok=True)
    marks: list[dict] = []

    fig, ax = plt.subplots(figsize=(9, 13))
    # ⚠️⚠️ **`np.flipud` を外さないこと**(2026-09-24 に間違えた)。
    # 地図の pgm は **保存時に上下反転されている**（map_server 慣例「原点=左下」に
    # 合わせるため、`pointcloud_to_occupancy_grid.py` が `np.flipud` して書く）。
    # つまり**配列の行0 は y が最大**。これを `origin="lower"` でそのまま描くと
    # **絵が上下逆になり、クリックで拾う y が鏡像になる**。
    # 実際、その状態で付けた印をもとに地図を編集し、**画面と違う場所を消した**。
    ax.imshow(np.flipud(planner_view(grid)), origin="lower",
              extent=[ox, ox + w * res, oy, oy + h * res], interpolation="nearest")
    ax.set_xticks(np.arange(np.floor(ox), ox + w * res, 1.0), minor=True)
    ax.set_yticks(np.arange(np.floor(oy), oy + h * res, 1.0), minor=True)
    ax.grid(which="minor", color="0.75", linewidth=0.4)
    ax.grid(which="major", color="0.3", linewidth=0.8)
    ax.set_title(f"{args.map_yaml.stem}  左クリック=印 / u=取り消し / s=保存")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    def save() -> None:
        (args.out_dir / "marks.json").write_text(
            json.dumps({"map": str(args.map_yaml), "marks": marks}, ensure_ascii=False, indent=2),
            encoding="utf-8")
        fig.savefig(args.out_dir / "marks.png", dpi=130, bbox_inches="tight")
        print(f"[mark] {len(marks)} 個を保存した → {args.out_dir}/marks.json")

    def on_click(event) -> None:
        if event.inaxes is not ax or event.button != 1:
            return
        if fig.canvas.toolbar.mode:      # 拡大/移動の最中はクリックを拾わない
            return
        marks.append({"n": len(marks) + 1, "x": round(float(event.xdata), 2),
                      "y": round(float(event.ydata), 2)})
        ax.plot(event.xdata, event.ydata, "o", ms=16, mfc="none", mec="red", mew=2.5)
        ax.annotate(str(len(marks)), (event.xdata, event.ydata), color="red",
                    fontsize=12, fontweight="bold", xytext=(8, 8), textcoords="offset points")
        print(f"[mark] {len(marks)}: x={event.xdata:.2f} y={event.ydata:.2f}")
        fig.canvas.draw_idle()

    def on_key(event) -> None:
        if event.key == "u" and marks:
            marks.pop()
            if ax.lines:
                ax.lines[-1].remove()
            if ax.texts:
                ax.texts[-1].remove()
            fig.canvas.draw_idle()
            print(f"[mark] 取り消した（残り {len(marks)}）")
        elif event.key == "s":
            save()

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    print("[mark] 左クリックで印。u=取り消し、s=保存。閉じても保存する")
    plt.show()
    save()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
