#!/usr/bin/env python3
"""反射強度を「色」として学習した 3DGS の出力を絵にする。

距離だけで学習したもの（`--tag main`）と、反射強度も入れたもの（`--tag intensity`）を
同じ視点から描いて並べる。反射強度つきのほうは**グレースケールの写真のように見える**——
これが「色として使えた場合のアウトプット」である。

    ../../Navigation/.venv/bin/python quickstart/plot_gs_intensity.py \\
        runs/20260904T183457_UiS_room_v2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.lidar_gs import (  # noqa: E402
    INTENSITY_SCALE,
    RenderConfig,
    read_splat_ply,
    render_rays,
    to_parameters,
)
from g1_mapping.pcd_io import read_pcd  # noqa: E402

CONFIG = RenderConfig(720, 180, 48, sigma=2.0)


def japanese_font() -> None:
    import matplotlib
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in ("Hiragino Sans", "Hiragino Kaku Gothic ProN",
                      "Noto Sans CJK JP", "IPAexGothic", "Arial Unicode MS"):
        if candidate in available:
            matplotlib.rcParams["font.family"] = candidate
            return


def camera_rays(position: np.ndarray, target: np.ndarray, width: int, height: int,
                fov_degrees: float = 75.0) -> np.ndarray:
    forward = target - position
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right = right / np.linalg.norm(right)
    up = np.cross(right, forward)
    extent = np.tan(np.radians(fov_degrees) / 2)
    xs = np.linspace(-extent * width / height, extent * width / height, width)
    ys = np.linspace(extent, -extent, height)
    grid_x, grid_y = np.meshgrid(xs, ys)
    directions = (forward[None, None, :]
                  + grid_x[..., None] * right[None, None, :]
                  + grid_y[..., None] * up[None, None, :])
    return directions.reshape(-1, 3).astype(np.float32)


def render(params, position, target, width, height, device, chunk=30_000):
    """(距離, 不透明度, 反射強度) の画像を返す。色が無いモデルなら強度は None。"""
    has_color = "colors" in params
    origin = torch.tensor(position, dtype=torch.float32, device=device)
    directions = torch.from_numpy(camera_rays(position, target, width, height)).to(device)
    depths, accumulations, colours = [], [], []
    with torch.no_grad():
        for start in range(0, len(directions), chunk):
            block = directions[start:start + chunk]
            if has_color:
                depth, acc, _, colour = render_rays(params, origin, block, CONFIG,
                                                    with_color=True)
                colours.append((colour / acc.clamp_min(0.1)).cpu().numpy())
            else:
                depth, acc, _ = render_rays(params, origin, block, CONFIG)
            depths.append((depth / acc.clamp_min(0.1)).cpu().numpy())
            accumulations.append(acc.cpu().numpy())
            if device == "mps":
                torch.mps.empty_cache()
    shape = (height, width)
    return (np.concatenate(depths).reshape(shape),
            np.concatenate(accumulations).reshape(shape),
            np.concatenate(colours).reshape(shape) * INTENSITY_SCALE if has_color else None)


def main() -> None:
    parser = argparse.ArgumentParser(description="反射強度つき 3DGS の出力を描く")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--depth-tag", default="main")
    parser.add_argument("--intensity-tag", default="intensity")
    parser.add_argument("--width", type=int, default=520)
    parser.add_argument("--height", type=int, default=380)
    parser.add_argument("--out", type=Path, default=Path(
        "../../../../docs/作業ログ/images/2026-09-05_3DGS_06_反射強度を色にした出力.png"))
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    japanese_font()
    import matplotlib.pyplot as plt

    device = ("mps" if torch.backends.mps.is_available()
              else "cuda" if torch.cuda.is_available() else "cpu")
    session = args.session_dir
    models = {}
    for label, tag in (("距離だけで学習", args.depth_tag),
                       ("反射強度も入れて学習", args.intensity_tag)):
        path = session / "gs" / f"gaussians_{tag}.ply"
        if not path.exists():
            raise SystemExit(f"見つかりません: {path}")
        splat = read_splat_ply(path)
        models[label] = to_parameters(
            {k: np.ascontiguousarray(v) for k, v in splat.items()}, device)
        print(f"{label}: {len(splat['means']):,} 個  色={'あり' if 'colors' in splat else 'なし'}")

    floor = -1.303
    poses = np.loadtxt(session / "benchmark/poses.txt")[:, 1:4]
    centre = poses[:, :2].mean(axis=0)
    views = {
        "室内を見わたす": (np.array([centre[0], centre[1], floor + 1.30]),
                    np.array([centre[0] + 6.0, centre[1] + 2.0, floor + 1.0])),
        "センサが通っていない視点（真横から）":
            (np.array([centre[0] - 4.0, centre[1] - 3.0, floor + 2.20]),
             np.array([centre[0] + 1.0, centre[1] + 1.0, floor + 0.6])),
    }

    figure, axes_grid = plt.subplots(len(views), 3, figsize=(14.5, 3.6 * len(views)))
    axes_grid = np.atleast_2d(axes_grid)
    for row, (name, (position, target)) in enumerate(views.items()):
        depth, acc, _ = render(models["距離だけで学習"], position, target,
                               args.width, args.height, device)
        _, acc_i, colour = render(models["反射強度も入れて学習"], position, target,
                                  args.width, args.height, device)
        visible = acc > 0.3
        panels = [
            ("距離だけの 3DGS — 距離", np.where(visible, depth, np.nan), "turbo",
             "距離 [m]", 0.5, float(np.nanpercentile(depth[visible], 97))),
            ("反射強度つき 3DGS — 距離", np.where(acc_i > 0.3, depth, np.nan), "turbo",
             "距離 [m]", 0.5, float(np.nanpercentile(depth[visible], 97))),
            ("反射強度つき 3DGS — 反射強度", np.where(acc_i > 0.3, colour, np.nan), "gray",
             "反射強度", 0.0, float(np.nanpercentile(colour[acc_i > 0.3], 98))),
        ]
        for column, (title, image, cmap, label, low, high) in enumerate(panels):
            axes = axes_grid[row, column]
            # 「どのガウシアンも支えていない穴」を、高い値の白と取り違えないように
            # 別の色（薄い青）で塗る。グレースケールだと白同士で見分けられない
            palette = matplotlib.colormaps[cmap].with_extremes(bad="#9db4c8")
            drawn = axes.imshow(image, cmap=palette, vmin=low, vmax=high)
            axes.set_title(f"{name}\n{title}", fontsize=9.5)
            axes.set_xticks([]); axes.set_yticks([])
            figure.colorbar(drawn, ax=axes, fraction=0.034, label=label)
    figure.suptitle("反射強度を「色」として学習させた 3DGS の出力"
                    "（薄い青 = どのガウシアンも支えていない穴）", fontsize=11.5)
    figure.tight_layout()
    out = (Path(__file__).resolve().parent / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


if __name__ == "__main__":
    main()
