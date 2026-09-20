#!/usr/bin/env python3
"""3DGS と OctoMap の掃除結果を並べた図を作る。

作業ログに貼るための 4 枚。

1. 高さ帯ごとの除去率 — 追従者の帯（床上 0.85〜1.25m）だけが落ちているか
2. 上面図 — 軌跡の上の塊が消えて、机の島が残っているか
3. 描画距離の誤差 — 3DGS が実測にどれだけ合うようになったか（学習の前後）
4. 学習後の不透明度の分布 — 追従者の居た場所と静止構造で差がついたか

3 と 4 は 3DGS にしか無い指標である。OctoMap は log-odds の票なので
「描画してみて実測と合うか」を測れない。

    ../../Navigation/.venv/bin/python quickstart/plot_gs_comparison.py \\
        runs/20260904T183457_UiS_room_v2 --tag main
"""
from __future__ import annotations

import argparse
import glob
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.lidar_gs import (  # noqa: E402
    RenderConfig,
    init_from_points,
    read_splat_ply,
    render_rays,
    to_parameters,
)
from g1_mapping.pcd_io import read_pcd  # noqa: E402
from g1_mapping.rebuild import _CdrReader  # noqa: E402

ODOM_TOPIC = "/unitree/slam_mapping/odom"

# 作業ログの CSS と同じ意味色を使う（青=計測 / 緑=実証済み / 赤=不可 / 橙=注意）
COLOR_ACCENT = "#0f6fc4"
COLOR_GOOD = "#14714a"
COLOR_BAD = "#a8202c"
COLOR_WARN = "#b1500f"
COLOR_INK = "#111620"
COLOR_LINE = "#c3cbd7"

HEIGHT_EDGES = [-0.2, 0.25, 0.45, 0.65, 0.85, 1.05, 1.25, 1.5, 1.8, 2.2, 3.5]


def japanese_font() -> None:
    """図に日本語を出せるフォントを選ぶ。無ければ黙って既定のままにする。"""
    import matplotlib
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in ("Hiragino Sans", "Hiragino Kaku Gothic ProN",
                      "Noto Sans CJK JP", "IPAexGothic", "Arial Unicode MS"):
        if candidate in available:
            matplotlib.rcParams["font.family"] = candidate
            return


def read_trajectory(session: Path) -> np.ndarray:
    """内蔵 SLAM の odom を軌跡として読む（eval_removal.py と同じ読み方）。"""
    bag = sorted(glob.glob(str(session / "raw/rosbag2/*.db3")))[0]
    connection = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    try:
        topic = connection.execute(
            "SELECT id FROM topics WHERE name=?", (ODOM_TOPIC,)).fetchone()[0]
        poses = []
        for (payload,) in connection.execute(
                "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic,)):
            reader = _CdrReader(payload)
            reader.int32(), reader.uint32(), reader.string(), reader.string()
            values = []
            for _ in range(3):
                remainder = reader.position % 8
                if remainder:
                    reader.position += 8 - remainder
                values.append(struct.unpack_from("<d", reader._buffer, reader.position)[0])
                reader.position += 8
            poses.append(values)
    finally:
        connection.close()
    return np.array(poses)


def estimate_floor(points: np.ndarray) -> float:
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def survived(reference: np.ndarray, cleaned: np.ndarray) -> np.ndarray:
    from scipy.spatial import cKDTree
    distance, _ = cKDTree(cleaned).query(reference, workers=-1)
    return distance <= 1e-4


def plot_height_bands(reference, removals, floor_z, out: Path) -> None:
    """高さ帯ごとの除去率。追従者の帯だけが落ちていれば成功。"""
    import matplotlib.pyplot as plt

    heights = reference[:, 2] - floor_z
    labels, totals = [], []
    rates = {name: [] for name in removals}
    for low, high in zip(HEIGHT_EDGES[:-1], HEIGHT_EDGES[1:]):
        band = (heights >= low) & (heights < high)
        if not band.any():
            continue
        labels.append(f"{low:.2f}〜{high:.2f}")
        totals.append(int(band.sum()))
        for name, removed in removals.items():
            rates[name].append(100 * (band & removed).sum() / band.sum())

    figure, axes = plt.subplots(figsize=(10.5, 4.6))
    positions = np.arange(len(labels))
    width = 0.8 / len(rates)
    colors = [COLOR_LINE, COLOR_ACCENT, COLOR_GOOD, COLOR_WARN]
    for index, (name, values) in enumerate(rates.items()):
        axes.bar(positions + index * width - 0.4 + width / 2, values, width,
                 label=name, color=colors[index % len(colors)])

    # 人の胴体がいる帯を示す
    lower = HEIGHT_EDGES.index(0.85) - 0.5
    upper = HEIGHT_EDGES.index(1.5) - 0.5
    axes.axvspan(lower, upper, color=COLOR_BAD, alpha=0.07, zorder=0)
    axes.text((lower + upper) / 2, 101, "追従者の胴体", ha="center", va="bottom",
              fontsize=9, color=COLOR_BAD)

    axes.set_xticks(positions)
    axes.set_xticklabels([f"{label}\n{total:,}" for label, total in zip(labels, totals)],
                         fontsize=8)
    axes.set_xlabel("床上の高さ [m] と、その帯にある元の点数")
    axes.set_ylabel("除去率 [%]")
    axes.set_ylim(0, 108)
    axes.legend(frameon=False, ncol=len(rates), loc="upper left", fontsize=9)
    axes.spines[["top", "right"]].set_visible(False)
    axes.grid(axis="y", color=COLOR_LINE, alpha=0.4, linewidth=0.6)
    axes.set_axisbelow(True)
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


def plot_top_views(clouds, trajectory, floor_z, out: Path) -> None:
    """床上 0.65〜1.25m だけを上から見る。追従者と机が同居する帯。"""
    import matplotlib.pyplot as plt

    figure, axes_list = plt.subplots(1, len(clouds), figsize=(4.0 * len(clouds), 4.3),
                                     sharex=True, sharey=True)
    for axes, (name, points) in zip(np.atleast_1d(axes_list), clouds.items()):
        heights = points[:, 2] - floor_z
        band = (heights >= 0.65) & (heights < 1.25)
        axes.scatter(points[band, 0], points[band, 1], s=0.12, c=COLOR_INK,
                     alpha=0.35, linewidths=0)
        axes.plot(trajectory[:, 0], trajectory[:, 1], color=COLOR_WARN, linewidth=1.4,
                  alpha=0.95, label="ロボットの軌跡")
        axes.set_title(f"{name}（{int(band.sum()):,} 点）", fontsize=10)
        axes.set_aspect("equal")
        axes.spines[["top", "right"]].set_visible(False)
        axes.tick_params(labelsize=8)
    span = trajectory[:, :2]
    axes_list[0].set_xlim(span[:, 0].min() - 6, span[:, 0].max() + 6)
    axes_list[0].set_ylim(span[:, 1].min() - 6, span[:, 1].max() + 6)
    axes_list[0].set_ylabel("Y [m]")
    axes_list[0].legend(frameon=False, fontsize=8, loc="upper left")
    figure.suptitle("床上 0.65〜1.25 m の上面図（人の腰と机の天板が同居する帯）", fontsize=11)
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


def plot_render_error(before: np.ndarray, after: np.ndarray, out: Path) -> None:
    """描画した距離と実測の差。3DGS にしか無い指標。"""
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(8.2, 4.0))
    bins = np.linspace(0, 6, 120)
    axes.hist(np.clip(before, 0, 6), bins=bins, color=COLOR_LINE, label="学習前（地図そのまま）")
    axes.hist(np.clip(after, 0, 6), bins=bins, color=COLOR_ACCENT, alpha=0.85,
              label="学習後")
    axes.axvline(np.median(before), color=COLOR_INK, linestyle=":", linewidth=1.2)
    axes.axvline(np.median(after), color=COLOR_ACCENT, linestyle=":", linewidth=1.2)
    axes.set_xlabel("|描画した距離 − 実測距離| [m]（6 m 以上は 6 m に丸めた）")
    axes.set_ylabel("光線の本数")
    axes.set_yscale("log")
    axes.legend(frameon=False, fontsize=9)
    axes.spines[["top", "right"]].set_visible(False)
    axes.set_title(f"中央値 {np.median(before):.2f} m → {np.median(after):.2f} m", fontsize=10)
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


def plot_opacity(regions: "dict[str, np.ndarray]", survival: "dict[str, str]",
                 initial_opacity: float, out: Path) -> None:
    """領域ごとの、学習後の不透明度の分布。仕組みが効いたかを直接見る。

    刈られたガウシアンはもう存在しないので、これは**生き残った分だけ**の分布である。
    そのため凡例に生存率を併記する。
    """
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(9.0, 4.2))
    bins = np.linspace(0, 1, 60)
    colors = [COLOR_BAD, COLOR_GOOD, COLOR_ACCENT, COLOR_WARN]
    for index, (name, values) in enumerate(regions.items()):
        if len(values) == 0:
            continue
        axes.hist(values, bins=bins, histtype="step", linewidth=1.8, density=True,
                  color=colors[index % len(colors)],
                  label=f"{name}  生存 {survival.get(name, '—')}・中央値 {np.median(values):.2f}")
    axes.axvline(initial_opacity, color=COLOR_INK, linestyle=":", linewidth=1.2)
    axes.annotate("初期値のまま＝光線が一度も\n当たっていない（OctoMap の「未知」）",
                  xy=(initial_opacity, axes.get_ylim()[1] * 0.72),
                  xytext=(initial_opacity + 0.09, axes.get_ylim()[1] * 0.78),
                  fontsize=8.5, color=COLOR_INK,
                  arrowprops=dict(arrowstyle="->", color=COLOR_INK, lw=0.9))
    axes.set_xlabel("学習後の不透明度 σ(opacity)　※生き残ったガウシアンのみ")
    axes.set_ylabel("密度")
    axes.legend(frameon=False, fontsize=9, loc="upper right")
    axes.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


def camera_rays(position: np.ndarray, target: np.ndarray, width: int, height: int,
                fov_degrees: float = 75.0) -> np.ndarray:
    """ピンホールカメラの光線方向を作る。3DGS の出力を絵にするためだけに使う。

    学習に使う LiDAR の光線は 360° なのでピンホールでは表せないが、
    「出来上がったガウシアン場を人が見る」ぶんには普通のカメラでよい。
    """
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


def render_view(params, position: np.ndarray, target: np.ndarray,
                width: int, height: int, device: str, chunk: int = 40_000):
    """1 視点ぶんの (距離画像, 不透明度画像) を返す。"""
    import torch

    config = RenderConfig(720, 180, 48, sigma=2.0)
    origin = torch.tensor(position, dtype=torch.float32, device=device)
    directions = torch.from_numpy(camera_rays(position, target, width, height)).to(device)
    depths, accumulations = [], []
    with torch.no_grad():
        for start in range(0, len(directions), chunk):
            block = directions[start:start + chunk]
            depth, acc, _ = render_rays(params, origin, block, config)
            depths.append((depth / acc.clamp_min(0.1)).cpu().numpy())
            accumulations.append(acc.cpu().numpy())
            if device == "mps":
                torch.mps.empty_cache()
    return (np.concatenate(depths).reshape(height, width),
            np.concatenate(accumulations).reshape(height, width))


def plot_novel_views(session: Path, reference: np.ndarray, splat, trajectory,
                     floor_z: float, out: Path, width: int = 420, height: int = 300) -> None:
    """学習前後のガウシアン場を、同じ仮想カメラから描画して並べる。

    これが「3DGS を当てた結果」そのものである。点群と違って面として埋まる。
    """
    import matplotlib.pyplot as plt
    import torch

    device = ("mps" if torch.backends.mps.is_available()
              else "cuda" if torch.cuda.is_available() else "cpu")
    initial = to_parameters(init_from_points(reference, init_opacity=0.3), device)
    trained = to_parameters({k: np.ascontiguousarray(v) for k, v in splat.items()}, device)

    centre = trajectory[:, :2].mean(axis=0)
    eye = np.array([centre[0], centre[1], floor_z + 1.3], dtype=np.float32)
    views = {
        "室内を見わたす": (eye, np.array([centre[0] + 6.0, centre[1] + 2.0, floor_z + 1.0])),
        "軌跡の上（追従者が居た所）": (
            np.array([centre[0], centre[1], floor_z + 2.6], dtype=np.float32),
            np.array([centre[0] + 0.5, centre[1] + 0.5, floor_z])),
    }

    figure, axes_grid = plt.subplots(len(views), 2, figsize=(11.0, 3.6 * len(views)))
    axes_grid = np.atleast_2d(axes_grid)
    for row, (name, (position, target)) in enumerate(views.items()):
        rendered = {}
        for label, params in (("学習前（地図そのまま）", initial), ("学習後の 3DGS", trained)):
            depth, acc = render_view(params, position, target, width, height, device)
            rendered[label] = np.where(acc > 0.3, depth, np.nan)
        # 左右で色スケールをそろえる。別々にすると「学習前も色が付いている」ように
        # 見えてしまうが、実際には学習前はすべて 1 m 以内で潰れている
        ceiling = float(np.nanpercentile(rendered["学習後の 3DGS"], 97))
        for column, (label, shown) in enumerate(rendered.items()):
            axes = axes_grid[row, column]
            image = axes.imshow(shown, cmap="turbo", vmin=0.5, vmax=ceiling)
            near = float(np.nanmedian(shown))
            axes.set_title(f"{name} — {label}（距離の中央値 {near:.1f} m）", fontsize=9.5)
            axes.set_xticks([]); axes.set_yticks([])
            figure.colorbar(image, ax=axes, fraction=0.032, label="距離 [m]")
    figure.suptitle("ガウシアン場を仮想カメラから描画したもの（色は距離）", fontsize=11)
    figure.tight_layout()
    figure.savefig(out, dpi=140)
    plt.close(figure)
    print(f"書いた: {out}")


def measure_render_error(session: Path, reference: np.ndarray,
                         splat: "dict[str, np.ndarray]", frames: int = 12
                         ) -> "dict[str, np.ndarray]":
    """held-out のスキャンを、学習前（地図そのまま）と学習後の両方で描画して誤差を測る。

    OctoMap には無い評価軸である。占有格子は「描画してみて実測と合うか」を測れない。
    """
    import torch

    device = ("mps" if torch.backends.mps.is_available()
              else "cuda" if torch.cuda.is_available() else "cpu")
    config = RenderConfig(720, 180, 48, sigma=2.0)
    initial = to_parameters(init_from_points(reference, init_opacity=0.3), device)
    trained = to_parameters({k: np.ascontiguousarray(v) for k, v in splat.items()}, device)

    files = sorted((session / "benchmark" / "pcd").glob("*.pcd"))
    chosen = [files[i] for i in np.linspace(0, len(files) - 1, frames).astype(int)]
    collected = {"before": [], "after": []}
    with torch.no_grad():
        for path in chosen:
            data = read_pcd(path)
            origin = torch.tensor(data.origin, dtype=torch.float32, device=device)
            offsets = torch.from_numpy(
                (data.points - data.origin).astype(np.float32)).to(device)
            truth = offsets.norm(dim=-1)
            keep = truth > 0.1
            offsets, truth = offsets[keep], truth[keep]
            for name, params in (("before", initial), ("after", trained)):
                depth, acc, _ = render_rays(params, origin, offsets, config)
                error = (depth / acc.clamp_min(0.1) - truth).abs()
                collected[name].append(error.cpu().numpy())
            if device == "mps":
                torch.mps.empty_cache()
    print(f"描画誤差: {frames} 枚 / {sum(len(x) for x in collected['before']):,} 本で測った")
    return {name: np.concatenate(values) for name, values in collected.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description="3DGS と OctoMap の比較図を作る")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--tag", default="main")
    parser.add_argument("--out-dir", type=Path,
                        default=Path("../../../../docs/作業ログ/images"))
    parser.add_argument("--prefix", default="2026-09-05_3DGS")
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    japanese_font()

    session = args.session_dir
    out_dir = (Path(__file__).resolve().parent / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    reference = read_pcd(session / "map/map_raw.pcd").points
    floor_z = estimate_floor(reference)
    trajectory = read_trajectory(session)
    print(f"元の地図 {len(reference):,} 点  床 z={floor_z:.3f} m")

    named = {
        "自前フィルタ": "map_clean.pcd",
        "OctoMap 3 m": "map_octomap_r3.pcd",
        "3DGS": "map_3dgs.pcd",
    }
    removals, clouds = {}, {"加工前": reference}
    for name, filename in named.items():
        path = session / "map" / filename
        if not path.exists():
            print(f"  飛ばす（無い）: {path}")
            continue
        points = read_pcd(path).points
        clouds[name] = points
        removals[name] = ~survived(reference, points)
        print(f"  {name}: {len(points):,} 点（{100*removals[name].mean():.1f}% 除去）")

    plot_height_bands(reference, removals, floor_z,
                      out_dir / f"{args.prefix}_01_高さ帯ごとの除去率.png")
    plot_top_views(clouds, trajectory, floor_z,
                   out_dir / f"{args.prefix}_02_上面図.png")

    # ── 3DGS にしか無い 2 枚 ──
    ply = session / "gs" / f"gaussians_{args.tag}.ply"
    if not ply.exists():
        print(f"3DGS の .ply が無いので図 3・4 は飛ばす: {ply}")
        return
    splat = read_splat_ply(ply)
    opacity = 1.0 / (1.0 + np.exp(-splat["opacities"]))

    from scipy.spatial import cKDTree
    heights = splat["means"][:, 2] - floor_z
    to_path, _ = cKDTree(trajectory[:, :2]).query(splat["means"][:, :2], workers=-1)
    human = (heights >= 0.25) & (heights < 1.8)
    groups = {
        "歩いた体積（動的のはず）": human & (to_path < 0.5),
        "遠方の構造（静的のはず）": human & (to_path >= 4.0),
        "天井・上部": heights >= 2.2,
    }
    # 同じ区分を元の地図の上でも作り、生存率を出す（刈られた分は .ply に無いため）
    ref_heights = reference[:, 2] - floor_z
    ref_to_path, _ = cKDTree(trajectory[:, :2]).query(reference[:, :2], workers=-1)
    ref_human = (ref_heights >= 0.25) & (ref_heights < 1.8)
    ref_groups = {
        "歩いた体積（動的のはず）": ref_human & (ref_to_path < 0.5),
        "遠方の構造（静的のはず）": ref_human & (ref_to_path >= 4.0),
        "天井・上部": ref_heights >= 2.2,
    }
    kept = ~removals["3DGS"] if "3DGS" in removals else np.ones(len(reference), bool)
    survival = {name: f"{100 * (mask & kept).sum() / max(int(mask.sum()), 1):.0f} %"
                for name, mask in ref_groups.items()}
    plot_opacity({name: opacity[mask] for name, mask in groups.items()}, survival,
                 0.3, out_dir / f"{args.prefix}_04_不透明度の分布.png")

    cache = session / "gs" / f"render_error_{args.tag}.npz"
    if cache.exists():
        data = np.load(cache)
    else:
        data = measure_render_error(session, reference, splat)
        np.savez_compressed(cache, **data)
    plot_render_error(data["before"], data["after"],
                      out_dir / f"{args.prefix}_03_描画誤差.png")
    plot_novel_views(session, reference, splat, trajectory, floor_z,
                     out_dir / f"{args.prefix}_05_ガウシアン場の描画.png")


if __name__ == "__main__":
    main()
