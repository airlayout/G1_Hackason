"""指紋の推定結果を地図に重ねて PNG に描く（localize.py から呼ぶ）。

- 背景: `nav_map_run`（白=自由、黒=占有、灰=未知）
- 小さな灰の点: 台帳の地点（合成スキャンを撃った所）
- 番号つきの矢印: 指紋の上位候補を ICP で詰めた後の base_link。色は fitness
- 赤の矢印と点: 採用した姿勢と、その姿勢で置いたスキャン（高さの帯）
- 緑の矢印: 比べる姿勢（タグや 2D 総当たり。与えたときだけ）
"""
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from fingerprint.build_db import load_nav_map  # noqa: E402
from jp_font import japanese_font  # noqa: E402

ARROW_M = 0.8
VIEW_MARGIN_M = 1.5


SAME_POSE_M = 0.3
SAME_POSE_DEG = 10.0


def group_results(results: list) -> list:
    """ICP で同じ姿勢に収まった候補をまとめる（番号が重なって読めなくなるのを防ぐ）。"""
    groups = []
    for r in sorted(results, key=lambda item: item["rank"]):
        for group in groups:
            head = group[0]
            dyaw = abs((r["base_yaw_deg"] - head["base_yaw_deg"] + 180) % 360 - 180)
            if (math.hypot(r["base_x"] - head["base_x"], r["base_y"] - head["base_y"]) < SAME_POSE_M
                    and dyaw < SAME_POSE_DEG):
                group.append(r)
                break
        else:
            groups.append([r])
    return groups


def rank_label(group: list) -> str:
    ranks = [r["rank"] for r in group]
    if len(ranks) == 1:
        return str(ranks[0])
    if ranks == list(range(ranks[0], ranks[-1] + 1)):
        return "%d-%d" % (ranks[0], ranks[-1])
    return ",".join(map(str, ranks))


def arrow(ax, x: float, y: float, yaw_deg: float, color: str, width: float) -> None:
    yaw = math.radians(yaw_deg)
    ax.annotate("", xy=(x + ARROW_M * math.cos(yaw), y + ARROW_M * math.sin(yaw)),
                xytext=(x, y),
                arrowprops={"arrowstyle": "-|>", "color": color, "lw": width})


def draw_result(nav_yaml: Path, meta: dict, db_xy: np.ndarray, points: np.ndarray, results: list,
                verdict: dict, truth, truth_label: str, out_png: Path) -> None:
    if japanese_font() is None:
        print("⚠️ 日本語フォントが無い。図の文字が化ける")
    grid, res, origin = load_nav_map(nav_yaml)
    height, width = grid.shape
    extent = (origin[0], origin[0] + width * res, origin[1], origin[1] + height * res)
    shade = np.where(grid >= 250, 1.0, np.where(grid <= 50, 0.0, 0.8))

    fig, axes = plt.subplots(1, 2, figsize=(16, 8.5), gridspec_kw={"width_ratios": [1.25, 1]})
    for ax in axes:
        ax.imshow(shade, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent)
        ax.set_aspect("equal")

    best = verdict["best"]
    yaw = math.radians(best["base_yaw_deg"])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    z = points[:, 2] + meta["sensor_height_m"]
    band = points[(z > 0.3) & (z < 1.9)]
    from fingerprint.localize import LIVOX_XYZ
    with np.errstate(all="ignore"):   # macOS の Accelerate が出す見かけの警告を黙らせる
        scan_xy = (band[:, :2] + LIVOX_XYZ[:2]) @ rotation.T + (best["base_x"], best["base_y"])

    for panel, ax in enumerate(axes):
        ax.scatter(db_xy[:, 0], db_xy[:, 1], s=1.5, c="#999999", label="台帳の地点")
        ax.scatter(scan_xy[::7, 0], scan_xy[::7, 1], s=0.6, c="#d62728", alpha=0.5,
                   label="採用姿勢に置いたスキャン（高さ 0.3〜1.9 m）")
        fits = np.array([r["fitness"] for r in results])
        norm = matplotlib.colors.Normalize(vmin=max(0.0, fits.min() - 0.05), vmax=fits.max())
        cmap = matplotlib.colormaps["viridis"]
        for group in group_results(results):
            head = group[0]
            color = cmap(norm(head["fitness"]))
            arrow(ax, head["base_x"], head["base_y"], head["base_yaw_deg"], color, 1.6)
            ax.text(head["base_x"] + 0.15, head["base_y"] + 0.15, rank_label(group), fontsize=9,
                    color="black", bbox={"fc": color, "ec": "none", "alpha": 0.8, "pad": 1})
        arrow(ax, best["base_x"], best["base_y"], best["base_yaw_deg"], "#d62728", 3.2)
        ax.plot(best["base_x"], best["base_y"], "o", color="#d62728", ms=8,
                label="採用 (%.2f, %.2f, %.1f°)" % (best["base_x"], best["base_y"],
                                                   best["base_yaw_deg"]))
        if truth is not None:
            arrow(ax, truth[0], truth[1], truth[2], "#2ca02c", 3.2)
            ax.plot(truth[0], truth[1], "s", color="#2ca02c", ms=8,
                    label="%s (%.2f, %.2f, %.1f°)" % (truth_label, *truth))
        if panel == 0:
            # ⚠️ 占有セルで範囲を取ると、地図の外れ値（x 60 m 付近）まで入って部屋が小さく写る
            free = np.argwhere(grid >= 250)
            ys = origin[1] + free[:, 0] * res
            xs = origin[0] + free[:, 1] * res
            ax.set_xlim(xs.min() - VIEW_MARGIN_M, xs.max() + VIEW_MARGIN_M)
            ax.set_ylim(ys.min() - VIEW_MARGIN_M, ys.max() + VIEW_MARGIN_M)
            ax.set_title("地図全体（番号 = 指紋の順位、色 = ICP の fitness）")
            sm = matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap)
            fig.colorbar(sm, ax=ax, fraction=0.035, label="ICP fitness（0.15 m 以内の点の割合）")
        else:
            ax.set_xlim(best["base_x"] - 4, best["base_x"] + 4)
            ax.set_ylim(best["base_y"] - 4, best["base_y"] + 4)
            ax.set_title("採用姿勢のまわり ±4 m")
            ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
        ax.set_xlabel("x [m]（map）")
        ax.set_ylabel("y [m]")

    # ⚠️ ✔ は Hiragino に無く豆腐になる
    status = "【採用】" if verdict["confident"] else "【自信なし】"
    fig.suptitle(status + verdict["message"].replace("採用: ", ""), fontsize=12,
                 color="black" if verdict["confident"] else "#b35900")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
