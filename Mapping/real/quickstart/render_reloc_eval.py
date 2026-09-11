#!/usr/bin/env python3
"""段 8 の各パターンを **1 本ずつ動画**にし、まとめの HTML を書く。

## なぜ動画なのか

数字の表だけでは「本当に戻ったのか」をユーザーが確かめられない（2026-09-11 の指示）。
1 パターン = 1 ファイルにする（`video-one-file-per-run`。繋げると見たい回まで早送りが要る）。

## 1 本に映すもの

- 左: 事前地図（旧 `nav_map`）の上に **信じていた姿勢（赤）/ 返った姿勢（青）/ 真値（緑）**、
  および**その姿勢に置いたライブ点群**（緑＝占有セルに乗った / 赤＝外れた）
- 右: **尤度の格子**（`reloc_probe --dump-grid`。探索が何を見て決めたか）
- 下: 経過・重畳 %・r/r0・所要秒・判定を常時

## 使い方

    Navigation/.venv/bin/python quickstart/render_reloc_eval.py \\
        runs/reloc_eval_20260911 [--fps 15] [--only 1,2]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jp_font import japanese_font         # noqa: E402
from measure_overlay import read_map      # noqa: E402
from overlay_at_pose import BAND_HI, BAND_LO  # noqa: E402

DPI = 110
SECONDS = (3.0, 3.0, 3.0, 5.0, 6.0)      # 段: 信じていた / 尤度 / 返った / 移る / 判定
STAGE_LABEL = ("① 門番が鳴った時点：信じていた姿勢",
               "② 探索：尤度の格子を評価（全点）",
               "③ 探索が返した姿勢",
               "④ その姿勢に点群を置き直す",
               "⑤ 二重ゲートの判定")


def cloud_xy(pts: np.ndarray, pose) -> np.ndarray:
    """base_link 系の点を pose(x, y, yaw_deg) に置いた map 系の xy。"""
    x, y, th = pose
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1],
                            y + s * pts[:, 0] + c * pts[:, 1]])


def hits(xy: np.ndarray, occ, res, ox, oy) -> np.ndarray:
    """占有セルに乗ったか（overlay_at_pose と同じ規約。上下反転を忘れない）。"""
    h, w = occ.shape
    ix = np.floor((xy[:, 0] - ox) / res).astype(int)
    iy = np.floor((xy[:, 1] - oy) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    out = np.zeros(len(xy), bool)
    out[ok] = occ[h - 1 - iy[ok], ix[ok]]
    return out


def read_grid(path: Path):
    """reloc_probe --dump-grid の出力。戻り値は (値, extent)。"""
    lines = path.read_text().splitlines()
    xmin, ymin, res, nx, ny = lines[1].split()
    xmin, ymin, res = float(xmin), float(ymin), float(res)
    nx, ny = int(nx), int(ny)
    g = np.array([[float(v) for v in ln.split()] for ln in lines[2:2 + ny]])
    return g, (xmin, xmin + nx * res, ymin, ymin + ny * res)


def render_one(r: dict, meta: dict, base: Path, occ, res, ox, oy, fps: int) -> Path:
    pts_all = np.loadtxt(base / r["scan"])
    pts = pts_all[(pts_all[:, 2] >= BAND_LO) & (pts_all[:, 2] <= BAND_HI)]
    # ⚠️ 全点描くと 1 フレーム数秒かかる。見た目は変わらないので間引く
    if len(pts) > 4000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 4000, replace=False)]

    believed = list(r["believed"]) + [0.0]
    returned = r["pose"]
    truth = r.get("truth")
    grid, extent = read_grid(base / "pattern{}.grid".format(r["id"]))

    # 見せる範囲: 姿勢と点群が入るように
    xs = [believed[0], returned[0]] + ([truth[0]] if truth else [])
    ys = [believed[1], returned[1]] + ([truth[1]] if truth else [])
    pad = 9.0
    xlim = (min(xs) - pad, max(xs) + pad)
    ylim = (min(ys) - pad, max(ys) + pad)

    fig = plt.figure(figsize=(13.5, 6.4))
    # ⚠️ 文字の行が地図の目盛りと重なるので、下の段をしっかり空ける
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 0.22], width_ratios=[1.35, 1],
                          hspace=0.40, wspace=0.14)
    axm, axg, axt = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])
    axt.axis("off")

    h, w = occ.shape
    axm.imshow(np.flipud(occ), origin="lower", cmap="gray_r", vmin=0, vmax=3.2,
               extent=(ox, ox + w * res, oy, oy + h * res), interpolation="nearest")
    axm.set_xlim(*xlim); axm.set_ylim(*ylim); axm.set_aspect("equal")
    axm.set_xlabel("map x [m]", fontsize=8); axm.set_ylabel("map y [m]", fontsize=8)
    axm.tick_params(labelsize=7)

    im = axg.imshow(grid, origin="lower", extent=extent, cmap="viridis",
                    interpolation="nearest")
    fig.colorbar(im, ax=axg, fraction=0.046, pad=0.02).ax.tick_params(labelsize=7)
    axg.set_title("尤度の格子（探索が見たもの。phi 方向の最大）", fontsize=9)
    axg.set_xlabel("map x [m]", fontsize=8); axg.tick_params(labelsize=7)
    axg.set_aspect("equal")

    # 緑（当たり）は地図の占有セル（灰）に重なって埋もれるので、少し大きく・前面に
    scat = axm.scatter([], [], s=2.6, c="none", linewidths=0, zorder=3)
    mk_b, = axm.plot([], [], "o", ms=11, mfc="none", mec="#d62728", mew=2.4, label="信じていた姿勢")
    mk_r, = axm.plot([], [], "o", ms=11, mfc="none", mec="#1f77b4", mew=2.4, label="探索が返した姿勢")
    mk_t, = axm.plot([], [], "*", ms=15, color="#2ca02c", label="真値")
    axm.legend(loc="upper right", fontsize=8, framealpha=0.9)
    title = axm.set_title("", fontsize=10)
    # ⚠️ family="monospace" にしない。日本語の等幅は macOS に無く、Menlo が選ばれて
    #    日本語が全部豆腐になる（jp_font.japanese_mono() の注記）。桁は書式で揃える
    txt = axt.text(0.005, 1.02, "", va="top", ha="left", fontsize=10.5,
                   transform=axt.transAxes, linespacing=1.6)

    bounds = np.cumsum(SECONDS)
    total = bounds[-1]
    frames = int(total * fps)
    out = base / "pattern{}_{}.mp4".format(r["id"], r["name"][:10].replace(" ", "").replace("/", ""))
    writer = FFMpegWriter(fps=fps, bitrate=3600, metadata={"title": r["name"]})

    with writer.saving(fig, str(out), DPI):
        for k in range(frames):
            t = k / fps
            stage = int(np.searchsorted(bounds, t, side="right"))
            stage = min(stage, len(SECONDS) - 1)
            # 点群をどの姿勢に置くか（④ で信じていた -> 返った へ動かす）
            if stage < 3:
                pose = believed
            elif stage == 3:
                u = (t - bounds[2]) / SECONDS[3]
                u = min(max(u, 0.0), 1.0)
                pose = [believed[i] + u * (returned[i] - believed[i]) for i in range(2)] + \
                       [u * returned[2]]
            else:
                pose = returned
            xy = cloud_xy(pts, pose)
            hit = hits(xy, occ, res, ox, oy)
            scat.set_offsets(xy)
            scat.set_color(np.where(hit, "#00b050", "#ff2a2a"))
            mk_b.set_data([believed[0]], [believed[1]])
            mk_r.set_data(([returned[0]], [returned[1]]) if stage >= 2 else ([], []))
            mk_t.set_data(([truth[0]], [truth[1]]) if (truth and stage >= 2) else ([], []))
            im.set_alpha(1.0 if stage >= 1 else 0.0)
            title.set_text("パターン {} — {}\n{}".format(r["id"], r["name"], STAGE_LABEL[stage]))

            now = 100.0 * hit.mean()
            lines = [
                "経過 {:4.1f}s / {:.0f}s     いま置いている姿勢の重畳 {:5.1f} %".format(t, total, now),
                "ゲート①3D 残差 r/r0 = {:.2f}（しきい {:.1f}）      一致率 {:.1f} %".format(
                    r["ratio"], r["gate_n"], 100 * r["match_rate"]),
                "ゲート②2D 重畳 {:.1f} % = 真値基準の {:.2f}（しきい {:.2f}）   探索 {:.2f} s".format(
                    r["overlay_pct"], r["overlay_frac"], meta["overlay_frac"], r["search_seconds"]),
            ]
            if stage >= 4:
                lines.append("判定: {}".format(r["final"]))
            if truth:
                lines[0] += "     真値との差 {:.3f} m / {:.2f}°".format(
                    r.get("error_m", float("nan")), r.get("error_deg", float("nan")))
            txt.set_text("\n".join(lines))
            txt.set_color("#111111" if stage < 4 or r["gate3d"] and r["gate2d"] else "#b00000")
            writer.grab_frame()
    plt.close(fig)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", type=Path, help="eval_reloc_patterns.py --out で指した場所")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--only", default=None, help="パターン id をカンマ区切りで")
    a = ap.parse_args()

    if japanese_font() is None:
        print("⚠️ 日本語フォントが見つからない。見出しが豆腐になる", file=sys.stderr)

    base = a.dir if a.dir.is_absolute() else Path.cwd() / a.dir
    meta = json.loads((base / "patterns.json").read_text())
    ref = base.parents[0] / "20260906T135940_UiS_room_v3" / meta["ref_map"]
    occ, res, ox, oy = read_map(ref)

    only = set(a.only.split(",")) if a.only else None
    made = []
    for r in meta["results"]:
        if only and r["id"] not in only:
            continue
        print("  パターン {} ({}) …".format(r["id"], r["name"]), flush=True)
        made.append((r, render_one(r, meta, base, occ, res, ox, oy, a.fps)))
        print("    -> {}".format(made[-1][1].name))
    print("\n{} 本書いた".format(len(made)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
