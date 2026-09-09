#!/usr/bin/env python3
"""check_navigation.py --record の記録から、走行の中身を動画にする。

**記録に真値があるか（= Isaac Sim か実機か）で右 3 段が変わる。**

sim（真値あり）:
  左: 部屋の障害物の上に **真値** と **測位の推定** を並べて描く。2 つが離れていくのが失敗の中身
  右: 「真値と推定のずれ」「真値の z（乗り上げ）」「Nav2 の指令」
  **指令は出続けているのに位置が動かない**ことと **z が 0.63 → 1.02 に上がる**ことを揃えて見せる

実機（真値なし。`truth` が null）:
  左: 推定の軌跡だけ
  右: 「**ゴールまでの距離**」「**ゴールとの向きの差**」「Nav2 の指令」
  ⚠️ この 3 段は 2026-09-09 の失敗を説明するために選んである。あの日は
  **距離が 0.28m まで落ちて平らになったのに、向きの差が 140° 台に張り付き、
  指令だけが出続けた**。原因はゴールの向きを機体と無関係な固定値で投げていたこと。
  真値が無い実機では「ずれ」も「pelvis の z」も描けないので、この 2 つで代える。

⚠️ **表題は記録によって変えること。** 同じ描画で、失敗した記録（AMCL）と
成功した記録（`G1_PERFECT_LOC=1`）の両方を描く。既定の文言は失敗した記録の
ものなので、成功した記録にそのまま使うと**動画が嘘をつく**。
`--title` / `--subtitle` / `--est-label` で渡す。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import Circle

plt.rcParams["font.family"] = ["Hiragino Sans", "DejaVu Sans"]
plt.rcParams["font.monospace"] = ["Menlo", "Hiragino Sans", "DejaVu Sans Mono"]

INK, INK2, INK3 = "#111620", "#46505f", "#6b7686"
LINE = "#dce1e9"
TRUTH, AMCL, GOAL_C, PLAN_C = "#0f6fc4", "#b1500f", "#14714a", "#14714a"
BAD, SURFACE = "#a8202c", "#ffffff"

STAND_Z = 0.63          # 正常に立っているときの pelvis の高さ [m]（実測）
# 障害物の高さの配色。低い＝薄い灰、高い＝濃い。真値（青）と AMCL（橙）に
# 色を取られないよう、地の部分は無彩色にしてある。
HEIGHT_CMAP = LinearSegmentedColormap.from_list(
    "height", ["#e4e8ef", "#aab3c1", "#6b7686", "#3b4453", "#1c2430"])
FPS = 10
TAIL = 10**9            # 軌跡は全部残す


def load_scene(npz_path: Path):
    d = np.load(npz_path)
    return d["occupied"], d["level"], d["origin"], float(d["cell"])


def occupied_bbox(occ: np.ndarray, origin: np.ndarray,
                  cell: float) -> tuple[float, float, float, float]:
    """占有セルが実際に在る範囲。npz の格子は周りに空白を抱えているので、
    そのまま extent を使うと地図が小さく写る。"""
    rows, cols = np.nonzero(occ)
    if not len(rows):
        raise SystemExit("[NG] シーンに占有セルが無い")
    return (float(origin[0]) + cols.min() * cell,
            float(origin[0]) + (cols.max() + 1) * cell,
            float(origin[1]) + rows.min() * cell,
            float(origin[1]) + (rows.max() + 1) * cell)


def fit_view(box: tuple[float, float, float, float], pad: float,
             panel_aspect: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """`box` が全部入るように、パネルの縦横比に合わせて余っている側へ余白を足す。

    ⚠️ `set_aspect("equal")` に任せると軸の箱が縮んで左右に白が残る。
    先に縦横比を合わせておけば、地図がパネルいっぱいに写る。
    `panel_aspect` は `ax.get_position()` から実測した値を渡すこと
    （gridspec の比から手計算すると wspace のぶん外れる）。
    """
    x0, x1, y0, y1 = box[0] - pad, box[1] + pad, box[2] - pad, box[3] + pad
    w, h = x1 - x0, y1 - y0
    if w / h < panel_aspect:
        w = h * panel_aspect
    else:
        h = w / panel_aspect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return (cx - w / 2, cx + w / 2), (cy - h / 2, cy + h / 2)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True)
    ap.add_argument("--scene", required=True, help="sim/octomap_sim.npz")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fast-after", type=float, default=100.0,
                    help="この秒数を過ぎたら早送りにする（機体は止まっているので）")
    ap.add_argument("--fast-factor", type=int, default=6)
    ap.add_argument("--title",
                    default="走行試験がどう失敗したか（掃除済み地図・Isaac Sim・3 回とも未到達）")
    ap.add_argument("--subtitle",
                    default="経路計画は 8/8 通っている。落ちたのは歩き始めてから。"
                            "AMCL が真値から離れ、機体は高さ 1.00 m の机の塊に乗り上げて止まる")
    ap.add_argument("--goal-yaw", type=float, default=None, metavar="RAD",
                    help="ゴールの向き[rad]。**記録に goal_yaw が無い古い実機記録**に使う"
                         "（2026-09-09 の記録は 0 を渡す。あの日の値がそれで、それが失敗の原因）")
    ap.add_argument("--est-label", default="AMCL の推定",
                    help="推定の凡例。測位を差し替えたら必ず変えること")
    ap.add_argument("--view", choices=("full", "fit"), default="full",
                    help="full: 地図全体を写す（既定）/ fit: 軌跡の周りだけに寄る")
    ap.add_argument("--segment", type=int, metavar="N",
                    help="N 回目のゴール（1 始まり）だけを描く。**動画は 1 回ぶんずつ "
                         "別ファイルにする**（繋げると見たい回まで早送りが要る）。"
                         "区切りは analyze_navigation.py と同じ「ゴールが変わる所」")
    args = ap.parse_args()

    data = json.loads((Path(args.rec) / "navigation.json").read_text())
    # ⚠️ **真値を必須にしないこと。** 実機の記録は truth が null なので、
    # 以前はここで「記録が空」と言って実機の走行を 1 本も描けなかった（2026-09-09）。
    track = [s for s in data["track"] if s["amcl"]]
    if not track:
        raise SystemExit("[NG] 記録が空（amcl が 1 つも無い）")
    HAS_TRUTH = all(s.get("truth") for s in track)
    print("[mode] " + ("sim（真値あり）" if HAS_TRUTH else "実機（真値なし）"))

    if args.segment is not None:
        # ゴールが変わったところで区切る（analyze_navigation.py と同じ規則）。
        segments: list[list[dict]] = []
        for s in track:
            g = tuple(s["goal"]) if s["goal"] else None
            if not segments or (segments[-1][0]["goal"] and
                                tuple(segments[-1][0]["goal"]) != g):
                segments.append([s])
            else:
                segments[-1].append(s)
        if not 1 <= args.segment <= len(segments):
            raise SystemExit(f"[NG] --segment は 1〜{len(segments)}（この記録の区間数）")
        track = segments[args.segment - 1]
        print(f"[segment] {args.segment}/{len(segments)} 区間目"
              f"（{len(track)} サンプル）")

    occ, lvl, origin, cell = load_scene(Path(args.scene))
    extent = (float(origin[0]), float(origin[0]) + occ.shape[1] * cell,
              float(origin[1]), float(origin[1]) + occ.shape[0] * cell)

    t0 = track[0]["t"]
    ts = np.array([s["t"] - t0 for s in track])
    amcl = np.array([s["amcl"] for s in track])
    cmds = np.array([s["cmd"] for s in track])
    if HAS_TRUTH:
        truth = np.array([s["truth"] for s in track])
        err = np.hypot(truth[:, 0] - amcl[:, 0], truth[:, 1] - amcl[:, 1])
        zs = np.array([s["sim_z"] if s["sim_z"] is not None else np.nan
                       for s in track])
    else:
        truth = zs = None
        # 実機の 2 段。① ゴールまでの距離 ② ゴールとの向きの差
        def _gxy(s):
            return s["goal"] if s["goal"] else [np.nan, np.nan]
        gxy = np.array([_gxy(s) for s in track], dtype=float)
        dist_goal = np.hypot(gxy[:, 0] - amcl[:, 0], gxy[:, 1] - amcl[:, 1])
        gy = np.array([s.get("goal_yaw") if s.get("goal_yaw") is not None
                       else args.goal_yaw for s in track], dtype=float)
        if np.all(np.isnan(gy)):
            raise SystemExit(
                "[NG] ゴールの向きが記録にも引数にも無い。--goal-yaw を渡すこと"
                "（2026-09-09 以前の記録には goal_yaw が入っていない）")
        d = amcl[:, 2] - gy
        yaw_err = np.abs(np.arctan2(np.sin(d), np.cos(d)))

    # 表示範囲。既定は「地図全体」（部屋のどこを歩いているのかが分かる）。
    # --view fit は軌跡とゴールの周りだけに寄る（短距離の記録で細部を見たいとき）。
    if args.view == "full":
        box, pad = occupied_bbox(occ, origin, cell), 0.5
    else:
        allx = np.concatenate([truth[:, 0], amcl[:, 0]]) if HAS_TRUTH else amcl[:, 0]
        ally = np.concatenate([truth[:, 1], amcl[:, 1]]) if HAS_TRUTH else amcl[:, 1]
        goals = np.array([s["goal"] for s in track if s["goal"]])
        if len(goals):
            allx = np.concatenate([allx, goals[:, 0]])
            ally = np.concatenate([ally, goals[:, 1]])
        box, pad = (allx.min(), allx.max(), ally.min(), ally.max()), 2.0

    # 乗り上げてからは何も動かないので、そこから先は間引いて早送りにする。
    # ⚠️ 早送りしていることは画面に出す（等速に見えると誤解を生む）。
    idxs = [i for i in range(len(track))
            if ts[i] <= args.fast_after or i % args.fast_factor == 0]
    print(f"[frames] {len(track)} サンプル → {len(idxs)} コマ"
          f"（{args.fast_after:.0f} s まで等速、以降 {args.fast_factor} 倍速）")

    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=SURFACE)
    # ⚠️ top は 0.885 まで上げられない。fit_view で縦横比を合わせた結果、軸の箱が
    # 縮まずに埋まるので、左パネルの表題（set_title）が副題に重なる。
    gs = fig.add_gridspec(3, 2, width_ratios=[1.45, 1.0], height_ratios=[1, 1, 1],
                          left=0.035, right=0.97, top=0.862, bottom=0.065,
                          wspace=0.14, hspace=0.42)
    ax = fig.add_subplot(gs[:, 0])
    ax_err = fig.add_subplot(gs[0, 1])
    ax_z = fig.add_subplot(gs[1, 1])
    ax_cmd = fig.add_subplot(gs[2, 1])

    # 軸の箱を実測して縦横比を合わせる（gridspec の比から計算すると wspace のぶん外れる）
    pos = ax.get_position()
    fw, fh = fig.get_size_inches()
    xlim, ylim = fit_view(box, pad, (pos.width * fw) / (pos.height * fh))
    print(f"[view] {args.view}  x {xlim[0]:.1f}〜{xlim[1]:.1f} m"
          f" / y {ylim[0]:.1f}〜{ylim[1]:.1f} m")

    fig.text(0.035, 0.955, args.title,
             fontsize=19, color=INK, weight="bold", va="center")
    fig.text(0.035, 0.917, args.subtitle,
             fontsize=12.5, color=INK2, va="center")

    # ⚠️ 毎コマ ax.clear() するので、カラーバーを ax の子（inset）に置くと消える。
    # figure 直下の軸に置いて 1 度だけ作る。
    cax = fig.add_axes([0.415, 0.115, 0.155, 0.016])
    cb_done = [False]

    writer = FFMpegWriter(fps=FPS, bitrate=3600,
                          metadata={"title": "走行試験の失敗"})

    with writer.saving(fig, args.out, dpi=120):
        for i in idxs:
            s = track[i]
            # ── 左：部屋と 2 つの軌跡 ─────────────────
            ax.clear()
            ax.set_facecolor("#f6f8fa")
            h = np.ma.masked_where(~occ, lvl)
            im = ax.imshow(h, origin="lower", extent=extent, cmap=HEIGHT_CMAP,
                           norm=Normalize(0.0, 2.0), interpolation="nearest")
            ax.set_xlim(*xlim); ax.set_ylim(*ylim)
            ax.set_aspect("equal")
            ax.tick_params(labelsize=9, colors=INK3)
            for sp in ax.spines.values():
                sp.set_color(LINE)
            ax.set_xlabel("map x [m]", fontsize=10, color=INK3)
            ax.set_ylabel("map y [m]", fontsize=10, color=INK3)

            if cb_done[0] is False:
                cb = fig.colorbar(im, cax=cax, orientation="horizontal")
                cb.set_label("障害物の高さ [m]", fontsize=10, color=INK3, labelpad=2)
                cb.ax.tick_params(labelsize=9, colors=INK3, length=2, pad=1)
                cb.outline.set_edgecolor(LINE)
                cb_done[0] = True

            # 今のゴールと経路
            if s["goal"]:
                ax.plot([s["goal"][0]], [s["goal"][1]], marker="*", ms=22,
                        color=GOAL_C, markeredgecolor="white", markeredgewidth=1.2,
                        zorder=7)
                ax.annotate("ゴール", (s["goal"][0], s["goal"][1]),
                            textcoords="offset points", xytext=(12, 8),
                            fontsize=11.5, color=GOAL_C, weight="bold", zorder=8)
            if s["plan"]:
                p = np.asarray(s["plan"])
                ax.plot(p[:, 0], p[:, 1], color=PLAN_C, lw=3.2, alpha=0.45,
                        zorder=4, label="Nav2 の経路")

            if HAS_TRUTH:
                ax.plot(truth[:i + 1, 0], truth[:i + 1, 1], color=TRUTH, lw=2.6,
                        zorder=6, label="真値（Isaac Sim）")
            ax.plot(amcl[:i + 1, 0], amcl[:i + 1, 1], color=AMCL, lw=2.6,
                    ls="--", zorder=6, label=args.est_label)
            if HAS_TRUTH:
                ax.plot([truth[i, 0], amcl[i, 0]], [truth[i, 1], amcl[i, 1]],
                        color=BAD, lw=2.0, ls=":", zorder=9)
                ax.add_patch(Circle((truth[i, 0], truth[i, 1]), 0.25, fill=False,
                                    ec=TRUTH, lw=2.4, zorder=10))
                # 地図全体を写すと 0.25 m の円は 10 px 程度になる。機体の位置を
                # 見失わないよう、縮尺に依らない点も重ねる。
                ax.plot([truth[i, 0]], [truth[i, 1]], "o", ms=7, color=TRUTH,
                        markeredgecolor="white", markeredgewidth=1.0, zorder=10)
            else:
                # 真値が無いので推定の周りに機体の円を描く
                ax.add_patch(Circle((amcl[i, 0], amcl[i, 1]), 0.30, fill=False,
                                    ec=AMCL, lw=2.2, zorder=10))
                # 向きが分かるように矢印を出す（この記録の失敗は向きの話なので要る）
                ax.annotate("", xy=(amcl[i, 0] + 0.55 * np.cos(amcl[i, 2]),
                                    amcl[i, 1] + 0.55 * np.sin(amcl[i, 2])),
                            xytext=(amcl[i, 0], amcl[i, 1]), zorder=11,
                            arrowprops=dict(arrowstyle="-|>", color=AMCL, lw=2.2))
            ax.plot([amcl[i, 0]], [amcl[i, 1]], "o", ms=9, color=AMCL,
                    markeredgecolor="white", zorder=10)
            if HAS_TRUTH and err[i] > 0.4:
                mx = (truth[i, 0] + amcl[i, 0]) / 2
                my = (truth[i, 1] + amcl[i, 1]) / 2
                ax.annotate(f"ずれ {err[i]:.2f} m", (mx, my),
                            textcoords="offset points", xytext=(8, -14),
                            fontsize=11.5, color=BAD, weight="bold", zorder=11,
                            bbox=dict(fc="white", ec="none", alpha=0.8, pad=1.5))
            ax.legend(loc="upper left", fontsize=10.5, framealpha=0.92)
            if ts[i] > args.fast_after:
                ax.text(0.985, 0.965, f"{args.fast_factor} 倍速",
                        transform=ax.transAxes, ha="right", va="top",
                        fontsize=12, color=INK2, weight="bold",
                        bbox=dict(fc="#eef1f6", ec=LINE, pad=3.0))

            climbed = HAS_TRUTH and (not np.isnan(zs[i])) and zs[i] > STAND_Z + 0.10
            if climbed:
                # 乗り上げた場所を図の中でも指しておく（真値 z だけでは場所が分からない）
                # ⚠️ 吹き出しの位置は**点**で置く。データ座標（m）で置くと
                # 縮尺が変わったときに機体へ重なる（--view full で踏んだ）。
                ax.annotate("高さ 1.00 m の机の塊の上\n（robot_radius より狭い所へ入り込んだ）",
                            (truth[i, 0], truth[i, 1]),
                            textcoords="offset points", xytext=(-70, -60),
                            fontsize=11.5, color=BAD, weight="bold", zorder=12,
                            ha="center",
                            arrowprops=dict(arrowstyle="->", color=BAD, lw=1.8),
                            bbox=dict(fc="white", ec=BAD, lw=1.0, alpha=0.92, pad=3.0))
            if HAS_TRUTH:
                head = (f"経過 {ts[i]:5.1f} s    真値 ({truth[i,0]:+.2f}, {truth[i,1]:+.2f})"
                        f"    推定 ({amcl[i,0]:+.2f}, {amcl[i,1]:+.2f})"
                        + ("    ← 机に乗り上げている" if climbed else ""))
            else:
                head = (f"経過 {ts[i]:5.1f} s    推定 ({amcl[i,0]:+.2f}, {amcl[i,1]:+.2f}, "
                        f"{np.degrees(amcl[i,2]):+.0f}°)"
                        f"    ゴールまで {dist_goal[i]:.2f} m"
                        f"    向きの差 {np.degrees(yaw_err[i]):.0f}°")
            ax.set_title(head, fontsize=13.5,
                         color=BAD if climbed else INK, pad=10, loc="left")

            # ── 右：3 つの時系列 ────────────────────
            panels = ((("[m]", "① 真値と推定のずれ"),
                       ("[m]", "② 真値の z（pelvis の高さ）"),
                       ("", "③ Nav2 が出している指令"))
                      if HAS_TRUTH else
                      (("[m]", "① ゴールまでの距離"),
                       ("[°]", "② ゴールとの向きの差"),
                       ("", "③ Nav2 が出している指令")))
            for a, (ylab, title) in zip((ax_err, ax_z, ax_cmd), panels):
                a.clear()
                a.set_facecolor(SURFACE)
                a.tick_params(labelsize=9.5, colors=INK3)
                for sp in a.spines.values():
                    sp.set_color(LINE)
                a.set_xlim(0, ts[-1] * 1.02)
                a.set_title(title, fontsize=12.5, color=INK, loc="left", pad=6)
                a.set_ylabel(ylab, fontsize=9.5, color=INK3)

            if HAS_TRUTH:
                ax_err.plot(ts[:i + 1], err[:i + 1], color=BAD, lw=2.4)
                ax_err.axhline(0.30, color=INK3, lw=1, ls=":")
                ax_err.text(ts[-1] * 0.99, 0.34, "合格の目安 0.30 m", fontsize=10,
                            color=INK3, ha="right")
                ax_err.set_ylim(0, max(1.0, np.nanmax(err) * 1.25))
                cur_a, unit_a, col_a = err[i], "m", BAD

                ax_z.plot(ts[:i + 1], zs[:i + 1], color=TRUTH, lw=2.4)
                ax_z.axhline(STAND_Z, color=INK3, lw=1, ls=":")
                ax_z.text(ts[-1] * 0.99, STAND_Z + 0.02,
                          f"正常に立っている {STAND_Z} m",
                          fontsize=10, color=INK3, ha="right")
                lo = np.nanmin(zs) if np.isfinite(np.nanmin(zs)) else 0.5
                hi = np.nanmax(zs) if np.isfinite(np.nanmax(zs)) else 1.1
                ax_z.set_ylim(min(0.55, lo - 0.05), max(1.15, hi + 0.08))
                cur_b, unit_b, col_b = zs[i], "m", (BAD if climbed else TRUTH)
            else:
                # ① ゴールまでの距離。**合格の線を割ってから平らになるのが見える**
                ax_err.plot(ts[:i + 1], dist_goal[:i + 1], color=AMCL, lw=2.4)
                ax_err.axhline(0.30, color=INK3, lw=1, ls=":")
                ax_err.text(ts[-1] * 0.99, 0.34,
                            "到達の許容 0.30 m（xy_goal_tolerance）",
                            fontsize=10, color=INK3, ha="right")
                ax_err.set_ylim(0, max(0.6, np.nanmax(dist_goal) * 1.15))
                inside = dist_goal[i] <= 0.30
                cur_a, unit_a, col_a = dist_goal[i], "m", (
                    "#14714a" if inside else AMCL)

                # ② ゴールとの向きの差。**ここが下がらないので終わらない**
                ax_z.plot(ts[:i + 1], np.degrees(yaw_err[:i + 1]),
                          color=BAD, lw=2.4)
                ax_z.axhline(20.0, color=INK3, lw=1, ls=":")
                ax_z.text(ts[-1] * 0.99, 24.0,
                          "到達の許容 20°（yaw_goal_tolerance 0.35 rad）",
                          fontsize=10, color=INK3, ha="right")
                ax_z.set_ylim(0, max(40.0, np.degrees(np.nanmax(yaw_err)) * 1.15))
                cur_b, unit_b, col_b = np.degrees(yaw_err[i]), "°", BAD

            for a, cur, unit, col in ((ax_err, cur_a, unit_a, col_a),
                                      (ax_z, cur_b, unit_b, col_b)):
                a.text(0.012, 0.86, "いま", transform=a.transAxes,
                       fontsize=12, color=INK3)
                a.text(0.075, 0.86, f"{cur:.2f} {unit}".replace(" °", "°"),
                       transform=a.transAxes, fontsize=13.5, color=col,
                       weight="bold", family="monospace")

            ax_cmd.plot(ts[:i + 1], cmds[:i + 1, 0], color=TRUTH, lw=2.2,
                        label="vx [m/s]")
            ax_cmd.plot(ts[:i + 1], cmds[:i + 1, 2], color=AMCL, lw=2.2,
                        label="yaw [rad/s]")
            ax_cmd.axhline(0, color=LINE, lw=1)
            ax_cmd.set_ylim(-1.2, 1.2)
            ax_cmd.legend(loc="upper right", fontsize=10, ncol=2, framealpha=0.9)
            ax_cmd.set_xlabel("経過 [s]", fontsize=9.5, color=INK3)

            writer.grab_frame()

    print(f"[OK] -> {args.out}  （{len(track)} サンプル / {ts[-1]:.0f} 秒ぶん）")


if __name__ == "__main__":
    main()
