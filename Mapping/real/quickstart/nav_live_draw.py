#!/usr/bin/env python3
"""`nav_live_view.py` の絵を描く側。橋との通信と HTTP は向こうに置いてある。

分けてあるのは、**絵の中身（膨張層・姿勢・重畳）だけを差し替えて試せる**ようにするため。
`render()` に State.snapshot() と同じ形の dict を渡せば、live でなくても 1 枚描ける。

⚠️ ここで扱う地図は **OccupancyGrid**（origin が左下・row-major で下から上）なので
`imshow(origin="lower")` がそのまま使える。PGM（`measure_overlay.read_map`）は
上下反転せずに返すので、予備地図として渡す側が `[::-1]` で向きを合わせること。

⚠️ matplotlib に日本語フォントは無い前提。PNG の中の文字は ASCII に統一し、
   日本語の説明は HTML 側（ブラウザが描く）に置く。
"""
from __future__ import annotations

import io
import logging
import math
import time

import numpy as np
from scipy.ndimage import binary_dilation

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPolygon, Rectangle
from matplotlib.transforms import Affine2D

from nav_live_msgs import TOPICS, tf_chain

matplotlib.rcParams["font.family"] = "DejaVu Sans"
# 等倍表示のため片方の軸は広がる。承知の上なので毎フレームの告知は出さない
logging.getLogger("matplotlib").setLevel(logging.ERROR)

COST_LETHAL, COST_INSCRIBED = 100, 99      # nav2_costmap_2d のコスト値
INFLATION_RADIUS_DEFAULT = 0.55            # params の inflation_radius。円で描くだけ
NEAREST_SEARCH_M = 5.0                     # 最寄り占有セルを探す窓の半径
STALE_S = 2.0                              # これ以上古い入力は「止まっている」とみなす
OCCUPIED_THRESH = 65                       # 事前地図で占有とみなす値（ROS の慣習）
# latched（transient_local）で 1 度しか来ないもの。古くても異常ではない
LATCHED = ("/map", "/tf_static")


# ── 色 ──────────────────────────────────────────────────────────────────
def cost_lut() -> np.ndarray:
    """コスト 0..100 と unknown(-1→101) を RGBA に。膨張層の勾配が見えること優先。"""
    lut = np.zeros((102, 4))
    anchors = [(0.0, (0.15, 0.45, 0.90)), (0.55, (0.10, 0.75, 0.55)), (1.0, (0.98, 0.80, 0.10))]
    for c in range(1, COST_INSCRIBED):
        f = (c - 1) / float(COST_INSCRIBED - 2)
        for (f0, c0), (f1, c1) in zip(anchors[:-1], anchors[1:]):
            if f0 <= f <= f1:
                u = (f - f0) / (f1 - f0)
                lut[c, :3] = [a + (b - a) * u for a, b in zip(c0, c1)]
                break
        lut[c, 3] = 0.30 + 0.45 * f
    lut[COST_INSCRIBED] = (1.00, 0.45, 0.00, 0.88)     # 内接円コスト = 触れたら当たる
    lut[COST_LETHAL] = (0.85, 0.05, 0.10, 0.95)        # 致命
    lut[101] = (0.0, 0.0, 0.0, 0.0)                    # unknown は透明
    return lut


LUT = cost_lut()


def grid_rgba(grid: np.ndarray, unknown_alpha: float = 0.0, alpha_scale: float = 1.0) -> np.ndarray:
    idx = np.where(grid < 0, 101, np.clip(grid.astype(np.int16), 0, 101))
    rgba = LUT[idx].copy()
    if alpha_scale != 1.0:
        rgba[..., 3] *= alpha_scale
    if unknown_alpha:
        rgba[idx == 101] = (0.45, 0.45, 0.50, unknown_alpha)
    return rgba


def wall_rgba(grid: np.ndarray) -> np.ndarray:
    """事前地図の占有セルだけを黒で。膨張層の上に重ねて壁の輪郭を残す。

    ⚠️ これが無いと、costmap の塗りで壁が隠れ、**位置が合っているかを見る**という
    この画面の主目的（scan が壁に乗るか）が読めなくなる。
    """
    rgba = np.zeros(grid.shape + (4,))
    rgba[grid >= OCCUPIED_THRESH] = (0.05, 0.05, 0.07, 0.85)
    return rgba


def map_rgb(grid: np.ndarray) -> np.ndarray:
    """事前地図（背景）。unknown は薄灰、free は白、占有は黒。"""
    img = np.full(grid.shape + (3,), 0.99)
    img[grid < 0] = (0.86, 0.87, 0.89)
    img[(grid > 0) & (grid < OCCUPIED_THRESH)] = (0.62, 0.62, 0.66)
    img[grid >= OCCUPIED_THRESH] = (0.12, 0.12, 0.14)
    return img


# ── 地図（OccupancyGrid）を map 系の絵として貼る ────────────────────────
def grid_affine(g: dict, tf_to_map: tuple[np.ndarray, float], ax):
    """grid をその frame から map 系へ載せる (extent, transform)。"""
    t, yaw = tf_to_map
    c, s = math.cos(yaw), math.sin(yaw)
    tx = t[0] + c * g["ox"] - s * g["oy"]
    ty = t[1] + s * g["ox"] + c * g["oy"]
    extent = [0.0, g["w"] * g["res"], 0.0, g["h"] * g["res"]]
    return extent, Affine2D().rotate(yaw + g["oyaw"]).translate(tx, ty) + ax.transData


def grid_cost_at(g: dict, tf_to_map, x: float, y: float) -> int | None:
    """map 系の (x,y) にあたるセルのコスト。範囲外なら None。"""
    t, yaw = tf_to_map
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = x - t[0], y - t[1]
    gx, gy = c * lx - s * ly, s * lx + c * ly          # grid の frame 系へ
    cy, sy = math.cos(-g["oyaw"]), math.sin(-g["oyaw"])
    ux, uy = gx - g["ox"], gy - g["oy"]
    px, py = cy * ux - sy * uy, sy * ux + cy * uy
    ix, iy = int(px / g["res"]), int(py / g["res"])
    if 0 <= ix < g["w"] and 0 <= iy < g["h"]:
        return int(g["grid"][iy, ix])
    return None


def nearest_occupied(g: dict, x: float, y: float) -> tuple[float, float, float] | None:
    """map 系 (x,y) から最寄りの占有セルまで。(距離, セル x, セル y)。"""
    res = g["res"]
    ix, iy = int((x - g["ox"]) / res), int((y - g["oy"]) / res)
    r = int(NEAREST_SEARCH_M / res)
    x0, x1 = max(0, ix - r), min(g["w"], ix + r + 1)
    y0, y1 = max(0, iy - r), min(g["h"], iy + r + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    ys, xs = np.nonzero(g["grid"][y0:y1, x0:x1] >= OCCUPIED_THRESH)
    if xs.size == 0:
        return None
    cx = g["ox"] + (x0 + xs + 0.5) * res
    cy = g["oy"] + (y0 + ys + 0.5) * res
    d = np.hypot(cx - x, cy - y)
    k = int(np.argmin(d))
    return float(d[k]), float(cx[k]), float(cy[k])


_DILATED: dict = {"grid": None, "dil": None}


def wall_dilated(base: dict) -> np.ndarray:
    """占有セルを 1 セル太らせたもの。地図が替わるまで使い回す。"""
    if _DILATED["grid"] is not base["grid"]:
        _DILATED.update(grid=base["grid"],
                        dil=binary_dilation(base["grid"] >= OCCUPIED_THRESH))
    return _DILATED["dil"]


def scan_on_wall(base: dict, pts: np.ndarray) -> tuple[float, int] | None:
    """map 系に起こした scan のうち、事前地図の壁（±1 セル）に乗った割合。

    `measure_overlay.count_hits` と同じ数え方の live 版。**絶対値で合否は決められない**
    （床の反射も外れとして数える。2026-09-11 の実測）が、測位がずれると必ず下がるので、
    位置が合っているかを目で見るときの裏取りに使える。
    """
    res = base["res"]
    ix = np.floor((pts[:, 0] - base["ox"]) / res).astype(int)
    iy = np.floor((pts[:, 1] - base["oy"]) / res).astype(int)
    m = (ix >= 0) & (ix < base["w"]) & (iy >= 0) & (iy < base["h"])
    if not m.any():
        return None
    return float(wall_dilated(base)[iy[m], ix[m]].mean()), int(m.sum())


def best_shift(base: dict, pts: np.ndarray, max_m: float = 0.4) -> tuple[float, float, float]:
    """scan を平行移動して当たりが最大になる (dx, dy, その割合) を返す。

    絶対値の重畳率は床の反射も外れに数えるので合否には使えないが、**ずらして最良が
    (0, 0) から離れる**なら、その分だけ推定姿勢が地図に対してずれている
    （この判定の考え方は measure_overlay と同じ）。
    """
    dil = wall_dilated(base)
    res, h, w = base["res"], base["h"], base["w"]
    step = res
    n = int(round(max_m / step))
    best = (0.0, 0.0, -1.0)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            dx, dy = i * step, j * step
            ix = np.floor((pts[:, 0] + dx - base["ox"]) / res).astype(int)
            iy = np.floor((pts[:, 1] + dy - base["oy"]) / res).astype(int)
            m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            if not m.any():
                continue
            hit = float(dil[iy[m], ix[m]].mean())
            if hit > best[2]:
                best = (dx, dy, hit)
    return best


def to_map(tf: dict, frame: str) -> tuple[tuple[np.ndarray, float], bool]:
    """frame → map の (t, yaw)。辿れなければ恒等とみなし False を返す。"""
    r = tf_chain(tf, "map", frame or "map")
    return (r, True) if r is not None else ((np.zeros(2), 0.0), False)


def xform(pts: np.ndarray, t_yaw) -> np.ndarray:
    t, yaw = t_yaw
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([t[0] + c * pts[:, 0] - s * pts[:, 1],
                     t[1] + s * pts[:, 0] + c * pts[:, 1]], 1)


# ── 描画 ────────────────────────────────────────────────────────────────
def draw_layers(ax, snap: dict, base: dict, base_tf, info: dict, zoom: bool) -> None:
    msgs, tf, ages = snap["msgs"], snap["tf"], info["topics"]
    extent, trans = grid_affine(base, base_tf, ax)
    ax.imshow(map_rgb(base["grid"]), origin="lower", interpolation="nearest",
              extent=extent, transform=trans, zorder=1)

    # 拡大側では global を薄くして、上に載る local を読めるようにする
    for key, z, unk, sc in (("/global_costmap/costmap", 2, 0.0, 0.55 if zoom else 1.0),
                            ("/local_costmap/costmap", 3, 0.08, 1.0)):
        g = msgs.get(key)
        if g is None:
            continue
        t_yaw, _ = to_map(tf, g["frame"])
        ext, tr = grid_affine(g, t_yaw, ax)
        ax.imshow(grid_rgba(g["grid"], unknown_alpha=unk, alpha_scale=sc), origin="lower",
                  interpolation="nearest", extent=ext, transform=tr, zorder=z)
        if key.startswith("/local"):
            rect = Rectangle((0, 0), ext[1], ext[3], fill=False, lw=1.4, ec="#0066cc", ls="--",
                             zorder=6)
            rect.set_transform(tr)
            ax.add_patch(rect)

    plan = msgs.get("/plan")
    if plan is not None and len(plan["pts"]):
        p = xform(plan["pts"], to_map(tf, plan["frame"])[0])
        ax.plot(p[:, 0], p[:, 1], "-", color="#1560ff", lw=2.2, zorder=7)
        ax.plot(p[-1, 0], p[-1, 1], "*", color="#1560ff", ms=15, zorder=7)

    ax.imshow(wall_rgba(base["grid"]), origin="lower", interpolation="nearest",
              extent=extent, transform=trans, zorder=5)

    scan = msgs.get("/scan")
    if scan is not None and scan["n"]:
        p = xform(scan["xy"], to_map(tf, scan["frame"])[0])
        fresh = (ages.get("/scan") or 1e9) < STALE_S
        ax.scatter(p[:, 0], p[:, 1], s=(16.0 if zoom else 4.0),
                   c=("#ff00aa" if fresh else "#9aa0a6"), marker="o", zorder=8,
                   linewidths=0.4, edgecolors=("#4a0030" if fresh else "#5f6368"))
        hit = scan_on_wall(base, p)
        info.setdefault("scan", {}).update(
            frame=scan["frame"], n=scan["n"], fresh=fresh,
            on_wall_pct=(None if hit is None else round(100 * hit[0], 1)),
            in_map=(None if hit is None else hit[1]),
            bbox=[round(float(v), 2) for v in (p[:, 0].min(), p[:, 0].max(),
                                               p[:, 1].min(), p[:, 1].max())])
        if hit is not None and not zoom:               # 拡大側では計算も上書きもしない
            dx, dy, pct = best_shift(base, p)
            info["scan"]["best_shift"] = [round(dx, 2), round(dy, 2)]
            info["scan"]["best_shift_pct"] = round(100 * pct, 1)

    fp = msgs.get("/local_costmap/published_footprint")
    if fp is not None and len(fp["pts"]) >= 3:
        poly = xform(fp["pts"], to_map(tf, fp["frame"])[0])
        ax.add_patch(MplPolygon(poly, closed=True, fill=False, ec="#00a000", lw=2.2, zorder=9))

    pose = info.get("pose")
    if pose:
        x, y, yaw = pose["x"], pose["y"], pose["yaw"]
        fresh = not pose["stale"]
        ax.add_patch(Circle((x, y), 0.16, fc=("#00c000" if fresh else "#9aa0a6"),
                            ec="black", lw=1.0, zorder=10))
        ax.arrow(x, y, 0.6 * math.cos(yaw), 0.6 * math.sin(yaw), width=0.045, head_width=0.17,
                 color=("black" if fresh else "#5f6368"), zorder=10, length_includes_head=True)
        if zoom:
            ax.add_patch(Circle((x, y), info["inflation"], fill=False, ec="#003366", ls=":",
                                lw=1.8, zorder=11))
            near = info.get("nearest")
            if near:
                ax.plot([x, near["cx"]], [y, near["cy"]], "-", color="#ff2a00", lw=2.2, zorder=12)
                ax.plot([near["cx"]], [near["cy"]], "x", color="#ff2a00", ms=10, mew=2.5, zorder=12)
                # ラベルは線に**直交**して逃がす。距離が短いと線上に置いても読めない
                vx, vy = near["cx"] - x, near["cy"] - y
                n_ = math.hypot(vx, vy) or 1.0
                ax.annotate(f"{near['d']:.2f} m", (near["cx"], near["cy"]), color="#ff2a00",
                            fontsize=12, fontweight="bold", zorder=13,
                            xytext=(-16 * vy / n_, 16 * vx / n_), textcoords="offset points",
                            ha="center", va="center")


def add_legend(fig, rect) -> None:
    """コスト → 色の対応。膨張層を読むのに要る。"""
    x, y, w, h = rect
    ax = fig.add_axes([x, y, w * 0.60, h])
    ax.imshow(grid_rgba(np.arange(0, 99, dtype=np.int16)[None, :]), aspect="auto",
              origin="lower", extent=[0, 98, 0, 1])
    ax.set_facecolor("white")
    ax.set_yticks([])
    ax.set_xticks([0, 98])
    ax.set_xticklabels(["free 0", "98"], fontsize=6.5)
    ax.set_title("costmap cost   (99=inscribed, 100=lethal)", fontsize=7.0, pad=2,
                 loc="left")
    for a2, cost, label in ((fig.add_axes([x + w * 0.66, y, w * 0.14, h]), COST_INSCRIBED,
                             "99"),
                            (fig.add_axes([x + w * 0.84, y, w * 0.14, h]), COST_LETHAL,
                             "100")):
        a2.imshow(grid_rgba(np.array([[cost]], dtype=np.int16)), aspect="auto")
        a2.set_facecolor("white")
        a2.set_xticks([]); a2.set_yticks([])
        a2.set_xlabel(label, fontsize=6.5, labelpad=2)
    for a2 in (ax,):
        a2.tick_params(length=2, pad=1)


def view_window(base: dict, info: dict, plan, full: bool, half: float) -> tuple[list, list]:
    """全体図の窓。既定はロボットと /plan が入る範囲（地図が広すぎると何も読めない）。"""
    x0, y0 = base["ox"], base["oy"]
    x1, y1 = x0 + base["w"] * base["res"], y0 + base["h"] * base["res"]
    if full or not info.get("pose"):
        return [x0, x1], [y0, y1]
    xs = [info["pose"]["x"] - half, info["pose"]["x"] + half]
    ys = [info["pose"]["y"] - half, info["pose"]["y"] + half]
    if plan is not None and len(plan):
        xs += [plan[:, 0].min() - 2.0, plan[:, 0].max() + 2.0]
        ys += [plan[:, 1].min() - 2.0, plan[:, 1].max() + 2.0]
    return ([max(x0, min(xs)), min(x1, max(xs))], [max(y0, min(ys)), min(y1, max(ys))])


def collect_info(snap: dict, base: dict, inflation: float, now: float) -> dict:
    """絵にする前に数値を出す。/status.json にそのまま出すのもこれ。"""
    msgs, tf = snap["msgs"], snap["tf"]
    ages = {t: (None if t not in snap["recv"] else round(now - snap["recv"][t], 2)) for t in TOPICS}
    info: dict = {"inflation": inflation, "topics": ages}
    tf_age = ages.get("/tf")

    r = tf_chain(tf, "map", "base_link")
    if r is not None:
        info["pose"] = {"x": float(r[0][0]), "y": float(r[0][1]), "yaw": float(r[1]),
                        "src": "tf map->base_link", "age": tf_age,
                        "stale": tf_age is None or tf_age > STALE_S}
    elif "/amcl_pose" in msgs:
        a = msgs["/amcl_pose"]
        info["pose"] = {"x": a["x"], "y": a["y"], "yaw": a["yaw"], "src": "/amcl_pose",
                        "age": ages.get("/amcl_pose"),
                        "stale": (ages.get("/amcl_pose") or 1e9) > STALE_S}
    if "/amcl_pose" in msgs:
        a = msgs["/amcl_pose"]
        info["amcl"] = {"x": round(a["x"], 3), "y": round(a["y"], 3),
                        "yaw_deg": round(math.degrees(a["yaw"]), 2),
                        "sigma_xy": [round(a["sx"], 3), round(a["sy"], 3)],
                        "sigma_yaw_deg": round(math.degrees(a["syaw"]), 2)}

    if info.get("pose"):
        x, y = info["pose"]["x"], info["pose"]["y"]
        near = nearest_occupied(base, x, y)
        if near:
            info["nearest"] = {"d": round(near[0], 3), "cx": near[1], "cy": near[2],
                               "inside_inflation": near[0] < inflation}
        for key, name in (("/global_costmap/costmap", "cost_global"),
                          ("/local_costmap/costmap", "cost_local")):
            g = msgs.get(key)
            if g is not None:
                info[name] = grid_cost_at(g, to_map(tf, g["frame"])[0], x, y)
    return info


def render(snap: dict, prior: dict | None, prior_name: str, inflation: float,
           zoom_m: float, full: bool, half: float) -> tuple[bytes, dict]:
    now = time.time()
    msgs = snap["msgs"]
    live_map = msgs.get("/map")
    base = live_map if live_map is not None else prior
    if base is None:
        raise RuntimeError("事前地図が無い（/map も来ず、--map も指定されていない）")

    info = collect_info(snap, base, inflation, now)
    info["map_source"] = ("live /map" if live_map is not None else f"local PGM {prior_name}")
    base_tf, _ = to_map(snap["tf"], base.get("frame", "map"))

    fig = plt.figure(figsize=(15.0, 8.2), dpi=100, facecolor="#f4f5f7")
    gs = fig.add_gridspec(1, 2, width_ratios=[1.3, 1.0], left=0.035, right=0.985,
                          top=0.73, bottom=0.05, wspace=0.11)
    ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    for ax, zoom in ((ax1, False), (ax2, True)):
        draw_layers(ax, snap, base, base_tf, info, zoom)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, color="#c8ccd4", lw=0.4, alpha=0.7)
        ax.tick_params(labelsize=8)

    plan = msgs.get("/plan")
    plan_pts = None
    if plan is not None and len(plan["pts"]):
        plan_pts = xform(plan["pts"], to_map(snap["tf"], plan["frame"])[0])
    xlim, ylim = view_window(base, info, plan_pts, full, half)
    ax1.set_xlim(*xlim)
    ax1.set_ylim(*ylim)
    ax1.set_title("overview: prior map + global costmap + /plan", fontsize=10)
    if info.get("pose"):
        x, y = info["pose"]["x"], info["pose"]["y"]
        ax2.set_xlim(x - zoom_m, x + zoom_m)
        ax2.set_ylim(y - zoom_m, y + zoom_m)
    else:
        ax2.set_xlim(*xlim)
        ax2.set_ylim(*ylim)
    sc = info.get("scan", {})
    ax2.set_title("zoom +-%.1f m: local costmap + footprint + /scan%s" % (
        zoom_m, "" if sc.get("fresh", True) else "  [scan STALE]"), fontsize=10)

    # 無線の飽和は測位を壊す。どれだけ引いているかを常に画面に出す
    dt = max(1e-3, now - snap.get("start", now))
    info["mbps"] = {t: round(n / 1e6 / dt, 3) for t, n in sorted(
        snap.get("nbytes", {}).items(), key=lambda kv: -kv[1])}
    info["mbps_total"] = round(sum(snap.get("nbytes", {}).values()) / 1e6 / dt, 3)

    # ── 見出し（ASCII。日本語の説明は HTML 側） ──
    p = info.get("pose")
    if not p:
        l1, c1 = "NO map->base_link TF and no /amcl_pose - cannot place the robot", "#c00000"
    else:
        l1 = "robot  x=%+.3f  y=%+.3f  yaw=%+7.2f deg   [%s, age %s s]%s" % (
            p["x"], p["y"], math.degrees(p["yaw"]), p["src"],
            "n/a" if p["age"] is None else ("%.1f" % p["age"]),
            "   ** STALE: FROZEN **" if p["stale"] else "")
        c1 = "#c00000" if p["stale"] else "#111111"
    near = info.get("nearest")
    if near is None:
        l2, c2 = "nearest occupied cell : n/a", "#555555"
    else:
        l2 = "nearest occupied cell : %.2f m   inflation_radius = %.2f m   ->  %s" % (
            near["d"], inflation,
            "INSIDE inflation - the foot is in the inflated layer" if near["inside_inflation"]
            else "outside inflation - clear")
        c2 = "#c00000" if near["inside_inflation"] else "#006000"
    l3 = "cost under robot : global=%s  local=%s    (99=inscribed, 100=lethal)" % (
        info.get("cost_global"), info.get("cost_local"))
    fig.text(0.035, 0.962, "G1 Nav2 live view", fontsize=15, fontweight="bold")
    shift = sc.get("best_shift")
    l4 = "scan on prior-map wall : %s of %s pts   best if shifted by %s -> %s  <- position check" % (
        "n/a" if sc.get("on_wall_pct") is None else "%.1f %%" % sc["on_wall_pct"],
        sc.get("n", "-"),
        "n/a" if shift is None else "(%+.2f,%+.2f) m" % (shift[0], shift[1]),
        "n/a" if sc.get("best_shift_pct") is None else "%.1f %%" % sc["best_shift_pct"])
    fig.text(0.035, 0.930, l1, fontsize=9.8, family="monospace", color=c1)
    fig.text(0.035, 0.902, l2, fontsize=9.8, family="monospace", color=c2)
    fig.text(0.035, 0.874, l3, fontsize=9.8, family="monospace")
    fig.text(0.035, 0.846, l4, fontsize=9.8, family="monospace",
             color="#555555" if sc.get("fresh", True) else "#c00000")
    fig.text(0.035, 0.818, "map source: %s    rendered %s    ws in: %.2f MB/s" % (
        info["map_source"], time.strftime("%H:%M:%S"), info.get("mbps_total", 0.0)),
        fontsize=8.5, color="#555555", family="monospace")

    rows = []
    for t_ in TOPICS:
        age = info["topics"][t_]
        mark = "" if (age is None or age <= STALE_S or t_ in LATCHED) else "  STALE"
        if t_ in snap.get("skipped", []):
            rows.append("%-34s %s" % (t_, "(--skip)"))
        else:
            rows.append("%-34s %s" % (t_, "(no data)" if age is None
                                      else "%6.1f s%s" % (age, mark)))
    fig.text(0.755, 0.975, "\n".join(rows), fontsize=7.4, family="monospace", va="top",
             color="#333333")
    add_legend(fig, [0.035, 0.770, 0.20, 0.020])

    info["counts"] = snap["count"]
    info["conn"] = snap["conn"]
    info["missing"] = [t for t in snap["missing"] if t not in snap.get("skipped", [])]
    info["skipped"] = snap.get("skipped", [])
    info["err"] = snap["err"]
    info["t"] = time.strftime("%Y-%m-%d %H:%M:%S")
    info["tf_tree"] = {c: {"parent": v[0], "static": v[4]} for c, v in snap["tf"].items()}
    info["frames"] = {k: v.get("frame") for k, v in msgs.items()}
    info["grids"] = {k: {"w": v["w"], "h": v["h"], "res": v["res"],
                         "origin": [round(v["ox"], 3), round(v["oy"], 3)], "frame": v["frame"]}
                     for k, v in msgs.items() if "grid" in v}

    buf = io.BytesIO()
    fig.savefig(buf, format="png", facecolor=fig.get_facecolor())
    plt.close(fig)
    return buf.getvalue(), info


def error_png(msg: str) -> bytes:
    fig = plt.figure(figsize=(15.0, 8.2), dpi=100, facecolor="#f4f5f7")
    fig.text(0.03, 0.95, "render failed", fontsize=16, color="#c00000", va="top")
    fig.text(0.03, 0.88, msg[-3000:], fontsize=8, family="monospace", va="top")
    buf = io.BytesIO()
    fig.savefig(buf, format="png", facecolor=fig.get_facecolor())
    plt.close(fig)
    return buf.getvalue()
