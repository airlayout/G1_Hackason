#!/usr/bin/env python3
"""ゴースト除去の前後を図にする（2026-09-13）。

部屋は map 系で 18.5 度傾いているので、**壁が水平・垂直になる向きに回して**描く
（ユーザーが画面上で囲んだ図と同じ向きになる）。
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
REAL = HERE.parents[1]
sys.path.insert(0, str(REAL / "quickstart"))
from jp_font import japanese_font              # noqa: E402
from score import load_map, REGIONS, FZ, WORK           # noqa: E402

if japanese_font() is None:
    raise SystemExit("日本語フォントが見つからない（豆腐になるので止める）")
YAW = 18.5
t = np.deg2rad(YAW)
RM = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
CAND = WORK / "cand" / "all"
OUT = Path("/Users/inouereo/git_research/physical_ai/docs/作業ログ/images")


def rot(xy):
    return np.asarray(xy) @ RM


def load(name):
    if name == "00_before":
        p, _ = load_map()
        return p
    return np.loadtxt(CAND / (name + ".txt"), skiprows=1)


def boxpath(b):
    c = np.array([[b[0], b[2]], [b[1], b[2]], [b[1], b[3]], [b[0], b[3]], [b[0], b[2]]])
    return rot(c)


def panel(ax, pts, title, boxes=True, zoom=None):
    m = (pts[:, 0] > -12) & (pts[:, 0] < 12) & (pts[:, 1] > -20) & (pts[:, 1] < 28)
    s = pts[m]
    h = s[:, 2] - FZ
    q = rot(s[:, :2])
    ax.set_facecolor("#0b1020")
    ax.scatter(q[:, 0], q[:, 1], c=h, cmap="jet", s=0.35 if zoom else 0.12,
               vmin=-0.2, vmax=2.0, linewidths=0, rasterized=True)
    if boxes:
        for n, b in REGIONS.items():
            r = boxpath(b)
            ax.plot(r[:, 0], r[:, 1], color="yellow", lw=1.6)
            ax.text(r[:, 0].min(), r[:, 1].max() + 0.3, n[0], color="yellow",
                    fontsize=13, weight="bold")
    if zoom:
        r = boxpath(REGIONS[zoom])
        ax.set_xlim(r[:, 0].min() - 1.2, r[:, 0].max() + 1.2)
        ax.set_ylim(r[:, 1].min() - 1.2, r[:, 1].max() + 1.2)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, color="white", fontsize=11, pad=6)


CASES = [("00_before", "前（掃除なし）"),
         ("1a_管A（09-12採用案）", "09-12 の採用案\n（A だけ・軌跡の管）"),
         ("z4_全部", "2026-09-13\n管A ＋ 管B ＋ 塊 ＋ 彫り")]

fig, axes = plt.subplots(2, 3, figsize=(15, 15), dpi=110,
                         gridspec_kw=dict(height_ratios=[2.1, 1]))
fig.patch.set_facecolor("#0b1020")
for j, (name, title) in enumerate(CASES):
    p = load(name)
    panel(axes[0, j], p, title)
    panel(axes[1, j], p, "領域 B の拡大", boxes=False, zoom="B_開けた床")
fig.suptitle("追従者ゴーストの除去（黄枠はユーザーが指した 2 箇所）",
             color="white", fontsize=15, y=0.965)
fig.tight_layout(rect=[0, 0, 1, 0.95])
OUT.mkdir(parents=True, exist_ok=True)
f = OUT / "2026-09-13-ghost-removal-three-methods.png"
fig.savefig(f, facecolor=fig.get_facecolor(), bbox_inches="tight")
print("書いた:", f)

# ── 2 枚目: どの手がどこを消したか ─────────────────────────────────
from filters import tube, blob, _cellkey, _support_cells, TALL   # noqa: E402
from score import Scorecard, cell_hmax, RES                      # noqa: E402

sc = Scorecard()
h2 = sc.p[:, 2] - FZ
key2 = _cellkey(sc.p[:, :2])
sup = _support_cells(sc.p, h2)
bd = (h2 >= 0.15) & (h2 <= TALL)
lh = cell_hmax(sc.ref.xyz)
z = np.load(WORK / "carve_votes_R4.npz")
vox = key2 * 4096 + (np.floor(sc.p[:, 2] / RES).astype(np.int64) + 2048)
ii = np.searchsorted(z["vox"], vox).clip(0, len(z["vox"]) - 1)
okk = z["vox"][ii] == vox
miss = np.where(okk, z["miss"][ii], 0); hit = np.where(okk, z["hit"][ii], 0)

M = [("管A（軌跡・領域A）", tube(sc.p, sc.d, R=0.50, keep_tall=True,
                             boxes=[tuple(REGIONS["A_廊下"])]), "#4da6ff"),
     ("管B（軌跡・領域B）", tube(sc.p, sc.d, R=0.70, keep_tall=False,
                             boxes=[tuple(REGIONS["B_開けた床"])]), "#3ddc84"),
     ("塊（人の形）",       blob(sc.p, live_hmax=lh, max_area=1.2, hi=1.20, min_cells=2), "#ffa726"),
     ("彫り（見通した）",   bd & (miss >= 4) & (miss > 10.0 * hit) & ~np.isin(key2, sup), "#ff4d6d")]

fig2, ax2 = plt.subplots(1, 2, figsize=(13, 11), dpi=110,
                         gridspec_kw=dict(width_ratios=[1.0, 1.25]))
fig2.patch.set_facecolor("#0b1020")
for ax, zoom, ttl in ((ax2[0], None, "部屋全体"), (ax2[1], "B_開けた床", "領域 B の拡大")):
    ax.set_facecolor("#0b1020")
    keepm = ~np.any([m for _, m, _ in M], axis=0)
    q = rot(sc.p[keepm][::4, :2])
    ax.scatter(q[:, 0], q[:, 1], c="#3a4a66", s=0.25 if zoom else 0.08, linewidths=0, rasterized=True)
    for n, m, c in M:
        q = rot(sc.p[m][::2, :2])
        ax.scatter(q[:, 0], q[:, 1], c=c, s=0.9 if zoom else 0.25, linewidths=0,
                   rasterized=True, label=n if zoom is None else None)
    if zoom is None:
        for n, b in REGIONS.items():
            r = boxpath(b); ax.plot(r[:, 0], r[:, 1], color="yellow", lw=1.4)
            ax.text(r[:, 0].min(), r[:, 1].max() + 0.3, n[0], color="yellow", fontsize=13, weight="bold")
        ax.legend(loc="upper right", markerscale=14, fontsize=10,
                  facecolor="#131c30", edgecolor="#3a4a66", labelcolor="white")
        cl = (sc.p[:, 0] > -12) & (sc.p[:, 0] < 12) & (sc.p[:, 1] > -20) & (sc.p[:, 1] < 28)
        e = rot(sc.p[cl][:, :2])
        ax.set_xlim(e[:, 0].min() - 0.5, e[:, 0].max() + 0.5)
        ax.set_ylim(e[:, 1].min() - 0.5, e[:, 1].max() + 0.5)
    else:
        r = boxpath(REGIONS[zoom])
        ax.set_xlim(r[:, 0].min() - 1.2, r[:, 0].max() + 1.2)
        ax.set_ylim(r[:, 1].min() - 1.2, r[:, 1].max() + 1.2)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(ttl, color="white", fontsize=12)
fig2.suptitle("どの手が、どこを消したか（灰色は残る点）", color="white", fontsize=15)
fig2.tight_layout(rect=[0, 0, 1, 0.96])
f2 = OUT / "2026-09-13-ghost-removal-by-method.png"
fig2.savefig(f2, facecolor=fig2.get_facecolor(), bbox_inches="tight")
print("書いた:", f2)
