#!/usr/bin/env python3
"""走る地図（Nav2 の静的レイヤ）の現行と候補を並べる（2026-09-14）。"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REAL = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REAL / "quickstart"))
from jp_font import japanese_font                     # noqa: E402
from check_map_clearance import load_map              # noqa: E402
from score import WORK, SESSION, DESK, REGIONS        # noqa: E402

if japanese_font() is None:
    raise SystemExit("日本語フォントが見つからない（豆腐になるので止める）")

OUT = Path("/Users/inouereo/git_research/physical_ai/docs/作業ログ/images")
YAW = 18.5
t = np.deg2rad(YAW)
RM = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
rot = lambda xy: np.asarray(xy) @ RM

MAPS = [("現行 nav_map_clean（2026-09-08）", SESSION / "map" / "nav_map_clean.yaml"),
        ("新 nav_map_run（2026-09-14）", SESSION / "map" / "nav_map_run.yaml")]
tr = np.loadtxt(SESSION / "mola_floor0" / "traj.txt", usecols=(1, 2))


def occ_xy(g):
    h, _ = g.shape
    yy, xx = np.nonzero(g.occupied)
    return np.c_[g.origin[0] + (xx + .5) * g.resolution,
                 g.origin[1] + (yy + .5) * g.resolution]


def box(b):
    c = np.array([[b[0], b[2]], [b[1], b[2]], [b[1], b[3]], [b[0], b[3]], [b[0], b[2]]])
    return rot(c)



# ── セル集合の差を取る（原点も大きさも違うので世界座標のセル添字で揃える）──────
def cellset(g):
    h, _ = g.shape
    yy, xx = np.nonzero(g.occupied)
    wx = g.origin[0] + (xx + .5) * g.resolution
    wy = g.origin[1] + (yy + .5) * g.resolution
    return set(zip(np.floor(wx / 0.1).astype(int).tolist(),
                   np.floor(wy / 0.1).astype(int).tolist()))


A = load_map(Path(MAPS[0][1]))      # 現行 clean
B = load_map(Path(MAPS[1][1]))      # 候補
sa, sb = cellset(A), cellset(B)
only_b = np.array(sorted(sb - sa), float) * 0.1 + 0.05      # 候補にだけ有る＝戻った障害物
only_a = np.array(sorted(sa - sb), float) * 0.1 + 0.05      # clean にだけ有る＝候補で消えた
both = np.array(sorted(sa & sb), float) * 0.1 + 0.05
print("元からある {:,} / 足した {:,} / 消えた {:,}".format(len(both), len(only_b), len(only_a)))

fig, axes = plt.subplots(1, 3, figsize=(16, 9), dpi=110)
fig.patch.set_facecolor("#0b1020")
q = rot(tr)
panels = [(MAPS[0][0], [(rot(occ_xy(A)), "#e6ebf2", 0.5, None)]),
          (MAPS[1][0], [(rot(occ_xy(B)), "#e6ebf2", 0.5, None)]),
          ("足したセル", [(rot(both), "#3a4a66", 0.5, "元からある {:,}".format(len(both))),
                          (rot(only_b), "#3ddc84", 2.2,
                           "足した ＝ 実在が裏取りできた障害物 {:,}".format(len(only_b)))]
           + ([(rot(only_a), "#ff4d6d", 2.2, "消えた {:,}".format(len(only_a)))] if len(only_a) else []))]
for ax, (title, layers) in zip(axes, panels):
    ax.set_facecolor("#0b1020")
    for pts, c, sz, lab in layers:
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], c=c, s=sz, linewidths=0, label=lab)
    ax.plot(q[:, 0], q[:, 1], color="#4da3e8", lw=0.9, alpha=.85)
    for b in REGIONS.values():
        r = box(b); ax.plot(r[:, 0], r[:, 1], color="yellow", lw=1.1)
    if layers[0][3]:
        ax.legend(loc="lower left", fontsize=9, markerscale=10,
                  facecolor="#131c30", edgecolor="#3a4a66", labelcolor="white")
    e = rot(occ_xy(A))
    ax.set_xlim(e[:, 0].min() - 1, e[:, 0].max() + 1)
    ax.set_ylim(e[:, 1].min() - 1, e[:, 1].max() + 1)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, color="white", fontsize=12, pad=8)
fig.suptitle("Nav2 が走る地図 — 差し替え（白＝占有セル、青＝機体が歩いた道、黄枠＝ゴーストの 2 箇所）",
             color="white", fontsize=14, y=0.965)
fig.tight_layout(rect=[0, 0, 1, 0.95])
OUT.mkdir(parents=True, exist_ok=True)
f = OUT / "2026-09-14-navmap-clean-vs-candidate.png"
fig.savefig(f, facecolor=fig.get_facecolor(), bbox_inches="tight")
print("書いた:", f)
