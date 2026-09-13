#!/usr/bin/env python3
"""3 案の最良と、その合わせ技を**同じスコアカード**で並べる（2026-09-13）。"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from score import Scorecard, REGIONS, WORK, FZ, RES     # noqa: E402
from filters import tube, blob, _cellkey, _support_cells, TALL   # noqa: E402
from score import cell_hmax                        # noqa: E402

A = [tuple(REGIONS["A_廊下"])]
B = [tuple(REGIONS["B_開けた床"])]

sc = Scorecard()
out = WORK / "cand" / "all"
h = sc.p[:, 2] - FZ
key = _cellkey(sc.p[:, :2])
support = _support_cells(sc.p, h)
band = (h >= 0.15) & (h <= TALL)
live_hmax = cell_hmax(sc.ref.xyz)


def carve_drop(mr=4.0, need=4, ratio=10.0):
    z = np.load(WORK / "carve_votes_R{:.0f}.npz".format(mr))
    vz = np.floor(sc.p[:, 2] / RES).astype(np.int64)
    vox = key * 4096 + (vz + 2048)
    idx = np.searchsorted(z["vox"], vox)
    ok = (idx < len(z["vox"])) & (z["vox"][np.minimum(idx, len(z["vox"]) - 1)] == vox)
    miss = np.where(ok, z["miss"][np.minimum(idx, len(z["vox"]) - 1)], 0)
    hit = np.where(ok, z["hit"][np.minimum(idx, len(z["vox"]) - 1)], 0)
    return band & (miss >= need) & (miss > ratio * hit) & ~np.isin(key, support)


D = {
    "管A":   tube(sc.p, sc.d, R=0.50, keep_tall=True,  boxes=A),
    "管B70": tube(sc.p, sc.d, R=0.70, keep_tall=False, boxes=B),
    "塊":    blob(sc.p, live_hmax=live_hmax, max_area=1.2, hi=1.20, min_cells=2),
    "彫":    carve_drop(4.0, 4, 10.0),
}

print(sc.header())
print(Scorecard.row(sc.evaluate("00_before", np.ones(len(sc.p), bool), out)))
rows = []
for name, parts in [
    ("1a_管A（09-12採用案）", ["管A"]),
    ("1d_管A+管B70",         ["管A", "管B70"]),
    ("3z_管A+塊",            ["管A", "塊"]),
    ("2z_管A+彫",            ["管A", "彫"]),
    ("z1_管A+管B70+塊",      ["管A", "管B70", "塊"]),
    ("z2_管A+管B70+彫",      ["管A", "管B70", "彫"]),
    ("z3_管A+塊+彫",         ["管A", "塊", "彫"]),
    ("z4_全部",              ["管A", "管B70", "塊", "彫"]),
]:
    dr = np.zeros(len(sc.p), bool)
    for k in parts:
        dr |= D[k]
    r = sc.evaluate(name, ~dr, out, save_txt=True)
    rows.append(r)
    print(Scorecard.row(r))
print("\n" + Scorecard.rule())

ok = [r for r in rows if r["verdict"] == "OK"]
if ok:
    best = max(ok, key=lambda r: r["drop_A_廊下"] + r["drop_B_開けた床"])
    print("\n最良（合格のうち A+B の削減が最大）: {}  A {:.1f}% / B {:.1f}% / "
          "壁の帯 {:.1f}% / 机の帯 {:.1f}%".format(
              best["name"], best["drop_A_廊下"], best["drop_B_開けた床"],
              best["wall"], best["deskband"]))
