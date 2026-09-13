"""案 3: 塊の形で判定する。領域を手で囲まない（全域に掛ける）。"""
import sys, numpy as np
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from score import Scorecard, live_points, FZ, RES, cell_hmax, WORK
from filters import blob, tube, _cellkey
from score import REGIONS

sc = Scorecard()
out = WORK / "cand" / "blob"

# ライブのセルごとの最高点（見えているセルだけ。見えない所は判定に使わない）
live_hmax = cell_hmax(sc.ref.xyz)
print("フィルタが使う 09-12 のセル: {:,}".format(len(live_hmax)))

print(sc.header())
print(Scorecard.row(sc.evaluate("00_before", np.ones(len(sc.p), bool), out)))

def case(name, **kw):
    dr = blob(sc.p, live_hmax=live_hmax, **kw)
    r = sc.evaluate(name, ~dr, out); print(Scorecard.row(r)); return r

for area in (0.6, 1.2, 2.0):
    case("3_塊_面積<={:.1f}m2".format(area), max_area=area, hi=1.20, min_cells=2)
case("3_塊_面積<=1.2_hi1.0", max_area=1.2, hi=1.00, min_cells=2)
case("3_塊_面積<=1.2_最小3", max_area=1.2, hi=1.20, min_cells=3)

# 合わせ技: A は軌跡の管（追従者は形が崩れて塊にならない）、それ以外は塊
A = [tuple(REGIONS["A_廊下"])]
dr = tube(sc.p, sc.d, R=0.50, keep_tall=True, boxes=A) | blob(sc.p, live_hmax=live_hmax,
                                                              max_area=1.2, hi=1.20, min_cells=2)
r = sc.evaluate("3z_A管+全域塊", ~dr, out, save_txt=True); print(Scorecard.row(r))
print("\n" + Scorecard.rule())
