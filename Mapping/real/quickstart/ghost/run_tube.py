"""案 1: 軌跡の管を領域ごとの設定で掃引する。"""
import sys, numpy as np
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from score import Scorecard, REGIONS, WORK
from filters import tube

A = [tuple(REGIONS["A_廊下"])]
B = [tuple(REGIONS["B_開けた床"])]

sc = Scorecard()
out = WORK / "cand" / "tube"
print(sc.header())
print(Scorecard.row(sc.evaluate("00_before", np.ones(len(sc.p), bool), out)))

def case(name, *specs):
    drop = np.zeros(len(sc.p), bool)
    for kw in specs:
        drop |= tube(sc.p, sc.d, **kw)
    r = sc.evaluate(name, ~drop, out)
    print(Scorecard.row(r)); return r

case("1a_Aのみ_R050_柱残す",  dict(R=0.50, keep_tall=True,  boxes=A))          # 前回の採用案
case("1b_A+B_R050_柱残す",   dict(R=0.50, keep_tall=True,  boxes=A+B))
case("1c_A柱残す+B柱消す",    dict(R=0.50, keep_tall=True,  boxes=A),
                              dict(R=0.50, keep_tall=False, boxes=B))
case("1d_A050+B070柱消す",   dict(R=0.50, keep_tall=True,  boxes=A),
                              dict(R=0.70, keep_tall=False, boxes=B))
case("1e_A050+B100柱消す",   dict(R=0.50, keep_tall=True,  boxes=A),
                              dict(R=1.00, keep_tall=False, boxes=B))
case("1f_全域_R050_柱残す",   dict(R=0.50, keep_tall=True,  boxes=None))
case("1g_全域_R070_柱残す",   dict(R=0.70, keep_tall=True,  boxes=None))
print("\n" + Scorecard.rule())
