"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K1）: 段 0 の止まっている点群で、扉の板から枠へ段が付く所（開口の縁）を
両側で読み、機体の思う扉の座標の横の外れを出す。numpy だけ。入力は geom_latch_side_1111.py の json（板の直線で前後と向きをそろえた後の
y 2 cm ごとの手前の面の中央 x_med）。

縁: 板（x≈−0.02）と枠（巻き尺で板の表の約 3 cm 手前）の中ほど EDGE_X を x_med が横切る y を、隣り合う 2 つの区間の直線補間で出す。
  蝶番の側は y < 0（−0.62〜−0.36）・把手の側は y > 0（0.36〜0.62）
横の外れ: 両側が読めたら (左の縁 + 右の縁)/2（扉の中心が 0 のはず）。片側だけなら、両側が読めた回の開口の幅の平均で片側から出す
確かめ: 開口の幅が回によらずそろうこと・枠の出（縁の外 4〜10 cm の x_med）が巻き尺の 3 cm と合うこと

  ../../.venv/bin/python k1_frame_edges_1111.py logs_real/20261111/geom_1111.json
"""
from __future__ import annotations

import argparse
import json
from typing import Dict, List, Optional

import numpy as np

BOARD_X = -0.02
EDGE_X = -0.035  # 板（−0.02）と枠（約 −0.05）の中ほど
SIDES = {"hinge": (-0.62, -0.36), "latch": (0.36, 0.62)}
MIN_POINTS = 20


def edge(profile: List[dict], side: str) -> Optional[float]:
    """板の側（中央寄り）から外へ向かって、x_med が EDGE_X より手前になる最初の所を補間で出す"""
    lo, hi = SIDES[side]
    pr = sorted((p for p in profile if lo <= p["y"] <= hi and p["n"] >= MIN_POINTS), key=lambda p: abs(p["y"]))
    for a, b in zip(pr, pr[1:]):
        if abs(abs(b["y"]) - abs(a["y"])) > 0.03:  # 抜けがある所は読まない
            return None
        if a["x_med"] > EDGE_X >= b["x_med"]:
            w = (a["x_med"] - EDGE_X) / (a["x_med"] - b["x_med"])
            return float(a["y"] + w * (b["y"] - a["y"]))
    return None


def frame_out_cm(profile: List[dict], y_edge: Optional[float], side: str) -> Optional[float]:
    if y_edge is None:
        return None
    s = 1.0 if side == "latch" else -1.0
    xs = [p["x_med"] for p in profile if 0.04 <= s * (p["y"] - y_edge) <= 0.10 and p["n"] >= MIN_POINTS]
    return round(100 * (BOARD_X - float(np.median(xs))), 1) if xs else None


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("geom_json")
    ap.add_argument("--window", default="段0")
    a = ap.parse_args(argv)
    with open(a.geom_json) as f:
        geom = json.load(f)
    rows: Dict[str, dict] = {}
    for label, r in geom.items():
        pr = r[a.window]["profile"]
        h, l = edge(pr, "hinge"), edge(pr, "latch")
        rows[label] = {"shift_cm": r[a.window]["shift_cm"], "yaw_err_deg": r[a.window]["yaw_err_deg"], "hinge": h, "latch": l,
                       "width": None if h is None or l is None else l - h,
                       "frame_out_cm": [frame_out_cm(pr, h, "hinge"), frame_out_cm(pr, l, "latch")]}
    widths = [v["width"] for v in rows.values() if v["width"] is not None]
    w_mean = float(np.mean(widths)) if widths else float("nan")
    print("回\t前後のそろえ [cm]\t向き [°]\t蝶番の側の縁\t把手の側の縁\t開口の幅\t横の外れ [cm]\t枠の出 [cm]（蝶番・把手）")
    for label, v in rows.items():
        if v["width"] is not None:
            lat = 100 * (v["hinge"] + v["latch"]) / 2
        elif v["hinge"] is not None:
            lat = 100 * (v["hinge"] + w_mean / 2)
        elif v["latch"] is not None:
            lat = 100 * (v["latch"] - w_mean / 2)
        else:
            lat = float("nan")
        hinge_s = "—" if v["hinge"] is None else f"{v['hinge']:+.3f}"
        latch_s = "—" if v["latch"] is None else f"{v['latch']:+.3f}"
        width_s = "—" if v["width"] is None else f"{v['width']:.3f}"
        note = "" if v["width"] is not None else "（片側・幅は平均）"
        print(f"{label}\t{v['shift_cm']:+.1f}\t{v['yaw_err_deg']:+.2f}\t{hinge_s}\t{latch_s}\t{width_s}\t{lat:+.1f}{note}\t{v['frame_out_cm']}")
    if widths:
        print(f"開口の幅: 平均 {w_mean:.3f} m・ばらつき（最大−最小）{100 * (max(widths) - min(widths)):.1f} cm（{len(widths)} 回）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
