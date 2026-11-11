"""11-11: 扉の周り（扉の板・枠・把手の側の壁・蝶番の側）の形を生の点群で測り、段 2 の左手の通り道を重ねた上から見た図を作る。OMEN の CPU。

各回について 2 つの止まっている区間（段 0 の初め 1.2 秒・構えの位置で腕を構えている 2.5 秒）の点群を、その回の FAST-LIO の口の姿勢で
扉の座標に置き、閉じた扉の板の点の中央が板の表（x = −0.02）に来るように前後をそろえる（そろえた量＝機体が思っていた扉の位置の前後のずれ）。
手の高さ（0.85〜1.15 m）の点を上から見て、把手の側の面が板の表から何 cm 手前にあるかを y 2 cm ごとに出す。
左手（指先・手のひら・肘）の通り道は、構えの区間のそろえ量で同じように直して重ねる（順運動学・pelvis の高さ 0.73 m）。

  python geom_latch_side_1111.py --out geom_1111.png --json geom_1111.json [--urdf ...]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_door_est as est  # noqa: E402
import render_staged_views_1111 as rv  # noqa: E402
import g1_arm_fk as fk  # noqa: E402

D = Path.home() / "work_inoue/real_data/20261111"
L = D / "door_real/logs"
RUNS = (("070921", "20261111T070910_run_staged_20261111_070910_li", "1 回目"),
        ("072728", "20261111T072717_run_staged_20261111_072717_li", "2 回目"),
        ("073522", "20261111T073511_run_staged_20261111_073511_li", "3 回目"),
        ("075139", "20261111T075128_run_staged_20261111_075128_li", "4 回目"))
BOARD_X = -0.02
Z_HAND = (0.85, 1.15)
Y_BINS = np.arange(-1.10, 1.20, 0.02)


def window_points(z, ev, sess: str, rows, lo: float, hi: float) -> np.ndarray:
    t, td = z["t"], ev["t_door_mono"]
    lm = z["lidar_msg"]
    ok = np.isfinite(lm[:, 0])
    off = float(np.median(lm[ok, 0] - t[ok]))
    sc = rv.load_scans(D / "mapping_runs" / sess, td + off + lo, td + off + hi)
    lev = est.level_from_scans([x for _, x in sc])
    Ps = []
    for s, x in sc:
        pz = rv.nearest_pose(rows, rows[:, 0], s)
        if pz is not None:
            Ps.append(rv.lidar_points_door(x, lev, pz))
    return np.concatenate(Ps) if Ps else np.zeros((0, 3))


def board_line(P: np.ndarray) -> Tuple[float, float]:
    """閉じた扉の板の点に直線 x = a + b·y を当てる（2 回目は ±2 cm の内で当て直す）。b が扉の座標の向きの誤り。
    11-11: 傾きを無視して中央の位置だけでそろえると、0.8 m 横の壁が 1° で 1.4 cm 違って見えた（壁が 4.3〜5.5 cm と出た誤り）"""
    m = (P[:, 1] > -0.40) & (P[:, 1] < 0.25) & (P[:, 2] > 0.3) & (P[:, 2] < 1.6) & (np.abs(P[:, 0] - BOARD_X) < 0.12)
    if m.sum() < 50:
        return float("nan"), 0.0
    A = np.column_stack([np.ones(int(m.sum())), P[m, 1]])
    c = np.linalg.lstsq(A, P[m, 0], rcond=None)[0]
    keep = np.abs(P[m, 0] - A @ c) < 0.02
    c = np.linalg.lstsq(A[keep], P[m, 0][keep], rcond=None)[0]
    return float(c[0]), float(c[1])


def align(P: np.ndarray, line: Tuple[float, float]) -> np.ndarray:
    """板の直線が x = BOARD_X に来るように前後をそろえる（傾きも直す）"""
    a, b = line
    out = P.copy()
    out[:, 0] = P[:, 0] - (a + b * P[:, 1]) + BOARD_X
    return out


def profile(P: np.ndarray) -> List[Dict[str, float]]:
    m = (P[:, 2] > Z_HAND[0]) & (P[:, 2] < Z_HAND[1]) & (P[:, 0] > -0.35) & (P[:, 0] < 0.6)
    out = []
    for y0 in Y_BINS:
        mm = m & (P[:, 1] >= y0) & (P[:, 1] < y0 + 0.02)
        if mm.sum() >= 8:
            out.append({"y": round(float(y0 + 0.01), 3), "n": int(mm.sum()), "x_front": round(float(np.percentile(P[mm, 0], 10)), 3),
                        "x_med": round(float(np.median(P[mm, 0])), 3)})
    return out


def arm_paths(z, ev, chain, line: Tuple[float, float]) -> Dict[str, np.ndarray]:
    st, t = z["staged"], z["t"]
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 2] == 2)
    k0 = int(np.flatnonzero(in2)[0])
    k1 = int(min(len(t), np.searchsorted(t, t[k0] + 2.6)))
    pose = z["pose_f"]
    out = {"L指先": [], "L手": [], "L肘": []}
    for k in range(k0, k1):
        if not np.isfinite(pose[k]).all():
            continue
        pts = rv.arm_points(chain, z["q"][k], z["quat"][k], pose[k], 0.73)
        for n in out:
            out[n].append(pts[n])
    return {n: align(np.array(v), line) for n, v in out.items()}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--json", required=True)
    ap.add_argument("--urdf", default=None)
    a = ap.parse_args(argv)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "Noto Sans CJK JP"
    rows = np.load(L / "fastlio_node_20261111_064227.npz")["rows"]
    chain = fk.load_chain(a.urdf)
    res, clouds, paths = {}, [], {}
    for run, sess, label in RUNS:
        z = rv.stock_arrays(str(L / f"door_stock_20261111_{run}.npz"))
        ev = rv.events(z)
        r = {}
        slope0 = 0.0
        for name, lo, hi in (("段0", 0.2, 1.4), ("構え", ev["stage2"] - 0.2, ev["stage2_policy"] - 0.1)):
            P = window_points(z, ev, sess, rows, lo, hi)
            if name == "段0":
                line = board_line(P)
                slope0 = line[1]
            else:
                # 構えでは板が斜めに少ししか写らず腕も混じるので、傾きは段 0 の値（REF で決まり FAST-LIO の向きのずれは小さい）・前後だけ合わせる
                m = (P[:, 1] > -0.40) & (P[:, 1] < 0.25) & (P[:, 2] > 0.3) & (P[:, 2] < 1.6) & (np.abs(P[:, 0] - BOARD_X) < 0.12)
                line = (float(np.median(P[m, 0] - slope0 * P[m, 1])), slope0)
            # 扉の中央寄り（y = −0.08）でのそろえ量（機体が扉を遠くに見ていた量）と、向きの誤り
            r[name] = {"shift_cm": round(100 * (BOARD_X - (line[0] + line[1] * -0.08)), 1),
                       "yaw_err_deg": round(math.degrees(math.atan(line[1])), 2)}
            P = align(P, line)
            r[name]["profile"] = profile(P)
            clouds.append((label, name, P))
            if name == "構え":
                paths[label] = arm_paths(z, ev, chain, line)
                pz = paths[label]
                ok = pz["L指先"][:, 1] > 0.47
                r["tip_outside_max_x_aligned"] = round(float(pz["L指先"][ok, 0].max()), 3) if ok.any() else None
        res[label] = r
        print(label, {k: (v["shift_cm"], v["yaw_err_deg"]) for k, v in r.items() if isinstance(v, dict)}, "指先（縁の外）の前後の最大（そろえた後）",
              r.get("tip_outside_max_x_aligned"), flush=True)
    Path(a.json).write_text(json.dumps(res, ensure_ascii=False, indent=1))
    # 図: 上から（扉が上・把手の側が左）
    fig, ax = plt.subplots(figsize=(12, 7.4), dpi=130)
    allP = np.concatenate([P for _, name, P in clouds if name == "段0"])  # 構えの区間は機体の腕が写り込むので形には使わない
    m = (allP[:, 2] > Z_HAND[0]) & (allP[:, 2] < Z_HAND[1])
    ax.scatter(-allP[m, 1], allP[m, 0], s=2.0, c="#7f7f7f", alpha=0.45, linewidths=0, label="点群（手の高さ 0.85〜1.15 m・4 回の段 0）")
    ax.axhline(BOARD_X, color="#333", lw=0.8, ls=":")
    ax.fill_between([-0.47, 0.47], [BOARD_X, BOARD_X], [0.02, 0.02], color="#444", alpha=0.5, lw=0)
    ax.plot([-0.406, -0.276], [-0.075, -0.075], color="#d62728", lw=5, solid_capstyle="round")
    ax.plot([-0.406], [-0.075], "o", color="#d62728", ms=7)
    for sgn in (-1, 1):
        ax.plot([sgn * 0.52, sgn * 1.1], [-0.07, -0.07], "--", color="#1f77b4", lw=1.3)
        ax.fill_between(sorted([sgn * 0.468, sgn * 0.518]), [-0.08, -0.08], [0.04, 0.04], color="#1f77b4", alpha=0.12, lw=0)
    colors = {"1 回目": "#2ca02c", "2 回目": "#ff7f0e", "3 回目": "#9467bd", "4 回目": "#e377c2"}
    for label, pz in paths.items():
        ax.plot(-pz["L指先"][:, 1], pz["L指先"][:, 0], color=colors[label], lw=1.6, label=f"{label} 左の指先（段 2 の 0〜2.6 秒）")
        ax.plot(-pz["L肘"][:, 1], pz["L肘"][:, 0], color=colors[label], lw=0.8, ls="--")
    ax.text(0.05, 0.035, "扉の板（閉・表は x=−0.02）", ha="center", fontsize=10)
    ax.annotate("レバー（付け根 y=0.406 から蝶番の側へ 13 cm）", xy=(-0.34, -0.075), xytext=(-0.05, -0.16), fontsize=9, color="#d62728",
                arrowprops=dict(arrowstyle="->", color="#d62728"))
    ax.plot([-0.52, -1.0], [-0.031, -0.031], color="#1f4e79", lw=2.5)
    ax.annotate("壁の面（巻き尺: 板の表の約 1 cm 手前・垂直）", xy=(-0.75, -0.031), xytext=(-0.98, 0.12), fontsize=9, color="#1f4e79",
                arrowprops=dict(arrowstyle="->", color="#1f4e79"))
    ax.plot([-0.47, -0.52], [-0.05, -0.05], color="#7f3f00", lw=3)
    ax.annotate("枠の前の角（巻き尺: 約 3 cm 手前）", xy=(-0.50, -0.05), xytext=(-0.60, 0.20), fontsize=9, color="#7f3f00",
                arrowprops=dict(arrowstyle="->", color="#7f3f00"))
    ax.annotate("扉の縁（y=0.47）", xy=(-0.47, 0.0), xytext=(-0.40, 0.08), fontsize=8, arrowprops=dict(arrowstyle="->", color="#555"))
    ax.text(-0.98, -0.058, "シムの脇の壁の面（−0.07）", fontsize=8, color="#1f77b4")
    ax.text(-0.53, 0.05, "シムの枠", ha="center", fontsize=8, color="#1f77b4")
    ax.set_xlim(-1.0, 0.2)
    ax.set_ylim(-0.45, 0.25)
    ax.set_aspect("equal")
    ticks = np.arange(-1.0, 0.21, 0.1)
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"{-v:+.1f}" for v in ticks])
    ax.set_xlabel("扉に沿った位置 y [m]（左＝把手の側・扉の中心が 0・縁は ±0.47）")
    ax.set_ylabel("扉からの前後 x [m]（0＝板の中心・上＝扉の奥）")
    ax.grid(alpha=0.25)
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title("扉の周りの形（11-11 の点群・扉の板の直線でそろえた）と、段 2 の左手の通り道（実線＝指先・破線＝肘）", fontsize=11)
    fig.tight_layout()
    fig.savefig(a.out)
    print("[geom] 書いた", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
