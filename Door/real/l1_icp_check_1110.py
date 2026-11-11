"""11-10 手順 L1 (a) の物差しの確かめ: 段 0 と着いた先の止まった区間どうしのライダーの動きを、3 つの出どころで並べる。

  ICP     … 止まった区間の点群を重ねて、点から面への ICP（lidar_still_icp.icp・粗い→細かいの 3 段。11-04 の正解と同じ）
  FAST-LIO … 再生の /Odometry_loc の区間の平均の姿勢（l1_fastlio_carry_1110.planar_track）
  推定器   … R0 の正解の行（lidar_door_est.DoorTracker）の区間の中央値を、段 0 のライダーの座標に直したもの

どれも段 0 の床で水平にした「段 0 のライダーの座標」（原点はライダーの真下・+x はライダーの前）での (前, 左, 向き)。
ICP は初期値を FAST-LIO と推定器の 2 通りにして、答えが初期値に引きずられていないかも見る。

使い方（Mac・Navigation/.venv。open3d が要る）:
  Navigation/.venv/bin/python Door/real/l1_icp_check_1110.py --lidar <切り出したセッション> --fastlio <tf_bag> \\
      --truth truth_183336_S.npz --anchor 1500:1525 --end 1545:1560 [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import l1_fastlio_carry_1110 as l1  # noqa: E402
import lidar_door_est as est  # noqa: E402
import lidar_still_icp as lsi  # noqa: E402


PERTURB = np.array([0.10, -0.10, np.radians(2.0)])  # ICP の初期値への依存を見るずらし
ICP_AGREE_M = 0.005  # 初期値を変えた ICP の答えがこれ以内でそろえば「よい」
ICP_MIN_FITNESS = 0.4


def planar_of(T: np.ndarray, level: est.Level) -> np.ndarray:
    """ICP の答え T（段 0 の床で水平にした着いた先の点 → 段 0 の水平の座標）から、着いた先のライダーの水平の位置と向き。
    ⚠️ T の並進そのものは「段 0 の床の水平で見たライダーの真下の点」で、着いた先で頭の傾きが変わると（腕を上げて 3° など）
    ライダーの高さ h × 傾きの変わりだけずれる（1.2 m × 3° ≈ 6 cm）。FAST-LIO・推定器と同じ「ライダーそのものの位置」にするため、
    着いた先のライダーの原点（段 0 の水平の座標では (0, 0, h)）を T で運ぶ。向きは推定器と同じ「傾きだけを外した」ZYX の向き"""
    p = T @ np.array([0.0, 0.0, level.h, 1.0])
    A = T[:3, :3] @ level.R @ est.FLIP.T
    return np.array([p[0], p[1], math.atan2(A[1, 0], A[0, 0])])


def to_mat(p: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    c, s = math.cos(p[2]), math.sin(p[2])
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = p[:2]
    return T


def tracker_motion(rows: np.ndarray, off: float, a: l1.Window, e: l1.Window) -> np.ndarray:
    """推定器の行（扉の座標でのライダー）の区間の中央値 → 段 0 のライダーの座標で見た着いた先"""
    def med(w: l1.Window) -> np.ndarray:
        m = w.mask(rows[:, 0] - off) & (rows[:, 5] > 0)
        if m.sum() < 5:
            raise SystemExit(f"[icp] 区間 {w} に推定器の受け入れた回が {int(m.sum())}")
        return np.r_[np.median(rows[m, 1:3], axis=0), float(np.median(np.unwrap(rows[m, 3])))]
    pa, pe = med(a), med(e)
    c, s = math.cos(-pa[2]), math.sin(-pa[2])
    d = pe[:2] - pa[:2]
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], l1.wrap(np.array([pe[2] - pa[2]]))[0]])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lidar", required=True, type=Path)
    ap.add_argument("--fastlio", required=True, type=Path)
    ap.add_argument("--truth", default=None, help="R0 の正解（無ければ推定器とは比べず、窓は --offset で壁の時計に直す）")
    ap.add_argument("--offset", type=float, default=0.0, help="--truth が無いとき: 壁 − 窓の時計 [s]（0 なら窓は壁の時計）")
    ap.add_argument("--anchor", required=True)
    ap.add_argument("--end", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    if a.truth:
        z = np.load(a.truth, allow_pickle=False)
        off = float(json.loads(str(z["meta_json"]))["clock_offset_s"])
        rows = z["lidar_rows"]
    else:
        off, rows = a.offset, None
    wa, we = l1.Window.parse(a.anchor), l1.Window.parse(a.end)

    # 段 0 の床（l1_fastlio_carry と同じ: 始めの REF_SKIP_S を捨てた 25 回。FAST-LIO の段 0 の姿勢も同じ区間で取る）
    scans0, ta0, ta1 = l1.anchor_scans(a.lidar, wa, off)
    level = est.level_from_scans(scans0)
    L = np.eye(4)
    L[:3, :3], L[2, 3] = level.R, level.h
    clouds = lsi.segment_clouds(a.lidar, [(wa.t0 + off, wa.t1 + off), (we.t0 + off, we.t1 + off)])  # 端は lsi.EDGE_S で削る
    lev = [c @ level.R.T + np.array([0.0, 0.0, level.h]) for c in clouds]

    # FAST-LIO の区間の平均の姿勢
    st, Ts = l1.load_fastlio(a.fastlio)
    recv, hdr = l1.scan_times(a.lidar)
    t_wall, ok, _ = l1.match_to_scans(st, recv, hdr)
    t_wall = t_wall[ok]
    t_mono = t_wall - off
    Ts = Ts[ok]
    T_a = l1.mean_pose(Ts[(t_wall >= ta0) & (t_wall <= ta1 + l1.SCAN_PERIOD_S)])
    T_e = l1.mean_pose(Ts[we.mask(t_mono)])
    fl = l1.planar_track(T_e[None], T_a, level)[0]

    # ICP の初期値は FAST-LIO・FAST-LIO から 10 cm・2° ずらした値・（あれば）推定器。答えがそろい、重なりが十分な時だけ「よい」
    tk = tracker_motion(rows, off, wa, we) if rows is not None else None
    inits = [("FAST-LIO から", fl), ("ずらした値から", fl + PERTURB)] + ([("推定器から", tk)] if tk is not None else [])
    res = {"label": a.label, "床の傾き [°]": round(level.tilt_deg(), 2), "点の数": [int(len(c)) for c in lev]}
    icps, fits = {}, []
    for name, init in inits:
        T, fit, rmse = lsi.icp(lev[1], lev[0], to_mat(init))
        icps[name] = planar_of(T, level)
        fits.append(fit)
        res[f"ICP（{name}）"] = {"前 [cm]": round(100 * icps[name][0], 2), "左 [cm]": round(100 * icps[name][1], 2),
                               "向き [°]": round(math.degrees(icps[name][2]), 3), "fitness": round(fit, 3), "rmse [cm]": round(100 * rmse, 2)}
    gt = np.mean([v for v in icps.values()], axis=0)
    spread = max(float(np.hypot(*(v[:2] - gt[:2]))) for v in icps.values())
    res["ICP の初期値による差 最大 [cm]"] = round(100 * spread, 2)
    res["ICP の確かさ"] = "よい" if spread <= ICP_AGREE_M and min(fits) >= ICP_MIN_FITNESS else "弱い"
    for name, p in (("FAST-LIO", fl), ("推定器", tk)):
        if p is None:
            continue
        d = p - gt
        res[name] = {"動き 前・左 [cm]・向き [°]": [round(100 * p[0], 2), round(100 * p[1], 2), round(math.degrees(p[2]), 3)],
                     "− ICP 前・左 [cm]": [round(100 * d[0], 2), round(100 * d[1], 2)], "− ICP 距離 [cm]": round(100 * float(np.hypot(*d[:2])), 2),
                     "− ICP 向き [°]": round(math.degrees(float(l1.wrap(np.array([d[2]]))[0])), 3)}
    for k, v in res.items():
        print(f"[icp] {k}: {v}", flush=True)
    if a.json:
        a.json.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
