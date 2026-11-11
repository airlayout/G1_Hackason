"""11-10 手順 L2 の確かめ: L1 の記録（生の点群・FAST-LIO の再生の出力）を door_fastlio_node の本体に逐次に流し、
L1（l1_fastlio_carry_1110.py・まとめて計算）と同じ値になるかと、R0 の正解に対する着いた先の外れを出す。

段 0 の扉の姿勢（REF に載せる値）は L1 と同じく、段 0 の 25 回の R0 の推定器の中央値（実機では段 0 の網 c2 の値になる）。
REF の時刻は L1 の段 0 の回の選び方（受け取り − 55 ms が段 0 の始め ＋ 1 秒以後）にそろえる。

使い方（OMEN・real_ana_1106 で）:
  python l2_replay_check_1110.py --label "18:33 寄る" --fastlio <tf_bag> --lidar <セッション> --truth truth_183336_S.npz \\
      --anchor 1500:1525 --end 1545:1560 [--l1-npz l1a_w1_vox.npz] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_fastlio_node as dfn  # noqa: E402
import l1_fastlio_carry_1110 as l1  # noqa: E402
import real_truth_1110 as rt  # noqa: E402

PASS_SAME_M = 0.002   # L1 と同じ値（逐次とまとめての差）
PASS_TRUTH_M = 0.02   # 計画書 9.4 L1 (a)・L2: R0 の正解に対して


def anchor_door_pose(truth: np.ndarray, recv: np.ndarray, anchor: l1.Window, off: float) -> np.ndarray:
    """L1 と同じ段 0 の回（受け取り − 55 ms が段 0 の始め ＋ REF_SKIP_S 以後の 25 回）の R0 の推定器の中央値"""
    t = recv - rt.SCAN_DELAY_S
    sel = np.nonzero((t >= anchor.t0 + off + rt.REF_SKIP_S) & (t <= anchor.t1 + off))[0][: rt.STILL_SCANS]
    if len(sel) < rt.STILL_SCANS:
        raise SystemExit(f"[l2] 段 0 の区間に回が {len(sel)} しか無い")
    ta0, ta1 = t[sel[0]], t[sel[-1]]
    tm = (truth[:, 0] >= ta0) & (truth[:, 0] <= ta1) & (truth[:, 5] > 0)
    if tm.sum() < 10:
        raise SystemExit(f"[l2] 段 0 の区間に正解の受け入れた回が {int(tm.sum())} しか無い")
    return np.r_[np.median(truth[tm, 1:3], axis=0), float(np.median(np.unwrap(truth[tm, 3])))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fastlio", required=True, type=Path)
    ap.add_argument("--lidar", required=True, type=Path)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--stock", default=None)
    ap.add_argument("--anchor", required=True)
    ap.add_argument("--end", required=True)
    ap.add_argument("--l1-npz", type=Path, default=None, help="L1 の --npz（lidar_fastlio と比べる）")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    z = np.load(a.truth, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    off, head_xy, truth = float(meta["clock_offset_s"]), tuple(meta["head_xy"]), z["lidar_rows"]
    stock = rt.load_stock(a.stock or meta["stock"])
    anchor, end = l1.Window.parse(a.anchor), l1.Window.parse(a.end)
    recv, _ = l1.scan_times(a.lidar)
    d_a = anchor_door_pose(truth, recv, anchor, off)

    node = dfn.FastlioCarry()
    t0 = time.time()
    out = dfn.run_replay(node, dfn.replay_events(a.lidar, a.fastlio), anchor.t0 + off + rt.SCAN_DELAY_S, tuple(d_a))
    cost = time.time() - t0
    if not out:
        raise SystemExit(f"[l2] 電文が 0（{node.last_error or node.stats}）")
    t_end = np.array([m.stamp for m in out])
    lid_end = np.array([[m.x, m.y, m.yaw] for m in out])
    # 推定器・正解と同じ「走査の真ん中」の時刻へ（L1 と同じ補間）
    t_mid = t_end - dfn.SCAN_PERIOD_S / 2
    lid = l1.interp_planar(t_mid, t_end, lid_end)
    rows = np.full((len(lid), len(rt.ROW_COLS)), np.nan)
    rows[:, 0], rows[:, 1:4], rows[:, 5] = t_mid, lid, 1.0
    pel = rt.pelvis_rows(rows, off, stock, head_xy)
    t_mono = t_mid - off

    jt = l1.match_rows(t_mid, truth[:, 0])
    acc = np.zeros(len(jt), bool)
    acc[jt >= 0] = truth[jt[jt >= 0], 5] > 0
    pel_tr = np.full_like(pel, np.nan)
    pel_tr[acc] = rt.pelvis_rows(truth[jt[acc]], off, stock, head_xy)
    d = pel - pel_tr
    d[:, 2] = l1.wrap(d[:, 2])
    me = end.mask(t_mono) & acc
    res: Dict = {"label": a.label, "電文": len(out), "本体の数え": node.stats, "段 0 の床の傾き [°]": round(node.level.tilt_deg(), 2),
                 "再生の手間 [ms/出力]": round(1000 * cost / max(1, node.stats["odom"]), 2),
                 "着いた先（止まった区間）": l1.err_stats(d[me]), "全部": l1.err_stats(d[acc])}
    if me.any():
        e_end = float(np.hypot(*np.median(d[me, :2], axis=0)))
        res["合否 R0 の正解に対して 2 cm"] = bool(e_end <= PASS_TRUTH_M)
    if a.l1_npz:
        z1 = np.load(a.l1_npz, allow_pickle=False)
        j = l1.match_rows(t_mono, z1["t_mono"])
        ok = j >= 0
        dd = lid[ok] - z1["lidar_fastlio"][j[ok]]
        dd[:, 2] = l1.wrap(dd[:, 2])
        res["L1 との差"] = {"同じ回": int(ok.sum()), "L1 の回": int(len(z1["t_mono"])),
                         "位置 最大 [mm]": round(1000 * float(np.hypot(dd[:, 0], dd[:, 1]).max()), 3) if ok.any() else None,
                         "向き 最大 [°]": round(math.degrees(float(np.abs(dd[:, 2]).max())), 4) if ok.any() else None}
        if ok.any():
            res["合否 L1 と同じ（2 mm）"] = bool(float(np.hypot(dd[:, 0], dd[:, 1]).max()) <= PASS_SAME_M)
    for k, v in res.items():
        print(f"[l2]   {k}: {v}", flush=True)
    if a.json:
        a.json.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
