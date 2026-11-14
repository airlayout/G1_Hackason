"""11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D・E・F）: pelvis を再生したシムの台本の回
（s0_replay_1114.sh の JSON Lines・毎周期の記録・static.npz）を、同じ回番号の実機の記録と、計画 7 節の物差しで並べる。Mac で回す。

  python s0_bench_cmp_1114.py --sim <jsonl> --records <records/start_… の写し> [--sim …] [--real "…/door_stock_*.npz"] [--json out.json]

物差し（実機とシムで同じ式）:
  下がり     2d の始めから終わりまでの手のひらの下がり [cm]
  外れた     実機: 作業ログ 11-13 の判定（fit_door_from_records の OPENED・RELEASED・LATCH_UNKNOWN）。
             シム: 本当の扉の角度が 3a に入ってから 1° を越えた（仮想の掛け金は外れる角度に届くまで 0.5° 内に止める）
  落ちた     3a の入りから手のひらが更に 4 cm 以上下がった（手がレバーの下）。時刻は 3a の入りから
  流れ       3a の入りから 1 秒の、手のひらの横（棒に沿う向き）の動き [cm]
  手の力     2d・3a の中央 [前後, 横, 上下] N（実機は関節の力からの見積もり、シムは同じ見積もり（台本の hand_f）と接触センサ）
  再生のずれ 3a の入りから 3 秒の pelvis（扉の座標）の、記録の差との水平のずれの最大 [cm]（段 D の線 1 cm）
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "real"))
import door_script as dscr  # noqa: E402
import fit_door_from_records as fdr  # noqa: E402
import hand_door_angle as hda  # noqa: E402

S2D, S3A, S3C = (dscr.SUBS.index(s) for s in ("2d", "3a", "3c"))
# 落ちた: 3a の入りから手のひらが更に FELL_EXTRA_M 下がった（11-13 実機: 落ちた 15・19・21 回目は更に 5〜6.5 cm、落ちなかった回は 1 cm 内）。
# 2d の始めからの下がりで数えると、シムは 2d で 4〜6 cm しか下げずにレバーが止まりに当たる（実機は 7〜10 cm）ので偏る（11-14）
FELL_EXTRA_M = 0.04
OPEN_DEG = 1.0
SLIDE_S = 1.0
REPLAY_CHECK_S = 3.0
# 毎周期の記録の列（script_teacher.py の traces）
C_T, C_SUB, C_PALM, C_POSE, C_HINGE, C_LEVER, C_FEST, C_FTRUE = 0, 1, slice(3, 6), slice(23, 27), 27, 28, slice(29, 32), slice(32, 35)


def _med(x) -> List[float]:
    x = np.asarray(x, dtype=float)
    x = x[np.all(np.isfinite(x), axis=1)] if x.ndim == 2 and len(x) else x
    return np.round(np.median(x, axis=0), 1).tolist() if len(x) else [math.nan] * 3


def metrics(t, sub, palm, f_est, f_true=None, hinge_deg=None) -> Dict:
    """実機とシムで同じ物差し（1 回ぶん）"""
    out: Dict = {}
    d2 = np.where(sub == S2D)[0]
    a3 = np.where(sub == S3A)[0]
    if not len(d2):
        return {"reached": "before_2d"}
    out["press_cm"] = round(100 * float(palm[d2[0], 2] - palm[d2[-1], 2]), 1)
    out["f_2d"] = _med(f_est[d2])
    if f_true is not None:
        out["f_true_2d"] = _med(f_true[d2])
    if not len(a3):
        out["reached"] = "2d"
        return out
    out["reached"] = "3a"
    k0 = int(a3[0])
    t0 = float(t[k0])
    out["dur_3a"] = round(float(t[a3[-1]] - t[a3[0]]), 2)
    out["f_3a"] = _med(f_est[a3])
    if f_true is not None:
        out["f_true_3a"] = _med(f_true[a3])
    after = np.arange(k0, len(t))
    drop = palm[k0, 2] - palm[after, 2]
    fell = np.where(drop > FELL_EXTRA_M)[0]
    out["fell"] = bool(len(fell))
    out["fell_t"] = round(float(t[after[fell[0]]] - t0), 2) if len(fell) else None
    # 3a の間（3c に入る前）・3a の入りから 1 秒の内に落ちたか（段 F の「3a で手が残るか」）
    out["fell_in_3a"] = bool(len(fell) and out["fell_t"] <= out["dur_3a"])
    out["fell_1s"] = bool(len(fell) and out["fell_t"] <= 1.0)
    k1 = after[np.searchsorted(t[after] - t0, SLIDE_S).clip(0, len(after) - 1)]
    out["slide_y_cm"] = round(100 * float(palm[k1, 1] - palm[k0, 1]), 1)
    if hinge_deg is not None:
        op = np.where(hinge_deg[after] > OPEN_DEG)[0]
        out["opened"] = bool(len(op))
        out["opened_t"] = round(float(t[after[op[0]]] - t0), 2) if len(op) else None
    out["to_3c"] = bool(np.any(sub[after] == S3C))
    return out


PUSH_N = 3.0


def metrics_3c(t, sub, angle_deg, pose, f_est) -> Dict:
    """3c の物差し（計画 13.2）: 最初の 3 秒の扉の開く速さ [°/s]・扉の最大 [°]・pelvis の進み（前）[cm]・押している割合（前後の力 < −3 N）"""
    m = np.where(sub == S3C)[0]
    if len(m) < 10:
        return {}
    t0 = float(t[m[0]])
    k3 = m[t[m] - t0 <= 3.0]
    fx = np.asarray(f_est)[m, 0]
    fx = fx[np.isfinite(fx)]
    return {"rate_3s": round(float((angle_deg[k3[-1]] - angle_deg[k3[0]]) / max(0.1, t[k3[-1]] - t[k3[0]])), 2),
            "max_deg": round(float(np.nanmax(angle_deg[m])), 1), "adv_cm": round(100 * float(pose[m[-1], 0] - pose[m[0], 0]), 1),
            "push_frac": round(float(np.mean(fx < -PUSH_N)), 2) if len(fx) else None, "dur_3c": round(float(t[m[-1]] - t0), 1)}


def palm_angle_deg(palm: np.ndarray, sub: np.ndarray, width: float) -> np.ndarray:
    """台本と同じ読み方の扉の角度（3a の入りの手のひらを基準に、蝶番の回りの手のひらの回り）"""
    a3 = np.where(sub == S3A)[0]
    out = np.zeros(len(palm))
    if not len(a3):
        return out
    line = hda.door_line(width, True)
    ref = palm[a3[0]]
    for k in range(a3[0], len(palm)):
        out[k] = np.degrees(hda.door_angle(line, ref, palm[k], "hinge"))
    return out


def real_rows(paths: List[str]) -> Dict[int, Dict]:
    out = {}
    for sr in fdr.extract_runs(paths):
        m = metrics(sr.t, sr.sub, sr.palm, sr.force)
        m["m3c"] = metrics_3c(sr.t, sr.sub, sr.angle_deg, sr.pose, sr.force)
        lb = sr.label
        m["latch"] = ("opened" if lb in fdr.OPENED_1113 else "unknown" if lb in fdr.LATCH_UNKNOWN_1113
                      else "released" if lb in fdr.RELEASED_1113 else "not")
        out[lb] = m
    return out


def sim_rows(jsonl: str, records: str, replay_npz: str) -> List[Dict]:
    rows = [json.loads(s) for s in open(jsonl)]
    static = dict(np.load(os.path.join(records, "static.npz"), allow_pickle=True)) if records else {}
    rel = static.get("door_lever_latch_release_deg")
    rep = np.load(replay_npz) if replay_npz else None
    out = []
    for r in rows:
        i, ep = r["env"], r["episode"]
        tr = f"{jsonl}.trace_env{i}_ep{ep}.npy"
        if not os.path.isfile(tr):
            continue
        a = np.load(tr)
        sub = a[:, C_SUB].astype(int)
        m = metrics(a[:, C_T], sub, a[:, C_PALM], a[:, C_FEST], a[:, C_FTRUE], np.degrees(np.abs(a[:, C_HINGE])))
        ang = palm_angle_deg(a[:, C_PALM], sub, float(r.get("door_width", 0.94)))
        m["m3c"] = metrics_3c(a[:, C_T], sub, ang, a[:, C_POSE], a[:, C_FEST])
        m["m3c_true"] = metrics_3c(a[:, C_T], sub, np.degrees(np.abs(a[:, C_HINGE])), a[:, C_POSE], a[:, C_FEST])
        m.update(push_back_n=r.get("push_back_n"), sway_n=r.get("sway_n"), push_f_peak=r.get("push_f_peak"))
        m.update(env=i, episode=ep, label=r.get("replay_label"), err_max_cm=r.get("replay_err_max_cm"),
                 release_deg=None if rel is None else round(float(rel[i]), 1),
                 lever_max_deg=round(float(np.degrees(np.max(np.abs(a[:, C_LEVER])))), 1), stop=r.get("stop", "")[:60])
        a3 = np.where(sub == S3A)[0]
        if rep is not None and len(a3) and r.get("replay_label") is not None:
            k = int(np.where(rep["labels"] == r["replay_label"])[0][0])
            t_rel = a[a3[0]:, C_T] - a[a3[0], C_T]
            use = t_rel <= REPLAY_CHECK_S
            p = a[a3[0]:, C_POSE][use]
            d_sim = p[:, :2] - p[0, :2]
            d_rec = np.stack([np.interp(t_rel[use], rep["t"], rep["d"][k, :, j]) for j in (0, 1)], axis=1)
            m["replay_dev_cm"] = round(100 * float(np.max(np.linalg.norm(d_sim - d_rec, axis=1))), 2)
        out.append(m)
    return out


def summarize(real: Dict[int, Dict], sims: List[Dict], check: List[int]) -> Dict:
    by = defaultdict(list)
    for m in sims:
        by[m.get("label")].append(m)
    lines, table = [], []
    agree_latch, n_latch, agree_fell, n_fell, fdiff = 0, 0, 0, 0, []
    for lb in sorted(by):
        ms, rr = by[lb], real.get(lb, {})
        at3 = [m for m in ms if m.get("reached") == "3a"]
        op = np.mean([m["opened"] for m in at3]) if at3 else math.nan
        fell = np.mean([m["fell"] for m in at3]) if at3 else math.nan
        fell_t = [m["fell_t"] for m in at3 if m.get("fell_t") is not None]
        f3 = np.nanmedian([m["f_3a"] for m in at3], axis=0) if at3 else [math.nan] * 3
        dev = [m["replay_dev_cm"] for m in at3 if m.get("replay_dev_cm") is not None]
        row = {"label": lb, "check": lb in check, "n": len(ms), "n_3a": len(at3), "sim_opened": op, "sim_fell": fell,
               "sim_fell_t": float(np.median(fell_t)) if fell_t else None, "sim_press": float(np.median([m["press_cm"] for m in ms if "press_cm" in m])),
               "sim_slide": float(np.median([m["slide_y_cm"] for m in at3])) if at3 else math.nan,
               "sim_f3a": np.round(f3, 1).tolist(), "replay_dev_cm": max(dev) if dev else None,
               "real": rr}
        table.append(row)
        if rr.get("latch") in ("opened", "released", "not") and at3:
            n_latch += 1
            agree_latch += int((op >= 0.5) == (rr["latch"] != "not"))
        if "fell" in rr and at3:
            n_fell += 1
            ok = (fell >= 0.5) == rr["fell"]
            if ok and rr["fell"] and fell_t and rr.get("fell_t") is not None:
                ok = abs(float(np.median(fell_t)) - rr["fell_t"]) <= 0.5
            agree_fell += int(ok)
        if "f_3a" in rr and at3:
            fdiff.append(np.abs(np.asarray(f3) - np.asarray(rr["f_3a"])))
    return {"table": table, "latch_agree": [agree_latch, n_latch], "fell_agree": [agree_fell, n_fell],
            "f3a_absdiff_med": np.round(np.nanmedian(fdiff, axis=0), 1).tolist() if fdiff else None}


def fmt(v, f="{:.0f}"):
    return "—" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f.format(v)


def print_table(s: Dict) -> None:
    print("  回  側    台(3a)  下げ cm 実/シム   外れた 実/シム(割合)   落ちた 実/シム(割合)・時刻 秒 実/シム   流れ cm 実/シム   "
          "3a の手の力 [前,横,上] 実 / シム   再生のずれ cm")
    for r in s["table"]:
        rr = r["real"]
        print(f"  {r['label']:2d}  {'確' if r['check'] else '決'}   {r['n']:2d}({r['n_3a']:2d})   {fmt(rr.get('press_cm'), '{:.1f}')} / {fmt(r['sim_press'], '{:.1f}')}"
              f"      {rr.get('latch', '—'):8s} / {fmt(r['sim_opened'], '{:.2f}')}"
              f"      {fmt(rr.get('fell'), '{}')} / {fmt(r['sim_fell'], '{:.2f}')}・{fmt(rr.get('fell_t'), '{:.2f}')} / {fmt(r['sim_fell_t'], '{:.2f}')}"
              f"      {fmt(rr.get('slide_y_cm'), '{:+.1f}')} / {fmt(r['sim_slide'], '{:+.1f}')}"
              f"      {rr.get('f_3a')} / {r['sim_f3a']}   {fmt(r['replay_dev_cm'], '{:.2f}')}")
    print(f"  外れた・外れないの当たり {s['latch_agree'][0]}/{s['latch_agree'][1]}・落ちた・落ちないの当たり（時刻 0.5 秒の内）"
          f"{s['fell_agree'][0]}/{s['fell_agree'][1]}・3a の手の力の差の中央 {s['f3a_absdiff_med']} N")


def print_g(real: Dict[int, Dict], groups: Dict[str, List[Dict]], gmap: Dict[str, int]) -> List[Dict]:
    """段 G: シムの組（脚を歩かせた 3c）ごとの中央 と、対応する実機の回の 3c の物差し"""
    out = []
    print("  組           台(3c)  最初 3 秒の扉 °/s 実/シム(手のひら)/シム(本当)   扉の最大 ° 実/シム/本当   pelvis の進み cm 実/シム   押した割合 実/シム   下がった回数")
    for tag, ms in groups.items():
        parts = tag.split("_")  # g_ga_off・g_ga_pushoff など → ga
        lb = gmap.get(parts[1] if len(parts) > 1 else tag, None)
        rr = real.get(lb, {}).get("m3c", {}) if lb else {}
        c = [m for m in ms if m.get("m3c")]
        ct = [m["m3c_true"] for m in c]
        med = lambda key, rows: float(np.median([r[key] for r in rows if r.get(key) is not None])) if rows else math.nan  # noqa: E731
        row = {"tag": tag, "real_label": lb, "n": len(ms), "n_3c": len(c), "real": rr,
               "sim": {k: med(k, [m["m3c"] for m in c]) for k in ("rate_3s", "max_deg", "adv_cm", "push_frac")},
               "sim_true": {k: med(k, ct) for k in ("rate_3s", "max_deg")},
               "push_back_n": med("push_back_n", c)}
        out.append(row)
        print(f"  {tag:14s} {len(ms):2d}({len(c):2d})   {fmt(rr.get('rate_3s'), '{:+.1f}')} / {fmt(row['sim']['rate_3s'], '{:+.1f}')} / {fmt(row['sim_true']['rate_3s'], '{:+.1f}')}"
              f"            {fmt(rr.get('max_deg'), '{:.1f}')} / {fmt(row['sim']['max_deg'], '{:.1f}')} / {fmt(row['sim_true']['max_deg'], '{:.1f}')}"
              f"        {fmt(rr.get('adv_cm'), '{:+.0f}')} / {fmt(row['sim']['adv_cm'], '{:+.0f}')}"
              f"          {fmt(rr.get('push_frac'), '{:.2f}')} / {fmt(row['sim']['push_frac'], '{:.2f}')}      {fmt(row['push_back_n'], '{:.1f}')}")
    return out


def print_by_tag(tags: List[str], sims: List[Dict]) -> List[Dict]:
    """段 F: 同じ体の動きの下で、形（--sim の名前）ごとに並べる。数えるのは 3a に入った台"""
    out = []
    print("  形           台  3a  外れた割合  落ちた割合・時刻の中央 秒  3a の間に落ちた・1 秒の内  3a の長さ  3c へ   流れ cm   レバーの最大°   3a の本当の力 [前,横,上]   止まり方（多い順）")
    for k, tag in enumerate(tags):
        ms = [m for m in sims if m.get("_src") == k]
        at3 = [m for m in ms if m.get("reached") == "3a"]
        fell_t = [m["fell_t"] for m in at3 if m.get("fell_t") is not None]
        stops = defaultdict(int)
        for m in ms:
            stops[(m.get("stop") or "（止まらず）")[:24]] += 1
        row = {"tag": tag, "n": len(ms), "n_3a": len(at3),
               "opened": float(np.mean([m["opened"] for m in at3])) if at3 else math.nan,
               "fell": float(np.mean([m["fell"] for m in at3])) if at3 else math.nan,
               "fell_t": float(np.median(fell_t)) if fell_t else None,
               "fell_in_3a": float(np.mean([m["fell_in_3a"] for m in at3])) if at3 else math.nan,
               "fell_1s": float(np.mean([m["fell_1s"] for m in at3])) if at3 else math.nan,
               "dur_3a": float(np.median([m["dur_3a"] for m in at3])) if at3 else math.nan,
               "to_3c": float(np.mean([m["to_3c"] for m in at3])) if at3 else math.nan,
               "slide": float(np.median([m["slide_y_cm"] for m in at3])) if at3 else math.nan,
               "lever_max": float(np.median([m["lever_max_deg"] for m in at3])) if at3 else math.nan,
               "f_true_3a": np.round(np.nanmedian([m["f_true_3a"] for m in at3], axis=0), 1).tolist() if at3 else None,
               "stops": sorted(stops.items(), key=lambda kv: -kv[1])[:3]}
        out.append(row)
        print(f"  {tag:12s} {row['n']:3d} {row['n_3a']:3d}     {fmt(row['opened'], '{:.2f}')}        {fmt(row['fell'], '{:.2f}')}・{fmt(row['fell_t'], '{:.2f}')}"
              f"             {fmt(row['fell_in_3a'], '{:.2f}')}・{fmt(row['fell_1s'], '{:.2f}')}            {fmt(row['dur_3a'], '{:.2f}')}    {fmt(row['to_3c'], '{:.2f}')}"
              f"   {fmt(row['slide'], '{:+.1f}')}      {fmt(row['lever_max'], '{:.1f}')}       {row['f_true_3a']}   {row['stops']}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="append", required=True, help="JSON Lines（毎周期の記録は同じ名前＋.trace_env…）")
    ap.add_argument("--records", action="append", default=[], help="--sim と同じ順の records/start_… の写し（static.npz）")
    ap.add_argument("--replay", default=os.path.join(HERE, "..", "real", "logs_real", "robot_20261113", "pelvis_replay_1114.npz"))
    ap.add_argument("--real", default=os.path.join(HERE, "..", "real", "logs_real", "robot_20261113", "door_stock_*.npz"))
    ap.add_argument("--json", default="")
    ap.add_argument("--by-tag", action="store_true", help="段 F: --sim ごと（形ごと）に並べる")
    ap.add_argument("--g", default="", help="段 G: 組の名前=実機の回（例 ga=1,gb=8,gc=15）。--sim の名前の g_<組>_push… で組を見分ける")
    a = ap.parse_args(argv)
    real = real_rows(sorted(glob.glob(a.real)))
    sims = []
    for k, j in enumerate(a.sim):
        rows = sim_rows(j, a.records[k] if k < len(a.records) else "", a.replay)
        for m in rows:
            m["_src"] = k
        sims += rows
    _, check = fdr.split_runs(sorted(real))
    if a.by_tag:
        s = {"by_tag": print_by_tag([os.path.basename(j).rsplit("_1114_", 1)[0] for j in a.sim], sims)}
    elif a.g:
        gmap = {kv.split("=")[0]: int(kv.split("=")[1]) for kv in a.g.split(",")}
        groups: Dict[str, List[Dict]] = defaultdict(list)
        for k, j in enumerate(a.sim):
            tag = os.path.basename(j).rsplit("_1114_", 1)[0]
            groups[tag] += [m for m in sims if m.get("_src") == k]
        s = {"g": print_g(real, groups, gmap)}
    else:
        s = summarize(real, sims, check)
        print_table(s)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"summary": s, "sim_rows": sims}, f, ensure_ascii=False, indent=1, default=float)
        print(f"[cmp] 書いた: {a.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
