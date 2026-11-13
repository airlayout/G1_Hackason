"""11-12 台本（S8）: 台本の回（door_run_stock.py --staged --script）の記録を、回ごとに 1 枚にまとめる。回と回の間に最初に見る。

  python ana_script_run.py logs_real/20261112/door_stock_<日時>.npz [--door 0.94,2.05,0.99,0.064] [--clear]

11-12 夜: 走らせ役を起こしたまま d を打ち直す形になったので、1 つの記録に回がいくつも入る。DOOR の区間ごとに分けて出す。
走らせ役も、回が止まった時にこの要約を出す（summarize_arrays・numpy だけの軽い集計）。--clear は当たりの模型
（door_clearance.py）で 2b・2c の間のすき間も出す（順運動学を回すので Mac で）。

出すもの: 止まった理由・小段ごとの時間と終わりの値（手のひら−当てる点・扉の角度・肩 pitch と肘の力・体の回り）・
pelvis の動き（脚を止めている小段の動きと、全体）・踏み出しの回数（片足の力が体重の 15% 未満になった回数）・
脚の指令・行き先と手のひらの差・送った目標と実の角度の差・腕の温度・周期。
"""
from __future__ import annotations

import argparse
import ast
import math
from typing import Dict, List, Tuple

import numpy as np

import door_script as dscr
import door_staged as ds

NAMES = ("肩pitch", "肩roll", "肩yaw", "肘", "手首roll", "手首pitch", "手首yaw")
LEGS_STILL = ("2a", "2b", "2c", "2d", "3a")  # 脚を止めている小段（walk_in_3a を除く）
STEP_FRACTION = 0.15  # 片足の力が両足の合計のこれ未満なら「その足が浮いている」
PHASE_DOOR_DEFAULT = 3  # door_run_stock.PHASES の DOOR の番号（記録の付帯情報 phases が無い時）


def load_meta(z) -> Dict:
    m = z["meta"]
    m = m.item() if m.shape == () else m
    return m if isinstance(m, dict) else ast.literal_eval(str(m))


def yaw_deg(quat: np.ndarray) -> np.ndarray:
    w, x, y, zz = quat.T
    return np.degrees(np.arctan2(2 * (w * zz + x * y), 1 - 2 * (y * y + zz * zz)))


def door_index(meta: Dict) -> int:
    phases = meta.get("phases")
    return list(phases).index("DOOR") if phases else PHASE_DOOR_DEFAULT


def door_spans(phase: np.ndarray, door_idx: int = PHASE_DOOR_DEFAULT) -> List[Tuple[int, int]]:
    """DOOR の区間（始め, 終わりの次）の一覧"""
    d = np.asarray(phase).astype(int) == door_idx
    out, k = [], 0
    while k < len(d):
        if d[k]:
            j = k
            while j < len(d) and d[j]:
                j += 1
            out.append((k, j))
            k = j
        else:
            k += 1
    return out


def count_steps(fz: np.ndarray) -> int:
    """片足が浮いた（力が合計の 15% 未満）回数。2〜3 歩踏み出すと 2〜3 回"""
    tot = np.nansum(fz, axis=1)
    lift = np.zeros(len(fz), dtype=bool)
    ok = tot > 1.0
    lift[ok] = np.nanmin(fz[ok], axis=1) < STEP_FRACTION * tot[ok]
    return int(np.sum(lift[1:] & ~lift[:-1]) + (1 if len(lift) and lift[0] else 0))


def summarize_arrays(z: Dict[str, np.ndarray], meta: Dict, door: str, clear: bool = False, title: str = "") -> str:  # noqa: C901
    """記録の配列（1 回ぶん・DOOR を含む）の要約。走らせ役の中からも呼ぶので numpy だけ（clear の時だけ順運動学）"""
    if "script" not in z:
        return f"[ana] {title}: 台本の列（script）が無い（--script の回ではない）"
    args = meta.get("args", {}) or {}
    w, _, lh, bs = (float(v) for v in (args.get("door") or door).split(","))
    lever_left = not args.get("lever_right", False)
    ref = ds.lever_ref_point(w, bs, lh, lever_left)
    t, sc, tau, q, pose = z["t"], z["script"], z["tau"], z["q"], z["pose"]
    ok = np.isfinite(sc[:, 0])
    out = [f"[ana] {title}".rstrip(), f"[ana] 止まった理由: {meta.get('reason', '?')}"]
    if not ok.any():
        return "\n".join(out + ["[ana] 台本に入っていない（段 0・1 で止まった）"])
    idx = np.where(ok)[0]
    t0 = t[idx[0]]
    yaw = yaw_deg(z["quat"])
    sub = sc[:, 0]
    starts = [idx[0]] + [k for a, k in zip(idx[:-1], idx[1:]) if sub[k] != sub[a]]
    out.append("[ana] 小段   始まり  長さ   手のひら−当てる点 [前,横,上] cm   扉°   肩pitch/肘の力 N·m（終わり・最大）  体の回り°  pelvis の動き cm")
    for n, k0 in enumerate(starts):
        k1 = (starts[n + 1] - 1) if n + 1 < len(starts) else idx[-1]
        seg = slice(k0, k1 + 1)
        d = 100 * (sc[k1, 4:7] - ref)
        tp, te = tau[seg, 15], tau[seg, 18]
        mv = 100 * math.hypot(pose[k1, 0] - pose[k0, 0], pose[k1, 1] - pose[k0, 1])
        out.append(f"[ana] {dscr.SUBS[int(sub[k0])]:7s} {t[k0] - t0:6.2f} {t[k1] - t[k0]:5.2f}   [{d[0]:+6.1f},{d[1]:+6.1f},{d[2]:+6.1f}]"
                   f"            {sc[k1, 7]:5.1f}   {tp[-1]:+5.1f}/{te[-1]:+5.1f}・{np.abs(tp).max():4.1f}/{np.abs(te).max():4.1f}"
                   f"            {(yaw[k1] - yaw[idx[0]] + 180.0) % 360.0 - 180.0:+5.1f}      {mv:5.1f}")  # ±180° の折り返し（11-13: −322.4 と出た）
    lag = sc[ok, 8]
    walk = z["vel_want"][ok][:, 0] > 0
    pulses = int(np.sum(walk[1:] & ~walk[:-1]) + (1 if walk[0] else 0))
    p_ok = pose[ok]
    still = np.isin(np.array([dscr.SUBS[int(v)] for v in sub[ok]]), LEGS_STILL)
    if still.any():
        ps = p_ok[still]
        still_mv = 100 * float(np.max(np.hypot(ps[:, 0] - ps[0, 0], ps[:, 1] - ps[0, 1])))
    else:
        still_mv = math.nan
    out.append(f"[ana] 扉の最大 {np.nanmax(sc[ok, 7]):.1f}°・遅れ（手のひら−行き先の上下）最大 "
               f"{100 * np.nanmax(np.abs(lag)) if np.isfinite(lag).any() else float('nan'):.1f} cm・手の前への出 最大 {np.nanmax(sc[ok, 10]):.2f} m")
    if "hand_f" in z:  # 11-13: 手が受ける力の見積もり（扉の座標）。前後は扉が押し返す向きが負・上下はレバーが押し上げる向きが正
        for name in ("3a", "3c"):
            kk = np.where(ok & (sub == dscr.SUBS.index(name)))[0]
            hf = z["hand_f"][kk]
            hf = hf[np.isfinite(hf).all(axis=1)]
            if len(hf) > 5:
                out.append(f"[ana] 手の力 {name}: 前後 中央 {np.median(hf[:, 0]):+.0f} N・山 {hf[:, 0].min():+.0f} N・押している割合 "
                           f"{100 * np.mean(hf[:, 0] < -3.0):.0f}%・レバーを押さえる力 中央 {np.median(hf[:, 2]):+.0f} N・横 中央 {np.median(hf[:, 1]):+.0f} N")
    steps = count_steps(z["foot_fz"][ok]) if "foot_fz" in z else -1
    out.append(f"[ana] 脚: 指令を出した回数 {pulses}・出していた時間 {walk.sum() * 0.02:.1f} 秒・踏み出し {steps} 回・"
               f"pelvis の進み 前 {100 * (p_ok[-1, 0] - p_ok[0, 0]):+.1f} 横 {100 * (p_ok[-1, 1] - p_ok[0, 1]):+.1f} cm・"
               f"向き {math.degrees(p_ok[-1, 2] - p_ok[0, 2]):+.1f}°・脚を止めている間の動き 最大 {still_mv:.1f} cm")
    goal_err = np.linalg.norm(sc[ok, 1:4] - sc[ok, 4:7], axis=1)
    if np.isfinite(goal_err).any():
        out.append(f"[ana] 行き先と手のひらの差: 中央 {100 * np.nanmedian(goal_err):.1f} cm・最大 {100 * np.nanmax(goal_err):.1f} cm")
    side = slice(7, 14) if args.get("lever_right") else slice(0, 7)
    qs = slice(22, 29) if args.get("lever_right") else slice(15, 22)
    sent, q_l = z["arm_sent"][ok][:, side], q[ok][:, qs]
    lead = np.nanmax(np.abs(sent - q_l), axis=0)
    out.append("[ana] 送った目標と実の角度の差の最大 [rad]: " + "・".join(f"{n} {v:.2f}" for n, v in zip(NAMES, lead)))
    temp = z["arm_temp"]
    after = slice(idx[-1], len(t))
    out.append(f"[ana] 腕の温度: 台本の始め 最高 {np.nanmax(temp[idx[0]]):.0f} °C → 台本の終わり {np.nanmax(temp[idx[-1]]):.0f} °C"
               f"（関節 {int(np.nanargmax(temp[idx[-1]]))}）・止めた後の最高 {np.nanmax(temp[after]):.0f} °C・"
               f"周期 中央 {1000 * np.median(z['period'][ok]):.1f} ms・最大 {1000 * z['period'][ok].max():.1f} ms")
    if clear:
        out.extend(clearance_lines(z, ok, args, w, lh, bs, lever_left))
    return "\n".join(out)


def clearance_lines(z, ok, args, w, lh, bs, lever_left) -> List[str]:
    """当たりの模型で、2b・2c の間の腕と扉の周りのすき間の最小（記録の姿勢で）"""
    import door_clearance as dcl
    import door_frame as dfr
    import g1_arm_fk as fk
    chain = fk.load_chain()
    g = dcl.load_geom(None, (w, 2.05, lh, bs), lever_left)
    obs = dcl.obstacles(g)
    pz = float(args.get("pelvis_z") or 0.73)
    out = []
    for si, name in ((1, "2b"), (2, "2c")):
        rows = np.where(ok & (z["script"][:, 0] == si))[0]
        if not len(rows):
            continue
        best = {k: (math.inf, "") for k in obs}
        for k in rows[::2]:
            segs = dcl.arm_segments(chain, z["q"][k], dfr.PlanarPose(*z["pose"][k]), z["quat"][k], pz, g,
                                    "left" if lever_left else "right")
            for o, (d, part) in dcl.clearances(segs, g, obs).items():
                if d < best[o][0]:
                    best[o] = (d, part)
        out.append(f"[ana] すき間（{name}・当たりの模型・記録の姿勢）cm: " + "・".join(f"{o} {100 * d:+.1f}（{p}）" for o, (d, p) in best.items()))
    return out


def summarize(path: str, door: str, clear: bool = False) -> str:
    z = np.load(path, allow_pickle=True)
    meta = load_meta(z)
    arrs = {k: z[k] for k in z.files if k != "meta"}
    spans = door_spans(arrs["phase"], door_index(meta)) if "phase" in arrs else []
    if len(spans) <= 1:
        return summarize_arrays(arrs, meta, door, clear, path)
    out = []
    for n, (a0, a1) in enumerate(spans):
        b1 = spans[n + 1][0] if n + 1 < len(spans) else len(arrs["t"])
        part = {k: v[a0:b1] for k, v in arrs.items() if hasattr(v, "__len__") and len(v) == len(arrs["t"])}
        m = dict(meta, reason=meta.get("reason", "?") if n + 1 == len(spans) else "（次の回の前に止まった。理由は画面の出力）")
        out.append(summarize_arrays(part, m, door, clear, f"{path} の {n + 1} 回目"))
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--door", default="0.94,2.05,0.99,0.064")
    ap.add_argument("--clear", action="store_true", help="当たりの模型のすき間も出す（Mac で）")
    a = ap.parse_args(argv)
    for f in a.npz:
        print(summarize(f, a.door, a.clear))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
