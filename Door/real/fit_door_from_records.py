"""11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 A・B）: 実機の台本の記録から、
シムの扉の値を幅付き（下・中央・上）で出す。新しく測らず、11-13 朝の記録だけを使う。

  python fit_door_from_records.py logs_real/robot_20261113/door_stock_*.npz [--out fit_door_1114.json] [--table]

出す値（扉の座標: +x 押して入る向き・把手が左ならレバーは +y・付け根は y = 幅/2 − バックセット）:
  レバーのばね   回り τ = τ0 + k·θ。τ は手の先の力から付け根の回りに取る（τ = s·Fz − 深さ·Fy・棒に沿う摩擦は回りを生まない）。
                 θ = atan(深さ / s)。s は手の先と付け根の横の距離、深さは触れた所（上下の力が 3 N を 0.2 秒越えた所）からの下がり
  レバーの止まり 3a の間のレバーの角度（押し下げ 10 cm の回は止まりに当たって手が棒に沿って流れる）
  掛け金        外れた回の 3a の角度の最小が上限・外れなかった回の最大が下限（無ければ下限は決まらない）
  手の滑り      棒に沿って手が流れている間の、棒に沿う力 ÷ 法線の力（＝動いている間の摩擦の比）
  クローザー    扉が開いた回の 3c の、手が扉を押す力 × 蝶番からの距離（開ける向きの回り）

手の力は hand_force.py（手の先 0.10 m）で関節の力から解く。基準は 2b の終わり 0.5 秒（触れる前・腕の形が近い）。
⚠️ 見積もり: 関節の摩擦・腕の加速は見ていない。手のひらの位置はなましていない姿勢（記録の pose）から出す。
値を決める回と確かめる回は split_runs で分ける（3 回に 1 回を確かめる側へ・計画 7 節）。
"""
from __future__ import annotations

import argparse
import glob
import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

import door_script as dscr

S2B, S2C, S2D, S3A, S3C = (dscr.SUBS.index(s) for s in ("2b", "2c", "2d", "3a", "3c"))
DT = 0.02                 # 記録の周期（50 Hz）
CONTACT_LINE_N = 3.0      # 触れたとみなす上下の力 [N]
CONTACT_HOLD = 10         # その力が続く周期（0.2 秒）
SLIDE_RATE = 0.005        # 手が棒に沿って流れているとみなす速さ [m/s]（0.2 秒の差で見る）
SLIDE_WIN = 10            # 流れの速さを見る幅（周期）
FELL_UNDER_M = 0.14       # 触れた所からこれより下がったら手はレバーの下（止まりの 45° で手が棒の先 13 cm まで流れても 13 cm。
                          # 11-13 は 15・19・21 回目が 17〜18 cm・棒に残った回は 12.4 cm まで）
MIN_FZ_N = 2.0            # ばねの合わせに使う上下の力の下限 [N]
CALIB_N = 25              # 力の基準の周期（2b の終わり 0.5 秒）
PUSH_LINE_N = 3.0         # 3c で「押している」とみなす前後の力 [N]
OPEN_WINDOW_S = 6.0       # クローザーを見る 3c の最初の秒
BAND_PCT = (10.0, 50.0, 90.0)


@dataclass(frozen=True)
class RunSeries:
    """1 回ぶん（2c〜3c）の系列。tip・palm は扉の座標 [m]、force は手が外から受ける力（扉の座標）[N]"""
    label: int
    t: np.ndarray
    sub: np.ndarray
    tip: np.ndarray
    palm: np.ndarray
    force: np.ndarray
    pose: np.ndarray
    angle_deg: np.ndarray
    y_root: float
    r_hinge: float
    stop_reason: str
    press_cm: float


# ── 幾何 ───────────────────────────────────────────────────
def lever_angle(depth, s_h):
    """触れた所からの下がり depth と付け根からの横の距離 s_h から、レバーの角度 [rad]（触れた所より上は 0）"""
    return np.arctan2(np.maximum(depth, 0.0), s_h) if isinstance(depth, np.ndarray) else (
        math.atan2(max(float(depth), 0.0), float(s_h)))


def lever_torque(fy, fz, s_h, depth):
    """手が受ける力（y・z）から、手がレバーを下げる向きに掛けている付け根の回り [N·m]"""
    return s_h * fz - depth * fy


def slide_ratio(fy, fz, theta) -> Tuple[float, float]:
    """坂 theta のレバーの上で手が受ける力を、法線と棒に沿う向き（付け根の側・上へ）に分けた比と法線の力"""
    fn = -math.sin(theta) * fy + math.cos(theta) * fz
    ft = math.cos(theta) * fy + math.sin(theta) * fz
    return (ft / fn if fn > 1e-6 else math.nan), fn


def contact_onset(fz: np.ndarray, line: float = CONTACT_LINE_N, hold: int = CONTACT_HOLD) -> Optional[int]:
    """上下の力が line を hold 周期続けて越えた最初の周期（無ければ None）"""
    above = np.asarray(fz) > line
    run = 0
    for i, a in enumerate(above):
        run = run + 1 if a else 0
        if run >= hold:
            return i - hold + 1
    return None


def fit_spring(theta: np.ndarray, tau: np.ndarray) -> Tuple[float, float, float]:
    """τ = τ0 + k·θ の最小二乗。(τ0 [N·m], k [N·m/rad], 決定係数)"""
    th, ta = np.asarray(theta, dtype=float), np.asarray(tau, dtype=float)
    ok = np.isfinite(th) & np.isfinite(ta)
    th, ta = th[ok], ta[ok]
    if th.size < 10 or np.ptp(th) < math.radians(5.0):
        raise ValueError(f"ばねを合わせる点が足りない（{th.size} 点・角度の幅 {math.degrees(np.ptp(th)) if th.size else 0:.1f}°）")
    k, tau0 = np.polyfit(th, ta, 1)
    pred = tau0 + k * th
    ss = float(np.sum((ta - ta.mean()) ** 2))
    return float(tau0), float(k), (1.0 - float(np.sum((ta - pred) ** 2)) / ss) if ss > 0 else math.nan


def band(values: Iterable[float]) -> Tuple[float, float, float]:
    """(下・中央・上) = 10・50・90 パーセンタイル（NaN は除く）"""
    v = np.asarray([x for x in values if x is not None], dtype=float)
    v = v[np.isfinite(v)]
    if not v.size:
        return (math.nan, math.nan, math.nan)
    lo, mid, hi = np.percentile(v, BAND_PCT)
    return float(lo), float(mid), float(hi)


def split_runs(labels: Sequence[int]) -> Tuple[List[int], List[int]]:
    """(値を決める回, 確かめる回)。並べた 3 回に 1 回（3・6・9… 番目）を確かめる側へ"""
    lab = sorted(labels)
    check = lab[2::3]
    return [x for x in lab if x not in check], check


def latch_bounds(runs: Sequence[Tuple[bool, float]]) -> Dict[str, Optional[float]]:
    """(外れたか, 3a のレバーの角度の最大 °) の並びから、掛け金が外れる角度の上限と下限"""
    rel = [a for r, a in runs if r and math.isfinite(a)]
    nrel = [a for r, a in runs if not r and math.isfinite(a)]
    return {"upper_deg": min(rel) if rel else None, "lower_deg": max(nrel) if nrel else None,
            "n_released": len(rel), "n_not_released": len(nrel)}


# ── 1 回ぶんの値 ───────────────────────────────────────────
def _slide_mask(s: np.ndarray) -> np.ndarray:
    """手が棒の先の側へ流れている周期（s が SLIDE_WIN 周期で SLIDE_RATE 以上の速さで増える）"""
    out = np.zeros(s.size, dtype=bool)
    if s.size > SLIDE_WIN:
        rate = (s[SLIDE_WIN:] - s[:-SLIDE_WIN]) / (SLIDE_WIN * DT)
        out[SLIDE_WIN // 2: SLIDE_WIN // 2 + rate.size] = rate > SLIDE_RATE
    return out


def lever_samples(sr: RunSeries) -> Dict[str, np.ndarray]:
    """触れた所から 3a の終わりまでの (θ, τ, s, 深さ, 流れているか, 小段)。触れていなければ空"""
    on_lever = np.isin(sr.sub, (S2C, S2D, S3A))
    idx = np.where(on_lever)[0]
    k0 = contact_onset(sr.force[idx, 2]) if idx.size else None
    empty = {k: np.zeros(0) for k in ("theta", "tau", "s", "depth", "slide", "sub", "fy", "fz", "i")}
    if k0 is None:
        return empty
    idx = idx[k0:]
    z0 = float(sr.tip[idx[0], 2])
    s = sr.y_root - sr.tip[idx, 1]
    depth = z0 - sr.tip[idx, 2]
    th = lever_angle(depth, s)
    fy, fz = sr.force[idx, 1], sr.force[idx, 2]
    return {"theta": th, "tau": lever_torque(fy, fz, s, depth), "s": s, "depth": depth, "slide": _slide_mask(s),
            "sub": sr.sub[idx], "fy": fy, "fz": fz, "i": idx}


def _med(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.median(x)) if x.size else math.nan


def run_summary(sr: RunSeries) -> Dict:  # noqa: C901
    """1 回ぶんの表の行（段 A の表・計画 12 節）"""
    row: Dict = {"label": sr.label, "stop": sr.stop_reason, "press_cm": sr.press_cm}
    for name, si in (("2c", S2C), ("2d", S2D), ("3a", S3A), ("3c", S3C)):
        row[f"dur_{name}"] = float(np.sum(sr.sub == si) * DT)
    smp = lever_samples(sr)
    if not smp["theta"].size:
        row.update(contact=False)
        return row
    in2d, in3a = smp["sub"] == S2D, smp["sub"] == S3A
    row["contact"] = True
    row["onset_before_2d_s"] = float(sr.t[np.where(sr.sub == S2D)[0][0]] - sr.t[smp["i"][0]]) if (sr.sub == S2D).any() else math.nan
    row["s_onset_cm"] = 100 * float(smp["s"][0])
    row["slide_cm"] = 100 * float(np.max(smp["s"][in2d | in3a]) - smp["s"][0]) if (in2d | in3a).any() else 0.0
    last2d = np.where(in2d)[0][-CONTACT_HOLD:]
    row["depth_2d_cm"] = 100 * _med(smp["depth"][last2d]) if last2d.size else math.nan
    row["theta_2d_deg"] = math.degrees(_med(smp["theta"][last2d])) if last2d.size else math.nan
    row["tau_2d_nm"] = _med(smp["tau"][last2d]) if last2d.size else math.nan
    a3 = in3a & (smp["depth"] < FELL_UNDER_M)
    row["theta_3a_deg"] = math.degrees(_med(smp["theta"][a3])) if a3.any() else math.nan
    row["theta_3a_max_deg"] = math.degrees(float(np.max(smp["theta"][a3]))) if a3.any() else math.nan
    row["tau_3a_nm"] = _med(smp["tau"][a3]) if a3.any() else math.nan
    row["fz_3a_n"] = _med(smp["fz"][a3]) if a3.any() else math.nan
    row["depth_max_cm"] = 100 * float(np.max(smp["depth"][in3a])) if in3a.any() else math.nan
    row["fell_under"] = bool(in3a.any() and np.max(smp["depth"][in3a]) > FELL_UNDER_M)
    ratios = [slide_ratio(fy, fz, th) for fy, fz, th in zip(smp["fy"], smp["fz"], smp["theta"])]
    rat = np.array([r for r, fn in ratios])
    fn_ok = np.array([fn > MIN_FZ_N for r, fn in ratios])
    sl = smp["slide"] & fn_ok & (in2d | in3a) & (smp["theta"] > math.radians(5.0))
    st = (~smp["slide"]) & fn_ok & a3
    row["slide_ratio"] = _med(rat[sl]) if sl.sum() >= 3 else math.nan
    row["stick_ratio"] = _med(rat[st]) if st.sum() >= 3 else math.nan
    m3a = sr.sub == S3A
    if m3a.any():
        row["fx_3a_n"] = _med(sr.force[m3a, 0])
        p = sr.pose[m3a]
        row["pelvis_3a_cm"] = 100 * float(p[-1, 0] - p[0, 0])
    m3c = np.where(sr.sub == S3C)[0]
    if m3c.size:
        fx = sr.force[m3c, 0]
        row["fx_3c_n"] = _med(fx)
        row["push_frac_3c"] = float(np.mean(fx < -PUSH_LINE_N))
        row["door_max_3c_deg"] = float(np.nanmax(sr.angle_deg[m3c]))
        k3 = m3c[: int(3.0 / DT)]
        row["door_rate_3s"] = float((sr.angle_deg[k3[-1]] - sr.angle_deg[k3[0]]) / max(DT, sr.t[k3[-1]] - sr.t[k3[0]]))
    return row


def closer_moment(runs: Sequence[RunSeries], opened: Set[int], pushing_only: bool = True) -> Tuple[float, float, float]:
    """扉が開いた回の 3c の最初 OPEN_WINDOW_S 秒の、手が扉を押す回り（−Fx × 蝶番からの距離）の回ごとの中央の幅。
    pushing_only: 押している周期（−Fx > PUSH_LINE_N）だけの中央（11-13: 手は 3〜4 割の時間 扉から離れ、その間は扉の
    油圧の粘りで戻りが遅いので、全部の周期の中央は扉を動かす回りより小さく出る）"""
    per = []
    for sr in runs:
        if sr.label not in opened:
            continue
        m = np.where(sr.sub == S3C)[0]
        m = m[sr.t[m] - sr.t[m[0]] <= OPEN_WINDOW_S] if m.size else m
        push = -sr.force[m, 0]
        if pushing_only:
            push = push[push > PUSH_LINE_N]
        if push.size:
            per.append(_med(push) * sr.r_hinge)
    return band(per)


def fit_all(runs: Sequence[RunSeries], opened: Set[int], released: Set[int], unknown: Set[int] = frozenset(),
            turned: Set[int] = frozenset()) -> Dict:
    """値を決める回だけで 4 節の値を幅付きで出す（確かめる回は表にだけ出す）。
    unknown: 掛け金が外れたか分からない回（掛け金の幅に使わない）。turned: 手の向きを回した回（手の先が棒に触れる点と
    違うので、ばね・滑りの比に使わない）"""
    labels = [r.label for r in runs]
    fit_l, check_l = split_runs(labels)
    rows = {r.label: run_summary(r) for r in runs}
    fit_runs = [r for r in runs if r.label in fit_l]
    springs = []
    for r in fit_runs:
        if r.label in turned:
            continue
        smp = lever_samples(r)
        use = np.isin(smp["sub"], (S2D, S3A)) & (smp["fz"] > MIN_FZ_N) & (smp["depth"] < FELL_UNDER_M) if smp["theta"].size else None
        try:
            tau0, k, r2 = fit_spring(smp["theta"][use], smp["tau"][use]) if use is not None else (math.nan,) * 3
        except ValueError:
            tau0, k, r2 = (math.nan,) * 3
        springs.append({"label": r.label, "tau0": tau0, "k": k, "r2": r2,
                        "tau40": tau0 + k * math.radians(40.0) if math.isfinite(tau0) else math.nan})
        rows[r.label].update(spring_tau0=tau0, spring_k=k, spring_r2=r2)
    fr = [rows[x] for x in fit_l]
    fr_geo = [rows[x] for x in fit_l if x not in turned]
    return {
        "runs_fit": fit_l, "runs_check": check_l,
        "lever": {"tau0_nm": band(s["tau0"] for s in springs), "k_nm_per_rad": band(s["k"] for s in springs),
                  "tau_at_40deg_nm": band(s["tau40"] for s in springs), "per_run": springs},
        "lever_angle_3a_deg": band(r.get("theta_3a_deg", math.nan) for r in fr_geo),
        "tau_hold_3a_nm": band(r.get("tau_3a_nm", math.nan) for r in fr_geo),
        "latch_release_deg": latch_bounds([(x in released, rows[x].get("theta_3a_max_deg", math.nan)) for x in fit_l
                                           if x not in unknown and x not in turned]),
        "friction": {"slide_ratio": band(r.get("slide_ratio", math.nan) for r in fr_geo),
                     "stick_ratio": band(r.get("stick_ratio", math.nan) for r in fr_geo),
                     "stick_ratio_max": max((r.get("stick_ratio", math.nan) for r in fr_geo
                                             if math.isfinite(r.get("stick_ratio", math.nan))), default=math.nan)},
        "closer": {"open_moment_nm": closer_moment(fit_runs, opened & set(fit_l)),
                   "open_moment_all_samples_nm": closer_moment(fit_runs, opened & set(fit_l), pushing_only=False),
                   "runs": sorted(opened & set(fit_l))},
        "excluded": {"latch_unknown": sorted(unknown), "hand_turned": sorted(turned)},
        "rows_fit": fr, "rows_check": [rows[x] for x in check_l],
    }


# ── 記録から取り出す（順運動学・Mac で） ───────────────────
def extract_runs(paths: Sequence[str], pelvis_z_default: float = 0.73) -> List[RunSeries]:
    """記録（door_run_stock.py の npz）から、3a に入った回の系列。回の番号は全部の記録の DOOR の区間を順に数えた番号
    （作業ログ 2026-11-13 の「走らせ役に d を打った順」の 1〜21 と同じ）"""
    import ana_script_run as asr
    import door_frame as dfr
    import g1_arm_fk as fk
    import hand_force as hfm

    chain = fk.load_chain()
    hf = hfm.HandForce(chain, fk.default_urdf(), "left", tip_m=hfm.TIP_M)
    out: List[RunSeries] = []
    label = 0
    for path in sorted(paths):
        z = np.load(path, allow_pickle=True)
        meta = asr.load_meta(z)
        args = meta.get("args", {}) or {}
        w, _, _, bs = (float(v) for v in (args.get("door") or "0.94,2.05,0.99,0.064").split(","))
        pz = float(args.get("pelvis_z") or pelvis_z_default)
        arr = {k: z[k] for k in ("t", "phase", "script", "q", "tau", "quat", "pose")}
        spans = asr.door_spans(arr["phase"], asr.door_index(meta))
        for n, (a0, a1) in enumerate(spans):
            label += 1
            sc = arr["script"][a0:a1]
            sub = sc[:, 0]
            if not (sub == S3A).any():
                continue
            q, tau, quat, pose, t = (arr[k][a0:a1] for k in ("q", "tau", "quat", "pose", "t"))
            hf.reset()
            for i in np.where(sub == S2B)[0][-CALIB_N:]:
                hf.calibrate(q[i], quat[i], tau[i])
            keep = np.where(np.isfinite(sub) & (sub >= S2C) & (sub <= S3C))[0]
            tip, palm, force = (np.full((keep.size, 3), np.nan) for _ in range(3))
            for j, i in enumerate(keep):
                roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat[i])
                R = dfr.rot_z(pose[i, 2]) @ dfr.tilt_matrix(roll, pitch)
                o = np.array([pose[i, 0], pose[i, 1], pz])
                p_hand = chain.link_pose("left_rubber_hand", q[i])[0]
                d = p_hand - chain.link_pose("left_wrist_yaw_link", q[i])[0]
                palm[j] = o + R @ p_hand
                tip[j] = o + R @ (p_hand + hfm.TIP_M * d / (np.linalg.norm(d) + 1e-9))
                hf.filt = None  # なましは後で揃えてかける
                force[j] = R @ hf.force(q[i], quat[i], tau[i])
            force = _smooth(force, 5)
            d2 = np.where(sub[keep] == S2D)[0]
            press = 100 * float(sc[keep[d2[0]], 6] - sc[keep[d2[-1]], 6]) if d2.size else math.nan
            y_root = w / 2 - bs
            reason = str(meta.get("reason", "")) if n + 1 == len(spans) else ""
            out.append(RunSeries(label=label, t=t[keep], sub=sub[keep], tip=tip, palm=palm, force=force, pose=pose[keep],
                                 angle_deg=sc[keep, 7], y_root=y_root, r_hinge=float(np.nanmedian(tip[:, 1])) + w / 2,
                                 stop_reason=reason, press_cm=press))
    return out


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    """周期の向きの移動平均（端は短い窓）"""
    out = np.empty_like(x)
    for i in range(len(x)):
        out[i] = np.nanmean(x[max(0, i - n // 2): i + n // 2 + 1], axis=0)
    return out


# ── 出力 ───────────────────────────────────────────────────
TABLE_COLS = (("label", "回", "{:>3d}"), ("press_cm", "下げ", "{:5.1f}"), ("dur_2d", "2d秒", "{:4.1f}"), ("dur_3a", "3a秒", "{:4.1f}"),
              ("dur_3c", "3c秒", "{:4.1f}"), ("s_onset_cm", "s始", "{:5.1f}"), ("slide_cm", "流れ", "{:5.1f}"),
              ("theta_2d_deg", "θ2d", "{:5.1f}"), ("theta_3a_deg", "θ3a", "{:5.1f}"), ("tau_2d_nm", "τ2d", "{:5.2f}"),
              ("tau_3a_nm", "τ3a", "{:5.2f}"), ("depth_max_cm", "深さ", "{:5.1f}"), ("slide_ratio", "滑比", "{:5.2f}"),
              ("stick_ratio", "止比", "{:5.2f}"), ("fx_3a_n", "Fx3a", "{:+5.1f}"), ("pelvis_3a_cm", "腰3a", "{:+5.1f}"),
              ("fx_3c_n", "Fx3c", "{:+5.1f}"), ("push_frac_3c", "押割", "{:4.2f}"), ("door_rate_3s", "扉/s", "{:+5.1f}"),
              ("spring_tau0", "τ0", "{:5.2f}"), ("spring_k", "k", "{:5.2f}"))


def format_table(rows: Sequence[Dict]) -> str:
    head = " ".join(f"{h:>5s}" for _, h, _ in TABLE_COLS) + "  下"
    lines = [head]
    for r in rows:
        cells = []
        for key, _, fmt in TABLE_COLS:
            v = r.get(key)
            cells.append(f"{'—':>5s}" if v is None or (isinstance(v, float) and not math.isfinite(v)) else fmt.format(v))
        lines.append(" ".join(cells) + ("  ⤓" if r.get("fell_under") else ""))
    return "\n".join(lines)


def _jsonable(o):
    if isinstance(o, dict):
        return {k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else round(float(o), 4)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


# 11-13 朝の作業ログ（2026-11-13_ドア課題_台本の実機21回）の判定。扉が開いた＝3c の最初の 3 秒で +1°/s 以上（1・7・8 回目）。
# 掛け金が外れた＝作業ログの「掛け金を外せた回 11/13」（12・14 回目は脚を止めた 3a で 10 cm 後ずさりして止まった）
OPENED_1113 = {1, 7, 8}
RELEASED_1113 = {1, 3, 7, 8, 9, 11, 13, 15, 16, 19, 21}
# 12・14 回目は脚を止めた 3a で押し戻されて止まった（3a の角度は外れた回より大きい 44〜47°）。15・19・21 回目は足踏みの 3a で
# 手がレバーの下へ落ちた（深さ 17〜18 cm）。「扉 3°」は手が滑った分の読みの見込み（11-13 夕の 3 回目の動画と同じ形）
LATCH_UNKNOWN_1113 = {12, 14, 15, 19, 21}
# 19〜21 回目は半身（手の向き 30°）。手の先の横の位置が、手が棒に触れる点と違う（s が 6〜8 cm と食い違う）
HAND_TURNED_1113 = {19, 20, 21}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--out", default="")
    ap.add_argument("--table", action="store_true")
    a = ap.parse_args(argv)
    paths = sorted(p for g in a.npz for p in glob.glob(g))
    runs = extract_runs(paths)
    res = fit_all(runs, OPENED_1113, RELEASED_1113, LATCH_UNKNOWN_1113, HAND_TURNED_1113)
    if a.table:
        print("[fit] 値を決める回:")
        print(format_table(res["rows_fit"]))
        print("[fit] 確かめる回（値には使わない）:")
        print(format_table(res["rows_check"]))
    lv = res["lever"]
    print(f"[fit] レバーのばね τ0 {lv['tau0_nm'][0]:.2f}・{lv['tau0_nm'][1]:.2f}・{lv['tau0_nm'][2]:.2f} N·m・"
          f"k {lv['k_nm_per_rad'][0]:.2f}・{lv['k_nm_per_rad'][1]:.2f}・{lv['k_nm_per_rad'][2]:.2f} N·m/rad・"
          f"40° で {lv['tau_at_40deg_nm'][1]:.2f} N·m")
    print(f"[fit] 3a のレバーの角度 {'・'.join(f'{v:.1f}' for v in res['lever_angle_3a_deg'])}°・掛け金 {res['latch_release_deg']}")
    print(f"[fit] 滑りの比（流れている間）{'・'.join(f'{v:.2f}' for v in res['friction']['slide_ratio'])}・"
          f"止まっている間 {'・'.join(f'{v:.2f}' for v in res['friction']['stick_ratio'])}")
    print(f"[fit] 3a で押さえている間の回り {'・'.join(f'{v:.2f}' for v in res['tau_hold_3a_nm'])} N·m")
    print(f"[fit] クローザーの開ける向きの回り（押している間）{'・'.join(f'{v:.1f}' for v in res['closer']['open_moment_nm'])} N·m・"
          f"全部の周期 {'・'.join(f'{v:.1f}' for v in res['closer']['open_moment_all_samples_nm'])} N·m（回 {res['closer']['runs']}）")
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(_jsonable(res), f, ensure_ascii=False, indent=1)
        print(f"[fit] 書いた: {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
