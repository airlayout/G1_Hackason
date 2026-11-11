"""11-10 手順 L2b: 手の前進量で出す扉の角度（hand_door_angle.py）を、シムの評価の記録で本当の扉の角度と比べる。numpy だけ（OMEN の CPU）。

記録: state_recorder の records/<名前>/states_*.npz（台の原点からの位置・0.02 s ごと）と static.npz。各台の最初の回だけを使う。
角度の基準は --ref-mode（既定 contact＝手が触れた・press＝0100 と同じ線）。閉じたレバーの当てる点は --lever-ref:
  first（既定）  記録の最初の歩の grasp_target ＋ 回ごとの片寄り（0100 と同じ σ 2 cm の正規）で固定。実機は段 2 の始めに 1 回測る
  closed        本当のレバー・扉の角度で「閉じている」間だけ追う（シムの 0100 と同じ。実機には無い本当の値を使う）
扉の線（蝶番の軸・押す向き）:
  2° 以上開いた台は、扉の板の中心の動きと本当の扉の角度から最小二乗で当てはめる（残差 FIT_MAX_RES_M を越えた台は捨てる）
  開かなかった台は、閉じた板の中心と掛け金の部品から作る名目の線（実機の door_line と同じ形）。誤作動（閉じた扉で 3° を越える）を数える
比べる場合（--variants）:
  ideal     本当の pelvis の姿勢 ∘ 順運動学
  still     pelvis は基準の時の姿勢のまま（順運動学だけ。脚を放した後の踏み出しを見ない）
  door2     段 0 の扉の位置の片寄り（x・y それぞれ σ 2 cm の正規・向き σ 1°。L1 (b) の c2 の外れの大きさ）
  door6     同じく x・y それぞれ σ 6 cm・向き σ 2°（s2f が学んだ扉の値の片寄り）
  fastlio   pelvis の姿勢を 10 回/秒・遅れ 0.1 s で受け取り、ゆっくりずれる（0.5 cm/s・0.2°/s の正規の速さ＋白い 3 mm）。
            順運動学は今の歩（姿勢だけ古い）
  fastlio_sync  fastlio の姿勢に、同じ歩の順運動学を足す（そろっているが 0.1〜0.18 s 遅れる）
  fastlio_odom  fastlio の姿勢から今の歩の 1 歩前までを /dog_odom の動き（距離を 0.93 倍に短く出す）でつなぐ。順運動学は今の歩
  fastlio_odom_door6  fastlio_odom と door6 を両方
乱数は台・場合ごとに別の流れ（default_rng([seed, 台, 場合の番号])）。場合を絞って回し直しても同じ値になる。
出すもの: 本当の角度 2〜4° での外れ・推定が 3° を越えた時の本当の角度・本当の 3° からの遅れ・早すぎ（本当 < 2°）・
基準が決まらないまま本当に 3° 開いた台・手が離れた扱いで本当の 3° の前に止まった台・開かなかった扉での誤作動。

使い方（OMEN）: python l2b_sim_eval_1110.py --records ~/work_inoue/doorman/records/start_1110_1703 \\
    --urdf ~/work_inoue/tmp_fk_1109/g1_29dof_rev_1_0.urdf --out l2b_1703.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

import g1_arm_fk as fk
import hand_door_angle as hda

HINGE, LEVER = 0, 1                     # door_dof_pos の並び（蝶番・レバー・掛け金）
PANEL, LATCH = 1, 3                     # door_body_names の door_panel・latch_link
CLOSED_LEVER_RAD = math.radians(5.0)    # 0100 の「閉じている」
CLOSED_HINGE_RAD = math.radians(1.0)
FIT_MIN_DEG = 2.0                       # 蝶番の軸を当てはめる最小の開き
FIT_MAX_RES_M = 0.002                   # 当てはめの残差の最大がこれを越えた台は捨てる
PRESS_REF_NOISE_M = 0.02                # 0100 の press_ref_noise
TARGET_DEG = 3.0                        # 段 2 の終わり（complete_door_deg）
BAND_DEG = (2.0, 4.0)
EARLY_DEG = 2.0
UNOPENED_MAX_DEG = 0.5                  # これより開かなかった台を「開かなかった扉」とする
FL_PERIOD_S, FL_LATENCY_S = 0.1, 0.1    # FAST-LIO の刻みと遅れ（10 Hz・走査 0.1 s）
FL_DRIFT_MPS, FL_YAW_DRIFT_DPS, FL_WHITE_M = 0.005, 0.2, 0.003
ODOM_SCALE = 0.93                       # /dog_odom は距離を 4〜10% 短く出す（10-21）
VARIANTS = ("ideal", "still", "door2", "door6", "fastlio", "fastlio_sync", "fastlio_odom", "fastlio_odom_door6")
DOOR_BIAS = {"door2": (0.02, 1.0), "door6": (0.06, 2.0), "fastlio_odom_door6": (0.06, 2.0)}
LEVER_REFS = ("first", "closed")


@dataclass(frozen=True)
class Episode:
    t: np.ndarray          # [s]
    hinge: np.ndarray      # 本当の扉の角度 [rad]
    lever: np.ndarray
    root_pos: np.ndarray   # [T, 3]
    root_quat: np.ndarray  # [T, 4] xyzw
    q: np.ndarray          # [T, 29]
    panel: np.ndarray      # [T, 3]
    grasp: np.ndarray      # [T, 3]
    force: np.ndarray      # [T] [N]
    latch: Optional[np.ndarray] = None  # [T, 3] 掛け金の部品（名目の線に使う）


def load_records(path: str) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    files = sorted(glob.glob(os.path.join(path, "states_*.npz")))
    if not files:
        raise SystemExit(f"[l2b] states_*.npz が無い: {path}")
    keys = ("step", "root_pos", "root_quat_xyzw", "dof_pos", "door_dof_pos", "door_body_pos", "grasp_target",
            "hand_lever_force", "episode_length")
    parts = [np.load(f) for f in files]
    data = {k: np.concatenate([p[k] for p in parts]) for k in keys}
    static = dict(np.load(os.path.join(path, "static.npz"), allow_pickle=True))
    names = [str(n) for n in static["dof_names"]]
    if tuple(names) != fk.JOINT_ORDER:
        raise SystemExit("[l2b] 記録の関節の並びが SDK の順（g1_arm_fk.JOINT_ORDER）と違う")
    bodies = [str(n) for n in static["door_body_names"]]
    if bodies[PANEL] != "door_panel" or bodies[LATCH] != "latch_link":
        raise SystemExit(f"[l2b] door_body_names の並びが違う（{PANEL} 番 door_panel・{LATCH} 番 latch_link）: {bodies}")
    return data, static


def first_episode(data: Dict[str, np.ndarray], env: int, dt: float) -> Episode:
    el = data["episode_length"][:, env]
    resets = np.nonzero(el[1:] < el[:-1])[0]
    end = int(resets[0]) + 1 if len(resets) else len(el)
    sl = slice(0, end)
    return Episode(t=np.arange(end) * dt, hinge=data["door_dof_pos"][sl, env, HINGE].astype(float),
                   lever=data["door_dof_pos"][sl, env, LEVER].astype(float),
                   root_pos=data["root_pos"][sl, env].astype(float), root_quat=data["root_quat_xyzw"][sl, env].astype(float),
                   q=data["dof_pos"][sl, env].astype(float), panel=data["door_body_pos"][sl, env, PANEL].astype(float),
                   grasp=data["grasp_target"][sl, env].astype(float),
                   force=np.linalg.norm(data["hand_lever_force"][sl, env].astype(float), axis=-1),
                   latch=data["door_body_pos"][sl, env, LATCH].astype(float))


def rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def fit_hinge(ep: Episode) -> Optional[Tuple[hda.DoorLine, float]]:
    """板の中心の動きと本当の角度から蝶番の軸（床の xy）を出す。(閉じた扉の線, 当てはめの残差の最大 [m])。開きが足りなければ None"""
    opened = np.nonzero(ep.hinge >= math.radians(1.0))[0]
    if np.degrees(ep.hinge.max()) < FIT_MIN_DEG or len(opened) < 3:
        return None
    closed = np.nonzero(ep.hinge[: opened[0]] < math.radians(0.05))[0]
    if len(closed) == 0:
        return None
    c0 = ep.panel[closed[-1], :2]
    best = None
    for sign in (1.0, -1.0):
        A = np.concatenate([np.eye(2) - rot2(sign * ep.hinge[i]) for i in opened])
        b = np.concatenate([ep.panel[i, :2] - rot2(sign * ep.hinge[i]) @ c0 for i in opened])
        H, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = float(np.abs(A @ H - b).max())
        if best is None or res < best[1]:
            best = (H, res, sign)
    H, res, sign = best
    along = (c0 - H) / np.linalg.norm(c0 - H)
    push = rot2(sign * math.pi / 2) @ along
    return hda.DoorLine(H, along, push), res


def nominal_line(ep: Episode, width: float, lever_left: bool) -> hda.DoorLine:
    """実機の door_line と同じ形の名目の線: 閉じた板の中心から掛け金の向きが蝶番 → 戸先、蝶番は中心から幅/2 手前。
    開く向きは把手の側で決まる（把手が左なら右回り＝door_line の open_sign）"""
    c0 = ep.panel[0, :2]
    along = ep.latch[0, :2] - c0
    along = along / np.linalg.norm(along)
    sign = hda.door_line(width, lever_left).open_sign
    return hda.DoorLine(c0 - width / 2 * along, along, rot2(sign * math.pi / 2) @ along)


def perturb_line(line: hda.DoorLine, center: np.ndarray, pos_sigma: float, yaw_sigma_deg: float,
                 rng: np.random.Generator) -> hda.DoorLine:
    """段 0 の扉の位置の片寄り（x・y それぞれ σ pos_sigma・向き σ yaw_sigma_deg）: 開口の中心のまわりに回し、ずらした線"""
    R = rot2(math.radians(yaw_sigma_deg) * rng.standard_normal())
    shift = pos_sigma * rng.standard_normal(2)
    return hda.DoorLine(center + R @ (line.hinge_xy - center) + shift, R @ line.along, R @ line.push)


def palms(chain: fk.ArmFK, ep: Episode) -> np.ndarray:
    return np.array([hda.palm_world(chain, ep.q[i], ep.root_pos[i], ep.root_quat[i], "left") for i in range(len(ep.t))])


def palms_pelvis(chain: fk.ArmFK, ep: Episode) -> np.ndarray:
    return np.array([fk.palm_in_pelvis(chain, ep.q[i], "left") for i in range(len(ep.t))])


def compose_palms(ep: Episode, local: np.ndarray, pos: np.ndarray, rot_idx: np.ndarray, local_idx: np.ndarray,
                  yaw_err: np.ndarray) -> np.ndarray:
    """手のひら = 受け取った pelvis の位置 pos ＋ 向き（rot_idx の歩の本当の向きに yaw_err を足す）∘ local_idx の歩の順運動学"""
    out = np.empty((len(ep.t), 3))
    for i in range(len(ep.t)):
        Rz = np.eye(3)
        Rz[:2, :2] = rot2(yaw_err[i])
        out[i] = pos[i] + Rz @ (hda._quat_xyzw_to_mat(ep.root_quat[rot_idx[i]]) @ local[local_idx[i]])  # noqa: SLF001
    return out


def fastlio_pose(ep: Episode, t0: int, rng: np.random.Generator) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """FAST-LIO で受け取る pelvis の姿勢: 0.1 s ごとに、0.1 s 前の値。基準の歩からゆっくりずれる"""
    dt = float(ep.t[1] - ep.t[0])
    n = len(ep.t)
    lag = int(round(FL_LATENCY_S / dt))
    period = int(round(FL_PERIOD_S / dt))
    idx = np.array([max(0, (i // period) * period - lag) for i in range(n)])
    v = FL_DRIFT_MPS * rng.standard_normal(3)
    v[2] = 0.0
    w = math.radians(FL_YAW_DRIFT_DPS) * rng.standard_normal()
    since = np.clip(ep.t[idx] - ep.t[t0], 0.0, None)
    white = FL_WHITE_M * rng.standard_normal((n // period + 1, 3))[np.arange(n) // period]
    white[:, 2] = 0.0
    return idx, since[:, None] * v + white, since * w


def lever_refs(ep: Episode, ref_bias: np.ndarray, mode: str) -> List[Optional[np.ndarray]]:
    """各歩で set_lever_ref に渡す点（None は渡さない）"""
    if mode == "first":
        return [ep.grasp[0] + ref_bias] + [None] * (len(ep.t) - 1)
    closed = (np.abs(ep.lever) < CLOSED_LEVER_RAD) & (np.abs(ep.hinge) < CLOSED_HINGE_RAD)
    return [ep.grasp[i] + ref_bias if closed[i] else None for i in range(len(ep.t))]


def run_estimator(line: hda.DoorLine, ep: Episode, palm: np.ndarray, refs: List[Optional[np.ndarray]], method: str,
                  ref_mode: str = "contact", lost_s: Optional[float] = None) -> Tuple[Optional[int], np.ndarray, Optional[int]]:
    """(基準が決まった歩, 推定の角度 [rad]・基準の前は 0, 手が離れた扱いになった歩)"""
    est = hda.HandDoorAngle(line, method=method, ref_mode=ref_mode, lost_s=lost_s)
    out = np.zeros(len(ep.t))
    fired_at = lost_at = None
    for i in range(len(ep.t)):
        if refs[i] is not None:
            est.set_lever_ref(refs[i])
        out[i] = est.step(float(ep.t[i]), palm[i], float(ep.force[i]))[0]
        if fired_at is None and est.released:
            fired_at = i
        if lost_at is None and est.lost:
            lost_at = i
    return fired_at, out, lost_at


def score(ep: Episode, fired_at: Optional[int], est: np.ndarray, lost_at: Optional[int] = None) -> dict:
    true_deg, est_deg = np.degrees(ep.hinge), np.degrees(est)
    true3 = np.nonzero(true_deg >= TARGET_DEG)[0]
    r: dict = {"fired": fired_at is not None, "opened3": bool(len(true3)),
               "unopened": bool(true_deg.max() < UNOPENED_MAX_DEG)}
    if fired_at is None:
        return r
    r["fire_lever_deg"] = float(np.degrees(ep.lever[fired_at]))
    r["fire_hinge_deg"] = float(true_deg[fired_at])
    r["lost_before_true3"] = lost_at is not None and (not len(true3) or lost_at < true3[0])
    band = (true_deg >= BAND_DEG[0]) & (true_deg <= BAND_DEG[1])
    band[:fired_at] = False
    r["band_err_deg"] = (est_deg[band] - true_deg[band]).tolist()
    cross = np.nonzero(est_deg >= TARGET_DEG)[0]
    if len(cross):
        r["true_at_est3"] = float(true_deg[cross[0]])
    if len(true3):
        r["est_at_true3"] = float(est_deg[true3[0]])
        if len(cross):
            r["delay_s"] = float(ep.t[cross[0]] - ep.t[true3[0]])
    return r


def _pct(a: np.ndarray, ps=(10, 50, 90)) -> dict:
    return {f"p{p}": round(float(np.percentile(a, p)), 2) for p in ps} if len(a) else {}


def summarize(rows: List[dict]) -> dict:
    """開いた台（当てはめた線）と、開かなかった台（名目の線・誤作動だけ）を分けて数える"""
    opened = [r for r in rows if not r["unopened"]]
    shut = [r for r in rows if r["unopened"]]
    fired = [r for r in opened if r["fired"]]
    band = np.concatenate([np.asarray(r["band_err_deg"]) for r in fired if r.get("band_err_deg")] or [np.zeros(0)])
    t_at = np.array([r["true_at_est3"] for r in fired if "true_at_est3" in r])
    e_at = np.array([r["est_at_true3"] for r in fired if "est_at_true3" in r])
    delay = np.array([r["delay_s"] for r in fired if "delay_s" in r])
    return {
        "episodes": len(opened), "fired": len(fired), "opened3": sum(r["opened3"] for r in opened),
        "opened3_not_fired": sum(r["opened3"] and not r["fired"] for r in opened),
        "lost_before_true3": sum(bool(r.get("lost_before_true3")) and r["opened3"] for r in fired),
        "fired_before_lever10": sum(r.get("fire_lever_deg", 99) < 10 for r in fired),
        "band_abs_err_deg": _pct(np.abs(band)) | ({"max": round(float(np.abs(band).max()), 2)} if len(band) else {}),
        "band_err_deg_median": round(float(np.median(band)), 2) if len(band) else None,
        "true_at_est3": _pct(t_at) | ({"min": round(float(t_at.min()), 2)} if len(t_at) else {}),
        "early_true_lt2": int((t_at < EARLY_DEG).sum()),
        "est_at_true3": _pct(e_at),
        "never_est3_but_true3": sum(("est_at_true3" in r) and ("true_at_est3" not in r) for r in fired),
        "delay_s": _pct(delay),
        "unopened": len(shut), "unopened_fired": sum(r["fired"] for r in shut),
        "unopened_false3": sum("true_at_est3" in r for r in shut),
    }


def eval_env(chain: fk.ArmFK, ep: Episode, line: hda.DoorLine, center: np.ndarray, seed: int, env: int,
             variants: List[str], methods: List[str], ref_mode: str = "contact", lever_ref: str = "first") -> Dict[str, dict]:
    """1 台の全部の場合。乱数は [seed, 台, 場合の番号] ごとに別の流れ"""
    base_rng = np.random.default_rng([seed, env, 0])
    refs = lever_refs(ep, PRESS_REF_NOISE_M * base_rng.standard_normal(3), lever_ref)
    world = palms(chain, ep)
    local = palms_pelvis(chain, ep)
    base_fire, _, _ = run_estimator(line, ep, world, refs, "hinge", ref_mode)
    n = len(ep.t)
    now = np.arange(n)
    out: Dict[str, dict] = {}
    for v in variants:
        rng = np.random.default_rng([seed, env, 1 + VARIANTS.index(v)])
        palm, use_line = world, line
        if v == "still" and base_fire is not None:
            idx = now.copy()
            idx[base_fire:] = base_fire
            palm = compose_palms(ep, local, ep.root_pos[idx], idx, now, np.zeros(n))
        if v.startswith("fastlio"):
            idx, perr, yerr = fastlio_pose(ep, base_fire if base_fire is not None else 0, rng)
            pos = ep.root_pos[idx] + perr
            if v == "fastlio":
                palm = compose_palms(ep, local, pos, idx, now, yerr)
            elif v == "fastlio_sync":
                palm = compose_palms(ep, local, pos, idx, idx, yerr)
            else:  # odom で今の 1 歩前までつなぐ（向きは odom の向き＝本当の向き）
                prev = np.maximum(now - 1, 0)
                step = ep.root_pos[prev] - ep.root_pos[idx]
                step[:, :2] *= ODOM_SCALE
                palm = compose_palms(ep, local, pos + step, prev, now, yerr)
        if v in DOOR_BIAS:
            use_line = perturb_line(line, center, DOOR_BIAS[v][0], DOOR_BIAS[v][1], rng)
        for m in methods:
            fired_at, est, lost_at = run_estimator(use_line, ep, palm, refs, m, ref_mode)
            out[f"{v}/{m}"] = score(ep, fired_at, est, lost_at)
    return out


def line_check(fit: hda.DoorLine, nom: hda.DoorLine) -> dict:
    """当てはめた線と名目の線の食い違い（実機の door_line の形がシムの扉と合うかの確かめ）"""
    return {"hinge_m": float(np.linalg.norm(fit.hinge_xy - nom.hinge_xy)),
            "along_deg": math.degrees(math.acos(max(-1.0, min(1.0, float(fit.along @ nom.along))))),
            "sign_ok": fit.open_sign == nom.open_sign}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--records", required=True)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=VARIANTS)
    ap.add_argument("--methods", nargs="+", default=list(hda.METHODS), choices=hda.METHODS)
    ap.add_argument("--envs", type=int, default=0, help="先頭から何台（0 = 全部）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ref-mode", default="contact", choices=hda.REF_MODES, help="角度の基準（hand_door_angle の ref_mode）")
    ap.add_argument("--lever-ref", default="first", choices=LEVER_REFS, help="閉じたレバーの当てる点の渡し方")
    a = ap.parse_args(argv)
    t_start = time.time()
    data, static = load_records(a.records)
    dt = float(static["dt"])
    chain = fk.load_chain(a.urdf)
    n_env = data["root_pos"].shape[1] if a.envs <= 0 else min(a.envs, data["root_pos"].shape[1])
    per: Dict[str, List[dict]] = {}
    checks, counts = [], {"fitted": 0, "bad_fit": 0, "partly_opened": 0, "unopened": 0}
    for e in range(n_env):
        ep = first_episode(data, e, dt)
        lever_left = float(static["door_open_lr"][e]) > 0
        nom = nominal_line(ep, float(static["door_width"][e]), lever_left)
        fit = fit_hinge(ep)
        if fit is not None and fit[1] > FIT_MAX_RES_M:
            counts["bad_fit"] += 1
            continue
        if fit is not None:
            counts["fitted"] += 1
            line = fit[0]
            checks.append(line_check(line, nom))
        elif np.degrees(ep.hinge.max()) < UNOPENED_MAX_DEG:
            counts["unopened"] += 1
            line = nom
        else:
            counts["partly_opened"] += 1  # 0.5〜2°: 線は当てはめられず、誤作動の枠にも入れない
            continue
        for k, r in eval_env(chain, ep, line, ep.panel[0, :2], a.seed, e, a.variants, a.methods, a.ref_mode, a.lever_ref).items():
            per.setdefault(k, []).append(r | {"env": e})
    check = {"hinge_m": _pct(np.array([c["hinge_m"] for c in checks]), (50, 90, 100)),
             "along_deg": _pct(np.array([c["along_deg"] for c in checks]), (50, 90, 100)),
             "sign_ok": f"{sum(c['sign_ok'] for c in checks)}/{len(checks)}"}
    result = {"records": a.records, "seed": a.seed, "ref_mode": a.ref_mode, "lever_ref": a.lever_ref, "counts": counts,
              "nominal_vs_fit": check, "summary": {k: summarize(v) for k, v in per.items()},
              "seconds": round(time.time() - t_start, 1)}
    with open(a.out, "w") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print(f"[l2b] 台の内訳 {counts}・名目の線と当てはめの差 {json.dumps(check, ensure_ascii=False)}")
    for k, s in result["summary"].items():
        print(f"[l2b] {k}: 立った {s['fired']}/{s['episodes']}・3°開いたのに立たない {s['opened3_not_fired']}・離れた扱いで止まった "
              f"{s['lost_before_true3']}・2〜4° の外れ 中央 {s['band_err_deg_median']}° |中央| {s['band_abs_err_deg'].get('p50')}° "
              f"p90 {s['band_abs_err_deg'].get('p90')}°・推定 3° の時の本当 {s['true_at_est3']}・早すぎ {s['early_true_lt2']}・"
              f"遅れ {s['delay_s']}・開かなかった扉 {s['unopened']} 台で立った {s['unopened_fired']}・3° を越えた {s['unopened_false3']}")
    print(f"[l2b] 書いた: {a.out}（{result['seconds']} 秒）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
