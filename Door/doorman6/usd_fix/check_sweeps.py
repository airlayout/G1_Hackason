"""確かめ ③④: 関節を 1 つずつ動かし、当たりの形が親子でない剛体に初めて触れる角度を求める（読むだけ・Mac）。

③ 肩 roll: 腕を外へ 0.6 rad 上げた所から内側へ動かし、腕（肩 roll より先の剛体）の当たりの形が torso_link の
   当たりの形に初めて触れる角度。ほかの関節は (a) 既定の姿勢 (b) 10-23 の実機で左腕が胴に押し付けられて止まったコマの値
   (c) 正解の記録で肩 roll が最も内側の 1% のコマの値（--golden）。(b)(c) は記録の肩 roll と比べる。
   参考に、胴を見た目のメッシュの点（凹みを残す）にした場合も求める（胴の当たりの形＝1 つの凸包が凹みを埋める影響を分ける）
④ 股 roll・足首 pitch・腰 roll/pitch: 既定の姿勢から可動域の両端まで動かし、初めて触れる角度と相手。
   足首 pitch の形と床: 足の当たりの形の最も低い点を床として、足首 pitch の形の最も低い点までの高さ（足首 roll と足の傾きごと）

触れる＝符号つき距離が 0 以下（PhysX の rest offset 0）。刻み 0.005 rad で探し、二分法で 1e-4 rad まで詰める。
使い方: uvx --with usd-core --with numpy --with scipy --with python-fcl python check_sweeps.py [--golden golden_45150.npz]
"""
from __future__ import annotations

import argparse
import json
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import g1_geom as gg

HERE = os.path.dirname(os.path.abspath(__file__))
FIXED = os.path.join(HERE, "fixed", "g1_29dof_rev_1_0_col", "g1_29dof_rev_1_0_col.usd")
ORIG = os.path.join(HERE, "orig", "g1_29dof_rev_1_0", "g1_29dof_rev_1_0.usd")
REAL_LOG = os.path.join(HERE, "..", "..", "real", "logs_real", "door_real_20261023_174544.npz")
STEP, TOL = 0.005, 1e-4


def downstream(robot: gg.Robot, joint: str) -> List[str]:
    """関節より先の剛体"""
    j = next(x for x in robot.joints if x.name == joint)
    out, stack = [], [j.b1]
    while stack:
        b = stack.pop()
        out.append(b)
        stack += [c for c, p in robot.parent.items() if p == b]
    return out


def watch_pairs(robot: gg.Robot, moving: Sequence[str], others: Optional[Sequence[str]] = None) -> List[Tuple[int, int]]:
    """動く剛体の形と、動かない剛体（others を渡せばその中だけ）の形の、親子でない組"""
    C = robot.colliders
    mv = set(moving)
    return [(i, j) for i, j in robot.collider_pairs()
            if (C[i].link in mv) != (C[j].link in mv)
            and (others is None or (C[i].link if C[j].link in mv else C[j].link) in others)]


def min_dist(robot: gg.Robot, dist: gg.Distance, q: np.ndarray, pairs) -> Tuple[float, Tuple[int, int]]:
    res = dist.scan(gg.fk(robot, q), pairs=pairs, within=0.02)
    if not res:
        return 1.0, (-1, -1)
    k = min(res, key=res.get)
    return res[k], k


def first_contact(robot: gg.Robot, dist: gg.Distance, q0: np.ndarray, jname: str, a0: float, a1: float,
                  pairs) -> Optional[dict]:
    """関節 jname を a0 から a1 へ動かし、pairs のどれかが初めて触れる角度。始めから触れていれば start_touch"""
    k = gg.DOF.index(jname)
    q = q0.copy()
    q[k] = a0
    d0, p0 = min_dist(robot, dist, q, pairs)
    if d0 <= 0:
        return {"angle": a0, "start_touch": True, "depth_mm": -1000 * d0, "pair": _pl(robot, p0)}
    n = max(1, int(math.ceil(abs(a1 - a0) / STEP)))
    prev = a0
    for a in np.linspace(a0, a1, n + 1)[1:]:
        q[k] = a
        d, _ = min_dist(robot, dist, q, pairs)
        if d <= 0:
            lo, hi = prev, a
            while abs(hi - lo) > TOL:
                mid = (lo + hi) / 2
                q[k] = mid
                if min_dist(robot, dist, q, pairs)[0] <= 0:
                    hi = mid
                else:
                    lo = mid
            q[k] = hi
            dd, pp = min_dist(robot, dist, q, pairs)
            return {"angle": float(hi), "start_touch": False, "pair": _pl(robot, pp), "start_clear_mm": 1000 * d0}
        prev = a
    return None


def _pl(robot: gg.Robot, p: Tuple[int, int]) -> str:
    C = robot.colliders
    return f"{C[p[0]].label} ↔ {C[p[1]].label}" if p[0] >= 0 else "-"


def fmt(r: Optional[dict]) -> str:
    if r is None:
        return "可動域の中で触れない"
    if r["start_touch"]:
        return f"始めから {r['depth_mm']:.1f} mm 食い込み（{r['pair']}）"
    return f"{r['angle']:+.4f} rad（{math.degrees(r['angle']):+.2f}°）{r['pair']}"


# ---------- ③ 肩 roll ----------
EXPECT = (0.05, 0.10)  # 期待（左の符号。右は符号を反転）。±0.05 rad 以内で合えば合格
RAISE = 0.6  # 腕を外へ上げる量 [rad]


def roll_contact(robot: gg.Robot, dist: gg.Distance, q: np.ndarray, side: str,
                 others: Optional[Sequence[str]] = ("torso_link",)) -> Optional[dict]:
    """姿勢 q の肩 roll を外へ RAISE だけ上げた所から内側の端へ動かし、腕の形が others（None なら親子でない全部）に初めて触れる角度"""
    jn = f"{side}_shoulder_roll_joint"
    k = gg.DOF.index(jn)
    lo, hi = robot.limits()
    sgn = 1.0 if side == "left" else -1.0  # 外へ＝左は +・右は −
    arm = downstream(robot, jn)
    return first_contact(robot, dist, q, jn, q[k] + sgn * RAISE, lo[k] if side == "left" else hi[k],
                         watch_pairs(robot, arm, others))


def blocked_frames(log: str) -> Tuple[np.ndarray, np.ndarray]:
    """左の肩 roll が胴へ押し付けられて止まったコマ（内側へ 5 N·m 超・目標が 0.05 rad 以上内側・速さ 0.3 rad/s 未満）"""
    z = np.load(log, allow_pickle=False)
    q, dq, tau, qs, t = z["q"], z["dq"], z["tau"], z["q_sent"], z["t"] - z["t"][0]
    j = gg.DOF.index("left_shoulder_roll_joint")
    m = (-tau[:, j] > 5) & ((q[:, j] - qs[:, j]) > 0.05) & (np.abs(dq[:, j]) < 0.3)
    return q[m], np.c_[t[m], tau[m, j]]


def golden_deep_frames(path: str, side: str, n_max: int = 30) -> np.ndarray:
    """正解の記録（各台の最初の dones=1 まで）で、肩 roll が最も内側の 1% のコマ（n_max 個まで等間隔に間引く）"""
    z = np.load(path, allow_pickle=False)
    Q, dn = z["dof_pos"].astype(float), z["dones"]
    qs = []
    for e in range(Q.shape[1]):
        hit = np.where(dn[:, e] > 0.5)[0]
        qs.append(Q[: (int(hit[0]) if len(hit) else Q.shape[0] - 1) + 1, e])
    A = np.vstack(qs)
    x = A[:, gg.DOF.index(f"{side}_shoulder_roll_joint")]
    sel = A[x <= np.percentile(x, 1)] if side == "left" else A[x >= np.percentile(x, 99)]
    return sel[np.linspace(0, len(sel) - 1, min(n_max, len(sel))).astype(int)]


def compare_set(robot: gg.Robot, dist: gg.Distance, Q: np.ndarray, side: str) -> List[dict]:
    k = gg.DOF.index(f"{side}_shoulder_roll_joint")
    rows = []
    for q in Q:
        r = roll_contact(robot, dist, q, side)
        ra = roll_contact(robot, dist, q, side, None)
        rows.append({"q_roll": float(q[k]), "contact_torso": None if r is None else r["angle"],
                     "pair_torso": None if r is None else r["pair"],
                     "contact_any": None if ra is None else ra["angle"], "pair_any": None if ra is None else ra["pair"]})
    return rows


def print_set(name: str, side: str, rows: List[dict], recorded: bool) -> None:
    qr = np.array([r["q_roll"] for r in rows])
    ct = np.array([np.nan if r["contact_torso"] is None else r["contact_torso"] for r in rows])
    ca = np.array([np.nan if r["contact_any"] is None else r["contact_any"] for r in rows])
    sgn = 1.0 if side == "left" else -1.0
    lo_e, hi_e = sorted((sgn * EXPECT[0], sgn * EXPECT[1]))
    ok = np.mean((ct >= lo_e - 0.05) & (ct <= hi_e + 0.05))
    print(f"   {name} {side}（{len(rows)} 姿勢）: 胴に触れる角度 {np.nanmin(ct):+.3f}〜{np.nanmax(ct):+.3f}（中央 {np.nanmedian(ct):+.3f}）・"
          f"期待 {lo_e:+.2f}〜{hi_e:+.2f} の ±0.05 に入る割合 {100 * ok:.0f}%")
    if recorded:
        print(f"      記録の肩 roll {np.nanmin(qr):+.3f}〜{np.nanmax(qr):+.3f}（中央 {np.nanmedian(qr):+.3f}）・"
              f"記録 − 触れる角度 の中央 {np.nanmedian(qr - ct):+.3f}・|差| の最大 {np.nanmax(np.abs(qr - ct)):.3f}")
    if np.nanmedian(np.abs(ca - ct)) > 1e-4 or np.isnan(ct).any():
        print(f"      胴以外も含め何かに触れる角度 中央 {np.nanmedian(ca):+.3f}")
    pc: Dict[str, int] = {}
    for r in rows:
        pc[r["pair_torso"]] = pc.get(r["pair_torso"], 0) + 1
    for p, n in sorted(pc.items(), key=lambda x: -x[1]):
        print(f"      最初に触れる組 {n:3d}: {p}")


# ---------- ③ の参考: 胴を見た目の点（凹みを残す）にした場合 ----------

def torso_visual_points(usd: str, voxel: float = 0.002) -> np.ndarray:
    """torso_link の見た目のメッシュの点（torso_link の座標・voxel [m] の格子で間引く）"""
    from pxr import Sdf
    base = Sdf.Layer.FindOrOpen(os.path.join(os.path.dirname(usd), "configuration", "g1_29dof_rev_1_0_base.usd"))
    P = np.array(base.GetPrimAtPath("/meshes/torso_link_rev_1_0/mesh").attributes["points"].default, dtype=float)
    _, keep = np.unique(np.floor(P / voxel).astype(np.int64), axis=0, return_index=True)
    return P[np.sort(keep)]


class VisualTorso:
    """胴の見た目の点が、腕の当たりの形（凸包・円柱）に入る最大の深さ [m]（正＝入っている）"""

    def __init__(self, robot: gg.Robot, side: str, TP: np.ndarray):
        from scipy.spatial import ConvexHull
        self.robot, self.TP = robot, TP
        arm = set(downstream(robot, f"{side}_shoulder_roll_joint"))
        self.shapes = [(c, ConvexHull(c.V).equations if c.kind == "convex" else None)
                       for c in robot.colliders if c.link in arm and c.kind in ("convex", "cylinder")]

    def depth(self, q: np.ndarray) -> Tuple[float, str]:
        W = gg.fk(self.robot, q)
        best = (-1.0, "-")
        for c, eq in self.shapes:
            M = np.linalg.inv(W[c.link] @ c.T) @ W["torso_link"]
            P = self.TP @ M[:3, :3].T + M[:3, 3]
            ctr = c.center if c.kind == "convex" else np.zeros(3)
            P = P[np.linalg.norm(P - ctr, axis=1) <= c.radius + 1e-9]
            if len(P) == 0:
                continue
            if eq is not None:
                d = -(P @ eq[:, :3].T + eq[:, 3]).max(axis=1)
            else:
                r, h = c.size
                d = np.minimum(r - np.hypot(P[:, 0], P[:, 1]), h / 2 - np.abs(P[:, 2]))
            if d.max() > best[0]:
                best = (float(d.max()), c.label)
        return best

    def contact(self, q0: np.ndarray, jname: str, a0: float, a1: float) -> Optional[Tuple[float, str]]:
        k = gg.DOF.index(jname)
        q = q0.copy()
        prev = a0
        for a in np.linspace(a0, a1, max(1, int(math.ceil(abs(a1 - a0) / STEP))) + 1)[1:]:
            q[k] = a
            if self.depth(q)[0] > 0:
                lo, hi = prev, a
                while abs(hi - lo) > TOL:
                    q[k] = (lo + hi) / 2
                    if self.depth(q)[0] > 0:
                        hi = q[k]
                    else:
                        lo = q[k]
                q[k] = hi
                return float(hi), self.depth(q)[1]
            prev = a
        return None


# ---------- ④ 股 roll・足首 pitch・腰 ----------

def joint_sweeps(robot: gg.Robot, dist: gg.Distance, q0: np.ndarray, joints: Sequence[str]) -> Dict[str, dict]:
    lo, hi = robot.limits()
    out = {}
    for jn in joints:
        k = gg.DOF.index(jn)
        pairs = watch_pairs(robot, downstream(robot, jn))
        out[jn] = {"lower": float(lo[k]), "upper": float(hi[k]), "start": float(q0[k]),
                   "to_upper": first_contact(robot, dist, q0, jn, q0[k], hi[k], pairs),
                   "to_lower": first_contact(robot, dist, q0, jn, q0[k], lo[k], pairs)}
    return out


def _lowest(c: gg.Collider, M: np.ndarray) -> float:
    """形 c を座標 M（剛体→床の座標）に置いたときの最も低い z"""
    T = M @ c.T
    R, t = T[:3, :3], T[:3, 3]
    if c.kind == "convex":
        return float((c.V @ M[:3, :3].T + M[:3, 3])[:, 2].min())
    if c.kind == "sphere":
        return float(t[2] - c.size[0])
    if c.kind == "box":
        h = np.array(c.size) / 2
        return float(t[2] - np.abs(R[2]) @ h)
    if c.kind == "cylinder":
        r, hh = c.size
        a = R[:, 2]
        return float(t[2] - abs(a[2]) * hh / 2 - r * math.sqrt(max(0.0, 1 - a[2] ** 2)))
    raise ValueError(c.kind)


def ankle_floor(robot: gg.Robot, side: str) -> dict:
    """足の当たりの形の最も低い点を床（z=0）として、足首 pitch の形の最も低い点の高さ [mm]。
    足首 roll（−15〜+15°）と、床に対する足の傾き（pitch −60〜+60°・roll −30〜+30°）ごと"""
    roll_j = next(j for j in robot.joints if j.name == f"{side}_ankle_roll_joint")
    foot = [c for c in robot.colliders if c.link == f"{side}_ankle_roll_link"]
    ap = [c for c in robot.colliders if c.link == f"{side}_ankle_pitch_link"]
    if not ap:
        return {"note": "足首 pitch に当たりの形が無い"}

    def rot(ax, a):
        return gg._rot(ax, a)

    grid = []
    for r in np.radians(np.arange(-15, 15.01, 2.5)):
        foot_in_ap = roll_j.A @ gg._rot(roll_j.axis, r) @ roll_j.B_inv  # 足首 pitch の座標から見た足
        ap_in_foot = np.linalg.inv(foot_in_ap)
        for tp in np.radians(np.arange(-60, 60.01, 2.5)):
            for tr in np.radians(np.arange(-30, 30.01, 5)):
                Rt = rot("Y", tp) @ rot("X", tr)  # 足を床に対して傾ける
                fl = min(_lowest(c, Rt) for c in foot)
                al = min(_lowest(c, Rt @ ap_in_foot) for c in ap)
                grid.append((math.degrees(r), math.degrees(tp), math.degrees(tr), 1000 * (al - fl)))
    G = np.array(grid)
    flat = G[(np.abs(G[:, 1]) < 1e-6) & (np.abs(G[:, 2]) < 1e-6)]
    worst = G[np.argmin(G[:, 3])]
    touching = G[G[:, 3] <= 0]
    return {"flat_min_mm": float(flat[:, 3].min()), "flat_max_mm": float(flat[:, 3].max()),
            "worst": {"ankle_roll_deg": worst[0], "foot_pitch_deg": worst[1], "foot_roll_deg": worst[2],
                      "clear_mm": worst[3]},
            "n_touch": int(len(touching)), "n_grid": int(len(G)),
            "touch_min_abs_pitch_deg": None if len(touching) == 0 else float(np.abs(touching[:, 1]).min())}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", default=FIXED)
    ap.add_argument("--orig", default=ORIG)
    ap.add_argument("--real-log", default=REAL_LOG)
    ap.add_argument("--golden", default=None, help="正解の記録（肩 roll が最も内側のコマでも③を求める）")
    ap.add_argument("--out", default=os.path.join(HERE, "check_sweeps.json"))
    a = ap.parse_args()
    q0 = gg.default_q()
    Qr, info = blocked_frames(a.real_log)
    sets = [("既定の姿勢", side, q0[None], False) for side in ("left", "right")]
    sets.append((f"10-23 の実機で左腕が胴に押し付けられて止まったコマ（t={info[:, 0].min():.1f}〜{info[:, 0].max():.1f} s・"
                 f"トルク最大 {info[:, 1].min():.1f} N·m）", "left", Qr, True))
    if a.golden:
        sets += [("正解の記録で肩 roll が最も内側の 1%", side, golden_deep_frames(a.golden, side), True)
                 for side in ("left", "right")]
    result = {}
    for tag, usd in (("fixed", a.usd), ("orig", a.orig)):
        for hull in ("true", "physx"):
            rb = gg.load_robot(usd, hull=hull)
            dist = gg.Distance(rb)
            key = f"{tag}/{hull}"
            print(f"\n######## {key}: {usd}（既存のメッシュの凸包: {'PhysX 風の外側の近似（上限の見積り）' if hull == 'physx' else '真の凸包'}）")
            res: Dict[str, object] = {}
            print(f"[③] 肩 roll を外へ {RAISE} rad 上げた所から内へ。腕（肩 roll より先）の形が torso_link の形に初めて触れる角度")
            for name, side, Q, rec in sets:
                rows = compare_set(rb, dist, Q, side)
                res[f"shoulder/{name}/{side}"] = rows
                print_set(name, side, rows, rec)
            if tag == "fixed" and hull == "true":
                TP = torso_visual_points(usd)
                print(f"[③ 参考] 胴を見た目の点（{len(TP)} 点・凹みを残す）、腕を当たりの形にした場合の当たり始め")
                for name, side, Q, rec in sets:
                    vt = VisualTorso(rb, side, TP)
                    k = gg.DOF.index(f"{side}_shoulder_roll_joint")
                    lo, hi = rb.limits()
                    sgn = 1.0 if side == "left" else -1.0
                    cv = np.array([np.nan if (r := vt.contact(q, f"{side}_shoulder_roll_joint", q[k] + sgn * RAISE,
                                                                 lo[k] if side == "left" else hi[k])) is None else r[0]
                                   for q in Q])
                    qr = Q[:, k]
                    res[f"visual_torso/{name}/{side}"] = cv.tolist()
                    msg = f"   {name} {side}: 触れる角度 {np.nanmin(cv):+.3f}〜{np.nanmax(cv):+.3f}（中央 {np.nanmedian(cv):+.3f}）"
                    if rec:
                        msg += f"・記録 − 触れる角度 の中央 {np.nanmedian(qr - cv):+.3f}・|差| の最大 {np.nanmax(np.abs(qr - cv)):.3f}"
                    print(msg)
            js = joint_sweeps(rb, dist, q0, ["left_hip_roll_joint", "right_hip_roll_joint",
                                             "left_ankle_pitch_joint", "right_ankle_pitch_joint",
                                             "waist_roll_joint", "waist_pitch_joint", "waist_yaw_joint"])
            res["joint_sweeps"] = js
            print("[④] 既定の姿勢から 1 関節ずつ可動域の端まで（相手は親子でない全部）")
            for jn, r in js.items():
                print(f"   {jn:24s} 可動域 {r['lower']:+.3f}〜{r['upper']:+.3f}")
                print(f"   {'':24s}  上へ: {fmt(r['to_upper'])}")
                print(f"   {'':24s}  下へ: {fmt(r['to_lower'])}")
            af = {s: ankle_floor(rb, s) for s in ("left", "right")}
            res["ankle_floor"] = af
            print("[④] 足首 pitch の形と床（足の当たりの形の最も低い点＝床）")
            for s, r in af.items():
                if "note" in r:
                    print(f"   {s}: {r['note']}")
                    continue
                w = r["worst"]
                print(f"   {s}: 足が床に平ら・足首 roll −15〜+15° で床からの高さ {r['flat_min_mm']:.1f}〜{r['flat_max_mm']:.1f} mm。"
                      f"足の傾き pitch ±60°・roll ±30° の中で最も低い {w['clear_mm']:.1f} mm（足首 roll {w['ankle_roll_deg']:+.1f}°・"
                      f"足 pitch {w['foot_pitch_deg']:+.1f}°・足 roll {w['foot_roll_deg']:+.1f}°）。床に触れる格子 {r['n_touch']}/{r['n_grid']}")
            result[key] = res
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=1, default=float)
    print(f"\n[sweeps] 結果: {a.out}")


if __name__ == "__main__":
    main()
