"""11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K4・K0 の下調べ）: 腕を振ると胴（pelvis）を鉛直軸のまわりに回そうとする
モーメントを、関節の角度の記録から出す。numpy だけ。シムの評価の記録（state_recorder）と実機の走らせ役の記録（door_stock_*.npz）の両方。

  M_z(t) = −d/dt Σ_i m_i (r_i × v_i)_z   （i は左右の腕の部品。r・v は pelvis の座標の重心の位置と速さ。部品の自転の分は入れない）
  腕が角運動量を得る分だけ、反作用で胴は逆へ回される（符号は「胴を回す向き」。上から見て反時計回りが正）
脚で踏ん張る力・扉や壁に当たった力は入らない（腕の振りの反動だけ）。速さは 0.02 s の差分を SMOOTH 歩で均す。

  シム: python arm_yaw_moment_1111.py sim <records/...> [<records/...> ...] [--t-max 2.5]
  実機: python arm_yaw_moment_1111.py real <door_stock_*.npz> [...]
各記録の、段 2（シムは回の始め）から t-max 秒までの |M_z| の最大（シムは台ごとの最大の 50・90% 点）と、正の向き（把手の側へ回す）の最大
"""
from __future__ import annotations

import argparse
import glob
import os
import xml.etree.ElementTree as ET
from typing import Dict, List

import numpy as np

import g1_arm_fk as fk

SMOOTH = 5  # 差分を均す歩の数（0.1 s）
ARM_PREFIX = ("left_shoulder", "left_elbow", "left_wrist", "left_rubber", "left_hand", "right_shoulder", "right_elbow",
              "right_wrist", "right_rubber", "right_hand")


def arm_links(urdf: str) -> Dict[str, tuple]:
    out = {}
    for ln in ET.parse(urdf).getroot().findall("link"):
        name = ln.get("name")
        ine = ln.find("inertial")
        if ine is None or not name.startswith(ARM_PREFIX):
            continue
        o = ine.find("origin")
        com = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        out[name] = (float(ine.find("mass").get("value")), com)
    return out


def com_positions(chain: fk.ArmFK, links: Dict[str, tuple], q29: np.ndarray) -> np.ndarray:
    """[部品, 3] の重心（pelvis の座標）"""
    P = []
    for name, (_, com) in links.items():
        p, R = chain.link_pose(name, q29)
        P.append(p + R @ com)
    return np.array(P)


def yaw_moment(chain: fk.ArmFK, links: Dict[str, tuple], Q: np.ndarray, dt: float) -> np.ndarray:
    """関節の角度の列 Q [T, 29]（SDK の順）から、胴を回すモーメント M_z [T] [N·m]"""
    m = np.array([v[0] for v in links.values()])
    R = np.array([com_positions(chain, links, q) for q in Q])  # [T, 部品, 3]
    V = np.gradient(R, dt, axis=0)
    Lz = (m[None, :] * (R[..., 0] * V[..., 1] - R[..., 1] * V[..., 0])).sum(axis=1)
    k = np.ones(SMOOTH) / SMOOTH
    Lz = np.convolve(Lz, k, mode="same")
    return -np.gradient(Lz, dt)


def sim_rows(rec: str, chain: fk.ArmFK, links: Dict[str, tuple], t_max: float) -> Dict[str, float]:
    st = np.load(os.path.join(rec, "static.npz"), allow_pickle=True)
    names = [str(n) for n in st["dof_names"]]
    order = [names.index(n) for n in fk.JOINT_ORDER]
    dt = float(st["dt"])
    files = sorted(glob.glob(os.path.join(rec, "states_*.npz")))
    dof = np.concatenate([np.load(f)["dof_pos"] for f in files])
    ep = np.concatenate([np.load(f)["episode_length"] for f in files])
    peaks, pos = [], []
    for e in range(dof.shape[1]):
        end = np.nonzero(np.diff(ep[:, e]) < 0)[0]
        L = int(min(end[0] + 1 if len(end) else len(ep), round(t_max / dt)))
        M = yaw_moment(chain, links, dof[:L, e][:, order], dt)
        peaks.append(np.abs(M).max())
        pos.append(M.max())
    return {"abs_p50": float(np.percentile(peaks, 50)), "abs_p90": float(np.percentile(peaks, 90)),
            "pos_p50": float(np.percentile(pos, 50)), "pos_p90": float(np.percentile(pos, 90))}


def real_rows(path: str, chain: fk.ArmFK, links: Dict[str, tuple], t_max: float) -> Dict[str, float]:
    z = np.load(path, allow_pickle=True)
    st, t = z["staged"], z["t"]
    in2 = np.isfinite(st[:, 0]) & (st[:, 0] == 2) & (st[:, 2] == 2)
    k0 = int(np.flatnonzero(in2)[0])
    k1 = int(np.searchsorted(t, t[k0] + t_max))
    tt = t[k0:k1] - t[k0]
    grid = np.arange(0.0, tt[-1], 0.02)
    Q = np.stack([np.interp(grid, tt, z["q"][k0:k1, j]) for j in range(29)], axis=1)  # 0.02 s の等間隔に直す
    M = yaw_moment(chain, links, Q, 0.02)
    k = int(np.argmax(np.abs(M)))
    return {"abs_max": float(np.abs(M).max()), "t_abs_max": float(grid[k]), "pos_max": float(M.max()), "t_pos_max": float(grid[int(np.argmax(M))])}


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=("sim", "real"))
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--t-max", type=float, default=2.5)
    ap.add_argument("--urdf", default=None)
    a = ap.parse_args(argv)
    chain = fk.load_chain(a.urdf)
    links = arm_links(a.urdf or fk.default_urdf())
    print(f"[arm_yaw_moment] 腕の部品 {len(links)} 個・質量の和 {sum(v[0] for v in links.values()):.2f} kg・0〜{a.t_max} s")
    for p in a.paths:
        r = sim_rows(p, chain, links, a.t_max) if a.kind == "sim" else real_rows(p, chain, links, a.t_max)
        print(os.path.basename(p.rstrip("/")), {k: round(v, 2) for k, v in r.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
