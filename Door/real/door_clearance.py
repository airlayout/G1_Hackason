"""11-12 夜（作業ログ 2026-11-12 の 6 章 ㋐）: 扉の周りの当たりの模型。numpy だけ。

11-12 の機体では、10 回のうち 4 回が手・前腕と扉・レバー・壁の当たりで止まった。机の上の確かめ（reach_check_script.py）が
腕の届きと可動域しか見ていなかったため。ここでは扉の周りを箱と円柱で置き、腕を太さのある線分で置いて、すき間を測る。

  geom = load_geom("door_geom.json", door=(幅, 高さ, レバーの高さ, バックセット))
  arm = arm_segments(chain, q29, pose, quat, pelvis_z, geom)    # 扉の座標の線分（手の先・手のひら・前腕）
  gaps = clearances(arm, geom)                                  # {"板": m, "棒": m, "付け根": m, "枠": m, "壁": m}（負は食い込み）

扉の座標は door_frame.py と同じ（原点＝開口の中心の床・+x＝押して入る向き・把手が左ならレバーは +y）。把手が右の扉は y を返す。
寸法の出どころ（11-11 の作業ログ 4 章・ユーザーの巻き尺）: 板の厚さ約 4 cm（表は x −0.02）・レバーは付け根（縁から 6.4 cm）から
蝶番の側へ 13 cm・棒の中心は板の表から約 5.5 cm・棒の上面は床から 100.8 cm・枠の前の角は板の表から約 3 cm 手前・壁は約 1 cm 手前。
手の形は URDF に無いので目安（手の先は手のひらの原点から 12 cm・太さ半径 2 cm）。11-12 の記録で答え合わせした（script_gate.py --replay）。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk


@dataclass(frozen=True)
class DoorGeom:
    width: float = 0.94
    height: float = 2.05
    lever_height: float = 0.99      # 巻き尺の「レバーの高さ」（DOOR の 3 つ目）。棒の中心の高さは bar_top − bar_r
    backset: float = 0.064
    board_half: float = 0.02        # 板の厚さの半分（板の表は x = −board_half）
    bar_len: float = 0.13           # 付け根から蝶番の側への棒の長さ
    bar_r: float = 0.011            # 棒の太さの半径
    bar_off: float = 0.055          # 板の表から棒の中心まで
    bar_top: float = 1.008          # 棒の上面の高さ（床から）
    frame_front: float = 0.03       # 枠の前の角が板の表より手前に出る量
    frame_w: float = 0.04           # 枠の幅（縁から壁まで）
    wall_front: float = 0.011       # 壁の面が板の表より手前に出る量
    hand_tip: float = 0.12          # 手のひらの原点から手の先まで（手の向きに）
    hand_r: float = 0.02            # 手の太さの半径
    forearm_r: float = 0.03         # 前腕の太さの半径
    upperarm_r: float = 0.035       # 上腕の太さの半径
    lever_left: bool = True
    note: str = ""


def load_geom(path: Optional[str] = None, door: Optional[Sequence[float]] = None, lever_left: bool = True) -> DoorGeom:
    """寸法のファイル（JSON・無くてよい）を読み、DOOR（幅,高さ,レバーの高さ,バックセット）で扉の寸法を上書きする"""
    raw = {}
    if path:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    known = {f.name for f in fields(DoorGeom)}
    bad = sorted(set(raw) - known)
    if bad:
        raise ValueError(f"扉の寸法のファイルに知らない名前: {bad}")
    g = replace(DoorGeom(), **{k: (v if isinstance(getattr(DoorGeom(), k), (bool, str)) else float(v)) for k, v in raw.items()})
    if door is not None:
        w, h, lh, bs = (float(v) for v in door)
        g = replace(g, width=w, height=h, lever_height=lh, backset=bs)
    g = replace(g, lever_left=lever_left)
    for k in ("bar_len", "bar_r", "bar_off", "hand_tip", "hand_r", "forearm_r", "upperarm_r", "frame_w"):
        if not getattr(g, k) > 0:
            raise ValueError(f"扉の寸法 {k} は正: {getattr(g, k)}")
    return g


@dataclass(frozen=True)
class Box:
    lo: np.ndarray
    hi: np.ndarray


@dataclass(frozen=True)
class Rod:
    a: np.ndarray
    b: np.ndarray
    r: float


def obstacles(g: DoorGeom) -> Dict[str, object]:
    """扉の周りの物（扉の座標）。把手が右の扉は y を返す"""
    lr = 1.0 if g.lever_left else -1.0
    face = -g.board_half
    edge = g.width / 2
    root_y = edge - g.backset
    bar_x = face - g.bar_off
    bar_z = g.bar_top - g.bar_r

    def ybox(x0, x1, y0, y1, z0, z1) -> Box:
        ya, yb = sorted((lr * y0, lr * y1))
        return Box(np.array([x0, ya, z0]), np.array([x1, yb, z1]))

    return {
        "板": ybox(face, g.board_half, -edge, edge, 0.0, g.height),
        "棒": Rod(np.array([bar_x, lr * root_y, bar_z]), np.array([bar_x, lr * (root_y - g.bar_len), bar_z]), g.bar_r),
        "付け根": ybox(bar_x, face, root_y - 0.015, root_y + 0.015, bar_z - 0.015, bar_z + 0.015),
        "枠": ybox(face - g.frame_front, g.board_half, edge, edge + g.frame_w, 0.0, g.height + 0.1),
        "壁": ybox(face - g.wall_front, 0.3, edge + g.frame_w, edge + 1.5, 0.0, g.height + 0.5),
    }


def _pt_box(p: np.ndarray, b: Box) -> float:
    d = np.maximum(0.0, np.maximum(b.lo - p, p - b.hi))
    out = float(np.linalg.norm(d))
    if out > 0:
        return out
    return -float(np.min(np.minimum(p - b.lo, b.hi - p)))  # 中に入っている（負）


def _seg_seg(p0, p1, q0, q1) -> float:
    """線分と線分の最も近い距離"""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e < 1e-12:
            s, t = min(1.0, max(0.0, -c / a)), 0.0
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = min(1.0, max(0.0, -c / a)), 0.0
            elif t > 1.0:
                s, t = min(1.0, max(0.0, (b - c) / a)), 1.0
    return float(np.linalg.norm((p0 + d1 * s) - (q0 + d2 * t)))


@dataclass(frozen=True)
class Seg:
    name: str
    a: np.ndarray
    b: np.ndarray
    r: float


def arm_points(chain: fk.ArmFK, q29: np.ndarray, side: str = "left") -> Dict[str, np.ndarray]:
    """pelvis の座標の 肘・手首・手のひら（腕の線分の端）"""
    p = chain.points(q29)
    s = "L" if side == "left" else "R"
    return {"肩": p[s + "肩"], "肘": p[s + "肘"], "手首": p[s + "手首"], "手": p[s + "手"]}


def arm_segments(chain: fk.ArmFK, q29: np.ndarray, pose: dfr.PlanarPose, quat_wxyz: Optional[np.ndarray], pelvis_z: float,
                 g: DoorGeom, side: str = "left") -> List[Seg]:
    """扉の座標の腕の線分: 上腕（肩→肘）・前腕（肘→手首）・手（手首→手のひら→手の先）"""
    pts = arm_points(chain, q29, side)
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat_wxyz) if quat_wxyz is not None else (0.0, 0.0)
    R = dfr.rot_z(pose.yaw) @ dfr.tilt_matrix(roll, pitch)
    T = np.array([pose.x, pose.y, pelvis_z])
    sh, el, wr, pa = (T + R @ pts[k] for k in ("肩", "肘", "手首", "手"))
    u = pa - wr
    u = u / max(1e-9, float(np.linalg.norm(u)))
    tip = pa + g.hand_tip * u
    return [Seg("上腕", sh, el, g.upperarm_r), Seg("前腕", el, wr, g.forearm_r), Seg("手", wr, pa, g.hand_r),
            Seg("手の先", pa, tip, g.hand_r)]


def seg_gap(s: Seg, ob) -> float:
    """線分（太さ s.r）と物のすき間 [m]（負は食い込み）"""
    if isinstance(ob, Rod):
        return _seg_seg(s.a, s.b, ob.a, ob.b) - s.r - ob.r
    n = max(2, int(math.ceil(float(np.linalg.norm(s.b - s.a)) / 0.005)) + 1)
    return min(_pt_box(s.a + (s.b - s.a) * (k / (n - 1)), ob) for k in range(n)) - s.r


def clearances(segs: Sequence[Seg], g: DoorGeom, obs: Optional[Dict[str, object]] = None) -> Dict[str, Tuple[float, str]]:
    """物ごとの、腕の一番近い所のすき間と、その腕の部分の名前"""
    obs = obs if obs is not None else obstacles(g)
    out = {}
    for name, ob in obs.items():
        best = (math.inf, "")
        for s in segs:
            d = seg_gap(s, ob)
            if d < best[0]:
                best = (d, s.name)
        out[name] = best
    return out


def bar_contact(segs: Sequence[Seg], g: DoorGeom) -> Tuple[float, float]:
    """手（手のひら→手の先）と棒の、一番近い所のすき間と、そこの棒の上の位置（付け根から蝶番の側へ m）。
    棒の上に手が乗っているかの判定に使う（位置が 0〜bar_len の内で、すき間が小さい）"""
    bar = obstacles(g)["棒"]
    hand = [s for s in segs if s.name == "手の先"][0]
    best = (math.inf, math.nan)
    for k in range(41):
        p = hand.a + (hand.b - hand.a) * (k / 40)
        ab = bar.b - bar.a
        t = min(1.0, max(0.0, float((p - bar.a) @ ab) / float(ab @ ab)))
        d = float(np.linalg.norm(p - (bar.a + ab * t))) - hand.r - bar.r
        if d < best[0]:
            best = (d, t * g.bar_len)
    return best
