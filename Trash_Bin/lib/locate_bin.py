"""頭カメラの深度から、ゴミ箱の置き方（上から見た中心と向き）を求める。sim / real 共通。

手順:
  1. 深度の画素を光学座標の点にし、Button_Press/J1-gen の HeadCameraTransform で pelvis 座標に直す
  2. ゴミ箱の上面の高さ（設定の上端 bin.height と開口の縁 bin.opening_z の間、少し余裕を持たせる）の点だけを残す
  3. 上から見た点を格子の画像にして、一番大きなかたまりを取り出す（床・壁・ロボットの手は高さで外れる）
  4. かたまりに長方形を当てはめる（cv2.minAreaRect）。長い辺を箱の y（幅 36 cm）、短い辺を箱の x（奥行 25 cm）とし、
     箱の x の向きはロボットから遠ざかる向きにとる
  5. 長方形の大きさが設定の寸法と大きく違えば失敗にする（ゴミ箱ではない、または一部しか写っていない）

一部しか写っていないとき（押す位置に立つと、ゴミ箱の手前と右側が画面の外に出る）は、4. の長方形の大きさが合わない。
そこで最後に、設定の寸法の長方形を、見えている部分に合わせ直す（fit_known_size）:
  上から見た格子の各区画を、カメラの視野に入っているか（画像の中に写る位置か）で分け、長方形の中の区画のうち
  「視野に入っていて上面の点がある」区画は +1、「視野に入っているのに点が無い」区画は −1、視野の外は 0 とし、
  長方形の外にはみ出した点は −1 とする。この点数が最大になる中心と向きを探す。
  視野の外は判断しないので、切れている部分は寸法から推測し、見えている縁（奥の縁・左の縁）で位置と向きが決まる。

⚠️ 実物はステンレスの鏡面なので、上面の深度が欠けやすい（矢田さんの指摘）。実機では欠け方を確かめること。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from bin_geometry import BinPose


class LocateError(RuntimeError):
    """ゴミ箱が見つからない、または形が合わない。"""


class TooLargeError(LocateError):
    """上面のかたまりが設定の寸法より大きすぎる（壁など別の物とつながった）。"""


@dataclass
class BinDetection:
    pose: BinPose
    size_xy: tuple[float, float]  # 当てはめた長方形の大きさ（箱の x = 奥行、箱の y = 幅）[m]
    n_points: int                 # 上面の高さにあった点の数
    partial: bool                 # 一部しか写っておらず、設定の寸法の長方形で合わせ直したか
    info: dict[str, Any] = field(default_factory=dict)

    def summary(self) -> str:
        c = self.pose.center_xy
        return (f"中心 [{c[0]:+.3f}, {c[1]:+.3f}] m、向き {np.degrees(self.pose.yaw):+.1f}°、"
                f"大きさ {self.size_xy[0] * 100:.1f} × {self.size_xy[1] * 100:.1f} cm、点 {self.n_points} 個"
                + ("（一部しか写っていない。設定の寸法で合わせ直した）" if self.partial else ""))


def depth_to_pelvis(frame: Any, transform: Any, q_waist: np.ndarray, step: int = 2) -> np.ndarray:
    """深度画像（RgbdFrame）の有効な画素を、pelvis 座標の点（N×3）にする。step 画素おきに間引く。"""
    d = frame.depth_m()[::step, ::step]
    intr = frame.intrinsics
    vs, us = np.mgrid[0:frame.depth.shape[0]:step, 0:frame.depth.shape[1]:step]
    ok = (d > 0.15) & (d < 3.0)
    z = d[ok]
    x = (us[ok] - intr.cx) / intr.fx * z
    y = (vs[ok] - intr.cy) / intr.fy * z
    r, p = transform.pelvis_from_optical(np.asarray(q_waist, dtype=float))
    pts = np.stack([x, y, z], axis=1) @ r.T + p
    return pts + transform.offset


@dataclass
class FloorLevel:
    """床を水平・設定の高さにする補正: p' = R (p − c) + c + [0, 0, dz]（c はカメラの位置。カメラの向きのずれを直すので
    カメラのまわりに回す）。"""
    rot: np.ndarray
    pivot: np.ndarray
    dz: float
    tilt_deg: float      # 補正した傾き（床の法線と鉛直の角度）
    n_points: int

    def apply(self, p: np.ndarray) -> np.ndarray:
        return (np.asarray(p) - self.pivot) @ self.rot.T + self.pivot + np.array([0.0, 0.0, self.dz])

    def unapply(self, p: np.ndarray) -> np.ndarray:
        return (np.asarray(p) - np.array([0.0, 0.0, self.dz]) - self.pivot) @ self.rot + self.pivot

    def summary(self) -> dict[str, float]:
        return {"tilt_deg": round(self.tilt_deg, 2), "dz_m": round(self.dz, 4), "n_points": self.n_points}


def _floor_level(pts: np.ndarray, transform: Any, q_waist: np.ndarray, z_floor: float,
                 search_m: float = 0.15, min_points: int = 1500, max_tilt_deg: float = 8.0) -> FloorLevel | None:
    """床の点（設定の床の高さ ± search_m）に平面を合わせ、床を水平・設定の高さにする補正を求める。

    まず傾き max_tilt_deg 以内の平面のうち点が一番多く乗るもの（RANSAC）を選び、その近くの点だけで 3 回合わせ直す。
    候補の高さの範囲には、壁の下の方や台の上面も入る。全部の点にいきなり平面を合わせると、ゴミ箱の背面を壁に付けたとき
    壁に引っ張られて床が 7.8° 傾いて求まり、上面の高さの点がずれて位置が最大 7.8 cm ずれた（2026-11-07）。
    点が少ない・水平に近い平面が見つからないときは None。
    """
    cand = pts[np.abs(pts[:, 2] - z_floor) < search_m]
    if len(cand) < min_points:
        return None
    rng = np.random.default_rng(0)  # 毎回同じ結果にする
    sub = cand if len(cand) <= 20000 else cand[rng.choice(len(cand), 20000, replace=False)]
    cos_max = float(np.cos(np.radians(max_tilt_deg)))
    best_n, best_p, best_in = None, None, 0
    for _ in range(300):
        p3 = sub[rng.choice(len(sub), 3, replace=False)]
        nrm = np.cross(p3[1] - p3[0], p3[2] - p3[0])
        ln = float(np.linalg.norm(nrm))
        if ln < 1e-9 or abs(nrm[2]) / ln < cos_max:
            continue  # 傾きすぎ（壁など）
        nrm = nrm / ln
        n_in = int(np.count_nonzero(np.abs((sub - p3[0]) @ nrm) < 0.015))
        if n_in > best_in:
            best_n, best_p, best_in = nrm, p3[0], n_in
    if best_n is None or best_in < min(min_points, len(sub)) // 2:
        return None
    near = cand[np.abs((cand - best_p) @ best_n) < 0.03]
    sel = near
    for _ in range(3):
        a = np.c_[sel[:, 0], sel[:, 1], np.ones(len(sel))]
        coef, *_ = np.linalg.lstsq(a, sel[:, 2], rcond=None)
        res = near[:, 2] - np.c_[near[:, 0], near[:, 1], np.ones(len(near))] @ coef
        sel = near[np.abs(res) < max(0.01, 2.5 * np.std(res[np.abs(res) < 0.05]) if np.any(np.abs(res) < 0.05) else 0.01)]
        if len(sel) < min_points // 2:
            return None
    n = np.array([-coef[0], -coef[1], 1.0])
    n /= np.linalg.norm(n)
    tilt = float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))))
    if tilt > max_tilt_deg:
        return None
    # n を鉛直（0, 0, 1）に重ねる回転（ロドリゲスの式）
    axis = np.cross(n, [0.0, 0.0, 1.0])
    s_, c_ = np.linalg.norm(axis), float(n[2])
    if s_ < 1e-9:
        rot = np.eye(3)
    else:
        k = axis / s_
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        rot = np.eye(3) + s_ * kx + (1 - c_) * (kx @ kx)
    _, pivot = transform.pelvis_from_optical(np.asarray(q_waist, dtype=float))
    pivot = pivot + transform.offset
    leveled = (sel - pivot) @ rot.T + pivot
    dz = float(z_floor - np.median(leveled[:, 2]))
    return FloorLevel(rot=rot, pivot=pivot, dz=dz, tilt_deg=tilt, n_points=int(len(sel)))


def locate_bin(frame: Any, transform: Any, q_waist: np.ndarray, cfg: dict[str, Any],
               band_m: tuple[float, float] = (0.02, 0.015), cell_m: float = 0.005,
               size_tol_m: float = 0.05, refine: bool = True, min_score: float = 0.6,
               join_m: float = 0.10, prior: BinPose | None = None, level_with_floor: bool = True,
               wall_above_m: float | None = None, wall_below_m: float = 0.08, wall_pad_m: float = 0.015,
               remove_walls: bool | None = None) -> BinDetection:
    """深度から、ゴミ箱の置き方を求める。見つからなければ LocateError。

    band_m: 上面の高さの範囲の余裕（開口の縁より下へ、上端より上へ）[m]
    cell_m: 上から見た格子の大きさ [m]
    size_tol_m: 長方形の大きさと設定の寸法の差の許容 [m]。これを超えて小さければ一部しか写っていないとみなす
        （大きい方は、遠いときの深度のノイズを考えて 3 倍まで許す）
    refine: 全部写っていても、設定の寸法の長方形で合わせ直す
    min_score: 合わせ直したときの点数（見えている上面の区画のうち、長方形とよく合った割合）の下限
    join_m: 一番大きなかたまりからこの距離以内のかたまりも、上面の一部としてまとめる [m]
    prior: 前回の置き方（近づきながら測り直すとき）。与えると、合わせ直しはその近く（±6 cm・±6°）だけを探すので速い
    level_with_floor: 床に平面を合わせ、床が水平で設定の高さ（−pelvis_height_m）になるように点を回し直す。
        頭カメラの向きが計算に使う値（robot.yaml）から 3° ずれると、1 m 先の点の高さが約 5 cm ずれて上面の高さの範囲から
        外れる（2026-11-05、乱しを入れたシミュレーションで、30 回中 15 回が「見つからない」で失敗した）。
        床が写っていなければ補正しない
    wall_above_m, wall_below_m, wall_pad_m: 壁の区画を、上面から除き、長方形の当てはめでも「見えない」扱いにする（見分け方は本文）。
        ゴミ箱の背面を壁に付けると、壁のうち上面と同じ高さの帯が上面とつながり、35 × 114 cm の 1 つのかたまりになって
        「ゴミ箱ではない」で失敗した（2026-11-07、深度のノイズありで 40 回中 40 回）
    """
    import cv2

    if remove_walls is None:
        # まず壁を除かずに測り、上面が大きすぎた（壁とつながった）ときだけ、壁を除いて測り直す。
        # 立つ位置ではゴミ箱の手前の縁が画像の下に切れ、前後の位置は後ろの縁だけで決まる。背面を壁に付けたときに壁と一緒に
        # 後ろの縁を除くと、前後の位置が決まらず 3 cm ずれた。遠いときは壁の帯が上面とつながるので除く必要がある（2026-11-07）
        kw = dict(band_m=band_m, cell_m=cell_m, size_tol_m=size_tol_m, refine=refine, min_score=min_score, join_m=join_m,
                  prior=prior, level_with_floor=level_with_floor, wall_above_m=wall_above_m, wall_below_m=wall_below_m,
                  wall_pad_m=wall_pad_m)
        try:
            return locate_bin(frame, transform, q_waist, cfg, remove_walls=False, **kw)
        except TooLargeError:
            det = locate_bin(frame, transform, q_waist, cfg, remove_walls=True, **kw)
            det.info["walls_removed"] = True
            return det

    b = cfg["bin"]
    ph = float(cfg["pelvis_height_m"])
    base = float(b.get("stand_height_m", 0.0))  # 台に載せた高さ
    z_top = float(b["height"]) + base - ph
    z_open = float(b["opening_z"]) + base - ph
    depth_m, width_m = float(b["depth"]), float(b["width"])

    pts = depth_to_pelvis(frame, transform, q_waist)
    level = None
    if level_with_floor:
        level = _floor_level(pts, transform, q_waist, -ph)
        if level is not None:
            pts = level.apply(pts)
    top = pts[(pts[:, 2] > z_open - band_m[0]) & (pts[:, 2] < z_top + band_m[1])]
    if len(top) < 200:
        raise LocateError(f"ゴミ箱の上面の高さの点が少ない（{len(top)} 個）。写っていないか、深度が欠けている")

    # 上から見た格子の画像にして、一番大きなかたまりを取り出す
    xy = top[:, :2]
    # 格子は、見えている点の範囲よりゴミ箱の大きさ分だけ広くとる。狭いと、長方形が格子の外へはみ出しても
    # 「見えているのに空」の減点が付かず、ずれた位置が選ばれる（2026-11-05、中心が 6 cm ずれた）
    margin = max(depth_m, width_m) * 0.7
    lo = xy.min(axis=0) - margin
    ij = np.floor((xy - lo) / cell_m).astype(int)
    h, w = ij.max(axis=0) + 1 + int(np.ceil(margin / cell_m))
    img = np.zeros((h, w), np.uint8)
    img[ij[:, 0], ij[:, 1]] = 255
    # 壁の区画を除く。壁のうち上面と同じ高さの帯に入った点が上面とつながるため。見分け方は 2 つ:
    # (1) 上面より wall_above_m 以上高い点がある区画（近いと壁の上の方が写る）。既定は使わない: 立つ位置で、背面に付けた壁の
    #     点を除く幅の分、ゴミ箱の後ろの縁まで削れて、位置が 3 cm 手前にずれた（2026-11-07）
    # (2) 上面の高さの帯より wall_below_m 以上低い点（床より 5 cm 上まで）が、ゴミ箱の幅より長くまっすぐ並ぶ線（遠いと壁の下の方だけ写る）。
    #     ゴミ箱の背面のすぐ後ろの壁は、ゴミ箱に隠れて下の方が写らないので、線を画像の端まで伸ばして除く。
    #     ゴミ箱自身の前面・側面（幅 36 cm 以下）は線として拾わない（拾うと前の縁が削れて前後の位置が決まらない）
    zz = pts[:, 2]

    def to_cells(p: np.ndarray) -> np.ndarray:
        c = np.zeros((h, w), np.uint8)
        q = np.floor((p[:, :2] - lo) / cell_m).astype(int)
        q = q[(q[:, 0] >= 0) & (q[:, 0] < h) & (q[:, 1] >= 0) & (q[:, 1] < w)]
        c[q[:, 0], q[:, 1]] = 255
        return c

    pad = 2 * int(round(wall_pad_m / cell_m)) + 1
    tall = np.zeros((h, w), bool)
    if remove_walls and wall_above_m is not None:
        above = cv2.morphologyEx(to_cells(pts[zz > z_top + wall_above_m]), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        tall = cv2.dilate(above, np.ones((pad, pad), np.uint8)) > 0
    low = cv2.morphologyEx(to_cells(pts[(zz < z_open - band_m[0] - wall_below_m) & (zz > -ph + 0.05)]),
                           cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    min_len = int((width_m + 0.15) / cell_m)
    # 線は途切れずに（すき間 5 cm まで）ゴミ箱の幅 + 15 cm 以上続くものだけ。すき間を大きく許すと、ゴミ箱の前面や中の壁の切れ端が
    # 1 本につながり、上面を横切って削った（2026-11-07、位置が最大 7.8 cm ずれた）
    lines = (cv2.HoughLinesP(low, 1, np.pi / 360, threshold=min_len, minLineLength=min_len, maxLineGap=int(0.05 / cell_m))
             if remove_walls else None)
    wall_mask = np.zeros((h, w), np.uint8)
    for x1_, y1_, x2_, y2_ in ([] if lines is None else lines[:, 0]):
        d = np.array([x2_ - x1_, y2_ - y1_], dtype=float)
        d /= max(np.linalg.norm(d), 1e-9)
        far = 2 * (h + w)
        p1 = (int(x1_ - d[0] * far), int(y1_ - d[1] * far))
        p2 = (int(x2_ + d[0] * far), int(y2_ + d[1] * far))
        cv2.line(wall_mask, p1, p2, 1, thickness=pad + 4)  # 深度のノイズで壁の点が前後に散る分も含める
    tall |= wall_mask.astype(bool)
    img[tall] = 0
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    # 深度のノイズで散った点（ぽつぽつした小さな点）を取り除く
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(img)
    if n < 2:
        raise LocateError("上面の点のかたまりが見つからない")
    # 一番大きなかたまりと、その近く（join_m 以内）のかたまりをまとめてゴミ箱の上面とする。
    # 手などが上面の一部を隠すと、かたまりが分かれる（2026-11-05、開始の姿勢の右手で分かれた）
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    main = (labels == k).astype(np.uint8)
    near = cv2.dilate(main, np.ones((2 * int(join_m / cell_m) + 1,) * 2, np.uint8)).astype(bool)
    keep = np.isin(labels, [j for j in range(1, n) if (near & (labels == j)).any()])
    cells = np.argwhere(keep)
    blob = (cells + 0.5) * cell_m + lo  # 格子の中心の座標（x, y）[m]

    (cx, cy), (s1, s2), ang = cv2.minAreaRect(blob.astype(np.float32))
    # 長方形の辺の向き（cv2 の角度は、1 つ目の辺 s1 の向き）
    a1 = np.radians(ang)
    e1 = np.array([np.cos(a1), np.sin(a1)])
    e2 = np.array([-np.sin(a1), np.cos(a1)])
    # pelvis の前方（x）に近い辺を箱の x（奥行）にする。ロボットはゴミ箱にだいたい正面から近づく（向きの差 ±45° 以内）前提。
    # 長い辺を幅とする決め方は、一部しか写っていないと取り違える（2026-11-05、MuJoCo で向きが 90° 近くずれた）
    if abs(e1[0]) >= abs(e2[0]):
        ex, ey, sx, sy = e1, e2, s1, s2
    else:
        ex, ey, sx, sy = e2, e1, s2, s1
    center = np.array([cx, cy])
    # 箱の x の向きは、ロボット（pelvis の原点）から遠ざかる向き
    if float(ex @ center) < 0:
        ex = -ex
    yaw = float(np.arctan2(ex[1], ex[0]))

    partial = abs(sy - width_m) > size_tol_m or abs(sx - depth_m) > size_tol_m
    # 遠いと深度のノイズで点が散り、当てはめた長方形が大きくなる（1 m で約 5 cm）。大きすぎの判定はゆるくし、
    # ゴミ箱かどうかは、設定の寸法の長方形を合わせたときの点数（min_score）で決める
    if sy > width_m + 3 * size_tol_m or sx > depth_m + 3 * size_tol_m:
        raise TooLargeError(f"上面が {sx * 100:.1f} × {sy * 100:.1f} cm（設定 {depth_m * 100:.0f} × {width_m * 100:.0f} cm）"
                          "より大きい。ゴミ箱ではない")
    score = None
    if partial or refine:
        # 設定の寸法の長方形を、見えている部分に合わせ直す
        visible = _visible_cells(transform, q_waist, frame, lo, (h, w), cell_m, z_top - 0.01, level=level)
        visible &= ~tall  # 壁などで除いた区画は、空かどうか分からない
        if prior is not None:
            center, yaw = np.asarray(prior.center_xy, dtype=float), float(prior.yaw)
            window = (0.06, 6.0)
        else:
            window = (0.20, 20.0)
        center, yaw, score = _fit_known_size(keep, visible, lo, cell_m, center, yaw, depth_m, width_m, window)
        if score < min_score:
            raise LocateError(f"設定の寸法の長方形が、見えている上面と合わない（点数 {score:.2f}、下限 {min_score}）")
    return BinDetection(pose=BinPose(center_xy=center, yaw=yaw), size_xy=(float(sx), float(sy)),
                        n_points=int(len(top)), partial=bool(partial),
                        info={"rect_center": [float(cx), float(cy)], "rect_size": [float(s1), float(s2)],
                              "rect_angle_deg": float(ang), "fit_score": score, "tall_cells": int(tall.sum()),
                              "floor_level": None if level is None else level.summary()})


def _visible_cells(transform: Any, q_waist: np.ndarray, frame: Any, lo: np.ndarray, shape: tuple[int, int],
                   cell_m: float, z: float, occlusion_m: float = 0.05, level: "FloorLevel | None" = None) -> np.ndarray:
    """上から見た格子の各区画（高さ z の点）が、カメラから見えるか（True / False）。

    画像の外に写る区画と、手前に物（手など）があって隠れている区画は「見えない」とする。隠れているかは、その区画が
    写るはずの画素の深度が、区画までの距離より occlusion_m 以上短いかで決める（深度が無い画素も「見えない」）。
    """
    intr = frame.intrinsics
    h, w = shape
    ii, jj = np.mgrid[0:h, 0:w]
    xy = (np.stack([ii, jj], axis=-1) + 0.5) * cell_m + lo
    p = np.concatenate([xy.reshape(-1, 2), np.full((h * w, 1), z)], axis=1)
    if level is not None:
        p = level.unapply(p)  # 床で補正した座標 → 補正する前の座標（カメラで写る位置を求めるため）
    p = p - transform.offset
    r, t = transform.pelvis_from_optical(np.asarray(q_waist, dtype=float))
    q = (p - t) @ r  # pelvis → 光学座標（r は直交行列なので逆は転置）
    zc = q[:, 2]
    u = intr.fx * q[:, 0] / np.maximum(zc, 1e-6) + intr.cx
    v = intr.fy * q[:, 1] / np.maximum(zc, 1e-6) + intr.cy
    ok = (zc > 0.1) & (u >= 0) & (u <= intr.width - 1) & (v >= 0) & (v <= intr.height - 1)
    dm = frame.depth_m()
    ui = np.clip(np.round(u).astype(int), 0, intr.width - 1)
    vi = np.clip(np.round(v).astype(int), 0, intr.height - 1)
    measured = dm[vi, ui]
    ok &= (measured > 0) & (measured > zc - occlusion_m)
    return ok.reshape(h, w)


def _rect_score(blob: np.ndarray, vis_empty: np.ndarray, lo: np.ndarray, cell_m: float, c: np.ndarray, a: float,
                depth_m: float, width_m: float) -> float:
    """中心 c・向き a の長方形の点数（_fit_known_size の説明を参照）。"""
    import cv2

    ex = np.array([np.cos(a), np.sin(a)])
    ey = np.array([-np.sin(a), np.cos(a)])
    corners = [c + sx * ex * depth_m / 2 + sy * ey * width_m / 2 for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    poly = np.array([(np.asarray(k) - lo) / cell_m for k in corners])[:, ::-1]  # (列 = y, 行 = x) の順
    mask = np.zeros(blob.shape, np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
    m = mask.astype(bool)
    blob_n = max(1, int(blob.sum()))
    return float((m & blob).sum() - (m & vis_empty).sum() - (~m & blob).sum()) / blob_n


def _fit_known_size(blob: np.ndarray, visible: np.ndarray, lo: np.ndarray, cell_m: float, center: np.ndarray,
                    yaw: float, depth_m: float, width_m: float,
                    window: tuple[float, float] = (0.20, 20.0)) -> tuple[np.ndarray, float, float]:
    """設定の寸法の長方形を、見えている上面（blob）に合わせる。(中心, 向き, 点数) を返す。

    点数 = (長方形の中で、視野に入っていて点がある区画 − 視野に入っているのに点が無い区画 − 長方形の外の点) ÷ 点の区画の数。
    はじめに中心 ±window[0] m・向き ±window[1]°（既定 ±20 cm・±20°）を粗く全部調べ、そのあと中心は 1 cm → 5 mm → 2 mm、向きは 2° → 1° → 0.5° と
    刻みを細かくしながら、点数が上がる向きへ動かす。
    """
    vis_empty = visible & ~blob

    def score(c: np.ndarray, a: float) -> float:
        return _rect_score(blob, vis_empty, lo, cell_m, c, a, depth_m, width_m)

    # 粗く全部調べる（既定は中心 ±20 cm を 2 cm おき、向き ±20° を 4° おき）。見えている部分が小さいと、近くだけを探すのでは
    # 正しい位置に届かない（点数が変わらない平らなところで止まる）
    best_c, best_a, best = np.asarray(center, dtype=float).copy(), float(yaw), -np.inf
    w_m, w_deg = window
    for da in np.radians(np.arange(-w_deg, w_deg + 1e-9, 2.0 if w_deg <= 8 else 4.0)):
        for dx in np.arange(-w_m, w_m + 1e-9, 0.02):
            for dy in np.arange(-w_m, w_m + 1e-9, 0.02):
                c, a = np.asarray(center) + [dx, dy], yaw + da
                sc = score(c, a)
                if sc > best:
                    best, best_c, best_a = sc, c, a
    for step_c, step_a in ((0.01, np.radians(2.0)), (0.005, np.radians(1.0)), (0.002, np.radians(0.5))):
        improved = True
        while improved:
            improved = False
            for dc, da in ((np.array([step_c, 0]), 0), (np.array([-step_c, 0]), 0), (np.array([0, step_c]), 0),
                           (np.array([0, -step_c]), 0), (np.zeros(2), step_a), (np.zeros(2), -step_a)):
                c, a = best_c + dc, best_a + da
                sc = score(c, a)
                if sc > best + 1e-9:
                    best, best_c, best_a, improved = sc, c, a, True
    return best_c, best_a, best


def project_to_image(points_pelvis: np.ndarray, transform: Any, q_waist: np.ndarray, intr: Any) -> np.ndarray:
    """pelvis 座標の点（N×3）を、頭カメラの画像の画素 (u, v)（N×2）にする。カメラの後ろの点は nan。"""
    p = np.atleast_2d(np.asarray(points_pelvis, dtype=float)) - transform.offset
    r, t = transform.pelvis_from_optical(np.asarray(q_waist, dtype=float))
    q = (p - t) @ r
    with np.errstate(divide="ignore", invalid="ignore"):
        u = intr.fx * q[:, 0] / q[:, 2] + intr.cx
        v = intr.fy * q[:, 1] / q[:, 2] + intr.cy
    uv = np.stack([u, v], axis=1)
    uv[q[:, 2] <= 0.05] = np.nan
    return uv


def draw_detection(frame: Any, transform: Any, q_waist: np.ndarray, cfg: dict[str, Any], det: BinDetection,
                   push_point_pelvis: np.ndarray | None = None) -> np.ndarray:
    """カラー画像に、測ったゴミ箱の上面（緑の長方形）・開口（黄）・押す点（赤）を描く（BGR）。見つけ方の確認用。"""
    import cv2

    from bin_geometry import local_geometry

    g = local_geometry(cfg)
    img = frame.color_bgr.copy()

    def rect(x0: float, x1: float, y0: float, y1: float, z: float) -> np.ndarray:
        local = np.array([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]])
        return np.array([det.pose.to_pelvis(p) for p in local])

    for pts, color in ((rect(g["x0"], g["x1"], g["ya"], g["yb"], g["z_top"]), (0, 255, 0)),
                       (rect(g["ox0"], g["ox1"], g["oy0"], g["oy1"], g["z_open"]), (0, 255, 255))):
        uv = project_to_image(pts, transform, q_waist, frame.intrinsics)
        if np.all(np.isfinite(uv)):
            cv2.polylines(img, [np.round(uv).astype(np.int32)], True, color, 2)
    if push_point_pelvis is not None:
        uv = project_to_image(push_point_pelvis, transform, q_waist, frame.intrinsics)[0]
        if np.all(np.isfinite(uv)):
            cv2.circle(img, (int(round(uv[0])), int(round(uv[1]))), 6, (0, 0, 255), -1)
    # cv2 は日本語を描けないので、英数字で書く
    c = det.pose.center_xy
    text = (f"bin center ({c[0]:+.3f}, {c[1]:+.3f}) m  yaw {np.degrees(det.pose.yaw):+.1f} deg"
            f"  seen {det.size_xy[0] * 100:.0f}x{det.size_xy[1] * 100:.0f} cm{'  partial' if det.partial else ''}")
    cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return img
