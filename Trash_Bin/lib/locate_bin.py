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


def locate_bin(frame: Any, transform: Any, q_waist: np.ndarray, cfg: dict[str, Any],
               band_m: tuple[float, float] = (0.02, 0.015), cell_m: float = 0.005,
               size_tol_m: float = 0.05, refine: bool = True, min_score: float = 0.6,
               join_m: float = 0.10) -> BinDetection:
    """深度から、ゴミ箱の置き方を求める。見つからなければ LocateError。

    band_m: 上面の高さの範囲の余裕（開口の縁より下へ、上端より上へ）[m]
    cell_m: 上から見た格子の大きさ [m]
    size_tol_m: 長方形の大きさと設定の寸法の差の許容 [m]。これを超えて小さければ一部しか写っていないとみなす
    refine: 全部写っていても、設定の寸法の長方形で合わせ直す
    min_score: 合わせ直したときの点数（見えている上面の区画のうち、長方形とよく合った割合）の下限
    join_m: 一番大きなかたまりからこの距離以内のかたまりも、上面の一部としてまとめる [m]
    """
    import cv2

    b = cfg["bin"]
    ph = float(cfg["pelvis_height_m"])
    z_top = float(b["height"]) - ph
    z_open = float(b["opening_z"]) - ph
    depth_m, width_m = float(b["depth"]), float(b["width"])

    pts = depth_to_pelvis(frame, transform, q_waist)
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
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
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
    if sy > width_m + size_tol_m or sx > depth_m + size_tol_m:
        raise LocateError(f"上面が {sx * 100:.1f} × {sy * 100:.1f} cm（設定 {depth_m * 100:.0f} × {width_m * 100:.0f} cm）"
                          "より大きい。ゴミ箱ではない")
    score = None
    if partial or refine:
        # 設定の寸法の長方形を、見えている部分に合わせ直す
        visible = _visible_cells(transform, q_waist, frame, lo, (h, w), cell_m, z_top - 0.01)
        center, yaw, score = _fit_known_size(keep, visible, lo, cell_m, center, yaw, depth_m, width_m)
        if score < min_score:
            raise LocateError(f"設定の寸法の長方形が、見えている上面と合わない（点数 {score:.2f}、下限 {min_score}）")
    return BinDetection(pose=BinPose(center_xy=center, yaw=yaw), size_xy=(float(sx), float(sy)),
                        n_points=int(len(top)), partial=bool(partial),
                        info={"rect_center": [float(cx), float(cy)], "rect_size": [float(s1), float(s2)],
                              "rect_angle_deg": float(ang), "fit_score": score})


def _visible_cells(transform: Any, q_waist: np.ndarray, frame: Any, lo: np.ndarray, shape: tuple[int, int],
                   cell_m: float, z: float, occlusion_m: float = 0.05) -> np.ndarray:
    """上から見た格子の各区画（高さ z の点）が、カメラから見えるか（True / False）。

    画像の外に写る区画と、手前に物（手など）があって隠れている区画は「見えない」とする。隠れているかは、その区画が
    写るはずの画素の深度が、区画までの距離より occlusion_m 以上短いかで決める（深度が無い画素も「見えない」）。
    """
    intr = frame.intrinsics
    h, w = shape
    ii, jj = np.mgrid[0:h, 0:w]
    xy = (np.stack([ii, jj], axis=-1) + 0.5) * cell_m + lo
    p = np.concatenate([xy.reshape(-1, 2), np.full((h * w, 1), z)], axis=1) - transform.offset
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
                    yaw: float, depth_m: float, width_m: float) -> tuple[np.ndarray, float, float]:
    """設定の寸法の長方形を、見えている上面（blob）に合わせる。(中心, 向き, 点数) を返す。

    点数 = (長方形の中で、視野に入っていて点がある区画 − 視野に入っているのに点が無い区画 − 長方形の外の点) ÷ 点の区画の数。
    はじめに中心 ±20 cm・向き ±20° を粗く全部調べ、そのあと中心は 1 cm → 5 mm → 2 mm、向きは 2° → 1° → 0.5° と
    刻みを細かくしながら、点数が上がる向きへ動かす。
    """
    vis_empty = visible & ~blob

    def score(c: np.ndarray, a: float) -> float:
        return _rect_score(blob, vis_empty, lo, cell_m, c, a, depth_m, width_m)

    # 粗く全部調べる（中心 ±20 cm を 2 cm おき、向き ±20° を 4° おき）。見えている部分が小さいと、近くだけを探すのでは
    # 正しい位置に届かない（点数が変わらない平らなところで止まる）
    best_c, best_a, best = np.asarray(center, dtype=float).copy(), float(yaw), -np.inf
    for da in np.radians(np.arange(-20, 21, 4)):
        for dx in np.arange(-0.20, 0.201, 0.02):
            for dy in np.arange(-0.20, 0.201, 0.02):
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
