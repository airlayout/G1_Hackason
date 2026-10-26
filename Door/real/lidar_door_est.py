"""段 5・L1（2026-10-23）: 頭のライダーの点群から、扉の座標でのライダーの姿勢と扉の角度を 1 回の測定ごとに出す。numpy だけ（PC2 の Python 3.8）。

流れ:
  1. 止まっている間（STAND の 2〜3 秒、扉は閉じている）の点群を重ね、床で水平にする（上下逆さまの取付も戻す）
  2. 閉じた扉の板の面と左右の縁（柱の段差）から「扉の座標」を決める（シミュレータと同じ: 原点＝開口の中心の床・
     +x＝押して入る向き・閉じた板の厚みの真ん中が x = 0）。door_lidar_geometry.py と同じ当て方
  3. 扉が開いても動かないもの（壁・柱・廊下の反対側の壁など）だけを 2 次元の地図にする。扉の板・開く範囲・自分の体の近くは除く
  4. 1 回の測定ごとに、床で水平にし直し（前の床の近くの点で当て直す）、高さ 0.3〜1.25 m の点を 2 次元の地図に
     点から線の ICP で重ね直す（初期値は前の 2 回からの等速の予測）→ 扉の座標でのライダーの x・y・向き
  5. 同じ座標で、蝶番のまわりの点から扉の角度（door_lidar_angle.door_angle と同じ当て方）

ここで出すのはライダー（頭）の床の上の姿勢。pelvis への直し（腰の関節・pelvis の傾き・取付）は制御のプロセスで行う（door_lidar_msg.py）。
⚠️ 扉の座標の x の原点は板の厚みの真ん中。ライダーは廊下側の面を見るので、厚みの半分（巻き尺）を渡す。
⚠️ 把手が左（蝶番が機体から見て右）の扉だけを確かめている（10-16・10-21 の扉）。右の扉は lever_side=-1 で u の向きを反転する。
"""
from __future__ import annotations

import math
import sys
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_angle as dla  # noqa: E402
import door_lidar_frame as dlf  # noqa: E402
import door_lidar_geometry as dlg  # noqa: E402

if sys.platform == "darwin":  # mac の BLAS（Accelerate）が matmul で出す見かけだけの警告。PC2 では本物の NaN を隠さない
    warnings.filterwarnings("ignore", category=RuntimeWarning, message=".*encountered in matmul")

FLIP = np.diag([1.0, -1.0, -1.0])  # 上下逆さまの取付を戻す（x 軸まわり 180°）
MIN_RANGE_M = 0.40  # これより近い点は自分の体・腕（10-21 の記録で 0.1〜0.2 m に腕が写る）
MAX_RANGE_M = 6.0
BAND_Z = (0.30, 1.25)  # [m] 壁・柱・扉を見る高さ（ライダーは床上約 1.21 m。上下逆さまなので上はほとんど見ない）
ANGLE_Z = (dla.Z_MIN, dla.Z_MAX)
FLOOR_TOL_M = 0.04  # 前の床の面からこの範囲の点で床を当て直す
FLOOR_R = (0.5, 3.0)  # [m] 床を当て直す点の水平距離
MIN_FLOOR_POINTS = 300
MAP_CELL_M = 0.02  # 地図の点の間引きと、近い点を引く格子
MAP_REACH_M = 0.30  # 地図の点からこの距離までの格子だけ近い点を持つ（それより遠い点は対応なし）
NORMAL_RADIUS_M = 0.08  # 地図の点の向き（法線）を出す近所
SLAB_KEEP_EDGE_M = 0.02  # 板を地図から除くとき、縁からこの幅は柱として残す
SWING_MARGIN_M = 0.10  # 扉が開く範囲（蝶番から幅＋これ）の点は地図にも重ね直しにも使わない
LEVER_REACH_M = 0.15  # 板の面から手前のこの距離まで（縁の内側）は、レバーと伸ばした手が来るので使わない
# 扉の角度の読みの確かめ（10-23、10-16 の開け閉め・10-21 の歩いた記録で決めた）:
#   ライダーが見る廊下側の面は、回る軸から板の厚みの分ずれるので、開くほど蝶番から線が離れる（10-16 で 60° 以上は中央 5.9 cm）
ANGLE_HINGE_MISS_M = 0.04  # 閉じたときの、蝶番から線までの距離の上限（10-21 の閉じた扉で最大 3.0 cm）
ANGLE_HINGE_MISS_OPEN_M = 0.08  # 開いた角度 θ で上限に足す分 ×(1 − cos θ)（90° で 12 cm）
ANGLE_MIN_SPREAD_M = 0.20  # 線に沿った点の広がり（5〜95 パーセンタイル）。読み違いは 0.05〜0.19 m だった
ANGLE_MAX_JUMP_DEG = 20.0  # 前に受け入れた値からの跳びの上限（読み違いは 1 回だけ飛ぶ。扉は 0.1 s で 20° も動かない）
ANGLE_JUMP_RESET_S = 1.0  # 受け入れた値がこれより古ければ跳びを見ない
CLOSED_MAX_DEG = 3.0  # 止まって測るとき、扉の角度がこれを越えたら「閉じていない」として扉を作らない
ICP_ITERS = 8
ICP_GATE_M = (0.25, 0.10)  # 対応を取る距離の上限（最初の回・最後の回。間は線形に絞る）
ICP_HUBER_M = 0.03
MIN_MATCHED = 150
# 受け入れの判定（10-23 のレビューで足した。予測からの跳びだけでは、壁に沿って少しずつ滑る外れを通してしまう）
ACCEPT_MAX_RMS_M = 0.05  # 重ね直しの残差（10-21 の歩いた記録で中央 15〜25 mm・最大 37〜49 mm）
ACCEPT_MIN_EIG = 0.02  # 正規方程式の最小の固有値（同じ記録で最小 0.063。小さいと壁に沿う向きが決まらない）
ACCEPT_MIN_MATCHED = 200
LEVEL_MAX_TILT_STEP_DEG = 3.0  # 1 回で床の傾きがこれ以上変わったら、床の当て直しを捨てて前の値を使う
LEVEL_MAX_H_STEP_M = 0.03
# 重ね直しの外れの見張り（10-23、10-16 の開け閉めの記録で一度外れて 5 m までずれていったので足した）
JUMP_POS_M = 0.10  # 予測からこれ以上跳んだ結果は捨てて予測を保つ（0.1 s で 10 cm＝1 m/s。歩く速さの上限は約 0.36 m/s）
JUMP_YAW_DEG = 8.0
MAX_SPEED_MPS = 0.6  # 等速の予測に使う速さの上限
MAX_YAW_RATE = 1.5  # [rad/s]
LOST_S = 1.0  # これだけ続けて捨てたら「見失った」（制御は立ったまま止まる）


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose2D:
    """扉の座標（または地図の座標）での床の上の姿勢"""

    x: float
    y: float
    yaw: float

    def compose(self, other: "Pose2D") -> "Pose2D":
        """self ⊕ other（other は self の座標で表した姿勢）"""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * other.x - s * other.y, self.y + s * other.x + c * other.y, wrap(self.yaw + other.yaw))

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(-(c * self.x + s * self.y), -(-s * self.x + c * self.y), -self.yaw)

    def apply(self, xy: np.ndarray) -> np.ndarray:
        """self の座標の点 [N, 2] を外の座標へ"""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return xy @ np.array([[c, s], [-s, c]]) + np.array([self.x, self.y])


@dataclass(frozen=True)
class Level:
    """ライダーの点 p を床で水平な座標 q = p @ R.T + [0, 0, h] へ（ライダーは (0, 0, h)、床は z = 0）"""

    R: np.ndarray
    h: float

    def apply(self, p: np.ndarray) -> np.ndarray:
        return p @ self.R.T + np.array([0.0, 0.0, self.h])

    def tilt_deg(self) -> float:
        """ライダーの上下の軸（取付を戻したもの）と鉛直のなす角"""
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.R[2] @ (FLIP @ np.array([0.0, 0.0, 1.0])))))))


def level_rotation(n: np.ndarray) -> np.ndarray:
    """床の法線 n（取付を戻した座標・上向き・単位）を真上に向ける、傾きだけの回転 R = Ry(β)·Rx(α)。
    最小の回転（door_lidar_angle.rot_between）だと、頭が前に 24° 垂れて横に 3° 傾くと向きが 0.64° 偏る（10-23 のレビュー）。
    傾きだけで作れば、向き（ZYX の yaw）は傾きによらない"""
    beta = math.asin(max(-1.0, min(1.0, -float(n[0]))))
    alpha = math.atan2(float(n[1]), float(n[2]))
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]])
    ry = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]])
    return ry @ rx


def valid_points(p: np.ndarray) -> np.ndarray:
    """有限で、自分の体より遠く・遠すぎない点（ライダーの座標）"""
    p = p[np.isfinite(p).all(1)]
    r = np.linalg.norm(p, axis=1)
    return p[(r > MIN_RANGE_M) & (r < MAX_RANGE_M)]


FLOOR_MAX_TILT_DEG = 40.0  # 床の候補の面は、取付を戻した座標の鉛直からこの角度まで（頭の垂れは 10-21 で 24°）


def level_from_scans(scans: Sequence[np.ndarray], iters: int = 400, tol: float = 0.02, seed: int = 0) -> Level:
    """止まっている間の回を重ねて床を当てる（door_lidar_angle.level と同じ考え）。
    ⚠️ ライダーより 0.6 m 下の点には壁の下の部分も入る。面の向きが鉛直から FLOOR_MAX_TILT_DEG 以内の候補だけを数え、
    壁を床と取り違えない（人工の点群で壁の点が多いと取り違えた。実物は近くの床が密なので起きていなかった）"""
    allp = np.concatenate([valid_points(s) for s in scans][::5] or [np.zeros((0, 3))]) @ FLIP.T
    low = allp[allp[:, 2] < -0.6]
    low = low[:: max(1, len(low) // 60000)]
    if len(low) < MIN_FLOOR_POINTS:
        raise ValueError(f"床の点が足りない（{len(low)} 点）")
    rng = np.random.default_rng(seed)
    cos_max = math.cos(math.radians(FLOOR_MAX_TILT_DEG))
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = low[rng.choice(len(low), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9 or abs(n[2]) / nn < cos_max:
            continue
        n /= nn
        inl = np.abs((low - a) @ n) < tol
        if inl.sum() > best_n:
            best, best_n = inl, int(inl.sum())
    if best is None:
        raise ValueError("床の面が見つからない")
    q = low[best]
    centroid = q.mean(0)
    n = np.linalg.svd(q - centroid, full_matrices=False)[2][-1]
    if n[2] < 0:
        n = -n
    return Level(level_rotation(n) @ FLIP, float(-(n @ centroid)))


def relevel(scan: np.ndarray, prev: Level) -> Optional[Level]:
    """前の床の面の近くの点で床を当て直す（歩くと頭が揺れる）。広い帯（±8 cm）で 1 回当ててから狭い帯（±4 cm）で当て直す。
    点が足りない・1 回で傾き 3° か高さ 3 cm を越えて変わった（当て違い）なら None（呼ぶ側は前の値を使う）"""
    cur = prev
    for tol in (2.0 * FLOOR_TOL_M, FLOOR_TOL_M):
        q = cur.apply(scan)
        r = np.hypot(q[:, 0], q[:, 1])
        m = (np.abs(q[:, 2]) < tol) & (r > FLOOR_R[0]) & (r < FLOOR_R[1])
        if m.sum() < MIN_FLOOR_POINTS:
            return None
        f = scan[m] @ FLIP.T  # 取付を戻しただけの座標で床に平面を当てる（ライダーが原点のまま）
        centroid = f.mean(0)
        n = np.linalg.svd(f - centroid, full_matrices=False)[2][-1]
        if n[2] < 0:
            n = -n
        cur = Level(level_rotation(n) @ FLIP, float(-(n @ centroid)))
    turn = math.degrees(math.acos(max(-1.0, min(1.0, float(cur.R[2] @ prev.R[2])))))  # 上向きの軸の向きの変わり（大きさだけでなく向きも）
    if turn > LEVEL_MAX_TILT_STEP_DEG or abs(cur.h - prev.h) > LEVEL_MAX_H_STEP_M:
        return None
    return cur


@dataclass(frozen=True)
class DoorFrame:
    """止まって測った扉。pose は水平にした座標（止まっていたときのライダーの下が原点）での扉の座標の姿勢"""

    pose: Pose2D
    width: float  # [m] 左右の縁の間
    face_x: float  # 扉の座標での、ライダーの見る板の面（廊下側）の x（＝ −厚み/2）
    lever_side: int  # +1: 把手が左（蝶番は −y）、−1: 把手が右
    lidar_in_door: Pose2D  # 止まっていたときのライダーの、扉の座標での姿勢
    info: dict


def door_frame_from_still(lev_scans: Sequence[np.ndarray], thickness_m: float, lever_side: int = +1) -> DoorFrame:
    """閉じた扉の前で止まっている間の（水平にした）回から、扉の座標を決める"""
    n0, c0 = dlf.reference_wall(list(lev_scans))
    t0 = np.array([-n0[1], n0[0]])
    if t0[1] < 0:
        t0 = -t0
    q = np.concatenate(list(lev_scans))
    v, u, z = q[:, :2] @ n0 - c0, q[:, :2] @ t0, q[:, 2]
    m = (np.abs(u) < 0.3) & (z > 0.4) & (z < 1.3) & (np.abs(v) < 0.04)
    if m.sum() < 200:
        raise ValueError(f"扉の板の点が足りない（{int(m.sum())} 点）")
    a, b = np.linalg.lstsq(np.c_[u[m], np.ones(m.sum())], v[m], rcond=None)[0]
    w = v - (a * u + b)
    left, jl = dlg.find_edge(u, w, z, +1)
    right, jr = dlg.find_edge(u, w, z, -1)
    if not (np.isfinite(left) and np.isfinite(right)):
        raise ValueError("扉の縁（柱の段差）が見つからない")
    width, wall = float(left - right), float(-c0)
    if not (0.6 < width < 1.3) or not (0.3 < wall < 2.5) or abs(math.degrees(math.atan(a))) > 20.0:
        raise ValueError(f"扉の測り方がおかしい（幅 {width:.3f} m・板までの距離 {wall:.3f} m・板の傾き {math.degrees(math.atan(a)):+.1f}°）")
    uc = 0.5 * (left + right)
    along = t0 + a * n0
    along /= np.linalg.norm(along)
    n_slab = np.array([along[1], -along[0]])
    if n_slab @ n0 < 0:
        n_slab = -n_slab  # 機体の方を向く
    p_face = (a * uc + b + c0) * n0 + uc * t0
    x_axis = -n_slab  # 押して入る向き
    origin = p_face + x_axis * (thickness_m / 2.0)
    pose = Pose2D(float(origin[0]), float(origin[1]), math.atan2(x_axis[1], x_axis[0]))
    info = {"wall_dist_m": float(-c0), "slab_tilt_deg": math.degrees(math.atan(a)), "edge_left_m": left, "edge_right_m": right,
            "jamb_left_m": jl, "jamb_right_m": jr, "slab_points": int(m.sum())}
    return DoorFrame(pose, float(left - right), -thickness_m / 2.0, int(lever_side), pose.inverse(), info)


def hinge_y(door: DoorFrame) -> float:
    return -door.lever_side * door.width / 2.0


def in_swing(xy_door: np.ndarray, door: DoorFrame) -> np.ndarray:
    """動くものが来る所: 扉の板の前（板・レバー・レバーに伸ばした手）と、開いた板が来る部屋の側（蝶番のまわり）。
    柱の段差（縁より外・板より手前）は横の位置を決める手がかりなので残す"""
    dx = xy_door[:, 0] - door.face_x  # 板の面から部屋の側が正
    dy = xy_door[:, 1] - hinge_y(door)
    front = (np.abs(xy_door[:, 1]) < door.width / 2.0 - SLAB_KEEP_EDGE_M) & (dx > -LEVER_REACH_M)
    swing = (dx > 0.005) & (np.hypot(dx, dy) < door.width + SWING_MARGIN_M)
    return front | swing


class StaticMap:
    """扉の座標で、扉が開いても動かない点の 2 次元の地図。近い点と法線を格子で引く"""

    def __init__(self, xy: np.ndarray, cell: float = MAP_CELL_M, reach: float = MAP_REACH_M) -> None:
        key = np.floor(xy / cell).astype(np.int64)
        _, first = np.unique(key[:, 0] * 1_000_003 + key[:, 1], return_index=True)
        pts = xy[np.sort(first)]  # 格子ごとに 1 点へ間引く
        self.points = pts
        self.normals = self._normals(pts)
        self.cell = cell
        lo = pts.min(0) - reach
        hi = pts.max(0) + reach
        self.origin = lo
        shape = np.ceil((hi - lo) / cell).astype(int) + 1
        self.shape = (int(shape[0]), int(shape[1]))
        self.nearest = np.full(self.shape, -1, dtype=np.int32)
        self._fill(reach)

    @staticmethod
    def _normals(pts: np.ndarray) -> np.ndarray:
        out = np.zeros_like(pts)
        for i0 in range(0, len(pts), 512):
            blk = pts[i0:i0 + 512]
            d2 = ((blk[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
            for j, row in enumerate(d2):
                nb = pts[row < NORMAL_RADIUS_M ** 2]
                if len(nb) < 3:
                    out[i0 + j] = (np.nan, np.nan)
                    continue
                c = nb - nb.mean(0)
                out[i0 + j] = np.linalg.svd(c, full_matrices=False)[2][-1]
        return out

    def _fill(self, reach: float) -> None:
        r = int(math.ceil(reach / self.cell))
        occ = np.zeros(self.shape, dtype=bool)
        ij = np.floor((self.points - self.origin) / self.cell).astype(int)
        occ[ij[:, 0], ij[:, 1]] = True
        near = occ.copy()
        for di in range(-r, r + 1):  # 地図の点の近くの格子だけ候補にする
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    near |= np.roll(np.roll(occ, di, 0), dj, 1)
        cells = np.argwhere(near)
        centers = self.origin + (cells + 0.5) * self.cell
        best = np.empty(len(cells), dtype=np.int32)
        for i0 in range(0, len(cells), 1024):  # 1024 × 地図の点 3000 で約 50 MB
            d2 = ((centers[i0:i0 + 1024, None, :] - self.points[None, :, :]) ** 2).sum(-1)
            k = d2.argmin(1)
            ok = d2[np.arange(len(k)), k] <= reach * reach
            best[i0:i0 + 1024] = np.where(ok, k, -1)
        self.nearest[cells[:, 0], cells[:, 1]] = best

    def lookup(self, xy: np.ndarray) -> np.ndarray:
        """点ごとの近い地図の点の番号（無ければ −1）"""
        ij = np.floor((xy - self.origin) / self.cell).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] < self.shape[1])
        out = np.full(len(xy), -1, dtype=np.int32)
        out[inside] = self.nearest[ij[inside, 0], ij[inside, 1]]
        return out


@dataclass(frozen=True)
class MatchResult:
    pose: Pose2D  # 扉の座標でのライダー
    matched: int
    rms_m: float
    min_eig: float  # 正規方程式の最小の固有値（小さいと壁に沿う向きが決まらない）
    ok: bool


def match_scan(smap: StaticMap, xy_lidar: np.ndarray, init: Pose2D, door: Optional[DoorFrame] = None) -> MatchResult:
    """水平にしたライダーの座標の点 [N, 2] を地図に重ね、扉の座標でのライダーの姿勢を返す（点から線の ICP・Huber）"""
    pose = init
    matched, rms, min_eig = 0, float("nan"), 0.0
    for it in range(ICP_ITERS):
        gate = ICP_GATE_M[0] + (ICP_GATE_M[1] - ICP_GATE_M[0]) * it / max(1, ICP_ITERS - 1)
        p = pose.apply(xy_lidar)
        keep = np.ones(len(p), dtype=bool) if door is None else ~in_swing(p, door)
        idx = smap.lookup(p)
        keep &= idx >= 0
        nrm = smap.normals[np.maximum(idx, 0)]
        keep &= np.isfinite(nrm).all(1)
        m = smap.points[np.maximum(idx, 0)]
        r = ((p - m) * nrm).sum(1)
        keep &= np.abs(r) < gate
        matched = int(keep.sum())
        if matched < MIN_MATCHED:
            return MatchResult(pose, matched, float("nan"), 0.0, False)
        p, nrm, r = p[keep], nrm[keep], r[keep]
        rel = p - np.array([pose.x, pose.y])  # 回転は今の姿勢の位置のまわり
        J = np.c_[nrm, nrm[:, 1] * rel[:, 0] - nrm[:, 0] * rel[:, 1]]
        wgt = np.minimum(1.0, ICP_HUBER_M / np.maximum(np.abs(r), 1e-9))
        H = (J * wgt[:, None]).T @ J
        g = (J * wgt[:, None]).T @ r
        delta = -np.linalg.solve(H + 1e-9 * np.eye(3), g)
        pose = Pose2D(pose.x + delta[0], pose.y + delta[1], wrap(pose.yaw + delta[2]))  # 回転は今の位置のまわりなので位置はそのまま足す
        rms = float(np.sqrt(np.mean(r ** 2)))
        min_eig = float(np.linalg.eigvalsh(H / matched)[0])
        if abs(delta[0]) < 1e-4 and abs(delta[1]) < 1e-4 and abs(delta[2]) < 1e-4:
            break
    return MatchResult(pose, matched, rms, min_eig, True)


def door_angle_in_door(q_door: np.ndarray, door: DoorFrame) -> Tuple[float, int]:
    """扉の座標の点 [N, 3] から扉の角度 [deg]（閉じて 0・押し開けて正）と使った点の数。
    当て方は door_lidar_angle.door_angle（蝶番のまわりの点の角度の山 → 主成分の直線）。ただし歩いて斜めから見ると、
    板でない点の塊（自分の腕・柱・部屋の中）が山になって −15〜+20° と読んだ（10-21 の 04・08b、扉は閉じていた）ので、
    線が蝶番の近くを通らない・線に沿った点の広がりが短い回は「測れない」にする"""
    ang, n, miss, spread = door_angle_detail(q_door, door)
    if not math.isfinite(ang):
        return ang, n
    limit = ANGLE_HINGE_MISS_M + ANGLE_HINGE_MISS_OPEN_M * (1.0 - math.cos(math.radians(max(0.0, ang))))
    if miss > limit or spread < ANGLE_MIN_SPREAD_M:
        return float("nan"), n
    return ang, n


def door_angle_detail(q_door: np.ndarray, door: DoorFrame) -> Tuple[float, int, float, float]:
    """door_lidar_angle.door_angle と同じ点の選び方・当て方で、角度 [deg]・点の数・蝶番から線までの距離 [m]・線に沿った広がり [m]"""
    band = (q_door[:, 2] > ANGLE_Z[0]) & (q_door[:, 2] < ANGLE_Z[1])
    u = q_door[band, 1] * door.lever_side  # 蝶番から縁へ向かう向きが +（把手が左なら +y）
    v = door.face_x - q_door[band, 0]  # 板の面から機体の側が正（door_lidar_angle と同じ）
    du, dv = u + door.width / 2.0, -v  # 蝶番から
    r = np.hypot(du, dv)
    phi = np.degrees(np.arctan2(dv, du))
    m = (r > dla.R_MIN) & (r < dla.R_MAX) & (phi > -8.0) & (phi < 115.0) & (dv > -0.03)
    nan = float("nan")
    if m.sum() < dla.MIN_POINTS:
        return nan, int(m.sum()), nan, nan
    hist, edges = np.histogram(phi[m], bins=np.arange(-8.0, 116.0, dla.BIN_DEG))
    peak = edges[hist.argmax()] + dla.BIN_DEG / 2
    k = m & (np.abs(phi - peak) < dla.PEAK_HALF_DEG)
    if k.sum() < dla.MIN_POINTS:
        return nan, int(k.sum()), nan, nan
    pts = np.stack([du[k], dv[k]], 1)
    c = pts.mean(0)
    d = np.linalg.svd(pts - c, full_matrices=False)[2][0]
    if d[0] < 0:
        d = -d
    miss = abs(float(c[0] * d[1] - c[1] * d[0]))  # 蝶番（原点）から線までの距離
    along = (pts - c) @ d
    spread = float(np.percentile(along, 95) - np.percentile(along, 5))
    return math.degrees(math.atan2(d[1], d[0])), int(k.sum()), miss, spread


def band_xy(q: np.ndarray) -> np.ndarray:
    return q[(q[:, 2] > BAND_Z[0]) & (q[:, 2] < BAND_Z[1]), :2]


@dataclass(frozen=True)
class ScanEstimate:
    lidar: Pose2D  # 扉の座標でのライダー（頭）の床の上の姿勢（捨てた回は予測の値）
    accepted: bool  # 重ね直しが成功し、予測からの跳びも上限の内だった
    lost: bool  # LOST_S 続けて捨てた（姿勢は当てにならない）
    angle_deg: float  # nan なら測れない
    angle_points: int
    match: MatchResult
    tilt_deg: float  # ライダーの傾き（頭の休む角度の見張り用）
    lidar_height_m: float


def relocalize(smap: StaticMap, xy_lidar: np.ndarray, door: DoorFrame,
               xs: Sequence[float] = tuple(np.arange(-2.5, 0.01, 0.1)), ys: Sequence[float] = tuple(np.arange(-2.0, 2.01, 0.1)),
               yaws_deg: Sequence[float] = tuple(range(-180, 180, 5)), tol: float = 0.05) -> MatchResult:
    """初期値の無いとき（記録で比べるとき・別の場所から始まった記録）: 位置と向きを粗く全部試し、当たる点の多い所から ICP"""
    sub = xy_lidar[:: max(1, len(xy_lidar) // 600)]
    best, best_n = None, -1
    for yd in yaws_deg:
        c, s = math.cos(math.radians(yd)), math.sin(math.radians(yd))
        rot = sub @ np.array([[c, s], [-s, c]])
        for x in xs:
            for y in ys:
                p = rot + np.array([x, y])
                idx = smap.lookup(p)
                ok = idx >= 0
                d = np.abs(((p[ok] - smap.points[idx[ok]]) * smap.normals[idx[ok]]).sum(1))
                n = int(np.sum(d < tol))
                if n > best_n:
                    best, best_n = Pose2D(float(x), float(y), math.radians(yd)), n
    return match_scan(smap, xy_lidar, best, door)


class DoorTracker:
    """止まって測った扉を基準に、1 回の測定ごとに扉の座標でのライダーの姿勢と扉の角度を出す。
    実機: DoorTracker.from_still（STAND の間の回から）。記録で比べる: 別の記録で作った扉と地図を渡し、init で最初の姿勢を与える"""

    def __init__(self, door: DoorFrame, smap: StaticMap, level: Level, init: Pose2D) -> None:
        self.door = door
        self.map = smap
        self.level = level
        self.init = init
        self.prev: List[Tuple[float, Pose2D]] = []  # 受け入れた回（時刻, 姿勢）の最後の 2 つ
        self.last_angle: Optional[Tuple[float, float]] = None  # (時刻, 角度 [deg])
        self.last_accept_t: Optional[float] = None
        self.rejected = 0

    @classmethod
    def from_still(cls, still_scans: Sequence[np.ndarray], thickness_m: float, lever_side: int = +1) -> "DoorTracker":
        scans = [valid_points(s) for s in still_scans]
        level = level_from_scans(scans)
        lev = [level.apply(s) for s in scans]
        door = door_frame_from_still(lev, thickness_m, lever_side)
        to_door = door.pose.inverse()  # 水平にした座標 → 扉の座標
        q = np.concatenate(lev)
        closed, _ = door_angle_in_door(np.c_[to_door.apply(q[:, :2]), q[:, 2]], door)
        if math.isfinite(closed) and abs(closed) > CLOSED_MAX_DEG:
            raise ValueError(f"止まって測った時に扉が閉じていない（{closed:+.1f}°）")
        xy = to_door.apply(band_xy(q))
        return cls(door, StaticMap(xy[~in_swing(xy, door)]), level, door.lidar_in_door)

    def predict(self, t: float) -> Pose2D:
        """前の 2 回からの等速の予測（無ければ最初の姿勢）"""
        if not self.prev:
            return self.init
        if len(self.prev) == 1 or t <= self.prev[-1][0]:
            return self.prev[-1][1]
        (t0, p0), (t1, p1) = self.prev[-2], self.prev[-1]
        dt = t1 - t0
        if dt <= 1e-3 or dt > 0.5 or t - t1 > 0.5:
            return p1
        d = p0.inverse().compose(p1)
        k = (t - t1) / dt
        speed, rate = math.hypot(d.x, d.y) / dt, abs(d.yaw) / dt
        if speed > MAX_SPEED_MPS or rate > MAX_YAW_RATE:
            return p1  # 速すぎる差は外れの名残。伸ばさない
        return p1.compose(Pose2D(d.x * k, d.y * k, d.yaw * k))

    def _plausible_speed(self, t: float, pose: Pose2D) -> bool:
        """最後に受け入れた姿勢からの速さが上限の内か（少しずつ滑る外れを、積み重なる前に止める）"""
        if not self.prev or t <= self.prev[-1][0]:
            return True
        t1, p1 = self.prev[-1]
        dt = t - t1
        if dt > 1.0:
            return True  # 間が空いたら速さでは判定しない（跳びの上限と当てはまりで見る）
        d = p1.inverse().compose(pose)
        return math.hypot(d.x, d.y) / dt <= MAX_SPEED_MPS and abs(d.yaw) / dt <= MAX_YAW_RATE

    def step(self, t: float, scan: np.ndarray, init: Optional[Pose2D] = None) -> ScanEstimate:
        p = valid_points(scan)
        lev = relevel(p, self.level)
        if lev is not None:
            self.level = lev
        q = self.level.apply(p)
        pred = init or self.predict(t)
        res = match_scan(self.map, band_xy(q), pred, self.door)
        jump = pred.inverse().compose(res.pose)
        accepted = (res.ok and math.hypot(jump.x, jump.y) <= JUMP_POS_M and abs(math.degrees(jump.yaw)) <= JUMP_YAW_DEG
                    and res.rms_m <= ACCEPT_MAX_RMS_M and res.min_eig >= ACCEPT_MIN_EIG and res.matched >= ACCEPT_MIN_MATCHED
                    and self._plausible_speed(t, res.pose))
        if accepted:
            self.prev = (self.prev + [(t, res.pose)])[-2:]
            self.last_accept_t, self.rejected = t, 0
            pose = res.pose
        else:
            self.rejected += 1
            pose = pred
        lost = not accepted and (self.last_accept_t is None or t - self.last_accept_t > LOST_S)
        xy = pose.apply(q[:, :2])
        q_door = np.c_[xy, q[:, 2]]
        angle, n = door_angle_in_door(q_door, self.door) if accepted else (float("nan"), 0)
        if math.isfinite(angle):
            last = self.last_angle
            if last is not None and t - last[0] < ANGLE_JUMP_RESET_S and abs(angle - last[1]) > ANGLE_MAX_JUMP_DEG:
                angle = float("nan")  # 1 回だけの跳び
            else:
                self.last_angle = (t, angle)
        return ScanEstimate(pose, accepted, lost, angle, n, res, self.level.tilt_deg(), self.level.h)
