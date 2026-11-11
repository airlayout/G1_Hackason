"""11-11 T3（計画 docs/plan/2026-11-11-g1-door-stage2-decision.md の R5・stage2-fix.md 9.4 の K1）: 段 0 の止まっている点群で、
機体の思う扉の座標（REF の姿勢）を直す。numpy だけ（PC2 の door_fastlio_node.py から呼ぶ。Python 3.8 でも動く書き方）。

入力は、REF の姿勢で扉の座標に置いた段 0 の点（points_in_door。render_staged_views_1111.lidar_points_door と同じ置き方）。
  1. 前後と向き: 閉じた扉の板（廊下側の面は x = BOARD_X）の点に直線 x = a + b·y を当てる（geom_latch_side_1111.board_line の形。
     1 回目の窓を ±15 cm に広げ、±2 cm での当て直しを 2 回にした）。本当の向き = 思っている向き + atan(b)
  2. 横: 板の直線で回してそろえた後、手の高さ（0.85〜1.15 m）の点の x の中央値を y に沿って出し、板（−0.02）から枠（約 −0.05）へ
     段が付く所（x の中央値が EDGE_X = −0.035 を外へ向かって横切る y）を両側で読む（k1_frame_edges_1111.edge と同じ）。
     オフラインの 2 cm の区間の代わりに、2 cm の窓を 5 mm ずつずらす（中央値が切り替わるのは窓の中心が縁に来た所なので、区間の
     切れ目による ±5 mm の段が消える）。両側が読めたら中点、片側だけなら開口の幅（点群で測った 0.960 m。巻き尺の板の 0.94 ではない）
     の半分で出す
  3. 直し G（思っている扉の座標 → 本当の扉の座標の平面の剛体変換 p_本当 = R(dyaw)·p_思う + (dx, dy)）を上限で切って返す。
     REF の姿勢（扉の座標でのライダーの床の上の姿勢）は G ⊕ REF（apply_fix）
把手が右の扉は y を裏返して同じ計算をし、結果を裏返して戻す。
扉の座標は door_frame.py と同じ（原点＝開口の中心の床・+x＝押して入る向き・+y＝左・把手が左なら蝶番は −y）。
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_door_est as est  # noqa: E402

Pose = Tuple[float, float, float]

BOARD_X = -0.02             # 閉じた板の廊下側の面（扉の座標・厚み 4 cm の半分）
EDGE_X = -0.035             # 板（−0.02）と枠（巻き尺で約 −0.05）の中ほど。開口の縁はここを横切る所
OPENING_W = 0.960           # 開口の幅（K1: 11-11 の点群で両側の縁が読めた 3 回の平均。口の式で REF の後の 25 回に当てると
                            # 両側が読めた 9 回で 0.939〜0.960・中央 0.958）
Z_BOARD = (0.3, 1.6)        # 板の直線を当てる高さ（床から）
BOARD_GATE_M = 0.15         # 板の直線の 1 回目: 思っている板の面から ±15 cm の点（オフラインは ±12 cm。上限 8 cm・3° のずれで
                            # 板の端は 10 cm 動き、±12 cm では雑音の片側が切れて向きが 0.3° 小さく出た）
BOARD_INLIER_M = 0.02       # 2 回目から: 前の直線から ±2 cm の点で当て直す
BOARD_REFITS = 2            # ±2 cm での当て直しの回数（オフラインは 1 回。2 回目で 1 回目の外れの引きずりを消す）
BOARD_MIN_POINTS = 100      # 2 回目の点の最小（オフラインは 12 回で 50。口は 25 回）
BOARD_MIN_SPREAD_M = 0.25   # 2 回目の点の y の広がり（5〜95 パーセンタイル）の最小。狭いと傾きが決まらない
BOARD_HINGE_MARGIN_M = 0.08  # 板の直線を当てる y の範囲: 蝶番の側の縁から 8 cm 内（幅 0.96 で −0.40）
BOARD_LEVER_MARGIN_M = 0.23  # 把手の側の縁から 23 cm 内（レバーを避ける。幅 0.96 で +0.25）
Z_HAND = (0.85, 1.15)       # 縁を読む高さ（手の高さ）
PROFILE_X = (-0.35, 0.6)    # 縁を読む点の前後の範囲（そろえた後）
PROFILE_WIN_M = 0.02        # x の中央値を取る窓の幅（オフラインの区間と同じ 2 cm）
PROFILE_STEP_M = 0.005      # 窓をずらす幅
PROFILE_MIN_POINTS = 8      # 窓の点の最小（これより少ない窓は中央値を出さない）
EDGE_MIN_POINTS = 20        # 縁を読むのに使う窓の点の最小（k1_frame_edges_1111 の MIN_POINTS）
EDGE_GAP_M = 0.03           # 使える窓の間がこれより空いたら、その側の縁は読まない（抜けがある）
EDGE_WIN_IN_M = 0.12        # 縁を探す範囲: |y| が 幅/2 − 12 cm 〜 幅/2 + 14 cm（幅 0.96 で 0.36〜0.62）
EDGE_WIN_OUT_M = 0.14
WIDTH_TOL_M = 0.03          # 両側の縁の間が開口の幅からこれより違えば、どちらかの縁の読み違いとして横は直さない
LATERAL_REFIT_M = 0.005     # 横の外れがこれより大きければ、板の直線の範囲をその分ずらして当て直す（1 回だけ）
LIMIT_X_M = 0.08            # 直しの上限（前後・向き・横）。越えたら上限で切ってログに残す
LIMIT_YAW_DEG = 3.0
LIMIT_Y_M = 0.10
SHIFT_AT_Y = -0.08          # 前後のそろえ量を読む所（オフラインの表と同じ・扉の中央寄り。把手が右なら +0.08）
CROP_X = (-0.35, 0.6)       # points_in_door で残す範囲（板の直線と縁の読みに使う所だけ）
CROP_Y_M = 1.2

FIX_COLS = ("ok", "lateral_ok", "dx", "dy", "dyaw", "shift_m", "yaw_err_deg", "lateral_m", "hinge_y", "latch_y", "width_m",
            "board_points", "board_rms_m", "board_spread_m")


@dataclass(frozen=True)
class CloudFix:
    """段 0 の点群での直し。dx・dy・dyaw は思っている扉の座標 → 本当の扉の座標（p_本当 = R(dyaw)·p_思う + (dx, dy)）。
    ok が False なら直さない（why に理由）。lateral_ok が False なら横（dy）は 0 のまま（前後と向きだけ直す）"""

    ok: bool
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    lateral_ok: bool = False
    shift_m: float = math.nan      # 前後のそろえ（y = SHIFT_AT_Y で BOARD_X − 板の直線。正＝機体は扉を遠くに見ていた）
    yaw_err_deg: float = math.nan  # 向きの誤り atan(b)（本当の向き = 思っている向き + これ）
    lateral_m: float = math.nan    # そろえた後の思っている扉の座標での開口の中心の y（オフラインの「横の外れ」。dy = −これ）
    hinge_y: float = math.nan      # 読めた縁（そろえた後の y）。読めなければ nan
    latch_y: float = math.nan
    width_m: float = math.nan      # 両側が読めた時の縁の間
    board_points: int = 0
    board_rms_m: float = math.nan
    board_spread_m: float = math.nan
    clamped: str = ""              # 上限で切った時の中身（空なら切っていない）
    why: str = ""                  # 直さなかった・一部だけ直した理由

    def row(self) -> Tuple[float, ...]:
        """ログの 1 行（FIX_COLS の順）"""
        return (float(self.ok), float(self.lateral_ok), self.dx, self.dy, self.dyaw, self.shift_m, self.yaw_err_deg, self.lateral_m,
                self.hinge_y, self.latch_y, self.width_m, float(self.board_points), self.board_rms_m, self.board_spread_m)

    def summary(self) -> str:
        """ログの 1 行の文（cm・°）"""
        if not self.ok:
            return f"直さない（{self.why}）"
        note = f"・{self.why}" if self.why else ""
        lat = (f"横 {100 * self.lateral_m:+.1f} cm（縁 蝶番 {_cm(self.hinge_y)}／把手 {_cm(self.latch_y)}・幅 {_cm(self.width_m)}{note}）"
               if self.lateral_ok else f"横は直さない（{self.why}）")
        out = (f"前後 {100 * self.shift_m:+.1f} cm・向き {self.yaw_err_deg:+.2f}°・{lat}・板 {self.board_points} 点"
               f"（残差 {1000 * self.board_rms_m:.1f} mm・広がり {self.board_spread_m:.2f} m）")
        return out + (f"・⚠️ 上限で切った: {self.clamped}" if self.clamped else "")


def _cm(v: float) -> str:
    return "—" if not math.isfinite(v) else f"{v:+.3f}"


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ── 点を扉の座標に置く ─────────────────────────────────────────
def points_in_door(scans: Sequence[np.ndarray], level: est.Level, pose: Pose) -> np.ndarray:
    """段 0 の回（ライダーの座標の点）を床で水平にし、REF の姿勢（扉の座標でのライダーの床の上の姿勢）で扉の座標に置く [N, 3]。
    z は床からの高さ。直しに使う所（扉の周り）だけ残す（25 回ぶんを全部持たない）"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rot = np.array([[c, s], [-s, c]])
    out = []
    for xyz in scans:
        q = level.apply(est.valid_points(np.asarray(xyz, dtype=float)))
        xy = q[:, :2] @ rot + np.array([pose[0], pose[1]])
        keep = ((xy[:, 0] > CROP_X[0]) & (xy[:, 0] < CROP_X[1]) & (np.abs(xy[:, 1]) < CROP_Y_M)
                & (q[:, 2] > Z_BOARD[0]) & (q[:, 2] < Z_BOARD[1]))
        out.append(np.column_stack([xy[keep], q[keep, 2]]))
    return np.concatenate(out) if out else np.zeros((0, 3))


# ── 板の直線 ─────────────────────────────────────────────────
@dataclass(frozen=True)
class BoardLine:
    a: float
    b: float
    points: int
    rms_m: float
    spread_m: float


def fit_board_line(P: np.ndarray, y_range: Tuple[float, float]) -> Tuple[Optional[BoardLine], str]:
    """閉じた扉の板の点に直線 x = a + b·y を当てる（1 回目は思っている面の ±15 cm・その後は前の直線の ±2 cm で BOARD_REFITS 回）。
    当てられなければ (None, 理由)"""
    m = ((P[:, 1] > y_range[0]) & (P[:, 1] < y_range[1]) & (P[:, 2] > Z_BOARD[0]) & (P[:, 2] < Z_BOARD[1])
         & (np.abs(P[:, 0] - BOARD_X) < BOARD_GATE_M))
    n1 = int(m.sum())
    if n1 < BOARD_MIN_POINTS:
        return None, f"板の点が足りない（{n1} 点・{BOARD_MIN_POINTS} 要る）"
    x, y = P[m, 0], P[m, 1]
    A = np.column_stack([np.ones(n1), y])
    c = np.linalg.lstsq(A, x, rcond=None)[0]
    for _ in range(BOARD_REFITS):
        keep = np.abs(x - A @ c) < BOARD_INLIER_M
        n2 = int(keep.sum())
        if n2 < BOARD_MIN_POINTS:
            return None, f"板の直線の ±{100 * BOARD_INLIER_M:.0f} cm の点が足りない（{n2} 点・{BOARD_MIN_POINTS} 要る）"
        c = np.linalg.lstsq(A[keep], x[keep], rcond=None)[0]
    lo, hi = np.percentile(y[keep], [5.0, 95.0])
    spread = float(hi - lo)
    if spread < BOARD_MIN_SPREAD_M:
        return None, f"板の点の広がりが狭い（{spread:.2f} m・{BOARD_MIN_SPREAD_M} m 要る。腕などで隠れた疑い）"
    rms = float(np.sqrt(np.mean((x[keep] - A[keep] @ c) ** 2)))
    return BoardLine(float(c[0]), float(c[1]), n2, rms, spread), ""


def align_points(P: np.ndarray, dx: float, dyaw: float) -> np.ndarray:
    """思っている扉の座標の点を、原点のまわりに dyaw 回して x に dx 足す（横はまだ直さない）"""
    c, s = math.cos(dyaw), math.sin(dyaw)
    out = P.copy()
    out[:, 0] = c * P[:, 0] - s * P[:, 1] + dx
    out[:, 1] = s * P[:, 0] + c * P[:, 1]
    return out


# ── 開口の縁 ─────────────────────────────────────────────────
def profile(Q: np.ndarray, y_max: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """手の高さの点の x の中央値を、y に沿って PROFILE_WIN_M の窓を PROFILE_STEP_M ずつずらして出す → (窓の中心 y, 点の数, x の中央値)。
    点が PROFILE_MIN_POINTS より少ない窓の中央値は nan"""
    m = (Q[:, 2] > Z_HAND[0]) & (Q[:, 2] < Z_HAND[1]) & (Q[:, 0] > PROFILE_X[0]) & (Q[:, 0] < PROFILE_X[1])
    order = np.argsort(Q[m, 1])
    y, x = Q[m, 1][order], Q[m, 0][order]
    k = int(math.ceil(y_max / PROFILE_STEP_M))
    centers = np.arange(-k, k + 1) * PROFILE_STEP_M
    lo = np.searchsorted(y, centers - PROFILE_WIN_M / 2.0, side="left")
    hi = np.searchsorted(y, centers + PROFILE_WIN_M / 2.0, side="left")
    n = hi - lo
    xmed = np.array([float(np.median(x[i:j])) if j - i >= PROFILE_MIN_POINTS else math.nan for i, j in zip(lo, hi)])
    return centers, n, xmed


def find_edge(centers: np.ndarray, n: np.ndarray, xmed: np.ndarray, side: int, half: float) -> Optional[float]:
    """片側（side = −1 は蝶番の側の −y・+1 は把手の側の +y）の開口の縁の y。板の側（中央寄り）から外へ向かって、
    x の中央値が EDGE_X より手前になる最初の所を隣り合う窓の直線補間で出す。読めなければ None"""
    u = side * centers  # 外へ向かって増える
    sel = (n >= EDGE_MIN_POINTS) & np.isfinite(xmed) & (u >= half - EDGE_WIN_IN_M) & (u <= half + EDGE_WIN_OUT_M)
    us, xs = u[sel], xmed[sel]
    order = np.argsort(us)
    us, xs = us[order], xs[order]
    for i in range(len(us) - 1):
        if us[i + 1] - us[i] > EDGE_GAP_M + 1e-9:
            return None  # 抜けがある所は読まない
        if xs[i] > EDGE_X >= xs[i + 1]:
            w = (xs[i] - EDGE_X) / (xs[i] - xs[i + 1])
            return float(side * (us[i] + w * (us[i + 1] - us[i])))
    return None


def lateral_from_edges(hinge: Optional[float], latch: Optional[float], width: float) -> Tuple[float, str]:
    """開口の縁から、そろえた後の思っている扉の座標での開口の中心の y。出せなければ (nan, 理由)。
    片側だけの時も (値, 理由) を返す（理由は記録用）"""
    if hinge is not None and latch is not None:
        w = latch - hinge
        if abs(w - width) > WIDTH_TOL_M:
            return math.nan, f"両側の縁の間 {w:.3f} m が開口の幅 {width:.3f} ± {100 * WIDTH_TOL_M:.0f} cm の外（縁の読み違いの疑い）"
        return (hinge + latch) / 2.0, ""
    if hinge is not None:
        return hinge + width / 2.0, "把手の側の縁が読めない（蝶番の側と開口の幅から）"
    if latch is not None:
        return latch - width / 2.0, "蝶番の側の縁が読めない（把手の側と開口の幅から）"
    return math.nan, "開口の縁が両側とも読めない"


# ── 直し ─────────────────────────────────────────────────────
def _clip(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


@dataclass(frozen=True)
class _Pass:
    """1 回の当て（板の直線 → 回してそろえる → 縁）"""

    line: BoardLine
    dx: float
    dyaw: float
    hinge: Optional[float]
    latch: Optional[float]
    lateral: float
    why_lat: str


def _one_pass(P: np.ndarray, width: float, y_range: Tuple[float, float]) -> Tuple[Optional[_Pass], str]:
    half = width / 2.0
    line, why = fit_board_line(P, y_range)
    if line is None:
        return None, why
    dyaw = math.atan(line.b)
    dx = BOARD_X - line.a * math.cos(dyaw)  # 原点のまわりに回すと板の直線は x = a·cos(dyaw) に来る
    Q = align_points(P, dx, dyaw)
    centers, n, xmed = profile(Q, half + EDGE_WIN_OUT_M + PROFILE_WIN_M)
    hinge = find_edge(centers, n, xmed, -1, half)
    latch = find_edge(centers, n, xmed, +1, half)
    lateral, why_lat = lateral_from_edges(hinge, latch, width)
    return _Pass(line, dx, dyaw, hinge, latch, lateral, why_lat), ""


def _measure_left(P: np.ndarray, width: float) -> CloudFix:
    """把手が左（蝶番は −y）の扉の向きの点で直しを出す"""
    half = width / 2.0
    y_range = (-(half - BOARD_HINGE_MARGIN_M), half - BOARD_LEVER_MARGIN_M)
    r, why = _one_pass(P, width, y_range)
    if r is None:
        return CloudFix(False, why=why)
    # 横が外れていたら、板の直線を当てる範囲を本当の扉の板の上にずらして 1 回だけ当て直す（レバー・枠を範囲に入れない）。
    # 当て直しで直線か横が出せなくなったら 1 回目を使う
    if math.isfinite(r.lateral) and abs(r.lateral) >= LATERAL_REFIT_M:
        r2, _ = _one_pass(P, width, (y_range[0] + r.lateral, y_range[1] + r.lateral))
        if r2 is not None and math.isfinite(r2.lateral):
            r = r2
    line, dx, dyaw, hinge, latch, lateral, why_lat = r.line, r.dx, r.dyaw, r.hinge, r.latch, r.lateral, r.why_lat
    lat_ok = math.isfinite(lateral)
    dy = -lateral if lat_ok else 0.0
    clamped = []
    if abs(dx) > LIMIT_X_M:
        clamped.append(f"前後 {100 * dx:+.1f}→{100 * _clip(dx, LIMIT_X_M):+.1f} cm")
    if abs(math.degrees(dyaw)) > LIMIT_YAW_DEG:
        clamped.append(f"向き {math.degrees(dyaw):+.2f}→{_clip(math.degrees(dyaw), LIMIT_YAW_DEG):+.2f}°")
    if abs(dy) > LIMIT_Y_M:
        clamped.append(f"横 {100 * dy:+.1f}→{100 * _clip(dy, LIMIT_Y_M):+.1f} cm")
    return CloudFix(True, dx=_clip(dx, LIMIT_X_M), dy=_clip(dy, LIMIT_Y_M), dyaw=math.radians(_clip(math.degrees(dyaw), LIMIT_YAW_DEG)),
                    lateral_ok=lat_ok, shift_m=BOARD_X - (line.a + line.b * SHIFT_AT_Y), yaw_err_deg=math.degrees(dyaw),
                    lateral_m=lateral, hinge_y=math.nan if hinge is None else hinge, latch_y=math.nan if latch is None else latch,
                    width_m=(latch - hinge) if hinge is not None and latch is not None else math.nan,
                    board_points=line.points, board_rms_m=line.rms_m, board_spread_m=line.spread_m,
                    clamped="・".join(clamped), why=why_lat)


def _mirror(fix: CloudFix) -> CloudFix:
    """y を裏返した計算の結果を元の向きへ（並進の y・回り・横の値の符号が変わる。縁は左右が入れ替わって符号が変わる）"""
    return replace(fix, dy=-fix.dy, dyaw=-fix.dyaw, yaw_err_deg=-fix.yaw_err_deg, lateral_m=-fix.lateral_m,
                   hinge_y=-fix.hinge_y, latch_y=-fix.latch_y)


def measure(P: np.ndarray, width: float = OPENING_W, lever_right: bool = False) -> CloudFix:
    """扉の座標（REF の姿勢で置いた）の段 0 の点 P [N, 3] から直しを出す。width は開口の幅（縁から縁）。
    lever_right なら把手が右（蝶番は +y）の扉"""
    P = np.asarray(P, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3:
        return CloudFix(False, why=f"点の形が違う {P.shape}")
    P = P[np.isfinite(P).all(1)]
    if not (0.5 < width < 1.5):
        return CloudFix(False, why=f"開口の幅 {width} m がおかしい")
    if lever_right:
        return _mirror(_measure_left(P * np.array([1.0, -1.0, 1.0]), width))
    return _measure_left(P, width)


def apply_fix(pose: Pose, fix: CloudFix) -> Pose:
    """REF の姿勢（思っている扉の座標でのライダーの床の上の姿勢）を本当の扉の座標へ（G ⊕ pose）。ok でなければそのまま"""
    if not fix.ok:
        return (float(pose[0]), float(pose[1]), float(pose[2]))
    c, s = math.cos(fix.dyaw), math.sin(fix.dyaw)
    x, y, yaw = (float(v) for v in pose)
    return (fix.dx + c * x - s * y, fix.dy + s * x + c * y, wrap(yaw + fix.dyaw))
