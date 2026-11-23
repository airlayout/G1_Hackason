"""2 台のスマホを並べたステレオの、平行化と時刻合わせの計算（画面・映像・較正の方法に依らない部分）。

- 平行化: 各カメラのレンズの値（K・D）と 2 台の位置関係（R・T）から、OpenCV の stereoRectify で
  R1・R2・P1・P2 を求める。R・T は「左の座標 → 右の座標」（X_r = R X_l + T）。
- 左右: DROID-SLAM のステレオは「右のカメラが左のカメラの +x にある」と決め打ち（基線 0.1）。
  右が右にあれば T[0] < 0。逆なら左右の役を入れ替える。
- 時刻: ブラウザから送る 2 台はシャッターがそろわない。「右は左より何秒遅れて OMEN に届くか」を測る。
  既定は動きで測る（2 台を固定したまま振ると、画の動きの大きさの時間変化が 2 台で同じ形になる。
  その相互相関の山）。点滅（publish/flash.html）で測る道も残す。

（較正そのものは scene_calib.py。市松模様は使わない — ユーザーの決定 11-17）
"""

from __future__ import annotations

import json
import pathlib
import time

import cv2
import numpy as np


def swap_sides(K1, D1, K2, D2, R, T):
    """左右の役を入れ替える。X_r = R X_l + T → X_l = R^T X_r - R^T T"""
    Rs = R.T
    return K2, D2, K1, D1, Rs, -Rs @ T


def rectify(K1, D1, K2, D2, R, T, size: tuple[int, int]) -> dict:
    """平行化して calib の辞書にする。右が左の +x に無ければ役を入れ替え swapped=True"""
    K1, K2 = np.asarray(K1, float), np.asarray(K2, float)
    D1, D2 = np.asarray(D1, float).reshape(-1), np.asarray(D2, float).reshape(-1)
    R, T = np.asarray(R, float), np.asarray(T, float).reshape(3)
    swapped = bool(T[0] > 0)
    if swapped:
        K1, D1, K2, D2, R, T = swap_sides(K1, D1, K2, D2, R, T)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    return {
        "size": [int(size[0]), int(size[1])],
        "K1": K1, "D1": D1, "K2": K2, "D2": D2, "R": R, "T": T,
        "R1": R1, "R2": R2, "P1": P1, "P2": P2, "Q": Q,
        "baseline_m": float(np.linalg.norm(T)),
        "swapped": swapped,
        "vertical_baseline": bool(abs(T[1]) > abs(T[0])),
    }


def rectify_points(calib: dict, pts: np.ndarray, side: str) -> np.ndarray:
    K, D, Rr, P = (
        (calib["K1"], calib["D1"], calib["R1"], calib["P1"])
        if side == "left"
        else (calib["K2"], calib["D2"], calib["R2"], calib["P2"])
    )
    return cv2.undistortPoints(
        np.asarray(pts, np.float64).reshape(-1, 1, 2), np.asarray(K), np.asarray(D), R=np.asarray(Rr), P=np.asarray(P)
    ).reshape(-1, 2)


def epipolar_dy(calib: dict, left_pts: np.ndarray, right_pts: np.ndarray) -> np.ndarray:
    """同じ点の、平行化した後の上下のずれ（画素）。0 に近いほど良い"""
    return np.abs(rectify_points(calib, left_pts, "left")[:, 1] - rectify_points(calib, right_pts, "right")[:, 1])


def rectify_maps(calib: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    size = tuple(calib["size"])
    m1 = cv2.initUndistortRectifyMap(
        np.asarray(calib["K1"]), np.asarray(calib["D1"]), np.asarray(calib["R1"]),
        np.asarray(calib["P1"])[:3, :3], size, cv2.CV_32FC1,
    )
    m2 = cv2.initUndistortRectifyMap(
        np.asarray(calib["K2"]), np.asarray(calib["D2"]), np.asarray(calib["R2"]),
        np.asarray(calib["P2"])[:3, :3], size, cv2.CV_32FC1,
    )
    return m1[0], m1[1], m2[0], m2[1]


def rectified_intrinsics(calib: dict) -> tuple[float, float, float, float]:
    """平行化した左のカメラの (fx, fy, cx, cy)"""
    P1 = np.asarray(calib["P1"])
    return float(P1[0, 0]), float(P1[1, 1]), float(P1[0, 2]), float(P1[1, 2])


def average_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    """回転の平均（四元数の外積の和の最大の固有ベクトル）"""
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(np.stack(Rs)).as_quat()
    q[q[:, 3] < 0] *= -1
    w, v = np.linalg.eigh(q.T @ q)
    return Rotation.from_quat(v[:, np.argmax(w)]).as_matrix()


def relative_pose_spread(Rs: list[np.ndarray], ts: list[np.ndarray], R0: np.ndarray, t0: np.ndarray) -> dict:
    """組ごとの「左→右」の姿勢の、平均からのばらつき（2 台が固定されていたかの目安）"""
    from scipy.spatial.transform import Rotation

    rot = np.array([np.degrees(Rotation.from_matrix(R0.T @ R).magnitude()) for R in Rs])
    d0 = t0 / np.linalg.norm(t0)
    dirs = np.array([np.degrees(np.arccos(np.clip((t / np.linalg.norm(t)) @ d0, -1, 1))) for t in ts])
    lens = np.array([np.linalg.norm(t) for t in ts]) / np.linalg.norm(t0)
    return {
        "rot_dev_deg": rot, "dir_dev_deg": dirs, "len_ratio": lens,
        "rot_spread_deg": float(np.median(rot)), "dir_spread_deg": float(np.median(dirs)),
        "len_spread": float(np.median(np.abs(lens - 1))),
    }


RIGID_ROT_DEG = 1.0  # 固定された 2 台なら、組ごとの向きのばらつき（中央値）はこれより小さい
RIGID_DIR_DEG = 3.0


def robust_relative_pose(Rs: list[np.ndarray], ts: list[np.ndarray], min_keep: int = 5):
    """組ごとの「左→右」の姿勢から外れを除いて平均する。(R, t, 使った数, ばらつき)

    外れの線は中央値からの散らばり（MAD）の 3 倍（下限 1 度・2 度・25%）。残りが少なくても全部の組には
    逃がさない（11-17 の実機で、逃がした先に登録の失敗した 1 組が混ざり、結果が壊れた）。
    """
    if not Rs:
        raise ValueError("左右がそろった組が無い")
    R0 = average_rotation(Rs)
    t0 = np.median(np.stack(ts), axis=0)
    sp = relative_pose_spread(Rs, ts, R0, t0)
    rot, dirs, lens = sp["rot_dev_deg"], sp["dir_dev_deg"], sp["len_ratio"]
    mad = lambda x: float(np.median(np.abs(x - np.median(x))))  # noqa: E731
    keep = (
        (rot <= max(1.0, np.median(rot) + 3 * mad(rot)))
        & (dirs <= max(2.0, np.median(dirs) + 3 * mad(dirs)))
        & (np.abs(lens - 1) <= max(0.25, 3 * mad(lens)))
    )
    idx = np.flatnonzero(keep)
    if len(idx) < min(min_keep, len(Rs)):
        raise ValueError(f"外れを除くと {len(idx)} 組しか残らない（2 台が固定されていない見込み）")
    R = average_rotation([Rs[i] for i in idx])
    t = np.mean(np.stack([ts[i] for i in idx]), axis=0)
    spread = relative_pose_spread([Rs[i] for i in idx], [ts[i] for i in idx], R, t)
    return R, t, int(len(idx)), spread


def _to_jsonable(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    return v


def save_calib(path: pathlib.Path, calib: dict, extra: dict | None = None) -> None:
    data = {k: _to_jsonable(v) for k, v in calib.items()}
    data.update(extra or {})
    data.setdefault("created", time.strftime("%Y-%m-%d %H:%M:%S"))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def load_calib(path: pathlib.Path) -> dict:
    data = json.loads(pathlib.Path(path).read_text())
    for k in ("K1", "D1", "K2", "D2", "R", "T", "R1", "R2", "P1", "P2", "Q"):
        data[k] = np.asarray(data[k], dtype=np.float64)
    return data


# ---- 時刻合わせ（白黒の点滅） ----


def transitions(times: np.ndarray, values: np.ndarray) -> list[tuple[float, bool]]:
    """明るさが中ほどを横切った時刻（前後のこまの間で直線に補う）と、明るくなったか"""
    times, values = np.asarray(times, float), np.asarray(values, float)
    if len(values) < 3:
        return []
    lo, hi = np.percentile(values, 10), np.percentile(values, 90)
    if hi - lo < 20:  # 点滅が写っていない
        return []
    mid = (lo + hi) / 2
    out = []
    for i in range(1, len(values)):
        a, b = values[i - 1] - mid, values[i] - mid
        if a == 0 or (a < 0) == (b < 0):
            continue
        f = a / (a - b)
        out.append((float(times[i - 1] + f * (times[i] - times[i - 1])), bool(b > 0)))
    return out


def estimate_offset(
    t_left: np.ndarray, v_left: np.ndarray, t_right: np.ndarray, v_right: np.ndarray, max_offset: float = 0.5
) -> dict:
    """右が左より何秒遅れて届くか。同じ向きの切り替わりのうち、いちばん近い物どうしを組にする"""
    tl, tr = transitions(t_left, v_left), transitions(t_right, v_right)
    diffs = []
    for t, rising in tl:
        cands = [u - t for u, r in tr if r == rising and abs(u - t) <= max_offset]
        if cands:
            diffs.append(min(cands, key=abs))
    if not diffs:
        return {"n_events": 0, "left_events": len(tl), "right_events": len(tr)}
    d = np.asarray(diffs)
    med = float(np.median(d))
    return {
        "offset_s": med,
        "jitter_std_s": float(d.std()),
        "jitter_mad_s": float(np.median(np.abs(d - med))),
        "n_events": int(len(d)),
        "left_events": len(tl),
        "right_events": len(tr),
    }


# ---- 時刻合わせ（2 台を一緒に振ったときの動き） ----


def estimate_offset_xcorr(
    t_left: np.ndarray, v_left: np.ndarray, t_right: np.ndarray, v_right: np.ndarray,
    max_offset: float = 0.5, step: float = 0.002,
) -> dict:
    """2 つの時系列（届いた時刻・動きの大きさ）の相互相関の山から、右が左より何秒遅れて届くか。

    同じ刻みに直線で補い、平均 0・分散 1 にそろえてから、ずらしを ±max_offset で探す。
    山の高さ（peak_corr）が低いときは、動きが足りないか 2 台が別の物を写している。
    """
    tl, vl = np.asarray(t_left, float), np.asarray(v_left, float)
    tr, vr = np.asarray(t_right, float), np.asarray(v_right, float)
    t0, t1 = max(tl[0], tr[0]) + max_offset, min(tl[-1], tr[-1]) - max_offset
    if t1 - t0 < 3.0:
        return {"ok": False, "why": "too short"}
    grid = np.arange(t0, t1, step)
    a = np.interp(grid, tl, vl)
    a = (a - a.mean()) / (a.std() + 1e-9)
    best = (-np.inf, 0.0)
    for lag in np.arange(-max_offset, max_offset + step / 2, step):
        b = np.interp(grid + lag, tr, vr)
        b = (b - b.mean()) / (b.std() + 1e-9)
        c = float(np.mean(a * b))
        if c > best[0]:
            best = (c, float(lag))
    return {"ok": best[0] > 0.5, "offset_s": best[1], "peak_corr": best[0], "secs": float(t1 - t0)}
