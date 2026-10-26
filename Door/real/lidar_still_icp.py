"""段 1（2026-10-21）: 止まっている区間ごとの本当の動きを、頭のライダーの点群の位置合わせ（ICP）で出し、脚の odometry（/dog_odom）と比べる。

1. 止まっている区間は take_quicklook.py と同じ決め方（odometry の速さ・リモコンに触れていない）
2. 区間ごとに、端を EDGE_S ずつ削ってライダーの回を重ねる（全周・半径 MIN_RANGE〜MAX_RANGE。近くは自分の体と見守りの人）
3. 最初の区間に対する各区間の姿勢を、点から面への ICP（粗い→細かいの 3 段）で出す。初期値は odometry
4. ライダーの姿勢を、取付（LIVOX_XYZ。dog_odom_to_tf.py と同じ値）で robot_center の姿勢に直し、
   床で水平にした面の中の 前・左・向き にして odometry と並べる

⚠️ 正解（ICP）の当てはまり（fitness・rmse）も出す。悪い区間は結果より先に計器を疑う。
⚠️ ライダーは robot_center の 11 cm 後ろにあるので、その場で 90° 回るとライダーは約 18 cm 動く。比べるのは robot_center どうし。

使い方: Navigation/.venv/bin/python Door/real/lidar_still_icp.py <ライダーのセッション> <odom のセッション> <remote.log> [--json 出力]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_angle as dla  # noqa: E402
from crop_lidar_bag import iter_lidar, scan_xyz  # noqa: E402
from dog_odom_bag import load_odom  # noqa: E402
from take_quicklook import body_frame, remote_events, still_segments  # noqa: E402

warnings.filterwarnings("ignore", category=RuntimeWarning)  # mac の BLAS が matmul で出す見かけだけの警告

# [m] /dog_odom の robot_center から見たライダーの水平の位置。2026-10-21 の 04（その場旋回 4 回）から逆に解いた値の平均
# （左回り 2 回と右回り 2 回で odometry の誤差の符号が逆になるので、平均で打ち消す。1 回ごとの値は ±2 cm でそろった）。
# ⚠️ dog_odom_to_tf.py の LIVOX_XYZ (-0.113, -0.057) は MOLA の base_link 基準（09-16）で、robot_center とは 16 cm 違う。
#    あの値で直すと、その場旋回で 22〜33 cm 並進したように見える（見かけ）
LIVOX_XY_DEFAULT = (0.047, -0.016)
LIVOX_XY = np.array(LIVOX_XY_DEFAULT)
EDGE_S = 0.4  # [s] 区間の端を削る
MIN_RANGE, MAX_RANGE = 0.6, 25.0  # [m]
VOXELS = (0.20, 0.10, 0.05)  # [m] ICP の 3 段
MAX_CORR = (0.60, 0.25, 0.10)  # [m]


def segment_clouds(li_session: Path, segs_abs: list[tuple[float, float]]) -> list[np.ndarray]:
    """区間ごとに、ライダーの座標のまま点を重ねる"""
    clouds: list[list[np.ndarray]] = [[] for _ in segs_abs]
    for recv_ns, _stamp, payload in iter_lidar(li_session):
        t = recv_ns / 1e9
        for k, (a, b) in enumerate(segs_abs):
            if a + EDGE_S <= t <= b - EDGE_S:
                xyz = scan_xyz(payload)
                r = np.linalg.norm(xyz, axis=1)
                clouds[k].append(xyz[(r > MIN_RANGE) & (r < MAX_RANGE)])
    return [np.concatenate(c) if c else np.zeros((0, 3)) for c in clouds]


def to_pcd(xyz: np.ndarray, voxel: float) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(voxel)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
    return pcd


def icp(src: np.ndarray, dst: np.ndarray, init: np.ndarray) -> tuple[np.ndarray, float, float]:
    """src を dst に重ねる変換（dst ← src）と、最後の段の fitness・rmse"""
    T = init
    res = None
    for voxel, corr in zip(VOXELS, MAX_CORR):
        res = o3d.pipelines.registration.registration_icp(
            to_pcd(src, voxel), to_pcd(dst, voxel), corr, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = res.transformation
    return T, float(res.fitness), float(res.inlier_rmse)


def planar(T: np.ndarray) -> tuple[float, float, float]:
    """床で水平にした座標の変換から (x, y, yaw[rad])"""
    return float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])


def lidar_to_center(x: float, y: float, yaw: float) -> np.ndarray:
    """最初の区間のライダーから見たライダーの動き → 最初の区間の robot_center から見た robot_center の動き（前・左・向き）"""
    c, s = math.cos(yaw), math.sin(yaw)
    # p_c(k) = p_l(k) − R(yaw)·o + o（最初の区間の robot_center 原点 = ライダー原点 − o）
    o = LIVOX_XY
    dx = x - (c * o[0] - s * o[1]) + o[0]
    dy = y - (s * o[0] + c * o[1]) + o[1]
    return np.array([dx, dy, yaw])


def center_to_lidar_init(f: float, l: float, yaw: float) -> tuple[float, float]:
    o = LIVOX_XY
    c, s = math.cos(yaw), math.sin(yaw)
    return f + (c * o[0] - s * o[1]) - o[0], l + (s * o[0] + c * o[1]) - o[1]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("li", type=Path)
    ap.add_argument("odom", type=Path)
    ap.add_argument("remote", type=Path)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--livox-xy", type=float, nargs=2, default=list(LIVOX_XY_DEFAULT), help="robot_center から見たライダー [m]")
    a = ap.parse_args()
    global LIVOX_XY
    LIVOX_XY = np.array(a.livox_xy)

    tr = load_odom(a.odom)
    t = tr.recv_s - tr.recv_s[0]
    fx, fy, yaw = body_frame(tr)
    segs = still_segments(tr, remote_events(a.remote, tr.recv_s[0]))
    if len(segs) < 2:
        raise SystemExit("[icp] 止まっている区間が 2 つ未満")
    segs_abs = [(sa + tr.recv_s[0], sb + tr.recv_s[0]) for sa, sb in segs]
    clouds = segment_clouds(a.li, segs_abs)

    # 床で水平にする変換（最初の区間で決め、全部の区間に同じものを使う。ライダーは機体に固定）
    R, h = dla.level([clouds[0][::3]])
    L = np.eye(4)
    L[:3, :3], L[2, 3] = R, h
    Linv = np.linalg.inv(L)
    lev = [c @ R.T + np.array([0.0, 0.0, h]) for c in clouds]

    odo = []
    for sa, sb in segs:
        m = (t >= sa + EDGE_S) & (t <= sb - EDGE_S)
        odo.append(np.array([fx[m].mean(), fy[m].mean(), math.radians(float(yaw[m].mean()))]))

    rows = []
    print(f"== {a.odom.name}: 区間 {len(segs)} 個・床で水平にしたライダーの高さ {h:.3f} m")
    print("  区間        odometry（前・左 m・向き°）        ライダー ICP（同）               差（odom − ICP）           fitness・rmse")
    for k in range(1, len(segs)):
        f0, l0, y0 = odo[0]
        fk, lk, yk = odo[k]
        # odometry を最初の区間の robot_center の座標に（body_frame は記録の最初の姿勢が基準なので、区間 0 の姿勢で直す）
        c0, s0 = math.cos(y0), math.sin(y0)
        df, dl = fk - f0, lk - l0
        o_rel = np.array([c0 * df + s0 * dl, -s0 * df + c0 * dl, yk - y0])
        ix, iy = center_to_lidar_init(*o_rel)
        init_lev = np.eye(4)
        cy, sy = math.cos(o_rel[2]), math.sin(o_rel[2])
        init_lev[:2, :2] = [[cy, -sy], [sy, cy]]
        init_lev[0, 3], init_lev[1, 3] = ix, iy
        # ICP は水平にした座標で: 区間 k の点（k の座標）を区間 0 の座標へ重ねる変換 = k の姿勢
        T, fit, rmse = icp(lev[k], lev[0], init_lev)
        lx, ly, lyaw = planar(T)
        truth = lidar_to_center(lx, ly, lyaw)
        diff = o_rel - truth
        rows.append({"区間": k, "時刻": [round(segs[k][0], 1), round(segs[k][1], 1)],
                     "odom": [round(float(v), 4) for v in o_rel], "icp": [round(float(v), 4) for v in truth],
                     "fitness": round(fit, 3), "rmse": round(rmse, 4),
                     "roll_pitch_deg": [round(math.degrees(math.atan2(T[2, 1], T[2, 2])), 2),
                                        round(math.degrees(-math.asin(max(-1.0, min(1.0, T[2, 0])))), 2)]})
        print(f"  0→{k}  {o_rel[0]:+.3f} {o_rel[1]:+.3f} {math.degrees(o_rel[2]):+7.2f}°   "
              f"{truth[0]:+.3f} {truth[1]:+.3f} {math.degrees(truth[2]):+7.2f}°   "
              f"{diff[0] * 100:+5.1f} {diff[1] * 100:+5.1f} cm {math.degrees(diff[2]):+5.2f}°   {fit:.2f}・{rmse * 100:.1f} cm")
    # 隣り合う区間どうしの動き（1 区間ぶんの距離・回転）でも並べる
    print("  隣どうし    odometry の距離・回転        ICP の距離・回転          距離の比（odom/ICP）")
    poses_o = [np.zeros(3)] + [np.array(r["odom"]) for r in rows]
    poses_t = [np.zeros(3)] + [np.array(r["icp"]) for r in rows]
    legs = []
    for k in range(1, len(poses_o)):
        do = np.hypot(*(poses_o[k][:2] - poses_o[k - 1][:2]))
        dt_ = np.hypot(*(poses_t[k][:2] - poses_t[k - 1][:2]))
        ro, rt = math.degrees(poses_o[k][2] - poses_o[k - 1][2]), math.degrees(poses_t[k][2] - poses_t[k - 1][2])
        ratio = do / dt_ if dt_ > 0.05 else float("nan")
        legs.append({"区間": [k - 1, k], "odom_m": round(float(do), 4), "icp_m": round(float(dt_), 4),
                     "odom_deg": round(ro, 2), "icp_deg": round(rt, 2), "比": round(ratio, 4) if ratio == ratio else None})
        print(f"  {k - 1}→{k}     {do:.3f} m {ro:+7.2f}°            {dt_:.3f} m {rt:+7.2f}°           "
              f"{'—' if ratio != ratio else f'{ratio:.3f}'}")
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps({"odom": a.odom.name, "segments": [list(s) for s in segs], "rows": rows, "legs": legs},
                                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
