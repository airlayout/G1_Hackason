"""段 5・L2（2026-10-23）: 歩いた後の枠の運び方を、録った記録で比べる。機体は要らない。

比べる 2 つ（FAST_LIO は別の道具 fastlio_vs_icp_1021.py）:
  (a) 毎回の測定で当て直す  lidar_door_est.DoorTracker の扉の座標でのライダーの姿勢（止まった区間ごとの中央値）
  (c) /dog_odom で運ぶ      区間 0 の姿勢に odometry の動きを足す（lidar_still_icp.py の odom の列）
正解は lidar_still_icp.py の ICP（止まった区間どうしの動き・robot_center の 前・左・向き）。先に --json で書いておく。

扉と地図は、記録の始めの 2.5 秒で作れれば（扉の正面に立っている）それを使い、作れなければ --ref の記録（10-21 の 01、扉の正面で 60 秒静止）
で作って、最初の回の位置を relocalize で探す。

使い方: Navigation/.venv/bin/python Door/real/lidar_track_vs_icp.py <ライダーのセッション> <icp の json> --ref <01 のライダーのセッション> [--json 出力]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lidar_door_est as L  # noqa: E402
from crop_lidar_bag import iter_lidar, scan_xyz  # noqa: E402
from lidar_still_icp import EDGE_S, LIVOX_XY_DEFAULT  # noqa: E402

STILL_SCANS = 25  # 扉と地図を作る回（10 Hz で 2.5 秒）
THICKNESS_M = 0.04  # 板の厚み（10-21 の扉は未測。比べるのは動きなので効かない）


def load_scans(session: Path):
    t, scans = [], []
    for recv_ns, _stamp, payload in iter_lidar(session):
        t.append(recv_ns / 1e9)
        scans.append(scan_xyz(payload))
    return np.array(t), scans


def make_tracker(scans: List[np.ndarray], ref: Path | None) -> tuple[L.DoorTracker, str]:
    try:
        return L.DoorTracker.from_still(scans[:STILL_SCANS], THICKNESS_M), "この記録の始め"
    except ValueError as e:
        if ref is None:
            raise
        why = str(e)
    _, ref_scans = load_scans(ref)
    base = L.DoorTracker.from_still(ref_scans[:STILL_SCANS], THICKNESS_M)
    level = L.level_from_scans([L.valid_points(s) for s in scans[:STILL_SCANS]])
    xy = L.band_xy(level.apply(L.valid_points(scans[0])))
    res = L.relocalize(base.map, xy, base.door)
    if not res.ok:
        raise SystemExit(f"[track] 最初の回の位置が見つからない（当たった点 {res.matched}）")
    return L.DoorTracker(base.door, base.map, level, res.pose), f"--ref の地図（{why}）・最初の位置 {res.pose}"


def lidar_to_center_rel(p0: L.Pose2D, pk: L.Pose2D, o: np.ndarray) -> np.ndarray:
    """扉の座標でのライダーの姿勢 2 つ → 区間 0 の robot_center から見た robot_center の動き（前・左・向き）"""
    off = L.Pose2D(-float(o[0]), -float(o[1]), 0.0)  # ライダーの座標で見た robot_center
    c0, ck = p0.compose(off), pk.compose(off)
    d = c0.inverse().compose(ck)
    return np.array([d.x, d.y, d.yaw])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("li", type=Path)
    ap.add_argument("icp_json", type=Path)
    ap.add_argument("--ref", type=Path)
    ap.add_argument("--livox-xy", type=float, nargs=2, default=list(LIVOX_XY_DEFAULT))
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    o = np.array(a.livox_xy)
    truth = json.loads(a.icp_json.read_text())
    t, scans = load_scans(a.li)
    tracker, how = make_tracker(scans, a.ref)
    est = [tracker.step(ti, s) for ti, s in zip(t, scans)]
    ok = np.array([e.accepted for e in est])
    pose = np.array([[e.lidar.x, e.lidar.y, e.lidar.yaw] for e in est])
    rms = np.array([e.match.rms_m for e in est])
    eig = np.array([e.match.min_eig for e in est])
    jump = np.r_[0.0, np.hypot(*np.diff(pose[:, :2], axis=0).T)]
    odom_t0 = None  # icp の json の区間は odometry の記録の始めからの秒。ライダーの時刻へは区間の絶対時刻で合わせる
    segs = truth["segments"]
    # lidar_still_icp.py は区間を odometry の始め基準の秒で残すので、ライダーの記録の時刻との差を odometry の始めで埋める
    odom_session = Path(str(a.li).replace("_li", "_odom"))
    from dog_odom_bag import load_odom  # noqa: E402

    odom_t0 = float(load_odom(odom_session).recv_s[0])
    seg_pose = []
    for sa, sb in segs:
        m = (t >= odom_t0 + sa + EDGE_S) & (t <= odom_t0 + sb - EDGE_S) & ok
        if m.sum() < 3:
            seg_pose.append(None)
            continue
        yaw = math.atan2(np.median(np.sin(pose[m, 2])), np.median(np.cos(pose[m, 2])))
        seg_pose.append(L.Pose2D(float(np.median(pose[m, 0])), float(np.median(pose[m, 1])), yaw))
    rows = []
    print(f"== {a.li.name}: 回 {len(est)}・追えた回 {ok.mean() * 100:.1f}%・扉と地図 {how}")
    print(f"   重ね直しの残差 中央 {np.nanmedian(rms[ok]) * 1000:.1f} mm・最大 {np.nanmax(rms[ok]) * 1000:.1f} mm・"
          f"最小の固有値の最小 {np.min(eig[ok]):.4f}・1 回あたりの跳びの最大 {jump[ok].max() * 100:.1f} cm")
    print("  区間   正解 ICP（前・左 m・向き°）     (a) 当て直し − 正解          (c) odom − 正解           動いた距離")
    for r in truth["rows"]:
        k = r["区間"]
        icp = np.array(r["icp"])
        odom = np.array(r["odom"])
        if seg_pose[0] is None or seg_pose[k] is None:
            continue
        trk = lidar_to_center_rel(seg_pose[0], seg_pose[k], o)
        ea, ec = trk - icp, odom - icp
        for e in (ea, ec):
            e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        dist = float(np.hypot(icp[0], icp[1]))
        rows.append({"区間": k, "icp": icp.round(4).tolist(), "track": trk.round(4).tolist(), "odom": odom.round(4).tolist(),
                     "err_track_cm": round(float(np.hypot(*ea[:2])) * 100, 2), "err_track_deg": round(math.degrees(ea[2]), 2),
                     "err_odom_cm": round(float(np.hypot(*ec[:2])) * 100, 2), "err_odom_deg": round(math.degrees(ec[2]), 2),
                     "dist_m": round(dist, 3), "icp_fitness": r["fitness"]})
        print(f"  0→{k:<2}  {icp[0]:+.3f} {icp[1]:+.3f} {math.degrees(icp[2]):+7.2f}°   "
              f"{ea[0] * 100:+5.1f} {ea[1] * 100:+5.1f} cm {math.degrees(ea[2]):+5.2f}°   "
              f"{ec[0] * 100:+5.1f} {ec[1] * 100:+5.1f} cm {math.degrees(ec[2]):+5.2f}°   {dist:.2f} m（ICP {r['fitness']:.2f}）")
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps({"li": a.li.name, "how": how, "ok_ratio": float(ok.mean()),
                                      "rms_median_mm": float(np.nanmedian(rms[ok]) * 1000), "max_jump_cm": float(jump[ok].max() * 100),
                                      "rows": rows}, ensure_ascii=False, indent=1))
        np.savez_compressed(a.json.with_suffix(".npz"), t=t, pose=pose, ok=ok, rms=rms, eig=eig,
                            angle=np.array([e.angle_deg for e in est]), tilt=np.array([e.tilt_deg for e in est]))


if __name__ == "__main__":
    main()
