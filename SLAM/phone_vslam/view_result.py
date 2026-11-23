"""保存した SLAM の結果（results/<日時>_slam1/ の map.ply・traj.txt）を Rerun のウェブ表示で開く。

    python view_result.py ~/work_inoue/phone_slam/results/1117_131452_slam1 \\
        [--hosts 100.99.102.70] [--max-points 2000000]

表示は固定の口（:9090）。SLAM を回したままでも、別の記録として並ぶ。
map.ply は全部の画素の点（数百万〜数千万点）なので、ブラウザが重くならないよう間引いて送る。
traj.txt はキーフレームの軌跡（TUM の形: 時刻 x y z qx qy qz qw）。
2 台以上で 1 つの地図にした結果（run_multicam_slam.py）は traj_<名前>.txt がカメラごとにあり、色を分けて出す。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import viewer_hub  # noqa: E402
from plyfile import PlyData

TRAJ_COLOR = (230, 60, 60)


def load_ply(path: pathlib.Path, max_points: int, conf_thresh: float = 0.0, seed: int = 0) -> tuple[np.ndarray, np.ndarray, int]:
    """(点, 色, 絞る前の点の数)。conf の欄があり conf_thresh > 0 なら、それより確からしさの低い点を落とす"""
    v = PlyData.read(str(path))["vertex"].data
    xyz = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float32)
    rgb = np.stack([v["red"], v["green"], v["blue"]], axis=1).astype(np.uint8)
    n_all = len(xyz)
    if conf_thresh > 0:
        if "conf" not in v.dtype.names:
            raise SystemExit(f"[結果] {path} に確からしさ（conf）が無い。11-18 より前の保存物は絞り直せない")
        keep = v["conf"] > conf_thresh
        xyz, rgb = xyz[keep], rgb[keep]
    n = len(xyz)
    if n > max_points:
        keep = np.random.default_rng(seed).choice(n, max_points, replace=False)
        xyz, rgb = xyz[keep], rgb[keep]
    return xyz, rgb, n_all


def load_traj(path: pathlib.Path) -> np.ndarray:
    """(N, 8) の [t x y z qx qy qz qw]。無ければ空"""
    if not path.exists() or path.stat().st_size == 0:
        return np.zeros((0, 8))
    return np.atleast_2d(np.loadtxt(path))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("result_dir")
    p.add_argument("--hosts", default="127.0.0.1", help="表示の URL に載せる IP（カンマ区切り）")
    p.add_argument("--max-points", type=int, default=2_000_000)
    p.add_argument("--conf-thresh", type=float, default=0.0,
                   help="この確からしさより低い点を出さない（保存物に conf の欄があるとき。0 は全部。保存のときは 1.5 で絞ってある）")
    args = p.parse_args()

    d = pathlib.Path(args.result_dir).expanduser().resolve()
    ply = d / "map.ply"
    if not ply.exists():
        raise SystemExit(f"[結果] {ply} が無い（start_slam.sh stop で保存される）")

    # 表示の口は固定（viewer_hub）。送ったら表示のサーバーが持っているので、この処理は終わってよい
    tag = f" conf>{args.conf_thresh:g}" if args.conf_thresh > 0 else ""
    viewer_hub.connect(rr, "phone_vslam result", recording_id=f"phone_vslam result {d.name}{tag}", blueprint=
                       rrb.Blueprint(rrb.Spatial3DView(name=f"map {d.name}", origin="world"), collapse_panels=True),
                       clear=("world",))
    rr.log("world", rr.ViewCoordinates.RDF, static=True)

    xyz, rgb, n_all = load_ply(ply, args.max_points, args.conf_thresh)
    rr.log("world/map", rr.Points3D(xyz, colors=rgb, radii=-1.0), static=True)
    trajs = sorted(d.glob("traj_*.txt")) or [d / "traj.txt"]
    counts = []
    for c, path in enumerate(trajs):
        traj = load_traj(path)
        name = path.stem.removeprefix("traj_") if path.stem != "traj" else "trajectory"
        color = viewer_hub.cam_color(c) if len(trajs) > 1 else TRAJ_COLOR
        counts.append(f"{name} {len(traj)}")
        if len(traj) >= 2:
            rr.log(f"world/trajectory/{name}", rr.LineStrips3D([traj[:, 1:4]], colors=[color], radii=-1.5), static=True)
        if len(traj) >= 1:
            rr.log(f"world/keyframes/{name}", rr.Points3D(traj[:, 1:4], colors=[color], radii=-3.0), static=True)
    print(f"[結果] {d.name}: 点 {n_all:,}（送ったのは {len(xyz):,}）・キーフレーム {'・'.join(counts)}", flush=True)
    for host in [h for h in args.hosts.split(",") if h]:
        print(f"[結果] {viewer_hub.viewer_url(host)}", flush=True)
    rr.disconnect()  # 送り切ってから終わる


if __name__ == "__main__":
    main()
