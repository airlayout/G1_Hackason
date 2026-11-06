#!/usr/bin/env python3
"""事前地図の上で合成スキャンを撃ち、UpDown-SC の台帳（セッション）を作る。

    Navigation/.venv/bin/python build_db.py \\
        --map-pcd map_octomap_r4_s5_floor0.pcd --nav-map nav_map_run.yaml \\
        --ray-template fp_query_1.npz --out db_session

できたセッションは `scan_context_rebuild <out> db.scd gravity <out>/runtime_params.yaml`
にそのまま渡せる。

## なぜ合成なのか

過去の記録から台帳を作ると、記録ごとに座標系の流派が違う（aligned / floor0、
09-11 の地図の入れ替え）。**FAST_LIO が使っている事前地図そのもの**から作れば、
台帳の座標は測位と必ず一致する。

## 光線の型は実機のスキャンから取る

G1 の MID-360 は上下逆さで、前方に細い遮蔽帯があり、機体自身にも当たる。
理想的な視野（-7〜52 度）で撃つと実機と違う指紋になる。そこで**実機の静止スキャンの
光線の向き**（重力で水平に揃えた後）を型にして、各地点で同じ向きに撃つ。
機体に当たる光線（0.8 m 未満）は型から外す＝遮蔽もそのまま写る。

## 地点の選び方

`nav_map_run` の自由セルのうち、占有・未知から `--clearance` 以上離れた所を
`--step` 間隔で取る。向きは `--yaws` 通り（指紋は回転に強いが、遮蔽帯は機体に固定なので
向きを変えると指紋が少し変わる）。
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy import ndimage

BODY_HIT_M = 0.8        # これより近い点は機体自身に当たった光線
MAX_RANGE_M = 30.0      # UpDown-SC の max_radius と揃える
DIR_BIN_DEG = 0.5       # 光線の型を方位・仰角でこの幅に間引く
SPLIT_HEIGHT_M = 2.3    # 下段と天井の境目（床基準）。机より上・天井 2.7 m より下


def up_rotation(up: np.ndarray) -> np.ndarray:
    """`up` を +z に向ける回転。上下逆さ（up ≈ -z）でも破綻しない形で解く。"""
    up = up / np.linalg.norm(up)
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(up, z)
    s = np.linalg.norm(axis)
    c = float(up @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    axis /= s
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    angle = np.arctan2(s, c)
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def ray_template(npz_path: Path) -> tuple:
    """実機スキャンから (光線の単位ベクトル [重力水平], センサの床からの高さ) を返す。"""
    data = np.load(npz_path)
    rotation = up_rotation(data["accel"])
    raw = data["points"]
    # ⚠️ Livox は測れなかった点に桁外れの値を入れることがある（isfinite では落ちない）
    raw = raw[np.all(np.abs(raw) < 100.0, axis=1)]
    with np.errstate(all="ignore"):   # macOS の Accelerate が出す見かけの警告を黙らせる
        points = raw @ rotation.T
    ranges = np.linalg.norm(points, axis=1)
    keep = (ranges > BODY_HIT_M) & (ranges < MAX_RANGE_M)
    dirs = points[keep] / ranges[keep, None]
    # センサ高: 下向きの点の z の最頻値（床）
    hist, edges = np.histogram(points[:, 2], bins=np.arange(-3.0, 0.0, 0.02))
    sensor_height = -float(edges[np.argmax(hist)] + 0.01)
    az = np.degrees(np.arctan2(dirs[:, 1], dirs[:, 0]))
    el = np.degrees(np.arcsin(np.clip(dirs[:, 2], -1, 1)))
    cells = np.stack((np.floor(az / DIR_BIN_DEG), np.floor(el / DIR_BIN_DEG)), axis=1)
    _, first = np.unique(cells, axis=0, return_index=True)
    return dirs[first], sensor_height


def voxel_mesh(points: np.ndarray, voxel: float) -> o3d.t.geometry.RaycastingScene:
    """点を voxel の立方体にして光線判定の場を作る。"""
    keys = np.unique(np.floor(points / voxel).astype(np.int64), axis=0)
    corners = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    vertices = ((keys[:, None, :] + corners[None]) * voxel).reshape(-1, 3)
    triangles = (np.arange(len(keys))[:, None, None] * 8 + faces[None]).reshape(-1, 3)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(vertices.astype(np.float32)),
                        o3d.core.Tensor(triangles.astype(np.uint32)))
    return scene, len(keys)


def read_pgm(path: Path) -> np.ndarray:
    """P5（8 bit）だけ読む。Open3D は PGM を読めない。"""
    raw = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        match = re.compile(rb"\s*(#[^\n]*\n\s*)*(\S+)").match(raw, pos)
        tokens.append(match.group(2))
        pos = match.end()
    if tokens[0] != b"P5" or int(tokens[3]) > 255:
        raise ValueError("P5 / 8 bit の PGM ではない: %s" % path)
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(raw[pos + 1:pos + 1 + width * height], np.uint8).reshape(height, width)


def load_nav_map(yaml_path: Path) -> tuple:
    meta = yaml.safe_load(yaml_path.read_text())
    image = read_pgm(yaml_path.parent / meta["image"])
    # PGM は上の行が y 最大。配列を y 昇順に反転して扱う（build_grid と同じ罠を避ける）
    grid = np.flipud(image)
    return grid, float(meta["resolution"]), np.asarray(meta["origin"][:2], float)


def sample_poses(yaml_path: Path, step: float, clearance: float) -> np.ndarray:
    grid, res, origin = load_nav_map(yaml_path)
    free = grid >= 250
    dist = ndimage.distance_transform_edt(free) * res
    stride = max(1, int(round(step / res)))
    ys, xs = np.nonzero(dist[::stride, ::stride] >= clearance)
    return np.stack((origin[0] + (xs * stride + 0.5) * res,
                     origin[1] + (ys * stride + 0.5) * res), axis=1)


STORE_VOXEL_M = 0.25    # UpDown-SC の voxel_leaf と同じ。これより細かく持っても指紋は変わらない


def write_pcd(path: Path, points: np.ndarray) -> None:
    # ⚠️ 間引かずに書くと 1 地図で 3.5 GB を超え、12 向きで Mac のディスクが満杯になった（2026-09-26）
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud = cloud.voxel_down_sample(STORE_VOXEL_M)
    o3d.io.write_point_cloud(str(path), cloud, write_ascii=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map-pcd", required=True, type=Path)
    parser.add_argument("--nav-map", required=True, type=Path)
    parser.add_argument("--ray-template", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--step", type=float, default=0.5)
    parser.add_argument("--clearance", type=float, default=0.35)
    # ⚠️ 4 向き（90° おき）では、間の向き（35°）で指紋が外れた（2026-09-26）。
    # 光線の約 24% が機体に遮られ、遮られ方が機体に固定なので、回転不変が崩れる。
    # 24 向き（15° おき）なら台帳の最寄りの向きとの差は最大 7.5°。増えるのは初回の作成だけ
    parser.add_argument("--yaws", type=int, default=24)
    parser.add_argument("--voxel", type=float, default=0.1)
    args = parser.parse_args()

    dirs, sensor_height = ray_template(args.ray_template)
    map_points = np.asarray(o3d.io.read_point_cloud(str(args.map_pcd)).points)
    floor_z = float(np.median(map_points[np.abs(map_points[:, 2]) < 0.15, 2]))
    scene, voxels = voxel_mesh(map_points, args.voxel)
    poses_xy = sample_poses(args.nav_map, args.step, args.clearance)
    yaws = np.arange(args.yaws) * (2 * np.pi / args.yaws)
    print("光線 %d 本 / センサ高 %.3f m / 地図の床 z %.3f / voxel %d / 地点 %d x 向き %d"
          % (len(dirs), sensor_height, floor_z, voxels, len(poses_xy), len(yaws)), flush=True)

    frames_dir = args.out / "key_point_frame"
    frames_dir.mkdir(parents=True, exist_ok=True)
    sensor_z = floor_z + sensor_height
    tum_rows, gravity_rows, index = [], ["index,stamp,up_x,up_y,up_z"], 0
    for x, y in poses_xy:
        for yaw in yaws:
            c, s = np.cos(yaw), np.sin(yaw)
            world_dirs = dirs @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T
            origins = np.tile([x, y, sensor_z], (len(dirs), 1))
            rays = o3d.core.Tensor(np.hstack((origins, world_dirs)).astype(np.float32))
            t_hit = scene.cast_rays(rays)["t_hit"].numpy()
            hit = np.isfinite(t_hit) & (t_hit < MAX_RANGE_M)
            # 機体座標系（重力水平・センサ原点・機体の向き）で書く
            body = dirs[hit] * t_hit[hit, None]
            write_pcd(frames_dir / ("%d.pcd" % index), body)
            stamp = float(index)
            tum_rows.append("%.1f %.6f %.6f %.6f 0 0 %.9f %.9f"
                            % (stamp, x, y, sensor_z, np.sin(yaw / 2), np.cos(yaw / 2)))
            gravity_rows.append("%d,%.1f,0,0,1" % (index, stamp))
            index += 1
        if index % 200 < len(yaws):
            print("  %d 枚" % index, flush=True)

    (args.out / "optimized_poses_tum.txt").write_text("\n".join(tum_rows) + "\n")
    (args.out / "scan_context_gravity.csv").write_text("\n".join(gravity_rows) + "\n")
    write_runtime_params(args.out / "runtime_params.yaml", sensor_height)
    (args.out / "build_meta.json").write_text(json.dumps({
        "sensor_height_m": sensor_height, "floor_z": floor_z, "sensor_z": sensor_z,
        "rays": int(len(dirs)), "poses": int(len(poses_xy)), "yaws": int(len(yaws)),
        "step_m": args.step, "clearance_m": args.clearance, "voxel_m": args.voxel,
    }, indent=2))
    print("台帳 %d 枚 -> %s" % (index, args.out), flush=True)
    return 0


def write_runtime_params(path: Path, sensor_height: float) -> None:
    """UpDown-SC の既定値に、この機体と部屋の 2 点だけを上書きして書く。"""
    params = {
        "saveResultBodyFrame": True,
        "keyframe_source": "g1_synthetic_raycast",
        "scan_context": {
            "num_rings": 16, "num_sectors": 60, "max_radius": MAX_RANGE_M,
            "dual_z_layer_enable": True,
            "dual_z_split_height": SPLIT_HEIGHT_M, "dual_z_split_auto": False,
            "dual_z_split_auto_min": 1.5, "dual_z_split_auto_max": 4.5,
            "dual_z_split_auto_bin_size": 0.1, "dual_z_split_auto_histogram_max": 8,
            "dual_z_split_auto_min_layer_fraction": 0.05,
            "dual_z_split_auto_min_keyframes": 20,
            "origin_height_from_ground": round(sensor_height, 3),
            "ground_relative_dual_z_split_height": SPLIT_HEIGHT_M,
            "dual_z_low_weight": 0.3, "dual_z_high_weight": 0.7,
            "min_joint_rings": 2,
            "absent_upper_fallback_max_local_fraction": 0.05,
            "absent_upper_fallback_radius": 10, "absent_upper_fallback_min_keyframes": 3,
            "retrieval_height_offset": 0.1, "sector_support_exponent": 0.5,
            "vertical_boundary_margin": 0.1,
            "gravity_canonicalization_enable": True, "vertical_estimation_enable": False,
            "vertical_correction_min": -1.5, "vertical_correction_max": 1.5,
            "vertical_stable_fraction": 0.5,
            "voxel_leaf": 0.25, "candidate_top_k": 100, "yaw_top_k": 3,
            "distance_thresh": 0.5,
        },
    }
    path.write_text(yaml.safe_dump(params, sort_keys=False))


if __name__ == "__main__":
    raise SystemExit(main())
