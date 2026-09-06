#!/usr/bin/env python3
"""(B) メッシュ方式。点群を三角メッシュにして MuJoCo に載せ、凸包の問題を実測する。

## これは何のためのスクリプトか

`pcd_to_mjcf.py` の (A) 箱＋(C) hfield に対する**対案の検証**である。Isaac Sim では
静的コライダーに三角メッシュをそのまま使えるので (B) が有力になるが、**MuJoCo は
mesh の当たり判定を凸包で行う**。「部屋のメッシュを入れると凸包＝塊になって
ロボットが入れない」は本当か、入れないとしてどう回避するかを、実測で確かめる。

出力は2つのシーン。

- `..._mesh.xml`   … メッシュを1つの geom として入れた素直な版
- `..._meshtiled.xml` … メッシュをタイルに切り、タイルごとに geom にした版
  （凸分解の代わり。1タイルが十分小さければ凸包≒元の形になる）

CoACD / V-HACD による本式の凸分解はこの環境に入っていないので、
**空間タイル分割**で代用している。手法として粗いことは承知のうえで、
「凸包で潰れる」という MuJoCo の性質そのものを見せるのが目的。

## メッシュの作り方

法線推定＋Poisson ではなく、**3次元の占有格子にマーチングキューブ**を掛ける。
Poisson は水密な面を作ろうとするので、開口部（ドア・窓・観測できていない天井）に
膜を張る。占有格子なら**点があるところにだけ面ができる**ので、地図の被覆の穴が
そのまま穴として残り、後で「どこを観測できていないか」が見える。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/pcd_to_mesh_mjcf.py \\
        runs/20260904T183457_UiS_room_v2 map_octomap_r4.pcd
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage import measure

QUICKSTART = Path(__file__).resolve().parent
sys.path.insert(0, str(QUICKSTART))

from eval_removal import read_trajectory  # noqa: E402
from pcd_to_mjcf import ROBOT_INCLUDE, SPAWN_HEIGHT, SPAWN_JOINTS, level_cloud  # noqa: E402

VOXEL = 0.10               # マーチングキューブに掛ける格子の一辺[m]
Z_RANGE = (-0.30, 2.00)    # 床から (A) の障害物帯の上端まで。天井は入れない
SIGMA = 0.8                # 平滑化。0だと階段状の面になる
TRIANGLES = 60000          # 間引き後の目標三角数
TILES = (1.0, 1.5, 2.0, 3.0)  # 掃引するタイルの一辺[m]。小さいほど凸包が元の形に近づく
MAX_RANGE = 12.0           # 軌跡からこれ以上離れた点は捨てる（pcd_to_mjcf と同じ）


def build_mesh(points: np.ndarray, voxel: float, z_range: "tuple[float, float]",
               sigma: float, triangles: int) -> "tuple[o3d.geometry.TriangleMesh, dict]":
    """3次元の占有格子にマーチングキューブを掛けてメッシュにする。"""
    inside = (points[:, 2] >= z_range[0]) & (points[:, 2] < z_range[1])
    points = points[inside]
    low = np.array([points[:, 0].min(), points[:, 1].min(), z_range[0]]) - voxel
    index = np.floor((points - low) / voxel).astype(np.int64)
    shape = index.max(axis=0) + 3
    volume = np.zeros(shape, dtype=np.float32)
    volume[index[:, 0], index[:, 1], index[:, 2]] = 1.0
    occupancy = volume > 0.5
    occupied = int(volume.sum())
    if sigma > 0:
        volume = ndimage.gaussian_filter(volume, sigma=sigma)
        volume /= max(float(volume.max()), 1e-9)

    vertices, faces, _, _ = measure.marching_cubes(volume, level=0.5, spacing=(voxel,) * 3)
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices + low),
                                     o3d.utility.Vector3iVector(faces))
    raw_triangles = len(mesh.triangles)
    if triangles and raw_triangles > triangles:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=triangles)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh, occupancy, low, {"voxels_occupied": occupied,
                                  "triangles_raw": raw_triangles,
                                  "triangles": len(mesh.triangles),
                                  "vertices": len(mesh.vertices)}


def split_into_tiles(mesh: o3d.geometry.TriangleMesh,
                     tile: float) -> "list[o3d.geometry.TriangleMesh]":
    """三角形を重心の位置でタイルに振り分け、タイルごとのメッシュにする。

    MuJoCo は mesh geom ごとに凸包を取るので、**タイルを小さくするほど
    凸包の集合が元の形に近づく**。凸分解の代わりになる粗い手だが、
    部屋のように面が軸に沿っている形では実用になる。
    """
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    centroid = vertices[faces].mean(axis=1)
    key = np.floor(centroid[:, :2] / tile).astype(np.int64)
    order = np.lexsort((key[:, 1], key[:, 0]))
    key, faces = key[order], faces[order]
    boundary = np.nonzero(np.any(np.diff(key, axis=0) != 0, axis=1))[0] + 1
    pieces = []
    for chunk in np.split(faces, boundary):
        if len(chunk) < 4:
            continue
        used, remap = np.unique(chunk, return_inverse=True)
        piece = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices[used]),
            o3d.utility.Vector3iVector(remap.reshape(-1, 3)))
        piece.compute_vertex_normals()
        pieces.append(piece)
    return pieces


def hull_halfspaces(hull: o3d.geometry.TriangleMesh) -> "tuple[np.ndarray, np.ndarray]":
    """凸包を「全部の面の内側」という不等式に直す。戻りは (法線 Fx3, 定数 F)。"""
    vertices = np.asarray(hull.vertices)
    faces = np.asarray(hull.triangles)
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    normal = np.cross(b - a, c - a)
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    normal = normal / np.maximum(length, 1e-12)
    offset = np.einsum("ij,ij->i", normal, a)
    # 重心が内側に来るよう向きを揃える
    flip = (normal @ vertices.mean(axis=0) - offset) > 0
    normal[flip] *= -1.0
    offset[flip] *= -1.0
    return normal, offset


def blocked_by_hulls(hulls: "list[o3d.geometry.TriangleMesh]", reachable: np.ndarray,
                     low: np.ndarray, voxel: float) -> int:
    """到達可能な自由ボクセルのうち、どれかの凸包の内側に入ってしまう数を数える。

    **これが (B) の実害である。** 凸包は元の面より膨らむので、机と机の隙間や
    壁ぎわの通路が塞がる。「歩ける空間がどれだけ減るか」で測る。

    自由空間を「到達可能なもの」に限るのは、机の内部のような閉じた空洞を
    数えないため。そこが埋まってもロボットには関係ない。
    """
    grid = np.stack(np.nonzero(reachable), axis=1)
    points = low + (grid + 0.5) * voxel
    blocked = np.zeros(len(points), dtype=bool)
    for hull in hulls:
        normal, offset = hull_halfspaces(hull)
        corner = np.asarray(hull.vertices)
        inside_box = np.all((points >= corner.min(axis=0)) & (points <= corner.max(axis=0)), axis=1)
        candidate = np.nonzero(inside_box & ~blocked)[0]
        if not len(candidate):
            continue
        chunk = points[candidate]
        with np.errstate(all="ignore"):   # numpy の matmul が出す偽の FP 警告
            blocked[candidate] = np.all(chunk @ normal.T - offset <= 1e-9, axis=1)
    return int(blocked.sum())


def reachable_free(occupancy: np.ndarray, low: np.ndarray, voxel: float,
                   seed_xy: "tuple[float, float]") -> np.ndarray:
    """占有されていないボクセルのうち、開始地点と繋がっているものだけを返す。"""
    free = ~occupancy
    labels, _ = ndimage.label(free)
    index = np.floor((np.array([seed_xy[0], seed_xy[1], 0.6]) - low) / voxel).astype(int)
    index = np.clip(index, 0, np.array(occupancy.shape) - 1)
    label = labels[tuple(index)]
    if label == 0:                       # 開始点が占有側に落ちたら最大の空間を使う
        counts = np.bincount(labels.ravel())
        counts[0] = 0
        label = int(counts.argmax())
    return labels == label


HEAD = """<mujoco model="{model}">
  <!-- {provenance} -->
  <include file="{robot}"/>
  <statistic center="{cx:.3f} {cy:.3f} 1.000" extent="{extent:.3f}"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
    <global azimuth="-90" elevation="-45" offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
{assets}
  </asset>
  <worldbody>
    <light pos="{cx:.3f} {cy:.3f} 6" dir="0 0 -1" directional="true"/>
{geoms}
  </worldbody>
  <keyframe><key name="spawn" qpos="{sx:.3f} {sy:.3f} {sz:.3f} 1 0 0 0 {joints}"/></keyframe>
</mujoco>
"""


def write_scene(path: Path, model: str, provenance: str, assets: "list[str]",
                geoms: "list[str]", center: "tuple[float, float]", extent: float,
                spawn: "tuple[float, float]") -> None:
    path.write_text(HEAD.format(
        model=model, provenance=provenance, robot=ROBOT_INCLUDE,
        cx=center[0], cy=center[1], extent=extent,
        assets="\n".join(assets), geoms="\n".join(geoms),
        sx=spawn[0], sy=spawn[1], sz=SPAWN_HEIGHT, joints=SPAWN_JOINTS), encoding="utf-8")


def hull_union(hulls: "list[o3d.geometry.TriangleMesh]") -> "tuple[np.ndarray, np.ndarray]":
    """表示用に、凸包を1つの頂点配列・面配列へ連結する。"""
    vertices, faces, offset = [], [], 0
    for hull in hulls:
        v = np.asarray(hull.vertices)
        f = np.asarray(hull.triangles) + offset
        vertices.append(v)
        faces.append(f)
        offset += len(v)
    if not vertices:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)
    return (np.concatenate(vertices).astype(np.float32),
            np.concatenate(faces).astype(np.int32))


def drop_test(scene_path: Path, start: "tuple[float, float]") -> dict:
    """部屋の中にボールを置いて落とし、当たり判定の実体を測る。

    `mj_ray` では測れない。レイはメッシュの三角形を直接見るので、凸包で潰れていても
    素通りで床を返してしまう（2026-09-05 に一度これで誤判定した）。
    """
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    ball = model.body("probe").id
    mujoco.mj_resetData(model, data)
    address = model.body_jntadr[ball]
    data.qpos[model.jnt_qposadr[address]:model.jnt_qposadr[address] + 3] = [start[0], start[1], 1.20]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    final = data.qpos[model.jnt_qposadr[address]:model.jnt_qposadr[address] + 3].copy()
    return {"final_z": round(float(final[2]), 2),
            "drift_xy": round(float(np.hypot(final[0] - start[0], final[1] - start[1])), 2),
            "ngeom": int(model.ngeom)}


PROBE = ('    <body name="probe" pos="0 0 1.2"><freejoint/>'
         '<geom type="sphere" size="0.10" mass="1" rgba="0.9 0.3 0.2 1"/></body>')


def main() -> None:
    parser = argparse.ArgumentParser(description="点群をメッシュにして MuJoCo に載せる（B案）")
    parser.add_argument("session", type=Path)
    parser.add_argument("map", nargs="?", default="map_octomap_r4.pcd")
    parser.add_argument("--voxel", type=float, default=VOXEL)
    parser.add_argument("--sigma", type=float, default=SIGMA)
    parser.add_argument("--triangles", type=int, default=TRIANGLES)
    parser.add_argument("--tiles", type=float, nargs="+", default=list(TILES),
                        help="掃引するタイルの一辺[m]")
    parser.add_argument("--max-range", type=float, default=MAX_RANGE)
    parser.add_argument("--scene-dir", type=Path,
                        default=QUICKSTART.parents[2] / "Navigation/sim/assets/g1_description")
    args = parser.parse_args()

    session = args.session.resolve()
    scene_dir = args.scene_dir.resolve()
    sim_dir = session / "sim"
    sim_dir.mkdir(parents=True, exist_ok=True)
    name = json.loads((session / "manifest.json").read_text())["name"]
    variant = Path(args.map).stem.removeprefix("map_")
    stem = f"{name}_{variant}"

    points = np.asarray(o3d.io.read_point_cloud(str(session / "map" / args.map)).points)
    points = points[np.isfinite(points).all(axis=1)]
    leveled, rotation, floor_z, tilt = level_cloud(points)
    trajectory = read_trajectory(session)
    with np.errstate(all="ignore"):
        traj = trajectory @ rotation.T
    traj[:, 2] -= floor_z
    if args.max_range > 0:
        near = cKDTree(traj[:, :2]).query(leveled[:, :2])[0] <= args.max_range
        leveled = leveled[near]
    print(f"[mesh] 点 {len(leveled):,}（傾き {tilt:.2f}° を補正済み）", flush=True)

    mesh, occupancy, low, stats = build_mesh(leveled, args.voxel, Z_RANGE,
                                             args.sigma, args.triangles)
    print(f"[mesh] 占有ボクセル {stats['voxels_occupied']:,} → 三角 "
          f"{stats['triangles_raw']:,} → 間引き後 {stats['triangles']:,}", flush=True)

    mesh_path = scene_dir / f"_mesh_{stem}.stl"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    bound = mesh.get_axis_aligned_bounding_box()
    lo, hi = np.asarray(bound.min_bound), np.asarray(bound.max_bound)
    center = (float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2))
    extent = float(max(hi[0] - lo[0], hi[1] - lo[1]) / 2)
    spawn = (float(traj[len(traj) // 2, 0]), float(traj[len(traj) // 2, 1]))
    provenance = f"{session.name} / {args.map} から pcd_to_mesh_mjcf.py が生成"

    reachable = reachable_free(occupancy, low, args.voxel, spawn)
    free_voxels = int(reachable.sum())
    unit = args.voxel ** 3
    print(f"[mesh] 到達できる自由空間 {free_voxels:,} ボクセル = {free_voxels * unit:,.0f} m³",
          flush=True)

    display, results = {}, []
    for tile in [None] + list(args.tiles):
        if tile is None:
            pieces, tag, label = [mesh], "mesh", "メッシュ1枚"
        else:
            pieces = split_into_tiles(mesh, tile)
            tag, label = f"tile{int(tile * 10):02d}", f"タイル {tile:.1f} m"
        hulls = [piece.compute_convex_hull()[0] for piece in pieces]
        blocked = blocked_by_hulls(hulls, reachable, low, args.voxel)

        if tile is None:
            scene_path = scene_dir / f"_scene_{stem}_mesh.xml"
            assets = [f'    <mesh name="room" file="../{mesh_path.name}"/>']
            geoms = ['    <geom name="room" type="mesh" mesh="room" rgba="0.62 0.64 0.68 1"/>']
        else:
            tile_dir = scene_dir / f"_meshtiles_{stem}_{tag}"
            tile_dir.mkdir(exist_ok=True)
            for old in tile_dir.glob("*.stl"):
                old.unlink()
            assets, geoms = [], []
            for i, piece in enumerate(pieces):
                o3d.io.write_triangle_mesh(str(tile_dir / f"t{i:04d}.stl"), piece)
                assets.append(f'    <mesh name="t{i:04d}" file="../{tile_dir.name}/t{i:04d}.stl"/>')
                geoms.append(f'    <geom name="t{i:04d}" type="mesh" mesh="t{i:04d}" '
                             f'rgba="0.62 0.64 0.68 1"/>')
            scene_path = scene_dir / f"_scene_{stem}_{tag}.xml"
        write_scene(scene_path, f"{stem}_{tag}", provenance, assets, geoms + [PROBE],
                    center, extent, spawn)

        drop = drop_test(scene_path, spawn)
        vertices, faces = hull_union(hulls)
        display[tag] = (vertices, faces)
        results.append({
            "tag": tag, "label": label, "tile_m": tile,
            "pieces": len(pieces), "scene": scene_path.name,
            "triangles": int(sum(len(p.triangles) for p in pieces)),
            "hull_triangles": int(len(faces)),
            "blocked_voxels": blocked,
            "blocked_m3": round(blocked * unit, 1),
            "blocked_pct": round(blocked / max(free_voxels, 1) * 100.0, 1),
            **drop,
        })
        print(f"[{label:12s}] 破片 {len(pieces):4d} / geom {drop['ngeom']:4d} / "
              f"塞がる自由空間 {blocked * unit:7.1f} m³ ({results[-1]['blocked_pct']:4.1f}%) / "
              f"ボール最終 z {drop['final_z']:6.2f} m", flush=True)

    payload = {"vertices": np.asarray(mesh.vertices, dtype=np.float32),
               "faces": np.asarray(mesh.triangles, dtype=np.int32),
               "trajectory": traj[:, :2].astype(np.float32)}
    for tag, (vertices, faces) in display.items():
        payload[f"hull_{tag}_v"] = vertices
        payload[f"hull_{tag}_f"] = faces
    np.savez_compressed(sim_dir / "mesh.npz", **payload)

    report = {"session": session.name, "name": name, "variant": variant,
              "settings": {"voxel": args.voxel, "sigma": args.sigma,
                           "z_range": list(Z_RANGE), "max_range": args.max_range,
                           "tiles": args.tiles},
              "mesh": stats | {"file": mesh_path.name,
                               "extent_m": [round(float(v), 2) for v in (hi - lo)]},
              "free_voxels": free_voxels, "free_m3": round(free_voxels * unit, 1),
              "spawn": [round(spawn[0], 3), round(spawn[1], 3)],
              "variants": results}
    (sim_dir / "mesh.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                       encoding="utf-8")
    print(f"[完了] {sim_dir / 'mesh.json'}", flush=True)


if __name__ == "__main__":
    main()
