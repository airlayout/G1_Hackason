"""歩行路の外側の静的物体と、同じUSD形状から作るLiDAR用メッシュ。"""

from __future__ import annotations

import numpy as np


def add_landmarks():
    import isaaclab.sim as sim_utils

    boxes = [
        ((3.0, 2.5, 0.5), (0.7, 0.7, 1.0), (0.3, 0.5, 0.7)),
        ((5.0, -2.5, 0.75), (0.8, 0.8, 1.5), (0.7, 0.45, 0.25)),
        ((8.0, 2.0, 0.6), (1.0, 0.6, 1.2), (0.4, 0.65, 0.4)),
        ((11.0, 0.0, 1.0), (0.3, 6.0, 2.0), (0.55, 0.55, 0.6)),
    ]
    for i, (position, size, color) in enumerate(boxes):
        cfg = sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        )
        cfg.func(f"/World/CollectionScene/Marker{i}", cfg, translation=position)
    return ["/World/Ground", "/World/CollectionScene"]


def static_mesh(stage, root_paths: list[str]):
    """Cube/Meshをワールド座標へ展開する。ロボットの自己反射・動体は対象外。"""
    from pxr import Usd, UsdGeom

    vertices, triangles, paths = [], [], []
    cube_vertices = np.array(
        [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
        np.float32,
    )
    cube_triangles = np.array(
        [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ],
        np.int32,
    )
    offset = 0
    for path in root_paths:
        root = stage.GetPrimAtPath(path)
        if not root.IsValid():
            raise ValueError(f"LiDARの静的シーンがありません: {path}")
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Cube):
                points = cube_vertices * (UsdGeom.Cube(prim).GetSizeAttr().Get() / 2)
                faces = cube_triangles
            elif prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
                counts, indices = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
                faces, first = [], 0
                for count in counts:
                    faces.extend(
                        (indices[first], indices[first + j], indices[first + j + 1])
                        for j in range(1, count - 1)
                    )
                    first += count
                faces = np.asarray(faces, dtype=np.int32)
            else:
                continue
            matrix = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            points = points @ matrix[:3, :3] + matrix[3, :3]
            vertices.append(points)
            triangles.append(faces + offset)
            paths.append(str(prim.GetPath()))
            offset += len(points)
    if not vertices:
        raise ValueError("LiDARの静的シーンにCube/Meshがありません")
    return np.concatenate(vertices).astype(np.float32), np.concatenate(triangles).astype(np.int32), paths
