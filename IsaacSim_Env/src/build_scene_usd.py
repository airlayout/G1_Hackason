#!/usr/bin/env python3
"""実測地図の占有グリッド (npz) から Isaac Sim 用の障害物 USD を作る。

`clean_map.py` が出す `sim/octomap_sim.npz` を、`pcd_to_mjcf.py` と**同じ箱**
（`grid_boxes.boxes_from_grid`）にして 1 枚の結合三角形メッシュにし、静的
コライダーとして USD へ書き出す。地図を作り直すたびにこれを流せば、Nav2 が
読む 2D 地図と Isaac Sim の物理シーンが同じ元データから揃う。

床は入れない。`runner.py` が `GroundPlaneCfg`（平地）を別に敷く。実測の床の
凹凸は ±7 cm しかなく、平地で学習したポリシー（`Isaac-Velocity-Flat-G1-v0`）
には平地の方が安全という判断（2026-09-08）。

⚠️ **SimulationApp は起動しない。** USD の authoring は pxr だけで完結する。
   ただし pxr は Isaac Sim の python にしか入っていないので、実行は
   `"$ISAAC_SIM/python.sh"` で行う（`env.sh` を source すること）。

⚠️ **座標系の一致は別途確かめること。** この npz は `pcd_to_mjcf.py` 系
   （床の傾き補正で回転済み）から、Nav2 の PGM は `pcd_to_occupancy.py`
   （回転補正なし）から来ており、**別のスクリプト・別の crop** なので素朴には
   一致しない。両者の占有セルを共通グリッドに落として相互相関を取り、最適
   シフトが 1 セル以内であることを確認してから使う。20260906T135940_UiS_room_v3
   の `nav_map_clean` と `octomap_sim` では (0, 0) で一致した（2026-09-08 実測）。

使い方:
    source env.sh
    "$ISAAC_SIM/python.sh" src/build_scene_usd.py \\
        --npz  .../runs/<記録>/sim/octomap_sim.npz \\
        --out  assets/uis_room_v3_sim_obstacles.usd
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# grid_boxes は numpy だけに依存する（open3d / scipy を入れている
# pcd_to_mjcf.py から読むと、Isaac Sim 側にそれらが無いので落ちる）。
#
# ⚠️ Isaac Sim を動かす機械（192.168.123.200）に置いてあるのは、この
# リポジトリの **git 管理外の古いコピー**で、`Mapping/real/quickstart/` が
# 無いことがある。本来の置き場を先に見て、無ければこのスクリプトの隣を見る。
_HERE = Path(__file__).resolve().parent
for _candidate in (_HERE.parents[1] / "Mapping" / "real" / "quickstart", _HERE):
    if (_candidate / "grid_boxes.py").is_file():
        sys.path.insert(0, str(_candidate))
        break
else:
    raise SystemExit(
        "[NG] grid_boxes.py が見つからない。\n"
        f"     探した場所: {_HERE.parents[1] / 'Mapping/real/quickstart'}\n"
        f"                 {_HERE}\n"
        "     Mapping/real/quickstart/grid_boxes.py をこのスクリプトの隣へ"
        "コピーする。")
from grid_boxes import boxes_from_grid  # noqa: E402


def boxes_to_mesh(boxes: "list[tuple[float, float, float, float, float]]",
                  cell: float) -> "tuple[np.ndarray, np.ndarray]":
    """箱のリストを 1 つの結合三角形メッシュ（頂点, 三角形インデックス）にする。

    箱を 1 つずつ prim にすると数千 prim になって開くだけで重い。1 枚の
    メッシュにまとめる。
    """
    half_y = cell / 2.0
    n = len(boxes)
    points = np.empty((n * 8, 3), dtype=np.float32)
    tris = np.empty((n * 12, 3), dtype=np.int32)
    unit_verts = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ], dtype=np.float32)
    unit_tris = np.array([
        [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0],
    ], dtype=np.int32)
    for i, (cx, cy, cz, half_x, half_z) in enumerate(boxes):
        scale = np.array([half_x, half_y, half_z], dtype=np.float32)
        center = np.array([cx, cy, cz], dtype=np.float32)
        points[i * 8:(i + 1) * 8] = unit_verts * scale + center
        tris[i * 12:(i + 1) * 12] = unit_tris + i * 8
    return points, tris


def write_usd(out_path: Path, points: np.ndarray, tris: np.ndarray,
              prim_name: str) -> None:
    """静的コライダーの USD を書く。pxr が要る（Isaac Sim の python で実行）。"""
    from pxr import Usd, UsdGeom, UsdPhysics      # noqa: PLC0415

    prim_path = f"/World/{prim_name}"
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, prim_path)
    stage.SetDefaultPrim(world.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, f"{prim_path}/Obstacles")
    mesh.CreatePointsAttr(points.tolist())
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr(tris.reshape(-1).tolist())
    # 法線は当たり判定に不要。表示用の簡易カラー（Warehouse の壁材に近い灰）
    mesh.CreateDisplayColorAttr([(0.55, 0.55, 0.60)])

    # 静的コライダーにする。RigidBodyAPI は付けない（＝動かない世界ジオメトリ）。
    # 三角形メッシュそのものを当たり判定に使う（approximation="none"）。
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")

    stage.GetRootLayer().Save()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", required=True,
                    help="clean_map.py が出す sim/octomap_sim.npz")
    ap.add_argument("--out", required=True, help="書き出す .usd")
    ap.add_argument("--prim", default="RoomObstacles",
                    help="/World の下に作る prim 名")
    ap.add_argument("--mesh-npz", help="中間の頂点データも残す（差分確認用）")
    args = ap.parse_args()

    npz_path, out_path = Path(args.npz), Path(args.out)
    if not npz_path.is_file():
        print(f"[NG] npz が無い: {npz_path}")
        return 1
    if out_path.exists():
        print(f"[NG] 出力先が既にある: {out_path}")
        print("     上書きしたいなら先に消すか、別の名前にする")
        return 1

    d = np.load(npz_path)
    for key in ("occupied", "level", "origin", "cell"):
        if key not in d:
            print(f"[NG] {npz_path.name} に '{key}' が無い。"
                  f"clean_map.py が出した npz か確認する")
            return 1
    occupied, level = d["occupied"], d["level"]
    origin, cell = d["origin"], float(d["cell"])
    print(f"[load] {npz_path.name}: 占有 {int(occupied.sum())} セル / "
          f"格子 {occupied.shape} / origin {origin} / cell {cell}")

    boxes = boxes_from_grid(occupied, level, cell, origin)
    points, tris = boxes_to_mesh(boxes, cell)
    print(f"[mesh] 箱 {len(boxes)} / 頂点 {len(points)} / 三角形 {len(tris)}")

    if args.mesh_npz:
        np.savez_compressed(args.mesh_npz, points=points,
                            triangle_indices=tris.reshape(-1),
                            num_boxes=np.array(len(boxes)),
                            origin=origin, cell=np.array(cell))
        print(f"[mesh] -> {args.mesh_npz}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    write_usd(out_path, points, tris, args.prim)
    print(f"[OK] -> {out_path} ({out_path.stat().st_size / 1e6:.2f} MB)")
    print(f"     使い方: SCENE_USD={out_path} bash run_nav2.sh <地図.yaml>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
