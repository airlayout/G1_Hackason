"""作成した地図が実際の Warehouse と合っているかを検証する。

地図と実シーンの座標がずれていないかを、Isaac Sim 側の真の形状と
比べて確かめる。

やり方:
    Warehouse の全 Mesh の頂点をワールド座標で取り出し、地上 1.1 m
    （LiDAR の高さ）付近を通る部分だけを 2D に落として「正解の占有図」を
    作る。それを SLAM で作った地図と重ねて、ずれ量を測る。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/check_map_alignment.py
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="地図と実シーンの整合を検証")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
MAP_STEM: str = "/home/spacedata/isaac_dev/G1/SimEnvTest/maps/warehouse"
# LiDAR の高さ [m]。この高さを通る形状だけを 2D の障害物とみなす。
#
# 幅を広めに取る。狭くすると壁の一部しか拾えず、Warehouse の実際の
# 広がりを見誤る（実測: ±0.15 m だと Y +0.8〜+30.6 m しか拾えず、
# 実際の Y -41.4〜+33.4 m と大きく食い違った）。
SLICE_Z: float = 1.1
# 高さ方向の許容幅 [m]
SLICE_HALF: float = 1.0


def main() -> None:
    """実シーンの正解図を作り、地図と比較する。"""
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[NG] アセットサーバーに接続できません。")
    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    stage: Usd.Stage = stage_utils.get_current_stage()

    # 既存の地図を読む
    with open(f"{MAP_STEM}.yaml") as fh:
        cfg = yaml.safe_load(fh)
    res = float(cfg["resolution"])
    ox, oy = float(cfg["origin"][0]), float(cfg["origin"][1])
    img = np.array(Image.open(f"{MAP_STEM}.pgm").convert("L"))
    height, width = img.shape
    print(f"[INFO] 地図: {width} x {height} セル、原点 ({ox:.2f}, {oy:.2f})")

    # Warehouse の全 Mesh から、SLICE_Z 付近を通る頂点を集める
    xform_cache = UsdGeom.XformCache()
    points_2d: list[tuple[float, float]] = []
    mesh_count = 0

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        mesh_count += 1
        matrix = xform_cache.GetLocalToWorldTransform(prim)
        arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        # 同次座標で world へ変換
        m = np.array(matrix).T  # Gf.Matrix4d は行優先なので転置
        world = (m[:3, :3] @ arr.T).T + m[:3, 3]
        # LiDAR の高さを通る点だけ残す
        mask = np.abs(world[:, 2] - SLICE_Z) < SLICE_HALF
        for wx, wy in world[mask, :2]:
            points_2d.append((float(wx), float(wy)))

    print(f"[INFO] Mesh {mesh_count} 個から、高さ {SLICE_Z} m 付近の点 "
          f"{len(points_2d)} 個を抽出しました")
    if not points_2d:
        print("[NG] 該当する点がありません")
        return

    truth = np.array(points_2d)
    print(
        f"[INFO] 実シーンの範囲: X {truth[:, 0].min():+.1f} 〜 "
        f"{truth[:, 0].max():+.1f} m / Y {truth[:, 1].min():+.1f} 〜 "
        f"{truth[:, 1].max():+.1f} m"
    )

    # 地図側の障害物セルをワールド座標へ
    ys, xs = np.where(img < 50)
    map_pts = np.stack(
        [xs * res + ox, (height - 1 - ys) * res + oy], axis=1
    )
    print(f"[INFO] 地図の障害物セル: {len(map_pts)} 個")
    if len(map_pts) == 0:
        print("[NG] 地図に障害物がありません")
        return
    print(
        f"[INFO] 地図の障害物の範囲: X {map_pts[:, 0].min():+.1f} 〜 "
        f"{map_pts[:, 0].max():+.1f} m / Y {map_pts[:, 1].min():+.1f} 〜 "
        f"{map_pts[:, 1].max():+.1f} m"
    )

    # 重心のずれを見る（大まかな平行移動のずれ）
    truth_center = truth.mean(axis=0)
    map_center = map_pts.mean(axis=0)
    offset = map_center - truth_center
    print(
        f"\n[INFO] 実シーンの重心: ({truth_center[0]:+.2f}, "
        f"{truth_center[1]:+.2f})"
    )
    print(f"[INFO] 地図の重心:     ({map_center[0]:+.2f}, {map_center[1]:+.2f})")
    print(f"[INFO] 重心のずれ:     ({offset[0]:+.2f}, {offset[1]:+.2f}) m")

    # 地図の各障害物点が、実シーンの点にどれだけ近いかを測る
    # （全点比較は重いので地図側を間引く）
    sample = map_pts[:: max(1, len(map_pts) // 3000)]
    from scipy.spatial import cKDTree

    tree = cKDTree(truth)
    dist, _ = tree.query(sample, k=1)
    print(
        f"\n[INFO] 地図の障害物から実形状までの距離: "
        f"中央値 {np.median(dist):.2f} m / 平均 {dist.mean():.2f} m / "
        f"90%点 {np.percentile(dist, 90):.2f} m"
    )

    if np.median(dist) < 0.3:
        print("[OK] 地図は実シーンとよく合っている")
    elif np.median(dist) < 1.0:
        print("[WARN] 多少ずれている。ナビゲーションは可能だが精度は落ちる")
    else:
        print("[NG] 大きくずれている。地図を作り直すべき")

    print("\n[OK] 検証が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
