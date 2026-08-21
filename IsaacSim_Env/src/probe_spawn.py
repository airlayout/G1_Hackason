"""Warehouse 内の各地点がどれだけ開けているかを実測する調査用スクリプト。

自動巡回が特定の場所で物理的に動けなくなる問題の対策。
LiDAR を各候補地点に置いて全方位の余裕を測り、巡回の開始点として
適した「広い場所」を見つける。

G1 は歩かせず、LiDAR だけをテレポートさせて測るので短時間で終わる。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_spawn.py
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="スポーン候補地点の開放度を実測")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sensors import MultiMeshRayCaster, MultiMeshRayCasterCfg, patterns  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
# LiDAR の高さ（巡回時と同じ地上 1.1 m）
LIDAR_HEIGHT: float = 1.1
# 調べる範囲と刻み [m]
GRID_MIN: float = -30.0
GRID_MAX: float = 10.0
GRID_STEP: float = 5.0


def main() -> None:
    """格子状の候補地点で全方位の余裕を測る。"""
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, device=args.device))

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[NG] アセットサーバーに接続できません。")
    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    light = sim_utils.DomeLightCfg(intensity=1500.0)
    light.func("/World/DomeLight", light)

    # LiDAR を載せるためのダミーの剛体を作る。
    # これ自体は raycast の対象にしない（Warehouse だけを見る）。
    probe = sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        collision_props=sim_utils.CollisionPropertiesCfg(),
    )
    probe.func("/World/Probe", probe, translation=(0.0, 0.0, LIDAR_HEIGHT))

    pattern = patterns.LidarPatternCfg(
        channels=1,
        vertical_fov_range=(0.0, 0.0),
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=2.0,
    )
    lidar = MultiMeshRayCaster(
        MultiMeshRayCasterCfg(
            prim_path="/World/Probe",
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
            ray_alignment="yaw",
            pattern_cfg=pattern,
            mesh_prim_paths=["/World/Warehouse"],
            max_distance=30.0,
            debug_vis=False,
        )
    )

    sim.reset()
    for _ in range(5):
        sim.step(render=False)

    # isaacsim.core.prims の SingleXFormPrim は Isaac Sim 6.0 の
    # PhysxManager と非互換（_get_backend_utils が無い）ため、
    # USD の API で直接 xform を書き換える。
    from isaacsim.core.utils.prims import get_prim_at_path
    from pxr import Gf, UsdGeom

    probe_xform = UsdGeom.Xformable(get_prim_at_path("/World/Probe"))
    probe_xform.ClearXformOpOrder()
    translate_op = probe_xform.AddTranslateOp()

    results: list[tuple[float, float, float, float]] = []
    coords = np.arange(GRID_MIN, GRID_MAX + GRID_STEP, GRID_STEP)
    print(f"[INFO] {len(coords) ** 2} 地点を調べます...")

    for x in coords:
        for y in coords:
            translate_op.Set(Gf.Vec3d(float(x), float(y), LIDAR_HEIGHT))
            # 位置を反映させる
            sim.step(render=False)
            lidar.update(0.02, force_recompute=True)

            hits = lidar.data.ray_hits_w[0]
            origin = lidar.data.pos_w[0]
            finite = torch.isfinite(hits).all(dim=-1)
            if not finite.any():
                # 全方向 30 m 以上 = 極めて開けている（か、シーン外）
                results.append((float(x), float(y), 30.0, 30.0))
                continue
            d = torch.linalg.norm(hits[finite] - origin.unsqueeze(0), dim=-1)
            # 近すぎるものは無視する
            d = d[d > 0.3]
            if d.numel() == 0:
                results.append((float(x), float(y), 30.0, 30.0))
                continue
            results.append(
                (float(x), float(y), float(d.min()), float(d.mean()))
            )

    # 最小距離が大きい順（＝どの方向にも余裕がある）に並べる
    results.sort(key=lambda r: -r[2])
    print("\n[INFO] 開けている地点（最小距離が大きい順、上位 12）:")
    print("        座標              最小距離   平均距離")
    for x, y, dmin, dmean in results[:12]:
        print(f"    ({x:+6.1f}, {y:+6.1f})   {dmin:6.2f} m   {dmean:6.2f} m")

    best = results[0]
    print(
        f"\n[OK] 最も開けた地点: ({best[0]:+.1f}, {best[1]:+.1f}) "
        f"最小距離 {best[2]:.2f} m"
    )
    print("[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
