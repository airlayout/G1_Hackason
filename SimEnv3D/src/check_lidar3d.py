"""3D LiDAR が正しい点群を返すかを Isaac Sim 上で検証する。

確認すること:
    1. 期待した本数のレイが出ているか
    2. 前傾によって足元（近くの床）が見えているか
       -> 2D LiDAR（水平・地上 1.1 m）では見えなかった部分
    3. 上方向も見えているか（Mid-360 は +52 度まで）
    4. センサ座標系への変換が正しいか（ワールドとの往復）

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    "$ISAAC_SIM/python.sh" src/check_lidar3d.py --viz none
"""

from __future__ import annotations

import argparse

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="3D LiDAR の点群検証")
parser.add_argument("--settle-steps", type=int, default=60, help="姿勢を安定させるステップ数")
parser.add_argument("--tilt", type=float, default=20.0, help="前傾角 [deg]")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後に import する ---
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

from g1_twin.lidar3d import G1Lidar3D, TARGET_LIDAR_HEIGHT  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
SPAWN_HEIGHT: float = 0.80

PASS = "[OK]"
FAIL = "[NG]"


def main() -> None:
    """3D LiDAR を構築して点群の性質を検証する。"""
    failures = 0

    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[Check] アセットサーバーに接続できません。")
    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
    light.func("/World/DomeLight", light)

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(pos=(0.0, 0.0, SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)

    lidar = G1Lidar3D(
        robot_prim_path="/World/G1",
        mesh_prim_paths=["/World/Warehouse"],
        tilt_deg=args.tilt,
    )

    sim.reset()

    dt = sim.get_physics_dt()
    for _ in range(args.settle_steps):
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    lidar.update(dt)

    print()
    print("=" * 70)
    print("3D LiDAR の点群検証")
    print("=" * 70)

    # --- 1. レイ本数と当たり率 ---
    cloud = lidar.read_point_cloud()
    print(f"[Test] レイ本数と当たり率")
    print(f"  発射   : {cloud.num_beams} 本")
    print(f"  有効   : {cloud.num_valid} 点 ({cloud.hit_ratio * 100:.1f}%)")
    if cloud.num_valid > 0:
        print(f"  {PASS} 点群が取得できている")
    else:
        failures += 1
        print(f"  {FAIL} 点が 1 つも無い")

    # --- 2. 足元が見えているか（ワールド座標で低い点があるか）---
    points_w = lidar.read_points_world()
    sensor_z = float(lidar.position_w[2])
    print(f"[Test] 足元の可視性（センサ高さ {sensor_z:.2f} m）")
    if points_w.shape[0] > 0:
        z_w = points_w[:, 2]
        low = int((z_w < 0.3).sum())  # 地上 0.3 m 未満 = 床やパレットの高さ
        print(f"  最低点 : z = {float(z_w.min()):.2f} m")
        print(f"  最高点 : z = {float(z_w.max()):.2f} m")
        print(f"  地上 0.3 m 未満の点: {low} 点")
        if low > 0:
            print(f"  {PASS} 足元（低い障害物・床）が見えている")
            # 一番近い低い点までの水平距離を出す（何 m 先から床が見えるか）
            low_mask = z_w < 0.3
            horiz = torch.linalg.norm(
                points_w[low_mask][:, :2] - lidar.position_w[:2].unsqueeze(0), dim=-1
            )
            print(f"  [INFO] 最も近い低い点までの水平距離: {float(horiz.min()):.2f} m")
        else:
            failures += 1
            print(f"  {FAIL} 低い点が無い（前傾が効いていない可能性）")

        # --- 3. 上方向も見えているか ---
        print(f"[Test] 上方向の可視性")
        high = int((z_w > sensor_z + 0.5).sum())
        print(f"  センサより 0.5 m 以上高い点: {high} 点")
        if high > 0:
            print(f"  {PASS} 上方（棚の上段・天井）が見えている")
        else:
            failures += 1
            print(f"  {FAIL} 高い点が無い")
    else:
        failures += 2
        print(f"  {FAIL} ワールド点群が空")

    # --- 4. センサ座標系への変換の検証 ---
    print(f"[Test] センサ座標系への変換")
    if cloud.num_valid > 0 and points_w.shape[0] == cloud.num_valid:
        # センサ座標の点の距離とワールド座標での距離が一致するはず
        dist_sensor = torch.linalg.norm(cloud.points_sensor, dim=-1)
        dist_world = torch.linalg.norm(
            points_w - lidar.position_w.unsqueeze(0), dim=-1
        )
        max_diff = float((dist_sensor - dist_world).abs().max())
        if max_diff < 1e-3:
            print(f"  {PASS} 距離が一致（最大差 {max_diff:.6f} m）")
        else:
            failures += 1
            print(f"  {FAIL} 距離が一致しない（最大差 {max_diff:.6f} m）")

        # 前方の点はセンサ座標で x が正のはず
        forward = int((cloud.points_sensor[:, 0] > 0).sum())
        print(f"  [INFO] センサ座標で x>0（前方）の点: {forward} / {cloud.num_valid}")
    else:
        failures += 1
        print(f"  {FAIL} 点数が一致しない")

    print("=" * 70)
    if failures:
        print(f"{FAIL} {failures} 件が失敗しました")
    else:
        print(f"{PASS} すべて成功しました")


main()

simulation_app.close()
