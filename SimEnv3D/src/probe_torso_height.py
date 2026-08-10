"""torso_link の実際の高さと LiDAR の設置高さを確認する。

check_lidar3d.py が「センサ高さ 0.75 m」と報告した。lidar3d.py は
TORSO_HEIGHT=0.753 を前提に offset +0.347 を足して地上 1.1 m に置くつもり
だったので、想定と 0.35 m 食い違っている。

考えられる原因:
    a) torso_link が実際には 0.40 m 付近にあり、+0.347 して 0.75 m になった
    b) offset が効いていない（0.753 のまま）
    どちらなのかで修正内容が変わるため、実測して切り分ける。

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    "$ISAAC_SIM/python.sh" src/probe_torso_height.py --viz none
"""

from __future__ import annotations

import argparse

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="torso_link と LiDAR の高さ確認")
parser.add_argument("--settle-steps", type=int, default=120, help="姿勢を安定させるステップ数")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後に import する ---
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

from g1_twin.lidar3d import G1Lidar3D, LIDAR_OFFSET_Z, TORSO_HEIGHT  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
SPAWN_HEIGHT: float = 0.80


def main() -> None:
    """torso_link の高さと LiDAR の設置高さを比べる。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[Probe] アセットサーバーに接続できません。")
    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
    light.func("/World/DomeLight", light)

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(pos=(0.0, 0.0, SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)

    lidar = G1Lidar3D(robot_prim_path="/World/G1", mesh_prim_paths=["/World/Warehouse"])

    sim.reset()

    dt = sim.get_physics_dt()
    for _ in range(args.settle_steps):
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    lidar.update(dt)

    # body 名から torso_link の index を引く
    body_names = robot.data.body_names
    pos_w = wp.to_torch(robot.data.body_pos_w)[0]  # (num_bodies, 3)

    print()
    print("=" * 70)
    print("torso_link と LiDAR の高さ")
    print("=" * 70)
    print(f"lidar3d.py の前提: TORSO_HEIGHT={TORSO_HEIGHT}, offset={LIDAR_OFFSET_Z:+.3f}")
    print(f"  -> 期待する設置高さ: {TORSO_HEIGHT + LIDAR_OFFSET_Z:.3f} m")
    print()

    # 主要な body の高さを出す
    for name in ("pelvis", "torso_link", "waist_yaw_link", "head_link"):
        if name in body_names:
            idx = body_names.index(name)
            print(f"  {name:20s} z = {float(pos_w[idx, 2]):.3f} m")

    root_z = float(wp.to_torch(robot.data.root_pos_w)[0][2])
    sensor_z = float(lidar.position_w[2])
    print()
    print(f"  root (pelvis)         z = {root_z:.3f} m")
    print(f"  LiDAR (実測)          z = {sensor_z:.3f} m")
    print(f"  期待                  z = {TORSO_HEIGHT + LIDAR_OFFSET_Z:.3f} m")
    print(f"  差                      = {sensor_z - (TORSO_HEIGHT + LIDAR_OFFSET_Z):+.3f} m")
    print()

    if "torso_link" in body_names:
        torso_z = float(pos_w[body_names.index("torso_link"), 2])
        print(f"[判定] torso_link の実測高さ: {torso_z:.3f} m")
        if abs(torso_z - TORSO_HEIGHT) > 0.05:
            print(f"  -> TORSO_HEIGHT={TORSO_HEIGHT} は誤り。実測は {torso_z:.3f} m。")
            correct_offset = 1.1 - torso_z
            print(f"  -> 地上 1.1 m に置くには offset = {correct_offset:+.3f} が必要。")
        else:
            print(f"  -> TORSO_HEIGHT は妥当。offset の適用を確認すること。")
        # offset が効いているかを見る
        print(f"[判定] LiDAR - torso_link = {sensor_z - torso_z:+.3f} m "
              f"(offset {LIDAR_OFFSET_Z:+.3f} が効いていれば一致)")

    print("=" * 70)


main()

simulation_app.close()
