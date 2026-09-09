"""レイの実際の発射位置を確認する（offset が効いているか）。

probe_torso_height.py で pos_w が offset を含まない（torso_link と同じ 0.745 m）
ことが分かった。IsaacLab の ray_caster.py を読むと:

    line 217: self.ray_starts += offset_pos     # レイの始点には効く
    line 233: pos_w = obtain_world_pose_from_view(...)  # cfg.offset を含まない

つまり **レイは offset ぶん高い位置から出ているが、pos_w はそれを含まない**
可能性がある。もしそうなら、pos_w を原点として距離を計算している
lidar.py / lidar3d.py の距離が offset ぶん誤る。

ここでは ray_starts_w（実際の発射位置）を直接見て確認する。

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    "$ISAAC_SIM/python.sh" src/probe_ray_origin.py --viz none
"""

from __future__ import annotations

import argparse

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="レイの発射位置の確認")
parser.add_argument("--settle-steps", type=int, default=120, help="姿勢を安定させるステップ数")
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

from g1_twin.lidar3d import G1Lidar3D, LIDAR_OFFSET_Z  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
SPAWN_HEIGHT: float = 0.80


def main() -> None:
    """レイの発射位置と pos_w を比べる。"""
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

    sensor = lidar._sensor  # 内部を直接見る（検証目的）
    pos_w = sensor.data.pos_w[0]

    print()
    print("=" * 70)
    print("レイの発射位置と pos_w")
    print("=" * 70)
    print(f"cfg.offset.pos の z   : {LIDAR_OFFSET_Z:+.3f} m")
    print(f"data.pos_w            : z = {float(pos_w[2]):.3f} m")

    # ray_starts はセンサ座標系での始点（offset が足されている）
    if hasattr(sensor, "ray_starts"):
        rs = sensor.ray_starts[0] if sensor.ray_starts.ndim == 3 else sensor.ray_starts
        print(f"ray_starts[0] (ローカル): {[round(float(v), 3) for v in rs[0]]}")
        print(f"  ray_starts の z の一意な値: "
              f"{sorted(set(round(float(v), 3) for v in rs[:, 2]))[:5]}")

    # 真下を向くレイの当たり点から、実際の発射高さを逆算する。
    # 床は z=0 なので、真下のレイの距離がそのまま発射高さになる。
    hits_w = sensor.data.ray_hits_w[0]
    dirs = sensor.ray_directions[0] if sensor.ray_directions.ndim == 3 else sensor.ray_directions

    # 最も下を向いているレイ
    lowest = int(torch.argmin(dirs[:, 2]))
    hit = hits_w[lowest]
    print()
    print(f"最も下を向くレイ: dir={[round(float(v), 3) for v in dirs[lowest]]}")
    print(f"  当たり点  : {[round(float(v), 3) for v in hit]}")
    print(f"  当たり点の z: {float(hit[2]):.3f} m（床なら 0 付近）")

    # そのレイの発射位置を逆算: hit = start + t * dir、床(z=0)に当たったなら
    # start_z = -t * dir_z + 0 なので、水平距離から t を出す
    if float(hit[2]) < 0.1:  # 床に当たっている
        horiz = float(torch.linalg.norm(hit[:2] - pos_w[:2]))
        dir_z, dir_horiz = float(dirs[lowest, 2]), float(
            torch.linalg.norm(dirs[lowest, :2])
        )
        if dir_horiz > 1e-6:
            t = horiz / dir_horiz
            start_z = float(hit[2]) - t * dir_z
            print()
            print(f"[逆算] 床への当たりから求めた発射高さ: {start_z:.3f} m")
            print(f"  pos_w の z                        : {float(pos_w[2]):.3f} m")
            print(f"  差                                : {start_z - float(pos_w[2]):+.3f} m")
            if abs(start_z - float(pos_w[2]) - LIDAR_OFFSET_Z) < 0.05:
                print(f"  -> offset {LIDAR_OFFSET_Z:+.3f} がレイに効いている。")
                print(f"     **pos_w は offset を含まないので距離計算に使うと誤る。**")
            elif abs(start_z - float(pos_w[2])) < 0.05:
                print(f"  -> レイも pos_w と同じ高さ。offset が全く効いていない。")
            else:
                print(f"  -> どちらとも一致しない。要調査。")

    print("=" * 70)


main()

simulation_app.close()
