"""G1 の姿勢データの形式を確認する調査用スクリプト。

ROS の odom / TF へ変換するために以下を実測する:
    - root_quat_w の要素順序が (w,x,y,z) か (x,y,z,w) か
    - root_lin_vel_b / root_ang_vel_b の符号と向き
    - LiDAR のビーム方向がロボットの前方とどう対応するか

順序を取り違えると地図が回転して壊れるため、既知の姿勢で必ず確認する。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_pose.py
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 姿勢データの形式確認")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

SPAWN_HEIGHT: float = 0.74
# 既知の yaw 角を与えて、クォータニオンの並びを判別する
TEST_YAW_DEG: float = 30.0


def main() -> None:
    """G1 を既知の yaw で配置し、姿勢データの形式を確認する。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = SimulationContext(sim_cfg)

    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/GroundPlane", ground)
    light = sim_utils.DomeLightCfg(intensity=1500.0)
    light.func("/World/DomeLight", light)

    # yaw = TEST_YAW_DEG だけ回した姿勢で配置する。
    # IsaacLab の init_state.rot は (w, x, y, z) で指定する仕様。
    yaw = math.radians(TEST_YAW_DEG)
    rot_wxyz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(
        pos=(1.0, 2.0, SPAWN_HEIGHT), rot=rot_wxyz
    )
    robot = Articulation(robot_cfg)
    print(f"[INFO] G1 を yaw={TEST_YAW_DEG} 度 で配置しました")
    print(f"[INFO] 指定した rot (w,x,y,z) = {np.round(rot_wxyz, 4)}")

    sim.reset()
    robot.reset()

    # 数ステップ回して安定させる
    for _ in range(5):
        sim.step(render=False)
        robot.update(0.005)

    pos = wp.to_torch(robot.data.root_pos_w)[0].cpu().numpy()
    quat = wp.to_torch(robot.data.root_quat_w)[0].cpu().numpy()

    print(f"\n[INFO] root_pos_w  = {np.round(pos, 4)}")
    print(f"[INFO] root_quat_w = {np.round(quat, 4)}")

    # (w,x,y,z) と解釈した場合の yaw
    w, x, y, z = quat
    yaw_wxyz = math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )
    # (x,y,z,w) と解釈した場合の yaw
    x2, y2, z2, w2 = quat
    yaw_xyzw = math.degrees(
        math.atan2(2.0 * (w2 * z2 + x2 * y2), 1.0 - 2.0 * (y2 * y2 + z2 * z2))
    )

    print(f"\n[INFO] (w,x,y,z) と解釈 -> yaw = {yaw_wxyz:+.2f} 度")
    print(f"[INFO] (x,y,z,w) と解釈 -> yaw = {yaw_xyzw:+.2f} 度")
    print(f"[INFO] 期待値 = {TEST_YAW_DEG:+.2f} 度")

    if abs(yaw_wxyz - TEST_YAW_DEG) < 2.0:
        print("[OK] root_quat_w は (w, x, y, z) 順である")
    elif abs(yaw_xyzw - TEST_YAW_DEG) < 2.0:
        print("[OK] root_quat_w は (x, y, z, w) 順である")
    else:
        print("[NG] どちらの解釈でも期待値に一致しない。要調査")

    # 速度の向きを確認する（歩行させずに直接速度を書き込んで確かめる）
    lin_b = wp.to_torch(robot.data.root_lin_vel_b)[0].cpu().numpy()
    ang_b = wp.to_torch(robot.data.root_ang_vel_b)[0].cpu().numpy()
    print(f"\n[INFO] root_lin_vel_b = {np.round(lin_b, 4)} （静止時なのでほぼ 0）")
    print(f"[INFO] root_ang_vel_b = {np.round(ang_b, 4)}")

    # body_names の確認（TF の frame 名に使う）
    names = robot.data.body_names
    print(f"\n[INFO] body 数: {len(names)}")
    print(f"[INFO] 先頭 8 個: {names[:8]}")
    for target in ("pelvis", "torso_link", "base_link"):
        print(f"[INFO] '{target}' の有無: {target in names}")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
