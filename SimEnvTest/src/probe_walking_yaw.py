"""歩行中に root_quat_w が向きの変化を正しく反映するかを確認する。

Nav2 で旋回指令を出しても /odom の yaw が変わらない問題の切り分け。
Isaac Sim 上では G1 が足踏みしながら旋回しているのに、
/odom は yaw が変わらないと報告していた。

以前 probe_pose.py で「root_quat_w は (w,x,y,z) 順」と確認したが、
あれは静止状態での検証だった。歩行中に姿勢が正しく更新されるかは
別の問題なので、ここで確かめる。

pelvis（root）と torso_link の両方の yaw を追跡し、どちらが
実際の旋回を表しているかを見る。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_walking_yaw.py
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="歩行中の姿勢更新の確認")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

import sys  # noqa: E402

sys.path.insert(0, "/home/spacedata/isaac_dev/G1/SimEnvTest/src")
from g1_twin.checkpoint import resolve_checkpoint  # noqa: E402
from g1_twin.command import VelocityCommand  # noqa: E402
from g1_twin.policy import G1FlatPolicy  # noqa: E402

PHYSICS_DT: float = 0.005
CONTROL_DT: float = 0.02
DECIMATION: int = 4
SPAWN_HEIGHT: float = 0.74
# 与える旋回指令 [rad/s]
TURN_RATE: float = 0.8
# 何 step 回すか（50Hz なので 500 step = シム内 10 秒 = 期待 458 度）
NUM_STEPS: int = 500


def quat_to_yaw(q: np.ndarray) -> float:
    """(x,y,z,w) から yaw [deg] を取り出す（IsaacLab の順序）。"""
    x, y, z, w = (float(v) for v in q)
    return math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


def main() -> None:
    """その場旋回させて姿勢の変化を追う。"""
    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, device=args.device))

    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/GroundPlane", ground)
    light = sim_utils.DomeLightCfg(intensity=1500.0)
    light.func("/World/DomeLight", light)

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(pos=(0.0, 0.0, SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)
    policy = G1FlatPolicy(resolve_checkpoint(""), device=args.device)

    sim.reset()
    robot.reset()
    policy.reset()

    names = robot.data.body_names
    torso_idx = names.index("torso_link")
    pelvis_idx = names.index("pelvis")

    command = VelocityCommand(vx=0.0, vy=0.0, yaw_rate=TURN_RATE)
    print(f"[INFO] 旋回指令 yaw_rate={TURN_RATE} rad/s を {NUM_STEPS} step 与えます")
    print(f"[INFO] 期待される回転量: {math.degrees(TURN_RATE * NUM_STEPS * CONTROL_DT):.0f} 度")

    samples: list[tuple[int, float, float, float]] = []

    for step in range(NUM_STEPS):
        data = robot.data
        obs = policy.build_observation(
            base_lin_vel_b=wp.to_torch(data.root_lin_vel_b)[0],
            base_ang_vel_b=wp.to_torch(data.root_ang_vel_b)[0],
            projected_gravity_b=wp.to_torch(data.projected_gravity_b)[0],
            command=command.as_tuple(),
            joint_pos_rel=(
                wp.to_torch(data.joint_pos) - wp.to_torch(data.default_joint_pos)
            )[0],
            joint_vel=(
                wp.to_torch(data.joint_vel) - wp.to_torch(data.default_joint_vel)
            )[0],
        )
        action = policy.act(obs)
        targets = policy.joint_position_targets(
            action, wp.to_torch(data.default_joint_pos)[0]
        )
        robot.set_joint_position_target(targets.unsqueeze(0))
        robot.write_data_to_sim()
        for _ in range(DECIMATION):
            sim.step(render=False)
        robot.update(CONTROL_DT)

        if step % 50 == 0 or step == NUM_STEPS - 1:
            root_quat = wp.to_torch(robot.data.root_quat_w)[0].cpu().numpy()
            body_quat = wp.to_torch(robot.data.body_quat_w)[0]
            pelvis_yaw = quat_to_yaw(body_quat[pelvis_idx].cpu().numpy())
            torso_yaw = quat_to_yaw(body_quat[torso_idx].cpu().numpy())
            root_yaw = quat_to_yaw(root_quat)
            # 実際の角速度も見る
            ang_z = float(wp.to_torch(robot.data.root_ang_vel_b)[0][2])
            samples.append((step, root_yaw, pelvis_yaw, torso_yaw))
            if step % 200 == 0:
                print(f"        生の root_quat_w = {np.round(root_quat, 4)}")
            print(
                f"    step {step:4d}: root={root_yaw:+7.1f}  "
                f"pelvis={pelvis_yaw:+7.1f}  torso={torso_yaw:+7.1f}  "
                f"ang_vel_z={ang_z:+.3f}"
            )

    first, last = samples[0], samples[-1]
    print("\n[INFO] 回転量（最初 -> 最後）:")
    for label, i in (("root", 1), ("pelvis", 2), ("torso", 3)):
        delta = (last[i] - first[i] + 180.0) % 360.0 - 180.0
        print(f"    {label:7s}: {delta:+7.1f} 度")

    expected = math.degrees(TURN_RATE * NUM_STEPS * CONTROL_DT)
    root_delta = (last[1] - first[1] + 180.0) % 360.0 - 180.0
    print(f"\n[INFO] 期待値: {expected:.0f} 度（360 度を超えるため巻き付く）")
    if abs(root_delta) > 30.0:
        print("[OK] root_quat_w は歩行中の旋回を反映している")
    else:
        print("[NG] root_quat_w が旋回を反映していない -> odom の配信元を見直す")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
