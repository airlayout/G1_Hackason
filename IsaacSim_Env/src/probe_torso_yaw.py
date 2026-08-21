"""torso_link が pelvis に対してどれだけ回るかを実測する調査用スクリプト。

地図が壊れる原因の切り分け。

odom / TF は pelvis（root）の姿勢を配信しているが、LiDAR は torso_link に
取り付けてある。歩行中に torso が pelvis に対して yaw 方向へ回ると、
スキャンの実際の向きと TF が示す向きがずれ、SLAM はスキャンを正しく
重ねられない。そのずれの大きさを測る。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_torso_yaw.py
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="torso と pelvis の姿勢差の実測")
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

import pathlib
import sys  # noqa: E402

# このファイルの位置から src/ を求める（リポジトリの置き場所に依存しない）
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from g1_twin.checkpoint import resolve_checkpoint  # noqa: E402
from g1_twin.command import VelocityCommand  # noqa: E402
from g1_twin.policy import G1FlatPolicy  # noqa: E402

SPAWN_HEIGHT: float = 0.74
PHYSICS_DT: float = 0.005
CONTROL_DT: float = 0.02
DECIMATION: int = 4


def quat_to_yaw(q: np.ndarray) -> float:
    """(w,x,y,z) から yaw [rad] を取り出す。"""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> None:
    """前進させながら pelvis と torso の yaw 差を測る。"""
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
    print(f"[INFO] torso_link index={torso_idx} / pelvis index={pelvis_idx}")

    command = VelocityCommand(vx=0.5, vy=0.0, yaw_rate=0.0)
    yaw_diffs: list[float] = []
    pitch_rolls: list[tuple[float, float]] = []

    for step in range(500):
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

        # 100 step 以降（歩行が安定してから）を計測する
        if step < 100:
            continue

        root_quat = wp.to_torch(robot.data.root_quat_w)[0].cpu().numpy()
        body_quat = wp.to_torch(robot.data.body_quat_w)[0]
        torso_quat = body_quat[torso_idx].cpu().numpy()

        root_yaw = quat_to_yaw(root_quat)
        torso_yaw = quat_to_yaw(torso_quat)
        # -180..180 に正規化した差
        diff = math.degrees(
            (torso_yaw - root_yaw + math.pi) % (2 * math.pi) - math.pi
        )
        yaw_diffs.append(diff)

        # torso の pitch / roll も見る（レイが上下を向く原因になるため）
        w, x, y, z = torso_quat
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))
        roll = math.degrees(
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        )
        pitch_rolls.append((pitch, roll))

    diffs = np.array(yaw_diffs)
    print(f"\n[INFO] 計測サンプル数: {diffs.size}")
    print(
        f"[INFO] torso と pelvis の yaw 差: "
        f"平均 {diffs.mean():+.2f} 度 / 標準偏差 {diffs.std():.2f} 度"
    )
    print(f"[INFO] yaw 差の範囲: {diffs.min():+.2f} 〜 {diffs.max():+.2f} 度")
    amplitude = diffs.max() - diffs.min()
    print(f"[INFO] yaw 差の振れ幅: {amplitude:.2f} 度")

    pr = np.array(pitch_rolls)
    print(
        f"[INFO] torso の pitch: {pr[:, 0].min():+.2f} 〜 {pr[:, 0].max():+.2f} 度"
    )
    print(f"[INFO] torso の roll:  {pr[:, 1].min():+.2f} 〜 {pr[:, 1].max():+.2f} 度")

    # 判定: 振れ幅が大きいと 30 m 先で数 m の誤差になる
    error_at_30m = 30.0 * math.tan(math.radians(amplitude / 2.0))
    print(f"\n[INFO] この振れ幅は 30 m 先で ±{error_at_30m:.2f} m の誤差になる")
    if amplitude > 5.0:
        print(
            "[NG] torso が pelvis に対して大きく回っている。"
            "LiDAR は pelvis に付けるべき"
        )
    else:
        print("[OK] torso の回転は小さい。地図が壊れる主因ではない")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
