"""歩行中の torso_link (LiDAR 搭載部位) と pelvis (base_link) の姿勢差を実測する。

ステップ 4「"base" vs "yaw" の比較」のための調査スクリプト。

背景:
    ROS 側の TF 構成は次の通り（ros_bridge.py 参照）:
        動的 TF   odom -> base_link       pelvis の実測姿勢（歩行中も追従）
        静的 TF   base_link -> lidar3d    固定の取り付け位置 + 前傾のみ

    LiDAR は base_link (pelvis) ではなく torso_link に付いている。
    静的 TF は「torso_link は pelvis に対して常に一定姿勢」という前提を
    置いているが、歩行中は torso が pelvis に対して pitch/roll/yaw 方向に
    揺れる可能性がある。この揺れが大きいと、octomap が積む点群の位置が
    実際とずれ、地図にノイズとして乗る。

    このスクリプトは、その揺れ（torso の pelvis に対する相対回転）を
    実際に歩かせて測り、代表的な計測距離での位置誤差に換算する。
    誤差が実用上無視できる大きさかどうかで、静的 TF のままでよいか
    （動的 TF や ray_alignment="yaw" への変更が要るか）を判断する。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_walk_tilt.py
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="torso と pelvis の姿勢差（歩行中）の実測")
parser.add_argument("--steps", type=int, default=500, help="計測ステップ数（安定後）")
parser.add_argument("--settle-steps", type=int, default=100, help="歩行が安定するまでのステップ数")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"] + sys.argv[1:])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from g1_twin.checkpoint import resolve_checkpoint  # noqa: E402
from g1_twin.command import VelocityCommand  # noqa: E402
from g1_twin.policy import G1FlatPolicy  # noqa: E402

SPAWN_HEIGHT: float = 0.74
PHYSICS_DT: float = 0.005
CONTROL_DT: float = 0.02
DECIMATION: int = 4

# 代表的な計測距離 [m]（build_map_3d.py / octomap で実際に使う範囲）
SAMPLE_DISTANCES: tuple[float, ...] = (2.0, 5.0, 10.0)


def quat_mul_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """(x, y, z, w) 順のクォータニオン同士を掛け合わせる（Hamilton 積）。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])


def quat_conj_xyzw(q: np.ndarray) -> np.ndarray:
    """単位クォータニオンの共役（= 逆回転）を返す。(x, y, z, w) 順。"""
    x, y, z, w = q
    return np.array([-x, -y, -z, w])


def quat_xyzw_to_rpy_deg(q: np.ndarray) -> tuple[float, float, float]:
    """(x, y, z, w) 順のクォータニオンから roll/pitch/yaw [deg] を取り出す。

    IsaacLab の root_quat_w / body_quat_w は (x, y, z, w) 順
    （`base_articulation_data.py` の `QUAT_XYZW_ELEMENT_NAMES` で確認済み）。
    `src/probe_torso_yaw.py` は `w, x, y, z = quat` と誤って展開しており
    （(w,x,y,z) 順だと誤解していた）、値が正しくない可能性があるため
    ここでは使わず、xyzw 順で正しく展開し直す。
    """
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main() -> None:
    """前進させながら torso の pelvis に対する相対姿勢を測る。"""
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

    names = robot.data.body_names
    torso_idx = names.index("torso_link")
    print(f"[INFO] torso_link index={torso_idx}")

    command = VelocityCommand(vx=0.5, vy=0.0, yaw_rate=0.0)
    samples: list[tuple[float, float, float]] = []  # (roll, pitch, yaw) [deg]

    total_steps = args.settle_steps + args.steps
    for step in range(total_steps):
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

        if step < args.settle_steps:
            continue

        pelvis_quat = wp.to_torch(robot.data.root_quat_w)[0].cpu().numpy()
        torso_quat = wp.to_torch(robot.data.body_quat_w)[0][torso_idx].cpu().numpy()

        # torso の姿勢を pelvis 基準に変換した相対回転（= 静的 TF が無視する分）
        q_rel = quat_mul_xyzw(quat_conj_xyzw(pelvis_quat), torso_quat)
        roll, pitch, yaw = quat_xyzw_to_rpy_deg(q_rel)
        samples.append((roll, pitch, yaw))

    arr = np.array(samples)
    roll_arr, pitch_arr, yaw_arr = arr[:, 0], arr[:, 1], arr[:, 2]

    print(f"\n[INFO] 計測サンプル数: {len(samples)}")
    for name, a in (("roll", roll_arr), ("pitch", pitch_arr), ("yaw", yaw_arr)):
        print(
            f"[INFO] torso の pelvis に対する {name}: "
            f"平均 {a.mean():+.3f} 度 / 標準偏差 {a.std():.3f} 度 / "
            f"範囲 {a.min():+.3f} 〜 {a.max():+.3f} 度"
        )

    # pitch/roll の振れ幅（静的 TF が torso を pelvis と同一視することで
    # 生じる、最大の姿勢誤差）を、代表距離での位置誤差に換算する。
    # 誤差 ≈ 距離 * sin(角度)（小角近似ではなく厳密な sin を使う）
    tilt_amplitude_deg = float(max(roll_arr.max() - roll_arr.min(), pitch_arr.max() - pitch_arr.min()))
    print(f"\n[INFO] pitch/roll の最大振れ幅: {tilt_amplitude_deg:.3f} 度")
    print("[INFO] この振れ幅による代表距離での位置誤差（静的 TF を使う場合）:")
    for dist in SAMPLE_DISTANCES:
        err = dist * math.sin(math.radians(tilt_amplitude_deg))
        print(f"       {dist:5.1f} m 先 -> ±{err:.3f} m")

    print()
    if tilt_amplitude_deg > 3.0:
        print(
            "[NG] torso は pelvis に対して無視できない大きさで揺れている。"
            "静的 TF (base_link -> lidar3d) では歩行中の誤差が大きい。"
            "動的 TF にするか ray_alignment=\"yaw\" への変更を検討すること。"
        )
    else:
        print(
            "[OK] torso の pelvis に対する揺れは小さい。"
            "静的 TF (base_link -> lidar3d) のままで実用上問題ない。"
        )

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
