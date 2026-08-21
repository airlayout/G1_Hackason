"""コマンドに対する追従を自動で検証する（キーボード操作なしで確認できる）。

一定時間ごとに (vx, vy, yaw_rate) を切り替え、実際の移動量が指令と整合するかを見る。
観測の組み立てが正しいかを客観的に確認するためのスクリプト。
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 コマンド追従テスト")
parser.add_argument("--checkpoint", type=str, default="", help="checkpoint.pt のパス")
parser.add_argument("--seconds", type=float, default=4.0, help="各コマンドの継続時間 [s]")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from g1_twin.checkpoint import resolve_checkpoint  # noqa: E402
from g1_twin.command import VelocityCommand  # noqa: E402
from g1_twin.runner import CONTROL_DT, DECIMATION, PHYSICS_DT, G1TwinRunner, RunnerConfig  # noqa: E402

# 検証するコマンド列: (説明, コマンド, 期待する主な変化)
TEST_CASES: list[tuple[str, VelocityCommand]] = [
    ("静止", VelocityCommand(0.0, 0.0, 0.0)),
    ("前進", VelocityCommand(0.5, 0.0, 0.0)),
    ("後退0.2", VelocityCommand(-0.2, 0.0, 0.0)),
    ("左移動", VelocityCommand(0.0, 0.4, 0.0)),
    ("右移動", VelocityCommand(0.0, -0.4, 0.0)),
    # 旋回の追従性を確認する（速度を振って傾向を見る）
    ("左旋回0.3", VelocityCommand(0.0, 0.0, 0.3)),
    ("左旋回0.5", VelocityCommand(0.0, 0.0, 0.5)),
    ("左旋回1.0", VelocityCommand(0.0, 0.0, 1.0)),
    ("右旋回0.5", VelocityCommand(0.0, 0.0, -0.5)),
    ("右旋回1.0", VelocityCommand(0.0, 0.0, -1.0)),
    # 前進しながらの旋回（実際の操作に近い）
    ("前進+旋回", VelocityCommand(0.4, 0.0, 0.5)),
]

# 注記: 後退 -0.3 以上は `VelocityCommand.clamped()` で -0.2 に丸められるため
# ここでは検証しない。転倒することは cmd_test6.log で既に確認済み。


def yaw_from_quat(quat: torch.Tensor) -> float:
    """クォータニオン (x, y, z, w) から yaw 角 [rad] を取り出す。

    IsaacLab の root_quat_w は (x, y, z, w) 順である。
    base_articulation_data.py の docstring に明記されており、
    実測でも初期姿勢（無回転）の生値が [~0, ~0, ~0, 1.0] と
    w が最後に来ることを確認した。
    """
    x, y, z, w = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> None:
    """各コマンドを順に与え、実測の速度を報告する。"""
    checkpoint_path = resolve_checkpoint(args.checkpoint)

    sim = SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, device=args.device))
    runner = G1TwinRunner(
        checkpoint_path, RunnerConfig(use_warehouse=False, device=args.device)
    )
    runner.build_scene()
    sim.reset()

    robot = runner._robot  # テスト用に内部を直接参照する
    policy = runner._policy
    robot.reset()
    policy.reset()

    steps_per_case = int(args.seconds / CONTROL_DT)
    print(f"[INFO] 各コマンド {args.seconds}s ({steps_per_case} steps) で検証します")

    # 最初に少し立たせて安定させる
    for _ in range(int(1.0 / CONTROL_DT)):
        _advance(sim, robot, policy, runner, VelocityCommand())

    results: list[str] = []
    for label, raw_command in TEST_CASES:
        # 実際の運用と同じくクランプ後の値を使う。
        # クランプで丸められた場合に「通った」と誤認しないよう明示する。
        command = raw_command.clamped()
        if command.as_tuple() != raw_command.as_tuple():
            print(
                f"[WARN] {label}: 指令が学習範囲にクランプされました "
                f"{raw_command.as_tuple()} -> {command.as_tuple()}"
            )
        # 各コマンドごとに初期状態へ戻す。
        # これをしないと一度転倒した後の結果が全て無意味になる。
        _reset_robot(robot, policy)
        for _ in range(int(1.5 / CONTROL_DT)):
            _advance(sim, robot, policy, runner, VelocityCommand())

        start_pos = wp.to_torch(robot.data.root_pos_w)[0].clone()
        start_yaw = yaw_from_quat(wp.to_torch(robot.data.root_quat_w)[0])

        # yaw の変化を毎ステップ積算する（始点終点比較では
        # 回転して戻る動きや 1 周を超える回転を取りこぼす）
        cumulative_yaw = 0.0
        prev_yaw = start_yaw
        for _ in range(steps_per_case):
            _advance(sim, robot, policy, runner, command)
            now_yaw = yaw_from_quat(wp.to_torch(robot.data.root_quat_w)[0])
            step_dyaw = math.atan2(
                math.sin(now_yaw - prev_yaw), math.cos(now_yaw - prev_yaw)
            )
            cumulative_yaw += step_dyaw
            prev_yaw = now_yaw

        end_pos = wp.to_torch(robot.data.root_pos_w)[0]
        end_yaw = yaw_from_quat(wp.to_torch(robot.data.root_quat_w)[0])

        delta = end_pos - start_pos
        # ワールド座標の移動量を測る（yaw が回るため厳密な胴体座標ではない）
        measured_vx = float(delta[0]) / args.seconds
        measured_vy = float(delta[1]) / args.seconds
        dyaw = math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw))
        measured_yaw_rate = cumulative_yaw / args.seconds
        naive_yaw_rate = dyaw / args.seconds
        height = float(end_pos[2])

        # 胴体座標系での実速度（ポリシーが追従すべき量そのもの）
        body_lin = wp.to_torch(robot.data.root_lin_vel_b)[0]
        body_ang = wp.to_torch(robot.data.root_ang_vel_b)[0]

        fallen = "  <-- 転倒" if height < 0.4 else ""
        line = (
            f"[TEST] {label:6s} 指令=({command.vx:+.2f},{command.vy:+.2f},{command.yaw_rate:+.2f}) "
            f"変位平均=({measured_vx:+.2f},{measured_vy:+.2f},{measured_yaw_rate:+.2f}) "
            f"胴体速度=({float(body_lin[0]):+.2f},{float(body_lin[1]):+.2f},"
            f"{float(body_ang[2]):+.2f}) "
            f"yaw積算={measured_yaw_rate:+.2f}/始終差={naive_yaw_rate:+.2f} "
            f"z={height:.2f}{fallen}"
        )
        print(line)
        results.append(line)

    print("\n[INFO] === まとめ ===")
    for line in results:
        print(line)

    runner.close()


def _reset_robot(robot, policy) -> None:
    """ロボットを初期姿勢・初期位置へ完全に戻す。

    `Articulation.reset()` だけでは転倒した胴体の姿勢が戻らないため、
    ルートの位置・姿勢・速度と関節状態を明示的に書き戻す。
    """
    root_pose = wp.to_torch(robot.data.default_root_pose).clone()
    robot.write_root_pose_to_sim_index(root_pose=root_pose)
    root_vel = wp.to_torch(robot.data.default_root_vel).clone()
    robot.write_root_velocity_to_sim_index(root_velocity=root_vel)

    joint_pos = wp.to_torch(robot.data.default_joint_pos).clone()
    joint_vel = wp.to_torch(robot.data.default_joint_vel).clone()
    robot.write_joint_position_to_sim_index(position=joint_pos)
    robot.write_joint_velocity_to_sim_index(velocity=joint_vel)

    robot.reset()
    policy.reset()


def _advance(sim, robot, policy, runner, command: VelocityCommand) -> None:
    """1 制御ステップ進める。"""
    obs = runner._compute_observation(command)
    action = policy.act(obs)
    default_joint_pos = wp.to_torch(robot.data.default_joint_pos)[0]
    targets = policy.joint_position_targets(action, default_joint_pos)
    robot.set_joint_position_target(targets.unsqueeze(0))
    robot.write_data_to_sim()
    for _ in range(DECIMATION):
        sim.step(render=False)
    robot.update(CONTROL_DT)


if __name__ == "__main__":
    main()
    simulation_app.close()
