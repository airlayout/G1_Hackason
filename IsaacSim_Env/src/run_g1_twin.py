"""G1 デジタルツイン操作環境のエントリポイント。

Warehouse シーンに配置した G1 を、キーボードの速度指令で歩かせる。

実行方法:
    cd <このリポジトリ>/IsaacSim_Env
    bash run.sh

操作:
    W / S : 前進 / 後退
    A / D : 左移動 / 右移動
    Q / E : 左旋回 / 右旋回
    SPACE : 停止
    SHIFT : 低速（微調整）
"""

from __future__ import annotations

import os
import argparse

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 デジタルツイン操作環境")
parser.add_argument(
    "--checkpoint",
    type=str,
    default="",
    help="歩行ポリシーの checkpoint.pt のパス（未指定なら自動ダウンロード）",
)
parser.add_argument(
    "--flat",
    action="store_true",
    help="Warehouse を使わず平地で実行する（動作確認用）",
)
parser.add_argument(
    "--scene-usd",
    type=str,
    default="",
    help=(
        "Warehouse の代わりに読み込む USD ファイルのパス（実測地図から生成した"
        "障害物メッシュ等）。床は含まれない前提で別途平地を敷く。--flat より優先"
    ),
)
parser.add_argument("--x", type=float, default=0.0, help="G1 のスポーン X 座標")
parser.add_argument("--y", type=float, default=0.0, help="G1 のスポーン Y 座標")
parser.add_argument(
    "--ros",
    action="store_true",
    help="ROS 2 へ /scan と /odom を配信する（SLAM / Nav2 で使う）",
)
parser.add_argument(
    "--command-source",
    type=str,
    default="keyboard",
    choices=("keyboard", "patrol", "ros", "goto"),
    help=(
        "速度指令の供給源。keyboard: 手動操作 / "
        "patrol: LiDAR を見て自動巡回（SLAM 用）/ ros: Nav2 の /cmd_vel / "
        "goto: --goto-x/--goto-y の座標へ障害物回避なしで直進（Nav2 不要）"
    ),
)
parser.add_argument(
    "--patrol-seed", type=int, default=0, help="自動巡回の乱数種（再現性のため）"
)
parser.add_argument(
    "--goto-x", type=float, default=0.0, help="--command-source goto の目標 X 座標"
)
parser.add_argument(
    "--goto-y", type=float, default=0.0, help="--command-source goto の目標 Y 座標"
)
parser.add_argument(
    "--max-steps",
    type=int,
    default=0,
    help="この制御ステップ数で自動終了する（0 なら無制限）。自動巡回の時間制限に使う",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- ここから下は Isaac Sim 起動後にのみ import 可能 ---
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from g1_twin.checkpoint import resolve_checkpoint  # noqa: E402
from g1_twin.runner import CONTROL_DT, PHYSICS_DT, G1TwinRunner, RunnerConfig  # noqa: E402


def main() -> None:
    """シーンを構築してシミュレーションを実行する。"""
    checkpoint_path = resolve_checkpoint(args.checkpoint)

    # 物理シミュレーションの設定（学習時と同じ刻み）
    sim_cfg = sim_utils.SimulationCfg(dt=PHYSICS_DT, device=args.device)
    sim = SimulationContext(sim_cfg)

    # 自動巡回・Nav2 連携には LiDAR と ROS が要るので暗黙に有効化する
    enable_ros = args.ros or args.command_source in ("patrol", "ros")

    config = RunnerConfig(
        use_warehouse=not args.flat,
        scene_usd_path=args.scene_usd,
        spawn_xy=(args.x, args.y),
        device=args.device,
        enable_ros=enable_ros,
        enable_camera=args.enable_cameras,
        command_source=args.command_source,
        goto_xy=(args.goto_x, args.goto_y),
        patrol_seed=args.patrol_seed,
        max_steps=args.max_steps,
        # ビューアのカメラを G1 に追従させる（既定オフ）。
        # 長い距離を歩かせると起動時の固定カメラでは画角から出てしまうため、
        # 見せる・録画するときだけ環境変数で有効にする。
        # 他の設定（SCENE_USD / SPAWN_X / G1_PERFECT_LOC）と同じ流儀。
        follow_camera=os.environ.get("G1_FOLLOW_CAM", "0") == "1",
    )
    runner = G1TwinRunner(checkpoint_path, config)
    runner.build_scene()

    # シーン構築後にシミュレーションを初期化する
    sim.reset()

    # カメラを G1 の周辺へ向ける
    sim.set_camera_view(eye=(3.0, 3.0, 2.5), target=(args.x, args.y, 0.8))

    # 指令の供給源に応じて必要なものだけ起動する
    if args.command_source == "keyboard":
        runner.start_keyboard()
    elif args.command_source == "patrol":
        runner.start_patrol()
    elif args.command_source == "goto":
        runner.start_goto()

    if enable_ros:
        runner.start_ros()

    try:
        runner.run(sim, simulation_app)
    finally:
        runner.close()


if __name__ == "__main__":
    main()
    # 参考実装（IsaacLab の tutorials）と同じく、main() の外で閉じる
    simulation_app.close()
