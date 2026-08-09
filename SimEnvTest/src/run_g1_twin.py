"""G1 デジタルツイン操作環境のエントリポイント。

Warehouse シーンに配置した G1 を、キーボードの速度指令で歩かせる。

実行方法:
    cd /home/spacedata/isaac_dev/G1/SimEnvTest
    bash run.sh

操作:
    W / S : 前進 / 後退
    A / D : 左移動 / 右移動
    Q / E : 左旋回 / 右旋回
    SPACE : 停止
    SHIFT : 低速（微調整）
"""

from __future__ import annotations

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
parser.add_argument("--x", type=float, default=0.0, help="G1 のスポーン X 座標")
parser.add_argument("--y", type=float, default=0.0, help="G1 のスポーン Y 座標")
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

    config = RunnerConfig(
        use_warehouse=not args.flat,
        spawn_xy=(args.x, args.y),
        device=args.device,
    )
    runner = G1TwinRunner(checkpoint_path, config)
    runner.build_scene()

    # シーン構築後にシミュレーションを初期化する
    sim.reset()

    # カメラを G1 の周辺へ向ける
    sim.set_camera_view(eye=(3.0, 3.0, 2.5), target=(args.x, args.y, 0.8))

    runner.start_keyboard()

    try:
        runner.run(sim, simulation_app)
    finally:
        runner.close()


if __name__ == "__main__":
    main()
    # 参考実装（IsaacLab の tutorials）と同じく、main() の外で閉じる
    simulation_app.close()
