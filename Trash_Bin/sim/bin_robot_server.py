"""ループバックの模擬ロボット（Button_Press/J1-gen の sim/sim_robot_server.py）を、シーンをゴミ箱にして起動する。

実機用のスクリプト（real/push_bin.py）を、当日と同じコマンドに --network-interface lo --camera-config camera_sim.yaml を
付けて、このマシンの中だけで試すためのもの（DDS は lo・domain 1、カメラは 127.0.0.1 だけ。実機とは混ざらない）。

    # 端末 A
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/bin_robot_server.py
    # 端末 B
    G1_HuggingFace/venv/bin/python Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml --locate-only

J1-gen のファイルは変えない。sim_robot_server の中の名前を 2 つ差し替えるだけ:
- load_config("sim_scene.yaml") → ゴミ箱のシーンの設定（configs/bin_scene.yaml）
- SimBackend → BinSimBackend（シーンの組み立てをゴミ箱にしたもの。sim/push_bin_sim.py）
開始の姿勢は J1-gen と同じ検出の姿勢（両腕を下ろしてカメラの視野から外す）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Button_Press" / "J1-gen" / "sim"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from push_bin_sim import BinSimBackend  # noqa: E402  （J1-gen と lib/ のパスも通る）
from bin_scene import load_scene_config  # noqa: E402
import sim_robot_server as srv  # noqa: E402


def main() -> int:
    # ゴミ箱の設定だけ先に読む（--scene）。残りの引数は J1-gen の sim_robot_server にそのまま渡す
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--scene", help="ゴミ箱の設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    args, rest = p.parse_known_args()
    scene = load_scene_config(args.scene)
    orig_load = srv.load_config
    srv.load_config = lambda name: scene if name == "sim_scene.yaml" else orig_load(name)
    srv.SimBackend = BinSimBackend
    sys.argv = [sys.argv[0], *rest]
    return srv.main()


if __name__ == "__main__":
    sys.exit(main())
