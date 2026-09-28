"""MuJoCo（胴体固定）で、腕を指定の関節角へ動かして戻す（タスク1の確認用）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/move_arm_sim.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/move_arm_sim.py --delta-deg -30 0 0 20 0 0 0
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/move_arm_sim.py --emulate lowcmd --no-gravity-comp

画面は出さない（ヘッドレス）。--realtime を付けると実時間で進むので、Ctrl+C の安全終了を試せる。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm.run_move import add_common_args, run  # noqa: E402
from common.config import load_config  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p, default_delta=[-20.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0])
    p.add_argument("--emulate", choices=["arm_sdk", "lowcmd"], help="近似する経路（既定は設定ファイル）")
    p.add_argument("--no-gravity-comp", action="store_true", help="重力補償を切る（実機の lowcmd に近い）")
    p.add_argument("--realtime", action="store_true", help="実時間で進める")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    args = p.parse_args()

    sim_cfg = dict(load_config(args.arm_config)["sim"])
    if args.emulate:
        sim_cfg["emulate"] = args.emulate
    if args.no_gravity_comp:
        sim_cfg["gravity_compensation"] = False
    if args.realtime:
        sim_cfg["realtime"] = True
    return run(args, path="sim", dry_run=False, confirm=args.confirm, overrides={"sim": sim_cfg})


if __name__ == "__main__":
    sys.exit(main())
