"""エージェントを、実機と同じ口（DDS + ZMQ のカメラ）で動かす。模擬 G1 でも実機でも同じコマンド。

    P=~/miniconda3/envs/lerobot/bin/python
    # 模擬 G1（先に別のターミナルで sim/mujoco/g1_sim_server.py --seed N を起動しておく）
    $P Button_Press/Yada/contest/run_dds.py --agent Button_Press/Yada/contest/example_agent
    # 実機（ラボ PC から有線で。指示は引数で渡す）
    $P Button_Press/Yada/contest/run_dds.py --agent ... --network-interface <NIC> --camera-host <PC2> \\
        --target up --instruction "上のボタンを押して" --dry-run

ランナー（contest/runner.py）とエージェントは、Python の API（evaluate.py）と同じものを使う。
ロボットだけが contest/robots/real_g1.py（arm_sdk + LocoClient + RGB-D）になる。
口が lo なら模擬 G1 につなぐ（指示は模擬 G1 が書いた task.json から読む。判定も模擬 G1 が行う）。

⚠️ 実機では、まず --dry-run（指令を送らない）で動かし、J1-gen の real/REAL_DAY_PROCEDURE.md の手順に従うこと。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config  # noqa: E402
from contest.interface import load_agent  # noqa: E402
from contest.runner import run_episode  # noqa: E402

# 模擬 G1 が書く指示のファイル（contest/evaluate_dds.py から起動されたときは、環境変数 YADA_TASK_FILE で渡される）
TASK_FILE = Path(os.environ.get("YADA_TASK_FILE", REPO_ROOT / "_local" / "button_press_yada" / "sim_server" / "task.json"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--agent", required=True, help="agent.py か、それがあるフォルダ")
    ap.add_argument("--network-interface", default="lo", help="lo なら模擬 G1。実機は G1 につないでいる有線 NIC の名前")
    ap.add_argument("--camera-host", default="127.0.0.1", help="RGB-D サーバのアドレス（実機は PC2 の IP）")
    ap.add_argument("--rgbd-port", type=int, default=5556)
    ap.add_argument("--target", choices=["up", "down"], help="押すボタン（実機用。模擬 G1 では task.json から読む）")
    ap.add_argument("--instruction", default="", help="指示の文（実機用）")
    ap.add_argument("--dry-run", action="store_true", help="指令を送らない（実機で最初に試すとき）")
    ap.add_argument("--no-loco", action="store_true", help="下半身の LocoClient を使わない")
    args = ap.parse_args()

    contest_cfg = load_config("contest.yaml")
    if args.network_interface == "lo" and args.target is None:
        task = json.loads(TASK_FILE.read_text())
        print(f"[run_dds] 模擬 G1 の指示: seed {task['seed']}、「{task['instruction']}」")
        trial = SimpleNamespace(seed=task["seed"], target=task["target"], instruction=task["instruction"])
        contest_cfg = {**contest_cfg, "time_limit_s": float(task["time_limit_s"])}
    else:
        if args.target is None:
            ap.error("実機では --target が要る")
        trial = SimpleNamespace(seed=-1, target=args.target, instruction=args.instruction or args.target)

    from contest.robots.real_g1 import RealG1Robot, StateTimeoutError

    agent = load_agent(args.agent)
    robot = RealG1Robot(contest_cfg, args.network_interface, args.camera_host, args.rgbd_port,
                        dry_run=args.dry_run, use_loco=not args.no_loco)
    try:
        res = run_episode(robot, agent, trial, contest_cfg, verbose=True)
    except StateTimeoutError as e:
        # 模擬 G1 は判定のあと少しして止まるので、そのあとはここに来る（実機なら通信が切れた）
        print(f"[run_dds] {e}（模擬 G1 なら、判定が済んでサーバが止まった）")
        return 0
    finally:
        robot.close()
        agent.close()
    print(f"[run_dds] エージェント側の結果: {res.outcome}（{res.steps} 周期、act() の平均 {res.act_time_mean_ms} ms）。"
          "点灯の判定は、模擬 G1 ならサーバの result_seed*.json、実機なら人が行う")
    if res.error:
        print(res.error)
    return 0


if __name__ == "__main__":
    sys.exit(main())
