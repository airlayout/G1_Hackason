"""Isaac Sim でエージェントを評価する。evaluate_isaac.sh から起動する（直接は起動しない）。

    bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/Yada/contest/example_agent --seeds smoke
    bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/<チーム名> --seeds practice
    bash Button_Press/Yada/contest/evaluate_isaac.sh --agent ... --seed 3 --viz kit     # 画面で見る

Isaac Sim のアプリを 1 回だけ起動し、試行ごとに新しいステージに世界を作り直す（起動に数分かかるため）。
エージェントは Isaac Sim の Python（env_isaaclab）で動くので、使うライブラリはそこに入れておくこと。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# --- Isaac Sim の起動は他の import より先に行う ---
from isaaclab.app import AppLauncher

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contest.evaluate import add_eval_args  # noqa: E402（Isaac Sim に依存しない）

parser = argparse.ArgumentParser(description="Isaac Sim でエージェントを評価する")
add_eval_args(parser)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- ここから下は Isaac Sim 起動後にのみ import できる ---
from contest.evaluate import run  # noqa: E402


def make_isaac_robot(trial, contest_cfg: dict, a: argparse.Namespace):
    from contest.robots.isaac_robot import IsaacRobot

    return IsaacRobot(trial.scene_cfg, contest_cfg, device=a.device)


def main() -> None:
    run(args, "isaac", make_isaac_robot)


if __name__ == "__main__":
    main()
    # main() の外で閉じる（finally で閉じると例外が隠れる。リポジトリ直下の CLAUDE.md）
    simulation_app.close()
