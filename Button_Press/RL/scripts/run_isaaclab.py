"""Isaac Lab の rsl_rl のスクリプト（train.py / play.py）を、このパッケージを登録してから動かす。

    $ISAAC_SIM/python.sh Button_Press/RL/scripts/run_isaaclab.py train --task G1-ButtonPress-Teacher-v0 ...

Isaac Lab 本体のファイルは変えない。button_press_rl を import すると、学習環境が gymnasium に登録される。
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

SCRIPTS = {"train": "train.py", "play": "play.py"}


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in SCRIPTS:
        raise SystemExit(f"使い方: run_isaaclab.py {{{'|'.join(SCRIPTS)}}} [Isaac Lab のスクリプトの引数]")
    lab = Path(os.environ.get("ISAACLAB", ""))
    script_dir = lab / "scripts" / "reinforcement_learning" / "rsl_rl"
    script = script_dir / SCRIPTS[sys.argv[1]]
    if not script.exists():
        raise SystemExit(f"[run_isaaclab] Isaac Lab のスクリプトが無い: {script}（env.sh の ISAACLAB を確かめる）")
    import button_press_rl.tasks  # noqa: F401  学習環境を登録する

    # train.py は同じフォルダの cli_args を import する
    sys.path.insert(0, str(script_dir))
    sys.argv = [str(script)] + sys.argv[2:]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
