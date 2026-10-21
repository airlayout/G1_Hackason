#!/usr/bin/env python3
"""--task button|bottleで既存のボタン押しとボトル押しを切り替える。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from task_cli import dispatch


def main(argv=None):
    from run_vision_mujoco import main as button_main
    from run_bottle_mujoco import main as bottle_main
    return dispatch(button_main, bottle_main, argv)


if __name__ == "__main__":
    raise SystemExit(main())
