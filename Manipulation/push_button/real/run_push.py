#!/usr/bin/env python3
"""--task button|bottle で対象を選ぶ。既定は実機指令を送らない計画表示。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from task_cli import dispatch


def main(argv=None):
    from align_button import main as button_main
    from push_bottle import main as bottle_main
    return dispatch(button_main, bottle_main, argv)


if __name__ == "__main__":
    raise SystemExit(main())
