#!/usr/bin/env python3
"""--task button|bottle|check-arm で対象を選ぶ。既定は実機指令を送らない計画表示。"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from task_cli import dispatch


def main(argv=None):
    # 接続確認はSDKだけのPython環境でも使えるようにする。
    def button_main(args):
        from align_button import main
        return main(args)

    def bottle_main(args):
        from push_bottle import main
        return main(args)

    def check_main(args):
        from check_arm_connection import main
        return main(args)

    return dispatch(button_main, bottle_main, argv, extra_tasks={"check-arm": check_main})


if __name__ == "__main__":
    raise SystemExit(main())
