"""既存の入口を保持したまま、起動時に押す対象を選ぶ。"""

import argparse
import sys


def dispatch(button_main, bottle_main, argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--task", choices=("button", "bottle"), default="button")
    args, remaining = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    if "--help" in remaining or "-h" in remaining:
        print("対象の選択: --task button|bottle (既定: button)\n")
    return (button_main if args.task == "button" else bottle_main)(remaining)
