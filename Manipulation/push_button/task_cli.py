"""既存の入口を保持したまま、起動時に押す対象を選ぶ。"""

import argparse
import sys


def dispatch(button_main, bottle_main, argv=None, *, extra_tasks=None):
    handlers = {"button": button_main, "bottle": bottle_main}
    handlers.update(extra_tasks or {})
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--task", choices=tuple(handlers), default="button")
    args, remaining = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    if "--help" in remaining or "-h" in remaining:
        print(f"対象の選択: --task {'|'.join(handlers)} (既定: button)\n")
    return handlers[args.task](remaining)
