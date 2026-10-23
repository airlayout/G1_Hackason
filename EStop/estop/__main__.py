"""起動: python -m estop --port /dev/ttyUSB0 --iface eth0

端末に `reset` / `damp` と入力して Enter で操作者の明示操作を送る。
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading

from .io import G1Port, SerialLoopSource
from .monitor import OPERATOR_COMMANDS, run
from .safe_stop import SafeStop


def _read_operator(out: queue.Queue[str]) -> None:
    for line in sys.stdin:
        text = line.strip()
        if text in OPERATOR_COMMANDS:
            out.put(text)
        else:
            print(f"[estop] 不明な入力: {text!r}（{'/'.join(OPERATOR_COMMANDS)}）")


def main() -> None:
    ap = argparse.ArgumentParser(description="G1 緊急停止")
    ap.add_argument("--port", required=True, help="ボタンの USB-シリアル (例 /dev/ttyUSB0)")
    ap.add_argument("--iface", required=True, help="G1 と繋がる NIC 名 (例 eth0)")
    args = ap.parse_args()

    operator: queue.Queue[str] = queue.Queue()
    threading.Thread(target=_read_operator, args=(operator,), daemon=True).start()
    print("[estop] 起動。reset=復帰 damp=ダンピング")
    run(SerialLoopSource(args.port), G1Port(args.iface), SafeStop(), operator)


if __name__ == "__main__":
    main()
