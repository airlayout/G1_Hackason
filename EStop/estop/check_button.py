"""ボタン配線の確認: python -m estop.check_button --port /dev/ttyUSB0

押す前=OK、押した後=STOP、USB を抜く=STOP になることを目で確認する。
"""

from __future__ import annotations

import argparse
import time

from .io import SerialEchoSource

POLL_S = 0.01  # SEND_PERIOD_S より十分短くする


def main() -> None:
    ap = argparse.ArgumentParser(description="緊急停止ボタンの配線確認")
    ap.add_argument("--port", required=True, help="USB-シリアル (例 /dev/ttyUSB0)")
    args = ap.parse_args()

    source = SerialEchoSource(args.port)
    last: bool | None = None
    print("[check] 開始。押す前 → 押す → 戻す → USBを抜く の順に確認。Ctrl+C で終了")
    while True:
        ok = source.is_ok()
        if ok != last:
            print(f"[check] {time.strftime('%H:%M:%S')} {'OK  (閉じている)' if ok else 'STOP(開 / 断線 / 抜け)'}")
            last = ok
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
