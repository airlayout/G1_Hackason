#!/usr/bin/env python3
"""その場で決まった角度だけ回る指令を `loco_driver.py` へ UDP で送る（PC2 の python3.8）。

    python3 spin_step.py --deg 45

`live/initial_pose.sh` がタグを探して回るときに使う。**回った量は測らない。**
タグが見えた時点の姿勢をそのまま種にするので、45 度きっちりでなくてよい。

## 安全

- 速度の上限・後退の禁止・ウォッチドッグは全部 `loco_driver.py` 側にある
- ここは「角速度 × 時間」ぶん送って、最後に**ゼロを数回送る**だけ
- 途中で落ちても、指令が 0.5 秒途切れればドライバが自分で止める
"""
import argparse
import json
import math
import socket
import time

PORT = 47600
RATE_HZ = 20.0
DEFAULT_YAW_RATE = 0.4   # [rad/s] loco_driver の上限 0.5 より下
ZERO_REPEATS = 5


def send(sock: socket.socket, port: int, vyaw: float) -> None:
    payload = json.dumps({"vx": 0.0, "vy": 0.0, "vyaw": vyaw}).encode("utf-8")
    sock.sendto(payload, ("127.0.0.1", port))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--deg", type=float, default=45.0, help="回る角度（正で左回り）")
    parser.add_argument("--yaw-rate", type=float, default=DEFAULT_YAW_RATE)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    rate = math.copysign(abs(args.yaw_rate), args.deg)
    duration = abs(math.radians(args.deg) / args.yaw_rate)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        started = time.monotonic()
        while time.monotonic() - started < duration:
            send(sock, args.port, rate)
            time.sleep(1.0 / RATE_HZ)
    finally:
        for _ in range(ZERO_REPEATS):
            send(sock, args.port, 0.0)
            time.sleep(1.0 / RATE_HZ)
        sock.close()
    print("[spin] %.0f 度ぶん送った（%.2f rad/s × %.1f s）" % (args.deg, rate, duration),
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
