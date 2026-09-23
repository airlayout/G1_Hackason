"""実機 G1 のカメラを ZMQ で配信する。**ロボット本体で動かす。**

    [G1 本体] camera_server.py --device 2 ──ZMQ(5555)──> [操作PC] UI の video.source: zmq

⚠️ **モーターには一切触らない。** カメラを読んで流すだけ。

lerobot 同梱の `run_g1_server.py --camera` でも配信できるが、あれは
**DDS↔ZMQ のロボット指令ブリッジ**（`LowCmd` をロボットへ中継し、
`MotionSwitcherClient` を持つ）で、カメラはその「おまけ」に過ぎない。
**映像を見るためだけに指令経路を開けるべきではない**ので、こちらを使う。
（`run_g1_server.py` の `--camera-device` 既定値 4 は、実機では開けない。）

通信形式は lerobot の ZMQ カメラと同じにしてあるので、UI 側
（`Perception/common/camera/zmq_camera.py` の `ZmqFrameSource`）はそのまま使える:

    PUB ソケットで JSON 文字列 {"images": {"<name>": "<base64 の JPEG>"}} を配信

⚠️ **JPEG は RGB で符号化する。** 受信側が `cv2.COLOR_RGB2BGR` で戻す前提のため、
BGR のまま送ると**赤と青が入れ替わる**。

使い方（ロボット側）:
    python3 camera_server.py --list            # 開けるカメラを一覧する
    python3 camera_server.py --device 2        # 配信を始める
"""
from __future__ import annotations

import argparse
import base64
import json
import signal
import sys
import time

import cv2
import numpy as np
import zmq

MAX_DEVICE = 8


def list_devices() -> None:
    """開けるカメラを一覧する。**現地で最初にこれを叩くこと。**

    G1 には /dev/video0..7 が並んでいるが、実際に読めるのは一部だけで、
    どれが何かはラベルからは分からない（平均輝度が手掛かりになる）。
    """
    print("開けるカメラ:")
    found = False
    for i in range(MAX_DEVICE):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                print("  --device %d  %dx%d  平均輝度 %.0f" % (i, w, h, frame.mean()))
                found = True
        cap.release()
    if not found:
        print("  1台も開けなかった")


def main() -> int:
    parser = argparse.ArgumentParser(description="G1 のカメラを ZMQ で配信する")
    parser.add_argument("--list", action="store_true", help="開けるカメラを一覧して終わる")
    parser.add_argument("--device", type=int, default=2,
                        help="/dev/videoN の N（既定 2。実測で RGB が出ていた）")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--name", default="head_camera",
                        help="UI 側 config.yaml の camera_name と合わせる")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0,
                        help="配信の上限。UI は最新1枚しか使わないので上げすぎても無駄")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    if args.list:
        list_devices()
        return 0

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("[カメラ] /dev/video%d を開けない。--list で開けるものを確認すること"
              % args.device, file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # ⚠️ 取り込みバッファを最小にする。溜まると映像が遅れて届く
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok, frame = cap.read()
    if not ok or frame is None:
        print("[カメラ] /dev/video%d は開くが読めない" % args.device, file=sys.stderr)
        cap.release()
        return 1
    h, w = frame.shape[:2]

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    # ⚠️ 送信キューを溜めない。溜めると再接続時に古い絵が一気に流れる
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind("tcp://%s:%d" % (args.bind, args.port))
    print("[カメラ] /dev/video%d %dx%d を tcp://%s:%d へ配信する（name=%s, %.0f fps 上限）"
          % (args.device, w, h, args.bind, args.port, args.name, args.fps))
    print("[カメラ] ⚠️ モーターには触らない。映像を読んで流すだけ")

    running = {"v": True}

    def stop(signum, frame_):
        running["v"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpeg_quality)]
    interval = 1.0 / max(0.1, args.fps)
    sent = 0
    bytes_sent = 0
    t_report = time.time()

    while running["v"]:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[カメラ] 読めなくなった。0.5 秒待って続ける", file=sys.stderr)
            time.sleep(0.5)
            continue

        # ⚠️ 受信側が RGB2BGR で戻すので、ここで RGB にしてから符号化する
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ok, buf = cv2.imencode(".jpg", rgb, encode_params)
        if not ok:
            continue
        payload = json.dumps({
            "images": {args.name: base64.b64encode(buf.tobytes()).decode("ascii")}
        })
        sock.send_string(payload)
        sent += 1
        bytes_sent += len(payload)

        now = time.time()
        if now - t_report >= 10.0:
            dt = now - t_report
            print("[カメラ] %.1f fps / %.0f KB/s（累計 %d 枚）"
                  % (sent / dt, bytes_sent / dt / 1024, sent))
            bytes_sent = 0
            sent = 0
            t_report = now

        rest = interval - (time.time() - t0)
        if rest > 0:
            time.sleep(rest)

    print("[カメラ] 停止する")
    cap.release()
    sock.close()
    ctx.term()
    return 0


if __name__ == "__main__":
    sys.exit(main())
