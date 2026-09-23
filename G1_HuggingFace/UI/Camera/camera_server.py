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

⚠️ **番号（/dev/videoN の N）で指定しないこと。** USB が挿し直されたり再認識されると
番号が変わる（2026-09-23 の実機で video6 → video7 に変わり、配信が止まった）。
`--list` が出す **by-id の安定した名前**を使うこと。

使い方（ロボット側）:
    python3 camera_server.py --list            # 開けるカメラと安定名を一覧する
    python3 camera_server.py --device /dev/v4l/by-id/usb-..._webcam_...-video-index0
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
import os
import zmq

MAX_DEVICE = 8


def _by_id_map() -> dict:
    """/dev/videoN -> by-id の安定した名前 の対応を作る。"""
    out = {}
    base = "/dev/v4l/by-id"
    if not os.path.isdir(base):
        return out
    for name in sorted(os.listdir(base)):
        link = os.path.join(base, name)
        out.setdefault(os.path.realpath(link), link)
    return out


def list_devices() -> None:
    """開けるカメラを一覧する。**現地で最初にこれを叩くこと。**

    G1 には /dev/video0.. が並んでいるが、実際に読めるのは一部だけで、
    どれが何かはラベルからは分からない（平均輝度が手掛かりになる）。
    by-id の安定名も併記するので、**配信にはそちらを使うこと**。
    """
    stable = _by_id_map()
    print("開けるカメラ:")
    found = False
    for i in range(MAX_DEVICE):
        dev = "/dev/video%d" % i
        if not os.path.exists(dev):
            continue
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                name = ""
                try:
                    with open("/sys/class/video4linux/video%d/name" % i) as fp:
                        name = fp.read().strip()
                except OSError:
                    pass
                print("  %s  %dx%d  平均輝度 %.0f  %s" % (dev, w, h, frame.mean(), name))
                if dev in stable:
                    print("      --device %s   ← ⚠️ 番号ではなくこちらを使う" % stable[dev])
                else:
                    print("      --device %s   ⚠️ 安定名が無い。挿し直すと番号が変わる" % dev)
                found = True
        cap.release()
    if not found:
        print("  1台も開けなかった")


def main() -> int:
    parser = argparse.ArgumentParser(description="G1 のカメラを ZMQ で配信する")
    parser.add_argument("--list", action="store_true", help="開けるカメラを一覧して終わる")
    parser.add_argument("--device", default="/dev/video0",
                        help="カメラ。**by-id の安定名を推奨**（番号は挿し直すと変わる）")
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

    def open_capture():
        """カメラを開く。番号でも パスでも by-id でも受ける。"""
        target = int(args.device) if str(args.device).isdigit() else str(args.device)
        cap = cv2.VideoCapture(target)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        # ⚠️ 取り込みバッファを最小にする。溜まると映像が遅れて届く
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    cap = open_capture()
    if cap is None:
        print("[カメラ] %s を開けない。--list で開けるものを確認すること"
              % args.device, file=sys.stderr)
        return 1

    ok, frame = cap.read()
    if not ok or frame is None:
        print("[カメラ] %s は開くが読めない" % args.device, file=sys.stderr)
        cap.release()
        return 1
    h, w = frame.shape[:2]

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    # ⚠️ 送信キューを溜めない。溜めると再接続時に古い絵が一気に流れる
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind("tcp://%s:%d" % (args.bind, args.port))
    print("[カメラ] %s %dx%d を tcp://%s:%d へ配信する（name=%s, %.0f fps 上限）"
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
    # ⚠️ USB が再認識されると read() が延々と失敗し続ける（2026-09-23 に実機で発生）。
    # 一定回数失敗したら**開き直す**。by-id を指定していればここで復帰できる
    fails = 0
    REOPEN_AFTER = 10

    while running["v"]:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok or frame is None:
            fails += 1
            if fails % REOPEN_AFTER == 0:
                print("[カメラ] %d 回読めない。開き直す（USB の再認識を疑う）" % fails,
                      file=sys.stderr)
                try:
                    cap.release()
                except Exception:
                    pass
                cap = open_capture()
                if cap is None:
                    print("[カメラ] 開き直せない。%s があるか確認すること" % args.device,
                          file=sys.stderr)
                    cap = cv2.VideoCapture(-1)   # 次の周回でまた開き直す
            time.sleep(0.5)
            continue
        fails = 0

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
