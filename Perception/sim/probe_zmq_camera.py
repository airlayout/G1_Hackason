"""G1の一人称視点(head_camera)をZMQ経由で受け取り、画像として取得できることを確認する。

シミュレーションでも実機でも同じスクリプトが使える(--host を変えるだけ)。
手順・実測値は Perception/sim/README.md と Perception/real/README.md を参照。

使い方:
  端末1: ./G1_HuggingFace/venv/bin/python SimpleWalk/sim/release_band_and_walk_forward.py
  端末2: ./G1_HuggingFace/venv/bin/python Perception/sim/probe_zmq_camera.py
"""
import argparse
import base64
import json
import time
from pathlib import Path

import cv2
import numpy as np
import zmq

def decode_jpeg(encoded: str) -> np.ndarray:
    decoded_bytes = base64.b64decode(encoded)
    np_arr = np.frombuffer(decoded_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="localhost", help="配信元。実機の場合はG1のIP(192.168.123.164)")
    parser.add_argument("--port", type=int, default=5555, help="ZMQのポート")
    parser.add_argument("--frames", type=int, default=30, help="受信するフレーム数")
    parser.add_argument("--save", type=int, default=3, help="PNGとして保存する枚数")
    parser.add_argument("--out-dir", default="_local/perception/sim", help="保存先ディレクトリ")
    parser.add_argument("--timeout", type=float, default=20.0, help="1枚あたりの受信待ち時間(秒)")
    parser.add_argument("--camera", default=None, help="カメラ名。省略時は最初に見つかったもの")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")   # 全メッセージを購読
    sock.setsockopt(zmq.CONFLATE, 1)            # 最新の1枚だけ保持し、遅延を溜めない
    sock.setsockopt(zmq.RCVTIMEO, int(args.timeout * 1000))
    sock.connect(f"tcp://{args.host}:{args.port}")
    print(f"[接続] tcp://{args.host}:{args.port} を購読開始(最大{args.timeout}秒待機)", flush=True)

    # 途中で return しても、また例外が出ても、ソケットとコンテキストを必ず閉じるために
    # 以降を try/finally で囲む。finally では後片付けだけを行い例外は握りつぶさないので、
    # 失敗の原因が隠れることはない。
    try:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        intervals: list[float] = []   # 受信間隔(実時間)
        latencies: list[float] = []   # 配信時刻から受信までの遅延
        prev_wall: float | None = None
        camera_name: str | None = args.camera
        image: np.ndarray | None = None
        received = 0

        for _ in range(args.frames):
            try:
                raw = sock.recv_string()
            except zmq.Again:
                if received == 0:
                    print("[エラー] 画像が1枚も届かなかった。シミュレーションが起動しているか確認する。", flush=True)
                    return 1
                print(f"[警告] {received}枚受信した時点でタイムアウトした(配信が止まった可能性)。", flush=True)
                break

            now = time.time()
            msg = json.loads(raw)
            images: dict[str, str] = msg.get("images", {})
            if not images:
                print("[警告] images が空のメッセージを受信した。", flush=True)
                continue

            if camera_name is None:
                camera_name = next(iter(images))
                print(f"[情報] 利用可能なカメラ: {list(images.keys())} → '{camera_name}' を使う", flush=True)
            if camera_name not in images:
                print(f"[エラー] カメラ '{camera_name}' がメッセージに無い。含まれるのは {list(images.keys())}", flush=True)
                return 1

            image = decode_jpeg(images[camera_name])
            received += 1

            sent_at = float(msg.get("timestamps", {}).get(camera_name, now))
            latencies.append(now - sent_at)
            if prev_wall is not None:
                intervals.append(now - prev_wall)
            prev_wall = now

            # 最初の数枚だけ、チャンネル順の違いが分かるように2通りで保存する。
            # cv2.imwrite は配列をBGRとみなして書き出すため、元がRGBなら赤と青が入れ替わる。
            # ループ回数ではなく実際に受信できた枚数で数える(images が空で continue した
            # 周回を数えてしまうと、保存される枚数が指定より少なくなるため)。
            if received <= args.save:
                cv2.imwrite(str(out_dir / f"frame_{received:03d}_asis.png"), image)
                cv2.imwrite(str(out_dir / f"frame_{received:03d}_swapped.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

            print(f"[受信] {received:3d}枚目  shape={image.shape}  遅延={now - sent_at:.3f}s", flush=True)

        if image is None:
            print("[エラー] 有効な画像を1枚も取得できなかった。", flush=True)
            return 1

        print("\n=== 結果 ===", flush=True)
        print(f"カメラ名        : {camera_name}", flush=True)
        print(f"解像度          : 幅{image.shape[1]} × 高さ{image.shape[0]}, チャンネル{image.shape[2]}", flush=True)
        print(f"データ型        : {image.dtype}  値の範囲 {image.min()}〜{image.max()}", flush=True)
        print(f"受信枚数        : {received}", flush=True)
        if intervals:
            mean_interval = sum(intervals) / len(intervals)
            print(f"平均受信間隔    : {mean_interval:.3f}s → {1.0 / mean_interval:.2f} fps", flush=True)
        if latencies:
            print(f"平均遅延        : {sum(latencies) / len(latencies):.3f}s", flush=True)
        print(f"保存先          : {out_dir.resolve()}", flush=True)
        print("RESULT_OK", flush=True)
        return 0
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    raise SystemExit(main())
