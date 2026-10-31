#!/usr/bin/env python3
"""人を検出したら、写真を G1 に保存し、PC へも配信する(G1 の PC2 で動かす)。

    # GPU(TensorRT)で。システムの Python 3.8 で動かす(エンジンの作り方は common/detector/trt_yolo.py)
    cd ~/perception && /usr/bin/python3 real/person_alert.py --webcam 6 --rotate cw --backend tensorrt --model yolo26n.engine
    # CPU(onnxruntime)で
    cd ~/perception && ~/miniforge3/envs/lerobot/bin/python real/person_alert.py --webcam 6 --rotate cw --model yolo26n.onnx --threads 4

人が --min-frames フレーム続けて写ったら「出来事」にし、そのあと --cooldown 秒は出さない(人がいる間は --cooldown 秒ごとに出し直す)。
出来事ごとに、枠を描いた写真と記録を --out-dir(既定 _local/perception/person_alert)に保存する。写真は --max-photos 枚まで
(超えたら古いものから消す)。PC が受け取っているかどうかに関わらず、G1 には必ず残す。

PC へは ZMQ の PUB で配信する(--port、既定 5560)。PC から G1 へつなぐ向きなので、PC 側(WSL の NAT の中でも)の設定は要らない。
受け取りは real/person_alert_receiver.py。PC がつないでいない間の出来事は配信されず、G1 にだけ残る。
送るもの(どちらも 2 つの部分に分けた multipart):
  - 出来事: [b"event", 記録のJSON] + [JPEG]
  - 生きている合図(--heartbeat 秒ごと): [b"heartbeat", {"time", "fps", "frames", "events", "detector"} のJSON]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.alert import AlertStore, EventJudge, draw_detections, make_record  # noqa: E402
from common.detector import build_detector  # noqa: E402

# リポジトリの中(Perception/real/)ならリポジトリ直下、PC2 に写した ~/perception/real/ なら ~/perception を基準にする
_HERE = Path(__file__).resolve()
REPO_ROOT = _HERE.parents[2] if (_HERE.parents[2] / "Perception").is_dir() else _HERE.parents[1]

ROTATIONS = {"cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--webcam", type=int, metavar="INDEX", help="Webカメラ(OpenCVのデバイス番号。PC2 に挿したものは 6)")
    src.add_argument("--zmq", metavar="HOST:PORT", help="ZMQのRGB配信(run_g1_server.py --camera と同じ形式)")
    p.add_argument("--camera-name", default="head_camera", help="--zmq の画像の名前")
    p.add_argument("--rotate", choices=list(ROTATIONS), help="検出の前に画像を回す(PC2 の Web カメラは cw)")
    p.add_argument("--backend", choices=["onnx", "tensorrt"], default="onnx")
    p.add_argument("--model", required=True, help="onnx なら .onnx、tensorrt なら .engine(隣に .names.json が要る)")
    p.add_argument("--threads", type=int, help="onnxruntime のスレッド数(onnx のときだけ)")
    p.add_argument("--conf", type=float, default=0.5, help="人とみなす確からしさのしきい値")
    p.add_argument("--min-frames", type=int, default=3, help="何フレーム続けて人が写ったら出来事にするか")
    p.add_argument("--cooldown", type=float, default=10.0, help="出来事を出したあと、次を出さない秒数")
    p.add_argument("--max-photos", type=int, default=1000, help="G1 に残す写真の上限(超えたら古いものから消す)")
    p.add_argument("--out-dir", type=Path)
    p.add_argument("--port", type=int, default=5560, help="PC へ配信する ZMQ の PUB のポート")
    p.add_argument("--heartbeat", type=float, default=5.0, help="生きている合図を送る間隔(秒)")
    p.add_argument("--jpeg-quality", type=int, default=85)
    p.add_argument("--max-seconds", type=float, help="この秒数で終える(動作確認用。既定は Ctrl-C まで)")
    return p.parse_args()


def build_source(args: argparse.Namespace):
    if args.zmq:
        from common.camera.zmq_camera import ZmqFrameSource

        host, port = args.zmq.rsplit(":", 1)
        return ZmqFrameSource(server_address=host, port=int(port), camera_name=args.camera_name, timeout_ms=5000)
    from common.camera.webcam import WebcamSource

    return WebcamSource(device_index=args.webcam)


def build_publisher(port: int):
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 10)  # PC が遅くても G1 側にためこまない(あふれた分は捨てる。写真は G1 に残っている)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f"tcp://*:{port}")
    return sock


def send(sock, parts: list[bytes]) -> bool:
    """送れなければ(あふれたとき)捨てて False。PUB は受け取る相手がいなければ黙って捨てる。"""
    import zmq

    try:
        sock.send_multipart(parts, flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir or REPO_ROOT / "_local" / "perception" / "person_alert"
    cfg = {"backend": args.backend, "model": str(Path(args.model).expanduser()), "classes": ["person"],
           "confidence_threshold": args.conf}
    detector = build_detector(cfg)
    if args.threads and args.backend == "onnx":
        from common.detector.onnx_yolo import OnnxYoloDetector

        detector = OnnxYoloDetector(cfg["model"], cfg["classes"], args.conf, num_threads=args.threads)
    judge = EventJudge(min_frames=args.min_frames, cooldown_s=args.cooldown)
    store = AlertStore(out_dir, max_photos=args.max_photos)
    sock = build_publisher(args.port)
    print(f"[person_alert] 検出器 {detector.device}、保存先 {out_dir}（写真 {args.max_photos} 枚まで）、配信 tcp://*:{args.port}")
    print(f"[person_alert] 人が {args.min_frames} フレーム続いたら出来事、そのあと {args.cooldown:.0f} 秒は出さない。Ctrl-C で終了")

    frames = events = 0
    t_start = t_beat = t_last_frame = time.time()
    frames_at_beat = 0
    try:
        with build_source(args) as source:
            while args.max_seconds is None or time.time() - t_start < args.max_seconds:
                img = source.read()
                if img is None:
                    # ZMQ の配信は新しい画像が無いと空のメッセージを送ってくるので、1 回では止めない
                    if time.time() - t_last_frame > 5.0:
                        print("[person_alert] ❌ 5 秒間画像が届かない(カメラが他のプロセスに使われていないか・配信が動いているかを確かめる)")
                        return 1
                    continue
                t_last_frame = time.time()
                if args.rotate:
                    img = cv2.rotate(img, ROTATIONS[args.rotate])
                dets = detector.detect(img)
                frames += 1
                now = time.time()
                if judge.update(bool(dets), now):
                    events += 1
                    record = make_record(dets, now, frames)
                    ok, buf = cv2.imencode(".jpg", draw_detections(img, dets), [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                    jpeg = buf.tobytes()
                    path = store.save(jpeg, record)
                    sent = send(sock, [b"event", json.dumps(dict(record, photo=path.name)).encode(), jpeg])
                    confs = " ".join(f"{d['confidence']:.2f}" for d in record["detections"][:5])
                    print(f"[person_alert] {record['time_text']} 人 {record['count']} 人（{confs}）→ {path.name}"
                          + ("" if sent else "（配信があふれて送れなかった）"))
                if now - t_beat >= args.heartbeat:
                    fps = (frames - frames_at_beat) / (now - t_beat)
                    beat = {"time": now, "fps": round(fps, 1), "frames": frames, "events": events, "detector": detector.device}
                    send(sock, [b"heartbeat", json.dumps(beat).encode()])
                    t_beat, frames_at_beat = now, frames
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print(f"[person_alert] 終了: {frames} フレーム、出来事 {events} 件、保存先 {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
