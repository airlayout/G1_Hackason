"""頭の赤外線カメラ（D435i の左 IR、/dev/video2）を、そのときの関節・IMU と一緒に録る（機体には何も送らない）。

V2（画像から扉の状態を推定するモデル、Door/doorman/door_vision_train.py）を実機の画像で試すための記録。
推定の入力は灰色の画像と腰の 3 関節・重力の向きなので、各コマに rt/lowstate の関節 29 本と IMU の四元数を添える。
正解はライダー（同時に rec_take.sh で録る）。時刻はどれも PC2 の time.time_ns()。

出力: <出力先>/frames/<番号>.jpg（640×360・灰色・品質 95）と <出力先>/index.npz
  t_ns (n,)  コマを受け取った時刻 / q (n, 29) 関節 / quat (n, 4) IMU の四元数 (w, x, y, z) / lowstate_age_s (n,)
index.npz は 50 コマごとに書き直す（途中で落ちても残る）。

使い方（PC2）: python3 rec_ir.py <出力先> <秒> [--fps 10]
⚠️ 開いた直後の数枚は自動露出が効かず白く飛ぶ（09-16）ので、WARMUP 枚は捨てる
⚠️ 裏で起こすと SIGINT が無視の設定で引き継がれるので、受け口を自分で付ける（watch_remote.py と同じ）
"""
import argparse
import os
import signal
import threading
import time

import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

DEVICE = "/dev/video2"
WARMUP = 15
OUT_W, OUT_H = 640, 360


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("secs", type=float)
    ap.add_argument("--fps", type=float, default=10.0)
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "frames"), exist_ok=True)

    latest = {"q": np.zeros(29), "quat": np.array([1.0, 0, 0, 0]), "t": 0.0}
    lock = threading.Lock()

    def on_low(msg) -> None:
        q = np.array([msg.motor_state[i].q for i in range(29)])
        quat = np.array(msg.imu_state.quaternion, dtype=np.float64)
        with lock:
            latest.update(q=q, quat=quat, t=time.time())

    ChannelFactoryInitialize(0, "eth0")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_low, 10)

    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        raise SystemExit(f"[ir] {DEVICE} を開けない（カメラの USB・/dev/video* を確かめる）")
    for _ in range(WARMUP):
        cap.read()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())

    rows_t, rows_q, rows_quat, rows_age = [], [], [], []

    def flush() -> None:
        np.savez(os.path.join(a.out, "index.npz"), t_ns=np.array(rows_t, np.int64), q=np.array(rows_q),
                 quat=np.array(rows_quat), lowstate_age_s=np.array(rows_age))

    period = 1.0 / a.fps
    t_end = time.time() + a.secs
    next_t = time.time()
    print(f"[ir] 録り始めた {DEVICE} → {a.out}（{a.fps:.0f} 枚/秒）", flush=True)
    while not stop.is_set() and time.time() < t_end:
        ok, frame = cap.read()  # 30 fps で読み続け、間引いて保存する（溜まった古いコマを使わない）
        now = time.time()
        if not ok:
            print("[ir] 読めないコマ", flush=True)
            continue
        if now < next_t:
            continue
        next_t = max(next_t + period, now)  # 読みが詰まった後にまとめて保存しない
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
        idx = len(rows_t)
        cv2.imwrite(os.path.join(a.out, "frames", f"{idx:06d}.jpg"), small, [cv2.IMWRITE_JPEG_QUALITY, 95])
        with lock:
            rows_q.append(latest["q"].copy())
            rows_quat.append(latest["quat"].copy())
            rows_age.append(now - latest["t"])
        rows_t.append(int(now * 1e9))
        if idx % 50 == 49:
            flush()
    flush()
    cap.release()
    n = len(rows_t)
    dur = (rows_t[-1] - rows_t[0]) / 1e9 if n > 1 else 0.0
    print(f"[ir] 終わり {n} 枚・{dur:.1f} 秒・{(n - 1) / dur if dur > 0 else 0:.2f} 枚/秒・"
          f"lowstate の古さ 最大 {max(rows_age) if rows_age else float('nan'):.3f} s", flush=True)


if __name__ == "__main__":
    main()
