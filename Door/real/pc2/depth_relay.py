"""11-06: 頭カメラ D435i の深度を Python 3.8 の pyrealsense2 で受け、最新の 1 枚を /dev/shm に置く（door_depth_shm.py の書式）。

PC2 の lerobot（3.12）には pyrealsense2 が入らないので、カメラ側（door_vision_node.py --depth shm）と分けた（11-06 ユーザー決定）。
深度だけを開く（カラーは純正の映像サービスが握っている）。11-06 実測: 深度 29.8 枚/秒と VideoClient のカラー 32.9 枚/秒が同時に取れた。
z16 → m・学習の形への切り出しは読む側でやる（ここは受けて置くだけ）。終わるときに置いたファイルを消す（古い枚を次の回に読ませない）。

使い方（PC2・機体の python3＝3.8。run_vision.sh・depth_probe.py が起こす）:
  python3 depth_relay.py [--seconds 10] [--parent-pid <pid>]
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # 手元（Door/real/pc2/ の下）
sys.path.insert(0, str(HERE))  # PC2（~/door_real/ に同じ並びで置く）
import door_depth_shm as dds  # noqa: E402

STATS_EVERY_S = 2.0


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--path", default=dds.PATH)
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=0.0, help="この秒で終わる（0 は止められるまで）")
    ap.add_argument("--parent-pid", type=int, default=0, help="このプロセスが居なくなったら終わる")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    import pyrealsense2 as rs

    if os.path.exists(a.path):
        os.remove(a.path)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
    prof = pipe.start(cfg)
    scale = float(prof.get_device().first_depth_sensor().get_depth_scale())
    intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intrinsics = (float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy))
    print(f"[depth_relay] 深度 {a.width}×{a.height}@{a.fps}・尺度 {scale}・内部の値 {intrinsics}・置き場 {a.path}", flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    t_start = time.monotonic()
    last_stats, n, n_prev, n_err, write_ms = t_start, 0, 0, 0, []
    try:
        while not stop.is_set():
            now = time.monotonic()
            if a.seconds and now - t_start >= a.seconds:
                break
            if now - last_stats >= STATS_EVERY_S:
                if a.parent_pid and not pid_alive(a.parent_pid):
                    print(f"[depth_relay] 親（pid {a.parent_pid}）が終わったので終わる", flush=True)
                    break
                print(f"[depth_relay] {(n - n_prev) / (now - last_stats):4.1f} 枚/秒・置く {np.mean(write_ms) if write_ms else float('nan'):.1f} ms・"
                      f"失敗 {n_err}", flush=True)
                last_stats, n_prev, write_ms = now, n, []
            try:
                frames = pipe.wait_for_frames(1000)
                z16 = np.asanyarray(frames.get_depth_frame().get_data())
            except Exception as e:  # noqa: BLE001  受けられない回は数えて続ける
                n_err += 1
                print(f"[depth_relay] 深度を受けられない: {type(e).__name__}: {e}", flush=True)
                continue
            t_rx = time.monotonic()
            n += 1
            dds.write_frame(a.path, z16, n, t_rx, scale, intrinsics)
            write_ms.append((time.monotonic() - t_rx) * 1000.0)
            if n == 1:
                print("[depth_relay] 最初の深度を置いた", flush=True)
    finally:
        pipe.stop()
        for p in (a.path, a.path + ".tmp"):
            if os.path.exists(p):
                os.remove(p)
        print(f"[depth_relay] 終わる（{n} 枚・失敗 {n_err}）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
