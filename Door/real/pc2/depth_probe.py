"""頭カメラ（D435i）の深度が PC2 で取れるかを確かめる（読むだけ。機体には何も送らない）。計画 D1（2026-11-06）。

確かめること:
  1. librealsense（pyrealsense2）で深度だけ（カラーは開かない）を 848×480・30 枚/秒で受けられるか・内部の値
  2. --color: 同時に純正の映像サービス（VideoClient）のカラーが取れ続けるか（深度を開くとぶつからないか）
  3. 学習と同じ画角・形（54×96）に取り出した深度の測れた割合と、画面の中央の距離（扉の前に置いて巻き尺と比べる）
記録: --out の npz に、取り出した深度（54×96・10 Hz）・元の深度（2 Hz）・内部の値・カラーの JPEG（1 Hz）。
     door_vision_node.py の --depth-npz（機体なしの空回し）にそのまま渡せる（depth は元の深度 m・intrinsics）。

使い方（PC2・lerobot の Python。深度は既定で機体の python3（3.8）の中継 depth_relay.py を子として起こして受ける。
11-06: lerobot の 3.12 には pyrealsense2 が入らなかった＝PyPI の wheel は glibc 2.38 が要り、PC2 は 2.31）:
  ~/miniforge3/envs/lerobot/bin/python depth_probe.py --seconds 10 --color --out logs/depth_probe.npz
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import List

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # 手元（Door/real/pc2/ の下）
sys.path.insert(0, str(HERE))  # PC2（deploy_to_pc2.sh は ~/door_real/ に同じ並びで置く）
import door_vision_encoder as dve  # noqa: E402
import door_vision_node as dvn  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--color", action="store_true", help="純正の映像サービスのカラーも同時に受ける")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default="")
    ap.add_argument("--depth", choices=("shm", "rs"), default="shm",
                    help="shm＝Python 3.8 の中継（depth_relay.py）を子として起こして読む（PC2 の 3.12 には pyrealsense2 が入らない）")
    ap.add_argument("--relay-py", default="/usr/bin/python3", help="中継を回す Python（機体に元からある 3.8 の pyrealsense2）")
    return ap.parse_args(argv)


def open_depth(a: argparse.Namespace):
    """(深度の受け口, 中継の子プロセスか None)。shm は最初の枚が置かれるまで待つ（起動の時間を枚数/秒に入れない）"""
    if a.depth == "rs":
        import pyrealsense2 as rs

        print(f"[depth_probe] pyrealsense2・機器 {[d.get_info(rs.camera_info.name) for d in rs.context().devices]}", flush=True)
        return dvn.RealSenseDepthRx(a.width, a.height, a.fps), None
    relay = subprocess.Popen([a.relay_py, "-u", str(HERE / "depth_relay.py"), "--width", str(a.width), "--height", str(a.height),
                              "--fps", str(a.fps), "--parent-pid", str(os.getpid())])
    depth = dvn.ShmDepthRx()
    t_wait = time.monotonic()
    while depth.latest() is None:
        if relay.poll() is not None or time.monotonic() - t_wait > 15.0:
            relay.terminate()
            raise SystemExit("[depth_probe] 中継から深度が来ない（上の depth_relay の出力を見る）")
        time.sleep(0.05)
    depth.n_rx, depth.n_err = 0, 0
    return depth, relay


def main(argv=None) -> int:
    a = parse_args(argv)
    depth, relay = open_depth(a)
    cam = None
    if a.color:
        dvn.init_dds(a.iface, drop_tracing=True)
        cam = dvn.CameraRx(3.0)
    small: List[np.ndarray] = []
    full: List[np.ndarray] = []
    jpegs: List[bytes] = []
    t0 = time.monotonic()
    last_seq, next_small, next_full, next_jpeg = -1, t0, t0, t0
    try:
        while time.monotonic() - t0 < a.seconds:
            now = time.monotonic()
            got = depth.latest()
            if got is not None and got[2] != last_seq:
                last_seq = got[2]
                if now >= next_small:
                    small.append(dve.depth_from_intrinsics(got[0], *depth.intrinsics))
                    next_small += 0.1
                if now >= next_full:
                    full.append(got[0].astype(np.float16))
                    next_full += 0.5
            if cam is not None and now >= next_jpeg:
                c = cam.latest()
                if c is not None:
                    jpegs.append(bytes(c[0]))
                    next_jpeg += 1.0
            time.sleep(0.005)
    finally:
        dt = time.monotonic() - t0
        depth.stop()
        if relay is not None:
            relay.terminate()
            relay.wait(5.0)
        if cam is not None:
            cam.stop()
    print(f"[depth_probe] 深度 {depth.n_rx / dt:.1f} 枚/秒（失敗 {depth.n_err}）・内部の値 fx {depth.intrinsics[0]:.1f} "
          f"fy {depth.intrinsics[1]:.1f} cx {depth.intrinsics[2]:.1f} cy {depth.intrinsics[3]:.1f}", flush=True)
    if cam is not None:
        print(f"[depth_probe] カラー（VideoClient）{cam.n_rx / dt:.1f} 枚/秒（失敗 {cam.n_err}・code {cam.last_code}）", flush=True)
    if small:
        s = np.stack(small)
        h, w = s.shape[1:]
        center = s[:, h // 3: 2 * h // 3, w // 3: 2 * w // 3]
        valid_c = center[center > 0]
        print(f"[depth_probe] 学習の形の深度 {s.shape}: 測れた割合 {(s > 0).mean():.3f}（中央 {(center > 0).mean():.3f}）・"
              f"画面の中央の距離 中央 {np.median(valid_c) if valid_c.size else float('nan'):.3f} m", flush=True)
    if a.out and full:
        out = Path(a.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out, depth=np.stack(full), intrinsics=np.array(depth.intrinsics), small=np.stack(small),
                            jpeg=np.array([np.frombuffer(j, dtype=np.uint8) for j in jpegs], dtype=object))
        print(f"[depth_probe] 書いた: {out}（元の深度 {len(full)} 枚・学習の形 {len(small)} 枚・カラー {len(jpegs)} 枚）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
