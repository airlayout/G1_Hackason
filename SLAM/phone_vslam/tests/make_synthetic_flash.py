"""時刻のずれの道具（measure_sync.py）を電話なしで通すための、白黒の点滅の映像。

    python tests/make_synthetic_flash.py <出す所> [--secs 40]

0.7〜1.5 秒のばらばらの間隔で白と黒を切り替える（publish/flash.html と同じ作り）。
ffmpeg の tpad で片方だけ遅らせて cam1・cam2 に流すと、measure_sync.py がそのずれを当てるはず。
"""

from __future__ import annotations

import argparse
import pathlib

import cv2
import numpy as np

SIZE = (1280, 720)
FPS = 30


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("out")
    p.add_argument("--secs", type=float, default=40.0)
    args = p.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(3)
    edges = np.cumsum(rng.uniform(0.7, 1.5, int(args.secs)))
    w = cv2.VideoWriter(str(out / "flash.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FPS, SIZE)
    for k in range(int(args.secs * FPS)):
        white = np.searchsorted(edges, k / FPS) % 2 == 1
        w.write(np.full((SIZE[1], SIZE[0], 3), 235 if white else 15, np.uint8))
    w.release()
    print(f"[合成] 点滅 {args.secs:.0f} 秒 → {out}/flash.mp4")


if __name__ == "__main__":
    main()
