"""rendertest_1023.sh の 2 つの試し（ri4・ri1）の、1 秒あたりの歩数と、描いた画のくっきり具合を比べて判定する。

速さ: 各試しの train.log の「Computation: N steps/s (Collection: c s」の、始めの 10 回を除いた中央
くっきり具合: 確かめの記録（camera_pitch_debug_dir の直下の PNG）の、灰色の画のラプラシアンの分散（輪郭が強いほど大きい）の中央。
  画には学習の加工（明るさ・ぼかし・雑音、確率つき）が入っているので、多くの画の中央で比べる。明るさの中央も出す
判定: 速さの比 ≥ SPEED_MIN（1.3）かつ くっきり具合の比が 1 ± SHARP_TOL（0.15）以内なら「入れる」

使い方: python analyze_rendertest.py <tests/rendertest_1023>
"""
from __future__ import annotations

import glob
import re
import sys
from pathlib import Path

import imageio.v2 as iio
import numpy as np

SPEED_MIN = 1.3
SHARP_TOL = 0.15
ANSI = re.compile(r"\x1b\[[0-9;]*m")
RE = re.compile(r"Computation: (\d+) steps/s \(Collection: ([\d.]+)s")


def speed(train_log: str) -> tuple[float, float, int]:
    vals = [(float(m.group(1)), float(m.group(2))) for m in RE.finditer(ANSI.sub("", Path(train_log).read_text(errors="replace")))]
    vals = vals[10:]
    if not vals:
        raise SystemExit(f"速さが読めない: {train_log}")
    a = np.array(vals)
    return float(np.median(a[:, 0])), float(np.median(a[:, 1])), len(vals)


def sharpness(cam_dir: str) -> tuple[float, float, int]:
    files = sorted(glob.glob(f"{cam_dir}/*.png"))
    if not files:
        raise SystemExit(f"画が無い: {cam_dir}")
    lap, mean = [], []
    for f in files:
        g = iio.imread(f).astype(np.float32)
        g = g[..., :3].mean(axis=2) if g.ndim == 3 else g
        l = g[1:-1, 1:-1] * -4 + g[:-2, 1:-1] + g[2:, 1:-1] + g[1:-1, :-2] + g[1:-1, 2:]
        lap.append(float(l.var()))
        mean.append(float(g.mean()))
    return float(np.median(lap)), float(np.median(mean)), len(files)


def main() -> None:
    out = Path(sys.argv[1])
    res = {}
    for name in ("ri4", "ri1"):
        tl = (out / f"trainlog_{name}.txt").read_text().strip()
        sps, coll, n = speed(tl)
        sh, br, m = sharpness(str(out / f"cam_{name}"))
        res[name] = (sps, coll, n, sh, br, m)
        print(f"{name}: 1 秒あたり {sps:.0f} 歩（集める段 {coll:.2f} 秒、{n} 回の中央）・くっきり具合 {sh:.1f}・明るさ {br:.1f}（{m} 枚の中央）")
    speed_ratio = res["ri4"][0] / res["ri1"][0]
    sharp_ratio = res["ri4"][3] / res["ri1"][3]
    print(f"速さの比 {speed_ratio:.2f}（線 {SPEED_MIN} 以上）・くっきり具合の比 {sharp_ratio:.2f}（線 {1 - SHARP_TOL:.2f}〜{1 + SHARP_TOL:.2f}）")
    ok = speed_ratio >= SPEED_MIN and abs(sharp_ratio - 1) <= SHARP_TOL
    print("判定: 入れる" if ok else "判定: 入れない")


if __name__ == "__main__":
    main()
