"""rollout の mp4 から「等間隔 6 フレーム」のコンタクトシート（JPEG）を作る。

mp4 を開かなくても、各エピソードで何が起きたかをざっと見られる。ファイル名は mp4 の先頭 8 文字。
使い方: python3 -I make_contact_sheets.py --videos-dir <mp4 のあるディレクトリ> --out-dir <JPEG の出力先>
依存: av numpy pillow（VLA/sim/requirements-viewer.txt）
"""
import argparse
import pathlib

import av
import numpy as np
from PIL import Image

FPS = 20.0  # rollout の動画は 20fps
TILE_W, TILE_H = 480, 180
COLS, ROWS = 3, 2


def make_sheet(mp4: pathlib.Path, out: pathlib.Path) -> int:
    with av.open(str(mp4)) as c:
        frames = [f.to_ndarray(format="rgb24") for f in c.decode(video=0)]
    idx = np.linspace(0, len(frames) - 1, COLS * ROWS).astype(int)
    sheet = Image.new("RGB", (TILE_W * COLS, TILE_H * ROWS))
    for k, i in enumerate(idx):
        tile = Image.fromarray(frames[i]).resize((TILE_W, TILE_H))
        sheet.paste(tile, ((k % COLS) * TILE_W, (k // COLS) * TILE_H))
    sheet.save(out, quality=88)
    return len(frames)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--videos-dir", type=pathlib.Path, required=True, help="mp4 を（サブディレクトリ含め）探すディレクトリ")
    ap.add_argument("--out-dir", type=pathlib.Path, required=True, help="JPEG の出力先")
    args = ap.parse_args()

    mp4s = sorted(args.videos_dir.expanduser().rglob("*.mp4"))
    if not mp4s:
        raise SystemExit(f"[error] {args.videos_dir} に mp4 が無い")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for mp4 in mp4s:
        out = args.out_dir / f"{mp4.name[:8]}.jpg"
        n = make_sheet(mp4, out)
        print(f"[sheet] {mp4.name[:8]} frames={n} ({n / FPS:.1f}s) -> {out}")


if __name__ == "__main__":
    main()
