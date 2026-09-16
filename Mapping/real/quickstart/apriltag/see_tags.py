#!/usr/bin/env python3
"""PC2 の頭カメラ（IR）で AprilTag が見えているかを、その場で確かめる。

## これは何のための道具か

**タグを貼る位置を決めるため**の道具である。1 枚貼って、これを走らせて、
「見えている / 距離 / 画面のどこ / 一辺が何 px」を読む。読めなければ貼り直す。
貼り終わってから「実は見えていなかった」を防ぐ。

## なぜ IR（/dev/video2）なのか

2026-09-15 に PC2 で実測した結果:

- `/dev/video4`（カラー）は **`videohub_pc4` が握っていて開けない**（S_FMT が EBUSY）。
  純正のカメラ配信サービスなので、止めると純正アプリの映像が消える
- `/dev/video0`（深度 Z16）は OpenCV が開けず、生 V4L2 でも DQBUF が返ってこない
- **`/dev/video2`（左 IR）は空いていて 1280x720 で 25.7 fps 出る**

IR はむしろ都合がよい。D435i の IR はグローバルシャッタなので**歩いている最中でも
像が歪まない**し、出力の時点で rectified なので歪み係数が実質ゼロである。

⚠️ IR プロジェクタが点いていると、タグの上に点模様が乗って読めなくなることがある。
2026-09-15 の実測では点いていなかったが、深度を使う何かが動くと点く。
点模様が見えたら `--save-dir` の画像で確かめること。

## 使い方（PC2 で）

    python3 see_tags.py --seconds 10
    python3 see_tags.py --seconds 10 --tag-mm 160 --save-dir /tmp/tagrun
    python3 see_tags.py --frames 1 --save-dir /tmp/one     # 1 枚だけ見る
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect

IR_DEVICE = "/dev/video2"
READ_TIMEOUT_S = 15


class ReadTimeout(Exception):
    pass


def _raise_timeout(*_):
    raise ReadTimeout()


def open_ir(device: str, width: int, height: int, fps: int):
    """IR ノードを開く。OpenCV が固まることがあるので必ず alarm を張る。"""

    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(
            f"{device} を開けない。`python3 -c \"import cv2\"` が通るか、"
            "他のプロセスが握っていないかを見る")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    got = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
           int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != (width, height):
        capture.release()
        raise RuntimeError(f"要求 {width}x{height} に対し {got[0]}x{got[1]} になった")
    return capture


def summarize(records: dict, frames: int) -> None:
    """ID ごとに、貼る位置を決めるのに要る数字だけを出す。"""

    if not records:
        print("\n⛔ 1 枚も検出できなかった。")
        print("   ・タグが視野に入っているか（頭カメラは下を向いている。低い位置に貼る）")
        print(f"   ・一辺が {tag_detect.MIN_SIDE_PIXELS:.0f} px 未満だと読めない。"
          "近づくか、大きいタグにする")
        print("   ・照明の映り込み・紙のたわみ・斜めすぎ（60 度超）も落ちる原因")
        return
    print(f"\n=== {frames} 枚のまとめ ===")
    header = "ID   検出率     一辺px     距離m    入射角     位置    曖昧さ"
    print(header)
    print("-" * len(header))
    for tag_id in sorted(records):
        rows = records[tag_id]
        rate = 100.0 * len(rows) / frames
        sides = np.median([r["side"] for r in rows])
        dists = [r["dist"] for r in rows if r["dist"] is not None]
        angs = [r["inc"] for r in rows if r["inc"] is not None]
        ambs = [r["amb"] for r in rows if r["amb"] is not None]
        where = max(set(r["where"] for r in rows), key=[r["where"] for r in rows].count)
        print("%-4d %5.1f%%   %7.1f   %6s   %6s   %6s   %6s" % (
            tag_id, rate, sides,
            "%.2f" % np.median(dists) if dists else "-",
            "%.0f" % np.median(angs) if angs else "-",
            where,
            "%.2f" % np.median(ambs) if ambs else "-"))
    print()
    print("読み方:")
    print("  ・検出率 95% 未満 → 貼り直すか近づける。まばらだと起動時に使えない")
    print(f"  ・一辺 {tag_detect.MIN_SIDE_PIXELS:.0f} px を切ると読めなくなる。余裕を 1.5 倍は取る")
    print(f"  ・曖昧さが {tag_detect.AMBIGUITY_LIMIT} を超える枚が多い → 向きは信じない。"
          "少し斜めから見える位置へ貼り替える")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=IR_DEVICE)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--frames", type=int, default=0, help="枚数で切る（--seconds より優先）")
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--tag-mm", type=float, default=160.0,
                        help="黒い正方形の一辺 [mm]。印刷した紙に刷ってある値")
    parser.add_argument("--intrinsics", default=None,
                        help="calibrate_intrinsics.py が書いた JSON。無ければ画角から仮置き")
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--save-dir", default=None, help="生画像と描き込み画像の置き場")
    parser.add_argument("--save-every", type=int, default=1,
                        help="何枚に 1 枚保存するか。⚠️ PNG の書き出しは重い "
                             "（1280x720 で 25.7 fps → 9.5 fps。2026-09-15 実測）")
    parser.add_argument("--quiet", action="store_true", help="1 枚ごとの行を出さない")
    parser.add_argument("--warmup", type=int, default=12,
                        help="最初に捨てる枚数。⚠️ 開いた直後の数枚は自動露出が"
                             "効いておらず真っ白に飛ぶ（2026-09-16 に実測）")
    parser.add_argument("--fast", action="store_true",
                        help="角の精密化を切る。86.8 → 16.7 ms/枚。"
                             "⚠️ 取付測定のばらつきが 3 倍になる（高さ σ5 → σ18 mm）")
    arguments = parser.parse_args()

    if arguments.intrinsics:
        camera_matrix, distortion, meta = tag_detect.load_intrinsics(arguments.intrinsics)
        print(f"[内部パラメータ] {arguments.intrinsics} "
              f"fx={camera_matrix[0, 0]:.1f} fy={camera_matrix[1, 1]:.1f} "
              f"再投影 RMS={meta.get('rms', float('nan')):.3f} px")
        if meta.get("width") != arguments.width or meta.get("height") != arguments.height:
            print(f"⚠️ 校正は {meta.get('width')}x{meta.get('height')} で取った。"
                  f"いまは {arguments.width}x{arguments.height}。距離が系統的にずれる")
    else:
        camera_matrix = tag_detect.default_camera_matrix(
            arguments.width, arguments.height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 内部パラメータは未校正。画角 {arguments.hfov_deg:.0f}deg からの仮置き "
              f"（fx={camera_matrix[0, 0]:.1f}）。距離は数 % ずれる")

    save_dir = Path(arguments.save_dir) if arguments.save_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    detector = tag_detect.make_detector(arguments.family, refine=not arguments.fast)
    tag_m = arguments.tag_mm / 1000.0
    capture = open_ir(arguments.device, arguments.width, arguments.height, arguments.fps)
    print(f"[カメラ] {arguments.device} {arguments.width}x{arguments.height} "
          f"/ タグ一辺 {arguments.tag_mm:.0f} mm / {arguments.family}")

    signal.signal(signal.SIGALRM, _raise_timeout)
    # ⚠️ 開いた直後の数枚は自動露出が追いついておらず白飛びする。数える前に捨てる。
    for _ in range(max(0, arguments.warmup)):
        signal.alarm(READ_TIMEOUT_S)
        capture.read()
        signal.alarm(0)
    records: dict = {}
    frames = 0
    started = time.time()
    last_annotated = None
    try:
        while True:
            if arguments.frames:
                if frames >= arguments.frames:
                    break
            elif time.time() - started >= arguments.seconds:
                break
            signal.alarm(READ_TIMEOUT_S)
            ok, frame = capture.read()
            signal.alarm(0)
            if not ok:
                continue
            frames += 1
            gray = tag_detect.to_gray(frame)
            found = tag_detect.detect(detector, gray)
            for item in found:
                try:
                    pose = tag_detect.solve_pose(item, tag_m, camera_matrix, distortion)
                except RuntimeError:
                    pose = None
                records.setdefault(item.tag_id, []).append({
                    "side": item.side_pixels,
                    "dist": pose.distance_m if pose else None,
                    "inc": pose.incidence_deg if pose else None,
                    "amb": pose.ambiguity if pose else None,
                    "where": item.where_in_frame(arguments.width, arguments.height),
                })
            if not arguments.quiet:
                if found:
                    parts = []
                    for item in found:
                        text = f"ID{item.tag_id} {item.side_pixels:.0f}px"
                        if item.pose is not None:
                            text += (f" {item.pose.distance_m:.2f}m "
                                     f"{item.pose.incidence_deg:.0f}deg "
                                     f"{item.where_in_frame(arguments.width, arguments.height)}")
                            if not item.pose.trustworthy:
                                text += " ⚠️曖昧"
                        parts.append(text)
                    print(f"{frames:4d}  " + " | ".join(parts), flush=True)
                elif frames % 10 == 0:
                    print(f"{frames:4d}  —", flush=True)
            if save_dir and frames % max(1, arguments.save_every) == 0:
                cv2.imwrite(str(save_dir / f"raw_{frames:05d}.png"), gray)
                last_annotated = tag_detect.annotate(gray, found)
                if found:
                    cv2.imwrite(str(save_dir / f"ann_{frames:05d}.jpg"), last_annotated,
                                [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    except ReadTimeout:
        print(f"⛔ カメラの read が {READ_TIMEOUT_S} 秒返ってこない。"
              "他のプロセスが握っていないか見る")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        signal.alarm(0)
        capture.release()

    elapsed = max(time.time() - started, 1e-6)
    print(f"\n{frames} 枚 / {elapsed:.1f} s = {frames / elapsed:.1f} fps")
    if save_dir and last_annotated is not None:
        cv2.imwrite(str(save_dir / "last_annotated.jpg"), last_annotated,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        print(f"画像: {save_dir}")
    summarize(records, frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
