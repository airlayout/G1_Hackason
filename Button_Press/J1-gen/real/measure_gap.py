"""横から撮った画像（または動画のフレーム）で、指先とボトル（ボタン）の面とのすき間を測る。

押し込みが「どれくらい届かなかったか」を、定規やボトルの太さを物差しにして、画像から測る。

    # 画像（クリックで測る。ウィンドウが開く）。物差しは定規の 30 cm の両端、続けて指先とボトルの面
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/measure_gap.py <画像> --ref-cm 30
    # 動画の 1 フレーム（--frame は 0 始まり）。物差しにボトルの太さ（500 ml の茶は約 6.5 cm）を使うとき
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/measure_gap.py <動画.mp4> --frame 900 --ref-cm 6.5
    # クリックせずに、画素の座標で与える（ウィンドウが開けない環境）: 物差しの 2 点、すき間の 2 点
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/measure_gap.py <画像> --ref-cm 30 --points 100 400 400 400 520 300 580 300

クリックの順番: 1〜2 点目 = 物差しの両端、3 点目 = 指先、4 点目 = ボトルの面（指先に向かい合う点）。r で最初から、q で終わる。

⚠️ 物差しは、測りたいすき間と同じ奥行き（カメラから同じ距離）・同じ向きに置く。真横から撮らないと、すき間は
実際より短く出る（斜めから撮ると、押す向きの距離が縮んで見える）。誤差の目安は ±1 cm（真横）〜 ±2 cm（斜め）。
結果は注釈つきの画像（_local/button_press/gap/）と JSON に残し、実機の記録（runs/<日時>/events.jsonl の press_result の
tip_advance_m、計画の押し込みの長さ）と突き合わせる。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT  # noqa: E402


def gap_cm(ref_a: tuple[float, float], ref_b: tuple[float, float], ref_cm: float,
           tip: tuple[float, float], surface: tuple[float, float]) -> dict[str, float]:
    """物差し（2 点の間が ref_cm）から、指先とボトルの面の間の距離 [cm] を出す。

    distance: 2 点の直線距離。along_ref: 物差しの向きに沿った成分（物差しを押す向きに置いたとき、押す向きのすき間）。
    """
    ref_px = math.dist(ref_a, ref_b)
    if ref_px < 1.0:
        raise ValueError("物差しの 2 点が近すぎる")
    cm_per_px = ref_cm / ref_px
    d = np.array(surface, dtype=float) - np.array(tip, dtype=float)
    u = (np.array(ref_b, dtype=float) - np.array(ref_a, dtype=float)) / ref_px
    return {"cm_per_px": cm_per_px, "ref_px": ref_px, "distance_cm": float(np.linalg.norm(d) * cm_per_px),
            "along_ref_cm": float(abs(d @ u) * cm_per_px)}


def load_image(path: Path, frame: int | None) -> np.ndarray:
    if path.suffix.lower() in (".mp4", ".mov", ".avi", ".mkv"):
        cap = cv2.VideoCapture(str(path))
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame or 0)
        ok, img = cap.read()
        cap.release()
    else:
        img, ok = cv2.imread(str(path)), None
        ok = img is not None
    if not ok:
        raise FileNotFoundError(f"画像（フレーム）を読めない: {path}（フレーム {frame}）")
    return img


def pick_points(img: np.ndarray) -> list[tuple[float, float]]:
    """ウィンドウで 4 点をクリックしてもらう。"""
    pts: list[tuple[float, float]] = []
    labels = ["物差しの端 1", "物差しの端 2", "指先", "ボトルの面"]
    scale = min(1.0, 900 / max(img.shape[:2]))
    view = cv2.resize(img, None, fx=scale, fy=scale)

    def redraw() -> None:
        v = view.copy()
        for i, (x, y) in enumerate(pts):
            cv2.circle(v, (int(x * scale), int(y * scale)), 4, (0, 0, 255), -1)
            cv2.putText(v, labels[i], (int(x * scale) + 6, int(y * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        nxt = labels[len(pts)] if len(pts) < 4 else "q で終わる / r でやり直し"
        cv2.putText(v, f"click: {nxt}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
        cv2.imshow("measure_gap", v)

    def on_mouse(ev: int, x: int, y: int, *_: object) -> None:
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x / scale, y / scale))
            redraw()

    cv2.namedWindow("measure_gap")
    cv2.setMouseCallback("measure_gap", on_mouse)
    redraw()
    while True:
        k = cv2.waitKey(50) & 0xFF
        if k == ord("r"):
            pts.clear()
            redraw()
        if (k == ord("q") and len(pts) == 4) or (k == 27):
            break
    cv2.destroyAllWindows()
    return pts


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("source", type=Path, help="画像または動画")
    p.add_argument("--frame", type=int, help="動画のフレーム番号（0 始まり）")
    p.add_argument("--ref-cm", type=float, required=True, help="物差し（クリックする 2 点の間）の実際の長さ [cm]")
    p.add_argument("--points", type=float, nargs=8, metavar="PX",
                   help="クリックせずに、物差し 2 点・指先・ボトルの面の画素の座標（x y を 4 組）")
    p.add_argument("--out-dir", type=Path, default=REPO_ROOT / "_local" / "button_press" / "gap")
    args = p.parse_args()

    img = load_image(args.source, args.frame)
    if args.points is not None:
        v = args.points
        pts = [(v[0], v[1]), (v[2], v[3]), (v[4], v[5]), (v[6], v[7])]
    else:
        try:
            pts = pick_points(img)
        except cv2.error as e:
            print(f"[gap] ❌ ウィンドウを開けない（{e}）。--points で座標を与える")
            return 1
        if len(pts) < 4:
            print("[gap] 4 点そろわなかったので終わる")
            return 1
    r = gap_cm(pts[0], pts[1], args.ref_cm, pts[2], pts[3])
    print(f"[gap] 物差し {r['ref_px']:.0f} px = {args.ref_cm:g} cm（1 px = {r['cm_per_px'] * 10:.2f} mm）")
    print(f"[gap] 指先とボトルの面の間: 直線 {r['distance_cm']:.1f} cm、物差しの向きに沿った成分 {r['along_ref_cm']:.1f} cm")
    print("[gap] ⚠️ 真横から撮っていないと、実際より短く出る。物差しは同じ奥行き・向きに置く")
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.source.stem}"
    ann = img.copy()
    for (x, y), c in zip(pts, [(255, 0, 0), (255, 0, 0), (0, 0, 255), (0, 0, 255)]):
        cv2.circle(ann, (int(x), int(y)), 6, c, -1)
    cv2.line(ann, tuple(int(t) for t in pts[0]), tuple(int(t) for t in pts[1]), (255, 0, 0), 2)
    cv2.line(ann, tuple(int(t) for t in pts[2]), tuple(int(t) for t in pts[3]), (0, 0, 255), 2)
    cv2.putText(ann, f"gap {r['distance_cm']:.1f} cm (along ref {r['along_ref_cm']:.1f})", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.imwrite(f"{stem}.jpg", ann)
    Path(f"{stem}.json").write_text(json.dumps({
        "source": str(args.source), "frame": args.frame, "ref_cm": args.ref_cm, "points_px": pts, **r}, indent=2, ensure_ascii=False))
    print(f"[gap] 保存: {stem}.jpg / .json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
