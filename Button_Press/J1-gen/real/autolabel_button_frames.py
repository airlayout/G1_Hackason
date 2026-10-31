"""動画から取り出したフレーム（640×360）に、呼びボタンの正解の枠を**自動で**付ける。YOLO の学習データの形式で保存する。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/real/autolabel_button_frames.py <フレームのフォルダ> --prefix IMG_1037 --out _local/button_press/button_real/IMG_1037

枠の位置は今の YOLO が見つけたもの（重なった枠は 1 つにまとめる）。**向き（up / down）は YOLO の読みではなく、柱の並びで決める**:
点灯した ▼ を up と読み違えるので（2026-10-22）、読みは使わない。
- 車いすの印（青い四角）が写っていれば、印より下の枠は上から ▲（up）・▼（down）、上の枠は印に近い順に ▼（down）・▲（up）
  （ラボの柱は、上から 一般用 ▲▼、印、車いす用 ▲▼）
- 印が写っていなければ、枠が 2 つで、上下の間隔がボタンの直径の 1.3〜3.5 倍のときだけ、上 = up・下 = down
- 印の下の枠が 1 つのときは、▲ か ▼ のどちらかの見落としかもしれないので使わない
- 幅÷高さが 0.4〜2.5 の外の枠は、ボタンではないとして外してから並べる。一般用 ▼ のそばの点字を YOLO がボタンと見つけ、
  印の上の並びがずれて点灯した ▼ を ▲ と付けたため（IMG_1073、2026-10-31。点字は約 3.0、本物のボタンは 0.6〜2.1）
- それ以外（枠が 1 つ・3 つ以上で並びが決まらない、画面の端で切れた枠がある、印の上下で枠が 3 つ以上）は、そのフレームを使わない
  （ラベルを付けずに保存すると、見つけたボタンを「無い」と教えてしまうため）
YOLO が見落としたボタンはラベルに入らない（素手が被さった ▲ など）。向きの正解率を見る実験用で、見落としの率は測れない。
フレーム名は <prefix>_<番号>.jpg。出力は images/*.png と labels/*.txt（real/label_button_photos.py と同じ形式。
sim/merge_button_datasets.py で混ぜる）。summary.csv に、使った・使わなかった理由を残す。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config  # noqa: E402
from common.elevator_buttons import make_candidate_detector  # noqa: E402

CLASS_ID = {"up": 0, "down": 1}


def iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def merge_boxes(cands: list[tuple[tuple[int, int, int, int], float]], thr: float = 0.4) -> list[tuple[int, int, int, int]]:
    """重なった枠（クラス違いの重複）を、確信度の高いほうだけにする。"""
    keep: list[tuple[tuple[int, int, int, int], float]] = []
    for box, score in sorted(cands, key=lambda c: -c[1]):
        if all(iou(box, k[0]) < thr for k in keep):
            keep.append((box, score))
    return [k[0] for k in keep]


def find_icon(bgr: np.ndarray) -> tuple[float, float] | None:
    """車いすの印（青い四角）の中心 (x, y)。無ければ None。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 0] >= 100) & (hsv[..., 0] <= 125) & (hsv[..., 1] >= 150) & (hsv[..., 2] >= 90)).astype(np.uint8)
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
    best = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area >= 120 and 0.5 <= w / max(h, 1) <= 2.0 and area / max(w * h, 1) >= 0.45 and (best is None or area > best[0]):
            best = (area, cent[i])
    return None if best is None else (float(best[1][0]), float(best[1][1]))


def assign(boxes: list[tuple[int, int, int, int]], icon: tuple[float, float] | None) -> tuple[list[str] | None, str]:
    """柱の並びで、枠ごとの向きを決める。決まらなければ (None, 理由)。"""
    order = sorted(range(len(boxes)), key=lambda i: (boxes[i][1] + boxes[i][3]) / 2)
    cy = lambda i: (boxes[i][1] + boxes[i][3]) / 2  # noqa: E731
    labels: list[str | None] = [None] * len(boxes)
    if icon is not None:
        below = [i for i in order if cy(i) > icon[1]]
        above = [i for i in order if cy(i) <= icon[1]][::-1]  # 印に近い順
        if len(below) > 2 or len(above) > 2:
            return None, "印の上下で枠が 3 つ以上"
        if len(below) == 1:
            return None, "印の下の枠が 1 つ（▲ か ▼ か、見落としがあって決まらない）"
        for k, i in enumerate(below):
            labels[i] = ("up", "down")[k]
        for k, i in enumerate(above):
            labels[i] = ("down", "up")[k]
    else:
        if len(boxes) != 2:
            return None, f"印が無く、枠が {len(boxes)} 個"
        a, b = order
        diam = ((boxes[a][2] - boxes[a][0]) + (boxes[b][2] - boxes[b][0])) / 2
        ratio = (cy(b) - cy(a)) / max(diam, 1)
        if not 1.3 <= ratio <= 3.5:
            return None, f"印が無く、上下の間隔が直径の {ratio:.1f} 倍"
        labels[a], labels[b] = "up", "down"
    return [str(x) for x in labels], ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", type=Path)
    ap.add_argument("--prefix", required=True, help="使うフレーム名の前の部分（例: IMG_1037）")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--margin-px", type=int, default=3, help="画面の端からこの距離以内の枠は、切れているとみなす")
    ap.add_argument("--min-aspect", type=float, default=0.4, help="幅÷高さがこれより小さい枠は、ボタンではないとして外す")
    ap.add_argument("--max-aspect", type=float, default=2.5, help="幅÷高さがこれより大きい枠（点字など）は、ボタンではないとして外す")
    args = ap.parse_args()

    cfg = load_config("elevator.yaml")
    cfg["detector"] = "yolo"
    det = make_candidate_detector(cfg)
    (args.out / "images").mkdir(parents=True, exist_ok=True)
    (args.out / "labels").mkdir(parents=True, exist_ok=True)
    rows, used, odd_boxes = [], 0, 0
    for p in sorted(args.frames.glob(f"{args.prefix}_*.jpg")):
        bgr = cv2.imread(str(p))
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        boxes = merge_boxes([(c.bbox, c.score) for c in det.detect(rgb)])
        n_odd = len(boxes)
        boxes = [b for b in boxes if args.min_aspect <= (b[2] - b[0]) / max(b[3] - b[1], 1) <= args.max_aspect]
        n_odd -= len(boxes)
        odd_boxes += n_odd
        why = ""
        if not boxes:
            why = "枠が無い"
        elif any(b[0] <= args.margin_px or b[1] <= args.margin_px or b[2] >= w - args.margin_px or b[3] >= h - args.margin_px
                 for b in boxes):
            why = "画面の端で切れた枠がある"
        labels = None
        if not why:
            labels, why = assign(boxes, find_icon(bgr))
        if labels is None:
            rows.append([p.name, len(boxes), "", why])
            continue
        cv2.imwrite(str(args.out / "images" / f"{p.stem}.png"), bgr)
        lines = []
        for (x1, y1, x2, y2), lab in zip(boxes, labels):
            lines.append(f"{CLASS_ID[lab]} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
        (args.out / "labels" / f"{p.stem}.txt").write_text("\n".join(lines) + "\n")
        rows.append([p.name, len(boxes), " ".join(labels), ""])
        used += 1
    with open(args.out / "summary.csv", "w", newline="") as f:
        csv.writer(f).writerows([["frame", "boxes", "labels", "skipped_because"]] + rows)
    skipped: dict[str, int] = {}
    for r in rows:
        if r[3]:
            skipped[r[3]] = skipped.get(r[3], 0) + 1
    print(f"[autolabel] {args.prefix}: {used} / {len(rows)} 枚にラベルを付けた → {args.out}")
    print(f"[autolabel]   ボタンの形でないので外した枠 … {odd_boxes} 個")
    for k, v in sorted(skipped.items(), key=lambda kv: -kv[1]):
        print(f"[autolabel]   使わない: {k} … {v} 枚")
    return 0


if __name__ == "__main__":
    sys.exit(main())
