"""撮った写真で、エレベーターの呼びボタン（▲▼）を見つけられるかを確かめる。classic と YOLO を並べて比べる。

    P=G1_HuggingFace/venv/bin/python
    # 実機の収録（real/record.py --label button）。深度と腰の角度があれば、▲▼ の 1 組を選ぶところまで試す
    $P Button_Press/J1-gen/real/check_button_photos.py _local/button_press/recordings/<日時>_button
    # ただの写真（.png / .jpg）やフォルダ（スマートフォンで撮ったものなど）。画像の中の検出だけ
    $P Button_Press/J1-gen/real/check_button_photos.py ~/photos/elevator/
    # 正解の枠と比べる（YOLO の形式のラベル。images/ と同じ並びの labels/ に .txt があれば読む）
    $P Button_Press/J1-gen/real/check_button_photos.py _local/button_press/button_yolo/dataset/images/val

出力: _local/button_press/button_check/<日時>/
    <名前>.jpg      左が classic、右が YOLO。枠の色: 緑 = ▲（up）、赤 = ▼（down）、黄 = 記号を読めない
                    正解のラベルがあれば白い細い枠で重ねる。下の行に、▲▼ の 1 組を選べたか（深度があるとき）
    summary.csv     1 枚 1 行: 見つけた数、正解との一致（ラベルがあるとき）、1 組を選べたか
画面にも、検出器ごとの合計を出す。

YOLO の重みは configs/elevator.yaml の yolo.weights（sim/train_button_yolo.py で作る）。無ければ classic だけ。
シミュレーションの画像だけで学習した重みなので、本物のボタンでは見つけられないことがある。このツールは、
それを実機の日に確かめるためのもの（見つけられなければ、写真にラベルを付けて学習データに足す。README 参照）。
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config, resolve_repo_path  # noqa: E402
from common.elevator_buttons import (  # noqa: E402
    ARROWS, ButtonFinder, ButtonNotFound, Candidate2D, CandidateDetector, make_candidate_detector)

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}
COLORS = {"up": (0, 200, 0), "down": (0, 0, 230), None: (0, 220, 230)}  # BGR
IOU_MATCH = 0.5


@dataclass
class Photo:
    """確かめる 1 枚。深度・内部パラメータ・腰の角度は、実機の収録のときだけある。"""

    name: str
    bgr: np.ndarray
    depth_m: np.ndarray | None = None
    K: np.ndarray | None = None
    waist_q: np.ndarray | None = None
    truth: list[tuple[tuple[float, float, float, float], str]] = field(default_factory=list)


@dataclass
class Score:
    """正解の枠との比べ方（IoU 0.5 以上で同じボタンとみなす）。"""

    correct: int = 0  # 記号も合っている
    wrong_arrow: int = 0  # 枠は合っているが、▲▼ を取り違えた（読めないも含む）
    missed: int = 0  # 見つけられなかった
    extra: int = 0  # 正解に無い所で見つけた

    def add(self, o: "Score") -> None:
        self.correct += o.correct
        self.wrong_arrow += o.wrong_arrow
        self.missed += o.missed
        self.extra += o.extra


# ---- 読み込み -------------------------------------------------------------------


def read_yolo_labels(image: Path, w: int, h: int, names: tuple[str, ...] = ARROWS) -> list | None:
    """画像のパスの images/ を labels/ にした .txt（YOLO の形式）を読む。無ければ None。"""
    parts = list(image.parts)
    if "images" not in parts:
        return None
    i = len(parts) - 1 - parts[::-1].index("images")
    txt = Path(*parts[:i], "labels", *parts[i + 1:]).with_suffix(".txt")
    if not txt.is_file():
        return None
    out = []
    for line in txt.read_text().splitlines():
        v = line.split()
        if len(v) != 5:
            continue
        c, cx, cy, bw, bh = int(v[0]), *(float(x) for x in v[1:])
        out.append((((cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h),
                    names[c] if 0 <= c < len(names) else str(c)))
    return out


def load_photos(paths: list[Path], every: int) -> list[Photo]:
    """画像のファイル・フォルダ・実機の収録のフォルダ（meta.json がある）から読む。"""
    from common.recording import Recording

    photos: list[Photo] = []
    for p in paths:
        if (p / "meta.json").is_file() and (p / "frames.jsonl").is_file():
            rec = Recording(p)
            for k in range(0, len(rec), every):
                f = rec.load(k)
                ph = Photo(name=f"{p.name}_{f.index:06d}", bgr=f.color_bgr, waist_q=f.waist_q)
                if f.rgbd is not None:
                    it = f.rgbd.intrinsics
                    ph.depth_m = f.rgbd.depth_m()
                    ph.K = np.array([[it.fx, 0, it.cx], [0, it.fy, it.cy], [0, 0, 1]], float)
                photos.append(ph)
            continue
        files = sorted(q for q in p.rglob("*") if q.suffix.lower() in IMAGE_EXTS) if p.is_dir() else [p]
        for q in files[::every]:
            bgr = cv2.imread(str(q), cv2.IMREAD_COLOR)
            if bgr is None:
                print(f"[check] ⚠️ 読めない: {q}")
                continue
            truth = read_yolo_labels(q, bgr.shape[1], bgr.shape[0])
            photos.append(Photo(name=q.stem, bgr=bgr, truth=truth or []))
    return photos


# ---- 比べる ---------------------------------------------------------------------


def iou(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def score_against(cands: list[Candidate2D], truth: list) -> Score:
    """IoU の大きい順に 1 対 1 で組にして数える。"""
    pairs = sorted(((iou(c.bbox, t[0]), i, j) for i, c in enumerate(cands) for j, t in enumerate(truth)),
                   reverse=True)
    used_c: set[int] = set()
    used_t: set[int] = set()
    s = Score()
    for v, i, j in pairs:
        if v < IOU_MATCH or i in used_c or j in used_t:
            continue
        used_c.add(i)
        used_t.add(j)
        if cands[i].arrow == truth[j][1]:
            s.correct += 1
        else:
            s.wrong_arrow += 1
    s.missed = len(truth) - len(used_t)
    s.extra = len(cands) - len(used_c)
    return s


def try_pair(finder: ButtonFinder | None, ph: Photo) -> str:
    """深度があれば、▲▼ の 1 組を選ぶところまで試す。結果を 1 行で返す。"""
    if finder is None or ph.depth_m is None or ph.K is None:
        return "-"
    if ph.waist_q is None:
        return "腰の角度が無い（lowstate なしの収録）"
    try:
        call = finder.find(ph.bgr[:, :, ::-1], ph.depth_m, ph.K, ph.waist_q)
    except ButtonNotFound as e:
        return f"NG: {e}"
    return f"OK: {call.summary()}"


# ---- 描く -----------------------------------------------------------------------


def draw(ph: Photo, cands: list[Candidate2D] | None, title: str, pair: str) -> np.ndarray:
    img = ph.bgr.copy()
    for box, _ in ph.truth:
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        cv2.rectangle(img, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), (255, 255, 255), 1)
    for c in cands or []:
        x1, y1, x2, y2 = c.bbox
        col = COLORS.get(c.arrow, COLORS[None])
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        label = f"{c.arrow or '?'} {c.score:.2f}"
        cv2.putText(img, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(img, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    head = f"{title}: " + ("(no weights)" if cands is None else f"{len(cands)} found")
    cv2.rectangle(img, (0, 0), (img.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(img, head, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    if pair != "-":
        ok = pair.startswith("OK")
        cv2.rectangle(img, (0, img.shape[0] - 22), (img.shape[1], img.shape[0]), (0, 0, 0), -1)
        cv2.putText(img, "pair: OK" if ok else "pair: NG (see summary.csv)", (6, img.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0) if ok else (0, 0, 255), 1)
    return img


# ---- 本体 -----------------------------------------------------------------------


def make_detectors(cfg: dict, use_yolo: bool) -> dict[str, CandidateDetector | None]:
    dets: dict[str, CandidateDetector | None] = {"classic": make_candidate_detector({**cfg, "detector": "classic"})}
    if not use_yolo:
        return dets
    weights = resolve_repo_path(cfg["yolo"]["weights"])
    if not weights.is_file():
        print(f"[check] ⚠️ YOLO の重みが無いので classic だけ: {weights}")
        dets["yolo"] = None
        return dets
    dets["yolo"] = make_candidate_detector({**cfg, "detector": "yolo"})
    return dets


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=Path, help="画像・画像のフォルダ・実機の収録のフォルダ")
    ap.add_argument("--every", type=int, default=1, help="N 枚に 1 枚だけ見る（収録が長いとき）")
    ap.add_argument("--no-yolo", action="store_true", help="classic だけ（ultralytics が無い PC など）")
    ap.add_argument("--confidence", type=float, help="YOLO の確信度の下限（既定は configs/elevator.yaml）")
    ap.add_argument("--out", type=Path, help="出力先（既定は _local/button_press/button_check/<日時>/）")
    args = ap.parse_args()

    cfg = load_config("elevator.yaml")
    if args.confidence is not None:
        cfg["yolo"]["confidence"] = args.confidence
    photos = load_photos([p.expanduser() for p in args.paths], max(1, args.every))
    if not photos:
        print("[check] ❌ 画像が 1 枚も無い")
        return 1
    dets = make_detectors(cfg, not args.no_yolo)
    robot_cfg = load_config("robot.yaml")
    finders = {k: ButtonFinder.from_config(robot_cfg, cfg, detector=d) for k, d in dets.items() if d is not None}
    out = args.out or REPO_ROOT / "_local" / "button_press" / "button_check" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    n_truth = sum(1 for ph in photos if ph.truth)
    print(f"[check] {len(photos)} 枚（正解のラベルあり {n_truth} 枚）、検出器 {', '.join(dets)} → {out}")

    totals = {k: Score() for k in dets}
    found = {k: 0 for k in dets}
    pair_ok = {k: [0, 0] for k in dets}  # [選べた, 試した]
    rows = []
    for ph in photos:
        panels = []
        row: dict[str, object] = {"name": ph.name, "truth": len(ph.truth) if ph.truth else ""}
        for k, d in dets.items():
            cands = None if d is None else d.detect(ph.bgr[:, :, ::-1])
            pair = try_pair(finders.get(k), ph)
            panels.append(draw(ph, cands, k, pair))
            if cands is None:
                continue
            found[k] += len(cands)
            row[f"{k}_found"] = len(cands)
            row[f"{k}_arrows"] = " ".join(c.arrow or "?" for c in sorted(cands, key=lambda c: c.bbox[1]))
            if ph.truth:
                s = score_against(cands, ph.truth)
                totals[k].add(s)
                row.update({f"{k}_correct": s.correct, f"{k}_wrong_arrow": s.wrong_arrow,
                            f"{k}_missed": s.missed, f"{k}_extra": s.extra})
            if pair != "-":
                row[f"{k}_pair"] = pair
                pair_ok[k][1] += 1
                pair_ok[k][0] += pair.startswith("OK")
        cv2.imwrite(str(out / f"{ph.name}.jpg"), np.hstack(panels), [cv2.IMWRITE_JPEG_QUALITY, 90])
        rows.append(row)

    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    for k, d in dets.items():
        if d is None:
            continue
        msg = f"[check] {k}: 見つけた枠 {found[k]} 個"
        if n_truth:
            s = totals[k]
            n = s.correct + s.wrong_arrow + s.missed
            msg += (f"、正解 {n} 個のうち 記号まで正しい {s.correct}（{s.correct / max(n, 1):.0%}）、"
                    f"▲▼ の取り違え・読めない {s.wrong_arrow}、見落とし {s.missed}、余分 {s.extra}")
        if pair_ok[k][1]:
            msg += f"、▲▼ の 1 組を選べた {pair_ok[k][0]} / {pair_ok[k][1]} 枚"
        print(msg)
    print(f"[check] 画像と summary.csv: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
