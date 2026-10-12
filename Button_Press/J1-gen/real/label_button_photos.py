"""撮った写真に、呼びボタンの正解の枠（up = ▲ / down = ▼）を付ける。YOLO の学習データの形式で保存する。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/real/label_button_photos.py _local/button_press/recordings/<日時>_button_hall --every 3

最初は、今の検出器（YOLO の重みがあれば YOLO、無ければ classic）が見つけた枠を入れておくので、直すだけでよい。
途中でやめても、保存した写真の枠は残る（もう一度起動すると、続きから。保存した枠を読み直す）。

操作（画面の上に表示する）:
    左ドラッグ        枠を足す（今のクラス。1 = up ▲、2 = down ▼ で切り替え）
    右クリック        その位置の枠を消す
    c                 マウスの下の枠のクラスを up ⇔ down で入れ替える
    Space / Enter     保存して次へ（枠が 0 個でも保存する。ボタンが写っていない写真も学習に使う）
    x                 この写真は使わない（保存しない。消してあれば消す）
    b                 前の写真へ
    q / Esc           やめる（今の写真は保存しない）

枠の付け方: 一般用も車いす用も、▲▼ のボタンはすべて付ける（学習データと同じ）。ボタンの丸い面を囲む。
半分以上隠れた・画面の端で切れたボタンには付けない。

出力: _local/button_press/button_real/<収録の名前>/images/*.png と labels/*.txt（YOLO の形式）。
sim/merge_button_datasets.py で、シミュレーションの学習データと混ぜる（README の「実機の写真を学習データに足す」）。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config, resolve_repo_path  # noqa: E402
from common.elevator_buttons import ARROWS, make_candidate_detector  # noqa: E402

OUT_ROOT = REPO_ROOT / "_local" / "button_press" / "button_real"
COLORS = {"up": (0, 200, 0), "down": (0, 0, 230)}  # BGR
MIN_BOX_PX = 4


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float
    cls: str

    def contains(self, x: float, y: float) -> bool:
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2

    def area(self) -> float:
        return (self.x2 - self.x1) * (self.y2 - self.y1)


@dataclass
class Sheet:
    """1 枚の写真と、その枠。"""

    name: str
    bgr: np.ndarray
    boxes: list[Box] = field(default_factory=list)

    def add(self, x1: float, y1: float, x2: float, y2: float, cls: str) -> bool:
        h, w = self.bgr.shape[:2]
        x1, x2 = sorted((float(np.clip(x1, 0, w)), float(np.clip(x2, 0, w))))
        y1, y2 = sorted((float(np.clip(y1, 0, h)), float(np.clip(y2, 0, h))))
        if x2 - x1 < MIN_BOX_PX or y2 - y1 < MIN_BOX_PX:
            return False
        self.boxes.append(Box(x1, y1, x2, y2, cls))
        return True

    def _at(self, x: float, y: float) -> Box | None:
        """その位置の枠（重なっていれば小さいほう）。"""
        hits = [b for b in self.boxes if b.contains(x, y)]
        return min(hits, key=Box.area) if hits else None

    def delete_at(self, x: float, y: float) -> bool:
        b = self._at(x, y)
        if b is not None:
            self.boxes.remove(b)
        return b is not None

    def toggle_at(self, x: float, y: float) -> bool:
        b = self._at(x, y)
        if b is not None:
            b.cls = "down" if b.cls == "up" else "up"
        return b is not None

    def yolo_lines(self) -> list[str]:
        h, w = self.bgr.shape[:2]
        return [f"{ARROWS.index(b.cls)} {(b.x1 + b.x2) / 2 / w:.6f} {(b.y1 + b.y2) / 2 / h:.6f} "
                f"{(b.x2 - b.x1) / w:.6f} {(b.y2 - b.y1) / h:.6f}" for b in self.boxes]


def save(sheet: Sheet, out: Path) -> None:
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out / "images" / f"{sheet.name}.png"), sheet.bgr):
        raise RuntimeError(f"書き込めない: {out / 'images' / sheet.name}.png")
    (out / "labels" / f"{sheet.name}.txt").write_text("".join(line + "\n" for line in sheet.yolo_lines()))


def remove(sheet: Sheet, out: Path) -> None:
    for p in (out / "images" / f"{sheet.name}.png", out / "labels" / f"{sheet.name}.txt"):
        p.unlink(missing_ok=True)


def load_saved(sheet: Sheet, out: Path) -> bool:
    """保存した枠があれば読む（続きから付けるとき）。"""
    txt = out / "labels" / f"{sheet.name}.txt"
    if not txt.is_file():
        return False
    h, w = sheet.bgr.shape[:2]
    sheet.boxes.clear()
    for line in txt.read_text().splitlines():
        v = line.split()
        if len(v) == 5:
            c, cx, cy, bw, bh = int(v[0]), *(float(x) for x in v[1:])
            sheet.boxes.append(Box((cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h, ARROWS[c]))
    return True


def prelabel(sheet: Sheet, detector) -> None:
    """今の検出器の結果を入れておく（▲▼ を読めなかった枠は入れない）。"""
    for c in detector.detect(sheet.bgr[:, :, ::-1]):
        if c.arrow in ARROWS:
            sheet.add(*c.bbox, c.arrow)


def make_detector(cfg: dict):
    if resolve_repo_path(cfg["yolo"]["weights"]).is_file():
        try:
            return "yolo", make_candidate_detector({**cfg, "detector": "yolo"})
        except ImportError as e:
            print(f"[label] ⚠️ YOLO を使えないので classic で下書きする: {e}")
    return "classic", make_candidate_detector({**cfg, "detector": "classic"})


# ---- 画面 -----------------------------------------------------------------------


def render(sheet: Sheet, k: int, n: int, cls: str, drag: tuple | None, scale: float) -> np.ndarray:
    img = cv2.resize(sheet.bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for b in sheet.boxes:
        p1, p2 = (int(b.x1 * scale), int(b.y1 * scale)), (int(b.x2 * scale), int(b.y2 * scale))
        cv2.rectangle(img, p1, p2, COLORS[b.cls], 2)
        cv2.putText(img, b.cls, (p1[0], max(12, p1[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[b.cls], 1)
    if drag is not None:
        cv2.rectangle(img, (int(drag[0]), int(drag[1])), (int(drag[2]), int(drag[3])), COLORS[cls], 1)
    head = (f"{k + 1}/{n} {sheet.name}  class: {cls} (1=up 2=down)  drag=add  right=delete  c=swap  "
            "space=save+next  x=skip  b=back  q=quit")
    cv2.rectangle(img, (0, 0), (img.shape[1], 20), (0, 0, 0), -1)
    cv2.putText(img, head, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    return img


def main() -> int:
    from check_button_photos import load_photos  # 同じフォルダ。画像・フォルダ・実機の収録を読む

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path, help="実機の収録のフォルダ、または画像のフォルダ")
    ap.add_argument("--every", type=int, default=1, help="N 枚に 1 枚だけ付ける（1 秒ごとの収録なら 2〜3 で十分）")
    ap.add_argument("--out", type=Path, help="既定は _local/button_press/button_real/<フォルダの名前>/")
    ap.add_argument("--scale", type=float, default=1.5, help="画面の拡大率")
    ap.add_argument("--no-prelabel", action="store_true", help="検出器の結果を入れておかない")
    args = ap.parse_args()

    path = args.path.expanduser()
    out = args.out or OUT_ROOT / path.name
    photos = load_photos([path], max(1, args.every))
    if not photos:
        print("[label] ❌ 画像が 1 枚も無い")
        return 1
    sheets = [Sheet(ph.name, ph.bgr) for ph in photos]
    done = sum(load_saved(s, out) for s in sheets)
    detector = None
    if not args.no_prelabel and done < len(sheets):
        kind, detector = make_detector(load_config("elevator.yaml"))
        print(f"[label] 下書きの検出器: {kind}")
    print(f"[label] {len(sheets)} 枚（保存済み {done} 枚）→ {out}")

    win = "label_button_photos"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    st: dict = {"cls": "up", "drag": None, "mouse": (0.0, 0.0), "dirty": True}
    sc = args.scale

    def on_mouse(event: int, x: int, y: int, flags: int, param: object) -> None:
        st["mouse"] = (x / sc, y / sc)
        if event == cv2.EVENT_LBUTTONDOWN:
            st["drag"] = [x, y, x, y]
        elif event == cv2.EVENT_MOUSEMOVE and st["drag"] is not None:
            st["drag"][2:] = [x, y]
        elif event == cv2.EVENT_LBUTTONUP and st["drag"] is not None:
            x1, y1, _, _ = st["drag"]
            st["drag"] = None
            sheets[st["k"]].add(x1 / sc, y1 / sc, x / sc, y / sc, st["cls"])
        elif event == cv2.EVENT_RBUTTONDOWN:
            sheets[st["k"]].delete_at(x / sc, y / sc)
        else:
            return
        st["dirty"] = True

    cv2.setMouseCallback(win, on_mouse)
    # 続きから: 保存していない最初の写真
    k = next((i for i, s in enumerate(sheets) if not (out / "labels" / f"{s.name}.txt").is_file()), 0)
    prelabeled: set[int] = set()
    saved = skipped = 0
    while 0 <= k < len(sheets):
        st["k"] = k
        sheet = sheets[k]
        if detector is not None and k not in prelabeled and not (out / "labels" / f"{sheet.name}.txt").is_file():
            prelabel(sheet, detector)
            prelabeled.add(k)
        cv2.imshow(win, render(sheet, k, len(sheets), st["cls"], st["drag"], sc))
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("1"), ord("2")):
            st["cls"] = "up" if key == ord("1") else "down"
        elif key == ord("c"):
            sheet.toggle_at(*st["mouse"])
        elif key in (ord(" "), 13, 10):
            save(sheet, out)
            saved += 1
            k += 1
        elif key == ord("x"):
            remove(sheet, out)
            skipped += 1
            k += 1
        elif key == ord("b"):
            k = max(0, k - 1)
        elif key in (ord("q"), 27):
            break
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()
    n_saved = len(list((out / "labels").glob("*.txt"))) if (out / "labels").is_dir() else 0
    print(f"[label] 今回 保存 {saved} 枚、使わない {skipped} 枚。保存済みの合計 {n_saved} / {len(sheets)} 枚: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
