"""シミュレーションの学習データと、ラベルを付けた実機の写真（real/label_button_photos.py）を混ぜた学習データを作る。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/merge_button_datasets.py _local/button_press/button_real/<収録の名前> [<別の収録> ...]
    $P Button_Press/J1-gen/sim/train_button_yolo.py --data _local/button_press/button_yolo/dataset_mixed/data.yaml \\
        --out _local/button_press/button_yolo/weights/best_mixed.pt

- シミュレーションの学習データ（sim/make_button_dataset.py）は、学習用・検証用をそのまま使う
- 実機の写真は、--real-val-every 枚に 1 枚を検証用にし、残りを学習用にする。学習用は --real-repeat 回ずつ入れる
  （シミュレーションの 270 枚に比べて少ないため。同じ写真を何回も入れても、学習のたびに変形（拡大・色）が違う）
- 実機の検証用の写真の名前は real_ で始まる。学習のあとの検証の結果が、シミュレーションと実機に分かれて
  見えるわけではないので、実機だけの成績は real/check_button_photos.py で、出力の images/val_real を見る

ファイルは写す（シンボリックリンクにしない。GPU のある PC に tar で持っていくため）。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
WORK_DIR = REPO_ROOT / "_local" / "button_press" / "button_yolo"
IMAGE_EXTS = {".png", ".jpg", ".jpeg"}


def labeled_pairs(folder: Path) -> list[tuple[Path, Path]]:
    """images/ と labels/ で名前がそろう組（ラベルの無い写真は使わない）。"""
    out = []
    for img in sorted((folder / "images").iterdir()):
        txt = folder / "labels" / f"{img.stem}.txt"
        if img.suffix.lower() in IMAGE_EXTS and txt.is_file():
            out.append((img, txt))
    return out


def copy_pair(img: Path, txt: Path, out: Path, split: str, name: str) -> None:
    shutil.copy2(img, out / "images" / split / f"{name}{img.suffix.lower()}")
    shutil.copy2(txt, out / "labels" / split / f"{name}.txt")


def merge(sim: Path, reals: list[Path], out: Path, real_val_every: int, real_repeat: int) -> dict[str, int]:
    if out.exists():
        shutil.rmtree(out)
    for split in ("train", "val", "val_real"):
        (out / "images" / split).mkdir(parents=True)
        (out / "labels" / split).mkdir(parents=True)
    counts = {"sim_train": 0, "sim_val": 0, "real_train": 0, "real_val": 0}
    for split in ("train", "val"):
        for img, txt in labeled_pairs_split(sim, split):
            copy_pair(img, txt, out, split, img.stem)
            counts[f"sim_{split}"] += 1
    for folder in reals:
        for i, (img, txt) in enumerate(labeled_pairs(folder)):
            name = f"real_{folder.name}_{img.stem}"
            if real_val_every > 0 and i % real_val_every == real_val_every - 1:
                copy_pair(img, txt, out, "val", name)
                copy_pair(img, txt, out, "val_real", name)
                counts["real_val"] += 1
                continue
            for r in range(real_repeat):
                copy_pair(img, txt, out, "train", f"{name}_r{r}")
            counts["real_train"] += 1
    (out / "data.yaml").write_text(
        f"# sim/merge_button_datasets.py が作った（シミュレーション {sim}、実機 {', '.join(p.name for p in reals)}）\n"
        f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: up\n  1: down\n")
    return counts


def labeled_pairs_split(dataset: Path, split: str) -> list[tuple[Path, Path]]:
    imgs = dataset / "images" / split
    if not imgs.is_dir():
        return []
    return [(p, dataset / "labels" / split / f"{p.stem}.txt") for p in sorted(imgs.iterdir())
            if p.suffix.lower() in IMAGE_EXTS and (dataset / "labels" / split / f"{p.stem}.txt").is_file()]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real", nargs="+", type=Path, help="ラベルを付けた実機の写真のフォルダ（images/ と labels/ がある）")
    ap.add_argument("--sim", type=Path, default=WORK_DIR / "dataset", help="シミュレーションの学習データ")
    ap.add_argument("--out", type=Path, default=WORK_DIR / "dataset_mixed")
    ap.add_argument("--real-val-every", type=int, default=5, help="実機の写真の N 枚に 1 枚を検証用にする（0 なら全部学習用）")
    ap.add_argument("--real-repeat", type=int, default=3, help="実機の学習用の写真を何回ずつ入れるか")
    args = ap.parse_args()

    if not (args.sim / "images" / "train").is_dir():
        print(f"[merge] ❌ シミュレーションの学習データが無い: {args.sim}（先に sim/make_button_dataset.py）")
        return 1
    for p in args.real:
        if not (p / "images").is_dir() or not (p / "labels").is_dir():
            print(f"[merge] ❌ images/ と labels/ が無い: {p}（real/label_button_photos.py で作る）")
            return 1
    c = merge(args.sim, args.real, args.out, args.real_val_every, max(1, args.real_repeat))
    if c["real_train"] + c["real_val"] == 0:
        print("[merge] ❌ ラベルの付いた実機の写真が 0 枚")
        return 1
    print(f"[merge] ✅ シミュレーション 学習 {c['sim_train']}・検証 {c['sim_val']} 枚、実機 学習 {c['real_train']} 枚"
          f"（× {max(1, args.real_repeat)}）・検証 {c['real_val']} 枚 → {args.out / 'data.yaml'}")
    print(f"[merge] 実機の検証用だけの成績: real/check_button_photos.py {args.out / 'images' / 'val_real'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
