"""エレベーターの呼びボタンの YOLO（クラス up / down）を学習する。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/make_button_dataset.py          # 先に学習データを作る
    $P Button_Press/J1-gen/sim/train_button_yolo.py            # 学習（GPU が無ければ CPU。30〜60 分ほど）
    $P Button_Press/J1-gen/sim/train_button_yolo.py --epochs 5 --fraction 0.2   # 試しに短く

学習した重みは _local/button_press/button_yolo/weights/best.pt に写す（git の対象外）。
configs/elevator.yaml の detector を yolo にすると、エージェントがこの重みを使う
（評価のときだけ変えるなら、環境変数 J1GEN_BUTTON_DETECTOR=yolo）。

元にするモデルは、リポジトリにある yolo26n.pt（Ultralytics、AGPL-3.0。Perception も使っているもの）。
公開のエレベーターボタンの重み（Kshaw17-web/End-to-end-elevator-button-detection）は、ライセンスの記載が
無いので使わない（common/elevator_buttons.py の説明を参照）。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
WORK_DIR = REPO_ROOT / "_local" / "button_press" / "button_yolo"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=str(WORK_DIR / "dataset" / "data.yaml"))
    ap.add_argument("--base", default=str(FEATURE_DIR / "yolo26n.pt"), help="元にする重み")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--fraction", type=float, default=1.0, help="学習データのうち使う割合（試しに短く回すとき）")
    ap.add_argument("--device", default="", help="空なら自動（GPU があれば GPU）。cpu / 0 など")
    ap.add_argument("--name", default="up_down")
    args = ap.parse_args()

    data = Path(args.data)
    if not data.is_file():
        print(f"[train] ❌ 学習データが無い: {data}（先に sim/make_button_dataset.py を実行する）")
        return 1
    from ultralytics import YOLO

    model = YOLO(args.base)
    # 左右の反転は使わない（▲▼ は上下の向きで区別する。左右の反転では変わらないが、上下の反転は禁止）
    model.train(data=str(data), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch, fraction=args.fraction,
                device=args.device or None, project=str(WORK_DIR / "runs"), name=args.name, exist_ok=True,
                flipud=0.0, fliplr=0.5, mosaic=1.0, workers=4, verbose=False, plots=True)
    best = WORK_DIR / "runs" / args.name / "weights" / "best.pt"
    if not best.is_file():
        print(f"[train] ❌ 重みができなかった: {best}")
        return 1
    out = WORK_DIR / "weights" / "best.pt"
    out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, out)
    metrics = YOLO(str(out)).val(data=str(data), imgsz=args.imgsz, device=args.device or None, verbose=False,
                                 project=str(WORK_DIR / "runs"), name=f"{args.name}_val", exist_ok=True)
    print(f"[train] ✅ 重み: {out}")
    print(f"[train] 検証用データで mAP50 {metrics.box.map50:.3f}、mAP50-95 {metrics.box.map:.3f}"
          f"（クラスごとの mAP50: {dict(zip(metrics.names.values(), [round(float(v), 3) for v in metrics.box.ap50]))}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
