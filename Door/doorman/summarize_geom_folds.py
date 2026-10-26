"""実験 A（10-21）のまとめ: K 個の fold の検証の推定を合わせて、96 種類の扉すべてを「学習に使わなかった扉」として集計する。

使い方: python summarize_geom_folds.py v2model/geom_1021  → 表を出し、<場所>/summary.json に書く
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from door_vision_geom_train import TARGETS, summarize


def main() -> None:
    root = Path(sys.argv[1])
    parts = [np.load(f) for f in sorted(root.glob("fold*/pred_val.npz"))]
    if not parts:
        raise SystemExit(f"[geom] fold*/pred_val.npz が無い: {root}")
    data = {k: np.concatenate([p[k] for p in parts]) for k in ("pred", "y", "stage", "door", "group")}
    idx = np.arange(len(data["y"]))
    r = summarize(data["pred"], data, idx)
    print(f"== 実験 A のまとめ: fold {len(parts)} 個・画像 {r['n']} 枚・扉 {len(np.unique(data['door']))} 種類（どれも学習に使わなかった扉）")
    pi = r["per_image"]
    print("1 枚ごと（MAE）: " + "・".join(f"{n} {pi[n]['mae']:.2f}" for n in TARGETS) + "（長さ cm・角度 °）")
    print("距離ごとの寸法の誤差（cm、1 枚ごとの MAE）")
    for k, v in r["by_distance"].items():
        print(f"  {k:>9}: {v['n']:6d} 枚  幅 {v['width']:.2f}・レバーの高さ {v['lever_h']:.2f}・バックセット {v['backset']:.2f}")
    for key, label in (("per_episode", "回ごと（近づく間の中央値）"), ("per_door", "扉ごと")):
        if key in r:
            v = r[key]
            print(f"{label}（{v['n']}）: " + "・".join(
                f"{n} MAE {v[n]['mae']:.2f}・中央 {v[n]['p50']:.2f}・9 割 {v[n]['p90']:.2f}・最大 {v[n]['max']:.2f}" for n in TARGETS[4:]) + " cm")
    (root / "summary.json").write_text(json.dumps(r, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
