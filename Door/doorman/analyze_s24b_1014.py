"""段 2-4b（レバーの位置・形・扉の重さを広げた扉）の評価を、レバーの高さ別・バックセット別・扉の重さ別に分ける。

どの寸法で落ちるかを見る。台ごとの高さとバックセットは記録の static.npz にある（door_handle_height・door_handle_width）。
レバーの長さと断面は記録に無い（env が持っていない）ので、ここでは分けられない。
失敗の型の分け方は analyze_s24L_1014.py と同じ。

使い方（OMEN の ~/work_inoue/doorman で）:
    ~/miniforge3/envs/doorman6/bin/python analyze_s24b_1014.py records/start_<日時> [records/start_<日時> ...]
"""

import sys

import numpy as np

from analyze_s24L_1014 import CATS, load, wilson

# 2-4a で学習した範囲（高さ 0.85〜0.95 m・バックセット 80〜150 mm）と実物（約 1.0 m・64 mm）が分かれる区切り
HEIGHT_EDGES = [0.85, 0.95, 1.02, 1.10]  # [m]
BACKSET_EDGES = [0.03, 0.05, 0.08, 0.15]  # [m]（最後の区切りは 2-4a の扉の上限 150 mm まで）
WEIGHT_EDGES = [15.0, 50.0, 80.0, 120.0]  # [kg]（80〜120 は 2-4a・b1・b2 の扉の範囲、段 2-4b-3 で 15 kg まで広げる）
# 実物の近く（高さは目測なので幅を持たせる）
REAL_HEIGHT = (0.95, 1.05)
REAL_BACKSET = (0.05, 0.08)


def by_bins(name: str, values: np.ndarray, ok: np.ndarray, edges: list[float], unit: float, label: str) -> None:
    print(f"  {name}別の成功")
    for lo, hi in zip(edges[:-1], edges[1:], strict=True):
        m = (values >= lo) & (values < hi) if hi < edges[-1] else (values >= lo) & (values <= hi)
        n = int(m.sum())
        print(f"    {lo * unit:5.0f}〜{hi * unit:4.0f} {label}: {wilson(int(ok[m].sum()), n) if n else '台なし'}")


def report(rec: str) -> None:
    d = load(rec)
    s = np.load(f"{rec}/static.npz", allow_pickle=True)
    height = s["door_handle_height"][d["env"]]
    backset = s["door_handle_width"][d["env"]]
    ok = d["ok"]
    print(f"== {rec}  成功 {wilson(int(ok.sum()), len(ok))}")
    counts = [int((d["cat"] == k).sum()) for k in range(len(CATS))]
    print("  型: " + "・".join(f"{c} {n}" for c, n in zip(CATS, counts, strict=True)))
    by_bins("レバーの高さ", height, ok, HEIGHT_EDGES, 100, "cm")
    by_bins("バックセット", backset, ok, BACKSET_EDGES, 1000, "mm")
    by_bins("扉の重さ", s["door_weight"][d["env"]], ok, WEIGHT_EDGES, 1, "kg")
    real = (height >= REAL_HEIGHT[0]) & (height <= REAL_HEIGHT[1]) & (backset >= REAL_BACKSET[0]) & (backset <= REAL_BACKSET[1])
    near = f"高さ {REAL_HEIGHT[0]}〜{REAL_HEIGHT[1]} m・バックセット {REAL_BACKSET[0] * 1000:.0f}〜{REAL_BACKSET[1] * 1000:.0f} mm"
    print(f"  実物の近く（{near}）: {wilson(int(ok[real].sum()), int(real.sum())) if real.any() else '台なし'}")
    lever_fail = d["cat"] == CATS.index("レバーまで行けない")
    if lever_fail.any():
        h, b = np.median(height[lever_fail]), np.median(backset[lever_fail]) * 1000
        print(f"  「レバーまで行けない」台の高さの中央値 {h:.2f} m・バックセットの中央値 {b:.0f} mm")


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for rec in sys.argv[1:]:
        report(rec.rstrip("/"))


if __name__ == "__main__":
    main()
