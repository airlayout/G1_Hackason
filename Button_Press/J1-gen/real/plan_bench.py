"""押す動作の計画だけにかかる時間を測る（腕は動かさない・DDS も使わない）。PC2 で押す処理を動かせるかの確認用。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/plan_bench.py            # ノート PC・ラボ PC
    ~/button_press/venv/bin/python ~/button_press/repo/Button_Press/J1-gen/real/plan_bench.py   # PC2（PC2_ONBOARD_CHECK.md）

10/31 の配置 A〜F（REPORT_20261031.md）を sim/reach_map.py と同じ計画（IK・体と机との衝突・経由の姿勢・押し込みの直線）にかけ、
1 通りごとの時間（机の箱の作り直しを含む）と、通る / 断る（reach_map.py の段階の記号）を出す。
2026-10-31 の結果: ノート PC（Snapdragon）0.06〜0.14 秒、PC2（Orin NX）0.17〜0.32 秒。通る / 断るは両方で同じ。
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))

import reach_map as rm  # noqa: E402

# (名前, (机の縁 x, 縁からボトルの面まで, ボトルの左右 y, 天板 z, 縁の傾き))。10/31 の配置 A〜F と同じ
CELLS: list[tuple[str, tuple[float, float, float, float, float]]] = [
    ("A 縁32 右17", (0.32, 0.0, -0.17, 0.0, 0.0)),
    ("B 縁25 右17", (0.25, 0.0, -0.17, 0.0, 0.0)),
    ("C 縁32 右5", (0.32, 0.0, -0.05, 0.0, 0.0)),
    ("D 縁32 左0", (0.32, 0.0, 0.0, 0.0, 0.0)),
    ("E 縁20 右10", (0.20, 0.0, -0.10, 0.0, 0.0)),
    ("F 縁40 右17", (0.40, 0.0, -0.17, 0.0, 0.0)),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=3, help="1 通りあたりの繰り返し回数")
    args = ap.parse_args()

    t0 = time.perf_counter()
    rm.init_worker("right", "real", None)
    print(f"[bench] 準備（モデルの読み込み・衝突判定の用意） {time.perf_counter() - t0:.2f} 秒")
    for name, cell in CELLS:
        times: list[float] = []
        for _ in range(args.reps):
            rm._G["desk_key"] = None   # 机の箱の作り直しも毎回含める（実機でも 1 回ごとに作る）
            t = time.perf_counter()
            row = rm.run_cell(cell)
            times.append(time.perf_counter() - t)
        res = f"通る（{row['seed']}）" if row["plan_ok"] else f"断る（{row['stage']}）"
        print(f"[bench] {name}: {res}  中央値 {statistics.median(times):.2f} 秒"
              f"（最小 {min(times):.2f} / 最大 {max(times):.2f}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
